// vga.h - GTK 版
#ifndef VGA_H
#define VGA_H

#include <stdint.h>
#include <stdbool.h>
#include <gtk/gtk.h>

typedef enum {
    MODE_TEXT_80x25 = 0x03,
    MODE_TEXT_40x25 = 0x01,
    MODE_GFX_640x480 = 0x12,
    MODE_GFX_320x200 = 0x13,
    MODE_CGA_320x200 = 0x04,   // CGA 模式 4/5：320x200 四色，2bpp，双 bank 交错
    MODE_CGA_640x200 = 0x06    // CGA 模式 6   ：640x200 双色，1bpp，双 bank 交错
} VideoMode;

typedef struct {
    uint8_t r, g, b;
} RGBColor;

typedef struct {
    GtkWidget* window;
    GtkWidget* drawing_area;
    cairo_surface_t* surface;
    uint32_t* pixels;
    VideoMode mode;
    int width;
    int height;
    int cursor_x;
    int cursor_y;
    bool cursor_visible;
} VGA;

extern VGA vga;
extern RGBColor vga_palette[16];
extern uint8_t vga_palette_256[256][3];
extern uint8_t vga_dac_mask;
extern uint8_t vga_dac_read_index;
extern uint8_t vga_dac_write_index;

void vga_init(int argc, char** argv);
// -console：无窗口模式，vga_init 只初始化状态、不建 GTK 窗口/画布
void vga_set_headless(bool on);
void vga_cleanup(void);
void vga_set_mode(VideoMode mode);
bool vga_render(void);
void vga_clear(uint8_t attr);
// 机器复位（GTK「重启」按钮）：回到 80x25 文本模式并清屏
void vga_reset(void);

// ---- 显卡选件 ROM（C000 段）----
//   DIP 报 EGA/VGA 时 BIOS 会把 INT 10h 设成空 IRET、等显卡选件 ROM 接管；
//   这块 ROM 在 POST 的选件扫描里被 lcall，把 INT 10h 指回 BIOS 自带的处理程序。
void vga_video_rom_install(void);   // 写进 C0000（开机 / 复位后各调一次）

void vga_set_dac_entry(uint8_t index, uint8_t r, uint8_t g, uint8_t b);
void vga_set_default_palette(void);

// ---- CGA 端口 0x3D8（模式控制）/ 0x3D9（颜色选择）----
//   真实 CGA 上这两个寄存器才是"当前视频模式"的权威来源，
//   演示程序常常绕过 INT 10h 直接写它们，所以必须接进 vga 状态机。
void    vga_write_cga_mode(uint8_t val);    // 写 0x3D8
uint8_t vga_read_cga_mode(void);            // 读 0x3D8
void    vga_write_cga_color(uint8_t val);   // 写 0x3D9
uint8_t vga_read_cga_color(void);           // 读 0x3D9

// ---- 复合视频伪色（NTSC artifact colors）----
//   CGA 的复合输出把 R/G/B 三路分别门控 0°/120°/240° 的 3.58MHz 色载波，
//   相邻像素的明暗图案会在解调时产生"artifact 颜色"（8088 MPH 的 1K 色模式就靠它）。
//   打开后 CGA 各模式按复合信号解调出颜色，而不是直接用 RGBI 直接色。
void vga_set_composite(bool on);

gboolean vga_on_draw(GtkWidget* widget, cairo_t* cr, gpointer data);
gboolean vga_on_key_press(GtkWidget* widget, GdkEventKey* event, gpointer data);
gboolean vga_on_key_release(GtkWidget* widget, GdkEventKey* event, gpointer data);

// ---- 磁盘读写指示灯（窗口底部拉出的一条窄条）----
//   which: 0=硬盘(IDE) 1=软盘(FDC) 2=CD-ROM（以后加） 3=键盘
void vga_led_activity(int which);
bool vga_led_dirty(void);
#define VGA_PLANE_SIZE  0x10000
extern uint8_t vga_planes[4][VGA_PLANE_SIZE];
extern uint8_t vga_seq_regs[8];
extern uint8_t vga_gc_regs[16];
extern uint8_t vga_attr_regs[0x20];
extern uint8_t vga_attr_palette[16];
uint8_t vga_mem_read(uint32_t off);
void    vga_mem_write(uint32_t off, uint8_t val);
bool    vga_mem_owns(uint32_t addr);   // 判断该地址是否落在 VGA 窗口
#endif