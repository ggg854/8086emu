// vga.c - CGA 模拟（GTK 版，闪烁光标）
#include "vga.h"
#include "cpu.h"
#include "io.h"
#include "font8x16.h"
#include "cga_font_8x8.h"
#include "vga_rom_vga.h"
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

VGA vga;
// VGA 位平面显存
uint8_t vga_planes[4][VGA_PLANE_SIZE] = {0};

// 0xA0000-0xAFFFF 显存窗口是否由上面的位平面模型接管（cpu.h 的内存读写热路径据此分流，
// 见 cpu_mem_read / cpu_mem_write）。
//   只有真正的"位平面图形模式"才置 1：此时一屏像素分散在 4 个平面里，扁平 memory[]
//   根本放不下 Plane 1-3，客机的写必须过 Map Mask / 写模式 / 位掩码，读要按
//   Read Map Select 返回。
//   而 chain4 的 13h（256 色）在显存里是 4 个平面按字节交错（第 n 字节属于第 n%4 平面），
//   与"同一地址四个平面"的模型布局不同，仍走扁平 memory[]（渲染路径也已如此）。
bool vga_mem_window_active = false;

// 位平面显存的行跨距（字节）：由 CRTC Offset 寄存器(0x13)推导，见 vga_update_mode。
//   正常模式它就是"显示宽度/8"；驱动做虚拟桌面时会被设得更大，必须按它走。
static int vga_plane_stride = 80;

// ---- 字库选择：VGA 8x16（默认）或 CGA 8x8（authentic IBM CGA 字符 ROM）----
//   文本渲染按 16 行/字符走；选 CGA 时把 8x8 字形放到每个单元格的上 8 行（下 8 行留空）。
//   与 src/xt 驱动的 CGA/VGA 二选一概念一致，由 config 的 font 字段驱动。
static int vga_font_cga = 0;
static int vga_char_h   = 16;   // 字符单元格高度：VGA=16、CGA=8（各自原生，不放大）

static void vga_apply_font(void) {
  init_font();                         // 默认：VGA 8x16
  if (vga_font_cga) {
    memset(font8x16, 0, sizeof(font8x16));
    for (int c = 0; c < 256; c++)
      for (int i = 0; i < 8; i++)
        font8x16[c][i] = cga_font_8x8[c * 8 + i];   // CGA 8x8 原生（占 rows 0-7）
  }
}

void vga_set_font_cga(int on) {
  vga_font_cga = on ? 1 : 0;
  vga_char_h   = on ? 8 : 16;
  vga_apply_font();
  vga_set_mode(vga.mode);   // 文本画布高度 = 25*char_h，按字库重算（CGA=200、VGA=400）
}

// VGA 端口寄存器
uint8_t vga_seq_regs[8] = {0};
uint8_t vga_gc_regs[16] = {0};
uint8_t vga_crtc_regs[0x20] = {0};
uint8_t vga_attr_regs[0x20] = {0};
uint8_t vga_attr_palette[16] = {0, 1, 2, 3, 4, 5, 6, 7,
                                8, 9, 10, 11, 12, 13, 14, 15};
extern bool debug_mode;   // 主程序 -dbg 打开
extern void ui_mount_floppy(int drive);   // 主程序：运行时换盘（弹文件框 + 挂载）
extern void ui_reset_machine(void);       // 主程序：复位机器（重新 POST）

static void on_reset_machine(GtkWidget* w, gpointer data) {
  (void)w; (void)data;
  ui_reset_machine();
  if (vga.drawing_area) gtk_widget_grab_focus(vga.drawing_area);
}

// 顶部工具栏按钮回调
static void on_mount_floppy(GtkWidget* w, gpointer data) {
  (void)w;
  ui_mount_floppy(GPOINTER_TO_INT(data));
  if (vga.drawing_area) gtk_widget_grab_focus(vga.drawing_area);
}

// 「频率」菜单：切换客机主频（计时换算都走 cpu_clk_hz，立即生效）
static void on_set_freq(GtkWidget* w, gpointer data) {
  (void)w;
  cpu_clk_hz = (uint32_t)GPOINTER_TO_INT(data);
  if (vga.drawing_area) gtk_widget_grab_focus(vga.drawing_area);
}

// ---- 屏幕键盘（软键盘）：鼠标点按直接发 set-1 扫描码给客机 ----
//   按钮不抢焦点、窗口不接焦点，真实键盘仍可照常用；按下发 make、松开发 break（与真键盘一致）。
//   在软键盘窗口内任意处松手都补发一次 break，避免按下后拖出按钮不松手导致键卡住。
static GtkWidget* osk_window = NULL;
static uint8_t   osk_held   = 0;   // 当前按住的键（防卡键）

typedef struct { const char* label; uint8_t sc; int w; } OskKey;

static void osk_send_make(uint8_t sc)  { if (sc) { io_keyboard_push(sc); osk_held = sc; } }
static void osk_send_break(uint8_t sc) { if (sc) io_keyboard_push_release(sc); if (osk_held == sc) osk_held = 0; }

static void osk_on_press(GtkButton* b, gpointer d) {
    (void)d;
    osk_send_make((uint8_t)GPOINTER_TO_INT(g_object_get_data(G_OBJECT(b), "osk_sc")));
}
static void osk_on_release(GtkButton* b, gpointer d) {
    (void)d;
    osk_send_break((uint8_t)GPOINTER_TO_INT(g_object_get_data(G_OBJECT(b), "osk_sc")));
}
// 在软键盘窗口内任意处松手都补发 break（覆盖"按下后拖出按钮"的情况）
static gboolean osk_win_release(GtkWidget* w, GdkEventButton* e, gpointer d) {
    (void)w; (void)e; (void)d;
    if (osk_held) { io_keyboard_push_release(osk_held); osk_held = 0; }
    return FALSE;
}

static const OskKey osk_func[] = {
    {"Esc",0x01,1},{"F1",0x3B,1},{"F2",0x3C,1},{"F3",0x3D,1},{"F4",0x3E,1},
    {"F5",0x3F,1},{"F6",0x40,1},{"F7",0x41,1},{"F8",0x42,1},
    {"F9",0x43,1},{"F10",0x44,1},{"F11",0x57,1},{"F12",0x58,1},{NULL,0,0}
};
static const OskKey osk_num[] = {
    {"`",0x29,1},{"1",0x02,1},{"2",0x03,1},{"3",0x04,1},{"4",0x05,1},
    {"5",0x06,1},{"6",0x07,1},{"7",0x08,1},{"8",0x09,1},{"9",0x0A,1},
    {"0",0x0B,1},{"-",0x0C,1},{"=",0x0D,1},{"Bksp",0x0E,2},{NULL,0,0}
};
static const OskKey osk_q[] = {
    {"Tab",0x0F,2},{"Q",0x10,1},{"W",0x11,1},{"E",0x12,1},{"R",0x13,1},{"T",0x14,1},
    {"Y",0x15,1},{"U",0x16,1},{"I",0x17,1},{"O",0x18,1},{"P",0x19,1},
    {"[",0x1A,1},{"]",0x1B,1},{"\\",0x2B,1},{NULL,0,0}
};
static const OskKey osk_a[] = {
    {"Caps",0x3A,2},{"A",0x1E,1},{"S",0x1F,1},{"D",0x20,1},{"F",0x21,1},{"G",0x22,1},
    {"H",0x23,1},{"J",0x24,1},{"K",0x25,1},{"L",0x26,1},{";",0x27,1},{"'",0x28,1},
    {"Enter",0x1C,2},{NULL,0,0}
};
static const OskKey osk_z[] = {
    {"Shift",0x2A,3},{"Z",0x2C,1},{"X",0x2D,1},{"C",0x2E,1},{"V",0x2F,1},{"B",0x30,1},
    {"N",0x31,1},{"M",0x32,1},{",",0x33,1},{".",0x34,1},{"/",0x35,1},
    {"Shift",0x36,3},{NULL,0,0}
};
static const OskKey osk_bot[] = {
    {"Ctrl",0x1D,2},{"Alt",0x38,2},{"Space",0x39,6},{"AltGr",0x38,2},{"Ctrl",0x1D,2},{NULL,0,0}
};
static const OskKey osk_arrow[] = {
    {"↑",0x48,1},{"←",0x4B,1},{"↓",0x50,1},{"→",0x4D,1},
    {"Ins",0x52,1},{"Del",0x53,1},{"Home",0x47,1},{"End",0x4F,1},{"PgUp",0x49,1},{"PgDn",0x51,1},{NULL,0,0}
};

static void osk_build(void) {
    osk_window = gtk_window_new(GTK_WINDOW_TOPLEVEL);
    gtk_window_set_title(GTK_WINDOW(osk_window), "屏幕键盘");
    // 不抢主窗口键盘焦点：真实键盘仍送达客机，软键盘只负责鼠标点按
    gtk_window_set_accept_focus(GTK_WINDOW(osk_window), FALSE);
    gtk_window_set_focus_on_map(GTK_WINDOW(osk_window), FALSE);
    if (vga.window) gtk_window_set_transient_for(GTK_WINDOW(osk_window), GTK_WINDOW(vga.window));
    GtkWidget* vbox = gtk_box_new(GTK_ORIENTATION_VERTICAL, 3);
    gtk_container_set_border_width(GTK_CONTAINER(osk_window), 6);

    static const OskKey* rows[] = { osk_func, osk_num, osk_q, osk_a, osk_z, osk_bot, osk_arrow, NULL };
    const int unit = 30;
    for (int r = 0; rows[r]; r++) {
        GtkWidget* hbox = gtk_box_new(GTK_ORIENTATION_HORIZONTAL, 2);
        for (int i = 0; rows[r][i].label; i++) {
            int w = rows[r][i].w ? rows[r][i].w : 1;
            GtkWidget* b = gtk_button_new_with_label(rows[r][i].label);
            gtk_widget_set_can_focus(b, FALSE);
            gtk_widget_set_size_request(b, w * unit, 34);
            g_object_set_data(G_OBJECT(b), "osk_sc", GINT_TO_POINTER((int)rows[r][i].sc));
            g_signal_connect(b, "pressed",  G_CALLBACK(osk_on_press),   NULL);
            g_signal_connect(b, "released", G_CALLBACK(osk_on_release), NULL);
            gtk_box_pack_start(GTK_BOX(hbox), b, FALSE, FALSE, 0);
        }
        gtk_box_pack_start(GTK_BOX(vbox), hbox, FALSE, FALSE, 0);
    }
    gtk_container_add(GTK_CONTAINER(osk_window), vbox);
    g_signal_connect(osk_window, "button-release-event", G_CALLBACK(osk_win_release), NULL);
    // 关闭窗口只是隐藏，可再次从菜单打开
    g_signal_connect(osk_window, "delete-event", G_CALLBACK(gtk_widget_hide_on_delete), NULL);
}

static void on_toggle_osk(GtkWidget* w, gpointer d) {
    (void)w; (void)d;
    if (!osk_window) osk_build();
    if (gtk_widget_get_visible(osk_window)) gtk_widget_hide(osk_window);
    else gtk_widget_show_all(osk_window);
}

// 频率预设：CPU 型号 + 主频（标签按用户习惯写成"型号 频率"）
static const struct { const char* label; uint32_t hz; } freq_presets[] = {
  { "8086  4.77 MHz",       4772720u  },
  { "80186 8 MHz",          8000000u  },
  { "80286 12 MHz",        12000000u  },
  { "80286 20 MHz (Turbo)", 20000000u },
};
#define FREQ_PRESET_N ((int)(sizeof(freq_presets) / sizeof(freq_presets[0])))

// 文本模式的显存快照：和快照一致、且光标闪烁状态没变时，跳过整屏重绘
#define TEXT_SHADOW_SIZE (80 * 25 * 2)
static uint8_t text_shadow[TEXT_SHADOW_SIZE];
static bool    text_shadow_valid = false;
static int     text_last_blink  = -1;

// CGA 16 色（标准 IBM 调色板）
RGBColor vga_palette[16] = {
  {0x00,0x00,0x00}, {0x00,0x00,0xAA}, {0x00,0xAA,0x00}, {0x00,0xAA,0xAA},
  {0xAA,0x00,0x00}, {0xAA,0x00,0xAA}, {0xAA,0x55,0x00}, {0xAA,0xAA,0xAA},
  {0x55,0x55,0x55}, {0x55,0x55,0xFF}, {0x55,0xFF,0x55}, {0x55,0xFF,0xFF},
  {0xFF,0x55,0x55}, {0xFF,0x55,0xFF}, {0xFF,0xFF,0x55}, {0xFF,0xFF,0xFF}
};

uint8_t vga_palette_256[256][3];
uint8_t vga_dac_mask = 0xFF;
uint8_t vga_dac_read_index = 0;
uint8_t vga_dac_write_index = 0;

#define VGA_GFX_ADDR  0xA0000
#define CGA_TEXT_ADDR 0xB8000
#define MDA_TEXT_ADDR 0xB0000

// CGA 图形显存和文本显存同址（B8000），16KB 分两个 bank 交错：
//   偶数扫描线在 B8000+0x0000，奇数扫描线在 B8000+0x2000
#define CGA_GRAPHICS_SIZE 0x4000

// ============================================================
// CGA 端口 0x3D8 / 0x3D9 状态
//   0x3D8：bit0=80列 bit1=图形 bit2=黑白 bit3=视频使能 bit4=640点 bit5=闪烁
//   0x3D9：bit0-3=背景色 bit4=加亮 bit5=调色板选择
// ============================================================
static uint8_t cga_mode_reg  = 0x29;   // 默认：80x25 文本、彩色、视频开
static uint8_t cga_color_reg = 0x30;
static bool    cga_bw = false;         // 0x3D8 bit2：黑白模式
// 当前 CGA 图形模式是"由 0x3D8 设定"(true) 还是"由 VGA 寄存器识别"(false)。
//   CGA 端口设定的模式以端口为准，别被 VGA 寄存器默认值切走；
//   由寄存器识别出的 CGA 模式则允许后续寄存器写继续重算（能正确切回文本/平面）。
static bool    cga_from_port = false;
// CGA 图形帧缓冲的真实物理基址：由 VGA 寄存器识别出的 CGA 模式可能落在 0xA0000
// （mm=0x01，多数 VGA BIOS 的兼容模式 6），也可能落在 0xB8000（mm=0x03，真 CGA）。
// 渲染按此基址读交错显存，而不是写死的 CGA_TEXT_ADDR。
static uint32_t cga_fb_addr = CGA_TEXT_ADDR;

uint8_t vga_read_cga_mode(void)  { return cga_mode_reg; }
uint8_t vga_read_cga_color(void) { return cga_color_reg; }

// 逐扫描线寄存器阴影：8088 MPH 之类靠"逐行改写 0x3D9 背景色"移相来生成伪色（1024 色），
// 而渲染每帧只做一次、读到的是帧末的最终值。这里在每次写 0x3D9/0x3D8 时，按"写那一刻的
// 扫描线"记录进阴影数组；渲染时按行取当时的寄存器值，伪色移相才正确。
//   cga_line_fid[y] == cga_frame_id 才表示该行本帧被写过，否则回退到全局当前值。
static uint8_t  cga_line_color[200];
static uint32_t cga_line_fid[200];
static uint32_t cga_frame_id = 0;

// ============================================================
// 调色板
// ============================================================
void vga_set_dac_entry(uint8_t index, uint8_t r, uint8_t g, uint8_t b) {
  vga_palette_256[index][0] = r & 0x3F;
  vga_palette_256[index][1] = g & 0x3F;
  vga_palette_256[index][2] = b & 0x3F;
}

void vga_set_default_palette(void) {
  static const uint8_t ega16[16][3] = {
    {0,0,0}, {0,0,42}, {0,42,0}, {0,42,42},
    {42,0,0}, {42,0,42}, {42,21,0}, {42,42,42},
    {21,21,21}, {21,21,63}, {21,63,21}, {21,63,63},
    {63,21,21}, {63,21,63}, {63,63,21}, {63,63,63}
  };
  for (int i = 0; i < 16; i++) {
    vga_set_dac_entry(i, ega16[i][0], ega16[i][1], ega16[i][2]);
  }
  for (int i = 16; i < 256; i++) {
    uint8_t v = (i - 16) * 4;
    if (v > 63) v = 63;
    vga_set_dac_entry(i, v, v, v);
  }
}

// ============================================================
// 磁盘读写指示灯（窗口底部拉出的一条窄条，1:1 不随画面缩放）
// ============================================================
#define LED_BAR_H   16       // 指示灯条高度（像素）
#define LED_ON_US   150000   // 一次读写后亮起时长（微秒）

// 活动灯槽位：0=硬盘(IDE) 1=软盘(FDC) 2=CD-ROM 3=键盘
#define LED_ACT_N   4

static gint64      led_last[LED_ACT_N] = { 0, 0, 0, 0 };
static bool        led_dirty = false;

// 灯条顺序：电源 硬盘 软盘 CD 键盘 大写 数字 滚动
static const char* led_name[] = { "PWR", "HDD", "FDD", "CD", "KBD", "CAPS", "NUM", "SCR" };
#define LED_TOTAL   ((int)(sizeof(led_name) / sizeof(led_name[0])))

static uint8_t led_lock_shown = 0xFF;   // 上次画出来的锁定键状态（变了要重画）

void vga_led_activity(int which) {
  if (which < 0 || which >= LED_ACT_N) return;
  led_last[which] = g_get_monotonic_time();
  led_dirty = true;
}

static bool led_is_on(int which) {
  if (!led_last[which]) return false;
  return (g_get_monotonic_time() - led_last[which]) < LED_ON_US;
}

// 第 i 个灯当前亮不亮
static bool led_state(int i) {
  switch (i) {
    case 0: return true;                              // PWR：模拟器在跑就常亮
    case 1: return led_is_on(0);                      // HDD
    case 2: return led_is_on(1);                      // FDD
    case 3: return led_is_on(2);                      // CD（以后加）
    case 4: return led_is_on(3);                      // KBD：按一下键盘就闪
    case 5: return (io_keyboard_leds() & 0x01) != 0;  // Caps Lock
    case 6: return (io_keyboard_leds() & 0x02) != 0;  // Num Lock
    case 7: return (io_keyboard_leds() & 0x04) != 0;  // Scroll Lock
    default: return false;
  }
}

// 灯的状态有变化（点亮、到时熄灭、锁定键翻转）返回 true，并清标志
bool vga_led_dirty(void) {
  gint64 now = g_get_monotonic_time();
  for (int i = 0; i < LED_ACT_N; i++) {
    if (led_last[i] && now - led_last[i] >= LED_ON_US) {
      led_last[i] = 0;
      led_dirty = true;
    }
  }
  if (io_keyboard_leds() != led_lock_shown) led_dirty = true;
  bool d = led_dirty;
  led_dirty = false;
  return d;
}

// 画面之外的窗口高度：指示灯条
static int led_bar_y(void) {
  if (!vga.drawing_area) return LED_BAR_H;
  int win_h = gtk_widget_get_allocated_height(vga.drawing_area);
  int y = win_h - LED_BAR_H;
  return (y < 0) ? 0 : y;
}

// 灯条缓存：只在"某个灯状态变了"或窗口宽度变了时重画一次，
// 平时每帧只把缓存位图贴上去。原来每帧画 8 个方块 + 8 次 cairo 文字，
// 是绘制回调里最贵的一段。
static cairo_surface_t* led_cache = NULL;
static int     led_cache_w = 0;
static uint8_t led_cache_state = 0xFF;

static void draw_led_bar(cairo_t* cr) {
  int win_w = gtk_widget_get_allocated_width(vga.drawing_area);
  if (win_w < 1) win_w = 1;
  int bar_y = led_bar_y();

  uint8_t state = 0;
  for (int i = 0; i < LED_TOTAL; i++) if (led_state(i)) state |= (uint8_t)(1u << i);

  if (!led_cache || led_cache_w != win_w || led_cache_state != state) {
    if (led_cache) cairo_surface_destroy(led_cache);
    led_cache = cairo_image_surface_create(CAIRO_FORMAT_RGB24, win_w, LED_BAR_H);
    cairo_t* c2 = cairo_create(led_cache);

    cairo_set_source_rgb(c2, 0.08, 0.08, 0.08);
    cairo_rectangle(c2, 0, 0, win_w, LED_BAR_H);
    cairo_fill(c2);

    cairo_select_font_face(c2, "monospace", CAIRO_FONT_SLANT_NORMAL,
                           CAIRO_FONT_WEIGHT_NORMAL);
    cairo_set_font_size(c2, 9);

    double x = 8;
    for (int i = 0; i < LED_TOTAL; i++) {
      double v = (state & (1u << i)) ? 1.0 : 0.18;   // 亮白 / 暗灰
      cairo_set_source_rgb(c2, v, v, v);
      cairo_rectangle(c2, x, 3, 10, LED_BAR_H - 6);
      cairo_fill(c2);

      cairo_set_source_rgb(c2, 0.55, 0.55, 0.55);
      cairo_move_to(c2, x + 13, LED_BAR_H - 5);
      cairo_show_text(c2, led_name[i]);
      x += 44;
    }
    cairo_destroy(c2);

    led_cache_w = win_w;
    led_cache_state = state;
    // 记下画出的锁定键状态，供 vga_led_dirty() 比较
    led_lock_shown = io_keyboard_leds();
  }

  cairo_set_source_surface(cr, led_cache, 0, bar_y);
  cairo_paint(cr);
}

// ============================================================
// GTK 绘制回调
// ============================================================
gboolean vga_on_draw(GtkWidget* widget, cairo_t* cr, gpointer data) {
  (void)widget; (void)data;

  cairo_set_antialias(cr, CAIRO_ANTIALIAS_NONE);

  if (!vga.drawing_area || !vga.surface) return FALSE;

  int win_w = gtk_widget_get_allocated_width(vga.drawing_area);
  int win_h = gtk_widget_get_allocated_height(vga.drawing_area);
  if (win_w <= 0 || win_h <= 0) return FALSE;

  // 画面只占上面部分，底部留给指示灯条
  int usable_h = win_h - LED_BAR_H;
  if (usable_h < 1) usable_h = 1;

  int scale_x = win_w / vga.width;
  int scale_y = usable_h / vga.height;
  int scale = (scale_x < scale_y) ? scale_x : scale_y;
  if (scale < 1) scale = 1;

  double dst_w = vga.width * scale;
  double dst_h = vga.height * scale;
  double dst_x = (win_w - dst_w) / 2.0;
  double dst_y = (usable_h - dst_h) / 2.0;

  // 画面没铺满整个绘图区时才需要黑底（窗口被拉大时留黑边）
  if (dst_w < win_w || dst_h < usable_h) {
    cairo_set_source_rgb(cr, 0, 0, 0);
    cairo_paint(cr);
  }

  cairo_save(cr);
  cairo_translate(cr, dst_x, dst_y);
  cairo_scale(cr, scale, scale);
  cairo_set_source_surface(cr, vga.surface, 0, 0);
  cairo_pattern_set_filter(cairo_get_source(cr), CAIRO_FILTER_NEAREST);
  cairo_paint(cr);
  cairo_restore(cr);

  draw_led_bar(cr);

  return FALSE;
}

// ============================================================
// 初始化
// ============================================================
static bool vga_headless = false;   // -console：不建窗口，不渲染到 cairo

void vga_set_headless(bool on) { vga_headless = on; }

void vga_init(int argc, char** argv) {
  if (vga_headless) {
    // 无窗口模式：只把状态摆好，供 BIOS 的 INT 10h 路径使用
    vga.mode = MODE_TEXT_80x25;
    vga.width = 640;
    vga.height = 400;
    vga.cursor_x = 0;
    vga.cursor_y = 0;
    vga.cursor_visible = true;
    // 诊断用：headless 也分配帧缓冲（纯内存 + cairo image surface，不建任何 GTK 对象），
    // 这样 vga_render()/vga_debug_dump() 在无窗口模式下也能跑，便于对比渲染结果。
    vga.pixels = malloc(vga.width * vga.height * sizeof(uint32_t));
    memset(vga.pixels, 0, vga.width * vga.height * sizeof(uint32_t));
    vga.surface = cairo_image_surface_create_for_data(
        (unsigned char*)vga.pixels, CAIRO_FORMAT_RGB24,
        vga.width, vga.height, vga.width * sizeof(uint32_t));
    vga_apply_font();
    vga_set_default_palette();
    memset(vga_seq_regs, 0, sizeof(vga_seq_regs));
    memset(vga_gc_regs, 0, sizeof(vga_gc_regs));
    vga_seq_regs[2] = 0x0F;   // Map Mask 默认写全部平面
    vga_gc_regs[8]  = 0xFF;   // Bit Mask 默认全部位可改
    printf("[VGA] headless（-console）\n");
    return;
  }

  

  vga.mode = MODE_TEXT_80x25;
  vga.width = 640;
  vga.height = 400;
  vga.cursor_x = 0;
  vga.cursor_y = 0;
  vga.cursor_visible = true;

  vga.window = gtk_window_new(GTK_WINDOW_TOPLEVEL);
  gtk_window_set_title(GTK_WINDOW(vga.window), "IBM PC Emulator");
  gtk_window_set_default_size(GTK_WINDOW(vga.window), vga.width, vga.height + LED_BAR_H);
  // 焦点不在绘图区时（点了窗口边框/标题栏）也要能收到按键
  gtk_widget_add_events(vga.window,
                        GDK_KEY_PRESS_MASK |
                        GDK_KEY_RELEASE_MASK |
                        GDK_FOCUS_CHANGE_MASK);

  vga.drawing_area = gtk_drawing_area_new();
  gtk_widget_set_size_request(vga.drawing_area, vga.width, vga.height + LED_BAR_H);
  gtk_widget_set_can_focus(vga.drawing_area, TRUE);
  gtk_widget_add_events(vga.drawing_area,
                        GDK_KEY_PRESS_MASK |
                        GDK_KEY_RELEASE_MASK |
                        GDK_BUTTON_PRESS_MASK |
                        GDK_BUTTON_RELEASE_MASK |
                        GDK_POINTER_MOTION_MASK);
  // 顶部工具栏：「机器」菜单（换软盘 A:/B:、重启）与「频率」菜单（CPU 主频预设）。
  // BIOS 只在启动时选，运行中不可改；软盘可随时换。
  GtkWidget* vbox    = gtk_box_new(GTK_ORIENTATION_VERTICAL, 0);
  GtkWidget* menubar = gtk_menu_bar_new();

  // ---- 「机器」菜单 ----
  GtkWidget* mi_machine   = gtk_menu_item_new_with_label("Machine");
  GtkWidget* menu_machine = gtk_menu_new();
  gtk_menu_item_set_submenu(GTK_MENU_ITEM(mi_machine), menu_machine);
  GtkWidget* it_a = gtk_menu_item_new_with_label("Change Floppy A:");
  GtkWidget* it_b = gtk_menu_item_new_with_label("Change Floppy B:");
  GtkWidget* it_r = gtk_menu_item_new_with_label("Restart");
  gtk_menu_shell_append(GTK_MENU_SHELL(menu_machine), it_a);
  gtk_menu_shell_append(GTK_MENU_SHELL(menu_machine), it_b);
  gtk_menu_shell_append(GTK_MENU_SHELL(menu_machine), gtk_separator_menu_item_new());
  gtk_menu_shell_append(GTK_MENU_SHELL(menu_machine), it_r);
  // 屏幕键盘（软键盘）：鼠标点按送扫描码
  GtkWidget* it_kbd = gtk_menu_item_new_with_label("屏幕键盘");
  gtk_menu_shell_append(GTK_MENU_SHELL(menu_machine), it_kbd);
  g_signal_connect(it_kbd, "activate", G_CALLBACK(on_toggle_osk), NULL);
  g_signal_connect(it_a, "activate", G_CALLBACK(on_mount_floppy), GINT_TO_POINTER(0));
  g_signal_connect(it_b, "activate", G_CALLBACK(on_mount_floppy), GINT_TO_POINTER(1));
  g_signal_connect(it_r, "activate", G_CALLBACK(on_reset_machine), NULL);
  gtk_menu_shell_append(GTK_MENU_SHELL(menubar), mi_machine);

  // ---- 「频率」菜单（单选，默认勾上当前主频）----
  GtkWidget* mi_freq   = gtk_menu_item_new_with_label("Frequency");
  GtkWidget* menu_freq = gtk_menu_new();
  gtk_menu_item_set_submenu(GTK_MENU_ITEM(mi_freq), menu_freq);
  GSList* freq_group = NULL;
  for (int i = 0; i < FREQ_PRESET_N; i++) {
    GtkWidget* it = gtk_radio_menu_item_new_with_label(freq_group, freq_presets[i].label);
    freq_group = gtk_radio_menu_item_get_group(GTK_RADIO_MENU_ITEM(it));
    gtk_menu_shell_append(GTK_MENU_SHELL(menu_freq), it);
    g_signal_connect(it, "activate", G_CALLBACK(on_set_freq),
                     GINT_TO_POINTER((int)freq_presets[i].hz));
    if (freq_presets[i].hz == CPU_CLK_HZ)
      gtk_check_menu_item_set_active(GTK_CHECK_MENU_ITEM(it), TRUE);
  }
  gtk_menu_shell_append(GTK_MENU_SHELL(menubar), mi_freq);

  gtk_box_pack_start(GTK_BOX(vbox), menubar, FALSE, FALSE, 0);
  gtk_box_pack_start(GTK_BOX(vbox), vga.drawing_area, TRUE, TRUE, 0);
  gtk_container_add(GTK_CONTAINER(vga.window), vbox);

  g_signal_connect(vga.drawing_area, "draw",
                   G_CALLBACK(vga_on_draw), NULL);

  vga.pixels = malloc(vga.width * vga.height * sizeof(uint32_t));
  memset(vga.pixels, 0, vga.width * vga.height * sizeof(uint32_t));

  vga.surface = cairo_image_surface_create_for_data(
    (unsigned char*)vga.pixels,
    CAIRO_FORMAT_RGB24,
    vga.width,
    vga.height,
    vga.width * sizeof(uint32_t));

  vga_apply_font();
  vga_set_default_palette();
  memset(vga_seq_regs, 0, sizeof(vga_seq_regs));
  memset(vga_gc_regs, 0, sizeof(vga_gc_regs));
  vga_seq_regs[2] = 0x0F;   // Map Mask 默认写全部平面
  vga_gc_regs[8]  = 0xFF;   // Bit Mask 默认全部位可改

  gtk_widget_show_all(vga.window);
  gtk_widget_grab_focus(vga.drawing_area);

  printf("[VGA] initialized %dx%d\n", vga.width, vga.height);
}

void vga_cleanup(void) {
  if (led_cache) {
    cairo_surface_destroy(led_cache);
    led_cache = NULL;
  }
  if (vga.surface) {
    cairo_surface_destroy(vga.surface);
    vga.surface = NULL;
  }
  if (vga.pixels) {
    free(vga.pixels);
    vga.pixels = NULL;
  }
  vga.window = NULL;
  vga.drawing_area = NULL;
}

// ============================================================
// 画布按需重建
//   抽出来自 vga_set_mode，是因为 EGA/VGA 平面模式的分辨率要由 CRTC 动态推导
//   （如 EGA 0x0D=320x200x16、0x10=640x350x16）——这些尺寸的调整发生在
//   vga.mode 不变的情况下，必须能在不重设模式的前提下重算 stride。
// ============================================================
static void vga_ensure_canvas(int nw, int nh) {
  if (vga.pixels && nw == vga.width && nh == vga.height) return;

  if (vga.surface) { cairo_surface_destroy(vga.surface); vga.surface = NULL; }
  if (vga.pixels)  { free(vga.pixels); vga.pixels = NULL; }

  vga.width  = nw;
  vga.height = nh;

  if (!vga_headless) {
    gtk_widget_set_size_request(vga.drawing_area, vga.width, vga.height + LED_BAR_H);
    gtk_window_resize(GTK_WINDOW(vga.window), vga.width, vga.height + LED_BAR_H);
  }

  vga.pixels = malloc(vga.width * vga.height * sizeof(uint32_t));
  memset(vga.pixels, 0, vga.width * vga.height * sizeof(uint32_t));

  vga.surface = cairo_image_surface_create_for_data(
    (unsigned char*)vga.pixels,
    CAIRO_FORMAT_RGB24,
    vga.width,
    vga.height,
    vga.width * sizeof(uint32_t));
}

// ============================================================
// 模式切换
// ============================================================
void vga_set_mode(VideoMode mode) {
  vga.mode = mode;

  // 只有位平面模式（EGA/VGA 16 色：0Dh/0Fh/10h/12h）才把 A0000 窗口交给 vga_planes；
  //   文本、CGA、chain4 的 13h 都保持扁平显存，渲染路径不受影响。
  vga_mem_window_active = (mode == MODE_GFX_640x480);

  int nw = vga.width, nh = vga.height;
  switch (mode) {
    case MODE_TEXT_80x25: nw = 640; nh = 25 * vga_char_h; break;   // CGA 字库=200 行、VGA=400 行
    case MODE_TEXT_40x25: nw = 320; nh = 25 * vga_char_h; break;
    case MODE_GFX_640x480: nw = 640; nh = 480; break;
    case MODE_GFX_320x200: nw = 320; nh = 200; break;
    // CGA 图形：内部按 640x400 呈现（横向/纵向各放大一倍，保持 4:3 比例）
    case MODE_CGA_320x200: nw = 640; nh = 400; break;
    case MODE_CGA_640x200: nw = 640; nh = 400; break;
  }

  // ★ 只有画布尺寸真的变了才重建 surface / 改窗口尺寸。
  //   CGA 的 320x200 与 640x200 都按 640x400 呈现；8088 MPH 之类演示常在同一帧里
  //   快速切模式（320↔640）做滚动/多色效果，若每次都重建 surface + resize 窗口，
  //   会严重闪屏。
  fprintf(stderr, "[VGA] set_mode %d -> %dx%d\n", (int)mode, nw, nh);
  vga_ensure_canvas(nw, nh);

  vga.cursor_x = 0;
  vga.cursor_y = 0;
  text_shadow_valid = false;

  // ★ 切模式时不再清显存：真实 CGA 通过端口 0x3D8 切模式不会清 VRAM，清屏由
  //   BIOS（INT 10h set mode 自行填屏）或程序负责。每次切换都 memset 16KB 会把
  //   演示刚画好的内容冲掉 —— 这正是"切模式做效果就闪屏/白屏"的根因。
  //   复位路径由 vga_reset() → vga_clear() 负责清屏。

  if (!vga_headless) gtk_widget_grab_focus(vga.drawing_area);
}

// ============================================================
// CGA 端口 0x3D8 / 0x3D9
// ============================================================
void vga_write_cga_mode(uint8_t val) {
  cga_mode_reg = val;
  cga_bw = (val & 0x04) != 0;

  // bit3=0 表示关闭视频输出（程序趁消隐期改显存时常用）：只记寄存器，不切模式，
  // 否则会被误判成 40 列文本，反复清屏把画面冲掉
  if (!(val & 0x08)) return;

  // 位组合 → 视频模式；只在模式真的变了时才重设
  VideoMode want;
  if (val & 0x02)
    want = (val & 0x10) ? MODE_CGA_640x200 : MODE_CGA_320x200;
  else
    want = (val & 0x01) ? MODE_TEXT_80x25 : MODE_TEXT_40x25;

  cga_from_port = (want == MODE_CGA_320x200 || want == MODE_CGA_640x200);
  if (cga_from_port) cga_fb_addr = CGA_TEXT_ADDR;   // 0x3D8 设定的真 CGA 帧缓冲恒在 B8000
  if (want != vga.mode) vga_set_mode(want);
}

void vga_write_cga_color(uint8_t val) {
  cga_color_reg = val;
  // 记录逐扫描线阴影：写这一刻正在显示的扫描行用这个 0x3D9 值
  int ln = vga_current_scanline();
  if (ln >= 0 && ln < 200) { cga_line_color[ln] = val; cga_line_fid[ln] = cga_frame_id; }
}

// ============================================================
// 从 VGA 寄存器推导视频模式
//   客机显卡 ROM 接管 INT 10h 后直接写 Sequencer/GC/CRTC 寄存器进图形模式，
//   本模拟器此前只在 init/reset/CGA 0x3D8 时切模式，导致 Windows 等直接写寄存器的
//   程序一进图形模式就被渲染成文本（"退回文本模式"）。这里每次写 VGA 寄存器后重算模式。
// ============================================================
void vga_update_mode(void) {
    // ★ 临时诊断：把模式判定所用的寄存器变化写入文件（静默，不刷终端；定位后删除）
    {
        static uint8_t pg=0xFF, ps=0xFF, pc12=0xFF, pc01=0xFF, pc13=0xFF, pc07=0xFF, pmm=0xFF, pm=0xFF;
        static FILE* df = NULL;
        if (!df) df = fopen("d:/dos86/vga_diag.log", "w");
        uint8_t ng = vga_gc_regs[6], ns = vga_seq_regs[4];
        uint8_t nc12 = vga_crtc_regs[0x12], nc01 = vga_crtc_regs[0x01];
        uint8_t nc13 = vga_crtc_regs[0x13], nc07 = vga_crtc_regs[0x07];
        uint8_t nmm = (ng >> 2) & 0x03, nm = (uint8_t)vga.mode;  // mm 是 GC[6] 的 bit3-2
        if (ng!=pg||ns!=ps||nc12!=pc12||nc01!=pc01||nc13!=pc13||nc07!=pc07||nmm!=pmm||nm!=pm) {
            pg=ng; ps=ns; pc12=nc12; pc01=nc01; pc13=nc13; pc07=nc07; nmm=nmm; pm=nm;
            if (df) { fprintf(df, "[DBG] gc6=%02X seq4=%02X c12=%02X c01=%02X c13=%02X c07=%02X mm=%d -> mode=%d addr=%X win=%d fp=%d\n",
                    ng, ns, nc12, nc01, nc13, nc07, nmm, vga.mode, cga_fb_addr, vga_mem_window_active, cga_from_port);
                    fflush(df); }
        }
    }
    // CGA 模式若由 0x3D8 端口设定，则以端口为准（CGA 上没有 GC，VGA 寄存器可能是
    //   默认值，别被 GC[6] 误判切走）；由 VGA 寄存器识别出的 CGA 模式则允许后续重算。
    if ((vga.mode == MODE_CGA_320x200 || vga.mode == MODE_CGA_640x200) && cga_from_port)
        return;

    uint8_t gc  = vga_gc_regs[6];        // GC Misc 寄存器
    uint8_t seq = vga_seq_regs[4];       // Sequencer Memory Mode：bit3=chain4（256 色）
    // ★ GC Misc 的显存映射选择是 bit3-2（不是 bit0-1）：
    //   00/01=A0000(图形) 10=B0000(单色文本) 11=B8000(彩色文本)。
    //   原代码误取 bit0-1，会把 gc6=0x0D(真实 B8000 图形)错判成 A0000。
    uint8_t mm  = (gc >> 2) & 0x03;

    if (mm >= 0x02) {                    // B0000 单色文本 / B8000 彩色文本或 CGA 图形
        // ★ B8000 上"彩色文本"与"CGA 图形模式(04/05/06h)"同址：VGA BIOS 设这些图形
        //   模式时只写 VGA 寄存器、不写 0x3D8，且 GC[6] 内存映射与彩色文本同为 B8000。
        //   只按内存映射判会把 CGA 图形当文本，把图形位图当字符码画成满屏乱字符。
        //   用 Sequencer 时钟模式(Seq[1] bit0)区分：0=9 点字符时钟(文本)，1=8 点像素
        //   时钟(图形)；再加"垂直显示 <=300 行"约束，避免误伤 80x50 等 8 点时钟文本。
        int vde200 = vga_crtc_regs[0x12]
                   | ((vga_crtc_regs[0x07] & 0x02) << 7)
                   | ((vga_crtc_regs[0x07] & 0x40) << 3);
        // 加 GC[6] bit0（图形位）鉴别：避免把 80x50 等 8 点时钟文本模式误判成 CGA 图形。
        if (mm == 0x03 && (gc & 0x01) && (vga_seq_regs[1] & 0x01) && (vde200 + 1) <= 300) {
            uint8_t hd = vga_crtc_regs[0x01];
            VideoMode w2 = (hd >= 60) ? MODE_CGA_640x200 : MODE_CGA_320x200;
            cga_from_port = false;
            cga_fb_addr = CGA_TEXT_ADDR;
            if (vga.mode != w2) vga_set_mode(w2);
            return;
        }
        // 列数以 CRTC 的 Horizontal Displayed（index 01）为准。VGA 上"每行字符数"
        // 的权威来源是它，而不是 CGA 兼容口 0x3D8 的快照。
        //   原先这里只判断"当前是否已是文本模式"，而把被误切成 40 列的
        //   MODE_TEXT_40x25 也当作合法状态直接 return，于是只要有人往 0x3D8 写了
        //   一个缺少 bit0 的值，就再也回不到 80 列 —— 表现为 Windows 2.03 之类
        //   卡死在 40x25。
        //   CRTC 未编程（0）或取值不合理时一律按 80 列，这是绝大多数文本软件的常态。
        uint8_t hdisp = vga_crtc_regs[0x01];
        VideoMode want = ((hdisp == 0) || (hdisp >= 60))
                       ? MODE_TEXT_80x25 : MODE_TEXT_40x25;
        if (vga.mode != want) {
            
            vga_set_mode(want);
        }
        return;
    }
    // ★ chain4 是 Sequencer reg4 的 bit3（0x08），不是 bit0。
    //   VGA BIOS 的模式表里 13h 写的是 reg4=0x0E（bit1|bit2|bit3），
    //   16 色平面模式（0Dh/10h/12h）写的是 0x06（bit1|bit2，odd/even 关闭=顺序平面）。
    //   按 bit0 判会把 13h 误当成平面模式：既走了平面渲染（读到空平面→黑屏），
    //   又抢走了它本该用的扁平显存布局。
    if (seq & 0x08) {                    // chain4 → 320x200x256
        if (vga.mode != MODE_GFX_320x200) {
            
            vga_set_mode(MODE_GFX_320x200);
        }
        return;
    }
    // ★ CGA 兼容图形模式（4/5/6h）常由 VGA BIOS 映射在 0xA0000（mm=0x01/0x00，GC[6]=0x05 一类），
    //   此时 mm<0x02 不会被上面的 B8000 分支捕获，会掉进下面的位平面渲染 → 整屏乱码。
    //   识别条件：A0000 上只有图形模式（无文本），且"图形位(GC[6] bit0) +
    //   垂直显示<=300 行(200 行)"足以判定是 CGA —— 真·位平面 VGA/EGA 至少 350 行，
    //   不会被误伤；chain4(13h)已在上面单独处理。这里不强制 Seq[1] bit0（个别 BIOS 设
    //   CGA 兼容模式 6 未必置 8 点时钟位），以免漏判。
    //   帧缓冲基址按 mm 取：B8000(map=3)或 A0000(map=0/1)。渲染据此读交错显存。
    if ((mm < 0x02) && (gc & 0x01)) {
        int vdeCGA = vga_crtc_regs[0x12]
                   | ((vga_crtc_regs[0x07] & 0x02) << 7)
                   | ((vga_crtc_regs[0x07] & 0x40) << 3);
        if ((vdeCGA + 1) <= 300) {
            uint8_t hd = vga_crtc_regs[0x01];
            VideoMode w2 = (hd >= 60) ? MODE_CGA_640x200 : MODE_CGA_320x200;
            cga_from_port = false;
            cga_fb_addr = (mm == 0x03) ? CGA_TEXT_ADDR : VGA_GFX_ADDR;
            if (vga.mode != w2) vga_set_mode(w2);
            return;
        }
    }
    // 位平面图形（EGA/VGA 16 色：640x480 / 640x350 / 640x400，以及 EGA 320x200）：
    //   宽高一律由 CRTC 推导 —— 每个字符时钟 = 8 像素，故 宽度 = 水平显示字符数 * 8，
    //   高度 = 垂直显示值 + 1。唯有如此才能区分 EGA 0x0D(320x200x16) 与
    //   0x10(640x350x16)：原先写死 640 宽，会把 320 宽的模式按两倍拉伸并错行。
    int vde = vga_crtc_regs[0x12]
            | ((vga_crtc_regs[0x07] & 0x02) << 7)
            | ((vga_crtc_regs[0x07] & 0x40) << 3);
    int h = vde + 1;
    if (h < 200) h = 200;
    if (h > 480) h = 480;
    if (vga.mode != MODE_GFX_640x480) {
        
        vga_set_mode(MODE_GFX_640x480);
    }

    // ★ CRTC 的 Horizontal Displayed(0x01) 同样是"字符数 − 1"，必须 +1 才是 80/40 列。
    //   原先漏了 +1：mode 12h 写的 0x4F(=79) 被算成 632 宽、行跨距 79 字节，而客机
    //   是按 80 字节/行铺位平面 → 每行错开 1 字节，整幅画面沿对角撕裂
    //   （Windows 2.03 的桌面就是这种斜向拖影；实心色条因整体位移看不出、文字最明显）。
    int hdisp = vga_crtc_regs[0x01];
    int w = (hdisp ? hdisp + 1 : 80) * 8;  // 未编程按 80 列（640 宽）兜底
    if (w < 320) w = 320;
    if (w > 720) w = 720;
    // 行跨距 = CRTC Offset(0x13) 的字数 × 2；未编程/小于显示宽度时退化为显示宽度/8。
    int stride = vga_crtc_regs[0x13] * 2;
    vga_plane_stride = (stride > w / 8) ? stride : w / 8;
    {
        static int lw = -1, lh = -1, ls = -1;
        if (w != lw || h != lh || vga_plane_stride != ls) {
            lw = w; lh = h; ls = vga_plane_stride;
            fprintf(stderr, "[VGA] planar %dx%d stride=%d\n", w, h, vga_plane_stride);
        }
    }
    vga_ensure_canvas(w, h);

   
}

// CGA 4 色调色板：模式 4/5 用
static void cga_get_palette4(uint32_t* out, uint32_t bg) {
  static const uint8_t pal0[3] = { 2, 4, 6 };   // 绿 红 棕
  static const uint8_t pal1[3] = { 3, 5, 7 };   // 青 洋红 白
  out[0] = bg;
  for (int i = 0; i < 3; i++) {
    uint8_t idx = (cga_color_reg & 0x20) ? pal1[i] : pal0[i];
    if (cga_color_reg & 0x10) idx |= 0x08;       // bit4：加亮
    RGBColor c = vga_palette[idx & 0x0F];
    out[i + 1] = (c.r << 16) | (c.g << 8) | c.b;
  }
}

// ============================================================
// 复合视频伪色（NTSC artifact colors）
//   真实 CGA 的复合输出把 R/G/B 三路色信号分别门控 0°/120°/240° 的 3.58MHz 色载波，
//   再叠加亮度。黑(0/8)与白(7/15)因三路相消而无色度，其余 6 个有色色的相位
//   正好落在 60° 的整数倍上 —— 这就是 8088 MPH 1K 色模式的基础。
//   模拟：像素 → 14.318MHz 复合采样（一个载波周期 = 4 个采样），
//   对每个输出像素取相邻 4 个采样做正交解调得到 (I,Q) 与亮度，再按 NTSC 矩阵转 RGB。
// ============================================================
static bool cga_composite = false;

void vga_set_composite(bool on) { cga_composite = on; }

// 各直接色的色度矢量 (I,Q) = 255·(cosθ, sinθ)，θ = 色载波相位
static const int16_t comp_i[16] = {
      0, -128, -128, -255,  255,  128,  128,    0,
      0, -128, -128, -255,  255,  128,  128,    0
};
static const int16_t comp_q[16] = {
      0, -221,  221,    0,    0, -221,  221,    0,
      0, -221,  221,    0,    0, -221,  221,    0
};
// 直接色亮度（0..255）。老 CGA 上 1-6、9-14 各自同亮度。
static const uint8_t comp_luma[16] = {
      0, 128, 128, 128, 128, 128, 128, 192,
     64, 168, 168, 168, 168, 168, 168, 255
};

// 采样相位 0/90/180/270° 的 cos/sin（±1/0）
static const int ct[4] = { 1, 0, -1, 0 };
static const int st[4] = { 0, 1, 0, -1 };

// 预计算：颜色 c 在相位 ph 下的复合采样电平 = 亮度 + I·cos + Q·sin
static int16_t comp_v[16][4];
static bool    comp_v_ready = false;

static void comp_prepare(void) {
  if (comp_v_ready) return;
  for (int c = 0; c < 16; c++)
    for (int ph = 0; ph < 4; ph++)
      comp_v[c][ph] = (int16_t)(comp_luma[c] + comp_i[c] * ct[ph] + comp_q[c] * st[ph]);
  comp_v_ready = true;
}

// 伪色+ 增强参数（直接写死，改这里调效果）
//   之前 COMP_SAT=1.30 / COMP_CON=1.10 会把画面整体提亮、像蒙了一层浅白滤镜，
//   现在归一到 1.0（忠实 NTSC 复合伪色原貌）。
#define COMP_SAT  1.00f   // 色度增益：1.0=原版，>1 更艳
#define COMP_CON  1.00f   // 对比度：1.0=原版
#define COMP_BRI  0.00f   // 亮度偏移：0.0=原版

static uint32_t comp_px(const uint8_t* idx, int w, int spp, int p) {
    int v[4], ph[4];
    int lim = w * spp - 1;
    for (int k = 0; k < 4; k++) {
        int s = p * spp + k;
        if (s < 0) s = 0;
        if (s > lim) s = lim;
        int pi = s / spp;
        if (pi >= w) pi = w - 1;
        ph[k] = s & 3;
        v[k] = comp_v[idx[pi]][ph[k]];
    }

    int y  = (v[0] + v[1] + v[2] + v[3] + 2) >> 2;
    int ii = (v[0] * ct[ph[0]] + v[1] * ct[ph[1]] +
              v[2] * ct[ph[2]] + v[3] * ct[ph[3]]) / 2;
    int qq = (v[0] * st[ph[0]] + v[1] * st[ph[1]] +
              v[2] * st[ph[2]] + v[3] * st[ph[3]]) / 2;

    // ★ 伪色+ 增强
    ii = (int)(ii * COMP_SAT);
    qq = (int)(qq * COMP_SAT);
    y  = (int)((y - 128) * COMP_CON + 128 + COMP_BRI * 255.0f);

    int r = y + ( 956 * ii + 621 * qq) / 2000;
    int g = y + (-272 * ii - 647 * qq) / 2000;
    int b = y + (-1106 * ii + 1703 * qq) / 2000;

    if (r < 0) r = 0; else if (r > 255) r = 255;
    if (g < 0) g = 0; else if (g > 255) g = 255;
    if (b < 0) b = 0; else if (b > 255) b = 255;

    return ((uint32_t)r << 16) | ((uint32_t)g << 8) | (uint32_t)b;
}
// ============================================================
// VGA 位平面显存读写（模式 12h 等平面模式用）
// ============================================================
// ★ 锁存寄存器（latch）：真实 EGA/VGA 每次 CPU 读平面显存都会把四个平面在该地址
//   的内容锁存下来。写模式 1 以它为数据源 —— Windows 的位块传输、图形拷贝大量
//   依赖这条路径。没有它，写模式 1 根本无法工作。
static uint8_t vga_latch[4] = {0, 0, 0, 0};

// ★ 临时诊断：写模式 2/3 与带逻辑运算的写（图标 BitBlt 排查用）。
//   由环境变量 VGA_WM2_TRACE 开启，避免影响正常构建。
static FILE* g_vga_trace = NULL;
static long  g_vga_trace_n = 0;

void vga_mem_write(uint32_t off, uint8_t val) {
    if (off >= VGA_PLANE_SIZE) return;

    uint8_t plane_mask = vga_seq_regs[2];        // 序列器 reg2：写哪些 plane
    uint8_t bit_mask   = vga_gc_regs[8];          // GC reg8：位掩码
    uint8_t set_reset  = vga_gc_regs[0];          // GC reg0：置位复位
    uint8_t en_sr      = vga_gc_regs[1];          // GC reg1：允许置位复位
    uint8_t rotate     = vga_gc_regs[3] & 0x07;   // GC reg3 低 3 位：数据循环
    uint8_t logic_op   = (vga_gc_regs[3] >> 3) & 0x03;
    uint8_t write_mode = vga_gc_regs[5] & 0x03;   // GC reg5 低 2 位：写模式

    // ★ 不做"是否为零就兜底"的改写：0x00 是合法值。
    //   Map Mask=0 表示不写任何平面（真·空操作）；Bit Mask=0 表示不改任何位。
    //   Windows 用"写模式 0 + 位掩码=字形字节"画文字，空白扫描行的字形字节正是 0x00，
    //   若把 0x00 兜底成 0xFF，空白行会被整行涂成前景色 → 文字周围出现实心黑块。
    //   正确的默认值（Map Mask=0x0F / Bit Mask=0xFF）在 vga_reset() 里设定。

    uint8_t data = val;
    if (rotate) data = (uint8_t)((data >> rotate) | (data << (8 - rotate)));

    // ★ 临时诊断：追踪写模式 2/3 与带逻辑运算的写（图标 BitBlt 排查）
    if (g_vga_trace_n < 50000 && (write_mode == 2 || write_mode == 3 || logic_op != 0)) {
        if (!g_vga_trace) {
            const char* p = getenv("VGA_WM2_TRACE");
            if (p) g_vga_trace = fopen(p, "w");
        }
        if (g_vga_trace) {
            fprintf(g_vga_trace,
                "WM=%d EN_SR=%02X SR=%02X BM=%02X LOP=%d OFF=%05X DATA=%02X PM=%02X\n",
                write_mode, en_sr, set_reset, bit_mask, logic_op, off, val, plane_mask);
            g_vga_trace_n++;
            fflush(g_vga_trace);
        }
    }

    for (int p = 0; p < 4; p++) {
        if (!(plane_mask & (1 << p))) continue;
        uint8_t old = vga_planes[p][off];
        uint8_t newb = old;
        switch (write_mode) {
            case 0: {   // 写模式 0：普通写
                uint8_t src = (en_sr & (1 << p))
                              ? ((set_reset & (1 << p)) ? 0xFF : 0x00)
                              : data;
                newb = (uint8_t)((old & ~bit_mask) | (src & bit_mask));
                switch (logic_op) {
                    case 0: newb = newb; break;
                    case 1: newb = (uint8_t)(old & newb); break;
                    case 2: newb = (uint8_t)(old | newb); break;
                    case 3: newb = (uint8_t)(old ^ newb); break;
                }
                break;
            }
            case 1: {   // 写模式 1：锁存器 → 目标（bit_mask 保护其余位不被清掉）
                newb = (uint8_t)((old & ~bit_mask) | (vga_latch[p] & bit_mask));
                break;
            }
            case 2: {   // 写模式 2：位扩展（每像素独立取 置位复位 或 CPU 数据对应位）
                for (int b = 0; b < 8; b++) {
                    if (!(bit_mask & (1 << b))) continue;   // 位掩码保护其余像素
                    uint8_t bitval = (en_sr & (1 << p))
                                     ? (uint8_t)((set_reset & (1 << p)) ? 1 : 0)
                                     : (uint8_t)((data >> b) & 1);
                    if (bitval) newb |=  (uint8_t)(1u << b);
                    else        newb &= (uint8_t)~(1u << b);
                }
                break;
            }
            case 3: {   // 写模式 3：置位复位
                newb = (en_sr & (1 << p))
                       ? ((set_reset & (1 << p)) ? 0xFF : 0x00)
                       : old;
                newb = (uint8_t)((old & ~bit_mask) | (newb & bit_mask));
                break;
            }
            default: newb = old; break;
        }
        vga_planes[p][off] = newb;
    }
}

uint8_t vga_mem_read(uint32_t off) {
    if (off >= VGA_PLANE_SIZE) return 0xFF;

    // ★ CPU 每次读平面显存都会刷新四个锁存寄存器（写模式 1 的数据来源）
    for (int p = 0; p < 4; p++) vga_latch[p] = vga_planes[p][off];

    uint8_t read_mode = (vga_gc_regs[5] >> 3) & 0x01;   // GC reg5 bit3：读模式

    if (!read_mode) {                       // 读模式 0：返回 Read Map Select 选中的平面
        uint8_t read_map = vga_gc_regs[4] & 0x03;
        return vga_planes[read_map][off];
    }

    // 读模式 1：颜色比较 —— 逐像素判断该点颜色是否等于 Color Compare，
    //   Color Don't Care（reg7）里置 1 的位不参与比较。
    //   每一位对应一个像素，比较相等则该位置 1（用于 pattern 填充类的软件检索）。
    uint8_t cmp = vga_gc_regs[2] & 0x0F;    // Color Compare
    uint8_t cdc = vga_gc_regs[7] & 0x0F;    // Color Don't Care
    uint8_t cmp_mask = (uint8_t)((~cdc) & 0x0F);
    uint8_t res = 0;
    for (int b = 0; b < 8; b++) {
        uint8_t pix = 0;
        for (int p = 0; p < 4; p++)
            pix |= (uint8_t)(((vga_latch[p] >> b) & 1) << p);
        if (((pix ^ cmp) & cmp_mask) == 0) res |= (uint8_t)(1 << b);
    }
    return res;
}
// ============================================================
// 自检：直接验证 16 色位平面图形路径（不依赖任何客机）。
//   VGA_SELFTEST=1 时由主程序在启动客机前调用，画已知图案并落盘 vga_selftest.ppm。
//   期望结果：上 4 条 100px 高色带依次为 蓝(plane0)/绿(plane1)/红(plane2)/深灰(plane3)；
//   中下白色实心方块；最底行蓝色点应落在每个 8 像素组的同一侧（验证像素位序）。
// ============================================================
void vga_self_test(void) {
    // 按 640x480 重新分配帧缓冲（无头初始化默认 640x400，直接改高会越界写）
    size_t npx = (size_t)640 * 480;
    vga.pixels = (uint32_t*)realloc(vga.pixels, npx * sizeof(uint32_t));
    memset(vga.pixels, 0, npx * sizeof(uint32_t));
    vga.mode = MODE_GFX_640x480;
    vga_mem_window_active = true;
    vga.width = 640; vga.height = 480;
    vga_plane_stride = 80;
    vga_set_default_palette();
    static const uint8_t idpal[16] = {0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15};
    memcpy(vga_attr_palette, idpal, 16);

    for (int p = 0; p < 4; p++) memset(vga_planes[p], 0, VGA_PLANE_SIZE);

    vga_seq_regs[2] = 0x0F;            // 写全部平面
    vga_gc_regs[8]  = 0xFF;            // 位掩码全开

    // (1) 写模式 3（置位复位填充）：4 条水平色带，验证 平面 p → 颜色位 p
    vga_gc_regs[5] = 3;                // 写模式 3
    vga_gc_regs[1] = 0x0F;             // 允许置位复位
    static const uint8_t band[4] = {1, 2, 4, 8};
    for (int b = 0; b < 4; b++) {
        vga_gc_regs[0] = band[b];      // 置位复位值 = 颜色
        for (int y = b*100; y < b*100 + 100; y++)
            for (int xb = 0; xb < 80; xb++)
                vga_mem_write((uint32_t)(y*80 + xb), 0x00);
    }

    // (2) 写模式 2 + 置位复位(白)：实心白方块，验证位展开/置位复位
    vga_gc_regs[5] = 2;
    vga_gc_regs[0] = 0x0F;
    vga_gc_regs[1] = 0x0F;
    for (int y = 420; y < 460; y++)
        for (int xb = 200; xb < 240; xb++)
            vga_mem_write((uint32_t)(y*80 + xb), 0xFF);

    // (3) 写模式 2 无置位复位：每字节只置 bit0，验证像素位序
    vga_gc_regs[1] = 0x00;
    for (int y = 470; y < 480; y++)
        for (int xb = 0; xb < 80; xb++)
            vga_mem_write((uint32_t)(y*80 + xb), 0x01);

    vga_debug_dump("D:/dos86/data/vga_selftest.ppm");
    fprintf(stderr, "[VGA] self-test dumped -> D:/dos86/data/vga_selftest.ppm\n");
}

// ============================================================
// 完整渲染
// ============================================================
bool vga_render(void) {
    if (!vga.pixels || !vga.surface) return false;

    const uint32_t W = (uint32_t)vga.width;
    uint32_t* __restrict px = vga.pixels;

    // ★ 临时诊断：首次进入 planar 模式时把 4 个平面的字节指纹写入文件（静默；定位后删除）
    if (vga.mode == MODE_GFX_640x480) {
        static FILE* rf = NULL;
        static bool dumped = false;
        if (!rf) rf = fopen("d:/dos86/vga_planes_raw.log", "w");
        if (!dumped) {
            dumped = true;
            for (int pl = 0; pl < 4; pl++) {
                const uint8_t* p = &vga_planes[pl][0];
                fprintf(rf, "PL%d row0 :", pl); for (int i = 0; i < 32; i++) fprintf(rf, " %02X", p[i]);
                fprintf(rf, "\nPL%d row1(+80):", pl); for (int i = 0; i < 32; i++) fprintf(rf, " %02X", p[80 + i]);
                fprintf(rf, "\n");
            }
            fprintf(rf, "stride=%d width=%d height=%d\n", vga_plane_stride, vga.width, vga.height);
            fflush(rf);
        }
    }

    // ------------------------------------------------------------
    // 文本模式 80x25 / 40x25
    // ------------------------------------------------------------
    if (vga.mode == MODE_TEXT_80x25 || vga.mode == MODE_TEXT_40x25) {
        int cols = (vga.mode == MODE_TEXT_80x25) ? 80 : 40;

        int blink = (int)((g_get_monotonic_time() / 500000) % 2);
        if (text_shadow_valid && blink == text_last_blink &&
            memcmp(text_shadow, &memory[CGA_TEXT_ADDR], TEXT_SHADOW_SIZE) == 0) {
            return false;
        }
        memcpy(text_shadow, &memory[CGA_TEXT_ADDR], TEXT_SHADOW_SIZE);
        text_shadow_valid = true;
        text_last_blink = blink;

        for (uint32_t i = 0; i < W * (uint32_t)vga.height; i++) px[i] = 0x000000;

        // 文本模式不套复合伪色：真实 CGA 文本用标准 16 色调色板显示，伪色只产生于
        // 图形模式（每像素 1-2 位）。套伪色会把文字染成蓝/粉绿等 artifact 色，故禁用。
        for (int row = 0; row < 25; row++) {
                for (int col = 0; col < cols; col++) {
                    int offset = (row * 80 + col) * 2;
                    uint8_t ch   = memory[CGA_TEXT_ADDR + offset];
                    uint8_t attr = memory[CGA_TEXT_ADDR + offset + 1];
                    uint8_t fg = attr & 0x0F;
                    uint8_t bg = (attr >> 4) & 0x07;
                    RGBColor fgc = vga_palette[fg];
                    RGBColor bgc = vga_palette[bg];
                    uint32_t fg32 = (fgc.r << 16) | (fgc.g << 8) | fgc.b;
                    uint32_t bg32 = (bgc.r << 16) | (bgc.g << 8) | bgc.b;
                    const uint8_t* font = font8x16[ch];
                    int px0 = col * 8, py = row * vga_char_h;
                    for (int y = 0; y < vga_char_h; y++) {
                        uint8_t bits = font[y];
                        uint32_t* dst = &px[(py + y) * W + px0];
                        for (int x = 0; x < 8; x++)
                            dst[x] = (bits & (0x80 >> x)) ? fg32 : bg32;
                    }
                }
            }

        // 光标
        int page = memory[0x462] & 0x07;
        int cx = memory[0x450 + page * 2];
        int cy = memory[0x450 + page * 2 + 1];
        if (!(memory[0x460] & 0x20) && cx < cols && cy < 25) {
            gint64 now = g_get_monotonic_time();
            if (((now / 500000) % 2) == 0) {
                int px0 = cx * 8, py = cy * vga_char_h;
                for (int y = vga_char_h - 2; y < vga_char_h; y++) {
                    uint32_t* dst = &px[(py + y) * W + px0];
                    for (int x = 0; x < 8; x++) dst[x] = 0xAAAAAA;
                }
            }
        }
    }
    // ------------------------------------------------------------
    // VGA 320x200x256
    // ------------------------------------------------------------
    else if (vga.mode == MODE_GFX_320x200) {
        for (int y = 0; y < 200; y++) {
            const uint8_t* src = &memory[VGA_GFX_ADDR + y * 320];
            uint32_t* dst = &px[y * W];
            for (int x = 0; x < 320; x++) {
                uint8_t pix = src[x];
                uint8_t r6 = vga_palette_256[pix][0];
                uint8_t g6 = vga_palette_256[pix][1];
                uint8_t b6 = vga_palette_256[pix][2];
                uint8_t r = (r6 << 2) | (r6 >> 4);
                uint8_t g = (g6 << 2) | (g6 >> 4);
                uint8_t b = (b6 << 2) | (b6 >> 4);
                dst[x] = ((uint32_t)r << 16) | ((uint32_t)g << 8) | b;
            }
        }
    }
    // ------------------------------------------------------------
    // VGA 640x480x16（位平面 + 属性控制器 + DAC）
    // ------------------------------------------------------------
    else if (vga.mode == MODE_GFX_640x480) {
        // 预计算 16 个 raw → 最终 RGB
        static uint32_t pal16[16];
        static uint8_t  pal16_last[16] = {0xFF,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
        bool need = false;
        for (int i = 0; i < 16; i++) {
            if (vga_attr_palette[i] != pal16_last[i]) { need = true; break; }
        }
        if (need) {
            for (int i = 0; i < 16; i++) {
                uint8_t dac = vga_attr_palette[i] & 0x3F;
                uint8_t r6 = vga_palette_256[dac][0];
                uint8_t g6 = vga_palette_256[dac][1];
                uint8_t b6 = vga_palette_256[dac][2];
                uint8_t r = (r6 << 2) | (r6 >> 4);
                uint8_t g = (g6 << 2) | (g6 >> 4);
                uint8_t b = (b6 << 2) | (b6 >> 4);
                pal16[i] = ((uint32_t)r << 16) | ((uint32_t)g << 8) | b;
            }
            memcpy(pal16_last, vga_attr_palette, 16);
        }

        const uint8_t* p0 = vga_planes[0];
        const uint8_t* p1 = vga_planes[1];
        const uint8_t* p2 = vga_planes[2];
        const uint8_t* p3 = vga_planes[3];

        int bytes_per_row = vga.width / 8;   // ★ 显示宽度（像素/8），随 CRTC 推导
        int stride = vga_plane_stride;       // ★ 显存行跨距（字节），来自 CRTC Offset
        for (int y = 0; y < vga.height; y++) {
            uint32_t* dst = &px[y * W];
            uint32_t base = (uint32_t)y * (uint32_t)stride;
            for (int xb = 0; xb < bytes_per_row; xb++) {
                uint32_t off = (base + xb) & 0xFFFF;   // 平面地址 16 位回绕
                uint8_t b0 = p0[off], b1 = p1[off], b2 = p2[off], b3 = p3[off];
                for (int bit = 7; bit >= 0; bit--) {
                    uint8_t raw = ((b0 >> bit) & 1) |
                                  (((b1 >> bit) & 1) << 1) |
                                  (((b2 >> bit) & 1) << 2) |
                                  (((b3 >> bit) & 1) << 3);
                    *dst++ = pal16[raw];
                }
            }
        }
    }
    // ------------------------------------------------------------
    // CGA 320x200 四色
    // ------------------------------------------------------------
    else if (vga.mode == MODE_CGA_320x200) {
        RGBColor bgc = vga_palette[cga_color_reg & 0x0F];
        uint32_t bg = (bgc.r << 16) | (bgc.g << 8) | bgc.b;
        uint32_t pal[4];
        if (cga_bw) {
            pal[0] = bg;
            pal[1] = pal[2] = pal[3] = 0xFFFFFF;
        } else {
            cga_get_palette4(pal, bg);
        }

        if (cga_composite) {
            comp_prepare();
            static uint8_t  idx[320];
            static uint32_t line[320];
            for (int y = 0; y < 200; y++) {
                // 逐行取当时的 0x3D9：8088MPH 靠逐行改背景色把色度相位偏移，生成伪色
                // ★ 旧实现用 cga_line_fid[y]==cga_frame_id 判定逐行回退，但写 0x3D9 时
                //   由 vga_current_scanline() 反推的扫描行与渲染的 y 常对不齐，导致大多数
                //   行回退到 cga_color_reg（若被程序设成亮/白色）→ 整屏白屏并逐帧闪烁。
                //   改为统一用当前 cga_color_reg：标准 CGA 复合伪色，稳定且颜色正确。
                uint8_t creg = cga_color_reg;
                uint8_t pidx[4];
                pidx[0] = creg & 0x0F;
                if (cga_bw) {
                    pidx[1] = pidx[2] = pidx[3] = 15;
                } else {
                    static const uint8_t p0[3] = { 2, 4, 6 };
                    static const uint8_t p1[3] = { 3, 5, 7 };
                    for (int i = 0; i < 3; i++) {
                        uint8_t v = (creg & 0x20) ? p1[i] : p0[i];
                        if (creg & 0x10) v |= 0x08;
                        pidx[i + 1] = v;
                    }
                }
                uint32_t cga_off = cga_from_port ? (((y & 1) ? 0x2000u : 0) + ((uint32_t)(y >> 1)) * 80u)
                                                : ((uint32_t)y * 80u);
                const uint8_t* src = (cga_fb_addr == VGA_GFX_ADDR && vga_mem_window_active)
                                      ? &vga_planes[0][cga_off] : &memory[cga_fb_addr + cga_off];
                for (int x = 0; x < 320; x++) {
                    uint8_t b = src[x >> 2];
                    idx[x] = pidx[(b >> ((3 - (x & 3)) * 2)) & 3];
                }
                for (int x = 0; x < 320; x++) line[x] = comp_px(idx, 320, 2, x);
                int dy = y * 2;
                uint32_t* d0 = &px[dy * W];
                uint32_t* d1 = &px[(dy + 1) * W];
                for (int x = 0; x < 320; x++) {
                    uint32_t c = line[x];
                    int dx = x * 2;
                    d0[dx] = d0[dx + 1] = c;
                    d1[dx] = d1[dx + 1] = c;
                }
            }
        } else {
            for (int y = 0; y < 200; y++) {
                uint32_t cga_off = cga_from_port ? (((y & 1) ? 0x2000u : 0) + ((uint32_t)(y >> 1)) * 80u)
                                                : ((uint32_t)y * 80u);
                const uint8_t* src = (cga_fb_addr == VGA_GFX_ADDR && vga_mem_window_active)
                                      ? &vga_planes[0][cga_off] : &memory[cga_fb_addr + cga_off];
                int dy = y * 2;
                uint32_t* d0 = &px[dy * W];
                uint32_t* d1 = &px[(dy + 1) * W];
                for (int x = 0; x < 320; x++) {
                    uint8_t b = src[x >> 2];
                    uint32_t c = pal[(b >> ((3 - (x & 3)) * 2)) & 3];
                    int dx = x * 2;
                    d0[dx] = d0[dx + 1] = c;
                    d1[dx] = d1[dx + 1] = c;
                }
            }
        }
    }
    // ------------------------------------------------------------
    // CGA 640x200 双色
    // ------------------------------------------------------------
    else if (vga.mode == MODE_CGA_640x200) {
        uint32_t off = 0x000000;
        uint32_t on  = 0xFFFFFF;
        if (cga_composite) {
            comp_prepare();
            static uint8_t  idx[640];
            static uint32_t line[640];
            // 模式 6（640x200 2 色）前景恒为白(15)，背景 = 0x3D9 位 0-3。
            // 背景用真实调色板色（带 chroma），不能用硬编码的 0（黑），否则伪色相位
            // 恒定、整幅画面颜色错乱。统一用当前 cga_color_reg（逐行阴影回退已移除，
            // 因其扫描行对不齐会导致白屏闪烁）。
            uint8_t fg6 = 0x0F;
            for (int y = 0; y < 200; y++) {
                uint8_t creg = cga_color_reg;
                uint8_t bg6 = creg & 0x0F;
                uint32_t cga_off = cga_from_port ? (((y & 1) ? 0x2000u : 0) + ((uint32_t)(y >> 1)) * 80u)
                                                : ((uint32_t)y * 80u);
                const uint8_t* src = (cga_fb_addr == VGA_GFX_ADDR && vga_mem_window_active)
                                      ? &vga_planes[0][cga_off] : &memory[cga_fb_addr + cga_off];
                for (int x = 0; x < 640; x++)
                    idx[x] = (src[x >> 3] & (0x80 >> (x & 7))) ? fg6 : bg6;
                for (int x = 0; x < 640; x++) line[x] = comp_px(idx, 640, 1, x);
                int dy = y * 2;
                memcpy(&px[dy * W], line, 640 * 4);
                memcpy(&px[(dy + 1) * W], line, 640 * 4);
                }
                cairo_surface_mark_dirty(vga.surface);
                cga_frame_id++;
            return true;
        }
        for (int y = 0; y < 200; y++) {
            uint32_t cga_off = cga_from_port ? (((y & 1) ? 0x2000u : 0) + ((uint32_t)(y >> 1)) * 80u)
                                            : ((uint32_t)y * 80u);
            const uint8_t* src = (cga_fb_addr == VGA_GFX_ADDR && vga_mem_window_active)
                                  ? &vga_planes[0][cga_off] : &memory[cga_fb_addr + cga_off];
            int dy = y * 2;
            uint32_t* d0 = &px[dy * W];
            uint32_t* d1 = &px[(dy + 1) * W];
            for (int xb = 0; xb < 80; xb++) {
                uint8_t b = src[xb];
                for (int bit = 7; bit >= 0; bit--) {
                    uint32_t c = (b & (1 << bit)) ? on : off;
                    int dx = xb * 8 + (7 - bit);
                    d0[dx] = c;
                    d1[dx] = c;
                }
            }
        }
    }

    cairo_surface_mark_dirty(vga.surface);

    cga_frame_id++;
    return true;
}

// ============================================================
// 辅助
// ============================================================
// ★ 临时诊断：把当前帧缓冲写成 PPM（验证图标/黑块用，定位后删除）
void vga_debug_dump(const char* path) {
    if (!vga.pixels) return;
    vga_render();
    FILE* f = fopen(path, "wb");
    if (!f) return;
    fprintf(f, "P6\n%d %d\n255\n", vga.width, vga.height);
    for (int i = 0; i < vga.width * vga.height; i++) {
        uint32_t c = vga.pixels[i];
        unsigned char rgb[3] = { (unsigned char)((c >> 16) & 0xFF),
                                 (unsigned char)((c >> 8) & 0xFF),
                                 (unsigned char)(c & 0xFF) };
        fwrite(rgb, 1, 3, f);
    }
    fclose(f);
}

void vga_clear(uint8_t attr) {
  for (int i = 0; i < 80 * 25; i++) {
    memory[CGA_TEXT_ADDR + i * 2] = ' ';
    memory[CGA_TEXT_ADDR + i * 2 + 1] = attr;
  }
  vga.cursor_x = 0;
  vga.cursor_y = 0;
}

// 机器复位（GTK「重启」按钮）：回到 80x25 文本模式并清屏
void vga_reset(void) {
  vga_set_mode(MODE_TEXT_80x25);
  vga_set_default_palette();
  // VGA 复位后的寄存器默认值（BIOS 随后会按模式重新编程；这里先给规范初值，
  //   免得客机在编程前写显存时，Map Mask / Bit Mask 为 0 导致写入被吞）。
  memset(vga_seq_regs, 0, sizeof(vga_seq_regs));
  memset(vga_gc_regs, 0, sizeof(vga_gc_regs));
  vga_seq_regs[2] = 0x0F;   // Map Mask：写全部 4 个平面
  vga_gc_regs[8]  = 0xFF;   // Bit Mask：全部位可改
  vga_clear(0x07);
  vga.cursor_visible = true;
}

// ============================================================
// 显卡选件 ROM（C000 段）
//   DIP 报 EGA/VGA 时 BIOS POST 在 0x1f8 把 IVT 的 INT 10h 偏移改成 0xFF53
//   （ROM 里 INT 1Bh/1Ch 共用的那句 IRET），等于"本机没有视频服务"，
//   然后等显卡选件 ROM 来接管 —— 真机规矩。POST 的选件扫描（0x2a9 起）：
//   从 C000 按 2KB 步长走，认偏移 0 的 0xAA55 签名、偏移 2 的长度（512 字节块）、
//   以及全 ROM 字节累加为 0（校验例程 0x1a0f），通过就 lcall 段:0003。
//   这里直接内嵌 Bochs LGPL VGA BIOS（见 vga_rom_vga.h），不依赖外部文件、
//   也不需要在启动时做任何选择：开机就是一台装好标准 VGA 的机器。
//   （原内嵌的 IBM 1986 ROM 被误判为 CGA、拒绝一切图形模式，故替换为通用 VGA BIOS。）
// ============================================================
#define VIDEO_ROM_SEG   0xC000
#define VIDEO_ROM_ADDR  ((uint32_t)VIDEO_ROM_SEG << 4)
#define VIDEO_ROM_SIZE  0x8000        // C0000-C7FFF：真卡 ROM 器件 32KB

void vga_video_rom_install(void) {
  uint8_t* rom = &memory[VIDEO_ROM_ADDR];
  // 标准 VGA BIOS（Bochs LGPL 0.9c）按装在 C0000 编译，长度恰为 0x8000，
  // 直接整段搬入，无需偏移或前置填充。签名 55 AA、长度 64 块、校验和 0，POST 可识别。
  memset(rom, 0x00, VIDEO_ROM_SIZE);
  memcpy(rom, vga_rom_vga, VGA_ROM_VGA_SIZE);
}

