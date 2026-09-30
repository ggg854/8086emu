// vga.c - CGA 模拟（GTK 版，闪烁光标）
#include "vga.h"
#include "cpu.h"
#include "io.h"
#include "font8x16.h"
#include "vga_rom_ibm.h"
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

VGA vga;

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

uint8_t vga_read_cga_mode(void)  { return cga_mode_reg; }
uint8_t vga_read_cga_color(void) { return cga_color_reg; }

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
    init_font();
    vga_set_default_palette();
    printf("[VGA] headless（-console）\n");
    return;
  }

  gtk_init(&argc, &argv);

  vga.mode = MODE_TEXT_80x25;
  vga.width = 640;
  vga.height = 400;
  vga.cursor_x = 0;
  vga.cursor_y = 0;
  vga.cursor_visible = true;

  vga.window = gtk_window_new(GTK_WINDOW_TOPLEVEL);
  gtk_window_set_title(GTK_WINDOW(vga.window), "DOS86 Emulator");
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
  GtkWidget* mi_machine   = gtk_menu_item_new_with_label("机器");
  GtkWidget* menu_machine = gtk_menu_new();
  gtk_menu_item_set_submenu(GTK_MENU_ITEM(mi_machine), menu_machine);
  GtkWidget* it_a = gtk_menu_item_new_with_label("切换软盘 A:");
  GtkWidget* it_b = gtk_menu_item_new_with_label("切换软盘 B:");
  GtkWidget* it_r = gtk_menu_item_new_with_label("重启");
  gtk_menu_shell_append(GTK_MENU_SHELL(menu_machine), it_a);
  gtk_menu_shell_append(GTK_MENU_SHELL(menu_machine), it_b);
  gtk_menu_shell_append(GTK_MENU_SHELL(menu_machine), gtk_separator_menu_item_new());
  gtk_menu_shell_append(GTK_MENU_SHELL(menu_machine), it_r);
  g_signal_connect(it_a, "activate", G_CALLBACK(on_mount_floppy), GINT_TO_POINTER(0));
  g_signal_connect(it_b, "activate", G_CALLBACK(on_mount_floppy), GINT_TO_POINTER(1));
  g_signal_connect(it_r, "activate", G_CALLBACK(on_reset_machine), NULL);
  gtk_menu_shell_append(GTK_MENU_SHELL(menubar), mi_machine);

  // ---- 「频率」菜单（单选，默认勾上当前主频）----
  GtkWidget* mi_freq   = gtk_menu_item_new_with_label("频率");
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

  init_font();
  vga_set_default_palette();

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
// 模式切换
// ============================================================
void vga_set_mode(VideoMode mode) {
  vga.mode = mode;
  switch (mode) {
    case MODE_TEXT_80x25: vga.width = 640; vga.height = 400; break;
    case MODE_TEXT_40x25: vga.width = 320; vga.height = 400; break;
    case MODE_GFX_640x480: vga.width = 640; vga.height = 480; break;
    case MODE_GFX_320x200: vga.width = 320; vga.height = 200; break;
    // CGA 图形：内部按 640x400 呈现（横向/纵向各放大一倍，保持 4:3 比例）
    case MODE_CGA_320x200: vga.width = 640; vga.height = 400; break;
    case MODE_CGA_640x200: vga.width = 640; vga.height = 400; break;
  }

  if (vga.surface) cairo_surface_destroy(vga.surface);
  if (vga.pixels) free(vga.pixels);

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

  vga.cursor_x = 0;
  vga.cursor_y = 0;
  text_shadow_valid = false;

  // 清屏
  if (mode == MODE_TEXT_80x25 || mode == MODE_TEXT_40x25) {
    int cols = (mode == MODE_TEXT_80x25) ? 80 : 40;
    for (int i = 0; i < cols * 25; i++) {
      memory[CGA_TEXT_ADDR + i * 2] = ' ';
      memory[CGA_TEXT_ADDR + i * 2 + 1] = 0x07;
    }
  } else if (mode == MODE_CGA_320x200 || mode == MODE_CGA_640x200) {
    memset(&memory[CGA_TEXT_ADDR], 0, CGA_GRAPHICS_SIZE);
  } else if (mode == MODE_GFX_320x200) {
    memset(&memory[VGA_GFX_ADDR], 0, 320 * 200);
  } else if (mode == MODE_GFX_640x480) {
    memset(&memory[VGA_GFX_ADDR], 0, 640 * 480 / 2);
  }

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

  if (want != vga.mode) vga_set_mode(want);
}

void vga_write_cga_color(uint8_t val) {
  cga_color_reg = val;
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

// 由一行"直接色索引"算出第 p 个输出像素的复合色（0xRRGGBB）。
//   spp = 每个像素占几个 14.318MHz 采样（640 宽模式 = 1，320 宽模式 = 2）
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
  int y  = (v[0] + v[1] + v[2] + v[3] + 2) >> 2;   // 亮度 = 四采样均值（色度在一个周期内互相抵消）
  int ii = (v[0] * ct[ph[0]] + v[1] * ct[ph[1]] + v[2] * ct[ph[2]] + v[3] * ct[ph[3]]) / 2;
  int qq = (v[0] * st[ph[0]] + v[1] * st[ph[1]] + v[2] * st[ph[2]] + v[3] * st[ph[3]]) / 2;
  int r = y + ( 956 * ii + 621 * qq) / 2000;
  int g = y + (-272 * ii - 647 * qq) / 2000;
  int b = y + (-1106 * ii + 1703 * qq) / 2000;
  if (r < 0) r = 0; else if (r > 255) r = 255;
  if (g < 0) g = 0; else if (g > 255) g = 255;
  if (b < 0) b = 0; else if (b > 255) b = 255;
  return ((uint32_t)r << 16) | ((uint32_t)g << 8) | (uint32_t)b;
}

// ============================================================
// 渲染
// ============================================================
bool vga_render(void) {
  if (!vga.pixels || !vga.surface) return false;

  // ---------- 文本模式 ----------
  if (vga.mode == MODE_TEXT_80x25 || vga.mode == MODE_TEXT_40x25) {
    int cols = (vga.mode == MODE_TEXT_80x25) ? 80 : 40;

    // 显存和光标闪烁状态都没变 -> 直接跳过，避免每帧重画 640x400
    int blink = (int)((g_get_monotonic_time() / 500000) % 2);
    if (text_shadow_valid && blink == text_last_blink &&
        memcmp(text_shadow, &memory[CGA_TEXT_ADDR], TEXT_SHADOW_SIZE) == 0) {
      return false;
    }
    memcpy(text_shadow, &memory[CGA_TEXT_ADDR], TEXT_SHADOW_SIZE);
    text_shadow_valid = true;
    text_last_blink = blink;

    // 黑底
    for (int i = 0; i < vga.width * vga.height; i++) {
      vga.pixels[i] = 0x000000;
    }

    if (cga_composite) {
      // 复合伪色：按 14.318MHz 采样解调。80 列 = 每像素 1 采样，40 列 = 2 采样。
      comp_prepare();
      int w = cols * 8;
      int spp = (cols == 80) ? 1 : 2;
      static uint8_t  idx[640];
      static uint32_t line[640];
      for (int row = 0; row < 25; row++) {
        for (int y = 0; y < 16; y++) {
          for (int col = 0; col < cols; col++) {
            int offset = (row * 80 + col) * 2;
            uint8_t ch = memory[CGA_TEXT_ADDR + offset];
            uint8_t attr = memory[CGA_TEXT_ADDR + offset + 1];
            uint8_t fg = attr & 0x0F;
            uint8_t bg = (attr >> 4) & 0x07;
            uint8_t bits = font8x16[ch][y];
            for (int x = 0; x < 8; x++)
              idx[col * 8 + x] = (bits & (0x80 >> x)) ? fg : bg;
          }
          for (int x = 0; x < w; x++) line[x] = comp_px(idx, w, spp, x);
          // 8x16 字模：一条扫描线对一条屏幕扫描线
          int dy = row * 16 + y;
          memcpy(&vga.pixels[dy * vga.width], line, (size_t)w * 4);
        }
      }
    } else {
      for (int row = 0; row < 25; row++) {
        for (int col = 0; col < cols; col++) {
          int offset = (row * 80 + col) * 2;
          uint8_t ch = memory[CGA_TEXT_ADDR + offset];
          uint8_t attr = memory[CGA_TEXT_ADDR + offset + 1];
          uint8_t fg = attr & 0x0F;
          uint8_t bg = (attr >> 4) & 0x07;

          RGBColor fgc = vga_palette[fg];
          RGBColor bgc = vga_palette[bg];

          uint32_t fg32 = (fgc.r << 16) | (fgc.g << 8) | fgc.b;
          uint32_t bg32 = (bgc.r << 16) | (bgc.g << 8) | bgc.b;

          uint8_t* font = font8x16[ch];
          int px = col * 8;
          int py = row * 16;

          for (int y = 0; y < 16; y++) {
            uint8_t bits = font[y];
            for (int x = 0; x < 8; x++) {
              uint32_t c = (bits & (0x80 >> x)) ? fg32 : bg32;
              vga.pixels[(py + y) * vga.width + px + x] = c;
            }
          }
        }
      }
    }

    // ★ 闪烁光标（500ms 周期）：位置/形状取自 BIOS 写进 BDA 的值
    //   0x450 + page*2 = 光标位置 word，BIOS 用 MOV [0050],DX 写入，
    //   即低字节 = 列(DL)、高字节 = 行(DH)；0x462 = 当前显示页
    //   0x460 = 光标形状（仅用 bit5 判断是否关闭）
    //   形状固定：8x16 字符单元最下方两条扫描线（下划线）
    int page = memory[0x462] & 0x07;
    int cx = memory[0x450 + page * 2];       // 低字节 = 列
    int cy = memory[0x450 + page * 2 + 1];   // 高字节 = 行
    if (!(memory[0x460] & 0x20) && cx < cols && cy < 25) {
      gint64 now = g_get_monotonic_time();
      bool blink_on = ((now / 500000) % 2) == 0;
      if (blink_on) {
        int px = cx * 8, py = cy * 16;
        for (int y = 14; y <= 15; y++) {
          for (int x = 0; x < 8; x++) {
            int idx = (py + y) * vga.width + (px + x);
            if (idx >= 0 && idx < vga.width * vga.height) {
              vga.pixels[idx] = 0xAAAAAA;
            }
          }
        }
      }
    }
  }
  // ---------- 图形模式 320x200x256 ----------
  else if (vga.mode == MODE_GFX_320x200) {
    for (int y = 0; y < 200; y++) {
      for (int x = 0; x < 320; x++) {
        uint8_t pixel = memory[VGA_GFX_ADDR + y * 320 + x];
        uint8_t r6 = vga_palette_256[pixel][0];
        uint8_t g6 = vga_palette_256[pixel][1];
        uint8_t b6 = vga_palette_256[pixel][2];
        uint8_t r = (r6 << 2) | (r6 >> 4);
        uint8_t g = (g6 << 2) | (g6 >> 4);
        uint8_t b = (b6 << 2) | (b6 >> 4);
        vga.pixels[y * vga.width + x] = (r << 16) | (g << 8) | b;
      }
    }
  }
  // ---------- 图形模式 640x480x16 ----------
  else if (vga.mode == MODE_GFX_640x480) {
    for (int y = 0; y < 480; y++) {
      for (int x = 0; x < 640; x += 2) {
        uint8_t byte = memory[VGA_GFX_ADDR + y * 320 + x / 2];
        uint8_t hi = (byte >> 4) & 0x0F;
        uint8_t lo = byte & 0x0F;
        RGBColor c1 = vga_palette[hi];
        RGBColor c2 = vga_palette[lo];
        vga.pixels[y * vga.width + x] =
          (c1.r << 16) | (c1.g << 8) | c1.b;
        vga.pixels[y * vga.width + x + 1] =
          (c2.r << 16) | (c2.g << 8) | c2.b;
      }
    }
  }

  // ---------- CGA 320x200 四色（模式 4/5）：2bpp，双 bank 交错，2x2 放大 ----------
  else if (vga.mode == MODE_CGA_320x200) {
    RGBColor bgc = vga_palette[cga_color_reg & 0x0F];
    uint32_t bg = (bgc.r << 16) | (bgc.g << 8) | bgc.b;
    uint32_t pal[4];
    if (cga_bw) {
      // 黑白模式（0x3D8 bit2）：0=背景/黑，1=白
      pal[0] = bg;
      pal[1] = pal[2] = pal[3] = 0xFFFFFF;
    } else {
      cga_get_palette4(pal, bg);
    }

    for (int y = 0; y < 200; y++) {
      int base = CGA_TEXT_ADDR + ((y & 1) ? 0x2000 : 0) + (y >> 1) * 80;
      int dy = y * 2;
      if (cga_composite) {
        // 复合伪色：320 宽模式下 1 个像素 = 2 个 14.318MHz 采样
        comp_prepare();
        static uint8_t  idx[320];
        static uint32_t line[320];
        uint8_t pidx[4];
        pidx[0] = cga_color_reg & 0x0F;
        if (cga_bw) {
          pidx[1] = pidx[2] = pidx[3] = 15;
        } else {
          static const uint8_t p0[3] = { 2, 4, 6 };
          static const uint8_t p1[3] = { 3, 5, 7 };
          for (int i = 0; i < 3; i++) {
            uint8_t v = (cga_color_reg & 0x20) ? p1[i] : p0[i];
            if (cga_color_reg & 0x10) v |= 0x08;
            pidx[i + 1] = v;
          }
        }
        for (int x = 0; x < 320; x++) {
          uint8_t b = memory[base + (x >> 2)];
          idx[x] = pidx[(b >> ((3 - (x & 3)) * 2)) & 3];
        }
        for (int x = 0; x < 320; x++) line[x] = comp_px(idx, 320, 2, x);
        for (int x = 0; x < 320; x++) {
          uint32_t c = line[x];
          int dx = x * 2;
          vga.pixels[dy * 640 + dx]           = c;
          vga.pixels[dy * 640 + dx + 1]       = c;
          vga.pixels[(dy + 1) * 640 + dx]     = c;
          vga.pixels[(dy + 1) * 640 + dx + 1] = c;
        }
        continue;
      }
      for (int x = 0; x < 320; x++) {
        uint8_t b = memory[base + (x >> 2)];
        uint32_t c = pal[(b >> ((3 - (x & 3)) * 2)) & 3];
        int dx = x * 2;
        vga.pixels[dy * 640 + dx]         = c;
        vga.pixels[dy * 640 + dx + 1]     = c;
        vga.pixels[(dy + 1) * 640 + dx]   = c;
        vga.pixels[(dy + 1) * 640 + dx + 1] = c;
      }
    }
  }
  // ---------- CGA 640x200 双色（模式 6）：1bpp，双 bank 交错，纵向 2 倍 ----------
  else if (vga.mode == MODE_CGA_640x200) {
    uint32_t off = 0x000000;
    uint32_t on  = 0xFFFFFF;

    if (cga_composite) {
      // 复合伪色：640 宽模式下 1 个像素 = 1 个 14.318MHz 采样
      comp_prepare();
      static uint8_t  idx[640];
      static uint32_t line[640];
      for (int y = 0; y < 200; y++) {
        int base = CGA_TEXT_ADDR + ((y & 1) ? 0x2000 : 0) + (y >> 1) * 80;
        for (int x = 0; x < 640; x++) {
          uint8_t b = memory[base + (x >> 3)];
          idx[x] = (b & (0x80 >> (x & 7))) ? 15 : 0;
        }
        for (int x = 0; x < 640; x++) line[x] = comp_px(idx, 640, 1, x);
        int dy = y * 2;
        memcpy(&vga.pixels[dy * 640], line, 640 * 4);
        memcpy(&vga.pixels[(dy + 1) * 640], line, 640 * 4);
      }
      cairo_surface_mark_dirty(vga.surface);
      return true;
    }

    for (int y = 0; y < 200; y++) {
      int base = CGA_TEXT_ADDR + ((y & 1) ? 0x2000 : 0) + (y >> 1) * 80;
      int dy = y * 2;
      for (int x = 0; x < 640; x++) {
        uint8_t b = memory[base + (x >> 3)];
        uint32_t c = (b & (0x80 >> (x & 7))) ? on : off;
        vga.pixels[dy * 640 + x]       = c;
        vga.pixels[(dy + 1) * 640 + x] = c;
      }
    }
  }

  cairo_surface_mark_dirty(vga.surface);
  return true;
}

// ============================================================
// 辅助
// ============================================================
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
//   这里直接内嵌真 IBM VGA BIOS（见 vga_rom_ibm.h），不依赖外部文件、
//   也不需要在启动时做任何选择：开机就是一台装好 IBM VGA 的机器。
// ============================================================
#define VIDEO_ROM_SEG   0xC000
#define VIDEO_ROM_ADDR  ((uint32_t)VIDEO_ROM_SEG << 4)
#define VIDEO_ROM_SIZE  0x8000        // C0000-C7FFF：真卡 ROM 器件 32KB

void vga_video_rom_install(void) {
  uint8_t* rom = &memory[VIDEO_ROM_ADDR];
  // 前 0x2000 全 0（与原始 ROM 文件一致），24KB 选件 ROM 落在 C2000-C7FFF
  memset(rom, 0x00, VIDEO_ROM_SIZE);
  memcpy(rom + VGA_ROM_IBM_OFFSET, vga_rom_ibm, VGA_ROM_IBM_SIZE);
}

