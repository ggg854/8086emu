// dos86.c - GTK 版主程序（BIOS 引导）
#include "dos86.h"
#include "cpu.h"
#include "vga.h"
#include "ide.h"
#include "io.h"
#include "dma.h"
#include "fdc.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <gtk/gtk.h>

volatile bool emu_running = true;
bool debug_mode = false;
bool console_mode = false;   // -console：无窗口终端模式（见 dos86.h）
uint64_t insn_count = 0;   // 非 static：fdc.c 诊断用
static uint32_t last_sec = 0;
static uint64_t last_cyc = 0;

// ★ 客机主频：20MHz（Turbo XT）。宏定义见 cpu.h（CPU_CLK_HZ）。
//   "1 毫秒走 20000 个周期"，周期数由 cpu.c 的 8088 周期表给出。
//   PIT（CPU_CLK_HZ/PIT_CLK_HZ 分频）与 0x3DA 视频时序都按各自的板级时钟换算，
//   所以放大主频不会让定时器/显示时序跟着跑快。
#define CPU_CYC_PER_MS (CPU_CLK_HZ / 1000)

// ============================================================
// GTK 键 → 8086 扫描码
// ============================================================
static uint8_t gdk_to_scancode(guint keyval) {
    if (keyval >= 'a' && keyval <= 'z') {
        static const uint8_t sc[] = {
            0x1E,0x30,0x2E,0x20,0x12,0x21,0x22,0x23,0x17,0x24,0x25,0x26,0x32,
            0x31,0x18,0x19,0x10,0x13,0x1F,0x14,0x16,0x2F,0x11,0x2D,0x15,0x2C
        };
        return sc[keyval - 'a'];
    }
    if (keyval >= 'A' && keyval <= 'Z') {
        static const uint8_t sc[] = {
            0x1E,0x30,0x2E,0x20,0x12,0x21,0x22,0x23,0x17,0x24,0x25,0x26,0x32,
            0x31,0x18,0x19,0x10,0x13,0x1F,0x14,0x16,0x2F,0x11,0x2D,0x15,0x2C
        };
        return sc[keyval - 'A'];
    }
    if (keyval >= '0' && keyval <= '9') {
        static const uint8_t sc[] = {0x0B,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09,0x0A};
        return sc[keyval - '0'];
    }
    switch (keyval) {
        case GDK_KEY_Escape: return 0x01;
        case GDK_KEY_Return: case GDK_KEY_KP_Enter: return 0x1C;
        case GDK_KEY_BackSpace: return 0x0E;
        case GDK_KEY_Tab: return 0x0F;
        case GDK_KEY_space: return 0x39;
        case GDK_KEY_Shift_L: return 0x2A;
        case GDK_KEY_Shift_R: return 0x36;
        case GDK_KEY_Control_L: case GDK_KEY_Control_R: return 0x1D;
        case GDK_KEY_Alt_L: case GDK_KEY_Alt_R: return 0x38;
        case GDK_KEY_Up: return 0x48;
        case GDK_KEY_Down: return 0x50;
        case GDK_KEY_Left: return 0x4B;
        case GDK_KEY_Right: return 0x4D;
        case GDK_KEY_F1: return 0x3B;
        case GDK_KEY_F2: return 0x3C;
        case GDK_KEY_F3: return 0x3D;
        case GDK_KEY_F4: return 0x3E;
        case GDK_KEY_F5: return 0x3F;
        case GDK_KEY_F6: return 0x40;
        case GDK_KEY_F7: return 0x41;
        case GDK_KEY_F8: return 0x42;
        case GDK_KEY_F9: return 0x43;
        case GDK_KEY_F10: return 0x44;
        case GDK_KEY_minus: return 0x0C;
        case GDK_KEY_equal: return 0x0D;
        case GDK_KEY_bracketleft: return 0x1A;
        case GDK_KEY_bracketright: return 0x1B;
        case GDK_KEY_semicolon: return 0x27;
        case GDK_KEY_apostrophe: return 0x28;
        case GDK_KEY_grave: return 0x29;
        case GDK_KEY_comma: return 0x33;
        case GDK_KEY_period: return 0x34;
        case GDK_KEY_slash: return 0x35;
        case GDK_KEY_backslash: return 0x2B;

        // ---- 数字行上档符号：扫描码与下档数字键相同，字符由客机按 Shift 自行生成 ----
        //      （原表缺这些，按 Shift 打符号/数字时会被静默丢弃）
        case GDK_KEY_exclam: return 0x02;       // !  ← 1
        case GDK_KEY_at: return 0x03;           // @  ← 2
        case GDK_KEY_numbersign: return 0x04;   // #  ← 3
        case GDK_KEY_dollar: return 0x05;       // $  ← 4
        case GDK_KEY_percent: return 0x06;      // %  ← 5
        case GDK_KEY_asciicircum: return 0x07;  // ^  ← 6
        case GDK_KEY_ampersand: return 0x08;    // &  ← 7
        case GDK_KEY_asterisk: return 0x09;     // *  ← 8
        case GDK_KEY_parenleft: return 0x0A;    // (  ← 9
        case GDK_KEY_parenright: return 0x0B;   // )  ← 0
        case GDK_KEY_underscore: return 0x0C;   // _  ← -
        case GDK_KEY_plus: return 0x0D;         // +  ← =
        case GDK_KEY_braceleft: return 0x1A;    // {
        case GDK_KEY_braceright: return 0x1B;   // }
        case GDK_KEY_bar: return 0x2B;          // |
        case GDK_KEY_colon: return 0x27;        // :
        case GDK_KEY_quotedbl: return 0x28;     // "
        case GDK_KEY_asciitilde: return 0x29;   // ~
        case GDK_KEY_less: return 0x33;         // <
        case GDK_KEY_greater: return 0x34;      // >
        case GDK_KEY_question: return 0x35;     // ?

        // ---- 小键盘 ----
        case GDK_KEY_KP_0: return 0x52;
        case GDK_KEY_KP_1: return 0x4F;
        case GDK_KEY_KP_2: return 0x50;
        case GDK_KEY_KP_3: return 0x51;
        case GDK_KEY_KP_4: return 0x4B;
        case GDK_KEY_KP_5: return 0x4C;
        case GDK_KEY_KP_6: return 0x4D;
        case GDK_KEY_KP_7: return 0x47;
        case GDK_KEY_KP_8: return 0x48;
        case GDK_KEY_KP_9: return 0x49;
        case GDK_KEY_KP_Decimal: return 0x53;
        case GDK_KEY_KP_Add: return 0x4E;
        case GDK_KEY_KP_Subtract: return 0x4A;
        case GDK_KEY_KP_Multiply: return 0x37;
        case GDK_KEY_KP_Divide: return 0x35;

        // ---- 编辑键 / 锁定键 ----
        case GDK_KEY_Insert: return 0x52;
        case GDK_KEY_Delete: return 0x53;
        case GDK_KEY_Home: return 0x47;
        case GDK_KEY_End: return 0x4F;
        case GDK_KEY_Page_Up: return 0x49;
        case GDK_KEY_Page_Down: return 0x51;
        case GDK_KEY_Caps_Lock: return 0x3A;
        case GDK_KEY_Num_Lock: return 0x45;
        case GDK_KEY_Scroll_Lock: return 0x46;
    }
    return 0;
}

// ============================================================
// 鼠标捕获：点一下 CGA 屏幕就把指针"抓"进来（隐藏指针、只送相对位移），
// 按 Ctrl+Alt+← 释放。抓取期间光标被钉在画布中心，位移相对中心计算。
// ============================================================
static bool       mouse_grabbed = false;
static GdkCursor* mouse_blank_cursor = NULL;
static int        mouse_last_x = -1, mouse_last_y = -1;

static void mouse_capture(bool on) {
  GtkWidget* w = vga.drawing_area;
  GdkWindow* win = gtk_widget_get_window(w);
  if (!win) return;
  GdkScreen* scr = gtk_widget_get_screen(w);
  GdkSeat* seat = gdk_display_get_default_seat(gdk_screen_get_display(scr));

  if (on) {
    if (mouse_grabbed) return;
    if (!mouse_blank_cursor)
      mouse_blank_cursor = gdk_cursor_new_for_display(gdk_screen_get_display(scr),
                                                      GDK_BLANK_CURSOR);
    if (gdk_seat_grab(seat, win, GDK_SEAT_CAPABILITY_ALL_POINTING, TRUE,
                      mouse_blank_cursor, NULL, NULL, NULL) != GDK_GRAB_SUCCESS)
      return;
    mouse_grabbed = true;
    // 指针挪到画布正中：之后的位移都相对中心算
    GtkAllocation a;
    gtk_widget_get_allocation(w, &a);
    int rx, ry;
    gdk_window_get_root_coords(win, a.width / 2, a.height / 2, &rx, &ry);
    gdk_device_warp(gdk_seat_get_pointer(seat), scr, rx, ry);
    mouse_last_x = mouse_last_y = -1;
  } else {
    if (!mouse_grabbed) return;
    gdk_seat_ungrab(seat);
    mouse_grabbed = false;
    mouse_last_x = mouse_last_y = -1;
    io_mouse_button(1, false);            // 释放时别把键卡在按下状态
    io_mouse_button(2, false);
    io_mouse_button(3, false);
  }
}

gboolean vga_on_key_press(GtkWidget* widget, GdkEventKey* event, gpointer data) {
  (void)widget; (void)data;
  // Ctrl+Alt+←：释放鼠标捕获（只在已捕获时拦截，否则照常送给客机）
  if (mouse_grabbed && (event->state & GDK_CONTROL_MASK) && (event->state & GDK_MOD1_MASK) &&
      (event->keyval == GDK_KEY_Left || event->keyval == GDK_KEY_KP_Left)) {
    mouse_capture(false);
    return TRUE;
  }
  uint8_t sc = gdk_to_scancode(event->keyval);
  // 调试：手动按键是否到达/是否映射成功（sc=00 表示这个键被丢掉了）
  if (debug_mode) {
    printf("[KEY] keyval=%u (%s) hw=%u → sc=%02X\n",
           event->keyval, gdk_keyval_name(event->keyval) ? gdk_keyval_name(event->keyval) : "?",
           event->hardware_keycode, sc);
  }
  if (sc) io_keyboard_push(sc);
  if (event->keyval == GDK_KEY_F12) emu_running = false;
  return TRUE;
}

gboolean vga_on_key_release(GtkWidget* widget, GdkEventKey* event, gpointer data) {
    (void)widget; (void)data;
    uint8_t sc = gdk_to_scancode(event->keyval);
    if (sc) io_keyboard_push_release(sc);
    return TRUE;
}

// ---- 鼠标：GTK 指针事件 → 相对位移 / 按键 ----
gboolean vga_on_motion(GtkWidget* widget, GdkEventMotion* event, gpointer data) {
    (void)data;
    if (mouse_grabbed) {
        // 捕获中：指针被钉在画布中心，位移 = 当前坐标 − 中心，再把它挪回中心
        GtkAllocation a;
        gtk_widget_get_allocation(widget, &a);
        int cx = a.width / 2, cy = a.height / 2;
        int dx = (int)event->x - cx;
        int dy = (int)event->y - cy;
        if (dx || dy) {
            io_mouse_motion(dx, dy);
            GdkWindow* win = gtk_widget_get_window(widget);
            GdkScreen* scr = gtk_widget_get_screen(widget);
            GdkSeat* seat = gdk_display_get_default_seat(gdk_screen_get_display(scr));
            int rx, ry;
            gdk_window_get_root_coords(win, cx, cy, &rx, &ry);
            gdk_device_warp(gdk_seat_get_pointer(seat), scr, rx, ry);
        }
        return TRUE;
    }
    if (mouse_last_x >= 0) {
        int dx = (int)event->x - mouse_last_x;
        int dy = (int)event->y - mouse_last_y;
        if (dx || dy) io_mouse_motion(dx, dy);
    }
    mouse_last_x = (int)event->x;
    mouse_last_y = (int)event->y;
    return TRUE;
}

gboolean vga_on_button_press(GtkWidget* widget, GdkEventButton* event, gpointer data) {
    (void)widget; (void)data;
    if (event->button == 1 && !mouse_grabbed) {
        mouse_capture(true);        // 没捕获时，第一次左键只用于捕获，不传给客机
        return TRUE;
    }
    io_mouse_button((int)event->button, true);
    return TRUE;
}

gboolean vga_on_button_release(GtkWidget* widget, GdkEventButton* event, gpointer data) {
    (void)widget; (void)data;
    io_mouse_button((int)event->button, false);
    return TRUE;
}

static gboolean on_delete_event(GtkWidget* widget, GdkEvent* event, gpointer data) {
    (void)widget; (void)event; (void)data;
    emu_running = false;
    cpu_running = false;
    ide_cleanup();
    vga_cleanup();
    exit(0);
    return FALSE;
}

static void on_window_destroy(GtkWidget* widget, gpointer data) {
    (void)widget; (void)data;
}

// ============================================================
// -console：无窗口终端模式（类似 qemu -nographic）
//   · 输出：把 80x25 文本显存用 ANSI 重画到 stdout（内容变了才重画）
//   · 输入：后台线程阻塞读 stdin，把 ASCII 逐字翻成 8086 扫描码喂给键盘设备
//   · 退出：Ctrl+C / Ctrl+Z
// ============================================================
#define CON_BUF 1024
static uint8_t con_pend[CON_BUF];        // 主机已读入、还没喂给客机的字节
static int     con_head = 0, con_tail = 0;
static char    con_shadow[25][81];       // 上次画出去的屏幕内容

static void con_push_byte(uint8_t c) {
    int next = (con_tail + 1) % CON_BUF;
    if (next == con_head) return;        // 队列满就丢（正常不会发生）
    con_pend[con_tail] = c;
    con_tail = next;
}

static gpointer con_reader(gpointer arg) {
    (void)arg;
    int c;
    while ((c = getchar()) != EOF) {
        if (c == 0x03 || c == 0x1A) break;   // Ctrl+C / Ctrl+Z
        con_push_byte((uint8_t)c);
    }
    emu_running = false;   // Ctrl+C 或 stdin 读完（EOF）都退出，否则主循环会一直空转、不落盘
    return NULL;
}

// ASCII → 8086 扫描码；shift=true 表示这个字符在真键盘上要按住 Shift 才出得来
static bool con_map(uint8_t c, uint8_t* sc, bool* shift) {
    *shift = false;
    static const uint8_t lower[26] = {
        0x1E,0x30,0x2E,0x20,0x12,0x21,0x22,0x23,0x17,0x24,0x25,0x26,0x32,
        0x31,0x18,0x19,0x10,0x13,0x1F,0x14,0x16,0x2F,0x11,0x2D,0x15,0x2C
    };
    if (c >= 'a' && c <= 'z') { *sc = lower[c - 'a']; return true; }
    if (c >= 'A' && c <= 'Z') { *sc = lower[c - 'A']; *shift = true; return true; }
    if (c >= '0' && c <= '9') {
        *sc = (c == '0') ? 0x0B : (uint8_t)(0x02 + (c - '1'));
        return true;
    }
    switch (c) {
        case 0x0C: *sc = 0x3B; return true;   // TEMP: Ctrl-L(0x0C) → F1，用于自动复现 162 后按 F1
        case ' ':  *sc = 0x39; return true;
        case '-':  *sc = 0x0C; return true;
        case '=':  *sc = 0x0D; return true;
        case '[':  *sc = 0x1A; return true;
        case ']':  *sc = 0x1B; return true;
        case ';':  *sc = 0x27; return true;
        case '\'': *sc = 0x28; return true;
        case '`':  *sc = 0x29; return true;
        case ',':  *sc = 0x33; return true;
        case '.':  *sc = 0x34; return true;
        case '/':  *sc = 0x35; return true;
        case '\\': *sc = 0x2B; return true;
        // ---- 上档符号 ----
        case '!':  *sc = 0x02; *shift = true; return true;
        case '@':  *sc = 0x03; *shift = true; return true;
        case '#':  *sc = 0x04; *shift = true; return true;
        case '$':  *sc = 0x05; *shift = true; return true;
        case '%':  *sc = 0x06; *shift = true; return true;
        case '^':  *sc = 0x07; *shift = true; return true;
        case '&':  *sc = 0x08; *shift = true; return true;
        case '*':  *sc = 0x09; *shift = true; return true;
        case '(':  *sc = 0x0A; *shift = true; return true;
        case ')':  *sc = 0x0B; *shift = true; return true;
        case '_':  *sc = 0x0C; *shift = true; return true;
        case '+':  *sc = 0x0D; *shift = true; return true;
        case '{':  *sc = 0x1A; *shift = true; return true;
        case '}':  *sc = 0x1B; *shift = true; return true;
        case ':':  *sc = 0x27; *shift = true; return true;
        case '"':  *sc = 0x28; *shift = true; return true;
        case '~':  *sc = 0x29; *shift = true; return true;
        case '<':  *sc = 0x33; *shift = true; return true;
        case '>':  *sc = 0x34; *shift = true; return true;
        case '?':  *sc = 0x35; *shift = true; return true;
        case '|':  *sc = 0x2B; *shift = true; return true;
    }
    return false;
}

// 把一个字节变成键盘按键（含按下/松开）
static void con_feed_byte(uint8_t c) {
    uint8_t sc = 0;
    bool shift = false;
    if (c == '\r' || c == '\n')       sc = 0x1C;
    else if (c == '\b' || c == 0x7F)  sc = 0x0E;
    else if (c == '\t')               sc = 0x0F;
    else if (c == 0x1B)               sc = 0x01;   // ESC
    else if (!con_map(c, &sc, &shift)) return;

    if (shift) {
        io_keyboard_push(0x2A);           // 左 Shift 按下
        io_keyboard_push(sc);
        io_keyboard_push_release(sc);
        io_keyboard_push_release(0x2A);   // 左 Shift 松开
    } else {
        io_keyboard_push(sc);
        io_keyboard_push_release(sc);
    }
}

// 逐字喂键：等客机把上一个字节读走（0x60 空）再喂下一个，避免设备队列里堆字丢键
static void con_poll_input(void) {
    if (con_head == con_tail) return;
    if (!io_keyboard_idle()) return;
    uint8_t c = con_pend[con_head];
    con_head = (con_head + 1) % CON_BUF;
    con_feed_byte(c);
}

// 把 80x25 文本屏重画到终端
static void con_render(void) {
    char cur[25][81];
    for (int y = 0; y < 25; y++) {
        for (int x = 0; x < 80; x++) {
            uint8_t ch = memory[0xB8000 + (y * 80 + x) * 2];
            cur[y][x] = (ch >= 0x20 && ch < 0x7F) ? (char)ch : ' ';
        }
        cur[y][80] = 0;
    }
    if (memcmp(cur, con_shadow, sizeof(cur)) == 0) return;

    // 限速重画：终端整屏重画很贵，最多 ~30 次/秒（画面还在变，下次照样会画）
    static gint64 con_last_paint = 0;
    gint64 nowp = g_get_monotonic_time();
    if (nowp - con_last_paint < 33000) return;
    con_last_paint = nowp;

    memcpy(con_shadow, cur, sizeof(cur));

    fputs("\x1b[H", stdout);
    for (int y = 0; y < 25; y++) {
        fputs(cur[y], stdout);
        fputs("\x1b[K", stdout);
        if (y < 24) fputs("\r\n", stdout);
    }
    int cy = vga.cursor_y + 1, cx = vga.cursor_x + 1;
    if (cy < 1) cy = 1;
    if (cy > 25) cy = 25;
    if (cx < 1) cx = 1;
    if (cx > 80) cx = 80;
    printf("\x1b[%d;%dH", cy, cx);
    fflush(stdout);
}

static gboolean tick(gpointer data) {
    (void)data;
    static int halt_reported = 0;

    // 用户点了 ×，真正退出
    if (!emu_running) {
        if (!console_mode) gtk_main_quit();
        return console_mode ? TRUE : FALSE;
    }

    // ★ CPU 停了：继续渲染，不退出
    if (!cpu_running) {
        bool r = vga_render();
        if (!console_mode && (r || vga_led_dirty()))
            gtk_widget_queue_draw(vga.drawing_area);
        return TRUE;
    }

    // ★ 目标 20MHz：按"从锚点起应走掉的周期数 − 实际走掉的周期数"发配额。
    //   主机循环的一帧可能很粗（-console 下 Windows 的 usleep 粒度 + 每帧重绘 ≈ 30ms，
    //   折合 ~600k 周期），所以"单帧上限"必须容得下一整帧应走的量，否则永远追不上：
    //   旧值 12000（≈2.5ms@4.77MHz）实测把整机压到只有 0.38MHz。
    gint64 now_us = g_get_monotonic_time();
    static uint64_t anchor_us = 0, anchor_cyc = 0;
    if (anchor_us == 0) { anchor_us = (uint64_t)now_us; anchor_cyc = cpu_cycles; }

    // 正值 = 欠客机的周期数
    int64_t owed = (int64_t)anchor_cyc
                 + (int64_t)(CPU_CYC_PER_MS * (now_us - (gint64)anchor_us) / 1000)
                 - (int64_t)cpu_cycles;
    if (owed < 0) {                     // 跑超前（时间抖动/中断补账）：重锚，避免空转
        anchor_us = (uint64_t)now_us;
        anchor_cyc = cpu_cycles;
        owed = 0;
    }
    uint64_t cyc_budget = (uint64_t)owed;

    if (cyc_budget > CPU_CLK_HZ) {      // 落后 >1s（真·长停顿/系统挂起）：不追历史
        anchor_us = (uint64_t)now_us;
        anchor_cyc = cpu_cycles;
        cyc_budget = 0;
    }
    if (cyc_budget > CPU_CYC_PER_MS * 50) cyc_budget = CPU_CYC_PER_MS * 50;   // 单帧上限 ≈50ms 的活

    uint64_t cyc_this_frame = 0;

    uint64_t insn_this_frame = 0;

    // 单帧最多占用 25ms 墙钟（20MHz 下 ≈500k 周期）：正常帧的配额都远小于它，
    // 只有长时间停顿后要"补账"时才会碰到；补不上的零头留到后续帧继续追，
    // 这样按键/重绘不会被一次超长补账卡住。
    const gint64 frame_deadline = now_us + 25000;

    static uint16_t last_cs = 0, last_ip = 0;
    static int same_count = 0;

    // ★ HLT（等中断）状态下设备轮询必须与指令预算无关。
    //   以前这段逻辑放在 while 循环体里，但 deficit≤0 时 insn_budget=0、循环体一次都不进，
    //   结果：键盘/FDC/定时器中断永远送不进去，HALT 计数也永远不涨
    //   （表现就是用户看到的 "cpu_halted=1 却显示 HALT=0"）。
    if (cpu_halted) {
        if (!halt_reported) {
            if (debug_mode)
                printf("\n[HALT] CPU 执行 HLT，等待中断：CS=%04X IP=%04X FL=%04X (IF=%d) HLT累计=%llu\n",
                       cpu.cs, cpu.ip, cpu.flags, (cpu.flags >> 9) & 1,
                       (unsigned long long)hlt_count);
            halt_reported = 1;
        }
        io_keyboard_poll();
        io_fdc_poll();
        io_ide_poll();
        io_serial_poll();    // ★ 串口鼠标：停机期间也要组包/投 IRQ4，否则鼠标永远不动
        io_timer_poll();
        if (cpu_halted) {
            // ★ 停机期间 CPU 不执行指令，但硬件时钟照走：把这段虚拟时间补给周期计数器，
            //   否则 PIT 永远到不了点、IRQ0 送不进去 → 永远醒不过来。
            uint64_t adv = cyc_budget;   // 时间是照走的：把这帧该走的周期全补上
            cpu_cycles += adv;
            cpu_last_cycles = (uint32_t)adv;
            if (adv) io_pit_step((uint32_t)adv);
            cyc_budget = 0;                   // 仍无中断：本帧不跑指令，但渲染/标题照常
            anchor_us = (uint64_t)now_us;     // 重置锚点，唤醒后不暴补
            anchor_cyc = cpu_cycles;
        } else {
            if (!console_mode)
                printf("[HALT] 被中断唤醒，继续执行 CS=%04X IP=%04X\n", cpu.cs, cpu.ip);
            halt_reported = 0;
        }
    }

    while (cyc_this_frame < cyc_budget &&
           emu_running && cpu_running) {

        if ((insn_this_frame & 0x3FF) == 0 &&
            g_get_monotonic_time() >= frame_deadline) {
            break;
        }

        // 客机在循环中途执行了 HLT：本帧收尾，下一帧由 tick 开头的 HLT 处理接管
        if (cpu_halted) break;

        // ★ 复位请求（8042 输出端口的 0xFE 脉冲复位）：只在指令边界上执行。
        //   若在一条 OUT 指令的中途立刻复位，调用方随后还会推进 IP，
        //   复位后就不会从 0xFFFF:0000 取复位向量了。
        if (cpu_reset_pending) {
            cpu_reset_pending = false;
            cpu_reset();
            continue;
        }

        // ★ 这里的取指地址必须和 CPU 内部一致：保护模式下 cpu.cs 是选择子而非段值，
        //   用 (cs<<4)+ip 会落到错误的物理地址上（PM POST 期间 cs=0040，实测会把
        //   无关代码字节误读成 0xE6/0xEE，从而错误地走进下面的 OUT 周期特判）。
        uint32_t addr = cpu_seg_base(cpu.cs) + cpu.ip;
        uint8_t op = cpu_mem_read(addr);

        // ★ 检测 1：跑到 IVT 区（只警告 + 停 CPU，不退出）
        if (cpu.cs == 0x0000 && cpu.ip < 0x0400) {
            fprintf(stderr, "\n[CPU] 跑飞：CS=%04X IP=%04X 落在 IVT 区\n",
                    cpu.cs, cpu.ip);
            fprintf(stderr, "  AX=%04X BX=%04X CX=%04X DX=%04X\n",
                    cpu.ax, cpu.bx, cpu.cx, cpu.dx);
            fprintf(stderr, "  SI=%04X DI=%04X BP=%04X SP=%04X\n",
                    cpu.si, cpu.di, cpu.bp, cpu.sp);
            fprintf(stderr, "  DS=%04X ES=%04X SS=%04X CS=%04X\n",
                    cpu.ds, cpu.es, cpu.ss, cpu.cs);
            fprintf(stderr, "  FL=%04X\n", cpu.flags);
            cpu_running = false;
            break;   // ★ 跳出 while，不 return
        }

        // ★ 检测 2：CS:IP 不变超过 50 万次（只警告 + 停 CPU，不退出）
        if (cpu.cs == last_cs && cpu.ip == last_ip) {
            same_count++;
            if (same_count > 500000) {
                printf("\n[CPU] 死循环：CS=%04X IP=%04X 超过 50 万次\n",
                       cpu.cs, cpu.ip);
                printf("  字节: %02X %02X %02X %02X %02X\n",
                       cpu_mem_read(addr), cpu_mem_read(addr + 1),
                       cpu_mem_read(addr + 2), cpu_mem_read(addr + 3),
                       cpu_mem_read(addr + 4));
                printf("  AX=%04X BX=%04X CX=%04X DX=%04X\n",
                       cpu.ax, cpu.bx, cpu.cx, cpu.dx);
                printf("  SI=%04X DI=%04X BP=%04X SP=%04X\n",
                       cpu.si, cpu.di, cpu.bp, cpu.sp);
                printf("  DS=%04X ES=%04X SS=%04X FL=%04X\n",
                       cpu.ds, cpu.es, cpu.ss, cpu.flags);
                cpu_running = false;
                break;   // ★ 跳出 while，不 return
            }
        } else {
            last_cs = cpu.cs;
            last_ip = cpu.ip;
            same_count = 0;
        }

        // OUT 周期特判（周期来自 cpu.c 的表；0x3F8 不再特判成控制台输出，
        // 否则会绕过 UART 模型 —— 串口鼠标驱动做回环自检时写 THR 读不回来就认不到鼠标）
        cpu_prefix_step = false;   // 下面三条分支都是完整指令（cpu_execute_instruction 会自行维护本标志）
        if (op == 0xE6 || op == 0xEE) {
            uint16_t port;
            uint8_t val = cpu.ax & 0xFF;
            if (op == 0xE6) { port = cpu_mem_read(addr + 1); cpu.ip += 2; }
            else { port = cpu.dx; cpu.ip += 1; }
            io_write_port(port, val);
            cpu_last_cycles = (op == 0xE6) ? 14 : 12;   // OUT 周期（见 cpu.c 周期表）
            cpu_cycles += cpu_last_cycles;
        } else if (op == 0xE7 || op == 0xEF) {
            uint16_t port;
            uint16_t val = cpu.ax;
            if (op == 0xE7) { port = cpu_mem_read(addr + 1); cpu.ip += 2; }
            else { port = cpu.dx; cpu.ip += 1; }
            io_write_port16(port, val);
            cpu_last_cycles = (op == 0xE7) ? 18 : 16;
            cpu_cycles += cpu_last_cycles;
        } else {
            cpu_execute_instruction();
        }

        // PIT：按本步耗掉的周期数推进（按 CPU_CLK_HZ/PIT_CLK_HZ 换算成 PIT 时钟拍）。通道 0 计数到 0
        // 立刻请求 IRQ0 —— XT BIOS 自检给通道 0 装小值后只等很短时间，
        // 等到才认为定时器正常（否则报 101 停机）。
        io_pit_step(cpu_last_cycles);
        cyc_this_frame += cpu_last_cycles;
        if (!cpu_prefix_step) io_timer_poll();

        // 软盘中断（IRQ6）：BIOS 等中断的超时窗口只有 ~26ms，
        // 每 256 条指令轮询一次，避免等一整帧才投递而错过窗口
        // ★ 若上一步只执行了前缀字节（如 `36` 紧跟着的 CALL FAR），绝不能在这里投中断：
        //   真 8086 中"前缀+操作码"是一条指令，中断只在指令之间被识别。中断打在中间时，
        //   中断处理程序会消费掉 seg_override，回到操作码时前缀已失效 → 读错段
        //   （实测 DOS 因此 CALL FAR 到 6F8B:0000 空白内存，跑飞后表现为"20MHz 无法输入"）。
        if (!cpu_prefix_step && (insn_this_frame & 0xFF) == 0) {
            io_fdc_poll();
            io_ide_poll();
            io_serial_poll();    // ★ 串口鼠标：IRQ4（COM1 收数据）
            // ★ 键盘单独一条投递通道：不再只在每帧末尾投一次（那样最坏要等 16ms，
            //   而且本帧一旦提前 break 就整帧不投）。现在每 256 条指令投一次，
            //   只要 IF=1 就立刻进 IRQ1，手动输入不再"有时候没反应"。
            io_keyboard_poll();
            io_timer_poll();     // ★ 锁存的 IRQ0（BIOS tick）：CLI 期间不丢，IF=1 立刻投
        }

        insn_this_frame++;
        insn_count++;
    }

    // ★ 每帧都渲染 + 触发 GTK 重绘
    if (!console_mode) {
        bool r = vga_render();
        if (r || vga_led_dirty()) gtk_widget_queue_draw(vga.drawing_area);
    }

    // 标题：每秒更新一次。主频直接由"这一步走掉多少 CPU 周期"统计，
    // 目标 20MHz 时这里应稳定显示 19.x/20.0 MHz（旧版按指令数估算，无法反映周期级时序）。
    uint32_t now_ms = g_get_monotonic_time() / 1000;
    if (now_ms - last_sec >= 1000) {
        uint64_t dcyc = cpu_cycles - last_cyc;
        uint32_t mhz10 = (uint32_t)(dcyc / 100000);   // 0.1 MHz 精度
        char title[256];
        const char* st = cpu_halted ? "[HALT 等待中断]"
                       : (cpu_running ? "[运行中]" : "[已停止 死循环]");
        snprintf(title, sizeof(title),
                 "IBM PC Emulator - %u.%u MHz  %s  HALT=%d",
                 mhz10 / 10, mhz10 % 10, st, cpu_halted ? 1 : 0,
                 (unsigned long long)hlt_count);
        if (!console_mode) gtk_window_set_title(GTK_WINDOW(vga.window), title);
        last_sec = now_ms;
        last_cyc = cpu_cycles;
    }

    // 定时器中断：每 ~16ms 发一次 IRQ0（tick ≈60Hz，见 io_timer_tick 的说明）
    static uint32_t last_timer = 0;
    uint32_t now = g_get_monotonic_time() / 1000;
    if (now - last_timer >= 16) {
        last_timer = now;
        io_timer_tick();
    }

    // ★ 同样不能在前缀-操作码缝隙里投 IRQ1（帧预算刚好用完在前缀上时会发生）
    if (!cpu_prefix_step) { io_keyboard_poll(); io_serial_poll(); }
    return TRUE;
}

// ============================================================
// 文件对话框
// ============================================================
char* open_bios_dialog(void) {
    GtkWidget* dialog = gtk_file_chooser_dialog_new(
        "Select BIOS ROM (e.g. pcxtbios.bin)",
        NULL, GTK_FILE_CHOOSER_ACTION_OPEN,
        "_Skip", GTK_RESPONSE_CANCEL,
        "_Open", GTK_RESPONSE_ACCEPT, NULL);

    GtkFileFilter* filter = gtk_file_filter_new();
    gtk_file_filter_set_name(filter, "BIOS ROM (*.bin, *.rom)");
    gtk_file_filter_add_pattern(filter, "*.bin");
    gtk_file_filter_add_pattern(filter, "*.rom");
    gtk_file_filter_add_pattern(filter, "*.BIN");
    gtk_file_filter_add_pattern(filter, "*.ROM");
    gtk_file_chooser_add_filter(GTK_FILE_CHOOSER(dialog), filter);

    GtkFileFilter* all = gtk_file_filter_new();
    gtk_file_filter_set_name(all, "All files");
    gtk_file_filter_add_pattern(all, "*");
    gtk_file_chooser_add_filter(GTK_FILE_CHOOSER(dialog), all);

    static char filename[1024] = {0};
    if (gtk_dialog_run(GTK_DIALOG(dialog)) == GTK_RESPONSE_ACCEPT) {
        char* f = gtk_file_chooser_get_filename(GTK_FILE_CHOOSER(dialog));
        strncpy(filename, f, sizeof(filename) - 1);
        g_free(f);
    }
    gtk_widget_destroy(dialog);
    return filename[0] ? filename : NULL;
}

char* open_floppy_dialog(int drive) {
    char title[64];
    snprintf(title, sizeof(title), "Select Floppy Image for %c: (optional)", 'A' + drive);
    GtkWidget* dialog = gtk_file_chooser_dialog_new(
        title,
        NULL, GTK_FILE_CHOOSER_ACTION_OPEN,
        "_Skip", GTK_RESPONSE_CANCEL,
        "_Open", GTK_RESPONSE_ACCEPT, NULL);

    GtkFileFilter* filter = gtk_file_filter_new();
    gtk_file_filter_set_name(filter, "Floppy image (*.img, *.ima, *.bin)");
    gtk_file_filter_add_pattern(filter, "*.img");
    gtk_file_filter_add_pattern(filter, "*.ima");
    gtk_file_filter_add_pattern(filter, "*.bin");
    gtk_file_filter_add_pattern(filter, "*.IMG");
    gtk_file_filter_add_pattern(filter, "*.IMA");
    gtk_file_chooser_add_filter(GTK_FILE_CHOOSER(dialog), filter);

    GtkFileFilter* all = gtk_file_filter_new();
    gtk_file_filter_set_name(all, "All files");
    gtk_file_filter_add_pattern(all, "*");
    gtk_file_chooser_add_filter(GTK_FILE_CHOOSER(dialog), all);

    static char filename[1024];
    filename[0] = 0;    // 本函数会被 A:/B: 各调一次，必须清掉上一次的结果
    if (gtk_dialog_run(GTK_DIALOG(dialog)) == GTK_RESPONSE_ACCEPT) {
        char* f = gtk_file_chooser_get_filename(GTK_FILE_CHOOSER(dialog));
        strncpy(filename, f, sizeof(filename) - 1);
        g_free(f);
    }
    gtk_widget_destroy(dialog);
    return filename[0] ? filename : NULL;
}

char* open_disk_dialog(void) {
    GtkWidget* dialog = gtk_file_chooser_dialog_new(
        "Select Hard Disk Image (optional, default dos.img)",
        NULL, GTK_FILE_CHOOSER_ACTION_OPEN,
        "_Skip", GTK_RESPONSE_CANCEL,
        "_Open", GTK_RESPONSE_ACCEPT, NULL);

    GtkFileFilter* filter = gtk_file_filter_new();
    gtk_file_filter_set_name(filter, "Disk image (*.img, *.iso, *.bin)");
    gtk_file_filter_add_pattern(filter, "*.img");
    gtk_file_filter_add_pattern(filter, "*.iso");
    gtk_file_filter_add_pattern(filter, "*.bin");
    gtk_file_filter_add_pattern(filter, "*.IMG");
    gtk_file_chooser_add_filter(GTK_FILE_CHOOSER(dialog), filter);

    GtkFileFilter* all = gtk_file_filter_new();
    gtk_file_filter_set_name(all, "All files");
    gtk_file_filter_add_pattern(all, "*");
    gtk_file_chooser_add_filter(GTK_FILE_CHOOSER(dialog), all);

    static char filename[1024] = {0};
    if (gtk_dialog_run(GTK_DIALOG(dialog)) == GTK_RESPONSE_ACCEPT) {
        char* f = gtk_file_chooser_get_filename(GTK_FILE_CHOOSER(dialog));
        strncpy(filename, f, sizeof(filename) - 1);
        g_free(f);
    }
    gtk_widget_destroy(dialog);
    return filename[0] ? filename : NULL;
}

// ============================================================
// 命令行帮助
// ============================================================
static void print_usage(const char* exe) {
    printf(
"DOS86 - 8086/8088 PC 模拟器（GTK 前端 / -console 无窗口）\n"
"\n"
"用法：%s [选项]\n"
"不带任何选项直接运行：启动时依次弹出 GTK 选择框挑 BIOS、硬盘、软盘 A:、软盘 B:（取消则用默认值）。\n"
"\n"
"选项：\n"
"  -help                显示本帮助\n"
"  -bios <file>         BIOS ROM（默认 data/PCXTBIOS.BIN）\n"
"  -floppy <file>       软盘 A: 镜像（.img/.ima/.360）\n"
"  -floppy2 <file>      软盘 B: 镜像\n"
"  -disk <file>         硬盘镜像（默认 dos.img）\n"
"  -composite           CGA 复合视频伪色（NTSC artifact colors）\n"
"  -console             无窗口终端模式\n"
"  -dbg                 打开调试日志\n"
"\n"
"运行中用窗口顶部「机器」菜单换 A:/B: 软盘、重启；用「频率」菜单切换客机主频。\n", exe);
}

// 默认 BIOS：按顺序找 data/PCXTBIOS.BIN
static const char* find_default_bios(void) {
    static const char* cands[] = {
        "data/PCXTBIOS.BIN", "../data/PCXTBIOS.BIN", "../../data/PCXTBIOS.BIN",
        "PCXTBIOS.BIN", "data/pcxtbios.bin", "pcxtbios.bin", NULL
    };
    for (int i = 0; cands[i]; i++) {
        FILE* fp = fopen(cands[i], "rb");
        if (fp) { fclose(fp); return cands[i]; }
    }
    return NULL;
}

// 运行时换盘：弹文件框选镜像，挂到 A:(drive=0) 或 B:(drive=1)
void ui_mount_floppy(int drive) {
    if (console_mode) return;
    GtkWidget* dialog = gtk_file_chooser_dialog_new(
        drive ? "挂载软盘 B:" : "挂载软盘 A:",
        NULL, GTK_FILE_CHOOSER_ACTION_OPEN,
        "_取消", GTK_RESPONSE_CANCEL,
        "_挂载", GTK_RESPONSE_ACCEPT, NULL);

    GtkFileFilter* filter = gtk_file_filter_new();
    gtk_file_filter_set_name(filter, "软盘镜像 (*.img, *.ima, *.360, *.144)");
    gtk_file_filter_add_pattern(filter, "*.img");
    gtk_file_filter_add_pattern(filter, "*.ima");
    gtk_file_filter_add_pattern(filter, "*.360");
    gtk_file_filter_add_pattern(filter, "*.144");
    gtk_file_filter_add_pattern(filter, "*.IMG");
    gtk_file_chooser_add_filter(GTK_FILE_CHOOSER(dialog), filter);
    GtkFileFilter* all = gtk_file_filter_new();
    gtk_file_filter_set_name(all, "所有文件");
    gtk_file_filter_add_pattern(all, "*");
    gtk_file_chooser_add_filter(GTK_FILE_CHOOSER(dialog), all);

    if (gtk_dialog_run(GTK_DIALOG(dialog)) == GTK_RESPONSE_ACCEPT) {
        char* f = gtk_file_chooser_get_filename(GTK_FILE_CHOOSER(dialog));
        if (f) {
            ide_mount_floppy(drive, f);
            printf("[FLOPPY] %c: <- %s\n", drive ? 'B' : 'A', f);
            g_free(f);
        }
    }
    gtk_widget_destroy(dialog);
}

// ============================================================
// 机器复位（GTK 顶部「重启」按钮）：相当于按真机 RESET —— 
// 先把磁盘改动落盘，再清客机低端 RAM（ROM 在 F0000 起、保持不动）、复位各芯片，
// 让 BIOS 从头重新 POST。
// ============================================================
void ui_reset_machine(void) {
    if (console_mode) return;
    ide_flush();                 // 先落盘：像真机一样，复位不会丢已写入的内容
    memset(&memory[0], 0, 0xF0000);
    io_reset();
    dma_reset();
    fdc_reset();
    ide_reset();
    cpu_reset();
    vga_reset();
    vga_video_rom_install();     // 清 RAM 会把 C000 段的显卡选件 ROM 冲掉，必须重装
    printf("[RESET] 机器已复位，重新 POST …\n");
}

// ============================================================
// init_emulator
// ============================================================
void init_emulator(int argc, char** argv) {
    // 1) CPU 内存 + 复位
    cpu_init();

    // 2) VGA + GTK（gtk_init 在里面）；-console 下不建窗口
    if (console_mode) vga_set_headless(true);
    vga_init(argc, argv);

    // 3) BIOS ROM：只在启动时选（运行中不可改）。
    //    -bios 指定优先；否则弹 GTK 选择框，取消/失败则用默认 data/PCXTBIOS.BIN
    char* bios_file = NULL;
    for (int i = 1; i < argc - 1; i++) {
        if (strcmp(argv[i], "-bios") == 0) {
            bios_file = argv[i + 1];
            break;
        }
    }
    if (!bios_file && !console_mode) bios_file = open_bios_dialog();
    if (!bios_file) bios_file = (char*)find_default_bios();
    if (bios_file) {
        load_bios(bios_file);
    } else {
        printf("[BIOS] 找不到 BIOS ROM，请把 PCXTBIOS.BIN 放到 data/ 下，或用 -bios 指定\n");
    }

    // 3b) 显卡选件 ROM：DIP 报 EGA/VGA 时 BIOS 不提供 INT 10h，
    //     靠 C000 段这块 ROM 在选件扫描里把 INT 10h 指回 BIOS 自带的处理程序
    vga_video_rom_install();

    // 4) IDE 初始化（自动挂 dos.img）
    ide_init();

    // 5) 硬盘镜像：-disk 指定；GTK 模式下未指定则弹选择框（取消沿用 dos.img）
    char* disk_file = NULL;
    for (int i = 1; i < argc - 1; i++) {
        if (strcmp(argv[i], "-disk") == 0) {
            disk_file = argv[i + 1];
            break;
        }
    }
    if (!disk_file && !console_mode) disk_file = open_disk_dialog();
    if (disk_file) {
        ide_mount_disk(disk_file);
    }

    // 6) 软盘镜像：A: 用 -floppy；没指定就直接弹 GTK 文件选择框让你自己挑，
    //    运行中还能用窗口顶部按钮随时换盘
    char* floppy_file = NULL;
    for (int i = 1; i < argc - 1; i++) {
        if (strcmp(argv[i], "-floppy") == 0) {
            floppy_file = argv[i + 1];
            break;
        }
    }
    if (!floppy_file && !console_mode) floppy_file = open_floppy_dialog(0);
    ide_mount_floppy(0, floppy_file);   // NULL 也行，会挂空白软盘

    char* floppy2_file = NULL;
    for (int i = 1; i < argc - 1; i++) {
        if (strcmp(argv[i], "-floppy2") == 0) {
            floppy2_file = argv[i + 1];
            break;
        }
    }
    if (!floppy2_file && !console_mode) floppy2_file = open_floppy_dialog(1);
    ide_mount_floppy(1, floppy2_file);  // NULL 也行，会挂空白软盘

    // 7) 清屏
    vga_clear(0x07);

    // -console：没有窗口，GTK 信号一律不接
    if (console_mode) return;

    // 8) 键盘信号
    g_signal_connect(vga.window, "key-press-event",
                     G_CALLBACK(vga_on_key_press), NULL);
    g_signal_connect(vga.window, "key-release-event",
                     G_CALLBACK(vga_on_key_release), NULL);
    g_signal_connect(vga.drawing_area, "key-press-event",
                     G_CALLBACK(vga_on_key_press), NULL);
    g_signal_connect(vga.drawing_area, "key-release-event",
                     G_CALLBACK(vga_on_key_release), NULL);

    // 8b) 鼠标信号：点画布捕获（相对位移/按键），Ctrl+Alt+← 释放
    g_signal_connect(vga.drawing_area, "motion-notify-event",
                     G_CALLBACK(vga_on_motion), NULL);
    g_signal_connect(vga.drawing_area, "button-press-event",
                     G_CALLBACK(vga_on_button_press), NULL);
    g_signal_connect(vga.drawing_area, "button-release-event",
                     G_CALLBACK(vga_on_button_release), NULL);

    // 9) 窗口关闭
    g_signal_connect(vga.window, "delete-event",
                     G_CALLBACK(on_delete_event), NULL);
    g_signal_connect(vga.window, "destroy",
                     G_CALLBACK(on_window_destroy), NULL);

    gtk_widget_grab_focus(vga.drawing_area);
}

int main(int argc, char* argv[]) {
    setbuf(stdout, NULL);

    // -help：打印用法后直接退出（不建窗口）
    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "-help") == 0 || strcmp(argv[i], "--help") == 0 ||
            strcmp(argv[i], "-h") == 0) {
            print_usage(argv[0]);
            return 0;
        }
    }

    // -console 必须在 init_emulator 之前定下来：它决定要不要建 GTK 窗口；
    // 同时按用户要求，有 -console 就关掉 -dbg 日志
    bool composite_mode = false;
    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "-console") == 0) console_mode = true;
        if (strcmp(argv[i], "-composite") == 0) composite_mode = true;
    }
    // -console 下默认关日志（别打到屏幕上）；设了 DOS86_TRACE 才打开，便于无窗口排错
    if (console_mode) debug_mode = getenv("DOS86_TRACE") != NULL;

    printf(console_mode ? "80386 Emulator (console, BIOS boot)\n\n"
                        : "80386 Emulator (GTK, BIOS boot)\n\n");

    init_emulator(argc, argv);
    if (composite_mode) vga_set_composite(true);

    // 参数解析：-dbg 打开调试日志（-console 下不生效）
    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "-dbg") == 0 && !console_mode) debug_mode = true;
    }

    if (console_mode) {
        // ---- 终端模式：自己跑主循环（没有 GTK 事件循环）----
        printf("[CONSOLE] 终端模式（-console）：屏幕画在本终端，Ctrl+C 退出\n");
        fputs("\x1b[2J\x1b[H", stdout);
        memset(con_shadow, 0, sizeof(con_shadow));
        g_thread_new("con-in", con_reader, NULL);
        while (emu_running) {
            tick(NULL);
            con_poll_input();
            con_render();
            g_usleep(1000);          // 1ms 一拍，足够喂满 20MHz，也不烧 CPU
        }
        printf("\nEmulator stopped.\n");
        ide_cleanup();               // 把硬盘/软盘的改动写回镜像
        return 0;
    }

    // ★ 模拟心跳用 idle 驱动，不再用周期超时。
    //   原因：g_timeout_add 的 16ms 是"从回调被派发那一刻"起算的绝对时间，tick() 每帧
    //   要跑满 12ms 模拟，于是实际周期变成 28ms 以上；加上 Windows 上 GTK 主循环的调度，
    //   实测 tick() 每秒只被调用 10~23 次（应 ~62 次），速度只剩应有的 1/4，读盘一起慢。
    //   idle 源在按键事件(优先级 0)和重绘(120)之后运行，回调返回 TRUE 就立刻重新排队，
    //   tick() 内部按"实际经过时间"发指令配额，所以速度稳定锁在 20MHz，且不会饿死重绘。
    g_idle_add_full(G_PRIORITY_DEFAULT_IDLE, tick, NULL, NULL);

    // ★ 关键：只要用户没点 ×，gtk_main 返回了就重新跑
    while (emu_running) {
        gtk_main();
    }

    printf("Emulator stopped.\n");
    ide_cleanup();                   // 把硬盘/软盘的改动写回镜像
    return 0;
}