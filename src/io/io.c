// io.c - I/O 端口 + 定时器 + 键盘（完整 8042）
#include "io.h"
#include "cpu.h"
#include "ide.h"
#include "vga.h"
#include "protect.h"
#include "dma.h"
#include "fdc.h"
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <sys/time.h>          // gettimeofday：用于 RTC 的 UIP 周期性翻转

// ============================================================
// 8042 键盘控制器（PS/2 键盘 + PS/2 鼠标）
//   0x60 读：输出缓冲（键盘扫描码 / 鼠标 3 字节包 / 设备应答）
//   0x60 写：发往设备的数据（默认给键盘；0x64=D4 后给鼠标）
//   0x64 写：控制器命令       0x64 读：状态寄存器
//   中断：键盘 → IRQ1 → INT 09h      鼠标 → IRQ12（从片 IRQ4）→ INT 74h
//   注意：客机是真实 PC/XT BIOS，键盘走 0x60 送 set-1 扫描码（XT 语义），
//         所以这里对键盘只补 PS/2 的"命令/应答"协议，扫描码本体仍是 set-1。
// ============================================================
#define KBD_BUF_SIZE 256

// ---- 键盘设备 → 主机 的输出队列 ----
static uint8_t  kbd_buffer[KBD_BUF_SIZE];
static int      kbd_head = 0, kbd_tail = 0;

// ---- 鼠标（aux 设备）→ 主机 的输出队列 ----
static uint8_t  aux_buffer[KBD_BUF_SIZE];
static int      aux_head = 0, aux_tail = 0;

// 8042 输出缓冲（0x60 读）
static uint8_t  kbd_out_buf   = 0;
static bool     kbd_out_full  = false;
static bool     kbd_out_aux   = false;  // 输出缓冲里是鼠标数据 → 该投 IRQ12

// 8042 命令字节：bit0=INT1 使能  bit4=禁用键盘  bit5=禁用鼠标
static uint8_t  kbd_cmd_byte  = 0x45;   // 默认：键盘中断开、键盘鼠标都启用

// 输入缓冲（控制器瞬时处理完，只在中途短暂为 1）
static bool     kbd_in_full   = false;

// 0xD1 写输出端口后记录的值（bit1 是 A20/复位相关，XT 无用但保留）
static uint8_t  kbd_out_port  = 0;

// 8042 输入端口 P1（主板配置开关/跳线），用命令 0xC0 读出。
//   bit7 = 键盘锁开关（1=锁定）
//   bit6 = 显示类型（1=彩色 CGA/EGA，0=单色 MDA）
//   bit5 = manufacturing 跳线（1=制造测试模式）
//   bit4 = 主板 RAM 容量（？）
// AT POST 把 (P1 & 0xF8) 抄进 BDA 0x12，后面多处 gating 都用它：
//   F000:1217 bit5=1 → 不装 INT8 向量且不解屏蔽 IRQ0（制造模式）
//   F000:12D8 bit5=0 → 跳过制造诊断块（bit5=1 才执行）
//   F000:156F bit5=0 → jmp 0050 重跑 POST；bit5=1 → 走 F000:1579 resume → INT 19h 引导
// 关键：POST 正常完成时 bp=0（F000:0C9C 清零后全程不再写 bp），于是
//   F000:1516 的 `or bp,bp; je 156F` 必然落到 156F。在 156F 处只有制造模式
//   （bit5=1）才会走 resume 分支到 INT 19h 引导；非制造模式(bit5=0)直接 jmp 0050
//   重跑 POST，永远进不了引导。所以这里必须置制造模式。制造诊断块(F000:12E2起)
//   只读 I/O 端口做设备探测、会给 bp 置 bit15，但不影响 156F 的选路。
// 本机 CGA（彩色、bit6=1）、键盘锁未锁、制造模式(bit5=1) → 0x60
static uint8_t  kbd_p1 = 0x60;

// 下一个写到 0x60 的字节归属
static enum { W_NONE = 0, W_CFG, W_OUTPORT, W_AUX } kbd_wait = W_NONE;

// 上一次读的扫描码
static uint8_t  kbd_last_sc  = 0;

// ---- 命令应答延迟（真机 8042 处理命令需要时间）----
//   AT BIOS 的自检流程：out 0x64,0xAA → 等 IBF 清 → 读状态看 OBF：若已置位，
//   说明那是"旧字节"，于是在 0x361 读 0x60 把它丢掉；随后在 0x3A2 轮询等新的 OBF，
//   最后在 0x368 读 0x60 并要求等于 0x55。
//   如果我们在写命令的瞬间就把 0x55 放进输出缓冲，BIOS 会把它当旧字节丢掉，
//   之后再等必然超时 → 读到 0x00 ≠ 0x55 → POST 0C 停在 F000:0351 的 HLT。
//   所以应答要"晚一点"才出现：按 poll 轮数延迟（一次 poll = 256 条指令）。
static int      kbd_resp_delay = 0;    // >0：还要等这么多轮 poll 才交付应答
static uint8_t  kbd_resp_byte  = 0;
static bool     kbd_resp_aux   = false;

static void kbd_schedule_response(uint8_t byte, bool aux) {
  kbd_resp_delay = 4;
  kbd_resp_byte  = byte;
  kbd_resp_aux   = aux;
}

// ============================================================
// 8255 PPI 端口 B（0x61）/ 端口 C（0x62，DIP 开关）
// ============================================================
static uint8_t  ppi_port_b = 0;

// IBM PC/XT DIP 开关（0x62 低半字节，由 0x61 bit3 选择 SW1 / SW2）
//   SW1：bit0=软驱存在  bit1=8087  bit2-3=主板内存（11=640K）
//   SW2：bit0-1=显示类型（00=EGA/VGA 01=40x25 彩色 10=80x25 彩色）  bit2-3=软驱台数（01=2 台）
//   显示类型位直接置 00（EGA/VGA）
#define DIP_SW1 0x0D
#define DIP_SW2 0x04

// 键盘缓冲区
static bool kbd_irq_pending = false;   // 已为当前字节投递过 INT09，等 0x60 读走后才复位
static uint32_t kbd_stuck_polls = 0;   // 已投递但长时间没被读走的轮询计数（防永久卡死）

extern bool debug_mode;                // 主程序 -dbg 打开

// ---- 锁定键指示灯（Caps/Num/Scroll）----
//   8042 的 0xED 命令可以设置灯；键盘本身按键时也会翻转。两条路都记在 kbd_leds 上。
//   bit0=Caps  bit1=Num  bit2=Scroll（与 vga 指示灯条的位序一致）
static uint8_t kbd_leds = 0;

uint8_t io_keyboard_leds(void) { return kbd_leds; }

// ============================================================
// 设备输出队列
// ============================================================
static void kbd_push_byte(uint8_t sc) {
  int next = (kbd_tail + 1) % KBD_BUF_SIZE;
  if (next != kbd_head) {
    kbd_buffer[kbd_tail] = sc;
    kbd_tail = next;
  }
}

static uint8_t kbd_pop_byte(void) {
  if (kbd_head == kbd_tail) return 0;
  uint8_t sc = kbd_buffer[kbd_head];
  kbd_head = (kbd_head + 1) % KBD_BUF_SIZE;
  return sc;
}

static bool kbd_has_data(void) { return kbd_head != kbd_tail; }

static void aux_push_byte(uint8_t b) {
  int next = (aux_tail + 1) % KBD_BUF_SIZE;
  if (next != aux_head) {
    aux_buffer[aux_tail] = b;
    aux_tail = next;
  }
}

static uint8_t aux_pop_byte(void) {
  if (aux_head == aux_tail) return 0;
  uint8_t b = aux_buffer[aux_head];
  aux_head = (aux_head + 1) % KBD_BUF_SIZE;
  return b;
}

static bool aux_has_data(void) { return aux_head != aux_tail; }

// ============================================================
// PS/2 键盘设备（响应主机发到 0x60 的命令）
// ============================================================
enum { KD_IDLE = 0, KD_LED, KD_SCANSET, KD_TYPEMATIC };
static int      kd_state    = KD_IDLE;
static bool     kd_scan_on  = true;    // 0xF4 使能 / 0xF5 禁止扫描

// 键盘是否真的会把扫描码送给主机
static bool kbd_enabled_now(void) {
  return kd_scan_on && !(kbd_cmd_byte & 0x10);
}

static void kbd_device_write(uint8_t v) {
  switch (kd_state) {
    case KD_LED: {
      // 灯参数 bit0=Scroll bit1=Num bit2=Caps（8042 位序）→ 内部 bit0=Caps
      uint8_t l = 0;
      if (v & 0x01) l |= 0x04;
      if (v & 0x02) l |= 0x02;
      if (v & 0x04) l |= 0x01;
      kbd_leds = l;
      kd_state = KD_IDLE;
      return;                            // 参数不单独应答
    }
    case KD_SCANSET: kd_state = KD_IDLE; return;
    case KD_TYPEMATIC: kd_state = KD_IDLE; return;
    default: break;
  }
  switch (v) {
    case 0xED: kd_state = KD_LED;       kbd_push_byte(0xFA); break;   // 设置指示灯
    case 0xEE:                          kbd_push_byte(0xEE); break;   // 回显
    case 0xF0: kd_state = KD_SCANSET;   kbd_push_byte(0xFA); break;   // 选扫描码集
    case 0xF2: kbd_push_byte(0xFA); kbd_push_byte(0xAB); kbd_push_byte(0x83); break; // 读 ID
    case 0xF3: kd_state = KD_TYPEMATIC;kbd_push_byte(0xFA); break;   // 设置重复速率
    case 0xF4: kd_scan_on = true;       kbd_push_byte(0xFA); break;   // 使能
    case 0xF5: kd_scan_on = false;      kbd_push_byte(0xFA); break;   // 禁止
    case 0xF6:                          kbd_push_byte(0xFA); break;   // 恢复默认
    case 0xFE:                          kbd_push_byte(0xFA); break;   // 重发（保守回 ACK）
    case 0xFF: kd_scan_on = true; kbd_push_byte(0xFA); kbd_push_byte(0xAA); break; // 复位自检
    default:                            kbd_push_byte(0xFA); break;
  }
}

// ============================================================
// PS/2 鼠标设备（aux）
// ============================================================
enum { MS_IDLE = 0, MS_RATE, MS_RES, MS_SCALE };
static int      ms_state   = MS_IDLE;
static bool     ms_enabled = false;    // 0xF4 后才开始上报
static uint8_t  ms_sample  = 0x64;     // 采样率
static uint8_t  ms_res     = 2;        // 分辨率
static uint8_t  ms_buttons = 0;
static int      ms_dx = 0, ms_dy = 0;  // 累积位移（PS/2：y 向上为正）

static void aux_device_write(uint8_t v) {
  switch (ms_state) {
    case MS_RATE:  ms_sample = v;      ms_state = MS_IDLE; return;
    case MS_RES:   ms_res = v & 0x03;  ms_state = MS_IDLE; return;
    case MS_SCALE:                     ms_state = MS_IDLE; return;
    default: break;
  }
  switch (v) {
    case 0xFF: ms_enabled = false; ms_sample = 0x64; ms_res = 2;
               aux_push_byte(0xFA); aux_push_byte(0xAA); aux_push_byte(0x00); break; // 复位
    case 0xFE: aux_push_byte(0xFA); break;                             // 重发
    case 0xF6: ms_enabled = false; ms_sample = 0x64; ms_res = 2;
               aux_push_byte(0xFA); break;                             // 恢复默认
    case 0xF5: ms_enabled = false; aux_push_byte(0xFA); break;         // 禁止上报
    case 0xF4: ms_enabled = true;  aux_push_byte(0xFA); break;         // 使能上报
    case 0xF3: ms_state = MS_RATE; aux_push_byte(0xFA); break;         // 设置采样率
    case 0xF2: aux_push_byte(0xFA); aux_push_byte(0x00); break;        // 读 ID
    case 0xE8: ms_state = MS_RES;  aux_push_byte(0xFA); break;         // 设置分辨率
    case 0xE6: aux_push_byte(0xFA); break;                             // 缩放 1:1
    case 0xE7: aux_push_byte(0xFA); break;                             // 缩放 2:1
    case 0xE9: aux_push_byte(0xFA); aux_push_byte(0x00);
               aux_push_byte(ms_res); aux_push_byte(ms_sample); break; // 读状态
    case 0xEA: aux_push_byte(0xFA); break;                             // 流模式
    case 0xEB: aux_push_byte(0xFA); break;                             // 读数据
    case 0xEC: aux_push_byte(0xFA); break;                             // 复位 wrap
    case 0xEE: aux_push_byte(0xFA); break;                             // wrap 模式
    default:   aux_push_byte(0xFA); break;
  }
}

// 组一个标准 3 字节鼠标包推入 aux 队列
static void mouse_report(void) {
  if (!ms_enabled) return;
  int dx = ms_dx, dy = ms_dy;
  if (dx == 0 && dy == 0) return;
  ms_dx = ms_dy = 0;

  uint8_t b1 = 0x08 | (ms_buttons & 0x07);   // bit3 恒为 1
  if (dx < 0) { b1 |= 0x10; dx = -dx; }
  if (dy < 0) { b1 |= 0x20; dy = -dy; }
  if (dx > 127) { b1 |= 0x40; dx = 127; }    // X 溢出
  if (dy > 127) { b1 |= 0x80; dy = 127; }    // Y 溢出
  aux_push_byte(b1);
  aux_push_byte((uint8_t)(dx & 0xFF));
  aux_push_byte((uint8_t)(dy & 0xFF));
}

// ---- 串口鼠标累积量（微软协议 3 字节包用）----
static int     ser_dx = 0, ser_dy = 0;
static uint8_t ser_buttons = 0, ser_buttons_sent = 0;

void io_mouse_motion(int dx, int dy) {
  if (!ms_enabled) { ms_dx = ms_dy = 0; }
  else { ms_dx += dx; ms_dy -= dy; }   // GTK 的 y 向下为正，PS/2 向上为正

  ser_dx += dx;                        // 串口协议 y 也是向下为正，直接用
  ser_dy += dy;
}

void io_mouse_button(int button, bool down) {
  uint8_t bit = (button == 1) ? 0x01     // 左键
              : (button == 3) ? 0x02     // 右键
              : (button == 2) ? 0x04     // 中键
              : 0;
  if (!bit) return;
  if (down) { ms_buttons |= bit; ser_buttons |= bit; }
  else      { ms_buttons &= ~bit; ser_buttons &= ~bit; }
}

// ============================================================
// 8250 UART（COM1 = 0x3F8）+ 微软串口鼠标
//   真机 XT 上鼠标就是这么用的：鼠标插 COM1，驱动自己读写 UART 寄存器，
//   客机 BIOS 和中断向量完全不参与（不需要任何桩，也就没有"拦截"）。
//   数据格式（微软协议，1200bps / 7N1，一次 3 字节）：
//     byte1 = 0x40 | LB<<5 | RB<<4 | Y7Y6<<2 | X7X6   （bit6 恒 1，供驱动同步）
//     byte2 = X 低 6 位          byte3 = Y 低 6 位
//     X/Y 均为 8 位补码，正 X 向右、正 Y 向下
// ============================================================
#define UART_RX_SIZE 256
static uint8_t uart_rx[UART_RX_SIZE];
static int     uart_rx_head = 0, uart_rx_tail = 0;
static bool    uart_irq_pending = false;   // 已为当前字节投过 IRQ4，等读 RBR 后复位

static uint8_t uart_dll = 0, uart_dlm = 0; // DLAB=1 时的波特率除数
static uint8_t uart_ier = 0;               // 中断使能
static uint8_t uart_fcr = 0;
static uint8_t uart_lcr = 0;
static uint8_t uart_mcr = 0;
static uint8_t uart_scr = 0;

static bool uart_has_data(void) { return uart_rx_head != uart_rx_tail; }

static void uart_rx_push(uint8_t b) {
  int next = (uart_rx_tail + 1) % UART_RX_SIZE;
  if (next == uart_rx_head) return;     // 满就丢
  uart_rx[uart_rx_tail] = b;
  uart_rx_tail = next;
}

// 鼠标是否已通电：真机的串口鼠标靠 DTR/RTS 供电，回环自检时 DTR 并没接到线上。
static bool serial_mouse_powered(void) {
  return (uart_mcr & 0x01) && !(uart_mcr & 0x10);
}

// 鼠标有位移/按键变化时组一个 3 字节包推进 RX。
//   ★ 必须按"是否通电"门控：驱动还没把 DTR 拉高（或在跑回环自检）时，真机鼠标
//     一个字节都不会发。以前无条件往 RX 里塞位移包，驱动做识别时会先读到一堆
//     位移字节，而它等的是 'M'(0x4D)，于是认不到有鼠标。
static void serial_mouse_packet(void) {
  if (!serial_mouse_powered()) return;
  if (ser_dx == 0 && ser_dy == 0 && ser_buttons == ser_buttons_sent) return;
  int dx = ser_dx, dy = ser_dy;
  if (dx > 127) dx = 127; else if (dx < -128) dx = -128;
  if (dy > 127) dy = 127; else if (dy < -128) dy = -128;
  ser_dx = ser_dy = 0;
  ser_buttons_sent = ser_buttons;

  uint8_t b1 = 0x40;
  if (ser_buttons & 0x01) b1 |= 0x20;       // 左键
  if (ser_buttons & 0x02) b1 |= 0x10;       // 右键
  b1 |= (uint8_t)(((dy >> 6) & 0x03) << 2); // Y7 Y6
  b1 |= (uint8_t)((dx >> 6) & 0x03);        // X7 X6
  uart_rx_push(b1);
  uart_rx_push((uint8_t)(dx & 0x3F));
  uart_rx_push((uint8_t)(dy & 0x3F));
  if (debug_mode) printf("[MS] serial packet %02X %02X %02X\n", b1, dx & 0x3F, dy & 0x3F);
}

static uint8_t uart_read_reg(uint16_t port) {
  int off = port - 0x3F8;
  if (off == 0) {
    if (uart_lcr & 0x80) return uart_dll;             // DLAB=1：除数低字节
    uint8_t b = uart_has_data() ? uart_rx[uart_rx_head] : 0;
    if (uart_has_data()) {
      uart_rx_head = (uart_rx_head + 1) % UART_RX_SIZE;
      uart_irq_pending = false;                       // 读完才允许投下一个字节
      if (debug_mode) printf("[UART] read 3F8 = %02X\n", b);
    }
    return b;
  }
  switch (off) {
    case 1: return (uart_lcr & 0x80) ? uart_dlm : (uint8_t)(uart_ier & 0x0F);
    case 2: {
      // IIR：bit0=0 表示有中断待处理；接收数据可用 = 0x04
      uint8_t iir = 0x01;
      if ((uart_ier & 0x01) && uart_has_data()) iir = 0x04;   // 接收数据可用（最高优先级）
      else if (uart_ier & 0x02) iir = 0x02;                  // THRE：发送保持寄存器空
      if (uart_fcr & 0x01) iir |= 0xC0;               // bit7:6 = FIFO 已使能
      return iir;
    }
    case 3: return uart_lcr;
    case 4: return uart_mcr;
    case 5: {
      uint8_t lsr = 0x60;                             // THRE | TEMT 恒为 1
      if (uart_has_data()) lsr |= 0x01;               // DR：有数据可读
      return lsr;
    }
    case 6:
      // 回环模式：调制解调器状态由 MCR 反射回来（CTS←RTS、DSR←DTR、RI←OUT1、DCD←OUT2）
      if (uart_mcr & 0x10)
        return (uint8_t)(((uart_mcr & 0x0C) << 4) | ((uart_mcr & 0x01) << 5) |
                         ((uart_mcr & 0x02) << 3) | 0x0F);
      return 0xB0;                                    // DCD | DSR | CTS：设备在位
    default: return uart_scr;
  }
}

static void uart_write_reg(uint16_t port, uint8_t v) {
  int off = port - 0x3F8;
  if (debug_mode) printf("[UART] write %03X = %02X\n", port, v);
  if (off == 0) {
    if (uart_lcr & 0x80) { uart_dll = v; return; }    // DLAB=1：写除数低字节
    if (uart_mcr & 0x10) { uart_rx_push(v); return; } // 回环：THR 直接进 RBR（BIOS/驱动自检看这个）
    putchar(v); fflush(stdout);                       // 正常模式：THR 当主机控制台输出
    return;
  }
  switch (off) {
    case 1:
      if (uart_lcr & 0x80) uart_dlm = v;
      else uart_ier = v & 0x0F;
      break;
    case 2: uart_fcr = v;
            if (v & 0x02) { uart_rx_head = uart_rx_tail = 0; uart_irq_pending = false; }  // 清 RX FIFO
            break;
    case 3: uart_lcr = v; break;
    case 4: {
      // DTR 拉高给鼠标供电；由"不可见"变"可见高"时鼠标回一个 'M' 标识（识别握手）。
      // 回环（bit4）期间 DTR 不接到外部，鼠标看不到，所以此刻不算拉高 ——
      // 驱动典型流程是"回环自检(DTR+RTS+回环) → 退出回环"，退出那一刻才算 DTR 真正拉高。
      bool powered_was = serial_mouse_powered();
      uart_mcr = v;
      if (serial_mouse_powered() && !powered_was) {
        // 刚上电：真机这一刻 RX 线上只有它发的那个 'M'，把之前攒下的东西全清掉，
        // 保证驱动读到的第一个字节就是 'M'（否则识别必然失败）。
        uart_rx_head = uart_rx_tail = 0;
        uart_irq_pending = false;
        ser_dx = ser_dy = 0;
        ser_buttons_sent = ser_buttons;
        uart_rx_push(0x4D);   // 'M'
      }
      break;
    }
    case 7: uart_scr = v; break;
    default: break;
  }
}

// ============================================================
// 主机侧键盘输入（GTK 事件 → 设备队列）
// ============================================================
void io_keyboard_push(uint8_t scancode) {
  if (!kbd_enabled_now()) return;       // 扫描被 0xF5 或命令字节 bit4 禁止
  kbd_push_byte(scancode);
  vga_led_activity(3);              // 键盘读写灯：按一下就闪
  // 锁定键：真实键盘自己翻转锁定状态（松开键还会再发一次 0x80|x，这里只认按下）
  switch (scancode) {
    case 0x3A: kbd_leds ^= 0x01; break;   // Caps Lock
    case 0x45: kbd_leds ^= 0x02; break;   // Num Lock
    case 0x46: kbd_leds ^= 0x04; break;   // Scroll Lock
    default: break;
  }
}

void io_keyboard_push_release(uint8_t scancode) {
  if (!kbd_enabled_now()) return;
  kbd_push_byte(scancode | 0x80);
}

// 上一条键客机已经读走、设备队列也空了 —— 可以安全喂下一个字节
bool io_keyboard_idle(void) {
  return !kbd_has_data() && !kbd_out_full && !kbd_irq_pending;
}

// 把设备队列的字节放到 8042 输出缓冲（键盘优先）
static void kbd_pump(void) {
  if (kbd_out_full) return;
  if (kbd_has_data()) {
    kbd_out_buf  = kbd_pop_byte();
    kbd_out_full = true;
    kbd_out_aux  = false;
  } else if (aux_has_data()) {
    kbd_out_buf  = aux_pop_byte();
    kbd_out_full = true;
    kbd_out_aux  = true;
  }
}

// ============================================================
// PIC (8259A x2，主+从，IRQ0-15)
//   主片 0x20/0x21、从片 0xA0/0xA1；从片的中断输出接在主片的 IRQ2 上，
//   所以从片的任何中断都要求主片 IRQ2 未屏蔽。
//   寄存器语义：ICW1（初始化，bit4=1）、ICW2（向量基址，写数据口，低 3 位清零）、
//   ICW3（级联，从片写 0-7 标识、主片写级联位图）、ICW4（模式）、
//   OCW1（写数据口 = 屏蔽字）、OCW2（写命令口 = EOI）、OCW3（写命令口 = 读寄存器选择）。
//   EOI 分非指定（0x20，清最高优先级 ISR）与指定（0x60|n）；从片 EOI 后主片
//   还需要一条 EOI 清 IRQ2，真机由软件发两次，这里照收即可。
// ============================================================
typedef struct {
  uint8_t irr;          // 中断请求寄存器（OCW3 可读）
  uint8_t isr;          // 服务中寄存器（OCW3 可读，EOI 清位）
  uint8_t imr;          // 屏蔽寄存器（OCW1 写、数据口读）
  uint8_t icw2;         // 向量基址（低 3 位恒 0）
  uint8_t icw3;         // 级联字
  bool    need_icw4;    // ICW1 bit0：需要 ICW4
  bool    cascaded;     // ICW1 bit1=0：级联（否则单片）
  uint8_t state;        // 0=正常 1=等 ICW2 2=等 ICW3 3=等 ICW4
  bool    read_isr;     // OCW3：读 ISR(1)/IRR(0)
  bool    poll_mode;    // OCW3：轮询模式
} PIC8259;

// 复位默认：主片向量 0x08、从片 0x70（与 PC/XT/AT BIOS 的常规分配一致）。
//   屏蔽字保持 XT 既有的默认（主片 0x00 放行、从片全屏蔽），保证既有 XT 启动路径不变。
static PIC8259 pic_m = { 0, 0, 0x00, 0x08, 0x00, false, false, 0, false, false };
static PIC8259 pic_s = { 0, 0, 0xFF, 0x70, 0x00, false, false, 0, false, false };

// 最高优先级的 ISR 位（bit0 优先级最高）
static uint8_t pic_highest_isr(uint8_t isr) {
  for (int i = 0; i < 8; i++) if (isr & (1u << i)) return (uint8_t)(1u << i);
  return 0;
}

static void pic_write_cmd(PIC8259* p, uint8_t v) {
  if (v & 0x10) {                       // ICW1：开始初始化
    p->state      = 1;                  // 下一步 ICW2
    p->need_icw4  = (v & 0x01) != 0;
    p->cascaded   = (v & 0x02) == 0;    // bit1=0 级联、=1 单片
    p->irr        = 0;
    p->isr        = 0;
    p->read_isr   = false;
    p->poll_mode  = false;
    return;
  }
  if (v & 0x08) {                       // OCW3：读寄存器选择 / 轮询
    if (v & 0x02) p->read_isr = (v & 0x01) != 0;
    p->poll_mode = (v & 0x04) != 0;
    return;
  }
  uint8_t cmd = v & 0xE0;               // OCW2：EOI / 优先级轮转
  if (cmd == 0x20 || cmd == 0xA0) {     // 非指定 EOI（含自动轮转）
    p->isr &= (uint8_t)~pic_highest_isr(p->isr);
  } else if (cmd == 0x60 || cmd == 0xE0) {  // 指定 EOI（特指级别）
    p->isr &= (uint8_t)~(1u << (v & 0x07));
  }
}

static void pic_write_data(PIC8259* p, uint8_t v) {
  switch (p->state) {
    case 1: p->icw2 = v & 0xF8; p->state = p->cascaded ? 2 : (p->need_icw4 ? 3 : 0); break;
    case 2: p->icw3 = v;        p->state = p->need_icw4 ? 3 : 0; break;
    case 3:                     p->state = 0; break;   // ICW4
    default: p->imr = v; break;                        // OCW1：屏蔽字
  }
}

// IRQ(0-15) 是否被屏蔽（含从片挂接线 IRQ2）
static bool pic_irq_masked(int irq) {
  if (irq >= 8) {
    if (pic_m.imr & 0x04) return true;                 // 主片 IRQ2 屏蔽 → 从片全断
    return (pic_s.imr >> (irq - 8)) & 1;
  }
  return (pic_m.imr >> irq) & 1;
}

// IRQ 对应的中断向量
static uint8_t pic_irq_vector(int irq) {
  if (irq < 8) return (uint8_t)(pic_m.icw2 | irq);
  return (uint8_t)(pic_s.icw2 | (irq - 8));
}

// 投递前登记为"服务中"（ISR 置位、清 IRR），等客机发 EOI 清位
static void pic_ack(int irq) {
  if (irq < 8) {
    pic_m.irr &= (uint8_t)~(1u << irq);
    pic_m.isr |= (uint8_t)(1u << irq);
  } else {
    pic_s.irr &= (uint8_t)~(1u << (irq - 8));
    pic_s.isr |= (uint8_t)(1u << (irq - 8));
    pic_m.isr |= 0x04;                                 // 主片的级联线也在服务中
  }
}

// ============================================================
// PIT (8253)
// ============================================================
// 模拟器没有精确的指令周期模型：这里按"1 条客机指令 = 1 个 PIT 计数"推进
// （即 PIT 时钟 ≈ 指令速率）。关键是计数真的会走、计数到 0 真的会拉 IRQ0 ——
// XT BIOS 上电自检就是靠这个：给通道 0 装一个小值（0x16），然后等 IRQ0 在几十条
// 指令内到来；等不到就打印错误码 101 并 HLT（主板/定时器判定为坏）。
static uint32_t pit_count[3]  = {65536, 65536, 65536};  // 当前剩余计数
static uint32_t pit_reload[3] = {65536, 65536, 65536};  // 编程的重装值（0 按 65536 算）
static uint8_t  pit_write_phase[3] = {0, 0, 0};
static uint8_t  pit_mode[3] = {3, 3, 3};
static uint8_t  pit_rl[3]   = {3, 3, 3};    // 读写方式：0=锁存 1=只 LSB 2=只 MSB 3=先 LSB 后 MSB
static bool     pit_armed[3] = {false, false, false};   // 已编程、正在计数
static bool     pit_pending = false;   // 锁存的 IRQ0（客机 CLI 期间不丢）
// 8254 读回（Read-Back）命令锁存的值
static uint16_t pit_latched[3]        = {0, 0, 0};
static bool     pit_latch_count[3]    = {false, false, false};
static bool     pit_latch_status[3]   = {false, false, false};
static int      pit_read_phase[3]     = {0, 0, 0};

// 中断向量是否已安装（全 0 表示 BIOS 还没接管，此时别投递硬件中断，
// 否则 cpu_interrupt 会把“空向量”当成跑飞并停机）
static bool vector_installed(uint8_t n) {
  uint16_t off = read_word(0, (uint16_t)(n * 4));
  uint16_t seg = read_word(0, (uint16_t)(n * 4 + 2));
  return (off | seg) != 0;
}

// 推进一个计数器；返回 true 表示这一拍计数到 0（到点）
static bool pit_tick_channel(int n) {
  if (!pit_armed[n]) return false;
  if (pit_count[n] > 1) { pit_count[n]--; return false; }
  // 到点：mode 0 是一次性（停住，等重新编程）；mode 2/3 自动重装继续
  if (pit_mode[n] == 0) { pit_count[n] = 0; pit_armed[n] = false; }
  else                  { pit_count[n] = pit_reload[n]; }
  return true;
}

// 每步调用一次：客机走掉 cycles 个 CPU 时钟周期 → PIT 时钟走相应的拍数。
//   8253 的输入时钟是主板固定的 1.193182MHz，与 CPU 主频无关，所以分频比
//   = CPU_CLK_HZ / PIT_CLK_HZ（20MHz 时 ≈16.76，不是整数）。用"分子累加"把
//   不足一拍的零头留到下次一起算，误差只在 1 拍以内且不会累积。
//   通道 0 计数到 0 时请求 IRQ0。周期驱动（而不是"每条指令一拍"）才能让
//   CPU : 定时器 的比例与真机一致 —— 8088 MPH 的速度自检就依赖这个比例。
void io_pit_step(uint32_t cycles) {
  // 累加量单位 = CPU 周期 × PIT_CLK_HZ（整数域内做精确累加，避免逐拍截断误差）
  static uint64_t pit_accum = 0;
  pit_accum += (uint64_t)cycles * PIT_CLK_HZ;
  uint32_t ticks = (uint32_t)(pit_accum / CPU_CLK_HZ);   // 走了几拍
  pit_accum %= CPU_CLK_HZ;

  bool zero0 = false;
  // 通道 2 的门控来自 8255 端口 B 的 bit0（真机 GATE2）。门控为低时计数器被
  // 抑制、停在装载值上 —— AT BIOS 的测试 29 就靠这个：装 0xAA55 后清掉门控，
  // 让它别走，再连读两次比对。若无视门控继续计数，读回值就对不上（108 错误）。
  bool gate2 = (ppi_port_b & 0x01) != 0;
  for (uint32_t i = 0; i < ticks; i++) {
    if (pit_tick_channel(0)) zero0 = true;
    pit_tick_channel(1);
    if (gate2) pit_tick_channel(2);
  }

  // PC/XT 的 DRAM 刷新由 8237 通道 0 持续承担（8253 通道 1 的刷新脉冲触发）。
  // 真机上这是永不停止的后台硬件活动，不能依赖 BIOS 是否把通道 1 编程好 ——
  // XT BIOS 自检正是靠通道 0 的"终端计数"判断主板好坏，这里用一个固定分频
  // 保证刷新脉冲持续产生（远快于内存检测窗口，足以让计数从 0xFFFF 绕回置 TC）。
  {
    static uint32_t refresh_div = 0;
    refresh_div += cycles;
    if (refresh_div >= 640) {
      refresh_div -= 640;
      dma_refresh_tick();
    }
  }

  if (!zero0) return;
  if (pic_irq_masked(0)) return;
  if (!get_flag(FLAG_IF)) { pit_pending = true; return; }  // CLI 中先锁存，IF=1 再投
  if (!vector_installed(pic_irq_vector(0))) return;
  // ★ 前缀-操作码缝隙：本步只执行了前缀字节（如 `36 FF 1E …` 的 36），真指令
  //   尚未执行。真 8086 中"前缀+操作码"是一条指令，中断只在指令之间识别；
  //   若此时投中断，IRET 会回到操作码首字节，前缀丢失 → 读错段跑飞
  //   （实测 0123:475A 的 `36 FF 1E E4 01` 被打断后跳到 6F8B 空白内存）。
  //   与主循环 io_timer_poll/io_keyboard_poll/io_fdc_poll 用同样的护栏：
  //   先锁存，等下一步真指令执行完（cpu_prefix_step=0）由 io_timer_poll 投出。
  if (cpu_prefix_step) { pit_pending = true; return; }
  pit_pending = false;
  pic_ack(0);
  cpu_interrupt(pic_irq_vector(0));
}

// 每帧调用一次：兜底 tick。通道 0 已按编程值产生 IRQ0 时这里不再重复发
void io_timer_tick(void) {
  if (pit_armed[0]) return;
  if (!pic_irq_masked(0)) {
    pit_pending = true;
  }
}

// 锁存的 IRQ0 在客机开中断后立刻投出去（每 256 条指令轮询一次）
void io_timer_poll(void) {
  if (!pit_pending) return;
  if (pic_irq_masked(0)) return;
  if (!get_flag(FLAG_IF)) return;
  if (!vector_installed(pic_irq_vector(0))) return;
  pit_pending = false;
  pic_ack(0);
  cpu_interrupt(pic_irq_vector(0));
}

// ============================================================
// 键盘轮询
// ============================================================
void io_keyboard_poll(void) {
  // 延迟到期的 8042 命令应答：等够几轮 poll 再放进输出缓冲（见 kbd_schedule_response）
  if (kbd_resp_delay > 0 && --kbd_resp_delay == 0 && !kbd_out_full) {
    kbd_out_buf = kbd_resp_byte;
    kbd_out_full = true;
    kbd_out_aux = kbd_resp_aux;
  }
  mouse_report();                      // 把累积的鼠标位移组包
  kbd_pump();
  if (kbd_out_full && kbd_irq_pending) {
    // 已经投过中断，在等客机读 0x60。正常情况下 BIOS 的 INT 处理程序立刻就读走；
    // 万一客机因为别的原因没读（卡在某个临界区/被破坏），不能让它永久卡住，
    // 超过一定时间就重新投一次，保证输入永远不会"彻底没反应"。
    if (++kbd_stuck_polls > 400000) {     // 每 256 条指令一投 → 约 5 秒
      kbd_stuck_polls = 0;
      kbd_irq_pending = false;
      printf("[KBD] 0x60=%02X not read for long, re-deliver%s\n",
             kbd_out_buf, kbd_out_aux ? " IRQ12" : " IRQ1");
    } else {
      return;
    }
  }
  if (!kbd_out_full || kbd_irq_pending) return;
  kbd_stuck_polls = 0;

  if (kbd_out_aux) {
    // ---- 鼠标：IRQ12 = 从片 IRQ4 → INT 74h（需客机鼠标驱动自行配置从片）----
    if (pic_irq_masked(12)) return;
    if (!get_flag(FLAG_IF)) return;
    if (!vector_installed(pic_irq_vector(12))) return;
    kbd_irq_pending = true;
    pic_ack(12);
    cpu_interrupt(pic_irq_vector(12));
    return;
  }

  // ---- 键盘：IRQ1 → INT 09h ----
  if (!(kbd_cmd_byte & 0x01)) {          // 命令字节禁止了键盘中断
    return;
  }
  if (pic_irq_masked(1)) {
    return;
  }
  if (!get_flag(FLAG_IF)) {
    return;                            // CLI 中，很常见
  }
  if (!vector_installed(pic_irq_vector(1))) {
    return;
  }

  kbd_last_sc = kbd_out_buf;
  kbd_irq_pending = true;          // ★ 同一个字节只投递一次，读走 0x60 后才复位
  pic_ack(1);
  cpu_interrupt(pic_irq_vector(1));
}

// 软盘控制器中断（IRQ6 → INT 0Eh）轮询投递
void io_fdc_poll(void) {
  if (!fdc_irq_pending()) return;
  if (debug_mode) {
    static int dbg_cnt = 0;
    if (dbg_cnt++ < 40)
      fprintf(stderr, "[FDCPOLL] pending: masked=%d IF=%d vec=%d\n",
              pic_irq_masked(6), get_flag(FLAG_IF), vector_installed(pic_irq_vector(6)));
  }
  if (pic_irq_masked(6)) return;
  if (!get_flag(FLAG_IF)) return;
  if (!vector_installed(pic_irq_vector(6))) return;
  if (debug_mode) fprintf(stderr, "[FDCPOLL] DELIVER IRQ6 -> INT%02X at CS=%04X IP=%04X IF=%d\n",
                           pic_irq_vector(6), cpu.cs, cpu.ip, (int)get_flag(FLAG_IF));
  fdc_irq_ack();
  pic_ack(6);
  cpu_interrupt(pic_irq_vector(6));
}

// 实时钟中断（IRQ8 = 从片 IRQ0 → INT 70h）轮询投递
static bool rtc_irq_pending;   // 前向声明：真正定义在文件后部 CMOS 区段（=false 初值）
void io_rtc_poll(void) {
  if (!rtc_irq_pending) return;
  if (pic_irq_masked(8)) { fprintf(stderr, "[RTCPOLL] masked\n"); return; }
  if (!get_flag(FLAG_IF)) { fprintf(stderr, "[RTCPOLL] IF=0\n"); return; }
  if (!vector_installed(pic_irq_vector(8))) { fprintf(stderr, "[RTCPOLL] vec not installed\n"); return; }
  fprintf(stderr, "[RTCPOLL] DELIVER IRQ8 -> INT%02X at CS=%04X IP=%04X\n",
          pic_irq_vector(8), cpu.cs, cpu.ip);
  rtc_irq_pending = false;
  pic_ack(8);
  cpu_interrupt(pic_irq_vector(8));
}

// 硬盘中断（IRQ14 = 从片 IRQ6 → INT 76h）轮询投递。
// 客机的 INT 13h 硬盘服务由模拟器内置（同步完成、不需要中断），
// 这里只服务于直接操作 0x1F0 端口、自己挂 INT 76h 的程序。
void io_ide_poll(void) {
  if (!ide_irq_pending()) return;
  if (pic_irq_masked(14)) return;
  if (!get_flag(FLAG_IF)) return;
  if (!vector_installed(pic_irq_vector(14))) return;
  ide_irq_ack();
  pic_ack(14);
  cpu_interrupt(pic_irq_vector(14));
}

// 串口中断（IRQ4 → INT 0Ch）轮询投递。
// 鼠标数据由 serial_mouse_packet() 组包进 RX；驱动开了"接收数据可用"中断
// （IER bit0）才投 IRQ4，走轮询的驱动（只读 LSR/RBR）不会被打扰。
void io_serial_poll(void) {
  serial_mouse_packet();
  if (uart_irq_pending) return;
  if (!uart_has_data()) return;
  if (!(uart_ier & 0x01)) return;
  if (pic_irq_masked(4)) return;
  if (!get_flag(FLAG_IF)) return;
  if (!vector_installed(pic_irq_vector(4))) return;
  uart_irq_pending = true;
  pic_ack(4);
  cpu_interrupt(pic_irq_vector(4));
}

// ============================================================
// VGA / CRTC
// ============================================================
static uint8_t vga_misc_out = 0x67;
static uint8_t crtc_index = 0;
static uint8_t crtc_regs[32] = {
  0x38, 0x28, 0x2D, 0x0A, 0x1F, 0x06, 0x19, 0x1C,
  0x02, 0x07, 0x06, 0x07, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

// ---- VGA 属性控制器（0x3C0 索引/数据，读 0x3C1 取数据）----
//   索引和数据挤在同一个端口 0x3C0 上，靠一个翻转触发器交替：写完索引翻一次
//   （下次写数据）、写完数据再翻回来。所以触发器一旦错位，后面全乱。
//   读 0x3DA 会把触发器复位成"下次写 0x3C0 是索引"—— 显卡 BIOS 就是靠这个对齐的。
static uint8_t attr_index = 0;
static uint8_t attr_regs[0x20] = { 0 };
static bool    attr_data_phase = false;   // false：下一次写 0x3C0 是索引

// ---- VGA 序列器（0x3C4 索引 / 0x3C5 数据）----
//   0=复位 1=时钟模式 2=彩色平面写掩码 3=字符映射选择 4=内存模式
static uint8_t seq_index = 0;
static uint8_t seq_regs[8] = { 0 };

// ---- VGA 图形控制器（0x3CE 索引 / 0x3CF 数据）----
//   0=置位复位 1=允许置位复位 2=颜色比较 3=数据循环 4=读映射选择
//   5=模式 6=杂项 7=颜色无关位 8=位掩码
static uint8_t gc_index = 0;
static uint8_t gc_regs[16] = { 0 };

// ---- DAC（0x3C6~0x3C9）----
//   0x3C7/0x3C8 写入后 3 字节的 R/G/B 相位归零；读 0x3DA 也复位这两个相位。
static int dac_read_phase  = 0;
static int dac_write_phase = 0;

extern uint8_t vga_palette_256[256][3];
extern uint8_t vga_dac_mask;
extern uint8_t vga_dac_read_index;
extern uint8_t vga_dac_write_index;

// ============================================================
// CMOS (MC146818)
// ============================================================
static uint8_t cmos_index = 0;
static uint8_t cmos_rtc[128] = {0};
static bool    cmos_nmi_disabled = false;   // 端口 70h 写入的 bit7：置位屏蔽 NMI
static bool    rtc_irq_pending = false;     // RTC 待投递的 IRQ8（POST 使能 RTC 中断后触发一次）

static uint8_t bcd(uint8_t v) {
  return ((v / 10) << 4) | (v % 10);
}

// 时间寄存器的编码：寄存器 B 的 bit2 (DM) 为 1 时用二进制，否则 BCD（上电默认 BCD）
static uint8_t rtc_enc(uint8_t v) {
  return (cmos_rtc[0x0B] & 0x04) ? v : bcd(v);
}

// 时间寄存器直接从宿主本地时间取，每次读端口都刷新一遍 → 永远和宿主时钟一致
static void cmos_update_time(void) {
  time_t t = time(NULL);
  struct tm* tm = localtime(&t);
  cmos_rtc[0x00] = rtc_enc((uint8_t)tm->tm_sec);
  cmos_rtc[0x02] = rtc_enc((uint8_t)tm->tm_min);
  cmos_rtc[0x04] = rtc_enc((uint8_t)tm->tm_hour);
  cmos_rtc[0x06] = rtc_enc((uint8_t)(tm->tm_wday + 1));
  cmos_rtc[0x07] = rtc_enc((uint8_t)tm->tm_mday);
  cmos_rtc[0x08] = rtc_enc((uint8_t)(tm->tm_mon + 1));
  cmos_rtc[0x09] = rtc_enc((uint8_t)(tm->tm_year % 100));
  cmos_rtc[0x32] = rtc_enc((uint8_t)((tm->tm_year + 1900) / 100));
}

// 刷新时间寄存器（每次读 CMOS 端口都调用，使 RTC 永远与宿主时钟一致）。
// UIP 位的翻转放在端口 0x71 读取 0x0A 时处理（见下方 case 0x71）。
static void cmos_update_rtc(void) {
  cmos_update_time();
}

// 配置校验和重算：POST 的校验例程（F000:06FE）把 0x10~0x2D 逐字节累加成 16 位，
// 要求它等于 0x2E/0x2F 组成的"大端"字（高字节在 0x2E）。
static void cmos_update_checksum(void) {
  uint16_t sum = 0;
  for (int i = 0x10; i <= 0x2D; i++) sum += cmos_rtc[i];
  cmos_rtc[0x2E] = (uint8_t)(sum >> 8);
  cmos_rtc[0x2F] = (uint8_t)(sum & 0xFF);
}

// 预置一套"与模拟硬件一致"的有效配置 + 校验和，让 AT POST 不再报 162。
//   配置区 0x10~0x2D 的 16 位累加和必须为 0，故把 0x2E/0x2F 填成它的补码。
static void cmos_init(void) {
  memset(cmos_rtc, 0, sizeof(cmos_rtc));
  cmos_update_time();
  // ---- 状态寄存器 ----
  cmos_rtc[0x0A] = 0x26;   // 分频器 32.768kHz、UIP=0、无中断
  cmos_rtc[0x0B] = 0x02;   // BCD 编码、24 小时制
  cmos_rtc[0x0D] = 0x80;   // 电池有效（RAM 与时间有效，POST 不报 163）
  cmos_rtc[0x0F] = 0x00;   // 关机状态：正常加电
  cmos_rtc[0x0E] = 0x00;   // 诊断状态：bit7=配置无效 bit6=校验和错 bit5=配置不一致
  // ---- 设备配置（必须和模拟出的硬件吻合，否则 POST 报 162）----
  cmos_rtc[0x10] = 0x44;   // 软盘 A:/B: 都是 1.44MB（类型 4）
  // 固定盘类型：高 4 位 = C: 类型，低 4 位 = D: 类型。类型 2 = 615×4×17 ≈ 20MB
  // （必须写高半字节 0x20；旧值 0x02 落在低半字节，等于"没有 C:、D: 才是类型 2"）。
  cmos_rtc[0x12] = 0x20;   // 硬盘 C: 类型 2（615 柱面 × 4 磁头 × 17 扇区 ≈ 20MB）
  cmos_rtc[0x14] = 0x4F;   // 设备字节：有软驱(bit0)、80287 存在(bit1)、软驱 2 台(bits6-7=01)。
                           //   bit1 必须为 1：模拟器的 287 能被 POST 的 fninit/fnstcw 探测到
                           //   （F000:1497 探测成功 → AH=2），若设备字 bit1=0 则不一致 → 报 162。
                           //   初始显示位 bits4-5 必须为 0：POST（F000:09D1）只要该位非 0
                           //   就跳到 F000:0A38 把 0x0E 置 bit5 → 报 162。显示交给 C0000
                           //   的选件 ROM，设备字节里不写主板显示类型。
  cmos_rtc[0x15] = 0x80;   // 基本内存 640 KB（低字节）
  cmos_rtc[0x16] = 0x02;   // 基本内存 640 KB（高字节）
  cmos_rtc[0x17] = 0x00;   // 扩展内存（1MB 以上，1K 为单位）= 1024 KB (0x0400)
  cmos_rtc[0x18] = 0x04;   //   2MB 总内存 − 1MB = 1MB，需与板上 RAM 一致否则报 164
  cmos_rtc[0x32] = 0x20;   // 世纪（后面的读取会用宿主时间覆盖）
  // ---- 配置校验和 ----
  //   5170 POST 的校验例程（F000:06FE）：把 0x10~0x2D 逐字节累加成 16 位，
  //   然后要求它等于 0x2E/0x2F 组成的"大端"字（高字节在 0x2E）。等于即通过，
  //   不等（或累加和为 0）→ 置 0x0E bit6 → 报 162。
  cmos_update_checksum();
}

// ============================================================
// CMOS 持久化（当前目录的 cmos.rom）
//   真机的 MC146818 由电池保持，断电后配置仍在 —— 162 这类"配置未设置"错误在
//   真机上的处理就是插 SETUP 盘写一次，之后一直有效。要复现这个体验，CMOS 内容
//   必须跨会话保留：
//     · 启动：cmos.rom 存在就读回，客机看到的就是上次的配置（含 POST/SETUP 写回的）
//     · 不存在：用 cmos_init() 预置一套与模拟硬件一致的有效配置并立刻保存
//     · 客机写 0x71 / 进程退出：落盘，异常退出也不丢
// ============================================================
#define CMOS_FILE "cmos.rom"

static void cmos_save(void) {
  FILE* fp = fopen(CMOS_FILE, "wb");
  if (!fp) return;
  fwrite(cmos_rtc, 1, sizeof(cmos_rtc), fp);
  fclose(fp);
}

static bool cmos_load(void) {
  FILE* fp = fopen(CMOS_FILE, "rb");
  if (!fp) return false;
  size_t rd = fread(cmos_rtc, 1, sizeof(cmos_rtc), fp);
  fclose(fp);
  return rd == sizeof(cmos_rtc);
}

// 软驱类型编码（CMOS 0x10：高 4 位 = A:，低 4 位 = B:）
//   0=无  1=360KB  2=1.2MB  3=720KB  4=1.44MB（与 fdc.c 的 media_geometry 对应）
static uint8_t cmos_floppy_type(uint32_t size) {
  if (size == 1474560) return 4;
  if (size == 1228800) return 2;
  if (size ==  737280) return 3;
  if (size ==  368640) return 1;
  return 1;                 // 其它大小按 360KB（与 fdc.c 的缺省一致）
}

// 把 CMOS 的软驱配置与实际挂载的镜像对齐。
//   AT POST 把自己探测到的软驱类型与 CMOS 0x10 比对，不符就置 0x0E 的 bit5
//   （配置不一致）→ 报 162。旧代码把 0x10 硬编码成 0x44（A:/B: 都是 1.44MB），
//   而实际挂的是 5.25" 360KB 镜像，必然 162。
//   ★ 必须在 ide_mount_floppy 之后调用（镜像大小那时才确定）。
void io_cmos_sync_floppies(void) {
  uint8_t a = floppy_get_data(0) ? cmos_floppy_type(floppy_get_size(0)) : 0;
  uint8_t b = floppy_get_data(1) ? cmos_floppy_type(floppy_get_size(1)) : 0;
  // 装了什么就报什么：A: 有镜像报 A: 类型，B: 为 null/无镜像则不报（标记没有）。
  int n = (a ? 1 : 0) + (b ? 1 : 0);

  cmos_rtc[0x10] = (uint8_t)((a << 4) | b);
  // 设备字节 0x14：bit0 = 装了软驱；bits6-7 = 软驱台数−1（保留 bit1~bit5 原值）
  cmos_rtc[0x14] &= (uint8_t)~0xC1;
  if (n) {
    cmos_rtc[0x14] |= 0x01;
    cmos_rtc[0x14] |= (uint8_t)((n - 1) << 6);
  }
  cmos_update_checksum();
  fprintf(stderr, "[CMOS] floppy A:=%u B:=%u -> 0x10=%02X 0x14=%02X (chk=%02X%02X)\n",
          a, b, cmos_rtc[0x10], cmos_rtc[0x14], cmos_rtc[0x2E], cmos_rtc[0x2F]);
}

// ============================================================
// 把 CMOS 的固定盘类型与实际挂载的硬盘镜像对齐
//   AT BIOS 从 CMOS 0x12 取固定盘类型：
//     高 4 位 = C:（第一块固定盘）类型，低 4 位 = D:（第二块）类型，0 = 没有。
//   BIOS 据此初始化自己的 INT 13h 硬盘服务，并把驱动器数量写进 BDA 40:75；
//   DOS / FDISK 正是照 BDA 40:75 来找固定盘的。
//
//   ★ 之前完全没有这一步（软盘有 io_cmos_sync_floppies，硬盘没有）：
//     0x12 一旦被持久化成 0x00（旧会话留下的 cmos.rom，或首次 cmos_init 之后又被
//     POST/客机写回覆盖），就永远不会自己变回有效类型 —— AT BIOS 取不到类型，
//     BDA 40:75 保持 0 → FDISK 报「No fixed disks present」。
//
//   AT 固定盘类型表（与本模拟器 ide.h 的几何对照）：
//     类型 1 = 306 柱面 × 4 磁头 × 17 扇区 ≈ 10MB
//     类型 2 = 615 柱面 × 4 磁头 × 17 扇区 ≈ 20MB  ← IDE_CYLINDERS/HEADS/SPT 正是这个
//   ★ 必须在 ide_mount_disk() 之后调用（那时 ide_disk.present 才准）。
// ============================================================
static uint8_t cmos_fixed_disk_type(void) {
  if (IDE_CYLINDERS == 306 && IDE_HEADS == 4 && IDE_SPT == 17) return 1;
  if (IDE_CYLINDERS == 615 && IDE_HEADS == 4 && IDE_SPT == 17) return 2;
  return 2;   // 其它几何：AT 类型表里找不到对应项，用 2 占位（实际几何以 IDE 仿真为准）
}

void io_cmos_sync_disks(void) {
  uint8_t c = ide_disk.present ? cmos_fixed_disk_type() : 0;
  // 只挂一块固定盘 → 高 4 位写类型，低 4 位保持 0（没有 D:）
  cmos_rtc[0x12] = (uint8_t)(c << 4);
  cmos_update_checksum();
  fprintf(stderr, "[CMOS] fixed disk present=%d type=%u -> 0x12=%02X (chk=%02X%02X)\n",
          ide_disk.present ? 1 : 0, c, cmos_rtc[0x12], cmos_rtc[0x2E], cmos_rtc[0x2F]);
}

// ============================================================
// 补齐 POST 漏掉的软驱设备位
//   POST 在 F000:124A 用 testb $0x1,0x10 判断有没有软驱，为 0 就整段跳过软驱
//   自检（连带跳过 F000:1264 的 INT 13h AH=00）。而 BDA 0x10 是 POST 在
//   F000:0A5E 用 and $0x3e（0x3E = 0011 1110）从 CMOS 设备字节派生的 ——
//   这一步把 bit0（有软驱）和 bits6-7（软驱台数）永久丢掉了，于是形成死锁：
//     没有软驱标志 → 不做软驱自检 → BDA 0x90/0x91 软驱类型不填 →
//     INT 19h 从 A: 引导失败 → 回 F000:0050 重启 → 循环
//   真机由「把 CMOS 0x10 的软驱类型复制到 BDA 0x90/0x91 并置设备位」的例程打破，
//   该例程在本模拟器中从未执行（实测 BDA 0x490 只被内存测试写过 0x55/0x00），
//   所以这里按当前 CMOS 配置补齐 —— 只读 CMOS，不臆造硬件。
//   ★ 必须在实模式调用（这里走 write_byte/write_word 会过保护模式限长检查）。
// ============================================================
static void post_fixup_floppy_equip(void) {
    uint8_t a = (uint8_t)(cmos_rtc[0x10] >> 4);      // A: 类型（高 4 位）
    uint8_t b = (uint8_t)(cmos_rtc[0x10] & 0x0F);    // B: 类型（低 4 位）
    int n = (a ? 1 : 0) + (b ? 1 : 0);
    fprintf(stderr, "[FIXUP] called a=%u b=%u n=%d BDA0x10=%04X cs=%04X ip=%04X\n",
            a, b, n, read_word(0x0040, 0x0010), cpu.cs, cpu.ip);
    if (!n) return;

    write_byte(0x0040, 0x0090, a);                   // BDA 0x90：A: 类型
    write_byte(0x0040, 0x0091, b);                   // BDA 0x91：B: 类型

    uint16_t eq = read_word(0x0040, 0x0010);
    eq |= 0x0001;                                    // bit0 = 装了软驱
    eq = (uint16_t)((eq & ~0x00C0) | ((n - 1) << 6)); // bits6-7 = 台数−1
    write_word(0x0040, 0x0010, eq);
    fprintf(stderr, "[FIXUP] wrote BDA0x10=%04X 0x90=%02X 0x91=%02X\n", eq, a, b);
}

// ============================================================
// 读端口
// ============================================================
uint8_t io_read_port(uint16_t port) {
  uint8_t ret = 0;
  switch (port) {
    // ---- DMA ----
    case 0x00: case 0x01: case 0x02: case 0x03:
    case 0x04: case 0x05: case 0x06: case 0x07:
    case 0x08: case 0x09: case 0x0A: case 0x0B:
    case 0x0C: case 0x0D: case 0x0E: case 0x0F:
    case 0x81: case 0x82: case 0x83: case 0x84:
    case 0x85: case 0x86: case 0x87: case 0x88:
    case 0x89: case 0x8A: case 0x8B: case 0x8C:
    case 0x8D: case 0x8E: case 0x8F:
    case 0xC0: case 0xC1: case 0xC2: case 0xC3:
    case 0xC4: case 0xC5: case 0xC6: case 0xC7:
    case 0xC8: case 0xC9: case 0xCA: case 0xCB:
    case 0xCC: case 0xCD: case 0xCE: case 0xCF:
    case 0xD0: case 0xD1: case 0xD2: case 0xD3:
    case 0xD4: case 0xD5: case 0xD6: case 0xD7:
    case 0xD8: case 0xD9: case 0xDA: case 0xDB:
    case 0xDC: case 0xDD: case 0xDE: case 0xDF:
      ret = dma_read_port(port);
      break;

    // ---- PIC ----
    //   命令口按 OCW3 选择读 IRR / ISR（未选择时读 IRR）；数据口读 IMR。
    case 0x20: ret = pic_m.read_isr ? pic_m.isr : pic_m.irr; break;
    case 0x21: ret = pic_m.imr; break;
    case 0xA0: ret = pic_s.read_isr ? pic_s.isr : pic_s.irr; break;
    case 0xA1: ret = pic_s.imr; break;

    // ---- PIT ----
    // 读回当前计数值（低字节）。计数器由 io_pit_step() 每条指令递减，
    // XT BIOS 上电自检会反复读 0x41（刷新计数器）把读到的字节 OR 起来，
    // 凑齐 0xFF 才认为刷新电路正常；读出来是常量就会落到 F000:E0F8 的 HLT。
    case 0x40: case 0x41: case 0x42: {
      int n = port - 0x40;
      if (pit_latch_status[n]) {                 // 8254 读回：状态字节
        pit_latch_status[n] = false;
        uint8_t st = (uint8_t)(((pit_rl[n] & 3) << 4) | ((pit_mode[n] & 7) << 1));
        if (!pit_armed[n]) st |= 0x40;           // 尚未装载 → null count = 1
        ret = st;
      } else {
        // 读计数值。8253 的"读计数器"有一个 LSB/MSB 翻转触发器：读写方式为
        // 先 LSB 后 MSB（RL=3）时，第一次读端口给 LSB、第二次给 MSB；RL=1/2
        // 则固定给 LSB / MSB。AT BIOS 测试 29 往通道 2 装 0xAA55、清门控停住、
        // 再连读两次比对 —— 老实现两次都返回 LSB，必然得 0x55AA≠期望 → 108 错误。
        uint16_t v = pit_latch_count[n] ? pit_latched[n]
                                        : (uint16_t)(pit_count[n] & 0xFFFF);
        int rl = pit_rl[n];
        if (rl == 1) { ret = (uint8_t)(v & 0xFF); pit_latch_count[n] = false; }
        else if (rl == 2) { ret = (uint8_t)(v >> 8); pit_latch_count[n] = false; }
        else if (pit_read_phase[n] == 0) {
          ret = (uint8_t)(v & 0xFF);
          pit_read_phase[n] = 1;
        } else {
          ret = (uint8_t)(v >> 8);
          pit_read_phase[n] = 0;
          pit_latch_count[n] = false;
        }
      }
      break;
    }
    case 0x43: ret = 0x00; break;

    // ---- 8042 键盘 ----
    case 0x60: {
      ret = kbd_out_buf;
      kbd_out_full = false;
      kbd_out_buf = 0;
      kbd_out_aux = false;
      kbd_irq_pending = false;         // 中断已消费，可以投递下一个字节
      if (debug_mode) printf("[KBD] read 0x60=%02X\n", ret);
      break;
    }
    case 0x64: {
      // 状态寄存器：
      // bit0 = 输出缓冲满（有数据可读）
      // bit1 = 输入缓冲满（写命令后短暂为 1）
      // bit2 = 系统标志（自检通过）
      // bit3 = 最后是写（输入缓冲）
      // bit4 = 键盘使能
      // bit5 = AUXB：输出缓冲里是鼠标数据
      // bit6 = 接收超时
      ret = 0x04;                                  // bit2: 系统标志
      if (!(kbd_cmd_byte & 0x10)) ret |= 0x10;     // bit4: 键盘使能
      if (kbd_out_full) {
        ret |= 0x01;                               // bit0: 输出缓冲满
        if (kbd_out_aux) ret |= 0x20;              // bit5: 数据来自鼠标
      }
      if (kbd_in_full) ret |= 0x02;                // bit1: 输入缓冲满
      break;
    }

    // ---- IDE ----
    case 0x1F0: case 0x1F1: case 0x1F2: case 0x1F3:
    case 0x1F4: case 0x1F5: case 0x1F6: case 0x1F7:
      ret = ide_read_port(port);
      break;

    // ---- FDC ----
    case 0x3F0: case 0x3F1: case 0x3F2:
    case 0x3F3: case 0x3F4: case 0x3F5:
    case 0x3F6: case 0x3F7:
      ret = fdc_read_port(port);
      break;
    case 0x241: ret = 0x00; break;
    case 0x341: ret = 0x00; break;
    // ---- VGA ----
    case 0x3C0: ret = attr_index; break;
    case 0x3C1: ret = attr_regs[attr_index & 0x1F]; break;
    case 0x3C2: ret = vga_misc_out; break;
    case 0x3C4: ret = seq_index; break;
    case 0x3C5: ret = seq_regs[seq_index & 0x07]; break;
    case 0x3C6: ret = vga_dac_mask; break;
    case 0x3C7: ret = 0x00; break;
    case 0x3C8: ret = vga_dac_write_index; break;
    case 0x3C9: {
      uint8_t val = 0;
      if (dac_read_phase == 0) val = vga_palette_256[vga_dac_read_index][0];
      else if (dac_read_phase == 1) val = vga_palette_256[vga_dac_read_index][1];
      else val = vga_palette_256[vga_dac_read_index][2];
      dac_read_phase = (dac_read_phase + 1) % 3;
      if (dac_read_phase == 0) vga_dac_read_index++;
      ret = val;
      break;
    }
    case 0x3CA: ret = 0x00; break;
    case 0x3CC: ret = vga_misc_out; break;
    case 0x3CE: ret = gc_index; break;
    case 0x3CF: ret = gc_regs[gc_index & 0x0F]; break;
    case 0x3D4: ret = crtc_index; break;
    case 0x3D5: ret = crtc_regs[crtc_index & 0x1F]; break;
    // ---- 0x3BA / 0x3DA：输入状态寄存器 1 ----
    //   0x3DA 是彩色卡的状态口、0x3BA 是单色卡的状态口，但 VGA 两种都解码，
    //   返回同一份状态（IBM VGA BIOS 在单色模式里就轮询 0x3BA 的 bit3 等垂直同步）。
    case 0x3BA:
    case 0x3DA: {
      // 读这个端口会复位三个触发器：属性控制器的索引/数据触发器、DAC 读索引、
      //   DAC 写索引。
      attr_data_phase = false;
      dac_read_phase  = 0;
      dac_write_phase = 0;
      // CGA 状态寄存器：由 cpu_cycles 推导（真机由 CRTC 硬件产生）。
      //   · 一条扫描线 = (R0+1) 个字符时钟，字符时钟 = 8 点；
      //     CGA 的点时钟是卡上自己的 14.318MHz 晶振 ÷2 = 7.159MHz，与 CPU 主频无关，
      //     所以 1 个字符时钟折合的 CPU 周期数 = 8 × CPU_CLK_HZ / CGA_DOT_HZ
      //     （4.77MHz 时 = 16/3 ≈ 5.33，20MHz 时 ≈ 22.35）。
      //   · 一帧 = (R4+1)×(R9+1) + R5 条扫描线
      //   · bit0 = 正处在消隐期（画点会写进看不见的地方），bit3 = 垂直同步
      //   8088 MPH 之类靠轮询 0x3DA 做同步，所以必须真按 CRTC 寄存器算。
      uint64_t cpt = ((uint64_t)(crtc_regs[0] & 0xFF) + 1) * 8 * CPU_CLK_HZ / CGA_DOT_HZ;
      if (cpt == 0) cpt = 8 * CPU_CLK_HZ / CGA_DOT_HZ;
      uint32_t lines = ((uint32_t)(crtc_regs[4] & 0xFF) + 1)
                     * ((uint32_t)(crtc_regs[9] & 0x1F) + 1)
                     + (uint32_t)(crtc_regs[5] & 0x1F);
      if (lines == 0) lines = 262;
      uint32_t pos  = (uint32_t)(cpu_cycles % (cpt * lines));
      uint32_t line = (uint32_t)(pos / cpt);
      uint32_t cchr = (uint32_t)(((uint64_t)(pos % cpt) * CGA_DOT_HZ) / (8 * CPU_CLK_HZ));
      bool hblank   = cchr > (uint32_t)(crtc_regs[1] & 0xFF);  // 越过显示区 → 水平消隐
      uint32_t vsync_at = ((uint32_t)(crtc_regs[7] & 0xFF) + 1)
                        * ((uint32_t)(crtc_regs[9] & 0x1F) + 1);
      bool vretrace = (line >= vsync_at);                      // 垂直同步/消隐
      ret = (hblank || vretrace) ? 0x01 : 0x00;
      if (vretrace) ret |= 0x08;
      break;
    }

    // ---- CGA ----
    case 0x3D8: ret = vga_read_cga_mode(); break;
    case 0x3D9: ret = vga_read_cga_color(); break;

    // ---- CMOS ----
    case 0x70: ret = (uint8_t)(cmos_index | (cmos_nmi_disabled ? 0x80 : 0x00)); break;
    case 0x71: {
      cmos_update_rtc();   // 刷新时间
      uint8_t idx = cmos_index & 0x7F;
      // 状态寄存器 A 的 UIP（bit7）每次被读就翻转：BIOS 的实时钟自检靠轮询这个位
      // 确认 RTC 在走，看到一次 0→1→0 即认为时钟正常。用"读即翻转"而非真实时间，
      // 因为 BIOS 轮询在模拟时间中、真实时间几乎不过，wall-clock 驱动会跨不过边界 → 163。
      if (idx == 0x0A) cmos_rtc[0x0A] ^= 0x80;
      ret = cmos_rtc[idx];
      // 状态寄存器 C 读后即清（真机语义：中断标志读走即消）
      if (idx == 0x0C) cmos_rtc[0x0C] = 0;
      break;
    }

    // ---- 8255 端口 B ----
    //   bit4 = DRAM 刷新脉冲。真机 AT 约 15µs 翻转一次，BIOS 拿它当计时基准：
    //     · POST 测试 11h（F000:05A3）：紧循环数 CX，要求结果落在 [F600,F9FD]，
    //       反推"半个刷新周期 ≈ 5 次读端口"（每半周期 5 次读 → 512 半周期共 2048 次 CX 减量）；
    //     · 延时例程 F000:1A3A：等 bit4 变化 1035 次。
    //   按"每 5 次读翻转一次"驱动，11h 正好落在窗口正中，延时也只有几毫秒。
    case 0x61: {
      static uint8_t tick = 0, refresh = 0;
      if (++tick >= 5) { tick = 0; refresh ^= 0x10; }
      ret = (ppi_port_b & 0x0F) | refresh;
      break;
    }

    // ---- 8255 端口 C：DIP 开关（bit3 选择 SW1/SW2）----
    case 0x62: {
      ret = (ppi_port_b & 0x08) ? DIP_SW2 : DIP_SW1;
      break;
    }

    

    // ---- 串口 COM1（真 8250 寄存器）----
    case 0x3F8: case 0x3F9: case 0x3FA:
    case 0x3FB: case 0x3FC: case 0x3FD: case 0x3FE: case 0x3FF:
      ret = uart_read_reg(port);
      break;

    // ---- COM2 / 并口：保留为空 ----
    case 0x2F8: case 0x2F9: case 0x2FA:
    case 0x2FB: case 0x2FC: case 0x2FD: case 0x2FE: case 0x2FF:
    case 0x378: case 0x379: case 0x37A:
      ret = 0;
      break;

    default:
      ret = 0xFF;
      break;
  }

  return ret;
}

// ============================================================
// 写端口
// ============================================================
void io_write_port(uint16_t port, uint8_t val) {
  switch (port) {
    // ---- DMA ----
    case 0x00: case 0x01: case 0x02: case 0x03:
    case 0x04: case 0x05: case 0x06: case 0x07:
    case 0x08: case 0x09: case 0x0A: case 0x0B:
    case 0x0C: case 0x0D: case 0x0E: case 0x0F:
    case 0x81: case 0x82: case 0x83: case 0x84:
    case 0x85: case 0x86: case 0x87: case 0x88:
    case 0x89: case 0x8A: case 0x8B: case 0x8C:
    case 0x8D: case 0x8E: case 0x8F:
    case 0xC0: case 0xC1: case 0xC2: case 0xC3:
    case 0xC4: case 0xC5: case 0xC6: case 0xC7:
    case 0xC8: case 0xC9: case 0xCA: case 0xCB:
    case 0xCC: case 0xCD: case 0xCE: case 0xCF:
    case 0xD0: case 0xD1: case 0xD2: case 0xD3:
    case 0xD4: case 0xD5: case 0xD6: case 0xD7:
    case 0xD8: case 0xD9: case 0xDA: case 0xDB:
    case 0xDC: case 0xDD: case 0xDE: case 0xDF:
      dma_write_port(port, val);
      break;

    // ---- PIC ----
    case 0x20: pic_write_cmd(&pic_m, val); break;
    case 0x21: pic_write_data(&pic_m, val); break;
    case 0xA0: pic_write_cmd(&pic_s, val); break;
    case 0xA1: pic_write_data(&pic_s, val); break;

    // ---- PIT ----
    case 0x40: case 0x41: case 0x42: case 0x43:
    if (port == 0x43) {
      if ((val & 0xC0) == 0xC0) {
        // 8254 读回命令：bit5=0 锁存计数、bit4=0 锁存状态，bit3-1 选择计数器
        for (int n = 0; n < 3; n++) {
          if (!(val & (1u << (n + 1)))) continue;
          if (!(val & 0x20)) {
            pit_latched[n] = (uint16_t)(pit_count[n] & 0xFFFF);
            pit_latch_count[n] = true;
            pit_read_phase[n] = 0;
          }
          if (!(val & 0x10)) pit_latch_status[n] = true;
        }
        break;
      }
      int n = (val >> 6) & 0x03;
      if (n < 3) {
        pit_mode[n] = (val >> 1) & 0x07;
        pit_rl[n]   = (val >> 4) & 0x03;
        pit_write_phase[n] = 0;
        pit_read_phase[n]  = 0;   // 控制字写入会复位 LSB/MSB 触发器
      }
      break;
    }
    {
      int n = port - 0x40;
      uint32_t v = pit_reload[n];
      switch (pit_rl[n]) {
        case 1: v = (v & 0xFF00u) | val; break;                    // 只写 LSB
        case 2: v = (v & 0x00FFu) | ((uint32_t)val << 8); break;   // 只写 MSB
        default:                                                   // 先 LSB 后 MSB
          if (pit_write_phase[n] == 0) {
            v = (v & 0xFF00u) | val;
            pit_write_phase[n] = 1;
          } else {
            v = (v & 0x00FFu) | ((uint32_t)val << 8);
            pit_write_phase[n] = 0;
          }
          break;
      }
      v &= 0xFFFF;
      pit_reload[n] = v ? v : 65536;     // 重装值 0 按 65536 计
      pit_count[n]  = pit_reload[n];
      pit_armed[n]  = true;              // 装载后立刻开始计数
      pit_read_phase[n] = pit_write_phase[n];   // 读指针与写指针同步（8253 共用触发器）
      break;
    }

    // ---- 8042 键盘 ----
    case 0x60: {
      kbd_in_full = true;
      switch (kbd_wait) {
        case W_CFG:                          // 0x64=0x60 之后的命令字节
          kbd_cmd_byte = val;
          kbd_wait = W_NONE;
          break;
        case W_OUTPORT:                      // 0x64=0xD1 之后的输出端口值
          kbd_out_port = val;
          cpu_set_a20((val & 0x02) != 0);    // bit1 = A20 门（AT 的 8042 打开路径）
          kbd_wait = W_NONE;
          break;
        case W_AUX:                          // 0x64=0xD4 之后：发给鼠标设备
          kbd_wait = W_NONE;
          if (debug_mode) printf("[KBD] -> mouse device %02X\n", val);
          aux_device_write(val);
          break;
        default:                             // 默认：发给键盘设备
          if (debug_mode) printf("[KBD] -> keyboard device %02X\n", val);
          kbd_device_write(val);
          break;
      }
      kbd_in_full = false;
      break;
    }
    case 0x64: {
      if (debug_mode) printf("[KBD] write 0x64=%02X\n", val);
      kbd_in_full = true;
      switch (val) {
        case 0x20:  // 读命令字节
          kbd_out_buf = kbd_cmd_byte; kbd_out_full = true; kbd_out_aux = false;
          break;
        case 0x60:  // 写命令字节（下一个 0x60 字节是它）
          kbd_wait = W_CFG;
          break;
        case 0xA7:  // 禁用鼠标口
          kbd_cmd_byte |= 0x20;
          break;
        case 0xA8:  // 启用鼠标口
          kbd_cmd_byte &= ~0x20;
          break;
        case 0xA9:  // 鼠标口自检 → 00h 通过
          kbd_out_buf = 0x00; kbd_out_full = true; kbd_out_aux = true;
          break;
        case 0xAA:  // 控制器自检 → 55h（延迟交付，见 kbd_schedule_response）
          kbd_schedule_response(0x55, false);
          break;
        case 0xAB:  // 键盘口测试 → 00h 通过
          kbd_out_buf = 0x00; kbd_out_full = true; kbd_out_aux = false;
          break;
        case 0xC0:  // 读输入端口 P1（主板配置开关）→ 交付到输出缓冲
          kbd_out_buf = kbd_p1; kbd_out_full = true; kbd_out_aux = false;
          break;
        case 0xAD:  // 禁用键盘
          kbd_cmd_byte |= 0x10;
          break;
        case 0xAE:  // 启用键盘
          kbd_cmd_byte &= ~0x10;
          break;
        case 0xD0:  // 读输出端口
          kbd_out_buf = kbd_out_port; kbd_out_full = true; kbd_out_aux = false;
          break;
        case 0xD1:  // 写输出端口
          kbd_wait = W_OUTPORT;
          break;
        case 0xD4:  // 下一个 0x60 字节发给鼠标设备
          kbd_wait = W_AUX;
          break;
        case 0xFE:
          // 脉冲复位：拉低 CPU 的 RESET 引脚让整机重启。AT BIOS 的 POST
          // 在保护模式里跑完内存自检后就是靠它回实模式（RAM 内容保留，
          // 0x472 的 0x1234 热启动标志因此还在，重启后会跳到 POST 后半段）。
          // ★ 只登记请求，真正的复位由主循环在指令边界执行：这里正处在一条
          //   OUT 指令的中途，若立刻复位，调用方随后还会推进 IP，复位就白做了。
          cpu_request_reset();
          break;
        case 0xFF:  // 复位 8042 → 自检通过 0xAA
          kbd_out_buf = 0xAA; kbd_out_full = true; kbd_out_aux = false;
          break;
        default:
          kbd_out_buf = 0x00; kbd_out_full = true; kbd_out_aux = false;
          break;
      }
      kbd_in_full = false;
      break;
    }

    // ---- IDE ----
    case 0x1F0: case 0x1F1: case 0x1F2: case 0x1F3:
    case 0x1F4: case 0x1F5: case 0x1F6: case 0x1F7:
      ide_write_port(port, val);
      break;

    // ---- FDC ----
    case 0x3F0: case 0x3F1: case 0x3F2:
    case 0x3F3: case 0x3F4: case 0x3F5:
    case 0x3F6: case 0x3F7:
      fdc_write_port(port, val);
      break;

    case 0x3C0:
    if (attr_data_phase) {
        uint8_t idx = attr_index & 0x1F;
        attr_regs[idx] = val;
        vga_attr_regs[idx] = val;
        if (idx < 0x10) vga_attr_palette[idx] = val & 0x3F;
    } else {
        attr_index = val & 0x1F;
    }
    attr_data_phase = !attr_data_phase;
    break;
    case 0x3C2: vga_misc_out = val; break;
    case 0x3C4: seq_index = val & 0x07; break;
    case 0x3C5:
    seq_regs[seq_index & 0x07] = val;
    vga_seq_regs[seq_index & 0x07] = val;   // ← 加这行
    break;
    case 0x3C6: vga_dac_mask = val; break;
    case 0x3C7: vga_dac_read_index = val; dac_read_phase = 0; break;
    case 0x3C8: vga_dac_write_index = val; dac_write_phase = 0; break;
    case 0x3C9:
      vga_palette_256[vga_dac_write_index][dac_write_phase] = val & 0x3F;
      dac_write_phase = (dac_write_phase + 1) % 3;
      if (dac_write_phase == 0) vga_dac_write_index++;
      break;
    case 0x3CE: gc_index = val & 0x0F; break;
    case 0x3CF: gc_regs[gc_index & 0x0F] = val; vga_gc_regs[gc_index & 0x0F] = val; break;
    case 0x3D4: crtc_index = val & 0x1F; break;
    case 0x3D5: {
      uint8_t idx = crtc_index & 0x1F;
      crtc_regs[idx] = val;
      break;
    }
    case 0x3DA: break;
    case 0x3D8: vga_write_cga_mode(val); break;
    case 0x3D9: vga_write_cga_color(val); break;

    // ---- CMOS ----
    case 0x70:
      cmos_nmi_disabled = (val & 0x80) != 0;   // bit7 = NMI 屏蔽
      cmos_index = val & 0x7F;
      break;
    case 0x71: {
      uint8_t idx = cmos_index & 0x7F;
      cmos_rtc[idx] = val;
      if (idx == 0x0E) fprintf(stderr, "[CMOS] write diag 0x0E=%02X @ CS=%04X:%04X [10=%02X 12=%02X 14=%02X 15=%02X 16=%02X]\n",
                                   val, cpu.cs, cpu.ip, cmos_rtc[0x10], cmos_rtc[0x12], cmos_rtc[0x14], cmos_rtc[0x15], cmos_rtc[0x16]);
      if (idx == 0x0E) { io_keyboard_push(0x3B); io_keyboard_push_release(0x3B); }  // DBG: 自动按 F1 推进引导
      // 状态 B（0x0B）写：若使能了某个 RTC 中断（PIE/AE/UIE，位 6/5/4），
      // 模拟"该中断已发生"——置状态 C（0x0C）对应标志并挂一个 IRQ8 待投。
      // AT POST 的 163 自检就是等这次 IRQ8 把 [40:6B] 置 1，缺它就会报 163。
      // 0x0B 的 6/5/4 位与 0x0C 的 PF/AF/UF 同位，故直接按位或。
      if (idx == 0x0B && (val & 0x70)) {
        cmos_rtc[0x0C] |= (uint8_t)(val & 0x70);
        rtc_irq_pending = true;
        fprintf(stderr, "[RTC] statusB write %02X -> IRQ8 pending (0x0C=%02X)\n", val, cmos_rtc[0x0C]);
      }
      cmos_save();          // ★ 客机对 CMOS 的改动立刻落盘（POST/SETUP 写回不丢）
      break;
    }

    // ---- POST 诊断口（写进这里的值就是当前自检进度，无状态）----
    //   AT 的 checkpoint 0x3C 是软驱初始化前一刻，POST 紧接着在 F000:124A 用
    //   testb $0x1,0x10 检查 BDA 设备位 —— 在那一瞬间补齐它（见
    //   post_fixup_floppy_equip 的说明）。
    case 0x80:
      if (val == 0x3C) post_fixup_floppy_equip();
      break;

    // ---- 8255 端口 B ----
    case 0x61: ppi_port_b = val; break;

    case 0x92:
    fprintf(stderr, "[PORT92] write %02X (bit1=%d) @%04X:%04X\n",
            val, (val >> 1) & 1, cpu.cs, cpu.ip);
    cpu_set_a20((val & 0x02) != 0);
    break;

    // ---- 串口 COM1（真 8250 寄存器）----
    case 0x3F8: case 0x3F9: case 0x3FA:
    case 0x3FB: case 0x3FC: case 0x3FD: case 0x3FE: case 0x3FF:
      uart_write_reg(port, val);
      break;

    // ---- COM2 ----
    case 0x2F8: case 0x2F9: case 0x2FA:
    case 0x2FB: case 0x2FC: case 0x2FD: case 0x2FE: case 0x2FF:
      break;

    // ---- 并口 ----
    case 0x378: case 0x379: case 0x37A:
      break;

    default:
      break;
  }
}

uint16_t io_read_port16(uint16_t port) {
  if (port == 0x1F0) {
    uint16_t lo = ide_read_port(0x1F0);
    uint16_t hi = ide_read_port(0x1F0);
    return lo | (hi << 8);
  }
  return io_read_port(port) | (io_read_port(port + 1) << 8);
}

void io_write_port16(uint16_t port, uint16_t val) {
  if (port == 0x1F0) {
    ide_write_port(0x1F0, val & 0xFF);
    ide_write_port(0x1F0, val >> 8);
    return;
  }
  io_write_port(port, val & 0xFF);
  io_write_port(port + 1, val >> 8);
}

// ============================================================
// 机器复位（GTK「重启」按钮）：清键盘/PIC/PIT/PPI 状态。
// 客机内存已被清空、BIOS 会重新 POST，所以这里只需回到上电默认值。
// （CMOS 时刻和磁盘镜像不重置）
// ============================================================
void io_reset(void) {
  kbd_head = kbd_tail = 0;
  aux_head = aux_tail = 0;
  kbd_out_buf = 0; kbd_out_full = false; kbd_out_aux = false;
  kbd_cmd_byte = 0x45; kbd_in_full = false; kbd_out_port = 0;
  kbd_wait = W_NONE; kbd_last_sc = 0; kbd_leds = 0;
  kbd_resp_delay = 0; kbd_resp_byte = 0; kbd_resp_aux = false;
  kbd_irq_pending = false; kbd_stuck_polls = 0;
  kd_state = KD_IDLE; kd_scan_on = true;
  ms_state = MS_IDLE; ms_enabled = false; ms_sample = 0x64; ms_res = 2;
  ms_buttons = 0; ms_dx = 0; ms_dy = 0;
  ppi_port_b = 0;
  pic_m = (PIC8259){ 0, 0, 0x00, 0x08, 0x00, false, false, 0, false, false };
  pic_s = (PIC8259){ 0, 0, 0xFF, 0x70, 0x00, false, false, 0, false, false };
  for (int i = 0; i < 3; i++) {
    pit_count[i] = pit_reload[i] = 65536;
    pit_write_phase[i] = 0;
    pit_mode[i] = 3;
    pit_rl[i] = 3;
    pit_armed[i] = false;
    pit_latched[i] = 0;
    pit_latch_count[i] = false;
    pit_latch_status[i] = false;
    pit_read_phase[i] = 0;
  }
  pit_pending = false;
  cmos_nmi_disabled = false;
  // 串口 + 串口鼠标
  uart_rx_head = uart_rx_tail = 0; uart_irq_pending = false;
  uart_dll = uart_dlm = uart_ier = uart_fcr = 0;
  uart_lcr = uart_mcr = uart_scr = 0;
  ser_dx = ser_dy = 0; ser_buttons = ser_buttons_sent = 0;
}

// ============================================================
// 初始化 CMOS
// ============================================================
// 由 cpu_cycles + CRTC 寄存器推导"当前正在显示的扫描线"（0..199）。
// 与 0x3DA 状态口的时序算法完全一致，8088 MPH 之类靠逐行改写 0x3D9 把色度相位偏移
// 来生成伪色（1024 色模式），渲染必须按行取当时的寄存器值，这个函数把"写寄存器那一刻"
// 对应到正确的扫描行，供 vga.c 记录逐行阴影使用。
int vga_current_scanline(void) {
    uint64_t cpt = ((uint64_t)(crtc_regs[0] & 0xFF) + 1) * 8 * CPU_CLK_HZ / CGA_DOT_HZ;
    if (cpt == 0) cpt = 8 * CPU_CLK_HZ / CGA_DOT_HZ;
    uint32_t lines = ((uint32_t)(crtc_regs[4] & 0xFF) + 1)
                   * ((uint32_t)(crtc_regs[9] & 0x1F) + 1)
                   + (uint32_t)(crtc_regs[5] & 0x1F);
    if (lines == 0) lines = 262;
    uint32_t pos  = (uint32_t)(cpu_cycles % (cpt * lines));
    uint32_t line = (uint32_t)(pos / cpt);
    return (int)(line % 200);
}

// ★ 构造属性必须挂在这个函数上（之前误挂在上面的 vga_current_scanline 上，
//   导致本函数成了死代码：cmos_load / cmos_init / atexit(cmos_save) 全都没执行过，
//   客机一上电看到的 CMOS 是全 0 —— 其中 0x12 固定盘类型也是 0，硬盘自然不认）。
__attribute__((constructor))
static void io_init_constructor(void) {
  bool ok = cmos_load();
  // 掉电/失效标志：0x0D bit7 = 0 表示"电池没电、RAM 与时间无效"。真机没电就要重配，
  // 这里同理 —— 旧会话留下的镜像若带这个标志（或干脆读不出），重建一套有效配置，
  // 否则 POST 会拿 163/162 去卡引导、且固定盘类型仍是垃圾值。
  if (!ok || !(cmos_rtc[0x0D] & 0x80)) { cmos_init(); cmos_save(); }
  // ★ shutdown byte（0x0F）语义上是"上一次关机/重启的方式"，真机一上电 BIOS 就
  //   立刻把它清成 0（见 F000:00E7 的 out %al,$0x71）。它必须随会话重置：
  //   若持久化了上次的 0x06，下次启动 F000:0050 的跳转表会把 POST 直接送进中段
  //   （F000:10D8），从而跳过 F000:0693 那 120 个 IVT 向量的初始化 ——
  //   结果 INT 19h 引导向量为空，POST 走到 checkpoint 3A 后一引导就跑飞。
  cmos_rtc[0x0F] = 0x00;
  atexit(cmos_save);   // 正常退出（窗口关闭 / -console 结束）时落盘
}