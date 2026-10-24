// dma.c - 8237 DMA 控制器（AT：主片 8 位 + 从片 16 位）
//   主片：端口 0x00~0x0F，页寄存器 0x87/0x83/0x81/0x82（对应通道 0~3）
//   从片：端口 0xC0~0xCF，页寄存器 0x8F/0x8B/0x89/0x8A（对应通道 4~7），
//         寄存器都是 16 位单元（地址/计数以字为单位）。
//   对外沿用主片接口（FDC 用通道 2）；从片只保证寄存器读写语义完整。
#include "dma.h"
#include <stdio.h>
#include <string.h>

extern bool debug_mode;   // 主程序 -dbg 打开

typedef struct {
  uint16_t base_addr;
  uint16_t base_count;
  uint16_t cur_addr;
  uint16_t cur_count;
  uint8_t  mode;
  bool     masked;
} DMAChan;

typedef struct {
  DMAChan ch[4];
  uint8_t page[4];
  bool    ff;        // 字节指针翻转触发器（整片共用）
  uint8_t cmd;       // 命令寄存器
  uint8_t status;    // 状态寄存器（读后清）
  bool    is16;      // 16 位控制器（从片）
} DMACtrl;

static DMACtrl dma_m = { 0 };   // 主片（8 位）
static DMACtrl dma_s = { 0 };   // 从片（16 位）

// ---- 页寄存器（0x80~0x8F）----
//   AT 用一颗 74LS612 覆盖全部 16 个页寄存器，读写皆可。接到 DMA 通道的那 8 个
//   借用主/从片的 page[]，其余"空"寄存器（0x80/0x84~0x86/0x88/0x8C~0x8E）也必须
//   能存能取：AT BIOS POST 代码 08 会把 0x81~0x8E 整片写入递增图案再逐个读回校验，
//   只接 8 个真寄存器时会读空，比较失败 → F000:0322 停机。
static uint8_t dma_page_spare[16];

static uint8_t* dma_page_ptr(uint16_t port) {
  switch (port) {
    case 0x87: return &dma_m.page[0];
    case 0x83: return &dma_m.page[1];
    case 0x81: return &dma_m.page[2];
    case 0x82: return &dma_m.page[3];
    case 0x8F: return &dma_s.page[0];
    case 0x8B: return &dma_s.page[1];
    case 0x89: return &dma_s.page[2];
    case 0x8A: return &dma_s.page[3];
    default:   return &dma_page_spare[port & 0x0F];
  }
}

// ---- 端口 → 控制器映射（返回 NULL 表示该端口不是 DMA 端口）----
static DMACtrl* port_ctrl(uint16_t port) {
  if (port <= 0x0F) return &dma_m;
  if (port >= 0xC0 && port <= 0xDF) return &dma_s;
  return NULL;
}

// 主复位（写 0x0D / 0xCD）：清触发器/命令/状态，屏蔽所有通道
static void dma_master_clear(DMACtrl* c) {
  for (int i = 0; i < 4; i++) {
    c->ch[i].base_addr = c->ch[i].cur_addr = 0;
    c->ch[i].base_count = c->ch[i].cur_count = 0;
    c->ch[i].mode = 0;
    c->ch[i].masked = true;
  }
  c->ff = false;
  c->cmd = 0;
  c->status = 0;
}

// 从片寄存器偏移：从片是 16 位控制器，A0 不参与译码，寄存器按"字"对齐，
// 内部偏移 = (端口 - 0xC0) / 2。于是 0xC0/0xC1 → 偏移 0（CH4 地址）、
// 0xC2 → 偏移 1（CH4 计数）…… 0xCE → 偏移 7（CH7 计数）、0xDA → 偏移 13（从片主复位）。
// AT BIOS POST 正是这样测的：主片 inc dx 扫 0x00~0x07，从片 add dx,2 扫 0xC0~0xCE，
// 且测试前先 out 0xDA（从片主复位）——与主片的 out 0x0D 对称。
static uint16_t ctrl_reg_off(uint16_t port) { return (uint16_t)((port - 0xC0) >> 1); }

// ============================================================
// 读端口（只有主片的地址/计数寄存器对 CPU 可读，从片同理）
// ============================================================
uint8_t dma_read_port(uint16_t port) {
  // 页寄存器全部可回读（AT 用一颗 74LS612 覆盖 0x80~0x8F）：AT BIOS 早期 POST
  // 的测试 08 会把 0x81~0x8E 整片写入递增图案再逐个读回校验，任何一个读不回来
  // 都会在 F000:0323 停机。
  if (port >= 0x80 && port <= 0x8F) return *dma_page_ptr(port);
  DMACtrl* c = port_ctrl(port);
  if (!c) return 0xFF;

  uint16_t off = (port <= 0x0F) ? port : ctrl_reg_off(port);

  if (off <= 0x07) {
    int n = off >> 1;
    bool low = !c->ff;
    c->ff = !c->ff;
    if (off & 1) return low ? (uint8_t)(c->ch[n].cur_count & 0xFF)
                            : (uint8_t)(c->ch[n].cur_count >> 8);
    return            low ? (uint8_t)(c->ch[n].cur_addr & 0xFF)
                          : (uint8_t)(c->ch[n].cur_addr >> 8);
  }

  switch (off) {
    case 0x08: {                       // 状态寄存器（读后清）
      uint8_t v = c->status;
      c->status = 0;
      return v;
    }
    case 0x09: return 0x00;            // 请求寄存器
    case 0x0A: {                       // 屏蔽寄存器
      uint8_t v = 0;
      for (int i = 0; i < 4; i++) if (c->ch[i].masked) v |= (uint8_t)(1 << i);
      return v;
    }
    case 0x0B: return c->ch[0].mode;   // 模式（写专用，返回通道 0 的值）
    case 0x0C: return 0x00;            // 清触发器（写专用）
    case 0x0D: return 0x00;            // 主复位（写专用）→ 临时寄存器
    default:   return 0xFF;
  }
}

// ============================================================
// 写端口
// ============================================================
void dma_write_port(uint16_t port, uint8_t val) {
  if (port >= 0x80 && port <= 0x8F) { *dma_page_ptr(port) = val; return; }
  DMACtrl* c = port_ctrl(port);
  if (!c) return;

  uint16_t off = (port <= 0x0F) ? port : ctrl_reg_off(port);

  if (off <= 0x07) {
    int n = off >> 1;
    bool low = !c->ff;
    c->ff = !c->ff;
    if (off & 1) {   // 计数寄存器
      if (low) {
        c->ch[n].base_count = (uint16_t)((c->ch[n].base_count & 0xFF00) | val);
        c->ch[n].cur_count  = (uint16_t)((c->ch[n].cur_count  & 0xFF00) | val);
      } else {
        c->ch[n].base_count = (uint16_t)((c->ch[n].base_count & 0x00FF) | (val << 8));
        c->ch[n].cur_count  = (uint16_t)((c->ch[n].cur_count  & 0x00FF) | (val << 8));
      }
    } else {          // 地址寄存器
      if (low) {
        c->ch[n].base_addr = (uint16_t)((c->ch[n].base_addr & 0xFF00) | val);
        c->ch[n].cur_addr  = (uint16_t)((c->ch[n].cur_addr  & 0xFF00) | val);
      } else {
        c->ch[n].base_addr = (uint16_t)((c->ch[n].base_addr & 0x00FF) | (val << 8));
        c->ch[n].cur_addr  = (uint16_t)((c->ch[n].cur_addr  & 0x00FF) | (val << 8));
      }
    }
    return;
  }

  switch (off) {
    case 0x08: c->cmd = val; break;                     // 命令寄存器
    case 0x09: break;                                   // 请求寄存器（软请求，忽略）
    case 0x0A: {                                        // 单通道屏蔽
      int n = val & 0x03;
      c->ch[n].masked = (val & 0x04) ? true : false;
      break;
    }
    case 0x0B: c->ch[val & 0x03].mode = val; break;     // 模式寄存器
    case 0x0C: c->ff = false; break;                    // 清字节指针触发器
    case 0x0D: dma_master_clear(c); break;              // 主复位
    case 0x0E:                                          // 清所有屏蔽
      for (int i = 0; i < 4; i++) c->ch[i].masked = false;
      break;
    case 0x0F:                                          // 写所有屏蔽
      for (int i = 0; i < 4; i++) c->ch[i].masked = (val >> i) & 1;
      break;
    default: break;
  }
}

// ============================================================
// 供设备（FDC）查询/推进传输（主片 8 位通道）
// ============================================================
uint32_t dma_phys_addr(int ch) {
  ch &= 3;
  return ((uint32_t)dma_m.page[ch] << 16) | dma_m.ch[ch].cur_addr;
}

uint32_t dma_byte_count(int ch) {
  ch &= 3;
  return (uint32_t)dma_m.ch[ch].cur_count + 1;
}

void dma_advance(int ch, uint32_t n) {
  ch &= 3;
  while (n > 0) {
    if (dma_m.ch[ch].cur_count > 0) {
      dma_m.ch[ch].cur_count--;
    } else {
      dma_m.ch[ch].cur_count = 0xFFFF;
      dma_m.status |= (uint8_t)(1 << ch);   // 终端计数（EOP）
    }
    dma_m.ch[ch].cur_addr = (uint16_t)(dma_m.ch[ch].cur_addr + 1);
    n--;
  }
}

bool dma_channel_enabled(int ch) {
  return !dma_m.ch[ch & 3].masked;
}

// ============================================================
// DRAM 刷新（DMA 通道 0）
// ============================================================
// XT 上 8237 的通道 0 专职 DRAM 刷新：8253 通道 1 每 15.1µs 输出一个脉冲，
// 作为通道 0 的 DREQ，控制器内部走一个"伪传输"（verify，不真访存）——
// 地址 +1、计数 -1，计数从 0 绕回 0xFFFF 时输出 TC（EOP）。
//
// XT BIOS 上电自检就靠这个：它把通道 0 计数装成 0xFFFF 并解除屏蔽，然后跑完
// 整段内存检测；如果跑完时 DMA 状态寄存器（端口 0x08）的 bit0（通道 0 TC）
// 还没置位，就判定 DMA/主板故障 → 打印错误码 101 并停机（F000:E4DF 处检查）。
// 机器复位（GTK「重启」按钮）：等同软件的主复位
// ============================================================
void dma_reset(void) {
  dma_master_clear(&dma_m);
  dma_master_clear(&dma_s);
  for (int i = 0; i < 4; i++) { dma_m.page[i] = 0; dma_s.page[i] = 0; }
  memset(dma_page_spare, 0, sizeof(dma_page_spare));
  dma_s.is16 = true;
}
// ============================================================

// 真机上刷新通道永不停止，这里绕回即重装、忽略 autoinit。
void dma_refresh_tick(void) {
  if (dma_m.cmd & 0x04) return;      // 命令寄存器 bit2=1 → 控制器被禁用
  if (dma_m.ch[0].masked) return;    // 通道 0 被屏蔽
  if (dma_m.ch[0].cur_count > 0) {
    dma_m.ch[0].cur_count--;
  } else {
    dma_m.ch[0].cur_count = dma_m.ch[0].base_count;   // 绕回（0xFFFF 视作 65536）
    dma_m.status |= 0x01;                             // 通道 0 终端计数
  }
  dma_m.ch[0].cur_addr = (uint16_t)(dma_m.ch[0].cur_addr + 1);
}