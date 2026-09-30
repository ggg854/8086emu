// dma.h - 8237 DMA 控制器（PC/XT）
#ifndef DMA_H
#define DMA_H

#include <stdint.h>
#include <stdbool.h>

uint8_t dma_read_port(uint16_t port);
void    dma_write_port(uint16_t port, uint8_t val);

// ---- 供 FDC 等设备使用 ----
uint32_t dma_phys_addr(int ch);      // 20 位物理地址（页 << 16 | 当前地址）
uint32_t dma_byte_count(int ch);     // 剩余字节数（计数寄存器 + 1）
void     dma_advance(int ch, uint32_t n);   // 传输 n 字节后推进地址/计数
bool     dma_channel_enabled(int ch);       // 通道未被屏蔽

// PC/XT DRAM 刷新：由 8253 通道 1 的刷新脉冲驱动（见 dma.c 说明）
void     dma_refresh_tick(void);

// 机器复位（GTK「重启」按钮）：清控制器状态并屏蔽所有通道
void     dma_reset(void);

#endif