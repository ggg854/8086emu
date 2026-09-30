// fdc.h - NEC uPD765 软盘控制器（PC/XT：端口 0x3F0~0x3F7，DMA 通道 2，IRQ6）
#ifndef FDC_H
#define FDC_H

#include <stdint.h>
#include <stdbool.h>

uint8_t fdc_read_port(uint16_t port);
void    fdc_write_port(uint16_t port, uint8_t val);

// IRQ6（边沿）：由主循环轮询投递
bool fdc_irq_pending(void);
void fdc_irq_ack(void);

// 机器复位（GTK「重启」按钮）
void fdc_reset(void);

#endif