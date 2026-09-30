// io.h - I/O 端口 + 定时器 + 键盘
#ifndef IO_H
#define IO_H

#include <stdint.h>
#include <stdbool.h>

uint8_t  io_read_port(uint16_t port);
void     io_write_port(uint16_t port, uint8_t val);
uint16_t io_read_port16(uint16_t port);
void     io_write_port16(uint16_t port, uint16_t val);

void io_keyboard_push(uint8_t scancode);
void io_keyboard_push_release(uint8_t scancode);
// 主机侧：键盘设备队列与输出缓冲是否都已空（-console 逐字喂键用，避免设备队列溢出丢字）
bool io_keyboard_idle(void);

// PS/2 鼠标：相对位移（GTK 坐标，y 向下为正）+ 按键（1=左 2=中 3=右）
void io_mouse_motion(int dx, int dy);
void io_mouse_button(int button, bool down);

// 键盘锁定灯状态：bit0=Caps Lock bit1=Num Lock bit2=Scroll Lock
uint8_t io_keyboard_leds(void);

void io_timer_tick(void);
void io_pit_step(uint32_t cycles);   // 传"这一步走掉多少 CPU 周期"，PIT 按 CPU_CLK_HZ/PIT_CLK_HZ 分频走
void io_timer_poll(void);
void io_keyboard_poll(void);
void io_fdc_poll(void);
void io_ide_poll(void);   // IRQ14（从片 IRQ6 → INT 76h）
void io_serial_poll(void); // IRQ4（COM1 收数据 → INT 0Ch），串口鼠标走这里

// 机器复位（GTK「重启」按钮）：键盘/PIC/PIT/PPI 回到上电默认值
void io_reset(void);

#endif