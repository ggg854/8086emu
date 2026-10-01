// dos86.h - GTK 版
#ifndef DOS86_H
#define DOS86_H

#include "cpu.h"
#include "vga.h"
#include "ide.h"
#include <stdbool.h>
#include "config.h"

extern volatile bool emu_running;
extern bool debug_mode;
// -console：不建 GTK 窗口，把 80x25 文本屏画到终端、从 stdin 收键（类似 qemu -nographic）；
//          该模式下强制关闭 -dbg 日志
extern bool console_mode;


void init_emulator(const Config* cfg);

char* open_bios_dialog(void);
char* open_floppy_dialog(int drive);   // drive：0=A:，1=B:
char* open_disk_dialog(void);

#endif