// ide.h
#ifndef IDE_H
#define IDE_H

#include <stdint.h>
#include <stdbool.h>

#define IDE_SECTOR_SIZE 512
// 硬盘 CHS 几何：XT 时代标准「20MB 型」（615 柱面 × 4 磁头 × 17 扇区）。
// 之前用的 512×16×16 是非标准几何，XT 时代的 DOS/FDISK 会判为无效（容量显示空白、
// 报「All 4 partitions are allocated」）。415/615/4 磁头/17 扇区才是那个年代的标准。
#define IDE_CYLINDERS 615
#define IDE_HEADS     4
#define IDE_SPT       17
#define IDE_SECTORS   ((uint32_t)IDE_CYLINDERS * IDE_HEADS * IDE_SPT)   // 41820 扇区
#define IDE_DISK_SIZE (IDE_SECTORS * IDE_SECTOR_SIZE)                   // ≈20.4MB

// 软盘几何（1.44MB）
#define FLOPPY_CYLINDERS  80
#define FLOPPY_HEADS      2
#define FLOPPY_SECTORS    18
#define FLOPPY_SIZE       (FLOPPY_CYLINDERS * FLOPPY_HEADS * FLOPPY_SECTORS * IDE_SECTOR_SIZE)

// 软盘驱动器台数（A: = 0，B: = 1）
#define FLOPPY_DRIVES     2

typedef struct {
    uint8_t* data;
    uint32_t size;
    char filename[256];
    bool present;
} IDEDisk;

extern IDEDisk ide_disk;

void ide_init(void);
void ide_cleanup(void);
void ide_flush(void);          // 只写回镜像、不清缓冲（「重启」按钮用）

// 挂载（drive：0=A:，1=B:）
void ide_mount_floppy(int drive, const char* filename);
void ide_mount_disk(const char* filename);

// IDE 端口（0x1F0~0x1F7）
uint8_t  ide_read_port(uint16_t port);
void     ide_write_port(uint16_t port, uint8_t val);
uint16_t ide_read_port16(uint16_t port);
void     ide_write_port16(uint16_t port, uint16_t val);

// 供 FDC 访问软盘（drive：0=A:，1=B:；越界返回 NULL/0）
uint8_t* floppy_get_data(int drive);
uint32_t floppy_get_size(int drive);
void     floppy_mark_dirty(int drive);   // FDC 写盘/格式化后调用：退出时写回镜像

// 内置硬盘 BIOS：INT 13h 向量桩 + 硬盘服务（见 ide.c 说明）
void ide_int13_prepare(void);   // 每次执行到 INT 13h 指令时调用
void ide_int13_handle(void);    // INT 0F1h：实际处理硬盘请求

// IRQ14（从片 IRQ6 → INT 76h）
bool ide_irq_pending(void);
void ide_irq_ack(void);

// 机器复位（GTK「重启」按钮）
void ide_reset(void);

#endif