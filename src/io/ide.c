// ide.c - IDE 控制器 + 软盘镜像（纯硬件层）
#include "ide.h"
#include "cpu.h"
#include "vga.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

IDEDisk ide_disk;

// ============================================================
// IDE 控制器寄存器（ATA PIO）
// ============================================================
static uint8_t ide_error = 0;           // 0x1F1 读：错误码
static uint8_t ide_features = 0;        // 0x1F1 写：features
static uint8_t ide_sector_count = 0;    // 传输扇区数（0 表示 256）
static uint8_t ide_sector_num = 0;      // LBA0-7 / CHS 扇区号（1 起）
static uint8_t ide_cylinder_low = 0;    // LBA8-15
static uint8_t ide_cylinder_high = 0;   // LBA16-23
static uint8_t ide_drive_head = 0;      // bit4 设备 bit6 LBA 模式 LBA24-27
static uint8_t ide_status = 0;
static uint8_t ide_command = 0;

// ---- 数据阶段：512 字节暂存扇区 ----
static union { uint8_t b[IDE_SECTOR_SIZE]; uint16_t w[IDE_SECTOR_SIZE / 2]; } ide_buf;
static uint32_t ide_buf_pos = 0;
static bool     ide_drq = false;          // DRQ：可读/写数据端口
static bool     ide_buf_is_write = false; // true：主机写入（写盘命令）
static bool     ide_is_identify = false;  // 当前数据块是 IDENTIFY 响应
static uint32_t ide_cur_lba = 0;          // 正在传输的扇区
static uint16_t ide_sectors_left = 0;     // 本次命令剩余扇区数
static bool     ide_dirty = false;        // 有写盘，退出时需要落盘
static bool     ide_irq = false;          // IRQ14 待投递

#define IDE_ST_BSY   0x80
#define IDE_ST_DRDY  0x40
#define IDE_ST_DF    0x20
#define IDE_ST_DSC   0x10
#define IDE_ST_DRQ   0x08
#define IDE_ST_ERR   0x01
#define IDE_ST_READY (IDE_ST_DRDY | IDE_ST_DSC)                // 0x50
#define IDE_ST_DATA  (IDE_ST_DRDY | IDE_ST_DSC | IDE_ST_DRQ)   // 0x58

#define IDE_ERR_ABRT 0x04      // 命令被拒绝
#define IDE_ERR_IDNF 0x10      // 扇区号无效

extern bool debug_mode;

// 错误 / 成功：置状态位
static void ide_set_error(uint8_t code) {
    ide_error = code;
    ide_drq = false;
    ide_status = IDE_ST_READY | IDE_ST_ERR;
    if (debug_mode) printf("[IDE] command %02X error, error code %02X\n", ide_command, code);
}

static void ide_ok(bool irq) {
    ide_error = 0;
    ide_drq = false;
    ide_status = IDE_ST_READY;
    if (irq) ide_irq = true;
}

// 本次传输扇区数（寄存器值 0 = 256）
static uint16_t ide_xfer_count(void) {
    return ide_sector_count ? ide_sector_count : 256;
}

// 由寄存器解析起始 LBA（LBA / CHS 两种寻址）
static uint32_t ide_lba_from_regs(void) {
    if (ide_drive_head & 0x40) {           // LBA 模式（bit6=1）
        return ((uint32_t)(ide_drive_head & 0x0F) << 24)
             | ((uint32_t)ide_cylinder_high << 16)
             | ((uint32_t)ide_cylinder_low << 8)
             | (uint32_t)ide_sector_num;
    }
    // CHS 模式
    uint32_t cyl  = ((uint32_t)ide_cylinder_high << 8) | ide_cylinder_low;
    uint32_t head = ide_drive_head & 0x0F;
    uint32_t sect = ide_sector_num;        // 1..SPT
    if (sect == 0 || sect > IDE_SPT) return 0xFFFFFFFF;
    return (cyl * IDE_HEADS + head) * IDE_SPT + (sect - 1);
}

// 载入下一扇区到暂存缓冲（读命令）
static bool ide_load_sector(uint32_t lba) {
    if (lba >= IDE_SECTORS) return false;
    memcpy(ide_buf.b, ide_disk.data + (size_t)lba * IDE_SECTOR_SIZE, IDE_SECTOR_SIZE);
    ide_buf_pos = 0;
    return true;
}

// 把暂存缓冲写回镜像（写命令）
static void ide_store_sector(uint32_t lba) {
    if (lba >= IDE_SECTORS) return;
    memcpy(ide_disk.data + (size_t)lba * IDE_SECTOR_SIZE, ide_buf.b, IDE_SECTOR_SIZE);
    ide_dirty = true;
}

// 启动读 / 写数据阶段
static void ide_start_transfer(bool is_write) {
    uint32_t lba = ide_lba_from_regs();
    if (!ide_disk.present || lba == 0xFFFFFFFF || lba >= IDE_SECTORS) {
        ide_set_error(IDE_ERR_IDNF);
        return;
    }
    ide_cur_lba      = lba;
    ide_sectors_left = ide_xfer_count();
    ide_buf_is_write = is_write;
    ide_is_identify  = false;
    ide_error        = 0;
    if (is_write) {
        memset(ide_buf.b, 0, IDE_SECTOR_SIZE);
        ide_buf_pos = 0;
    } else if (!ide_load_sector(lba)) {
        ide_set_error(IDE_ERR_IDNF);
        return;
    }
    ide_drq = true;
    ide_status = IDE_ST_DATA;
}

// IDENTIFY DEVICE（0xEC）响应：256 个字
static void ide_fill_identify(void) {
    memset(ide_buf.b, 0, IDE_SECTOR_SIZE);
    ide_buf.w[0]  = 0x0040;                 // bit6=1：固定盘
    ide_buf.w[1]  = IDE_CYLINDERS;          // 柱面数
    ide_buf.w[3]  = IDE_HEADS;              // 磁头数
    ide_buf.w[5]  = IDE_SECTOR_SIZE;        // 每扇区字节数
    ide_buf.w[6]  = IDE_SPT;                // 每道扇区数
    memcpy(ide_buf.b + 20, "DOS86-IDE-0001      ", 20);            // 字 10-19：序列号
    memcpy(ide_buf.b + 46, "1.0     ", 8);                         // 字 23-26：固件版本
    memset(ide_buf.b + 54, ' ', 40);                               // 字 27-46：型号（40 字节，空格填充）
    memcpy(ide_buf.b + 54, "DOS86 VIRTUAL 64MB HDD", 22);
    ide_buf.w[49] = 0x0200;                 // bit9：支持 LBA
    ide_buf.w[51] = 0x0200;                 // PIO 传输周期模式 2
    ide_buf.w[53] = 0x0001;                 // 字 54-58 有效
    ide_buf.w[54] = IDE_CYLINDERS;
    ide_buf.w[55] = IDE_HEADS;
    ide_buf.w[56] = IDE_SPT;
    uint32_t chs_cap = (uint32_t)IDE_CYLINDERS * IDE_HEADS * IDE_SPT;
    ide_buf.w[57] = (uint16_t)(chs_cap & 0xFFFF);          // 当前容量（CHS，低）
    ide_buf.w[58] = (uint16_t)(chs_cap >> 16);             // 当前容量（CHS，高）
    ide_buf.w[60] = (uint16_t)(IDE_SECTORS & 0xFFFF);      // LBA 容量（低）
    ide_buf.w[61] = (uint16_t)(IDE_SECTORS >> 16);         // LBA 容量（高）
    ide_buf.w[100] = ide_buf.w[60];                        // 48 位容量低 16 位
    ide_buf.w[101] = ide_buf.w[61];
}

// 执行一条命令（写 0x1F7）
static void ide_execute_command(uint8_t cmd) {
    ide_command = cmd;
    ide_irq = false;
    if (!ide_disk.present || (ide_drive_head & 0x10)) {   // 无盘 / 从盘不存在
        ide_set_error(IDE_ERR_ABRT);
        return;
    }
    switch (cmd) {
        case 0xEC:                          // IDENTIFY DEVICE（不产生中断）
            ide_fill_identify();
            ide_is_identify  = true;
            ide_buf_is_write = false;
            ide_buf_pos      = 0;
            ide_error        = 0;
            ide_drq          = true;
            ide_status       = IDE_ST_DATA;
            break;
        case 0x20: case 0x21:               // READ SECTOR(S)（带/不带重试）
            ide_start_transfer(false);
            break;
        case 0x30: case 0x31:               // WRITE SECTOR(S)（带/不带重试）
            ide_start_transfer(true);
            break;
        case 0x40: case 0x41: {             // READ VERIFY SECTOR(S)
            uint32_t lba = ide_lba_from_regs();
            if (lba == 0xFFFFFFFF || lba + ide_xfer_count() > IDE_SECTORS)
                ide_set_error(IDE_ERR_IDNF);
            else
                ide_ok(true);
            break;
        }
        case 0x10:                          // RECALIBRATE
        case 0x70:                          // SEEK
        case 0x91:                          // INITIALIZE DEVICE PARAMETERS
        case 0xEF:                          // SET FEATURES
        case 0xE0:                          // STANDBY IMMEDIATE
        case 0xE1:                          // IDLE IMMEDIATE
        case 0xE7:                          // FLUSH CACHE
            ide_ok(true);
            break;
        case 0x90:                          // EXECUTE DEVICE DIAGNOSTIC
            ide_error  = 0x01;              // bit0=1：主盘自检通过
            ide_status = IDE_ST_READY;
            ide_irq    = true;
            break;
        case 0x08:                          // DEVICE RESET
            ide_ok(false);
            break;
        default:                            // 未实现 → ABORT
            ide_set_error(IDE_ERR_ABRT);
            break;
    }
}

bool ide_irq_pending(void) { return ide_irq; }
void ide_irq_ack(void)     { ide_irq = false; }

// ============================================================
// 软盘镜像（A: = 0，B: = 1）
// ============================================================
static uint8_t* floppy_data[FLOPPY_DRIVES] = { NULL, NULL };
static uint32_t floppy_size[FLOPPY_DRIVES] = { 0, 0 };
static bool     floppy_dirty[FLOPPY_DRIVES] = { false, false }; // 有写盘，退出时需要落盘
static char     floppy_name[FLOPPY_DRIVES][256];                // 挂载时的镜像文件名（空白盘为空串）

// FDC 每次写盘/格式化后调用：标记对应软盘需要写回镜像文件
void floppy_mark_dirty(int drive) {
    if (drive < 0 || drive >= FLOPPY_DRIVES) return;
    floppy_dirty[drive] = true;
}

// ============================================================
// 初始化
// ============================================================
void ide_init(void) {
    // ---- 硬盘 (dos.img) ----
    ide_disk.size = IDE_DISK_SIZE;
    ide_disk.present = false;
    strcpy(ide_disk.filename, "dos.img");

    FILE* fp = fopen(ide_disk.filename, "rb");
    if (fp) {
        ide_disk.data = malloc(IDE_DISK_SIZE);
        size_t rd = fread(ide_disk.data, 1, IDE_DISK_SIZE, fp);
        fclose(fp);
        if (rd < IDE_DISK_SIZE) {
            memset(ide_disk.data + rd, 0, IDE_DISK_SIZE - rd);
        }
        ide_disk.present = true;
        printf("IDE: Mounted %s (%d MB)\n",
               ide_disk.filename, IDE_DISK_SIZE / 1024 / 1024);
    } else {
        ide_disk.data = calloc(IDE_DISK_SIZE, 1);
        ide_disk.present = true;
        fp = fopen(ide_disk.filename, "wb");
        if (fp) {
            fwrite(ide_disk.data, 1, IDE_DISK_SIZE, fp);
            fclose(fp);
            printf("IDE: Created blank %s (%d MB)\n",
                   ide_disk.filename, IDE_DISK_SIZE / 1024 / 1024);
        } else {
            printf("IDE: Failed to create %s\n", ide_disk.filename);
            ide_disk.present = false;
        }
    }

    ide_status = 0x50;

    // ---- 软盘：默认空白（两台都先挂上），等 dos86.c 调 ide_mount_floppy ----
    for (int d = 0; d < FLOPPY_DRIVES; d++) {
        floppy_size[d] = FLOPPY_SIZE;
        floppy_data[d] = calloc(floppy_size[d], 1);
    }
    printf("IDE: Blank floppy mounted in A: and B: (1.44MB)\n");
}

// ============================================================
// 挂载软盘镜像（drive：0=A:，1=B:）
// ============================================================
void ide_mount_floppy(int drive, const char* filename) {
    if (drive < 0 || drive >= FLOPPY_DRIVES) return;
    floppy_dirty[drive] = false;
    floppy_name[drive][0] = 0;
    if (floppy_data[drive]) {
        free(floppy_data[drive]);
        floppy_data[drive] = NULL;
    }
    if (!filename) {
        // 无盘：config 里该驱动器为空，或运行期"弹出"。CMOS 据此不报该软驱存在，
        // 避免 POST 去测并不存在的驱动器而报 601（B:=4 空白盘误报）。
        floppy_size[drive] = 0;
        floppy_data[drive] = NULL;
        printf("IDE: No floppy in %c:\n", 'A' + drive);
        return;
    }
    FILE* ff = fopen(filename, "rb");
    if (!ff) {
        fprintf(stderr, "IDE: cannot open floppy %s\n", filename);
        floppy_size[drive] = FLOPPY_SIZE;
        floppy_data[drive] = calloc(floppy_size[drive], 1);
        return;
    }
    fseek(ff, 0, SEEK_END);
    long sz = ftell(ff);
    fseek(ff, 0, SEEK_SET);
    if (sz <= 0) {
        fclose(ff);
        floppy_size[drive] = FLOPPY_SIZE;
        floppy_data[drive] = calloc(floppy_size[drive], 1);
        return;
    }
    if (sz > (long)FLOPPY_SIZE) sz = FLOPPY_SIZE;
    floppy_size[drive] = (uint32_t)sz;
    floppy_data[drive] = calloc(floppy_size[drive], 1);
    size_t rd = fread(floppy_data[drive], 1, floppy_size[drive], ff);
    fclose(ff);
    if (rd < floppy_size[drive]) memset(floppy_data[drive] + rd, 0, floppy_size[drive] - rd);
    // 记住文件名：退出时若有写盘要写回这个文件（否则改动只留在内存里，重启就没了）
    strncpy(floppy_name[drive], filename, sizeof(floppy_name[drive]) - 1);
    floppy_name[drive][sizeof(floppy_name[drive]) - 1] = 0;
    printf("IDE: Mounted floppy %c: %s (%u bytes)\n", 'A' + drive, filename, floppy_size[drive]);
}

// ============================================================
// 挂载硬盘镜像
// ============================================================
void ide_mount_disk(const char* filename) {
    if (ide_disk.data) {
        free(ide_disk.data);
        ide_disk.data = NULL;
    }
    if (!filename) {
        ide_disk.data = calloc(IDE_DISK_SIZE, 1);
        ide_disk.size = IDE_DISK_SIZE;
        ide_disk.present = true;
        strcpy(ide_disk.filename, "dos.img");
        return;
    }
    FILE* fp = fopen(filename, "rb");
    if (!fp) {
        fprintf(stderr, "IDE: cannot open disk %s\n", filename);
        ide_disk.data = calloc(IDE_DISK_SIZE, 1);
        ide_disk.size = IDE_DISK_SIZE;
        ide_disk.present = true;
        return;
    }
    fseek(fp, 0, SEEK_END);
    long sz = ftell(fp);
    fseek(fp, 0, SEEK_SET);
    if (sz > (long)IDE_DISK_SIZE) sz = IDE_DISK_SIZE;
    ide_disk.data = calloc(IDE_DISK_SIZE, 1);
    ide_disk.size = IDE_DISK_SIZE;
    if (sz > 0) {
        size_t rd = fread(ide_disk.data, 1, sz, fp);
        (void)rd;
    }
    fclose(fp);
    ide_disk.present = true;
    strncpy(ide_disk.filename, filename, sizeof(ide_disk.filename) - 1);
    ide_disk.filename[sizeof(ide_disk.filename) - 1] = 0;
    printf("IDE: Mounted disk %s (%ld bytes)\n", filename, sz);
}

// ============================================================
// 清理：写回硬盘
// ============================================================
// 把硬盘/软盘的改动写回镜像文件（不清缓冲，所以「重启」按钮也能调用）
void ide_flush(void) {
    if (ide_disk.present && ide_disk.data) {
        if (ide_dirty) {
            printf("IDE: Flushing %u bytes to %s ...\n",
                   (unsigned)IDE_DISK_SIZE, ide_disk.filename);
            FILE* fp = fopen(ide_disk.filename, "wb");
            if (fp) {
                fwrite(ide_disk.data, 1, IDE_DISK_SIZE, fp);
                fclose(fp);
                printf("IDE: Flushed OK\n");
                ide_dirty = false;
            } else {
                printf("IDE: ERROR: cannot write %s\n", ide_disk.filename);
            }
        }
    }
    for (int d = 0; d < FLOPPY_DRIVES; d++) {
        if (!floppy_data[d] || !floppy_dirty[d]) continue;
        // 软盘写盘也要落盘，否则 DOS 在 A:/B: 上的改动一重启就丢了
        if (floppy_name[d][0]) {
            printf("IDE: Flushing floppy %c: to %s (%u bytes) ...\n",
                   'A' + d, floppy_name[d], floppy_size[d]);
            FILE* ff = fopen(floppy_name[d], "wb");
            if (ff) {
                fwrite(floppy_data[d], 1, floppy_size[d], ff);
                fclose(ff);
                printf("IDE: Flushed OK\n");
                floppy_dirty[d] = false;
            } else {
                printf("IDE: ERROR: cannot write %s\n", floppy_name[d]);
            }
        } else {
            printf("IDE: floppy %c: is blank memory disk, changes not saved\n", 'A' + d);
            floppy_dirty[d] = false;
        }
    }
}

void ide_cleanup(void) {
    ide_flush();
    if (ide_disk.data) {
        free(ide_disk.data);
        ide_disk.data = NULL;
        ide_disk.present = false;
    }
    for (int d = 0; d < FLOPPY_DRIVES; d++) {
        if (floppy_data[d]) {
            free(floppy_data[d]);
            floppy_data[d] = NULL;
        }
        floppy_dirty[d] = false;
    }
}

// ============================================================
// IDE 端口读（0x1F0~0x1F7）
// ============================================================
uint8_t ide_read_port(uint16_t port) {
    switch (port) {
    case 0x1F0: {
		if (!ide_drq) return 0;
		uint8_t v = ide_buf.b[ide_buf_pos++];
		vga_led_activity(0);
		if (ide_buf_pos >= IDE_SECTOR_SIZE) {
			ide_buf_pos = 0;
			if (ide_is_identify) {
				ide_is_identify = false;
				ide_drq = false;
				ide_status = IDE_ST_READY;
			} else {
				if (ide_sectors_left > 0) ide_sectors_left--;
				if (ide_sectors_left == 0) {
					ide_ok(true);
				} else {
					ide_cur_lba++;
					if (!ide_load_sector(ide_cur_lba)) {
						ide_set_error(IDE_ERR_IDNF);
						return v;
					}
					ide_drq = true;               // ★ 加这行
					ide_status = IDE_ST_DATA;
				}
			}
		}
		return v;
	}
        case 0x1F1: return ide_error;
        case 0x1F2: return ide_sector_count;
        case 0x1F3: return ide_sector_num;
        case 0x1F4: return ide_cylinder_low;
        case 0x1F5: return ide_cylinder_high;
        case 0x1F6: return ide_drive_head;
        case 0x1F7: return ide_status;
        default: return 0;
    }
}

// ============================================================
// IDE 端口写
// ============================================================
void ide_write_port(uint16_t port, uint8_t val) {
    switch (port) {
    case 0x1F0: {
		if (!ide_drq || !ide_buf_is_write) break;
		ide_buf.b[ide_buf_pos++] = val;
		vga_led_activity(0);
		if (ide_buf_pos >= IDE_SECTOR_SIZE) {
			ide_buf_pos = 0;
			ide_store_sector(ide_cur_lba);
			if (ide_sectors_left > 0) ide_sectors_left--;
			if (ide_sectors_left == 0) {
				ide_ok(true);
			} else {
				ide_cur_lba++;
				memset(ide_buf.b, 0, IDE_SECTOR_SIZE);
				ide_drq = true;               // ★ 加这行
				ide_status = IDE_ST_DATA;
			}
		}
		break;
	}
        case 0x1F1: ide_features = val; break;
        case 0x1F2: ide_sector_count = val; break;
        case 0x1F3: ide_sector_num = val; break;
        case 0x1F4: ide_cylinder_low = val; break;
        case 0x1F5: ide_cylinder_high = val; break;
        case 0x1F6: ide_drive_head = val; break;
        case 0x1F7: ide_execute_command(val); break;
        default: break;
    }
}

uint16_t ide_read_port16(uint16_t port) {
    uint16_t lo = ide_read_port(port);
    return lo | ((uint16_t)ide_read_port(port) << 8);
}

void ide_write_port16(uint16_t port, uint16_t val) {
    for (int i = 0; i < 2; i++) ide_write_port(port, (uint8_t)(val >> (i * 8)));
}

// ============================================================
// 供 FDC 访问软盘数据（drive：0=A:，1=B:）
// ============================================================
uint8_t* floppy_get_data(int drive) {
    if (drive < 0 || drive >= FLOPPY_DRIVES) return NULL;
    return floppy_data[drive];
}

uint32_t floppy_get_size(int drive) {
    if (drive < 0 || drive >= FLOPPY_DRIVES) return 0;
    return floppy_size[drive];
}

// ============================================================
// 内置硬盘 BIOS：INT 13h 硬盘服务
// ------------------------------------------------------------
// 本机 BIOS（Turbo XT BIOS）本身不带硬盘代码 —— 真机上硬盘的 INT 13h 服务由
// 硬盘卡上的选件 ROM 提供（模拟器不扫描选件 ROM）。这里改用"向量桩"方案：
//   · 执行到 INT 13h 指令、且向量仍指向 ROM 时，把 BIOS 原向量存起来，改写成一个
//     13 字节的小桩（放在 C800:0000 的硬盘卡 ROM 窗口，见下面的 ★ 说明）；
//   · 桩判断 DL：DL>=80h（硬盘）→ INT 0F1h，由下面的 C 代码处理；
//     DL<80h（软盘）→ 远跳回 BIOS 原处理程序，软盘行为完全不变；
//   · 无论 INT 13h 指令调用，还是 DOS 的 IO.SYS 挂上自己的 INT 13h 后经向量
//     转发（pushf + far call）过来的调用，都会命中这个桩。
// 桩代码：F6 C2 80  test dl,80h / 74 03  jz +3 / CD F1  int 0F1h / CF  iret
//         EA o o s s  jmp far 原向量
//
// ★ 桩和 INT 41h 参数表必须待在"客机可写"的段里，所以用 C800 而不是 F000：
//   PC-DOS 3.x 会把自己的 INT 13h 处理程序搬进 INT 41h 参数表所在的那个段
//   （真机 XT 上参数表在硬盘卡选件 ROM 的窗口 C800 段里，那块是 RAM），偏移 +1B00。
//   参数表若指向 F000（BIOS ROM 区，本模拟器 ≥0xF0000 写保护），DOS 处理程序搬
//   过去写不进去，FDISK 读 MBR 就落到 F000:1B20 的 ROM 垃圾上 → 报
//   「All 4 fixed disk partitions are allocated」。C800:0000 在本模拟器里是普通 RAM。
// ============================================================
#define INT13_STUB_SEG  0xC800
#define INT13_STUB_OFF  0x0000
#define INT41_TABLE_OFF 0x0020

static bool int13_stub_ready = false;

// 每次执行到 INT 13h 指令时调用：向量又回到 ROM 就（重新）装桩
void ide_int13_prepare(void) {
    // AT（286）BIOS 自带完整的 INT 13h：软盘处理程序在 F000，硬盘走 IDE 端口（由
    // ide.c 模拟）。这里不能装桩——桩固定在 C800:0000，而 AT 自带硬盘 BIOS 也会
    // 往 C800:0000 拷贝代码，二者冲突：后者覆盖桩的远跳目标字节，使软盘引导经桩
    // jmp 到垃圾地址（0883:674A）→ IRET 弹回 0:0 → 跑飞（Fly 0:0）。
    // 桩只服务于没有硬盘 BIOS 的 XT；AT 直接让 BIOS 原生处理 INT 13h 即可。
    if (cpu_is_286)
        return;

    uint16_t off = read_word(0, 0x13 * 4);
    uint16_t seg = read_word(0, 0x13 * 4 + 2);
    bool vector_is_stub = (seg == INT13_STUB_SEG && off == INT13_STUB_OFF);
    if (!vector_is_stub && seg < 0xF000) return;                  // 已被 DOS 接管，别动

    uint8_t* p = memory + ((uint32_t)INT13_STUB_SEG << 4) + INT13_STUB_OFF;
    // 桩和 INT 41h 参数表都在 RAM 段里，随时可能被客机覆盖 → 每次都校验内容，缺了就补
    bool installed = vector_is_stub && int13_stub_ready &&
                     p[0] == 0xF6 && p[1] == 0xC2 && p[2] == 0x80 &&
                     p[5] == 0xCD && p[6] == 0xF1 && p[7] == 0xCF &&
                     read_word(0, 0x41 * 4) == INT41_TABLE_OFF &&
                     read_word(0, 0x41 * 4 + 2) == INT13_STUB_SEG;
    if (!installed) {
        p[0] = 0xF6; p[1] = 0xC2; p[2] = 0x80;   // test dl, 80h
        p[3] = 0x74; p[4] = 0x03;                // jz +3 → 偏移 8
        p[5] = 0xCD; p[6] = 0xF1;                // int 0F1h
        p[7] = 0xCF;                             // iret
        p[8] = 0xEA;                             // jmp far 原向量（偏移 9 起填）
        // INT 41h 硬盘参数表（老 DOS 在没有 INT 13h AH=08 支持时用它）
        uint8_t* t = p + INT41_TABLE_OFF;
        t[0] = (uint8_t)(IDE_CYLINDERS & 0xFF);
        t[1] = (uint8_t)(IDE_CYLINDERS >> 8);
        t[2] = IDE_HEADS;
        t[3] = 0; t[4] = 0;                      // 减小写电流起始柱面（0=从第 0 柱面起）
        t[5] = 0xFF; t[6] = 0xFF;                // 预补偿起始柱面（FFFF=不做预补偿）
        t[7] = 0x0B;                             // 最大 ECC 突发长度（常见值 0x0B）
        t[8] = 0;                                // 控制字节
        t[9] = 0; t[10] = 0; t[11] = 0;          // 标准超时
        t[12] = 0; t[13] = 0;                    // 格式化超时
        t[14] = 0; t[15] = 0;                    // 校验超时
        // 让 DOS/BIOS 知道有 1 台硬盘（BDA 40:75）
        write_byte(0x40, 0x75, 1);
        write_byte(0x40, 0x74, 0);
        if (debug_mode)
            printf("[IDE] INT 41h old %04X:%04X -> forced to this table %04X:%04X\n",
                   read_word(0, 0x41 * 4 + 2), read_word(0, 0x41 * 4),
                   INT13_STUB_SEG, INT41_TABLE_OFF);
        write_word(0, 0x41 * 4, INT41_TABLE_OFF);
        write_word(0, 0x41 * 4 + 2, INT13_STUB_SEG);
        int13_stub_ready = true;
    }
    // 记录 BIOS 原向量（填进桩的远跳转），再把 INT 13h 指向桩。
    // 注意：只有向量本来指向别处（BIOS ROM）时才填，否则会把远跳转指回自己 → 死循环。
    if (!vector_is_stub) {
        p[9]  = (uint8_t)(off & 0xFF);
        p[10] = (uint8_t)(off >> 8);
        p[11] = (uint8_t)(seg & 0xFF);
        p[12] = (uint8_t)(seg >> 8);
        if (debug_mode) printf("[IDE] INT 13h stub installed, original vector %04X:%04X\n", seg, off);
    }
    write_word(0, 0x13 * 4, INT13_STUB_OFF);
    write_word(0, 0x13 * 4 + 2, INT13_STUB_SEG);
}

// INT 13h 的 CHS → LBA（CH=柱面低8位，CL 6-7=柱面高2位，CL 0-5=扇区，DH=磁头）
static uint32_t int13_chs_to_lba(uint8_t ch, uint8_t cl, uint8_t dh) {
    uint32_t cyl  = ((uint32_t)(cl & 0xC0) << 2) | ch;
    uint32_t sect = cl & 0x3F;
    uint32_t head = dh;
    if (sect == 0 || sect > IDE_SPT)   return 0xFFFFFFFF;
    if (head >= IDE_HEADS)             return 0xFFFFFFFF;
    if (cyl >= IDE_CYLINDERS)          return 0xFFFFFFFF;
    return (cyl * IDE_HEADS + head) * IDE_SPT + (sect - 1);
}

// 客机内存 <-> 缓冲（每字节单独拷，天然处理 64KB 段内回绕）
static void int13_mem_out(uint16_t seg, uint16_t off, const uint8_t* src, uint32_t n) {
    uint32_t base = (uint32_t)seg << 4;
    for (uint32_t i = 0; i < n; i++)
        memory[base + (uint16_t)(off + i)] = src[i];
}

static void int13_mem_in(uint16_t seg, uint16_t off, uint8_t* dst, uint32_t n) {
    uint32_t base = (uint32_t)seg << 4;
    for (uint32_t i = 0; i < n; i++)
        dst[i] = memory[base + (uint16_t)(off + i)];
}

// 返回：AH=0 成功且 CF=0；否则 AH=错误码且 CF=1
static void int13_fail(uint8_t err) {
    set_flag(FLAG_CF);
    cpu.ax = (uint16_t)((cpu.ax & 0x00FF) | ((uint16_t)err << 8));
}

// INT 0F1h：硬盘 INT 13h 服务（DL >= 80h）
void ide_int13_handle(void) {
    uint8_t ah = (uint8_t)(cpu.ax >> 8);
    uint8_t al = (uint8_t)(cpu.ax & 0xFF);
    uint8_t dl = (uint8_t)(cpu.dx & 0xFF);
    uint8_t dh = (uint8_t)(cpu.dx >> 8);
    uint8_t cl = (uint8_t)(cpu.cx & 0xFF);
    uint8_t ch = (uint8_t)(cpu.cx >> 8);

    if (debug_mode)
        printf("[IDE] INT13 AH=%02X DL=%02X AL=%02X CH=%02X CL=%02X DH=%02X ES:BX=%04X:%04X from=%04X:%04X\n",
               ah, dl, al, ch, cl, dh, cpu.es, cpu.bx,
               read_word(cpu.ss, (uint16_t)(cpu.sp + 2)), read_word(cpu.ss, cpu.sp));

    clear_flag(FLAG_CF);
    if (dl < 0x80) {                    // 软盘不该走到这里（桩已按 DL 分流）
        int13_fail(0x01);
        return;
    }
    if (dl != 0x80 || !ide_disk.present) {
        // 只模拟 0x80 这一台盘。POST 会用 AH=10h 探测 0x81（第二块盘）是否存在，
        //   此时必须返回「驱动器未就绪」(AH=80h) 而不是「非法功能」(01h) ——
        //   后者会让 POST 误判成控制器故障，进而走错误报告/复位路径。
        int13_fail(0x80);
        return;
    }

    switch (ah) {
        case 0x00:                       // 复位磁盘系统
            ide_drq = false; ide_irq = false; ide_status = IDE_ST_READY;
            cpu.ax = 0x0000;
            break;

        case 0x01:                       // 取上次操作状态
            cpu.ax = (uint16_t)(cpu.ax & 0x00FF);   // AH=0
            break;

        case 0x10:                       // 测试驱动器就绪（POST 探测硬盘是否存在）
            cpu.ax = (uint16_t)(cpu.ax & 0x00FF);   // AH=0：就绪
            break;

        case 0x02:                       // 读扇区（CHS）
        case 0x04: {                     // 校验扇区
            uint32_t lba = int13_chs_to_lba(ch, cl, dh);
            if (al == 0 || lba == 0xFFFFFFFF || lba + al > IDE_SECTORS) {
                int13_fail(0x04);        // 找不到扇区
                return;
            }
            if (ah == 0x02) {
                for (uint8_t i = 0; i < al; i++)
                    int13_mem_out(cpu.es, (uint16_t)(cpu.bx + (uint16_t)i * IDE_SECTOR_SIZE),
                                  ide_disk.data + (size_t)(lba + i) * IDE_SECTOR_SIZE,
                                  IDE_SECTOR_SIZE);
            }
            if (debug_mode)
                printf("[IDE] INT13 read LBA=%u -> %04X:%04X first 4 bytes=%02X %02X %02X %02X\n",
                       lba, cpu.es, cpu.bx,
                       ide_disk.data[(size_t)lba * IDE_SECTOR_SIZE + 0],
                       ide_disk.data[(size_t)lba * IDE_SECTOR_SIZE + 1],
                       ide_disk.data[(size_t)lba * IDE_SECTOR_SIZE + 2],
                       ide_disk.data[(size_t)lba * IDE_SECTOR_SIZE + 3]);
            vga_led_activity(0);
            cpu.ax = (uint16_t)((cpu.ax & 0x00FF) | (0x00 << 8));
            break;
        }

        case 0x03: {                     // 写扇区（CHS）
            uint32_t lba = int13_chs_to_lba(ch, cl, dh);
            if (al == 0 || lba == 0xFFFFFFFF || lba + al > IDE_SECTORS) {
                int13_fail(0x04);
                return;
            }
            for (uint8_t i = 0; i < al; i++)
                int13_mem_in(cpu.es, (uint16_t)(cpu.bx + (uint16_t)i * IDE_SECTOR_SIZE),
                             ide_disk.data + (size_t)(lba + i) * IDE_SECTOR_SIZE,
                             IDE_SECTOR_SIZE);
            ide_dirty = true;
            vga_led_activity(0);
            cpu.ax = (uint16_t)((cpu.ax & 0x00FF) | (0x00 << 8));
            break;
        }

        case 0x05:                       // 格式化磁道（DOS 不靠它低格硬盘，直接报成功）
        case 0x06: case 0x07: case 0x09: case 0x0A: case 0x0B: case 0x0C: case 0x0D:
            cpu.ax = (uint16_t)((cpu.ax & 0x00FF) | (0x00 << 8));
            break;

        case 0x08: {                     // 取驱动器参数
            uint16_t last_cyl = IDE_CYLINDERS - 2;
            uint8_t  spt      = (uint8_t)(IDE_SPT & 0x3F);
            uint8_t  cyl_hi   = (uint8_t)((last_cyl >> 8) & 0x03);
            cpu.cx = (uint16_t)(((uint16_t)cyl_hi << 6) | spt);          // CL
            cpu.cx = (uint16_t)((cpu.cx & 0x00FF) | ((last_cyl & 0xFF) << 8)); // CH
            cpu.dx = (uint16_t)(((uint16_t)(IDE_HEADS - 1) << 8) | 1);   // DH=最大磁头, DL=固定盘台数(只模拟1台)
            // 注意：这里刻意不动 ES:DI。真机 XT 硬盘 BIOS 会返回 ES:DI=DPT 指针，
            // 但 DOS 的 INT 13h 处理程序（0070:09E1）不保存 ES，于是 FDISK 拿到的 ES
            // 被改成 DPT 段（F000），之后读 MBR 就落到 F000:1B20（ROM 区）→ 分区表读
            // 不进自己的缓冲，报「All 4 fixed disk partitions are allocated」。
            // DOS 自身用 INT 41h 找 DPT，不依赖这个返回值，所以不返回是安全的。
            cpu.ax = 0x0000;
            if (debug_mode)
                printf("[IDE] INT13 AH=08 return CX=%04X DX=%04X\n", cpu.cx, cpu.dx);
            break;
        }

        case 0x15:                       // 取磁盘类型
            cpu.ax = (uint16_t)((cpu.ax & 0x00FF) | (0x03 << 8));        // AH=3：硬盘
            cpu.cx = (uint16_t)(IDE_SECTORS >> 16);                       // CX:DX = 扇区总数（CX 高位）
            cpu.dx = (uint16_t)(IDE_SECTORS & 0xFFFF);
            break;

        default:                         // 含 0x41/0x48 扩展功能：本模拟器不支持
            int13_fail(0x01);
            break;
    }
}

// ============================================================
// 机器复位（GTK「重启」按钮）：控制器寄存器回到上电值。
// 不碰 ide_dirty / ide_disk.data —— 待落盘的改动要留着，软盘缓冲同样保留。
// ============================================================
void ide_reset(void) {
    ide_error = 0; ide_features = 0; ide_sector_count = 0;
    ide_sector_num = 0; ide_cylinder_low = 0; ide_cylinder_high = 0;
    ide_drive_head = 0; ide_status = IDE_ST_READY; ide_command = 0;
    ide_buf_pos = 0; ide_drq = false; ide_buf_is_write = false;
    ide_is_identify = false; ide_cur_lba = 0; ide_sectors_left = 0;
    ide_irq = false;
    int13_stub_ready = false;   // 客机内存清零后重新装桩
}

