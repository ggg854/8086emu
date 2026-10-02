// fdc.c - NEC uPD765 / Intel 82072 软盘控制器（PC/AT）
//   端口：0x3F0 SRA / 0x3F1 SRB / 0x3F2 DOR / 0x3F4 MSR / 0x3F5 DATA / 0x3F7 DIR
//   数据走 DMA 通道 2（AT BIOS 用 DMA 模式）；命令结束产生 IRQ6 → INT 0Eh
//
//   主状态寄存器 MSR (0x3F4)：
//     bit7 RQM 1=数据寄存器可传输
//     bit6 DIO 1=FDC→CPU（结果/执行相位），0=CPU→FDC（命令相位）
//     bit5 NDM 1=非DMA(程序IO) 0=DMA（本模拟器固定 DMA 模式 = 0）
//     bit4 CB  1=命令忙（命令/执行/结果相位中）
//     bit3..0  当前被访问的驱动器忙标志（按驱动器号置位）
//   相位：
//     PH_IDLE    空闲，可收命令（RQM=1）
//     PH_CMD     收命令字节（DIO=0，RQM=1，CB=1）
//     PH_RESULT  结果字节可读（DIO=1，RQM=1，CB=1）
//   读/写命令在命令相位收齐后立即执行数据搬运（DMA）并进入结果相位 + 发 IRQ6。
//   SEEK / RECALIBRATE 收齐后立即寻道并置未决中断；结果由 SENSE INTERRUPT STATUS 取走。
#include "fdc.h"
#include "dma.h"
#include "ide.h"
#include "cpu.h"
#include "vga.h"

#include <string.h>
#include <stdio.h>
#include <time.h>

extern bool debug_mode;       // 主程序 -dbg 打开
extern uint64_t insn_count;   // 主程序指令计数（读盘耗时诊断用）

// ============================================================
// 相位
// ============================================================
enum { PH_IDLE = 0, PH_CMD, PH_RESULT };

static int      phase   = PH_IDLE;

static uint8_t  cmd[16];
static int      cmd_len = 0;    // 本命令总字节数（含操作码）
static int      cmd_pos = 0;
static uint8_t  opcode  = 0;

static uint8_t  res[16];
static int      res_len = 0, res_pos = 0;

static uint8_t  dor = 0;        // 0x3F2 数字输出寄存器

// 待 SENSE INTERRUPT STATUS 取走的中断状态队列。
//   uPD765 复位后会为每台盘(0..3)产生一个"未就绪"中断(ST0=0xC0|drv)，
//   正常 recal/seek 也会各产生一个；SENSE INTERRUPT 顺序取走。
typedef struct { uint8_t st0; uint8_t pcn; } PendInt;
static PendInt int_q[8];
static int     int_qhead = 0, int_qtail = 0;

// IRQ6 边沿，等主循环轮询投递
static bool     irq_line = false;

// ★ 中断锁存（供 SENSE INTERRUPT STATUS 读取，独立于 PIC 应答）
//   真机：FDC 的 INTRQ 与内部中断锁存独立；CPU 应答 INT 0E（清 PIC IRQ）并不清除
//   FDC 锁存的状态，只有 SENSE INTERRUPT STATUS 才清。故另设锁存，避免 INT 0E
//   先被服务后 SENSE INTERRUPT 读到 0x80（无中断）→ BIOS 误判 seek/recal 失败。
static uint8_t  intr_st0 = 0, intr_pcn = 0;
static bool     intr_latched = false;

// 每台驱动器当前柱面 + SPECIFY 参数
static uint8_t  pcn[4] = {0, 0, 0, 0};
static uint8_t  specify_srt_hut = 0xDF;
static uint8_t  specify_hlt_nd  = 0x02;

// 82077 扩展参数（CONFIGURE / PERPENDICULAR / LOCK）
static bool     fdc_locked   = false;   // LOCK：禁止 SPECIFY/CONFIGURE/PERP 复位
static bool     fdc_eis      = false;   // CONFIGURE：EIS（强制中断每扇区）
static bool     fdc_efifo    = false;   // CONFIGURE：FIFO 使能（本模拟器无 FIFO，仅记录）
static bool     fdc_poll     = true;    // CONFIGURE：POLL（驱动器轮询）
static uint8_t  fdc_fifothr  = 0;       // CONFIGURE：FIFO 阈值
static uint8_t  fdc_pretrk   = 0;       // CONFIGURE：预补偿道
static uint8_t  fdc_perp     = 0;       // PERPENDICULAR MODE

// 最近一次命令涉及的驱动器（MSR 忙标志用）
static int      cur_drive = 0;

// 磁盘更换锁存（DIR 0x3F7 的 bit7）：真 uPD765 在复位后为 1（迫使 BIOS 先
// recalibrate），recalibrate/seek 完成（磁头归位）后由硬件自动清 0。模拟器
// 必须正确建模——否则 BIOS 读引导扇区前看到 bit7=1 会判定"盘被换过"而中止、
// 根本不发 READ DATA，最终退回 ROM BASIC。盘已挂载（镜像常驻）时该位应为 0。
static bool     disk_change[4] = { false, false, false, false };

// ============================================================
// 几何：由该驱动器镜像大小推断
// ============================================================
typedef struct { uint32_t spt, heads, cyls; } Geo;

static Geo media_geometry(int drive) {
  uint32_t sz = floppy_get_size(drive);
  Geo g = { 9, 2, 40 };          // 缺省按 360KB
  if (sz == 163840)       { g.spt = 8;  g.cyls = 40; }   // 160KB
  else if (sz == 327680)  { g.spt = 8;  g.cyls = 80; }   // 320KB
  else if (sz == 368640)  { g.spt = 9;  g.cyls = 40; }   // 360KB
  else if (sz == 737280)  { g.spt = 9;  g.cyls = 80; }   // 720KB
  else if (sz == 1228800) { g.spt = 15; g.cyls = 80; }   // 1.2MB
  else if (sz == 1474560) { g.spt = 18; g.cyls = 80; }   // 1.44MB
  else if (sz == 2949120) { g.spt = 36; g.cyls = 80; }   // 2.88MB
  /* 其它大小按 360KB 处理 */
  return g;
}

// 扇区大小 = 128 << N（N=2 即 512 字节）
static uint32_t sector_size(uint8_t N) { return 128u << (N & 0x07); }

// 驱动器是否存在（已挂载有效镜像）
static bool drive_present(int drive) {
  return floppy_get_data(drive) != NULL && floppy_get_size(drive) > 0;
}

// ============================================================
// 中断
// ============================================================
static void fdc_raise_irq(uint8_t st0, uint8_t pcnv) {
  int_q[int_qtail].st0 = st0;
  int_q[int_qtail].pcn = pcnv;
  int_qtail = (int_qtail + 1) & 7;
  irq_line = true;
}

// 取走一个待处理中断；队列空返回 false（SENSE INTERRUPT 应回 ST0=0x80）
static bool fdc_int_pop(uint8_t* st0, uint8_t* pcnv) {
  if (int_qhead == int_qtail) return false;
  *st0 = int_q[int_qhead].st0;
  *pcnv = int_q[int_qhead].pcn;
  int_qhead = (int_qhead + 1) & 7;
  if (int_qhead == int_qtail) irq_line = false;
  return true;
}

static void fdc_int_clear(void) {
  int_qhead = int_qtail = 0;
  irq_line = false;
}

bool fdc_irq_pending(void) { return irq_line; }
void fdc_irq_ack(void)     { irq_line = false; }

// ============================================================
// 主状态寄存器
// ============================================================
static uint8_t fdc_msr(void) {
  if (!(dor & 0x04)) return 0x00;        // 控制器处于复位状态
  uint8_t m = 0x80;                       // RQM=1（数据寄存器就绪）
  if (phase != PH_IDLE) m |= 0x10;        // CB=1（命令/结果忙，bit4）
  if (phase == PH_RESULT) m |= 0x40;      // DIO=1（方向：控制器→CPU，bit6）
  m |= (uint8_t)(1 << (cur_drive & 3));   // 当前选中驱动器忙（bit0-3）
  return m;
}

static void fdc_result(const uint8_t* data, int n) {
  if (n > (int)sizeof(res)) n = (int)sizeof(res);
  if (n > 0) memcpy(res, data, n);
  res_len = n;
  res_pos = 0;
  phase = (n > 0) ? PH_RESULT : PH_IDLE;
  if (debug_mode) fprintf(stderr, "[FDCRES] len=%d phase=%d\n", n, phase);
}

// 命令字节数（操作码低 5 位区分命令，高位是 MT/MF/SK 标志）
static int fdc_cmd_length(uint8_t c) {
  switch (c & 0x1F) {
    case 0x02: return 9;   // READ TRACK
    case 0x03: return 3;   // SPECIFY
    case 0x04: return 2;   // SENSE DRIVE STATUS
    case 0x05: return 9;   // WRITE DATA
    case 0x06: return 9;   // READ DATA
    case 0x07: return 2;   // RECALIBRATE
    case 0x08: return 1;   // SENSE INTERRUPT STATUS
    case 0x09: return 9;   // WRITE DELETED DATA
    case 0x0A: return 2;   // READ ID
    case 0x0C: return 9;   // READ DELETED DATA
    case 0x0D: return 6;   // FORMAT TRACK
    case 0x0E: return 1;   // DUMPREG（结果 10 字节，但命令本身 1 字节）
    case 0x0F: return 3;   // SEEK
    case 0x10: return 1;   // VERSION
    case 0x11: return 9;   // SCAN EQUAL
    case 0x12: return 2;   // PERPENDICULAR MODE
    case 0x13: return 4;   // CONFIGURE
    case 0x14: return 2;   // LOCK
    case 0x15: return 9;   // VERIFY
    case 0x16: return 9;   // SCAN LOW OR EQUAL
    case 0x17: return 9;   // SCAN HIGH OR EQUAL
    case 0x19: return 9;   // SCAN (alt)
    default:   return 1;   // 非法命令
  }
}

// ============================================================
// 读盘 / 写盘 / 读道（DMA 通道 2）
//   is_write : 写盘；is_track : READ TRACK（整道从扇区 1 起）
// ============================================================
static void fdc_rw(bool is_write, bool is_track) {
  vga_led_activity(1);                       // 软盘灯：命令一进来就亮

  int drive = cmd[1] & 0x03;
  int head  = (cmd[1] >> 2) & 0x01;
  uint8_t C = cmd[2];
  uint8_t H = cmd[3];
  uint8_t R = cmd[4];
  uint8_t N = cmd[5];
  cur_drive = drive;

  Geo g = media_geometry(drive);
  uint32_t ssize = sector_size(N);
  uint32_t n = dma_byte_count(2);           // 由 DMA 终端计数决定传输字节数
  uint32_t addr = dma_phys_addr(2);
  uint8_t* img = floppy_get_data(drive);
  uint32_t sz  = floppy_get_size(drive);
  bool ready = drive_present(drive) && img && sz > 0;

  uint8_t st0 = 0, st1 = 0, st2 = 0;
  uint32_t transferred = 0;

  if (!ready) {
    st0 = (uint8_t)(0xC0 | (head << 2) | drive);   // IC=11 异常终止（驱动器未就绪）
    st1 = 0x80;                                    // 未就绪位
  } else {
    // 起始 LBA（C/H/S → 线性扇区）：扇区号 1 基
    uint32_t start_lba;
    if (is_track) start_lba = ((uint32_t)C * g.heads + (uint32_t)H) * g.spt; // 整道从 1 号起
    else          start_lba = ((uint32_t)C * g.heads + (uint32_t)H) * g.spt + (R ? (uint32_t)R - 1 : 0);
    uint32_t off = start_lba * 512u;

    if (off >= sz) {
      st1 |= 0x04;                                 // ND：起始扇区不存在
    } else {
      uint32_t avail = sz - off;
      uint32_t todo = n;
      if (todo > avail) { todo = avail; st1 |= 0x04; }   // 跨过介质末尾
      // 道尾检测
      if (is_track) {
        if (todo < n) st1 |= 0x80;                 // EN：请求超过整道
      } else {
        uint32_t last_sec = (R ? (uint32_t)R - 1 : 0) + (todo / (ssize ? ssize : 1));
        if (last_sec > g.spt) st1 |= 0x80;         // EN：越过道尾
      }
      for (uint32_t i = 0; i < todo; i++) {
        if (is_write) img[off + i] = memory[(addr + i) & 0xFFFFF];
        else          memory[(addr + i) & 0xFFFFF] = img[off + i];
        transferred++;
      }
      dma_advance(2, transferred);
    }
  }

  if (is_write && transferred) floppy_mark_dirty(drive);   // 写盘后标记，退出时写回镜像

  pcn[drive] = C;
  // ST0：IC 位（bit7-6）——未就绪=11，有错=01，正常=00
  st0 = (uint8_t)((head << 2) | drive);
  if (!ready)     st0 |= 0xC0;        // IC=11
  else if (st1)   st0 |= 0x40;        // IC=01

  uint32_t nsec = transferred / (ssize ? ssize : 1);
  uint8_t lastR = (uint8_t)(R + (nsec ? nsec - 1 : 0));
  if (is_track) lastR = (uint8_t)(R + nsec);

  uint8_t r[7] = { st0, st1, st2, C, (uint8_t)H, lastR, N };
  fdc_result(r, 7);
  fdc_raise_irq(st0, C);

  // 调试：软盘读后 dump 实际 DMA 地址与 0:7C00 内存（仅 -dbg 时）
  if (debug_mode && !is_write && drive == 0) {
    fprintf(stderr, "[FDC] FLOPPYREAD addr=%05X st0=%02X st1=%02X xfer=%u "
            "mem7C00=%02X %02X %02X %02X sig=%02X %02X\n",
            addr, st0, st1, transferred,
            memory[0x7C00], memory[0x7C01], memory[0x7C02], memory[0x7C03],
            memory[0x7DFE], memory[0x7DFF]);
  }

  if (debug_mode) {
    static clock_t last_ck = 0;
    static uint64_t last_insn = 0;
    clock_t ck = clock();
    printf("[FDC] %s drv=%d C=%u H=%d R=%u N=%u n=%u addr=%05X st1=%02X dt=%ldms insn=%llu\n",
           is_write ? "WRITE" : (is_track ? "READTRK" : "READ"), drive, C, head, R, N, n, addr, st1,
           (long)((ck - last_ck) * 1000 / CLOCKS_PER_SEC),
           (unsigned long long)(insn_count - last_insn));
    last_ck = ck;
    last_insn = insn_count;
  }
}

// SCAN 系列：读盘并与内存缓冲比较（不覆盖内存），设置 ST2 比较结果
static void fdc_scan(void) {
  vga_led_activity(1);

  int drive = cmd[1] & 0x03;
  int head  = (cmd[1] >> 2) & 0x01;
  uint8_t C = cmd[2];
  uint8_t H = cmd[3];
  uint8_t R = cmd[4];
  uint8_t N = cmd[5];
  cur_drive = drive;

  Geo g = media_geometry(drive);
  uint32_t ssize = sector_size(N);
  uint32_t n = dma_byte_count(2);
  uint32_t addr = dma_phys_addr(2);
  uint8_t* img = floppy_get_data(drive);
  uint32_t sz  = floppy_get_size(drive);
  bool ready = drive_present(drive) && img && sz > 0;

  uint8_t st0 = 0, st1 = 0, st2 = 0;
  uint32_t transferred = 0;

  if (!ready) {
    st0 = (uint8_t)(0xC0 | (head << 2) | drive);
    st1 = 0x80;
  } else {
    uint32_t start_lba = ((uint32_t)C * g.heads + (uint32_t)H) * g.spt + (R ? (uint32_t)R - 1 : 0);
    uint32_t off = start_lba * 512u;
    if (off >= sz) {
      st1 |= 0x04;
    } else {
      uint32_t avail = sz - off;
      uint32_t todo = n;
      if (todo > avail) { todo = avail; st1 |= 0x04; }
      bool mismatch = false;
      for (uint32_t i = 0; i < todo; i++) {
        uint8_t disk = img[off + i];
        uint8_t mem  = memory[(addr + i) & 0xFFFFF];
        if (disk != mem) { mismatch = true; st2 |= 0x04; break; }  // ST2: 比较失败
        transferred++;
      }
      dma_advance(2, transferred);
      (void)mismatch;
    }
    pcn[drive] = C;
  }

  st0 = (uint8_t)((head << 2) | drive);
  if (!ready)   st0 |= 0xC0;
  else if (st1) st0 |= 0x40;

  uint32_t nsec = transferred / (ssize ? ssize : 1);
  uint8_t lastR = (uint8_t)(R + (nsec ? nsec - 1 : 0));
  uint8_t r[7] = { st0, st1, st2, C, (uint8_t)H, lastR, N };
  fdc_result(r, 7);
  fdc_raise_irq(st0, C);

  if (debug_mode)
    printf("[FDC] SCAN drv=%d C=%u H=%d R=%u st1=%02X st2=%02X\n",
           drive, C, head, R, st1, st2);
}

// VERIFY：校验扇区存在/CRC，不搬数据到内存
static void fdc_verify(void) {
  vga_led_activity(1);

  int drive = cmd[1] & 0x03;
  int head  = (cmd[1] >> 2) & 0x01;
  uint8_t C = cmd[2];
  uint8_t H = cmd[3];
  uint8_t R = cmd[4];
  uint8_t N = cmd[5];
  cur_drive = drive;

  Geo g = media_geometry(drive);
  uint32_t ssize = sector_size(N);
  uint32_t n = dma_byte_count(2);
  uint8_t* img = floppy_get_data(drive);
  uint32_t sz  = floppy_get_size(drive);
  bool ready = drive_present(drive) && img && sz > 0;

  uint8_t st0 = 0, st1 = 0, st2 = 0;
  uint32_t transferred = 0;

  if (!ready) {
    st0 = (uint8_t)(0xC0 | (head << 2) | drive);
    st1 = 0x80;
  } else {
    uint32_t start_lba = ((uint32_t)C * g.heads + (uint32_t)H) * g.spt + (R ? (uint32_t)R - 1 : 0);
    uint32_t off = start_lba * 512u;
    if (off >= sz) {
      st1 |= 0x04;
    } else {
      uint32_t avail = sz - off;
      uint32_t todo = n;
      if (todo > avail) { todo = avail; st1 |= 0x04; }
      if (todo < n) st1 |= 0x80;          // EN
      transferred = todo;
      dma_advance(2, transferred);
    }
    pcn[drive] = C;
  }

  st0 = (uint8_t)((head << 2) | drive);
  if (!ready)   st0 |= 0xC0;
  else if (st1) st0 |= 0x40;

  uint32_t nsec = transferred / (ssize ? ssize : 1);
  uint8_t lastR = (uint8_t)(R + (nsec ? nsec - 1 : 0));
  uint8_t r[7] = { st0, st1, st2, C, (uint8_t)H, lastR, N };
  fdc_result(r, 7);
  fdc_raise_irq(st0, C);

  if (debug_mode)
    printf("[FDC] VERIFY drv=%d C=%u H=%d R=%u st1=%02X\n", drive, C, head, R, st1);
}

static void exec_format(void) {
  vga_led_activity(1);

  int drive = cmd[1] & 0x03;
  int head  = (cmd[1] >> 2) & 0x01;
  uint8_t N = cmd[2];
  uint8_t SC = cmd[3];
  uint8_t fill = cmd[5];
  cur_drive = drive;
  Geo g = media_geometry(drive);
  uint8_t* img = floppy_get_data(drive);
  uint32_t sz = floppy_get_size(drive);
  uint32_t addr = dma_phys_addr(2);

  uint8_t st1 = 0;
  uint32_t done = 0;
  // 格式化缓冲区每扇区 4 字节 ID 域(C,H,R,N)，数据区填命令给的填充字节
  if (drive_present(drive) && img) {
    for (int i = 0; i < (int)SC; i++) {
      uint8_t fc = memory[(addr + i * 4 + 0) & 0xFFFFF];
      uint8_t fh = memory[(addr + i * 4 + 1) & 0xFFFFF];
      uint8_t fr = memory[(addr + i * 4 + 2) & 0xFFFFF];
      uint32_t lba = ((uint32_t)fc * g.heads + (uint32_t)(fh & 1)) * g.spt + (fr ? (uint32_t)fr - 1 : 0);
      uint32_t off = lba * 512u;
      if (off + 512u > sz) { st1 |= 0x04; break; }
      memset(img + off, fill, 512);
      done++;
    }
    dma_advance(2, (uint32_t)done * 4);
    if (done) floppy_mark_dirty(drive);        // 格式化后标记，退出时写回镜像文件
  } else {
    st1 |= 0x80;
  }

  uint8_t st0 = (uint8_t)((head << 2) | drive | (st1 ? 0x40 : 0x00));
  uint8_t r[7] = { st0, st1, 0, cmd[2], (uint8_t)head, (uint8_t)(SC + 1), N };
  fdc_result(r, 7);
  fdc_raise_irq(st0, pcn[drive]);

  if (debug_mode)
    printf("[FDC] FORMAT drv=%d H=%d N=%u SC=%u fill=%02X done=%d st1=%02X\n",
           drive, head, N, SC, fill, done, st1);
}

// ============================================================
// 机器复位（GTK「重启」按钮）：命令相位/中断状态/柱面全部回到上电值
// ============================================================
void fdc_reset(void) {
  phase = PH_IDLE;
  cmd_len = cmd_pos = 0;
  opcode = 0;
  res_len = res_pos = 0;
  dor = 0;
  fdc_int_clear();
  cur_drive = 0;
  for (int i = 0; i < 4; i++) pcn[i] = 0;
  specify_srt_hut = 0xDF;
  specify_hlt_nd  = 0x02;
  // CONFIGURE/PERPENDICULAR/LOCK 在锁定时不被复位
  if (!fdc_locked) {
    fdc_eis = false; fdc_efifo = false; fdc_poll = true;
    fdc_fifothr = 0; fdc_pretrk = 0; fdc_perp = 0;
  }
}

// ============================================================
// 命令执行
// ============================================================
static void fdc_execute(void) {
  phase = PH_IDLE;                           // 默认：命令结束后没有结果相位
  // 调试：打印 FDC 命令（仅 -dbg 时，便于定位 boot 读与 601）
  if (debug_mode)
    fprintf(stderr, "[FDCCMD] op=%02X n=%d p=%02X %02X %02X %02X pcnA=%u pcnB=%u\n",
            opcode, cmd_len, cmd[1], cmd[2], cmd[3], cmd[4], pcn[0], pcn[1]);

  switch (opcode & 0x1F) {
    case 0x02:                               // READ TRACK
      fdc_rw(false, true);
      break;

    case 0x03: {                             // SPECIFY：无结果、无中断
      if (!fdc_locked) {
        specify_srt_hut = cmd[1];
        specify_hlt_nd  = cmd[2];
      }
      break;
    }

    case 0x04: {                             // SENSE DRIVE STATUS → ST3
      uint8_t hdus = cmd[1];
      int drive = hdus & 0x03;
      cur_drive = drive;
      // ST3：bit6=T00（在 0 道置 1）；bit2 恒 1；bit4 就绪位恒 1；bit1 磁头；bit0-1 驱动器。
      // bit7=DSKCHG（磁盘更换）必须反映 disk_change[drive] 真实状态：复位后置位、recal/seek
      // 完成后清除。若硬编码为 1，BIOS 读处理例程会误判“磁盘已更换”，陷入
      // recalibrate→SENSE 死循环，永不下发 READ 命令（op=06）→ 引导失败。
      uint8_t s = (uint8_t)(0x24 | (drive & 0x03));
      if (pcn[drive] == 0) s |= 0x40;        // T00：当前在 0 道
      if (hdus & 0x04) s |= 0x02;            // HDS → ST3 bit1
      s |= 0x10;                             // 就绪位(bit4) 恒 1（缺失盘 READY 上拉）
      if (disk_change[drive]) s |= 0x80;     // DSKCHG 反映真实状态
      else s &= ~0x80;
      // 写保护：可写镜像不报 WP（bit3=0）。如需支持只读镜像此处再加。
      fdc_result(&s, 1);
      if (debug_mode)
        printf("[FDC] SENSE-DRIVE d=%d hdus=%02X ST3=%02X dc=%d\n",
               drive, hdus, s, disk_change[drive]);
      break;
    }

    case 0x05: case 0x09:                    // WRITE DATA / WRITE DELETED
      fdc_rw(true, false);
      break;

    case 0x06: case 0x0C:                    // READ DATA / READ DELETED
      fdc_rw(false, false);
      break;

    case 0x07: {                             // RECALIBRATE
      int drive = cmd[1] & 0x03;
      cur_drive = drive;
      pcn[drive] = 0;
      fdc_int_clear();   // 丢弃复位阶段未读尽的复位中断，避免插队到 recal 结果前
      uint8_t st0;
      if (drive_present(drive)) {
        // 已连接盘：归位到 0 道，寻道结束（IC=00, SE=1）
        disk_change[drive] = false;          // recalibrate 完成清磁盘更换锁存
        st0 = (uint8_t)(0x20 | drive);
      } else {
        // 未连接盘：真实 uPD765 返回 equipment check（IC=01, EC=1）
        st0 = (uint8_t)(0x60 | drive);
      }
      if (debug_mode)
        printf("[FDC] RECAL drv=%d present=%d -> ST0=%02X\n", drive, drive_present(drive), st0);
      fdc_raise_irq(st0, 0x00);
      break;
    }

    case 0x08: {                             // SENSE INTERRUPT STATUS
      uint8_t r[2];
      int qcnt = (int_qtail - int_qhead) & 7;
      if (debug_mode)
        fprintf(stderr, "[FDC] SENSE-INT qcnt=%d h=%d t=%d\n", qcnt, int_qhead, int_qtail);
      if (fdc_int_pop(&r[0], &r[1])) {
        if (debug_mode)
          fprintf(stderr, "[FDC] SENSE-INT ST0=%02X PCN=%02X\n", r[0], r[1]);
      } else {
        r[0] = 0x80;                         // 无中断挂起 → ST0=0x80（非法命令）
        r[1] = pcn[cur_drive];
        if (debug_mode)
          fprintf(stderr, "[FDC] SENSE-INT (none) ST0=80\n");
      }
      fdc_result(r, 2);
      break;
    }

    case 0x0A: {                             // READ ID
      int hdus = cmd[1];
      int drive = hdus & 0x03;
      int head = (hdus >> 2) & 0x01;
      cur_drive = drive;
      uint8_t st0 = (uint8_t)((head << 2) | drive);
      uint8_t r[7] = { st0, 0x00, 0x00, pcn[drive], (uint8_t)head, 1, 2 };
      fdc_result(r, 7);
      fdc_raise_irq(st0, pcn[drive]);
      break;
    }

    case 0x0D:                               // FORMAT TRACK
      exec_format();
      break;

    case 0x0E: {                             // DUMPREG
      uint8_t r[10] = { pcn[0], pcn[1], pcn[2], pcn[3],
                        specify_srt_hut, specify_hlt_nd, 0, 0, 0, 0 };
      fdc_result(r, 10);
      break;
    }

    case 0x10: {                             // VERSION
      uint8_t r = 0x90;                      // 82072 / 765A
      fdc_result(&r, 1);
      break;
    }

    case 0x11: case 0x16: case 0x17: case 0x19:  // SCAN 系列
      fdc_scan();
      break;

    case 0x12:                               // PERPENDICULAR MODE：无结果相位
      if (!fdc_locked) fdc_perp = cmd[1];
      break;

    case 0x13: {                             // CONFIGURE：无结果相位
      if (!fdc_locked) {
        // cmd[2]: 0, EIS(bit7=0), EFIFO(bit6), POLL(bit5); 低 4 位=FIFO 阈值
        fdc_eis     = (cmd[2] & 0x40) != 0;
        fdc_efifo   = (cmd[2] & 0x20) != 0;
        fdc_poll    = (cmd[2] & 0x10) != 0;
        fdc_fifothr = (uint8_t)(cmd[2] & 0x0F);
        fdc_pretrk  = cmd[3];
      }
      break;
    }

    case 0x14: {                             // LOCK：返回当前锁状态（1 字节结果）
      fdc_locked = (cmd[1] & 0x01) != 0;
      uint8_t r = (uint8_t)(fdc_locked ? 0x01 : 0x00);
      fdc_result(&r, 1);
      break;
    }

    case 0x15:                               // VERIFY
      fdc_verify();
      break;

    case 0x0F: {                             // SEEK
      int hdus = cmd[1];
      int drive = hdus & 0x03;
      int head = (hdus >> 2) & 0x01;
      uint8_t cyl = cmd[2];
      cur_drive = drive;
      pcn[drive] = cyl;
      fdc_int_clear();   // 丢弃复位阶段未读尽的复位中断，避免插队到 seek 结果前
      uint8_t st0;
      if (drive_present(drive)) {
        disk_change[drive] = false;          // seek 完成清磁盘更换锁存
        st0 = (uint8_t)(0x20 | (head << 2) | drive);
      } else {
        st0 = (uint8_t)(0x60 | (head << 2) | drive);
      }
      if (debug_mode)
        printf("[FDC] SEEK drv=%d head=%d cyl=%u present=%d -> ST0=%02X\n",
               drive, head, cyl, drive_present(drive), st0);
      fdc_raise_irq(st0, cyl);
      break;
    }

    default:                                 // 非法命令：ST0 的 IC=10
      if (debug_mode)
        printf("[FDC] INVALID op=%02X (command misaligned?)\n", opcode);
      fdc_raise_irq(0x80, pcn[cur_drive]);
      break;
  }
}

// ============================================================
// 数据寄存器写（命令字节）
// ============================================================
static void fdc_write_data(uint8_t val) {
  if (debug_mode)
    fprintf(stderr, "[FDCW] val=%02X phase=%d cmd_pos=%d cmd_len=%d\n", val, phase, cmd_pos, cmd_len);
  if (phase == PH_RESULT) {
    // 结果还没读完就来了命令字节：说明上一条命令的字节数和 BIOS 期待的不一致。
    // 放弃旧结果、重开命令，避免之后所有命令错位。
    if (debug_mode)
      printf("[FDC] result phase received %02X, discard incomplete result and restart command\n", val);
    res_len = res_pos = 0;
    phase = PH_IDLE;
  }

  if (phase == PH_IDLE) {
    opcode = val;
    cmd[0] = val;
    cmd_pos = 1;
    cmd_len = fdc_cmd_length(val);
    if (cmd_len <= 1) fdc_execute();
    else phase = PH_CMD;
    return;
  }

  if (cmd_pos < (int)sizeof(cmd)) cmd[cmd_pos] = val;
  cmd_pos++;
  if (cmd_pos >= cmd_len) fdc_execute();
}

// ============================================================
// DOR（0x3F2）
//   位0-1 驱动器选择；位2 复位(1=运行) 位3 DMA 门；位4-7 各驱动器马达
// ============================================================
static void fdc_write_dor(uint8_t val) {
  bool was_enabled = (dor & 0x04) != 0;
  bool now_enabled = (val & 0x04) != 0;
  dor = val;

  if (!now_enabled) {                        // bit2=0：复位，清空一切
    phase = PH_IDLE;
    res_len = res_pos = 0;
    cmd_pos = 0;
    fdc_int_clear();
    cur_drive = 0;
    for (int i = 0; i < 4; i++) { pcn[i] = 0; }  // 复位：磁头归零；DSKCHG 锁存不置位
                                  // （真实 uPD765 复位不会断言磁盘更换，只有实际换盘才置位；
                                  // 若此处置位，BIOS 引导读会在“复位→recal→再复位”间死循环）
    return;
  }
  // 由复位进入使能：为每台盘(drive 0..3)各产生一个中断，ST0=0xC0|drv（IC=11=未就绪）。
  // 这是真实 uPD765 复位后的标准行为，也是 AT BIOS int 13h ah=0 复位例程的硬约束
  // （它循环 4 次 SENSE INTERRUPT STATUS，依次期望 0xC0/0xC1/0xC2/0xC3）。
  // recalibrate/seek 命令开始时清空未读尽的复位中断，避免插队到结果前导致错位(601)。
  if (!was_enabled) {
    for (int d = 0; d < 4; d++) fdc_raise_irq((uint8_t)(0xC0 | d), 0);
  }
  if (!was_enabled) {                        // 复位结束：真 uPD765 复位后不产生中断，
    phase = PH_IDLE;                         //   SENSE INTERRUPT STATUS 应回 ST0=0x80。
    res_len = res_pos = 0;
    cmd_pos = 0;
  }
  // bit2 一直是 1 时（只改马达/选择位）不动状态
}

// ============================================================
// 端口接口
// ============================================================
uint8_t fdc_read_port(uint16_t port) {
  switch (port) {
    case 0x3F0: return (uint8_t)(irq_line ? 0x80 : 0x00);  // SRA：bit7=中断挂起
    case 0x3F1: return 0x00;                 // SRB
    case 0x3F2: return dor;                  // DOR
    case 0x3F3: return 0x00;
    case 0x3F4: { uint8_t m = fdc_msr(); if (debug_mode) fprintf(stderr, "[FDCMR] MSR=%02X phase=%d\n", m, phase); return m; }            // MSR
    case 0x3F5: {                            // 数据寄存器
      if (phase != PH_RESULT || res_pos >= res_len) return 0xFF;
      uint8_t v = res[res_pos++];
      if (res_pos >= res_len) {
        phase = PH_IDLE;
        res_len = res_pos = 0;
      }
      return v;
    }
    case 0x3F6: return 0x00;
    case 0x3F7: {                          // DIR：bit7=磁盘更换锁存（盘常驻时为 0）
      uint8_t v = disk_change[dor & 0x03] ? 0x80 : 0x00;
      if (debug_mode)
        fprintf(stderr, "[FDCDIR] drive=%d dir=%02X disk_change=%d\n", dor & 0x03, v, disk_change[dor & 0x03]);
      return v;
    }
    default:    return 0xFF;
  }
}

void fdc_write_port(uint16_t port, uint8_t val) {
  switch (port) {
    case 0x3F2: fdc_write_dor(val); break;
    case 0x3F3: break;
    case 0x3F4: break;                       // 数据传输率/预补偿（忽略）
    case 0x3F5: fdc_write_data(val); break;
    case 0x3F7: break;                       // 固定数据传输率（忽略）
    default: break;
  }
}
