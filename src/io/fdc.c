// fdc.c - NEC uPD765 软盘控制器（PC/XT）
//   端口：0x3F0 SRA / 0x3F1 SRB / 0x3F2 DOR / 0x3F4 MSR / 0x3F5 DATA / 0x3F7 DIR
//   数据走 DMA 通道 2；命令结束产生 IRQ6（主循环轮询 → INT 0Eh）
//
//   相位由 MSR 表示：bit7 RQM(数据口可读写) bit6 DIO(1=FDC→CPU) bit4 CB(忙)
//     PH_CMD    ：正在收命令字节（CB=1，DIO=0）
//     PH_RESULT ：结果字节可读（CB=1，DIO=1），最后一个字节读走后 CB 变 0
//   ★ 命令长度必须和 BIOS 期待的结果字节数完全一致。少一个/多一个都会让 BIOS
//     的下一条命令字节落进结果相位被吞掉 → 之后所有命令错位 → BIOS 报 "Data error"。
//     所以这里宁可放宽（结果相位收到命令字节就放弃旧结果重开），绝不静默丢弃。
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
// 状态
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

// 待 SENSE INTERRUPT STATUS 取走的中断状态（只有一份：新中断覆盖旧中断）
static bool     int_valid = false;
static uint8_t  int_st0   = 0x80;
static uint8_t  int_pcn   = 0;

// IRQ6 边沿，等主循环轮询投递
static bool     irq_line = false;

// 每台驱动器当前柱面 + SPECIFY 参数
static uint8_t  pcn[4] = {0, 0, 0, 0};
static uint8_t  specify_srt_hut = 0xDF;
static uint8_t  specify_hlt_nd  = 0x02;

// ============================================================
// 几何：由该驱动器镜像大小推断，缺省按 360KB
// ============================================================
typedef struct { uint32_t spt, heads, cyls; } Geo;

static Geo media_geometry(int drive) {
  uint32_t sz = floppy_get_size(drive);
  Geo g = { 9, 2, 40 };
  if (sz == 737280)       { g.spt = 9;  g.cyls = 80; }   // 720KB
  else if (sz == 1228800) { g.spt = 15; g.cyls = 80; }   // 1.2MB
  else if (sz == 1474560) { g.spt = 18; g.cyls = 80; }   // 1.44MB
  else if (sz == 368640)  { g.spt = 9;  g.cyls = 40; }   // 360KB
  /* 其它大小按 360KB 处理 */
  return g;
}

// 扇区大小 = 128 << N（N=2 即 512 字节）
static uint32_t sector_size(uint8_t N) { return 128u << (N & 0x07); }

// ============================================================
// 中断
// ============================================================
static void fdc_raise_irq(uint8_t st0, uint8_t pcnv) {
  int_st0 = st0;
  int_pcn = pcnv;
  int_valid = true;
  irq_line = true;
}

bool fdc_irq_pending(void) { return irq_line; }
void fdc_irq_ack(void)     { irq_line = false; }

// ============================================================
// 主状态寄存器
// ============================================================
static uint8_t fdc_msr(void) {
  if (!(dor & 0x04)) return 0x00;        // 控制器处于复位状态
  switch (phase) {
    case PH_CMD:    return 0x90;         // RQM=1 DIO=0 CB=1
    case PH_RESULT: return 0xD0;         // RQM=1 DIO=1 CB=1
    default:        return 0x80;         // RQM=1，空闲（可写命令）
  }
}

static void fdc_result(const uint8_t* data, int n) {
  if (n > (int)sizeof(res)) n = (int)sizeof(res);
  if (n > 0) memcpy(res, data, n);
  res_len = n;
  res_pos = 0;
  phase = (n > 0) ? PH_RESULT : PH_IDLE;
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
    case 0x0E: return 1;   // DUMPREG
    case 0x0F: return 3;   // SEEK
    case 0x11: return 9;   // SCAN EQUAL
    case 0x19: return 9;   // SCAN LOW/EQUAL
    default:   return 1;   // 非法命令
  }
}

// ============================================================
// 读盘 / 写盘（DMA 通道 2）
//   传输字节数 = DMA 剩余计数（DMA 终端计数结束传输），不做任何 N 的限制：
//   旧代码遇到 N != 2 就塞 ST1 的 CRC 位，BIOS 会把它翻成 AH=10h，
//   DOS 于是报 "Data error reading drive A"。
// ============================================================
static void exec_read(void) {
  vga_led_activity(1);                       // 软盘读盘灯：命令一进来就亮

  int drive = cmd[1] & 0x03;
  int head  = (cmd[1] >> 2) & 0x01;
  uint8_t C = cmd[2];
  uint8_t R = cmd[4];
  uint8_t N = cmd[5];

  Geo g = media_geometry(drive);
  uint32_t ssize = sector_size(N);
  uint32_t lba = ((uint32_t)C * g.heads + (uint32_t)head) * g.spt + (R ? (uint32_t)R - 1 : 0);
  uint32_t off = lba * 512u;
  uint32_t sz  = floppy_get_size(drive);
  uint8_t* img = floppy_get_data(drive);
  uint32_t addr = dma_phys_addr(2);
  uint32_t n = dma_byte_count(2);

  uint8_t st1 = 0, st2 = 0;
  if (!img || sz == 0 || off >= sz) {
    st1 |= 0x04;                             // ND：扇区不存在
  } else {
    for (uint32_t i = 0; i < n; i++)
      memory[(addr + i) & 0xFFFFF] = (off + i < sz) ? img[off + i] : 0x00;
    if (off + n > sz) st1 |= 0x04;
    dma_advance(2, n);
  }

  pcn[drive] = C;
  uint32_t nsec = (n + ssize - 1) / ssize;   // 本次跨了几个扇区
  uint8_t st0 = (uint8_t)((head << 2) | drive | (st1 ? 0x40 : 0x00));
  uint8_t r[7] = { st0, st1, st2, C, (uint8_t)head, (uint8_t)(R + nsec), N };
  fdc_result(r, 7);
  fdc_raise_irq(st0, C);

  if (debug_mode) {
    static clock_t last_ck = 0;
    static uint64_t last_insn = 0;
    clock_t ck = clock();
    printf("[FDC] READ  drv=%d C=%u H=%d R=%u N=%u EOT=%u lba=%u n=%u addr=%05X st1=%02X dt=%ldms insn=%llu tick=%08X\n",
           drive, C, head, R, N, cmd[6], lba, n, addr, st1,
           (long)((ck - last_ck) * 1000 / CLOCKS_PER_SEC),
           (unsigned long long)(insn_count - last_insn),
           (unsigned)(memory[0x46C] | (memory[0x46D] << 8) | (memory[0x46E] << 16) | (memory[0x46F] << 24)));
    last_ck = ck;
    last_insn = insn_count;
  }
}

static void exec_write(void) {
  vga_led_activity(1);                       // 软盘写盘灯

  int drive = cmd[1] & 0x03;
  int head  = (cmd[1] >> 2) & 0x01;
  uint8_t C = cmd[2];
  uint8_t R = cmd[4];
  uint8_t N = cmd[5];

  Geo g = media_geometry(drive);
  uint32_t ssize = sector_size(N);
  uint32_t lba = ((uint32_t)C * g.heads + (uint32_t)head) * g.spt + (R ? (uint32_t)R - 1 : 0);
  uint32_t off = lba * 512u;
  uint32_t sz  = floppy_get_size(drive);
  uint8_t* img = floppy_get_data(drive);
  uint32_t addr = dma_phys_addr(2);
  uint32_t n = dma_byte_count(2);

  uint8_t st1 = 0, st2 = 0;
  if (!img || sz == 0 || off >= sz) {
    st1 |= 0x04;
  } else {
    for (uint32_t i = 0; i < n; i++)
      if (off + i < sz) img[off + i] = memory[(addr + i) & 0xFFFFF];
    if (off + n > sz) st1 |= 0x04;
    dma_advance(2, n);
    floppy_mark_dirty(drive);        // 写盘后标记，退出时写回镜像文件
  }

  pcn[drive] = C;
  uint32_t nsec = (n + ssize - 1) / ssize;
  uint8_t st0 = (uint8_t)((head << 2) | drive | (st1 ? 0x40 : 0x00));
  uint8_t r[7] = { st0, st1, st2, C, (uint8_t)head, (uint8_t)(R + nsec), N };
  fdc_result(r, 7);
  fdc_raise_irq(st0, C);

  if (debug_mode)
    printf("[FDC] WRITE drv=%d C=%u H=%d R=%u N=%u lba=%u n=%u addr=%05X st1=%02X\n",
           drive, C, head, R, N, lba, n, addr, st1);
}

static void exec_format(void) {
  vga_led_activity(1);

  int drive = cmd[1] & 0x03;
  int head  = (cmd[1] >> 2) & 0x01;
  uint8_t N = cmd[2];
  uint8_t SC = cmd[3];
  uint8_t fill = cmd[5];
  Geo g = media_geometry(drive);
  uint8_t* img = floppy_get_data(drive);
  uint32_t sz = floppy_get_size(drive);
  uint32_t addr = dma_phys_addr(2);

  uint8_t st1 = 0;
  int done = 0;
  // 格式化缓冲区每扇区 4 字节 ID 域(C,H,R,N)，数据区填命令给的填充字节
  for (int i = 0; i < (int)SC; i++) {
    uint8_t fc = memory[(addr + i * 4 + 0) & 0xFFFFF];
    uint8_t fh = memory[(addr + i * 4 + 1) & 0xFFFFF];
    uint8_t fr = memory[(addr + i * 4 + 2) & 0xFFFFF];
    uint32_t lba = ((uint32_t)fc * g.heads + (uint32_t)(fh & 1)) * g.spt + (fr ? (uint32_t)fr - 1 : 0);
    uint32_t off = lba * 512u;
    if (!img || off + 512u > sz) { st1 |= 0x04; break; }
    memset(img + off, fill, 512);
    done++;
  }
  dma_advance(2, (uint32_t)done * 4);
  if (done) floppy_mark_dirty(drive);        // 格式化后标记，退出时写回镜像文件

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
  int_valid = false; int_st0 = 0x80; int_pcn = 0;
  irq_line = false;
  for (int i = 0; i < 4; i++) pcn[i] = 0;
  specify_srt_hut = 0xDF;
  specify_hlt_nd  = 0x02;
}

// ============================================================
// 命令执行
// ============================================================
static void fdc_execute(void) {
  phase = PH_IDLE;                           // 默认：命令结束后没有结果相位

  switch (opcode & 0x1F) {
    case 0x03: {                             // SPECIFY：无结果、无中断
      specify_srt_hut = cmd[1];
      specify_hlt_nd  = cmd[2];
      break;
    }

    case 0x04: {                             // SENSE DRIVE STATUS → ST3
      uint8_t hdus = cmd[1];
      int drive = hdus & 0x03;
      uint8_t s = (uint8_t)(0x28 | drive);   // 双面 + 就绪
      if (hdus & 0x04) s |= 0x04;            // 磁头选择
      if (pcn[drive] == 0) s |= 0x10;        // 在 0 磁道
      fdc_result(&s, 1);
      break;
    }

    case 0x05: case 0x09:                    // WRITE DATA / WRITE DELETED
      exec_write();
      break;

    case 0x06: case 0x0C:                    // READ DATA / READ DELETED
    case 0x11: case 0x19:                    // SCAN（不支持，按读处理，别让 BIOS 卡住）
      exec_read();
      break;

    case 0x07: {                             // RECALIBRATE
      int drive = cmd[1] & 0x03;
      pcn[drive] = 0;
      fdc_raise_irq((uint8_t)(0x20 | drive), 0x00);
      break;
    }

    case 0x08: {                             // SENSE INTERRUPT STATUS
      uint8_t r[2];
      r[0] = int_valid ? int_st0 : 0x80;     // 无中断挂起 → ST0=0x80（非法）
      r[1] = int_valid ? int_pcn : pcn[0];
      int_valid = false;
      fdc_result(r, 2);
      break;
    }

    case 0x0A: {                             // READ ID
      int hdus = cmd[1];
      int drive = hdus & 0x03;
      int head = (hdus >> 2) & 0x01;
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

    case 0x0F: {                             // SEEK
      int hdus = cmd[1];
      int drive = hdus & 0x03;
      int head = (hdus >> 2) & 0x01;
      uint8_t cyl = cmd[2];
      pcn[drive] = cyl;
      fdc_raise_irq((uint8_t)(0x20 | (head << 2) | drive), cyl);
      break;
    }

    default:                                 // 非法命令：ST0 的 IC=10
      if (debug_mode)
        printf("[FDC] INVALID op=%02X（命令错位？）\n", opcode);
      fdc_raise_irq(0x80, pcn[0]);
      break;
  }
}

// ============================================================
// 数据寄存器写（命令字节）
// ============================================================
static void fdc_write_data(uint8_t val) {
  if (phase == PH_RESULT) {
    // 结果还没读完就来了命令字节：说明上一条命令的字节数和 BIOS 期待的不一致。
    // 旧实现在这里直接 return（静默吞字节），之后所有命令都会错位，BIOS 报 Data error。
    if (debug_mode)
      printf("[FDC] 结果相位收到 %02X，放弃未读完的结果重开命令\n", val);
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
// ============================================================
static void fdc_write_dor(uint8_t val) {
  bool was_enabled = (dor & 0x04) != 0;
  bool now_enabled = (val & 0x04) != 0;
  dor = val;

  if (!now_enabled) {                        // bit2=0：复位，清空一切
    phase = PH_IDLE;
    res_len = res_pos = 0;
    cmd_pos = 0;
    int_valid = false;
    irq_line = false;
    for (int i = 0; i < 4; i++) pcn[i] = 0;
    return;
  }
  if (!was_enabled) {                        // 复位结束：产生一次中断（SIS 返回 ST0=0xC0）
    phase = PH_IDLE;
    res_len = res_pos = 0;
    cmd_pos = 0;
    fdc_raise_irq(0xC0, 0x00);
  }
  // bit2 一直是 1 时（只改电机位）不动状态
}

// ============================================================
// 端口接口
// ============================================================
uint8_t fdc_read_port(uint16_t port) {
  switch (port) {
    case 0x3F0: return 0x00;                 // SRA
    case 0x3F1: return 0x00;                 // SRB
    case 0x3F2: return dor;                  // DOR
    case 0x3F3: return 0x00;
    case 0x3F4: return fdc_msr();            // MSR
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
    case 0x3F7: return 0x80;                 // DIR：无磁盘更换
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