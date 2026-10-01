// cpu.h - 8086 CPU Emulation Header
#ifndef CPU_H
#define CPU_H

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>

typedef struct {
	uint16_t ax, bx, cx, dx;
	uint16_t si, di, bp, sp;
	uint16_t cs, ds, es, ss;
	uint16_t fs, gs;
	uint16_t ip;
	uint16_t flags;
	bool prefix_66;   // 32 位操作数
        bool prefix_67;   // 32 位地址
	// ---- 286 保护模式的段缓存：选择子对应的基址/限长 ----
	//   实模式下基址 = 选择子 << 4、限长恒为 0xFFFF；保护模式下由描述符填入。
	//   索引：0=ES 1=CS 2=SS 3=DS 4=FS 5=GS（见 SEG_* 枚举）
	uint32_t seg_base[6];
	uint32_t seg_limit[6];
	uint8_t  seg_access[6];   // 描述符访问权限字节（写权限检查用）
} CPU_8086;

// 段寄存器索引
enum { SEG_ES = 0, SEG_CS = 1, SEG_SS = 2, SEG_DS = 3, SEG_FS = 4, SEG_GS = 5 };

#define FLAG_CF 0x0001
#define FLAG_PF 0x0004
#define FLAG_AF 0x0010
#define FLAG_ZF 0x0040
#define FLAG_SF 0x0080
#define FLAG_TF 0x0100
#define FLAG_IF 0x0200
#define FLAG_DF 0x0400
#define FLAG_OF 0x0800
// 80286 FLAGS 高位：IOPL（bit12-13）、NT（bit14）。实模式下恒为 0，
// 保护模式下 POPF/IRET 只有在 CPL=0 时才能改 IOPL，NT 由任务切换维护。
#define FLAG_IOPL 0x3000
#define FLAG_NT   0x4000

extern CPU_8086 cpu;
extern uint8_t* memory;
extern uint32_t memory_size;
extern bool cpu_running;
extern bool cpu_halted;
extern uint64_t hlt_count;   // 客机累计执行过的 HLT 次数

// ---- 客机主频与三块时钟的关系 ----
//   主频（Turbo XT，8088 跑 20MHz）。cpu_cycles 按它计数，周期数由 cpu.c 的
//   8088 周期表给出，模拟器的"虚拟时间"全部由 cpu_cycles 推导。
//   注意：PIT 和 CGA 的时钟都来自"插在主板/卡上自己的晶振"，与 CPU 主频无关，
//   所以它们相对 CPU 周期数的比例必须随主频一起换算（8088 MPH 的速度自检就靠这个）。
#define CPU_CLK_HZ_DEFAULT 20000000u   // 缺省 CPU 主频 20MHz（Turbo）
extern uint32_t cpu_clk_hz;            // 当前客机主频（运行时可改）
#define CPU_CLK_HZ   cpu_clk_hz         // 计时换算统一走这个
#define PIT_CLK_HZ    1193182u   // 8253 输入时钟（主板 14.31818MHz 晶振 ÷12，固定值）
#define CGA_DOT_HZ    7159090u   // CGA 点时钟（卡上 14.31818MHz 晶振 ÷2，固定值）

extern uint64_t cpu_cycles;
extern uint32_t cpu_last_cycles;   // 刚执行完的那一步耗掉的周期数
// 上一条"指令"只是前缀字节（26/2E/36/3E/64/65/66/67/F0/F2/F3）：
//   本模拟器把前缀当独立一步执行，而真 8086 里"前缀+操作码"是一条指令，
//   中断绝不能在两半之间投递，否则中断处理程序会消费掉 seg_override，
//   前缀失效 → 下一条指令读错段（实测导致 DOS 跳到未初始化内存）。
extern bool cpu_prefix_step;
void cpu_init(void);
void cpu_reset(void);
void cpu_execute_instruction(void);
void cpu_interrupt(uint8_t int_num);
// 异常/故障报告（除零、无效指令、通用保护、双重/三重故障）
void cpu_fault_dump(const char* kind, uint8_t vector, const char* why);
void cpu_exception(uint8_t vector, const char* why);
void cpu_invalid_opcode(uint8_t op);
uint8_t get_reg8_val(uint8_t reg);
void set_reg8_val(uint8_t reg, uint8_t val);
uint8_t read_byte(uint16_t seg, uint16_t off);
uint16_t read_word(uint16_t seg, uint16_t off);
void write_byte(uint16_t seg, uint16_t off, uint8_t val);
void write_word(uint16_t seg, uint16_t off, uint16_t val);

// ---- 80286 系统级接口 ----
//   A20 地址线：AT 上电默认开启（IBM AT 的 8042 输出口 bit1 上电为 1）。
//   AT BIOS 的 POST（1D 的 >1MB 填充、1F 的别名检查）都要求 A20 已开；
//   只有 XT（8088）没有 A20 门，地址永远 20 位折回。
extern bool cpu_a20_enabled;
// 机器类型：AT（286，有 A20 门，上电默认开启 A20）还是 XT（8088，无 A20 门，
// 地址永远 20 位折回）。由 load_bios 按 ROM 尺寸判定（AT BIOS 是 64KB 整体）。
extern bool cpu_is_286;
void cpu_set_a20(bool on);
// 复位请求：设备（8042 输出端口的 0xFE 脉冲复位）只登记"要复位"，真正的复位由
// 主循环在指令边界上执行。复位若在一条 I/O 指令的中途生效，调用它的代码随后
// 还会按旧指令推进 IP/周期，复位后的取指位置就被弄错了（实测复位后没有从
// 0xFFFF:0000 取 EA 5B E0 00 F0 的复位向量，而是从 0xFFFF:0002 开始乱解码）。
extern bool cpu_reset_pending;
void cpu_request_reset(void);
// 物理内存读写：过 A20 门，越出已装内存范围读回 0xFF（板上无 RAM），
// ROM 区（0xF0000~0xFFFFF）写入被忽略。
// ★ 它们是每条指令都会走的热路径，必须以 static inline 放在头文件里：
//   之前是非内联函数，每次内存访问都要多一次函数调用（外加 read_byte/
//   read_word 本身也是函数），实测把 POST 阶段的墙钟时间拖长了好几倍。
// 板上已装 RAM 上限：AT POST 的扩展内存自检靠"写模式 / 读回比对"，
//   所以 >1MB 必须真的能存能取；若像 XT 那样读回 0xFF，POST 会判定没有扩展内存。
#define CPU_RAM_INSTALLED  0x200000u
extern bool cpu_rom_write_protect;
static inline uint8_t cpu_mem_read(uint32_t addr) {
	addr = cpu_a20_enabled ? (addr & 0xFFFFFF) : (addr & 0xFFFFF);
	if (addr >= CPU_RAM_INSTALLED || addr >= memory_size) return 0xFF;
	return memory[addr];
}
void cpu_mem_watch(uint32_t addr, uint8_t val);   // TEMP 写监视
static inline void cpu_mem_write(uint32_t addr, uint8_t val) {
	addr = cpu_a20_enabled ? (addr & 0xFFFFFF) : (addr & 0xFFFFF);
	if (addr >= CPU_RAM_INSTALLED || addr >= memory_size) return;
	if (cpu_rom_write_protect && addr >= 0xF0000 && addr < 0x100000) return;
	cpu_mem_watch(addr, val);
	memory[addr] = val;
}
// 段缓存查询（实模式基址 = 选择子 << 4；保护模式查描述符填的缓存）
uint32_t cpu_seg_base(uint16_t sel);
uint32_t cpu_seg_limit(uint16_t sel);
uint8_t  cpu_seg_access(uint16_t sel);
// 装载段寄存器：实模式直装；保护模式校验权限并填基址/限长缓存
void cpu_load_seg_reg(int idx, uint16_t sel);
// 取字符串指令的可覆盖源段（消耗段覆盖前缀），供 OUTS 使用
uint16_t cpu_get_seg_ds(void);
void push(uint16_t val);
uint16_t pop(void);
void set_flag(uint16_t flag);
void clear_flag(uint16_t flag);
bool get_flag(uint16_t flag);
// 供 cpu386.c 使用
extern uint16_t* reg16_table[8];
uint32_t decode_modrm_addr(uint8_t modrm, uint16_t* disp_out);
void handle_modrm_ip(uint8_t modrm);
uint8_t read_modrm8(uint8_t modrm);
uint16_t read_modrm16(uint8_t modrm);
void write_modrm8(uint8_t modrm, uint8_t val);
void write_modrm16(uint8_t modrm, uint16_t val);
uint8_t get_reg8_val(uint8_t reg);
void set_reg8_val(uint8_t reg, uint8_t val);
void load_bios(const char* filename);
void cpu_install_exception_stubs(void);

#endif