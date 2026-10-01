// protect.h - 80286 保护模式（描述符 / 段装载 / 门级中断 / 远转移 / TSS）
#ifndef PROTECT_H
#define PROTECT_H

#include <stdint.h>
#include <stdbool.h>

// 段描述符（8 字节）。
//   80286 描述符是它的子集：byte6/byte7 为保留位（必须为 0），因此
//   限长只剩 16 位（字节粒度）、基址 24 位。这里按 386 的超集格式解析，
//   286 描述符（byte6=0）自然退化成正确结果。
typedef struct {
	uint16_t limit_low;          // 段限长低 16 位
	uint16_t base_low;           // 段基址低 16 位
	uint8_t  base_mid;           // 段基址中 8 位（286 的最高 8 位）
	uint8_t  access;             // 访问权限
	uint8_t  limit_high_flags;   // 限长高 4 位 + 标志（286 恒为 0）
	uint8_t  base_high;          // 386 才有；286 恒为 0
} __attribute__((packed)) SegmentDescriptor;

// 门描述符（8 字节）
typedef struct {
	uint16_t offset_low;
	uint16_t selector;
	uint8_t  count;        // 286：固定 0
	uint8_t  type_attr;
	uint16_t offset_high;  // 286：固定 0
} __attribute__((packed)) GateDescriptor;

// access 字节
#define SEG_ACCESS_PRESENT   0x80
#define SEG_ACCESS_DPL0      0x00
#define SEG_ACCESS_DPL1      0x20
#define SEG_ACCESS_DPL2      0x40
#define SEG_ACCESS_DPL3      0x60
#define SEG_ACCESS_S         0x10   // 1 = 代码/数据，0 = 系统
#define SEG_ACCESS_CODE      0x08   // 代码段
#define SEG_ACCESS_CONFORM   0x04   // 代码：一致段
#define SEG_ACCESS_EXPAND    0x04   // 数据：向下扩展
#define SEG_ACCESS_RW        0x02   // 代码：可读；数据：可写
#define SEG_ACCESS_ACCESSED  0x01

// 系统段类型（S=0）
#define SYS_TYPE_LDT         0x02
#define SYS_TYPE_TSS16_BUSY  0x03
#define SYS_TYPE_TSS16       0x01
#define SYS_TYPE_CALLGATE    0x04
#define SYS_TYPE_TASKGATE    0x05
#define SYS_TYPE_INTGATE     0x06
#define SYS_TYPE_TRAPGATE    0x07
#define SYS_TYPE_INTGATE32   0x0E
#define SYS_TYPE_TRAPGATE32  0x0F

// 异常向量
#define EXC_DE   0    // 除零
#define EXC_DB   1    // 调试
#define EXC_NMI  2    // 不可屏蔽中断
#define EXC_BP   3    // 断点
#define EXC_OF   4    // 溢出
#define EXC_BR   5    // 边界
#define EXC_UD   6    // 无效操作码
#define EXC_NM   7    // 设备不可用
#define EXC_DF   8    // 双重故障
#define EXC_TS   10   // 无效 TSS
#define EXC_NP   11   // 段不存在
#define EXC_SS   12   // 栈段错误
#define EXC_GP   13   // 通用保护
#define EXC_PF   14   // 页故障
#define EXC_MF   16   // 浮点错误
#define EXC_AC   17   // 对齐检查

// ---- 80286 系统寄存器状态 ----
typedef struct {
	bool     pe;          // MSW/CR0 的 PE 位
	uint16_t msw;         // 机器状态字（286 读回高 12 位为 1）
	uint32_t gdt_base;    // 24 位
	uint16_t gdt_limit;
	uint32_t idt_base;
	uint16_t idt_limit;
	uint16_t ldtr;        // LDT 选择子
	uint32_t ldtr_base;   // 由 LDT 描述符填入
	uint16_t ldtr_limit;
	uint16_t tr;          // TSS 选择子
	uint32_t tr_base;     // 由 TSS 描述符填入
	uint16_t tr_limit;
} ProtectState286;

extern ProtectState286 protect;

// ---- 保护模式流程日志（默认关闭，只对 -dbg 生效）----
#include <stdio.h>
extern bool debug_mode;
#define PMLOG(...) do { if (debug_mode) { \
	fprintf(stderr, __VA_ARGS__); fflush(stderr); } } while (0)

void protect_init(void);
void protect_reset(void);
void protect_set_pe(bool on);
int  protect_cpl(void);           // 当前特权级 = CS 的 RPL 位

// 描述符读取与字段解析
void protect_read_desc(uint32_t addr, SegmentDescriptor* d);
void protect_read_gate(uint32_t addr, GateDescriptor* g);
uint32_t desc_base(const SegmentDescriptor* d);
uint32_t desc_limit(const SegmentDescriptor* d);

// 装载段寄存器。成功时回填 base/limit/access 并返回 true；
// 失败时已投放异常（#GP/#NP/#SS）并返回 false。
bool protect_load_seg(int idx, uint16_t sel, uint32_t* base, uint32_t* limit, uint8_t* access);

// 0F 00 组的辅助：LLDT / LTR / VERR / VERW
bool protect_load_ldt(uint16_t sel);
bool protect_load_tr(uint16_t sel);
bool protect_verify_sel(uint16_t sel, bool for_write);

// 0F 02/03 组的辅助：LAR / LSL（只读取，不产生异常，失败返回 false）
bool protect_sel_access(uint16_t sel, uint8_t* access);
bool protect_sel_limit(uint16_t sel, uint32_t* limit);

// 保护模式的系统事件（cpu.c 在 PE=1 时改走这些路径）
void protect_far_jmp(uint16_t sel, uint16_t off);
void protect_far_call(uint16_t sel, uint16_t off);
void protect_far_ret(bool is_iret, uint16_t imm);   // RETF/RETF imm/IRET
void protect_interrupt(uint8_t vector, bool has_error, uint16_t error_code, bool is_soft);
void protect_exception(int vector, uint16_t error_code);
void protect_task_switch(uint16_t tss_sel);
void protect_fault_shutdown(const char* why);

#endif