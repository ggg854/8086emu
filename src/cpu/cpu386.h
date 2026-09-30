// cpu386.h - 386 指令超集（本机以 80286 为基准）
#ifndef CPU386_H
#define CPU386_H

#include <stdint.h>
#include <stdbool.h>

// 系统状态（PE / GDT / IDT / LDT / TSS）统一由 protect.h 的 ProtectState286 持有；
// 这里只保留 386 操作数/地址长度前缀的记录位。
typedef struct {
	bool prefix_66;   // 32 位操作数前缀（本机 16 位，仅记录）
	bool prefix_67;   // 32 位地址前缀
} CPU386State;

extern CPU386State cpu386;

void cpu386_init(void);
void cpu386_reset(void);
void cpu386_execute_0f(void);
void cpu386_execute_opcode(uint8_t opcode);

#endif