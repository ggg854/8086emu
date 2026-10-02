// cpu386.c - 80286 之上的 386 指令超集
//   说明：本模拟器以 80286 为基准（16 位操作数、286 描述符/门/TSS），
//   另保留一批 386 指令作为超集（用户要求）。系统寄存器（MSW/GDT/IDT/
//   LDT/TSS）统一由 protect.c 持有，这里只做指令解码与执行。
//
//   ★ IP 约定：cpu386_execute_0f 进入时 IP 指向 0F，函数一开头就 +1 让 IP
//     指向第二个操作码字节，于是 cpu.c 的 ModR/M 约定（modrm 在 IP+1、
//     disp 在 IP+2）继续成立，decode_modrm_addr/read_modrm*/write_modrm* 可
//     直接复用（否则带位移的内存操作数会读错字节）。
#include "cpu386.h"
#include "cpu.h"
#include "protect.h"
#include "io.h"
#include <stdio.h>
#include <string.h>
#include <time.h>
CPU386State cpu386;

// ============================================================
// 初始化 / 复位
// ============================================================
void cpu386_init(void) {
	memset(&cpu386, 0, sizeof(cpu386));
}

void cpu386_reset(void) {
	cpu386_init();
}

// ============================================================
// 0x0F 双字节
// ============================================================
void cpu386_execute_0f(void) {
	uint32_t addr = cpu_seg_base(cpu.cs) + cpu.ip;   // IP 指向 0F
	uint8_t op2 = cpu_mem_read(addr + 1);
	cpu.ip += 1;                                     // ★ 让 IP 指向 op2

	// 0F 80~8F：Jcc rel16
	if (op2 >= 0x80 && op2 <= 0x8F) {
		int16_t rel = (int16_t)read_word(cpu.cs, cpu.ip + 1);
		cpu.ip += 3;
		bool take = false;
		switch (op2) {
			case 0x80: take = get_flag(FLAG_OF); break;
			case 0x81: take = !get_flag(FLAG_OF); break;
			case 0x82: take = get_flag(FLAG_CF); break;
			case 0x83: take = !get_flag(FLAG_CF); break;
			case 0x84: take = get_flag(FLAG_ZF); break;
			case 0x85: take = !get_flag(FLAG_ZF); break;
			case 0x86: take = get_flag(FLAG_CF) || get_flag(FLAG_ZF); break;
			case 0x87: take = !get_flag(FLAG_CF) && !get_flag(FLAG_ZF); break;
			case 0x88: take = get_flag(FLAG_SF); break;
			case 0x89: take = !get_flag(FLAG_SF); break;
			case 0x8A: take = get_flag(FLAG_PF); break;
			case 0x8B: take = !get_flag(FLAG_PF); break;
			case 0x8C: take = get_flag(FLAG_SF) != get_flag(FLAG_OF); break;
			case 0x8D: take = get_flag(FLAG_SF) == get_flag(FLAG_OF); break;
			case 0x8E: take = get_flag(FLAG_ZF) || (get_flag(FLAG_SF) != get_flag(FLAG_OF)); break;
			case 0x8F: take = !get_flag(FLAG_ZF) && (get_flag(FLAG_SF) == get_flag(FLAG_OF)); break;
		}
		if (take) cpu.ip = (uint16_t)(cpu.ip + rel);
		return;
	}

	switch (op2) {
		// ---- 286 系统指令：SLDT / STR / LLDT / LTR / VERR / VERW (0F 00) ----
		case 0x00: {
			uint8_t modrm = cpu_mem_read(addr + 2);
			uint8_t reg = (modrm >> 3) & 0x07;
			uint8_t mod = (modrm >> 6) & 0x03;
			uint16_t sel = (mod == 3) ? *reg16_table[modrm & 0x07] : read_modrm16(modrm);
			switch (reg) {
				case 0:  // SLDT
					if (mod == 3) *reg16_table[modrm & 0x07] = protect.ldtr;
					else write_modrm16(modrm, protect.ldtr);
					break;
				case 1:  // STR
					if (mod == 3) *reg16_table[modrm & 0x07] = protect.tr;
					else write_modrm16(modrm, protect.tr);
					break;
				case 2:  // LLDT
					if (!protect_load_ldt(sel)) cpu_invalid_opcode(0x00);
					break;
				case 3:  // LTR
					if (!protect_load_tr(sel)) cpu_invalid_opcode(0x00);
					break;
				case 4:  // VERR
					if (protect_verify_sel(sel, false)) set_flag(FLAG_ZF);
					else clear_flag(FLAG_ZF);
					break;
				case 5:  // VERW
					if (protect_verify_sel(sel, true)) set_flag(FLAG_ZF);
					else clear_flag(FLAG_ZF);
					break;
				default:
					cpu_invalid_opcode(0x00);
					break;
			}
			handle_modrm_ip(modrm);
			cpu.ip += 2;
			break;
		}

		// ---- 286 系统指令：SGDT/SIDT/LGDT/LIDT/SMSW/LMSW (0F 01) ----
		case 0x01: {
			uint8_t modrm = cpu_mem_read(addr + 2);
			uint8_t reg = (modrm >> 3) & 0x07;
			uint8_t rm = modrm & 0x07;
			uint8_t mod = (modrm >> 6) & 0x03;
			if (mod == 3) {
				// SMSW / LMSW：286 上只有 reg=4/6 合法
				if (reg == 4) {
					*reg16_table[rm] = protect.msw;
				} else if (reg == 6) {
					uint16_t val = *reg16_table[rm];
					protect.msw = val;
					protect_set_pe((val & 0x0001) != 0);
				} else {
					cpu_invalid_opcode(0x01);
					break;
				}
				cpu.ip += 2;
			} else {
				// 286 伪描述符是 5 字节：2 字节限长 + 3 字节 24 位基址
				uint32_t ea = decode_modrm_addr(modrm, NULL);
				handle_modrm_ip(modrm);
				cpu.ip += 2;
				if (reg == 0 || reg == 1) {           // SGDT / SIDT
					uint32_t base = (reg == 0) ? protect.gdt_base : protect.idt_base;
					uint16_t lim  = (reg == 0) ? protect.gdt_limit : protect.idt_limit;
					cpu_mem_write(ea, (uint8_t)(lim & 0xFF));
					cpu_mem_write(ea + 1, (uint8_t)(lim >> 8));
					cpu_mem_write(ea + 2, (uint8_t)(base & 0xFF));
					cpu_mem_write(ea + 3, (uint8_t)((base >> 8) & 0xFF));
					cpu_mem_write(ea + 4, (uint8_t)((base >> 16) & 0xFF));
				} else if (reg == 2 || reg == 3) {     // LGDT / LIDT
					uint16_t lim = (uint16_t)(cpu_mem_read(ea) | (cpu_mem_read(ea + 1) << 8));
					uint32_t base = (uint32_t)cpu_mem_read(ea + 2) |
					                ((uint32_t)cpu_mem_read(ea + 3) << 8) |
					                ((uint32_t)cpu_mem_read(ea + 4) << 16);
					PMLOG("[PM] %s base=%06X lim=%04X @%04X:%04X\n",
					      (reg == 2) ? "LGDT" : "LIDT", base, lim, cpu.cs, cpu.ip);
					if (reg == 2) { protect.gdt_base = base; protect.gdt_limit = lim; }
					else          { protect.idt_base = base; protect.idt_limit = lim; }
				} else if (reg == 7) {                 // INVLPG（386 超集；无分页 → 空操作）
					;
				} else {
					cpu_invalid_opcode(0x01);
				}
			}
			break;
		}

		// ---- LAR (0F 02)：读访问权字节 ----
		case 0x02: {
			uint8_t modrm = cpu_mem_read(addr + 2);
			uint8_t reg = (modrm >> 3) & 0x07;
			uint16_t sel = read_modrm16(modrm);
			handle_modrm_ip(modrm);
			cpu.ip += 2;
			uint8_t acc;
			if (protect_sel_access(sel, &acc)) {
				*reg16_table[reg] = (uint16_t)(acc << 8);
				set_flag(FLAG_ZF);
			} else {
				clear_flag(FLAG_ZF);
			}
			break;
		}

		// ---- LSL (0F 03)：读段限长 ----
		case 0x03: {
			uint8_t modrm = cpu_mem_read(addr + 2);
			uint8_t reg = (modrm >> 3) & 0x07;
			uint16_t sel = read_modrm16(modrm);
			handle_modrm_ip(modrm);
			cpu.ip += 2;
			uint32_t lim;
			if (protect_sel_limit(sel, &lim)) {
				*reg16_table[reg] = (uint16_t)lim;
				set_flag(FLAG_ZF);
			} else {
				clear_flag(FLAG_ZF);
			}
			break;
		}

		// ---- CLTS (0F 06)：清 MSW.TS ----
		case 0x06:
			protect.msw &= ~0x0008;
			cpu.ip += 1;
			break;

		// ---- MOV r16, CRx (0F 20) ----
		case 0x20: {
			uint8_t modrm = cpu_mem_read(addr + 2);
			uint8_t cr = (modrm >> 3) & 0x07;
			uint8_t rm = modrm & 0x07;
			*reg16_table[rm] = (cr == 0) ? protect.msw : 0;
			cpu.ip += 2;
			break;
		}

		// ---- MOV CRx, r16 (0F 22) ----
		case 0x22: {
			uint8_t modrm = cpu_mem_read(addr + 2);
			uint8_t cr = (modrm >> 3) & 0x07;
			uint8_t rm = modrm & 0x07;
			if (cr == 0) {
				protect.msw = *reg16_table[rm];
				protect_set_pe((protect.msw & 0x0001) != 0);
			}
			cpu.ip += 2;
			break;
		}

		// ---- RDTSC (0F 31) ----
		case 0x31: {
			uint64_t t = (uint64_t)time(NULL);
			cpu.ax = (uint16_t)(t & 0xFFFF);
			cpu.dx = (uint16_t)((t >> 16) & 0xFFFF);
			cpu.ip += 1;
			break;
		}

		// ---- WRMSR (0F 30) / RDMSR (0F 32) ----
		case 0x30:
			cpu.ip += 1;
			break;
		case 0x32:
			cpu.ax = 0;
			cpu.dx = 0;
			cpu.ip += 1;
			break;

		// ---- CPUID (0F A2) ----
		case 0xA2: {
			cpu.ax = 1;
			cpu.bx = 0x756E;  // "nu"
			cpu.cx = 0x6C65;  // "el"
			cpu.dx = 0x4965;  // "eI"
			cpu.ip += 1;
			break;
		}

		// ---- BT / BTS / BTR / BTC (0F A3 / AB / B3 / BB) ----
		case 0xA3: case 0xAB: case 0xB3: case 0xBB: {
			uint8_t modrm = cpu_mem_read(addr + 2);
			uint8_t reg = (modrm >> 3) & 0x07;
			uint16_t bit = *reg16_table[reg] & 0x0F;
			uint16_t val = read_modrm16(modrm);
			bool cf = (val >> bit) & 1;
			if (cf) set_flag(FLAG_CF); else clear_flag(FLAG_CF);
			if (op2 == 0xAB) val |= (uint16_t)(1u << bit);       // BTS
			if (op2 == 0xB3) val &= (uint16_t)~(1u << bit);      // BTR
			if (op2 == 0xBB) val ^= (uint16_t)(1u << bit);       // BTC
			if (op2 != 0xA3) write_modrm16(modrm, val);
			handle_modrm_ip(modrm);
			cpu.ip += 2;
			break;
		}

		// ---- MOVZX (0F B6 / B7) ----
		case 0xB6: case 0xB7: {
			uint8_t modrm = cpu_mem_read(addr + 2);
			uint8_t reg = (modrm >> 3) & 0x07;
			if (op2 == 0xB6) *reg16_table[reg] = read_modrm8(modrm);
			else             *reg16_table[reg] = read_modrm16(modrm);
			handle_modrm_ip(modrm);
			cpu.ip += 2;
			break;
		}

		// ---- MOVSX (0F BE / BF) ----
		case 0xBE: case 0xBF: {
			uint8_t modrm = cpu_mem_read(addr + 2);
			uint8_t reg = (modrm >> 3) & 0x07;
			if (op2 == 0xBE) *reg16_table[reg] = (uint16_t)(int16_t)(int8_t)read_modrm8(modrm);
			else             *reg16_table[reg] = read_modrm16(modrm);
			handle_modrm_ip(modrm);
			cpu.ip += 2;
			break;
		}

		// ---- IMUL r16, r/m16 (0F AF) ----
		case 0xAF: {
			uint8_t modrm = cpu_mem_read(addr + 2);
			uint8_t reg = (modrm >> 3) & 0x07;
			int32_t result = (int16_t)*reg16_table[reg] * (int16_t)read_modrm16(modrm);
			*reg16_table[reg] = (uint16_t)result;
			if (result < -32768 || result > 32767) { set_flag(FLAG_CF); set_flag(FLAG_OF); }
			else { clear_flag(FLAG_CF); clear_flag(FLAG_OF); }
			handle_modrm_ip(modrm);
			cpu.ip += 2;
			break;
		}

		// ---- SETcc (0F 90~9F) ----
		case 0x90: case 0x91: case 0x92: case 0x93:
		case 0x94: case 0x95: case 0x96: case 0x97:
		case 0x98: case 0x99: case 0x9A: case 0x9B:
		case 0x9C: case 0x9D: case 0x9E: case 0x9F: {
			uint8_t modrm = cpu_mem_read(addr + 2);
			bool take = false;
			uint8_t cc = op2 & 0x0F;
			switch (cc) {
				case 0x0: take = get_flag(FLAG_OF); break;
				case 0x1: take = !get_flag(FLAG_OF); break;
				case 0x2: take = get_flag(FLAG_CF); break;
				case 0x3: take = !get_flag(FLAG_CF); break;
				case 0x4: take = get_flag(FLAG_ZF); break;
				case 0x5: take = !get_flag(FLAG_ZF); break;
				case 0x6: take = get_flag(FLAG_CF) || get_flag(FLAG_ZF); break;
				case 0x7: take = !get_flag(FLAG_CF) && !get_flag(FLAG_ZF); break;
				case 0x8: take = get_flag(FLAG_SF); break;
				case 0x9: take = !get_flag(FLAG_SF); break;
				case 0xA: take = get_flag(FLAG_PF); break;
				case 0xB: take = !get_flag(FLAG_PF); break;
				case 0xC: take = get_flag(FLAG_SF) != get_flag(FLAG_OF); break;
				case 0xD: take = get_flag(FLAG_SF) == get_flag(FLAG_OF); break;
				case 0xE: take = get_flag(FLAG_ZF) || (get_flag(FLAG_SF) != get_flag(FLAG_OF)); break;
				case 0xF: take = !get_flag(FLAG_ZF) && (get_flag(FLAG_SF) == get_flag(FLAG_OF)); break;
			}
			write_modrm8(modrm, take ? 1 : 0);
			handle_modrm_ip(modrm);
			cpu.ip += 2;
			break;
		}

		// ---- BSF / BSR (0F BC / BD) ----
		case 0xBC: case 0xBD: {
			uint8_t modrm = cpu_mem_read(addr + 2);
			uint8_t reg = (modrm >> 3) & 0x07;
			uint16_t val = read_modrm16(modrm);
			if (val == 0) {
				set_flag(FLAG_ZF);
			} else {
				clear_flag(FLAG_ZF);
				int bit = -1;
				if (op2 == 0xBC) { for (int i = 0; i < 16; i++) if (val & (1 << i)) { bit = i; break; } }
				else             { for (int i = 15; i >= 0; i--) if (val & (1 << i)) { bit = i; break; } }
				*reg16_table[reg] = (uint16_t)bit;
			}
			handle_modrm_ip(modrm);
			cpu.ip += 2;
			break;
		}

		// ---- BSWAP（386；本机 16 位，空转） ----
		case 0xC8: case 0xC9: case 0xCA: case 0xCB:
		case 0xCC: case 0xCD: case 0xCE: case 0xCF:
			cpu.ip += 1;
			break;

		// ---- XADD (0F C0 / C1) ----
		case 0xC0: case 0xC1: {
			uint8_t modrm = cpu_mem_read(addr + 2);
			uint8_t reg = (modrm >> 3) & 0x07;
			if (op2 == 0xC0) {
				uint8_t a = read_modrm8(modrm);
				uint8_t b = get_reg8_val(reg);
				write_modrm8(modrm, (uint8_t)(a + b));
				set_reg8_val(reg, a);
			} else {
				uint16_t a = read_modrm16(modrm);
				uint16_t b = *reg16_table[reg];
				write_modrm16(modrm, (uint16_t)(a + b));
				*reg16_table[reg] = a;
			}
			handle_modrm_ip(modrm);
			cpu.ip += 2;
			break;
		}

		// ---- SHLD / SHRD (0F A4 / A5 / AC / AD) ----
		case 0xA4: case 0xA5: case 0xAC: case 0xAD: {
			uint8_t modrm = cpu_mem_read(addr + 2);
			uint8_t reg = (modrm >> 3) & 0x07;
			uint8_t imm = cpu_mem_read(addr + 3) & 0x1F;
			if (imm > 16) imm = 16;
			if (op2 == 0xA4 || op2 == 0xA5) {
				uint16_t val = read_modrm16(modrm);
				uint16_t src = *reg16_table[reg];
				uint32_t combined = ((uint32_t)val << 16) | src;
				write_modrm16(modrm, (uint16_t)((combined << imm) >> 16));
			} else {
				uint16_t val = read_modrm16(modrm);
				uint16_t src = *reg16_table[reg];
				uint32_t combined = ((uint32_t)src << 16) | val;
				write_modrm16(modrm, (uint16_t)(combined >> imm));
			}
			handle_modrm_ip(modrm);
			cpu.ip += 3;   // op2 + modrm + imm8
			break;
		}

		// ---- CMPXCHG (0F B0 / B1) ----
		case 0xB0: case 0xB1: {
			uint8_t modrm = cpu_mem_read(addr + 2);
			uint8_t reg = (modrm >> 3) & 0x07;
			if (op2 == 0xB0) {
				uint8_t a = cpu.ax & 0xFF;
				uint8_t b = read_modrm8(modrm);
				if (a == b) { set_flag(FLAG_ZF); write_modrm8(modrm, get_reg8_val(reg)); }
				else { clear_flag(FLAG_ZF); cpu.ax = (uint16_t)((cpu.ax & 0xFF00) | b); }
			} else {
				uint16_t a = cpu.ax;
				uint16_t b = read_modrm16(modrm);
				if (a == b) { set_flag(FLAG_ZF); write_modrm16(modrm, *reg16_table[reg]); }
				else { clear_flag(FLAG_ZF); cpu.ax = b; }
			}
			handle_modrm_ip(modrm);
			cpu.ip += 2;
			break;
		}

		// ---- LSS / LFS / LGS (0F B2 / B4 / B5) ----
		case 0xB2: case 0xB4: case 0xB5: {
			uint8_t modrm = cpu_mem_read(addr + 2);
			uint8_t reg = (modrm >> 3) & 0x07;
			uint32_t ea = decode_modrm_addr(modrm, NULL);
			uint16_t off = (uint16_t)(cpu_mem_read(ea) | (cpu_mem_read(ea + 1) << 8));
			uint16_t seg = (uint16_t)(cpu_mem_read(ea + 2) | (cpu_mem_read(ea + 3) << 8));
			handle_modrm_ip(modrm);
			cpu.ip += 2;
			*reg16_table[reg] = off;
			if (op2 == 0xB2)      cpu_load_seg_reg(SEG_SS, seg);
			else if (op2 == 0xB4) cpu_load_seg_reg(SEG_FS, seg);
			else                  cpu_load_seg_reg(SEG_GS, seg);
			break;
		}

		// ---- NOP (0F 1F) ----
		case 0x1F: {
			uint8_t modrm = cpu_mem_read(addr + 2);
			handle_modrm_ip(modrm);
			cpu.ip += 2;
			break;
		}

		default:
			printf("[CPU] unimplemented 0F %02X @ CS:%04X IP:%04X\n", op2, cpu.cs, cpu.ip);
			cpu_invalid_opcode(op2);
			break;
	}
}

// ============================================================
// 单字节扩展（286 新指令 + 386 超集）
//   注意：0x68/0x69/0x6A/0x6B 已在 cpu.c 的 switch 中实现，这里不再重复。
// ============================================================
void cpu386_execute_opcode(uint8_t opcode) {
	uint32_t addr = cpu_seg_base(cpu.cs) + cpu.ip;

	switch (opcode) {
		// PUSHA (0x60)：286 语义，压入的是"减 2 之后"的 SP
		case 0x60: {
			uint16_t sp = cpu.sp;
			push(cpu.ax);
			push(cpu.cx);
			push(cpu.dx);
			push(cpu.bx);
			push(sp);
			push(cpu.bp);
			push(cpu.si);
			push(cpu.di);
			cpu.ip += 1;
			break;
		}

		// POPA (0x61)
		case 0x61: {
			cpu.di = pop();
			cpu.si = pop();
			cpu.bp = pop();
			pop();  // SP 丢弃
			cpu.bx = pop();
			cpu.dx = pop();
			cpu.cx = pop();
			cpu.ax = pop();
			cpu.ip += 1;
			break;
		}

		// BOUND r16, m16&16 (0x62)：索引越界 → #BR
		case 0x62: {
			uint8_t modrm = cpu_mem_read(addr + 1);
			uint8_t reg = (modrm >> 3) & 0x07;
			uint8_t mod = (modrm >> 6) & 0x03;
			if (mod == 3) { cpu_invalid_opcode(0x62); break; }
			uint32_t ea = decode_modrm_addr(modrm, NULL);
			int16_t idx = (int16_t)*reg16_table[reg];
			int16_t lo = (int16_t)(cpu_mem_read(ea) | (cpu_mem_read(ea + 1) << 8));
			int16_t hi = (int16_t)(cpu_mem_read(ea + 2) | (cpu_mem_read(ea + 3) << 8));
			cpu.ip += 2;
			handle_modrm_ip(modrm);
			if (idx < lo || idx > hi) {
				cpu_exception(EXC_BR, "BOUND 越界（索引超出上下界）");
			}
			break;
		}

		// ARPL r/m16, r16 (0x63)：只有保护模式且 CPL=0 之外才有意义；
		//   RPL 位比目标低时提升它，并置 ZF。
		case 0x63: {
			uint8_t modrm = cpu_mem_read(addr + 1);
			uint8_t reg = (modrm >> 3) & 0x07;
			uint8_t mod = (modrm >> 6) & 0x03;
			uint16_t src = *reg16_table[reg];
			uint16_t dst = (mod == 3) ? *reg16_table[modrm & 0x07] : read_modrm16(modrm);
			cpu.ip += 2;
			handle_modrm_ip(modrm);
			if (mod == 3 && (modrm & 0x07) == 0x04) { cpu_invalid_opcode(0x63); break; }
			if ((dst & 0x0003) < (src & 0x0003)) {
				dst = (uint16_t)((dst & 0xFFFC) | (src & 0x0003));
				if (mod == 3) *reg16_table[modrm & 0x07] = dst;
				else write_modrm16(modrm, dst);
				set_flag(FLAG_ZF);
			} else {
				clear_flag(FLAG_ZF);
			}
			break;
		}

		// INS / OUTS（0x6C~0x6F）：286 字符串 I/O，端点为 DX
		case 0x6C: {   // INSB
			uint8_t v = io_read_port(cpu.dx);
			write_byte(cpu.es, cpu.di, v);
			if (get_flag(FLAG_DF)) cpu.di--; else cpu.di++;
			cpu.ip += 1;
			break;
		}
		case 0x6D: {   // INSW
			uint16_t v = io_read_port16(cpu.dx);
			write_word(cpu.es, cpu.di, v);
			if (get_flag(FLAG_DF)) cpu.di -= 2; else cpu.di += 2;
			cpu.ip += 1;
			break;
		}
		case 0x6E: {   // OUTSB
			io_write_port(cpu.dx, read_byte(cpu_get_seg_ds(), cpu.si));
			if (get_flag(FLAG_DF)) cpu.si--; else cpu.si++;
			cpu.ip += 1;
			break;
		}
		case 0x6F: {   // OUTSW
			io_write_port16(cpu.dx, read_word(cpu_get_seg_ds(), cpu.si));
			if (get_flag(FLAG_DF)) cpu.si -= 2; else cpu.si += 2;
			cpu.ip += 1;
			break;
		}

		default:
			printf("[CPU] unimplemented opcode 0x%02X @ CS:%04X IP:%04X\n", opcode, cpu.cs, cpu.ip);
			cpu_invalid_opcode(opcode);
			break;
	}
}