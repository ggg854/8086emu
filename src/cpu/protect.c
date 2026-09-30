// protect.c - 80286 保护模式实现
//   描述符（8 字节 / 24 位基址）、段装载与权限检查、门级中断与陷阱、
//   远 CALL/JMP/RET 与特权级栈切换、16 位 TSS 任务切换。
#include "protect.h"
#include "cpu.h"
#include <stdio.h>
#include <string.h>

ProtectState286 protect;
int pm_log_budget = 600;   // TEMP

int protect_cpl(void) { return cpu.cs & 0x0003; }

void protect_init(void) {
	memset(&protect, 0, sizeof(protect));
	protect.msw = 0xFFF0;   // 286 MSW 复位值：高 12 位为 1、PE=0
	protect.pe = false;
}

void protect_reset(void) { protect_init(); }

void protect_set_pe(bool on) {
	if (on && !protect.pe) PMLOG("[PM] enter PE @%04X:%04X\n", cpu.cs, cpu.ip);
	if (!on && protect.pe) PMLOG("[PM] leave PE @%04X:%04X\n", cpu.cs, cpu.ip);
	protect.pe = on;
	if (on) protect.msw |= 0x0001; else protect.msw &= ~0x0001;
}

// ============================================================
// 描述符读取 / 字段解析
// ============================================================
void protect_read_desc(uint32_t addr, SegmentDescriptor* d) {
	uint8_t* p = (uint8_t*)d;
	for (int i = 0; i < 8; i++) p[i] = cpu_mem_read(addr + i);
}

void protect_read_gate(uint32_t addr, GateDescriptor* g) {
	uint8_t* p = (uint8_t*)g;
	for (int i = 0; i < 8; i++) p[i] = cpu_mem_read(addr + i);
}

uint32_t desc_base(const SegmentDescriptor* d) {
	return (uint32_t)d->base_low | ((uint32_t)d->base_mid << 16) | ((uint32_t)d->base_high << 24);
}

uint32_t desc_limit(const SegmentDescriptor* d) {
	uint32_t lim = (uint32_t)d->limit_low | ((uint32_t)(d->limit_high_flags & 0x0F) << 16);
	if (d->limit_high_flags & 0x80) lim = (lim << 12) | 0xFFF;   // 386 的 4KB 粒度
	return lim;
}

// 按选择子取出描述符及其基址/限长。touch=true 时置访问位。
//   返回 false 表示已投放异常。
static bool read_desc_sel(uint16_t sel, SegmentDescriptor* d, uint32_t* base,
                          uint32_t* limit, uint32_t* daddr, bool touch) {
	uint32_t tbl_base;
	uint16_t tbl_limit;
	if (sel & 0x0004) { tbl_base = protect.ldtr_base; tbl_limit = protect.ldtr_limit; }
	else              { tbl_base = protect.gdt_base;  tbl_limit = protect.gdt_limit;  }

	uint32_t index = (sel & 0xFFF8u) >> 3;
	uint32_t addr = tbl_base + index * 8;
	if (index * 8 + 7 > (uint32_t)tbl_limit) { protect_exception(EXC_GP, sel & 0xFFFC); return false; }

	protect_read_desc(addr, d);
	if (!(d->access & SEG_ACCESS_PRESENT)) {
		{	// TEMP NP-DBG
			static int once = 0;
			if (!once) {
				once = 1;
				fprintf(stderr, "[NP-DBG] pe=%d gdt=%06X/%04X idt=%06X/%04X\n",
				        protect.pe, protect.gdt_base, protect.gdt_limit,
				        protect.idt_base, protect.idt_limit);
				fprintf(stderr, "[NP-DBG] cs=%04X csbase=%06X ip=%04X ss=%04X ds=%04X es=%04X tr=%04X\n",
				        cpu.cs, cpu.seg_base[SEG_CS], cpu.ip, cpu.ss, cpu.ds, cpu.es, protect.tr);
				fprintf(stderr, "[NP-DBG] sel=%04X addr=%06X ??\n", sel, addr);
				for (int t = 0; t < 4; t++) {
					uint32_t a = protect.gdt_base + t * 8;
					fprintf(stderr, "[NP-DBG] GDT[%d] @%06X: %02X %02X %02X %02X %02X %02X %02X %02X\n",
					        t, a, cpu_mem_read(a), cpu_mem_read(a+1), cpu_mem_read(a+2),
					        cpu_mem_read(a+3), cpu_mem_read(a+4), cpu_mem_read(a+5),
					        cpu_mem_read(a+6), cpu_mem_read(a+7));
				}
				for (int t = 7; t <= 9; t++) {
					uint32_t a = protect.gdt_base + t * 8;
					fprintf(stderr, "[NP-DBG] GDT[%d] @%06X: %02X %02X %02X %02X %02X %02X %02X %02X\n",
					        t, a, cpu_mem_read(a), cpu_mem_read(a+1), cpu_mem_read(a+2),
					        cpu_mem_read(a+3), cpu_mem_read(a+4), cpu_mem_read(a+5),
					        cpu_mem_read(a+6), cpu_mem_read(a+7));
				}
				for (int t = 8; t <= 11; t++) {
					uint32_t a = protect.idt_base + t * 8;
					fprintf(stderr, "[NP-DBG] IDT[%d] @%06X: %02X %02X %02X %02X %02X %02X %02X %02X\n",
					        t, a, cpu_mem_read(a), cpu_mem_read(a+1), cpu_mem_read(a+2),
					        cpu_mem_read(a+3), cpu_mem_read(a+4), cpu_mem_read(a+5),
					        cpu_mem_read(a+6), cpu_mem_read(a+7));
				}
				uint32_t sb = cpu_seg_base(cpu.ss) + cpu.sp;
				fprintf(stderr, "[NP-DBG] stack @%06X:", sb);
				for (int t = -2; t < 6; t++)
					fprintf(stderr, " %04X", cpu_mem_read(sb + t*2) | (cpu_mem_read(sb + t*2 + 1) << 8));
				fprintf(stderr, "\n");
				fflush(stderr);
			}
		}
		protect_exception(EXC_NP, sel & 0xFFFC); return false;
	}
	if (touch && !(d->access & SEG_ACCESS_ACCESSED)) {
		d->access |= SEG_ACCESS_ACCESSED;
		cpu_mem_write(addr + 5, d->access);
	}
	*base = desc_base(d);
	*limit = desc_limit(d);
	if (daddr) *daddr = addr;
	return true;
}

bool protect_load_seg(int idx, uint16_t sel, uint32_t* base, uint32_t* limit, uint8_t* access) {
	// NULL 选择子：286 上只有 DS/ES/FS/GS 能装 NULL；CS/SS 装 NULL 是 #GP
	if ((sel & 0xFFF8u) == 0) {
		if (idx == SEG_CS || idx == SEG_SS) { protect_exception(EXC_GP, 0); return false; }
		*base = 0; *limit = 0; *access = 0;
		return true;
	}

	SegmentDescriptor d;
	if (!read_desc_sel(sel, &d, base, limit, NULL, true)) return false;
	*access = d.access;

	uint8_t cpl = (uint8_t)protect_cpl();
	uint8_t rpl = sel & 3;
	uint8_t dpl = (d.access >> 5) & 3;
	bool sys = !(d.access & SEG_ACCESS_S);
	bool code = (d.access & SEG_ACCESS_CODE) != 0;
	bool conform = (d.access & SEG_ACCESS_CONFORM) != 0;

	if (sys) { protect_exception(EXC_GP, sel & 0xFFFC); return false; }   // 系统段不能当普通段

	if (idx == SEG_CS) {
		if (!code) { protect_exception(EXC_GP, sel & 0xFFFC); return false; }
		if (conform) { if (dpl > cpl) { protect_exception(EXC_GP, sel & 0xFFFC); return false; } }
		else         { if (dpl != cpl || rpl != cpl) { protect_exception(EXC_GP, sel & 0xFFFC); return false; } }
	} else if (idx == SEG_SS) {
		if (code || !(d.access & SEG_ACCESS_RW)) { protect_exception(EXC_GP, sel & 0xFFFC); return false; }
		if (dpl != cpl || rpl != cpl) { protect_exception(EXC_GP, sel & 0xFFFC); return false; }
	} else {
		if (code && conform) { if (dpl > cpl) { protect_exception(EXC_GP, sel & 0xFFFC); return false; } } else                 { if (dpl < cpl || dpl < rpl) { protect_exception(EXC_GP, sel & 0xFFFC); return false; } }
	}
	PMLOG("[PM] seg%d=%04X base=%06X lim=%04X acc=%02X @%04X:%04X\n",
	      idx, sel, *base, *limit, *access, cpu.cs, cpu.ip);
	return true;
}

// ============================================================
// 0F 00 组辅助：LLDT / LTR / VERR / VERW
// ============================================================
// 安静地读选择子对应的描述符（不产生异常，失败只返回 false）
static bool quiet_desc(uint16_t sel, SegmentDescriptor* d, uint32_t* base, uint32_t* limit) {
	if ((sel & 0xFFF8u) == 0) return false;
	uint32_t tbl_base; uint16_t tbl_limit;
	if (sel & 0x0004) { tbl_base = protect.ldtr_base; tbl_limit = protect.ldtr_limit; }
	else              { tbl_base = protect.gdt_base;  tbl_limit = protect.gdt_limit;  }
	uint32_t index = (sel & 0xFFF8u) >> 3;
	if (index * 8 + 7 > (uint32_t)tbl_limit) return false;
	protect_read_desc(tbl_base + index * 8, d);
	if (!(d->access & SEG_ACCESS_PRESENT)) return false;
	*base = desc_base(d);
	*limit = desc_limit(d);
	return true;
}

bool protect_load_ldt(uint16_t sel) {
	if ((sel & 0xFFF8u) == 0) {
		protect.ldtr = 0; protect.ldtr_base = 0; protect.ldtr_limit = 0;
		return true;
	}
	SegmentDescriptor d; uint32_t b, l;
	if (!quiet_desc(sel, &d, &b, &l)) { protect_exception(EXC_GP, sel & 0xFFFC); return false; }
	if ((d.access & SEG_ACCESS_S) || (d.access & 0x1F) != SYS_TYPE_LDT) {
		protect_exception(EXC_GP, sel & 0xFFFC); return false;
	}
	protect.ldtr = sel & 0xFFFC;
	protect.ldtr_base = b;
	protect.ldtr_limit = (uint16_t)l;
	return true;
}

bool protect_load_tr(uint16_t sel) {
	if ((sel & 0xFFF8u) == 0) { protect_exception(EXC_GP, 0); return false; }
	SegmentDescriptor d; uint32_t b, l;
	if (!quiet_desc(sel, &d, &b, &l)) { protect_exception(EXC_GP, sel & 0xFFFC); return false; }
	if (d.access & SEG_ACCESS_S) { protect_exception(EXC_GP, sel & 0xFFFC); return false; }
	if ((d.access & 0x1F) != SYS_TYPE_TSS16) { protect_exception(EXC_GP, sel & 0xFFFC); return false; }
	// 置 busy，写回描述符
	uint32_t tbl = (sel & 4) ? protect.ldtr_base : protect.gdt_base;
	uint32_t daddr = tbl + (((sel & 0xFFF8u) >> 3) * 8) + 5;
	cpu_mem_write(daddr, (uint8_t)((d.access & 0xF8) | SYS_TYPE_TSS16_BUSY));
	protect.tr = sel & 0xFFFC;
	protect.tr_base = b;
	protect.tr_limit = (uint16_t)l;
	return true;
}

bool protect_verify_sel(uint16_t sel, bool for_write) {
	if (!protect.pe) return false;
	SegmentDescriptor d; uint32_t b, l;
	if (!quiet_desc(sel, &d, &b, &l)) return false;
	(void)b; (void)l;
	if (!(d.access & SEG_ACCESS_S)) return false;      // 系统段不可验证
	bool code = (d.access & SEG_ACCESS_CODE) != 0;
	bool conform = (d.access & SEG_ACCESS_CONFORM) != 0;
	uint8_t dpl = (d.access >> 5) & 3;
	uint8_t cpl = (uint8_t)protect_cpl();
	uint8_t rpl = sel & 3;
	if (code) {
		if (!conform) { if (dpl < cpl || dpl < rpl) return false; }
		else          { if (dpl > cpl) return false; }
		if (for_write) return false;                   // 代码段不可写
		return (d.access & SEG_ACCESS_RW) != 0;        // 代码段"可写"位其实是可读
	}
	if (for_write) return (d.access & SEG_ACCESS_RW) != 0;
	return true;
}

// ============================================================
// 0F 02/03 组辅助：LAR / LSL
// ============================================================
static bool lar_lsl_ok(uint16_t sel, SegmentDescriptor* d) {
	if (!protect.pe) return false;
	uint32_t b, l;
	if (!quiet_desc(sel, d, &b, &l)) return false;
	if (!(d->access & SEG_ACCESS_S)) return false;     // 只对代码/数据段有效
	bool code = (d->access & SEG_ACCESS_CODE) != 0;
	bool conform = (d->access & SEG_ACCESS_CONFORM) != 0;
	uint8_t dpl = (d->access >> 5) & 3;
	uint8_t cpl = (uint8_t)protect_cpl();
	uint8_t rpl = sel & 3;
	if (code && conform) { if (dpl > cpl) return false; }
	else                 { if (dpl < cpl || dpl < rpl) return false; }
	return true;
}

bool protect_sel_access(uint16_t sel, uint8_t* access) {
	SegmentDescriptor d;
	if (!lar_lsl_ok(sel, &d)) return false;
	*access = d.access;
	return true;
}

bool protect_sel_limit(uint16_t sel, uint32_t* limit) {
	SegmentDescriptor d;
	if (!lar_lsl_ok(sel, &d)) return false;
	*limit = desc_limit(&d);
	return true;
}

// ============================================================
// TSS（16 位，44 字节）
// ============================================================
enum {
	TSS_LINK = 0, TSS_SP0 = 2, TSS_SS0 = 4, TSS_SP1 = 6, TSS_SS1 = 8,
	TSS_SP2 = 10, TSS_SS2 = 12, TSS_IP = 14, TSS_FLAGS = 16,
	TSS_AX = 18, TSS_CX = 20, TSS_DX = 22, TSS_BX = 24, TSS_SP = 26,
	TSS_BP = 28, TSS_SI = 30, TSS_DI = 32, TSS_ES = 34, TSS_CS = 36,
	TSS_SS = 38, TSS_DS = 40, TSS_LDT = 42, TSS_SIZE = 44
};

static uint16_t tss_rw(uint32_t off) { return (uint16_t)(cpu_mem_read(protect.tr_base + off) |
                                     (cpu_mem_read(protect.tr_base + off + 1) << 8)); }
static void tss_ww(uint32_t off, uint16_t v) {
	cpu_mem_write(protect.tr_base + off, v & 0xFF);
	cpu_mem_write(protect.tr_base + off + 1, v >> 8);
}

// 从 TSS 取 ring 的栈（SS:SP）。ring 0/1/2 分别对应 TSS 的前三个字段对。
static bool tss_ring_stack(int ring, uint16_t* ss, uint16_t* sp) {
	if (ring < 0 || ring > 2) return false;
	static const uint32_t sp_off[3] = { TSS_SP0, TSS_SP1, TSS_SP2 };
	static const uint32_t ss_off[3] = { TSS_SS0, TSS_SS1, TSS_SS2 };
	*sp = tss_rw(sp_off[ring]);
	*ss = tss_rw(ss_off[ring]);
	return (*ss & 0xFFF8u) != 0;
}

// ============================================================
// 栈切换：把当前 SS:SP 从 TSS 换成目标环的栈
// ============================================================
static bool switch_stack(int new_cpl, uint16_t* old_ss, uint16_t* old_sp) {
	*old_ss = cpu.ss;
	*old_sp = cpu.sp;
	uint16_t nss, nsp;
	if (!tss_ring_stack(new_cpl, &nss, &nsp)) {
		
		protect_exception(EXC_TS, protect.tr & 0xFFFC);
		return false;
	}
	// 新栈必须是可写数据段，DPL = new_cpl（规范要求），且已存在
	SegmentDescriptor d; uint32_t b, l;
	if ((nss & 0xFFF8u) == 0 || !read_desc_sel(nss, &d, &b, &l, NULL, true)) return false;
	if ((d.access & SEG_ACCESS_S) == 0 || (d.access & SEG_ACCESS_CODE) ||
	    !(d.access & SEG_ACCESS_RW) || ((d.access >> 5) & 3) != new_cpl) {
		protect_exception(EXC_TS, nss & 0xFFFC);
		return false;
	}
	cpu.ss = nss & 0xFFFC;
	cpu.sp = nsp;
	cpu.seg_base[SEG_SS] = b;
	cpu.seg_limit[SEG_SS] = l;
	return true;
}

// ============================================================
// 远转移
// ============================================================
static uint32_t gate_offset(const GateDescriptor* g, uint8_t type) {
	if (type == SYS_TYPE_INTGATE32 || type == SYS_TYPE_TRAPGATE32)
		return (uint32_t)g->offset_low | ((uint32_t)g->offset_high << 16);
	return g->offset_low;   // 286 的门：16 位偏移
}

// 装入目标代码段（new_cpl 为期望的新特权级）
static bool load_cs_target(uint16_t sel, uint8_t new_cpl, uint32_t* base, uint32_t* limit) {
	SegmentDescriptor d;
	if (!read_desc_sel(sel, &d, base, limit, NULL, true)) return false;
	if (!(d.access & SEG_ACCESS_S) || !(d.access & SEG_ACCESS_CODE)) {
		protect_exception(EXC_GP, sel & 0xFFFC); return false;
	}
	uint8_t dpl = (d.access >> 5) & 3;
	bool conform = (d.access & SEG_ACCESS_CONFORM) != 0;
	if (conform) { if (dpl > protect_cpl()) { protect_exception(EXC_GP, sel & 0xFFFC); return false; } }
	else         { if (dpl != new_cpl) { protect_exception(EXC_GP, sel & 0xFFFC); return false; } }
	return true;
}

void protect_far_jmp(uint16_t sel, uint16_t off) {
	if ((sel & 0xFFF8u) == 0) { protect_exception(EXC_GP, 0); return; }

	SegmentDescriptor d;
	uint32_t b, l;
	if (!read_desc_sel(sel, &d, &b, &l, NULL, false)) return;

	// 调用门（不切换任务）：JMP 只能同特权级
	if (!(d.access & SEG_ACCESS_S) && (d.access & 0x1F) == SYS_TYPE_CALLGATE) {
		uint32_t tbl = (sel & 4) ? protect.ldtr_base : protect.gdt_base;
		uint32_t addr = tbl + (((sel & 0xFFF8u) >> 3) * 8);
		GateDescriptor g;
		protect_read_gate(addr, &g);
		uint8_t type = g.type_attr & 0x1F;
		uint8_t dpl = (g.type_attr >> 5) & 3;
		if (protect_cpl() > dpl) { protect_exception(EXC_GP, sel & 0xFFFC); return; }
		uint16_t tsel = g.selector;
		uint8_t tcpl = tsel & 3;
		if (tcpl != protect_cpl()) { protect_exception(EXC_GP, tsel & 0xFFFC); return; }
		uint32_t tb, tl;
		if (!load_cs_target(tsel, tcpl, &tb, &tl)) return;
		uint32_t toff = gate_offset(&g, type);
		if (toff > tl) { protect_exception(EXC_GP, 0); return; }
		cpu.cs = tsel & 0xFFFC;
		cpu.seg_base[SEG_CS] = tb;
		cpu.seg_limit[SEG_CS] = tl;
		cpu.ip = (uint16_t)toff;
		return;
	}

	uint8_t rpl = sel & 3;
	uint32_t tb, tl;
	if (!load_cs_target(sel, rpl, &tb, &tl)) return;
	if (off > tl) { protect_exception(EXC_GP, 0); return; }
	cpu.cs = sel & 0xFFFC;
	cpu.seg_base[SEG_CS] = tb;
	cpu.seg_limit[SEG_CS] = tl;
	cpu.ip = off;
}

void protect_far_call(uint16_t sel, uint16_t off) {
	if ((sel & 0xFFF8u) == 0) { protect_exception(EXC_GP, 0); return; }

	SegmentDescriptor d;
	uint32_t b, l;
	if (!read_desc_sel(sel, &d, &b, &l, NULL, false)) return;

	if (!(d.access & SEG_ACCESS_S) && (d.access & 0x1F) == SYS_TYPE_CALLGATE) {
		uint32_t tbl = (sel & 4) ? protect.ldtr_base : protect.gdt_base;
		uint32_t addr = tbl + (((sel & 0xFFF8u) >> 3) * 8);
		GateDescriptor g;
		protect_read_gate(addr, &g);
		uint8_t type = g.type_attr & 0x1F;
		uint8_t gdpl = (g.type_attr >> 5) & 3;
		if (protect_cpl() > gdpl) { protect_exception(EXC_GP, sel & 0xFFFC); return; }
		uint16_t tsel = g.selector;
		uint8_t tcpl = tsel & 3;
		uint32_t tb, tl;
		if (!load_cs_target(tsel, tcpl, &tb, &tl)) return;
		uint32_t toff = gate_offset(&g, type);
		if (toff > tl) { protect_exception(EXC_GP, 0); return; }

		uint16_t old_ss, old_sp;
		if (tcpl < protect_cpl()) {
			if (!switch_stack(tcpl, &old_ss, &old_sp)) return;
			push(old_ss);
			push(old_sp);
		}
		uint8_t nparams = g.count;
		(void)nparams;
		push(cpu.cs);
		push(cpu.ip);
		cpu.cs = tsel & 0xFFFC;
		cpu.seg_base[SEG_CS] = tb;
		cpu.seg_limit[SEG_CS] = tl;
		cpu.ip = (uint16_t)toff;
		return;
	}

	uint8_t rpl = sel & 3;
	uint32_t tb, tl;
	if (!load_cs_target(sel, rpl, &tb, &tl)) return;
	if (off > tl) { protect_exception(EXC_GP, 0); return; }
	push(cpu.cs);
	push(cpu.ip);
	cpu.cs = sel & 0xFFFC;
	cpu.seg_base[SEG_CS] = tb;
	cpu.seg_limit[SEG_CS] = tl;
	cpu.ip = off;
}

void protect_far_ret(bool is_iret, uint16_t imm) {
	uint16_t ret_ip = pop();
	uint16_t ret_cs = pop();
	uint16_t new_flags = cpu.flags;
	if (is_iret) new_flags = pop();

	SegmentDescriptor d;
	uint32_t tb, tl;
	if ((ret_cs & 0xFFF8u) != 0) {
		if (!read_desc_sel(ret_cs, &d, &tb, &tl, NULL, true)) return;
		if (!(d.access & SEG_ACCESS_S) || !(d.access & SEG_ACCESS_CODE)) {
			protect_exception(EXC_GP, ret_cs & 0xFFFC); return;
		}
		uint8_t rpl = ret_cs & 3;
		uint8_t dpl = (d.access >> 5) & 3;
		bool conform = (d.access & SEG_ACCESS_CONFORM) != 0;
		if (!conform && dpl != rpl) { protect_exception(EXC_GP, ret_cs & 0xFFFC); return; }
		if (conform && dpl > rpl) { protect_exception(EXC_GP, ret_cs & 0xFFFC); return; }
	} else { tb = 0; tl = 0; }

	cpu.cs = ret_cs & 0xFFFC;
	cpu.seg_base[SEG_CS] = tb;
	cpu.seg_limit[SEG_CS] = tl;
	cpu.ip = ret_ip;
	if (is_iret) cpu.flags = (uint16_t)(new_flags | 0x0002) & 0x7FFF;

	uint8_t rcpl = ret_cs & 3;
	if (rcpl > protect_cpl()) {
		// 返回外层：弹出 SS:SP
		uint16_t nsp = pop();
		uint16_t nss = pop();
		SegmentDescriptor sd;
		uint32_t sb, sl;
		if ((nss & 0xFFF8u) == 0 || !read_desc_sel(nss, &sd, &sb, &sl, NULL, true)) return;
		if ((sd.access & SEG_ACCESS_S) == 0 || (sd.access & SEG_ACCESS_CODE) ||
		    !(sd.access & SEG_ACCESS_RW) || ((sd.access >> 5) & 3) != rcpl) {
			protect_exception(EXC_GP, nss & 0xFFFC); return;
		}
		cpu.ss = nss & 0xFFFC;
		cpu.seg_base[SEG_SS] = sb;
		cpu.seg_limit[SEG_SS] = sl;
		cpu.sp = (uint16_t)(nsp + imm);
	} else {
		cpu.sp = (uint16_t)(cpu.sp + imm);
	}
}

// ============================================================
// 门级中断 / 异常投递
// ============================================================
// 已投递的异常：把错误码推入后再进门
static int in_pfault = 0;
static const bool vec_has_error[32] = {
	false,false,false,false,false,false,false,false,   // 0-7
	true, false,true, true, true, true, true, false,   // 8-15
	false,true, false,false,false,false,false,false,   // 16-23
	false,false,false,false,false,false,false,false
};

void protect_interrupt(uint8_t vector, bool has_error, uint16_t error_code, bool is_soft) {
	(void)has_error;
	PMLOG("[PM] int %02X (err=%04X) @%04X:%04X\n", vector, error_code, cpu.cs, cpu.ip);
	if ((uint32_t)vector * 8 + 7 > (uint32_t)protect.idt_limit) {
		protect_fault_shutdown("IDT 越界，无法投递");
		return;
	}
	GateDescriptor g;
	protect_read_gate(protect.idt_base + (uint32_t)vector * 8, &g);
	uint8_t type = g.type_attr & 0x1F;
	uint8_t dpl = (g.type_attr >> 5) & 3;

	if (is_soft && (type == SYS_TYPE_INTGATE || type == SYS_TYPE_TRAPGATE ||
	                type == SYS_TYPE_INTGATE32 || type == SYS_TYPE_TRAPGATE32)) {
		if (dpl < protect_cpl()) { protect_exception(EXC_GP, (uint16_t)(vector * 8 + 2)); return; }
	}

	uint16_t old_cs = cpu.cs, old_ip = cpu.ip, old_ss = cpu.ss, old_sp = cpu.sp;
	bool err = (vector < 32) && vec_has_error[vector];

	if ((g.type_attr & SEG_ACCESS_PRESENT) == 0) {
		fprintf(stderr, "[NP] IDT gate not present vector=%u cs:ip=%04X:%04X\n",
		        vector, cpu.cs, cpu.ip);
		protect_exception(EXC_NP, (uint16_t)(vector * 8 + 2)); return;
	}

	if (type == SYS_TYPE_TASKGATE) { protect_task_switch(g.selector); return; }
	if (type != SYS_TYPE_INTGATE && type != SYS_TYPE_TRAPGATE &&
	    type != SYS_TYPE_INTGATE32 && type != SYS_TYPE_TRAPGATE32) {
		if (in_pfault) { protect_fault_shutdown("IDT 项不是合法门 → 双重故障"); return; }
		in_pfault = 1;
		protect_exception(EXC_GP, (uint16_t)(vector * 8 + 2));
		in_pfault = 0;
		return;
	}

	uint16_t tsel = g.selector;
	uint8_t new_cpl = dpl;
	uint32_t tb, tl;
	if (!load_cs_target(tsel, new_cpl, &tb, &tl)) return;
	uint32_t toff = gate_offset(&g, type);
	if (toff > tl) { protect_exception(EXC_GP, 0); return; }

	if (new_cpl < protect_cpl()) {
		if (!switch_stack(new_cpl, &old_ss, &old_sp)) return;
		push(old_ss);
		push(old_sp);
	}
	push(cpu.flags);
	push(old_cs);
	push(old_ip);
	if (err) push(error_code);

	cpu.cs = tsel & 0xFFFC;
	cpu.seg_base[SEG_CS] = tb;
	cpu.seg_limit[SEG_CS] = tl;
	cpu.ip = (uint16_t)toff;
	if (type == SYS_TYPE_INTGATE || type == SYS_TYPE_INTGATE32) clear_flag(FLAG_IF);
	clear_flag(FLAG_TF);
}

void protect_exception(int vector, uint16_t error_code) {
	if (in_pfault) {
		in_pfault = 0;
		
		protect_interrupt(EXC_DF, true, 0, false);
		return;
	}
	in_pfault = 1;
	protect_interrupt((uint8_t)vector, true, error_code, false);
	in_pfault = 0;
}

void protect_fault_shutdown(const char* why) {
	(void)why;
	cpu_running = false;
}

// ============================================================
// 任务切换（286 16 位 TSS）
// ============================================================
void protect_task_switch(uint16_t tss_sel) {
	SegmentDescriptor d;
	uint32_t b, l;
	if ((tss_sel & 0xFFF8u) == 0 || !read_desc_sel(tss_sel, &d, &b, &l, NULL, true)) {
		protect_exception(EXC_TS, tss_sel & 0xFFFC);
		return;
	}
	uint8_t type = d.access & 0x1F;
	if ((d.access & SEG_ACCESS_S) || (type != SYS_TYPE_TSS16 && type != SYS_TYPE_TSS16_BUSY)) {
		protect_exception(EXC_TS, tss_sel & 0xFFFC);
		return;
	}
	if (l < TSS_SIZE - 1) { protect_exception(EXC_TS, tss_sel & 0xFFFC); return; }

	uint32_t new_tr = b;

	// 保存当前任务现场（若有 TR）
	if ((protect.tr & 0xFFF8u) != 0) {
		tss_ww(TSS_IP, cpu.ip);
		tss_ww(TSS_FLAGS, cpu.flags);
		tss_ww(TSS_AX, cpu.ax); tss_ww(TSS_CX, cpu.cx);
		tss_ww(TSS_DX, cpu.dx); tss_ww(TSS_BX, cpu.bx);
		tss_ww(TSS_SP, cpu.sp); tss_ww(TSS_BP, cpu.bp);
		tss_ww(TSS_SI, cpu.si); tss_ww(TSS_DI, cpu.di);
		tss_ww(TSS_ES, cpu.es); tss_ww(TSS_CS, cpu.cs);
		tss_ww(TSS_SS, cpu.ss); tss_ww(TSS_DS, cpu.ds);
		tss_ww(TSS_LDT, protect.ldtr);
	}

	// 新任务描述符置 busy
	uint32_t tbl = (tss_sel & 4) ? protect.ldtr_base : protect.gdt_base;
	uint32_t daddr = tbl + (((tss_sel & 0xFFF8u) >> 3) * 8);
	cpu_mem_write(daddr + 5, (uint8_t)((d.access & 0xF8) | SYS_TYPE_TSS16_BUSY));

	protect.tr = tss_sel & 0xFFFC;
	protect.tr_base = new_tr;
	protect.tr_limit = (uint16_t)l;

	// 恢复新任务现场
	cpu.ip    = tss_rw(TSS_IP);
	cpu.flags = tss_rw(TSS_FLAGS) & 0x7FFF;
	cpu.ax = tss_rw(TSS_AX); cpu.cx = tss_rw(TSS_CX);
	cpu.dx = tss_rw(TSS_DX); cpu.bx = tss_rw(TSS_BX);
	cpu.sp = tss_rw(TSS_SP); cpu.bp = tss_rw(TSS_BP);
	cpu.si = tss_rw(TSS_SI); cpu.di = tss_rw(TSS_DI);

	uint16_t nes = tss_rw(TSS_ES), ncs = tss_rw(TSS_CS);
	uint16_t nss = tss_rw(TSS_SS), nds = tss_rw(TSS_DS);
	SegmentDescriptor sd;
	if (!read_desc_sel(nes, &sd, &cpu.seg_base[SEG_ES], &cpu.seg_limit[SEG_ES], NULL, false)) return;
	cpu.es = nes & 0xFFFC;
	if (!read_desc_sel(ncs, &sd, &cpu.seg_base[SEG_CS], &cpu.seg_limit[SEG_CS], NULL, false)) return;
	cpu.cs = ncs & 0xFFFC;
	if (!read_desc_sel(nss, &sd, &cpu.seg_base[SEG_SS], &cpu.seg_limit[SEG_SS], NULL, false)) return;
	cpu.ss = nss & 0xFFFC;
	if (!read_desc_sel(nds, &sd, &cpu.seg_base[SEG_DS], &cpu.seg_limit[SEG_DS], NULL, false)) return;
	cpu.ds = nds & 0xFFFC;

	uint16_t nldt = tss_rw(TSS_LDT);
	protect.ldtr = nldt;
	protect.ldtr_base = 0; protect.ldtr_limit = 0;
	if ((nldt & 0xFFF8u) != 0) {
		SegmentDescriptor ld;
		uint32_t lb, ll;
		if (read_desc_sel(nldt, &ld, &lb, &ll, NULL, false) &&
		    !(ld.access & SEG_ACCESS_S) && (ld.access & 0x1F) == SYS_TYPE_LDT) {
			protect.ldtr_base = lb;
			protect.ldtr_limit = (uint16_t)ll;
		}
	}
	// 新任务没有 CR3/PG（286 无分页）
}