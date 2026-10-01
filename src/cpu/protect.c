// protect.c - 80286 保护模式实现（重写版）
#include "protect.h"
#include "cpu.h"
#include <stdio.h>
#include <string.h>

ProtectState286 protect;

int protect_cpl(void) { return cpu.cs & 0x0003; }

void protect_init(void) {
    memset(&protect, 0, sizeof(protect));
    protect.msw = 0xFFF0;
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
    if (d->limit_high_flags & 0x80) lim = (lim << 12) | 0xFFF;
    return lim;
}

// 投递异常的统一入口（防递归）
static void deliver_exception(int vector, uint16_t error_code);

// ============================================================
// 按选择子读描述符。失败时通过 deliver_exception 投递 #GP/#NP
// ============================================================
static bool read_desc_sel(uint16_t sel, SegmentDescriptor* d, uint32_t* base,
                          uint32_t* limit, uint32_t* daddr, bool touch) {
    uint32_t tbl_base;
    uint16_t tbl_limit;
    if (sel & 0x0004) { tbl_base = protect.ldtr_base; tbl_limit = protect.ldtr_limit; }
    else              { tbl_base = protect.gdt_base;  tbl_limit = protect.gdt_limit;  }

    uint32_t index = (sel & 0xFFF8u) >> 3;
    uint32_t addr = tbl_base + index * 8;

    // 边界检查
    if (index * 8 + 7 > (uint32_t)tbl_limit) {
        deliver_exception(EXC_GP, sel & 0xFFFC);
        return false;
    }

    protect_read_desc(addr, d);

    // P 位检查
    if (!(d->access & SEG_ACCESS_PRESENT)) {
        deliver_exception(EXC_NP, sel & 0xFFFC);
        return false;
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
    if ((sel & 0xFFF8u) == 0) {
        if (idx == SEG_CS || idx == SEG_SS) { deliver_exception(EXC_GP, 0); return false; }
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

    if (sys) { deliver_exception(EXC_GP, sel & 0xFFFC); return false; }

    if (idx == SEG_CS) {
        if (!code) { deliver_exception(EXC_GP, sel & 0xFFFC); return false; }
        if (conform) { if (dpl > cpl) { deliver_exception(EXC_GP, sel & 0xFFFC); return false; } }
        else         { if (dpl != cpl || rpl != cpl) { deliver_exception(EXC_GP, sel & 0xFFFC); return false; } }
    } else if (idx == SEG_SS) {
        if (code || !(d.access & SEG_ACCESS_RW)) { deliver_exception(EXC_GP, sel & 0xFFFC); return false; }
        if (dpl != cpl || rpl != cpl) { deliver_exception(EXC_GP, sel & 0xFFFC); return false; }
    } else {
        if (code && conform) { if (dpl > cpl) { deliver_exception(EXC_GP, sel & 0xFFFC); return false; } }
        else                 { if (dpl < cpl || dpl < rpl) { deliver_exception(EXC_GP, sel & 0xFFFC); return false; } }
    }
    return true;
}

// ============================================================
// 0F 00 组辅助
// ============================================================
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
    if (!quiet_desc(sel, &d, &b, &l)) { deliver_exception(EXC_GP, sel & 0xFFFC); return false; }
    if ((d.access & SEG_ACCESS_S) || (d.access & 0x1F) != SYS_TYPE_LDT) {
        deliver_exception(EXC_GP, sel & 0xFFFC); return false;
    }
    protect.ldtr = sel & 0xFFFC;
    protect.ldtr_base = b;
    protect.ldtr_limit = (uint16_t)l;
    return true;
}

bool protect_load_tr(uint16_t sel) {
    if ((sel & 0xFFF8u) == 0) { deliver_exception(EXC_GP, 0); return false; }
    SegmentDescriptor d; uint32_t b, l;
    if (!quiet_desc(sel, &d, &b, &l)) { deliver_exception(EXC_GP, sel & 0xFFFC); return false; }
    if (d.access & SEG_ACCESS_S) { deliver_exception(EXC_GP, sel & 0xFFFC); return false; }
    if ((d.access & 0x1F) != SYS_TYPE_TSS16) { deliver_exception(EXC_GP, sel & 0xFFFC); return false; }
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
    if (!(d.access & SEG_ACCESS_S)) return false;
    bool code = (d.access & SEG_ACCESS_CODE) != 0;
    bool conform = (d.access & SEG_ACCESS_CONFORM) != 0;
    uint8_t dpl = (d.access >> 5) & 3;
    uint8_t cpl = (uint8_t)protect_cpl();
    uint8_t rpl = sel & 3;
    if (code) {
        if (!conform) { if (dpl < cpl || dpl < rpl) return false; }
        else          { if (dpl > cpl) return false; }
        if (for_write) return false;
        return (d.access & SEG_ACCESS_RW) != 0;
    }
    if (for_write) return (d.access & SEG_ACCESS_RW) != 0;
    return true;
}

// ============================================================
// LAR / LSL
// ============================================================
static bool lar_lsl_ok(uint16_t sel, SegmentDescriptor* d) {
    if (!protect.pe) return false;
    uint32_t b, l;
    if (!quiet_desc(sel, d, &b, &l)) return false;
    if (!(d->access & SEG_ACCESS_S)) return false;
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
// TSS
// ============================================================
enum {
    TSS_LINK = 0, TSS_SP0 = 2, TSS_SS0 = 4, TSS_SP1 = 6, TSS_SS1 = 8,
    TSS_SP2 = 10, TSS_SS2 = 12, TSS_IP = 14, TSS_FLAGS = 16,
    TSS_AX = 18, TSS_CX = 20, TSS_DX = 22, TSS_BX = 24, TSS_SP = 26,
    TSS_BP = 28, TSS_SI = 30, TSS_DI = 32, TSS_ES = 34, TSS_CS = 36,
    TSS_SS = 38, TSS_DS = 40, TSS_LDT = 42, TSS_SIZE = 44
};

static uint16_t tss_rw(uint32_t off) {
    return (uint16_t)(cpu_mem_read(protect.tr_base + off) |
                     (cpu_mem_read(protect.tr_base + off + 1) << 8));
}
static void tss_ww(uint32_t off, uint16_t v) {
    cpu_mem_write(protect.tr_base + off, v & 0xFF);
    cpu_mem_write(protect.tr_base + off + 1, v >> 8);
}

static bool tss_ring_stack(int ring, uint16_t* ss, uint16_t* sp) {
    if (ring < 0 || ring > 2) return false;
    static const uint32_t sp_off[3] = { TSS_SP0, TSS_SP1, TSS_SP2 };
    static const uint32_t ss_off[3] = { TSS_SS0, TSS_SS1, TSS_SS2 };
    *sp = tss_rw(sp_off[ring]);
    *ss = tss_rw(ss_off[ring]);
    return (*ss & 0xFFF8u) != 0;
}

// ============================================================
// 栈切换
// ============================================================
static bool switch_stack(int new_cpl, uint16_t* old_ss, uint16_t* old_sp) {
    *old_ss = cpu.ss;
    *old_sp = cpu.sp;
    uint16_t nss, nsp;
    if (!tss_ring_stack(new_cpl, &nss, &nsp)) {
        deliver_exception(EXC_TS, protect.tr & 0xFFFC);
        return false;
    }
    SegmentDescriptor d; uint32_t b, l;
    if ((nss & 0xFFF8u) == 0 || !read_desc_sel(nss, &d, &b, &l, NULL, true)) return false;
    if ((d.access & SEG_ACCESS_S) == 0 || (d.access & SEG_ACCESS_CODE) ||
        !(d.access & SEG_ACCESS_RW) || ((d.access >> 5) & 3) != new_cpl) {
        deliver_exception(EXC_TS, nss & 0xFFFC);
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
    return g->offset_low;
}

static bool load_cs_target(uint16_t sel, uint8_t new_cpl, uint32_t* base, uint32_t* limit) {
    SegmentDescriptor d;
    if (!read_desc_sel(sel, &d, base, limit, NULL, true)) return false;
    if (!(d.access & SEG_ACCESS_S) || !(d.access & SEG_ACCESS_CODE)) {
        deliver_exception(EXC_GP, sel & 0xFFFC); return false;
    }
    uint8_t dpl = (d.access >> 5) & 3;
    bool conform = (d.access & SEG_ACCESS_CONFORM) != 0;
    if (conform) { if (dpl > protect_cpl()) { deliver_exception(EXC_GP, sel & 0xFFFC); return false; } }
    else         { if (dpl != new_cpl) { deliver_exception(EXC_GP, sel & 0xFFFC); return false; } }
    return true;
}

void protect_far_jmp(uint16_t sel, uint16_t off) {
    if ((sel & 0xFFF8u) == 0) { deliver_exception(EXC_GP, 0); return; }

    SegmentDescriptor d;
    uint32_t b, l;
    if (!read_desc_sel(sel, &d, &b, &l, NULL, false)) return;

    if (!(d.access & SEG_ACCESS_S) && (d.access & 0x1F) == SYS_TYPE_CALLGATE) {
        uint32_t tbl = (sel & 4) ? protect.ldtr_base : protect.gdt_base;
        uint32_t addr = tbl + (((sel & 0xFFF8u) >> 3) * 8);
        GateDescriptor g;
        protect_read_gate(addr, &g);
        uint8_t type = g.type_attr & 0x1F;
        uint8_t dpl = (g.type_attr >> 5) & 3;
        if (protect_cpl() > dpl) { deliver_exception(EXC_GP, sel & 0xFFFC); return; }
        uint16_t tsel = g.selector;
        uint8_t tcpl = tsel & 3;
        if (tcpl != protect_cpl()) { deliver_exception(EXC_GP, tsel & 0xFFFC); return; }
        uint32_t tb, tl;
        if (!load_cs_target(tsel, tcpl, &tb, &tl)) return;
        uint32_t toff = gate_offset(&g, type);
        if (toff > tl) { deliver_exception(EXC_GP, 0); return; }
        cpu.cs = tsel & 0xFFFC;
        cpu.seg_base[SEG_CS] = tb;
        cpu.seg_limit[SEG_CS] = tl;
        cpu.ip = (uint16_t)toff;
        return;
    }

    uint8_t rpl = sel & 3;
    uint32_t tb, tl;
    if (!load_cs_target(sel, rpl, &tb, &tl)) return;
    if (off > tl) { deliver_exception(EXC_GP, 0); return; }
    cpu.cs = sel & 0xFFFC;
    cpu.seg_base[SEG_CS] = tb;
    cpu.seg_limit[SEG_CS] = tl;
    cpu.ip = off;
}

void protect_far_call(uint16_t sel, uint16_t off) {
    if ((sel & 0xFFF8u) == 0) { deliver_exception(EXC_GP, 0); return; }

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
        if (protect_cpl() > gdpl) { deliver_exception(EXC_GP, sel & 0xFFFC); return; }
        uint16_t tsel = g.selector;
        uint8_t tcpl = tsel & 3;
        uint32_t tb, tl;
        if (!load_cs_target(tsel, tcpl, &tb, &tl)) return;
        uint32_t toff = gate_offset(&g, type);
        if (toff > tl) { deliver_exception(EXC_GP, 0); return; }

        uint16_t old_ss, old_sp;
        if (tcpl < protect_cpl()) {
            if (!switch_stack(tcpl, &old_ss, &old_sp)) return;
            push(old_ss);
            push(old_sp);
        }
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
    if (off > tl) { deliver_exception(EXC_GP, 0); return; }
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
            deliver_exception(EXC_GP, ret_cs & 0xFFFC); return;
        }
        uint8_t rpl = ret_cs & 3;
        uint8_t dpl = (d.access >> 5) & 3;
        bool conform = (d.access & SEG_ACCESS_CONFORM) != 0;
        if (!conform && dpl != rpl) { deliver_exception(EXC_GP, ret_cs & 0xFFFC); return; }
        if (conform && dpl > rpl) { deliver_exception(EXC_GP, ret_cs & 0xFFFC); return; }
    } else { tb = 0; tl = 0; }

    cpu.cs = ret_cs & 0xFFFC;
    cpu.seg_base[SEG_CS] = tb;
    cpu.seg_limit[SEG_CS] = tl;
    cpu.ip = ret_ip;
    if (is_iret) cpu.flags = (uint16_t)(new_flags | 0x0002) & 0x7FFF;

    uint8_t rcpl = ret_cs & 3;
    if (rcpl > protect_cpl()) {
        uint16_t nsp = pop();
        uint16_t nss = pop();
        SegmentDescriptor sd;
        uint32_t sb, sl;
        if ((nss & 0xFFF8u) == 0 || !read_desc_sel(nss, &sd, &sb, &sl, NULL, true)) return;
        if ((sd.access & SEG_ACCESS_S) == 0 || (sd.access & SEG_ACCESS_CODE) ||
            !(sd.access & SEG_ACCESS_RW) || ((sd.access >> 5) & 3) != rcpl) {
            deliver_exception(EXC_GP, nss & 0xFFFC); return;
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
static const bool vec_has_error[32] = {
    false,false,false,false,false,false,false,false,
    true, false,true, true, true, true, true, false,
    false,true, false,false,false,false,false,false,
    false,false,false,false,false,false,false,false
};

// 投递中标志：防止异常递归
static int in_delivery = 0;

// 投递异常的统一入口
static void deliver_exception(int vector, uint16_t error_code) {
    // 已经在投递中 → 升级为 #DF
    if (in_delivery) {
        if (vector == EXC_DF) {
            // #DF 也投不出去 → 三重故障
            fprintf(stderr, "[FAULT] triple fault (in_delivery @ vec %d)\n", vector);
            cpu_running = false;
            return;
        }
        // 升级为 #DF（不递增 in_delivery）
        protect_interrupt(EXC_DF, true, 0, false);
        return;
    }
    in_delivery = 1;
    protect_interrupt((uint8_t)vector, true, error_code, false);
    in_delivery = 0;
}

void protect_interrupt(uint8_t vector, bool has_error, uint16_t error_code, bool is_soft) {
    (void)has_error;
    PMLOG("[PM] int %02X (err=%04X) @%04X:%04X\n", vector, error_code, cpu.cs, cpu.ip);

    // IDT 越界 → 停机
    if ((uint32_t)vector * 8 + 7 > (uint32_t)protect.idt_limit) {
        fprintf(stderr, "[FAULT] IDT 越界 (vec %d) -> shutdown\n", vector);
        cpu_running = false;
        return;
    }

    GateDescriptor g;
    protect_read_gate(protect.idt_base + (uint32_t)vector * 8, &g);
    uint8_t type = g.type_attr & 0x1F;
    uint8_t dpl = (g.type_attr >> 5) & 3;

    // 软中断 DPL 检查
    if (is_soft && (type == SYS_TYPE_INTGATE || type == SYS_TYPE_TRAPGATE ||
                    type == SYS_TYPE_INTGATE32 || type == SYS_TYPE_TRAPGATE32)) {
        if (dpl < protect_cpl()) {
            deliver_exception(EXC_GP, (uint16_t)(vector * 8 + 2));
            return;
        }
    }

    // 门不存在
    if ((g.type_attr & SEG_ACCESS_PRESENT) == 0) {
        fprintf(stderr, "[FAULT] IDT gate not present (vec %d)\n", vector);
        if (vector == EXC_DF) {
            fprintf(stderr, "[FAULT] #DF gate not present -> triple fault\n");
            cpu_running = false;
            return;
        }
        // 投 #DF（用 protect_interrupt 而不是 deliver_exception，避免 in_delivery 判断）
        protect_interrupt(EXC_DF, true, 0, false);
        return;
    }

    if (type == SYS_TYPE_TASKGATE) { protect_task_switch(g.selector); return; }

    if (type != SYS_TYPE_INTGATE && type != SYS_TYPE_TRAPGATE &&
        type != SYS_TYPE_INTGATE32 && type != SYS_TYPE_TRAPGATE32) {
        fprintf(stderr, "[FAULT] IDT entry not a valid gate (vec %d type %02X)\n", vector, type);
        if (in_delivery) {
            fprintf(stderr, "[FAULT] triple fault (invalid gate in delivery)\n");
            cpu_running = false;
            return;
        }
        in_delivery = 1;
        protect_interrupt(EXC_GP, true, (uint16_t)(vector * 8 + 2), false);
        in_delivery = 0;
        return;
    }

    uint16_t tsel = g.selector;
    uint8_t new_cpl = dpl;
    uint32_t tb, tl;
    if (!load_cs_target(tsel, new_cpl, &tb, &tl)) return;
    uint32_t toff = gate_offset(&g, type);
    if (toff > tl) { deliver_exception(EXC_GP, 0); return; }

    uint16_t old_cs = cpu.cs, old_ip = cpu.ip, old_ss = cpu.ss, old_sp = cpu.sp;
    bool err = (vector < 32) && vec_has_error[vector];

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

// 对外的异常入口
void protect_exception(int vector, uint16_t error_code) {
    deliver_exception(vector, error_code);
}

void protect_fault_shutdown(const char* why) {
    (void)why;
    cpu_running = false;
}

// ============================================================
// 任务切换
// ============================================================
void protect_task_switch(uint16_t tss_sel) {
    SegmentDescriptor d;
    uint32_t b, l;
    if ((tss_sel & 0xFFF8u) == 0 || !read_desc_sel(tss_sel, &d, &b, &l, NULL, true)) {
        deliver_exception(EXC_TS, tss_sel & 0xFFFC);
        return;
    }
    uint8_t type = d.access & 0x1F;
    if ((d.access & SEG_ACCESS_S) || (type != SYS_TYPE_TSS16 && type != SYS_TYPE_TSS16_BUSY)) {
        deliver_exception(EXC_TS, tss_sel & 0xFFFC);
        return;
    }
    if (l < TSS_SIZE - 1) { deliver_exception(EXC_TS, tss_sel & 0xFFFC); return; }

    uint32_t new_tr = b;

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

    uint32_t tbl = (tss_sel & 4) ? protect.ldtr_base : protect.gdt_base;
    uint32_t daddr = tbl + (((tss_sel & 0xFFF8u) >> 3) * 8);
    cpu_mem_write(daddr + 5, (uint8_t)((d.access & 0xF8) | SYS_TYPE_TSS16_BUSY));

    protect.tr = tss_sel & 0xFFFC;
    protect.tr_base = new_tr;
    protect.tr_limit = (uint16_t)l;

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
}