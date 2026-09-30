// cpu.c - 8086/80286 CPU Emulation Implementation (Extended, BIOS boot)
#include "cpu.h"
#include "cpu386.h"
#include "protect.h"
#include "fpu.h"
#include <stdio.h>
#include <string.h>
#include "io.h"
#include "ide.h"

extern bool debug_mode;   // 主程序 -dbg 打开

// 客机主频（运行时可改，见窗口顶部「频率」菜单）。默认 20MHz Turbo。
uint32_t cpu_clk_hz = CPU_CLK_HZ_DEFAULT;

// 所有数据访问都过物理内存门：A20 关闭时折回 1MB 以内（与 8086 一致），
// 打开后可用 24 位地址；越出已装内存读回 0xFF、ROM 区写入被忽略。
#define MEM(a) cpu_mem_read((uint32_t)(a))
CPU_8086 cpu;
uint8_t* memory = NULL;
uint32_t memory_size = 0x200000;   // 2MB（见 cpu.h）
bool cpu_running = true;
static uint16_t seg_override = 0;
// 段覆盖前缀是否生效。不能用 seg_override==0 判断"没有覆盖"：ES/DS/SS 在早期
// POST 阶段经常就是 0000，那样会把合法的 ES: 前缀当成"无覆盖"丢掉。
static bool seg_override_active = false;
bool cpu_prefix_step = false;   // 上一步只执行了前缀字节（见 cpu.h 说明）
bool cpu_halted = false;
uint64_t hlt_count = 0;   // 客机累计执行过的 HLT 次数（cpu_halted 为 1 时它一定在涨）
// 客机时钟周期（按 CPU_CLK_HZ = 20MHz 计）；见 cpu.h
uint64_t cpu_cycles = 0;
uint32_t cpu_last_cycles = 0;
// 是否正在投递异常处理程序（用于双重/三重故障判定）；IRET 时清零
static int in_fault = 0;

// BIOS ROM 写保护：0xF0000~0xFFFFF 的写入被忽略（与真实硬件一致）。
//   注意地址必须先过 A20 门再看范围，且 1MB 以上（HMA）不是 ROM。
bool cpu_rom_write_protect = false;

// TEMP：写监视（GDT/IDT 关键字节 + GDT[9] base_mid 补丁序列 + GDT/IDT 被清零）
static int zero_budget = 400;
void cpu_mem_watch(uint32_t addr, uint8_t val) {
	// 一击定位：GDT 描述符区(0xD8A0..0xD930) 被写成 0；按 CS:IP 去重，只报新站点
	if (zero_budget > 0 && val == 0 && memory[addr] != 0 &&
	    addr >= 0xD8A0 && addr < 0xD930) {
		static uint16_t zcs[12], zip[12]; static int zc[12], zn = 0;
		int found = -1;
		for (int i = 0; i < zn; i++) if (zcs[i] == cpu.cs && zip[i] == cpu.ip) { found = i; break; }
		if (found >= 0) { zc[found]++; }
		else if (zn < 12) {
			zcs[zn] = cpu.cs; zip[zn] = cpu.ip; zc[zn] = 1; zn++;
			zero_budget--;
			fprintf(stderr, "[ZEROIP#%d] %06X old=%02X @%04X:%04X es=%04X(%06X) di=%04X cx=%04X\n",
			        zn, addr, memory[addr], cpu.cs, cpu.ip, cpu.es, cpu.seg_base[SEG_ES], cpu.di, cpu.cx);
			fflush(stderr);
		}
	}
	if (addr == 0xD8E5) PMLOG("[W] GDT8.access=%02X @%04X:%04X\n", val, cpu.cs, cpu.ip);
	if (addr == 0xD8ED) PMLOG("[W] GDT9.access=%02X @%04X:%04X\n", val, cpu.cs, cpu.ip);
	if (addr == 0xD0FD) PMLOG("[W] IDT11.type=%02X @%04X:%04X\n", val, cpu.cs, cpu.ip);
	if (addr == 0xD8EC) PMLOG("[W] GDT9.base_mid=%02X @%04X:%04X\n", val, cpu.cs, cpu.ip);
}

// ============================================================
// A20 地址线（80286）
//   IBM AT 的 8042 输出口 bit1 上电为 1 → A20 默认开启（真机如此：AT BIOS 的
//   POST 1D/1F 要求 A20 已开，且 BIOS 只在交出引导前用 AH=DD 关掉它）。
//   XT（8088）没有 A20 门，cpu_is_286=false，地址永远 20 位折回。
//   物理内存读写（cpu_mem_read/cpu_mem_write）是热路径，已内联在 cpu.h。
// ============================================================
bool cpu_a20_enabled = false;
bool cpu_is_286 = false;
bool cpu_reset_pending = false;

static int a20_budget = 40;   // TEMP
void cpu_set_a20(bool on) {
    if (on != cpu_a20_enabled && a20_budget > 0) {
        a20_budget--;
        fprintf(stderr, "[A20] %d->%d @%04X:%04X\n", (int)cpu_a20_enabled, (int)on, cpu.cs, cpu.ip);
        fflush(stderr);
    }
    cpu_a20_enabled = on;
}

void cpu_request_reset(void) {
    cpu_reset_pending = true;
}

// ============================================================
// 段基址 / 限长缓存（实模式基址 = 选择子 << 4；保护模式由描述符填入）
//   cpu_seg_base(sel) 按"选择子值"查缓存：同一个选择子必然对应同一个
//   描述符，所以基址/限长是选择子的函数，重名不会混淆。
// ============================================================
static uint16_t cpu_seg_sel(int idx) {
	switch (idx) {
		case SEG_ES: return cpu.es;
		case SEG_CS: return cpu.cs;
		case SEG_SS: return cpu.ss;
		case SEG_DS: return cpu.ds;
		case SEG_FS: return cpu.fs;
		default:     return cpu.gs;
	}
}

uint32_t cpu_seg_base(uint16_t sel) {
	if (!protect.pe) return (uint32_t)sel << 4;
	for (int i = 0; i < 6; i++) if (cpu_seg_sel(i) == sel) return cpu.seg_base[i];
	return (uint32_t)sel << 4;
}

uint32_t cpu_seg_limit(uint16_t sel) {
	if (!protect.pe) return 0xFFFF;
	for (int i = 0; i < 6; i++) if (cpu_seg_sel(i) == sel) return cpu.seg_limit[i];
	return 0xFFFF;
}

// 描述符访问字节。查不到（隐式访问等）按"可写数据段"处理，不做误判。
uint8_t cpu_seg_access(uint16_t sel) {
	if (!protect.pe) return 0x93;
	for (int i = 0; i < 6; i++) if (cpu_seg_sel(i) == sel) return cpu.seg_access[i];
	return 0x93;
}

// 选择子写在寄存器里之后，把基址/限长缓存同步过去
static void seg_cache_commit(int idx, uint16_t sel, uint32_t base, uint32_t limit, uint8_t access) {
	cpu.seg_base[idx] = base;
	cpu.seg_limit[idx] = limit;
	cpu.seg_access[idx] = access;
	(void)sel;
}

void cpu_load_seg_reg(int idx, uint16_t sel) {
	if (!protect.pe) {
		switch (idx) {
			case SEG_ES: cpu.es = sel; break;
			case SEG_CS: cpu.cs = sel; break;
			case SEG_SS: cpu.ss = sel; break;
			case SEG_DS: cpu.ds = sel; break;
			case SEG_FS: cpu.fs = sel; break;
			default:     cpu.gs = sel; break;
		}
		seg_cache_commit(idx, sel, (uint32_t)sel << 4, 0xFFFF, 0x93);
		return;
	}

	// 保护模式：先校验并取描述符，成功后才真正改寄存器
	uint32_t base, limit; uint8_t access;
	if (!protect_load_seg(idx, sel, &base, &limit, &access)) return;
	switch (idx) {
		case SEG_ES: cpu.es = sel; break;
		case SEG_CS: cpu.cs = sel; break;
		case SEG_SS: cpu.ss = sel; break;
		case SEG_DS: cpu.ds = sel; break;
		case SEG_FS: cpu.fs = sel; break;
		default:     cpu.gs = sel; break;
	}
	seg_cache_commit(idx, sel, base, limit, access);
}

// 字符串指令用的可覆盖源段（供 cpu386.c 的 OUTS 使用）
uint16_t cpu_get_seg_ds(void) {
	if (seg_override_active) { uint16_t s = seg_override; seg_override_active = false; return s; }
	return cpu.ds;
}

void cpu_init(void) {
    memory = calloc(memory_size, 1);   // 8MB（0x800000）
    memset(&memory[0xF0000], 0xFF, 0x10000);
    cpu386_init();
    protect_init();
    cpu_reset();
}

void cpu_reset(void) {
    memset(&cpu, 0, sizeof(CPU_8086));
    // ★ 8086 复位向量：CS=FFFF, IP=0000，从 BIOS ROM 开始执行
    cpu.cs = 0xFFFF;
    cpu.ip = 0x0000;
    cpu.flags = 0x0002;
    cpu_running = true;
    seg_override = 0;
    seg_override_active = false;
    cpu_prefix_step = false;
    cpu_halted = false;
    cpu.prefix_66 = false;   // ★ 这里
    cpu.prefix_67 = false;   // ★ 这里
    // 段缓存：实模式下基址 = 选择子<<4、限长 0xFFFF（保护模式由描述符填入）
    for (int i = 0; i < 6; i++) { cpu.seg_base[i] = 0; cpu.seg_limit[i] = 0xFFFF; cpu.seg_access[i] = 0x93; }
    protect_reset();
    cpu386_reset();
    cpu_a20_enabled = cpu_is_286;   // AT 上电 A20 已开；XT 无 A20 门（20 位折回）
}


// ★ 加载 BIOS ROM 到 0xF0000 末端
void load_bios(const char* filename) {
    FILE* fp = fopen(filename, "rb");
    if (!fp) {
        fprintf(stderr, "[BIOS] not found: %s\n", filename);
        return;
    }
    fseek(fp, 0, SEEK_END);
    long size = ftell(fp);
    fseek(fp, 0, SEEK_SET);
    if (size > 0x10000) size = 0x10000;
    fread(&memory[0x100000 - size], 1, size, fp);
    fclose(fp);
    cpu_rom_write_protect = true;
    // 整块 64KB 的 ROM 是 AT（286）置的 BIOS；XT 的 BIOS 只有 8KB。
    // 机器类型决定 A20 门上电默认值（见 cpu_reset），必须在装载后立刻生效。
    cpu_is_286 = (size == 0x10000);
    cpu_a20_enabled = cpu_is_286;
    printf("[BIOS] loaded %s (%ld bytes at 0x%05lX)\n",
           filename, size, 0x100000L - size);
}

// ============================================================
// 保护模式数据访问的限长检查（80286 段限长）
//   数据访问的最后一字节偏移 > 段限长 → #GP(13)。实模式限长恒为 0xFFFF，
//   且 16 位偏移 + size - 1 会到 0x10000，故只在保护模式下检查。
//   异常在访问发生时立即投递（cpu.cs/ip 变成处理程序入口），但译码还会继续
//   推进 cpu.ip（如 `cpu.ip += 2`），把它覆盖掉。所以记下入口地址，在指令
//   公共尾部恢复一次（见 cpu_execute_instruction 末尾）。
//   IBM AT 的 POST 就是靠这个自检：把 GDT 某段的限长设成 0，读 2 字节必须
//   触发 #GP，处理程序据此确认 CPU 真的做了限长检查。
// ============================================================
static bool seg_fault_abort = false;
static uint16_t seg_fault_cs = 0, seg_fault_ip = 0;

// 段访问检查：限长（读/写都要）+ 写权限（仅写）。
static void seg_access_check(uint16_t seg, uint32_t off, uint32_t size, bool is_write) {
    if (!protect.pe || seg_fault_abort) return;
    bool bad = (uint64_t)off + size - 1 > (uint64_t)cpu_seg_limit(seg);
    if (!bad && is_write) {
        // 286 写保护：代码段不可写；数据段访问字节的 RW 位为 0（只读）不可写。
        uint8_t a = cpu_seg_access(seg);
        if ((a & SEG_ACCESS_CODE) || !(a & SEG_ACCESS_RW)) bad = true;
    }
    if (bad) {
        protect_exception(EXC_GP, 0);
        seg_fault_cs = cpu.cs;
        seg_fault_ip = cpu.ip;
        seg_fault_abort = true;
    }
}

// 段式地址（含限长/写权限检查）。越权读写都已被 seg_access_check 投递 #GP。
static inline uint32_t seg_lin(uint16_t seg, uint16_t off, uint32_t size, bool is_write) {
    seg_access_check(seg, off, size, is_write);
    return cpu_seg_base(seg) + off;
}

uint8_t read_byte(uint16_t seg, uint16_t off) {
    return cpu_mem_read(seg_lin(seg, off, 1, false));
}

uint16_t read_word(uint16_t seg, uint16_t off) {
    uint32_t addr = seg_lin(seg, off, 2, false);
    return cpu_mem_read(addr) | (cpu_mem_read(addr + 1) << 8);
}

void write_byte(uint16_t seg, uint16_t off, uint8_t val) {
    cpu_mem_write(seg_lin(seg, off, 1, true), val);
}

void write_word(uint16_t seg, uint16_t off, uint16_t val) {
    uint32_t addr = seg_lin(seg, off, 2, true);
    cpu_mem_write(addr, val & 0xFF);
    cpu_mem_write(addr + 1, val >> 8);
}

void push(uint16_t val) {
    cpu.sp -= 2;
    write_word(cpu.ss, cpu.sp, val);
}

uint16_t pop(void) {
    uint16_t val = read_word(cpu.ss, cpu.sp);
    cpu.sp += 2;
    return val;
}

void set_flag(uint16_t flag) { cpu.flags |= flag; }
void clear_flag(uint16_t flag) { cpu.flags &= ~flag; }
bool get_flag(uint16_t flag) { return (cpu.flags & flag) != 0; }

// ============================================================
// 算术标志更新（含 OF / AF）
// ============================================================
static void update_arith_flags(uint16_t result, uint16_t a, uint16_t b, bool is_sub) {
    if (is_sub) {
        if (a < b) set_flag(FLAG_CF); else clear_flag(FLAG_CF);
        uint16_t diff = a ^ b;
        uint16_t res_xor = a ^ result;
        if ((diff & 0x8000) && (res_xor & 0x8000)) set_flag(FLAG_OF);
        else clear_flag(FLAG_OF);
        if (((a ^ b ^ result) & 0x10)) set_flag(FLAG_AF); else clear_flag(FLAG_AF);
    } else {
        if (result < a) set_flag(FLAG_CF); else clear_flag(FLAG_CF);
        uint16_t res_xor = a ^ result;
        if ((~(a ^ b) & res_xor & 0x8000)) set_flag(FLAG_OF);
        else clear_flag(FLAG_OF);
        if (((a ^ b ^ result) & 0x10)) set_flag(FLAG_AF); else clear_flag(FLAG_AF);
    }
    if (result == 0) set_flag(FLAG_ZF); else clear_flag(FLAG_ZF);
    if (result & 0x8000) set_flag(FLAG_SF); else clear_flag(FLAG_SF);
    int count = 0;
    for (int i = 0; i < 8; i++) if (result & (1 << i)) count++;
    if (count % 2 == 0) set_flag(FLAG_PF); else clear_flag(FLAG_PF);
}

static void update_arith_flags_8(uint8_t result, uint8_t a, uint8_t b, bool is_sub) {
    if (is_sub) {
        if (a < b) set_flag(FLAG_CF); else clear_flag(FLAG_CF);
        uint8_t diff = a ^ b;
        uint8_t res_xor = a ^ result;
        if ((diff & 0x80) && (res_xor & 0x80)) set_flag(FLAG_OF);
        else clear_flag(FLAG_OF);
        if (((a ^ b ^ result) & 0x10)) set_flag(FLAG_AF); else clear_flag(FLAG_AF);
    } else {
        if (result < a) set_flag(FLAG_CF); else clear_flag(FLAG_CF);
        uint8_t res_xor = a ^ result;
        if ((~(a ^ b) & res_xor & 0x80)) set_flag(FLAG_OF);
        else clear_flag(FLAG_OF);
        if (((a ^ b ^ result) & 0x10)) set_flag(FLAG_AF); else clear_flag(FLAG_AF);
    }
    if (result == 0) set_flag(FLAG_ZF); else clear_flag(FLAG_ZF);
    if (result & 0x80) set_flag(FLAG_SF); else clear_flag(FLAG_SF);
    int count = 0;
    for (int i = 0; i < 8; i++) if (result & (1 << i)) count++;
    if (count % 2 == 0) set_flag(FLAG_PF); else clear_flag(FLAG_PF);
}

// 逻辑运算标志（AND/OR/XOR/TEST）：CF=OF=0，AF 未定义
static void update_logic_flags16(uint16_t result) {
    if (result == 0) set_flag(FLAG_ZF); else clear_flag(FLAG_ZF);
    if (result & 0x8000) set_flag(FLAG_SF); else clear_flag(FLAG_SF);
    int count = 0;
    for (int i = 0; i < 8; i++) if (result & (1 << i)) count++;
    if (count % 2 == 0) set_flag(FLAG_PF); else clear_flag(FLAG_PF);
    clear_flag(FLAG_CF);
    clear_flag(FLAG_OF);
}

static void update_logic_flags8(uint8_t result) {
    if (result == 0) set_flag(FLAG_ZF); else clear_flag(FLAG_ZF);
    if (result & 0x80) set_flag(FLAG_SF); else clear_flag(FLAG_SF);
    int count = 0;
    for (int i = 0; i < 8; i++) if (result & (1 << i)) count++;
    if (count % 2 == 0) set_flag(FLAG_PF); else clear_flag(FLAG_PF);
    clear_flag(FLAG_CF);
    clear_flag(FLAG_OF);
}

uint16_t* reg16_table[] = {&cpu.ax, &cpu.cx, &cpu.dx, &cpu.bx, &cpu.sp, &cpu.bp, &cpu.si, &cpu.di};
static uint16_t* seg_table[] = {&cpu.es, &cpu.cs, &cpu.ss, &cpu.ds, &cpu.fs, &cpu.gs};
// 段寄存器索引（与 cpu.h 的 SEG_* 一致）；0x8C/0x8E/B2/B4/B5 用它做映射
static const int seg_index_tbl[6] = { SEG_ES, SEG_CS, SEG_SS, SEG_DS, SEG_FS, SEG_GS };

static uint16_t get_seg(void) {
    if (seg_override_active) { uint16_t s = seg_override; seg_override_active = false; return s; }
    return cpu.ds;
}

// 解析 ModRM 的有效地址：返回 16 位偏移（含 disp），*seg_out = 所用段选择子。
//   段覆盖前缀在此处"读"但不清（见下方说明），由 cpu_execute_instruction 末尾清。
static uint16_t decode_modrm_ea(uint8_t modrm, uint16_t* disp_out, uint16_t* seg_out) {
    uint8_t mod = (modrm >> 6) & 0x03;
    uint8_t rm = modrm & 0x07;
    uint16_t disp = 0;
    uint16_t seg = cpu.ds;
    uint32_t addr = 0;
    switch (rm) {
        case 0: addr = cpu.bx + cpu.si; break;
        case 1: addr = cpu.bx + cpu.di; break;
        case 2: addr = cpu.bp + cpu.si; seg = cpu.ss; break;
        case 3: addr = cpu.bp + cpu.di; seg = cpu.ss; break;
        case 4: addr = cpu.si; break;
        case 5: addr = cpu.di; break;
        case 6: addr = cpu.bp; seg = cpu.ss; break;
        case 7: addr = cpu.bx; break;
    }
    if (mod == 0 && rm == 6) {
            addr = read_word(cpu.cs, cpu.ip + 2);
            seg = cpu.ds;   // 直接寻址用 DS，不是 SS
    } else if (mod == 1) {
    	disp = (int8_t)read_byte(cpu.cs, cpu.ip + 2);
    	addr += disp;
	} else if (mod == 2) {
    	disp = read_word(cpu.cs, cpu.ip + 2);
    	addr += disp;
    }
    // 段覆盖前缀不能在解码有效地址时就消耗掉：read-modify-write 类指令
    // （如 ADD SS:[0032],CL）会先 read_modrm 再 write_modrm，各自调用本函数。
    // 若第一次就清掉，第二次读写会退回 DS，导致写入错误的段。
    // 统一由 cpu_execute_instruction 末尾清除。
    if (seg_override_active) { seg = seg_override; }
    if (disp_out) *disp_out = disp;
    if (seg_out) *seg_out = seg;
    // 有效地址是 16 位（8086/286 实模式把 bx+si 等算出的偏移折回 16 位）。
    return (uint16_t)addr;
}

uint32_t decode_modrm_addr(uint8_t modrm, uint16_t* disp_out) {
    uint16_t seg;
    uint16_t off = decode_modrm_ea(modrm, disp_out, &seg);
    return cpu_seg_base(seg) + off;
}

uint16_t read_modrm16(uint8_t modrm) {
    uint8_t mod = (modrm >> 6) & 0x03;
    uint8_t rm = modrm & 0x07;
    if (mod == 3) return *reg16_table[rm];
    uint16_t seg;
    uint16_t off = decode_modrm_ea(modrm, NULL, &seg);
    uint32_t addr = seg_lin(seg, off, 2, false);
    return cpu_mem_read(addr) | (cpu_mem_read(addr + 1) << 8);
}
uint8_t get_reg8_val(uint8_t reg);
void set_reg8_val(uint8_t reg, uint8_t val);
uint8_t read_modrm8(uint8_t modrm) {
    uint8_t mod = (modrm >> 6) & 0x03;
    uint8_t rm = modrm & 0x07;
    if (mod == 3) return get_reg8_val(rm);   // ★ 改这里
    uint16_t seg;
    uint16_t off = decode_modrm_ea(modrm, NULL, &seg);
    return cpu_mem_read(seg_lin(seg, off, 1, false));
}
uint8_t get_reg8_val(uint8_t reg) {
	if (reg < 4) return *reg16_table[reg] & 0xFF;
	return (*reg16_table[reg & 3] >> 8) & 0xFF;
}


void write_modrm16(uint8_t modrm, uint16_t val) {
    uint8_t mod = (modrm >> 6) & 0x03;
    uint8_t rm = modrm & 0x07;
    if (mod == 3) { *reg16_table[rm] = val; return; }
    uint16_t seg;
    uint16_t off = decode_modrm_ea(modrm, NULL, &seg);
    uint32_t addr = seg_lin(seg, off, 2, true);
    cpu_mem_write(addr, val & 0xFF);
    cpu_mem_write(addr + 1, val >> 8);
}

// 按已算好的有效地址写 16 位（用于一条指令里已经解码过一次 ModRM 的场合，
// 避免二次 decode_modrm_addr 丢掉段覆盖前缀）
void write_word_at(uint32_t addr, uint16_t val) {
    cpu_mem_write(addr, val & 0xFF);
    cpu_mem_write(addr + 1, val >> 8);
}

void write_modrm8(uint8_t modrm, uint8_t val) {
	uint8_t mod = (modrm >> 6) & 0x03;
	uint8_t rm = modrm & 0x07;
	if (mod == 3) {
		set_reg8_val(rm, val);
		return;
	}
	uint16_t seg;
	uint16_t off = decode_modrm_ea(modrm, NULL, &seg);
	cpu_mem_write(seg_lin(seg, off, 1, true), val);
}
void set_reg8_val(uint8_t reg, uint8_t val) {
    if (reg < 4) *reg16_table[reg] = (*reg16_table[reg] & 0xFF00) | val;
    else *reg16_table[reg & 3] = (*reg16_table[reg & 3] & 0x00FF) | (val << 8);
}

void handle_modrm_ip(uint8_t modrm) {
    uint8_t mod = (modrm >> 6) & 0x03;
    uint8_t rm = modrm & 0x07;
    if (mod == 3) return;
    if (mod == 0 && rm == 6) cpu.ip += 2;
    else if (mod == 1) cpu.ip += 1;
    else if (mod == 2) cpu.ip += 2;
}

// 取 ModR/M 后立即数偏移（用于 IMUL 等）
static int modrm_imm_offset(uint8_t modrm) {
    uint8_t mod = (modrm >> 6) & 0x03;
    uint8_t rm = modrm & 0x07;
    if (mod == 0 && rm == 6) return 2;
    if (mod == 1) return 1;
    if (mod == 2) return 2;
    return 0;
}

// ============================================================
// 8088 指令周期表（MCS-86 手册的 8088 原始周期数，与主频无关）
//   数值取自 Intel MCS-86 手册的 8088 表。模拟器用它把"一条指令"换算成
//   "多少时钟周期"，虚拟时间（PIT/CGA 时序）再由周期推导 —— 这样
//   CPU : 定时器 : 视频 三者的比例与真机一致，8088 MPH 的速度自检才可能通过。
// ============================================================
static uint32_t ea_cyc(uint8_t modrm) {
  // EA（有效地址）计算周期：8086/8088 地址方式代价表
  static const uint8_t t[3][8] = {
    {  7,  8,  8,  7,  5,  5,  6,  5 },   // mod=00：[BX+SI] [BX+DI] [BP+SI] [BP+DI] [SI] [DI] disp16 [BX]
    { 11, 12, 12, 11,  9,  9, 10,  9 },   // mod=01：+disp8
    { 15, 16, 16, 15, 13, 13, 14, 13 }    // mod=10：+disp16
  };
  uint8_t mod = (modrm >> 6) & 3;
  if (mod == 3) return 0;
  return t[mod][modrm & 7];
}

static bool jcc_taken(uint8_t cond) {
  bool cf = get_flag(FLAG_CF), zf = get_flag(FLAG_ZF), sf = get_flag(FLAG_SF);
  bool of = get_flag(FLAG_OF), pf = get_flag(FLAG_PF);
  switch (cond & 0x0F) {
    case 0x0: return of;                    // JO
    case 0x1: return !of;                   // JNO
    case 0x2: return cf;                    // JB/JC
    case 0x3: return !cf;                   // JAE/JNC
    case 0x4: return zf;                    // JE/JZ
    case 0x5: return !zf;                   // JNE/JNZ
    case 0x6: return cf || zf;              // JBE
    case 0x7: return !cf && !zf;            // JA
    case 0x8: return sf;                    // JS
    case 0x9: return !sf;                   // JNS
    case 0xA: return pf;                    // JP
    case 0xB: return !pf;                   // JNP
    case 0xC: return sf != of;              // JL
    case 0xD: return sf == of;              // JGE
    case 0xE: return zf || (sf != of);      // JLE
    default:  return !zf && (sf == of);     // JG
  }
}

// 一条指令耗掉的周期数；只读 cpu.ip 处的操作码与紧随的 ModR/M，不改任何状态
static uint32_t insn_cyc(uint8_t op, uint8_t modrm) {
  uint32_t ea = ea_cyc(modrm);
  bool reg = ((modrm >> 6) & 3) == 3;      // 操作数在寄存器里（无内存访问）
  uint8_t form = op & 0x07;

  // ---- 0x00~0x3F：ALU 组 / 段寄存器 PUSH-POP / ASCII 调整 / 段前缀 ----
  if (op <= 0x3F) {
    if (form <= 5) {
      switch (form) {
        case 0: return reg ? 3 : 16 + ea;   // Eb,Gb  内存←寄存器
        case 1: return reg ? 3 : 24 + ea;   // Ev,Gv
        case 2: return reg ? 3 :  9 + ea;   // Gb,Eb  寄存器←内存
        case 3: return reg ? 3 : 13 + ea;   // Gv,Ev
        default: return 4;                  // AL/AX, 立即数
      }
    }
    switch (op) {                           // form=6/7 的例外
      case 0x06: case 0x0E: case 0x16: case 0x1E: return 14;  // PUSH 段寄存器
      case 0x07: case 0x17: case 0x1F: return 12;             // POP  段寄存器
      case 0x27: case 0x2F: case 0x37: case 0x3F: return 4;   // DAA/DAS/AAA/AAS
      default: return 2;                                      // 段覆盖前缀
    }
  }

  if (op >= 0x40 && op <= 0x4F) return 3;                  // INC/DEC r16
  if (op >= 0x50 && op <= 0x57) return 15;                 // PUSH r16
  if (op >= 0x58 && op <= 0x5F) return 12;                 // POP  r16
  if (op >= 0x70 && op <= 0x7F)                            // Jcc
    return jcc_taken(op & 0x0F) ? 16 : 4;

  if (op >= 0x80 && op <= 0x83) {                          // ALU 立即数 → r/m
    if (reg) return 4;
    return ((op == 0x81 || op == 0x83) ? 23 : 17) + ea;
  }
  if (op == 0x84) return reg ? 3 : 9 + ea;                 // TEST Eb,Gb
  if (op == 0x85) return reg ? 3 : 13 + ea;                // TEST Ev,Gv
  if (op == 0x86) return reg ? 4 : 17 + ea;                // XCHG Eb,Gb
  if (op == 0x87) return reg ? 4 : 25 + ea;                // XCHG Ev,Gv
  if (op == 0x88) return reg ? 2 : 9 + ea;                 // MOV Eb,Gb
  if (op == 0x89) return reg ? 2 : 13 + ea;                // MOV Ev,Gv
  if (op == 0x8A) return reg ? 2 : 8 + ea;                 // MOV Gb,Eb
  if (op == 0x8B) return reg ? 2 : 12 + ea;                // MOV Gv,Ev
  if (op == 0x8C) return reg ? 2 : 9 + ea;                 // MOV r/m16,Sreg
  if (op == 0x8D) return 2;                                // LEA
  if (op == 0x8E) return reg ? 2 : 8 + ea;                 // MOV Sreg,r/m16
  if (op == 0x8F) return 17 + ea;                          // POP r/m16

  if (op == 0x90) return 3;                                // NOP（与 XCHG AX,AX 同）
  if (op >= 0x91 && op <= 0x97) return 3;                  // XCHG AX,r16
  if (op == 0x98) return 2;                                // CBW
  if (op == 0x99) return 5;                                // CWD
  if (op == 0x9A) return 28;                               // CALL far
  if (op == 0x9B) return 4;                                // WAIT
  if (op == 0x9C) return 14;                               // PUSHF
  if (op == 0x9D) return 12;                               // POPF
  if (op == 0x9E) return 4;                                // SAHF
  if (op == 0x9F) return 4;                                // LAHF

  if (op >= 0xA0 && op <= 0xA3)                            // MOV AL/AX,[moffs]
    return (op == 0xA1 || op == 0xA3) ? 14 : 10;

  if (op == 0xA4) return 18;                               // MOVSB
  if (op == 0xA5) return 26;                               // MOVSW
  if (op == 0xA6) return 22;                               // CMPSB
  if (op == 0xA7) return 30;                               // CMPSW
  if (op == 0xA8) return 4;                                // TEST AL,imm8
  if (op == 0xA9) return 4;                                // TEST AX,imm16
  if (op == 0xAA) return 11;                               // STOSB
  if (op == 0xAB) return 15;                               // STOSW
  if (op == 0xAC) return 12;                               // LODSB
  if (op == 0xAD) return 16;                               // LODSW
  if (op == 0xAE) return 15;                               // SCASB
  if (op == 0xAF) return 19;                               // SCASW

  if (op >= 0xB0 && op <= 0xBF) return 4;                  // MOV r8,imm8

  if (op == 0xC0 || op == 0xC1) return reg ? 5 : 20 + ea;  // 移位（立即数计数）
  if (op == 0xC2 || op == 0xC3) return 20;                 // RET
  if (op == 0xC4 || op == 0xC5) return 16 + ea;            // LES / LDS
  if (op == 0xC6) return reg ? 4 : 10 + ea;                // MOV Eb,Ib
  if (op == 0xC7) return reg ? 4 : 14 + ea;                // MOV Ev,Iv
  if (op == 0xCA) return 25;                               // RETF imm16
  if (op == 0xCB) return 26;                               // RETF
  if (op == 0xCC) return 52;                               // INT3
  if (op == 0xCD) return 51;                               // INT imm8
  if (op == 0xCE) return 53;                               // INTO
  if (op == 0xCF) return 32;                               // IRET

  if (op >= 0xD0 && op <= 0xD3) {                          // 移位 / 循环移位
    bool cl = (op & 1) != 0;                               // 计数来自 CL
    if (reg) return cl ? 8 + 4 * (cpu.cx & 0xFF) : 2;
    return (cl ? 20 : 15) + ea;
  }
  if (op == 0xD4) return 83;                               // AAM
  if (op == 0xD5) return 60;                               // AAD
  if (op == 0xD7) return 11;                               // XLAT

  if (op >= 0xE0 && op <= 0xE3) {                          // LOOPxx / JCXZ
    bool taken;
    if (op == 0xE3)      taken = (cpu.cx == 0);            // JCXZ
    else if (op == 0xE0) taken = (cpu.cx != 1) && !get_flag(FLAG_ZF);
    else if (op == 0xE1) taken = (cpu.cx != 1) &&  get_flag(FLAG_ZF);
    else                 taken = (cpu.cx != 1);            // LOOP
    return taken ? (op == 0xE3 ? 18 : 19) : (op == 0xE3 ? 6 : 5);
  }
  if (op == 0xE4) return 14;                               // IN  AL,imm8
  if (op == 0xE5) return 18;                               // IN  AX,imm8
  if (op == 0xE6) return 14;                               // OUT imm8,AL
  if (op == 0xE7) return 18;                               // OUT imm8,AX
  if (op == 0xE8) return 23;                               // CALL near
  if (op == 0xE9 || op == 0xEA || op == 0xEB) return 15;   // JMP
  if (op == 0xEC) return 12;                               // IN  AL,DX
  if (op == 0xED) return 16;                               // IN  AX,DX
  if (op == 0xEE) return 12;                               // OUT DX,AL
  if (op == 0xEF) return 16;                               // OUT DX,AX

  if (op == 0xF6 || op == 0xF7) {                          // 单操作数组
    uint8_t sub = (modrm >> 3) & 7;
    bool w = (op == 0xF7);
    switch (sub) {
      case 0: case 1: return reg ? 4 : ((w ? 13 : 9) + ea);        // TEST
      case 2: case 3: return reg ? 3 : ((w ? 24 : 16) + ea);       // NOT / NEG
      case 4: return reg ? (w ? 118 : 70) : ((w ? 122 : 74) + ea); // MUL
      case 5: return reg ? (w ? 128 : 80) : ((w ? 132 : 84) + ea); // IMUL
      case 6: return reg ? (w ? 144 : 80) : ((w ? 148 : 84) + ea); // DIV
      default:return reg ? (w ? 165 : 101) : ((w ? 169 : 105) + ea); // IDIV
    }
  }
  if (op == 0xFE) return reg ? 3 : 15 + ea;                // INC/DEC Eb
  if (op == 0xFF) {                                        // INC/DEC/CALL/JMP/PUSH Ev
    uint8_t sub = (modrm >> 3) & 7;
    switch (sub) {
      case 0: case 1: return reg ? 3 : 23 + ea;
      case 2: return 16 + ea;                              // CALL near
      case 3: return 37 + ea;                              // CALL far
      case 4: return 11 + ea;                              // JMP near
      case 5: return 24 + ea;                              // JMP far
      default:return 16 + ea;                              // PUSH r/m16
    }
  }
  if (op >= 0xF0 && op <= 0xFD) return 2;                  // 前缀 / HLT / 标志位操作
  return 4;                                                // 其余（186+ 扩展等）
}

// ============================================================
// 主译码循环
// ============================================================
void cpu_execute_instruction(void) {
    uint32_t addr = cpu_seg_base(cpu.cs) + cpu.ip;
    uint8_t opcode = cpu_mem_read(addr);

    // ★ 时序：先按操作码（及其 ModR/M）记下本步的周期数。
    //   前缀字节单独成一步，各记 2 周期；REP 的每一轮由递归调用自然累加。
    cpu_last_cycles = insn_cyc(opcode, cpu_mem_read(addr + 1));
    cpu_cycles += cpu_last_cycles;

  // 段前缀
    cpu_prefix_step = false;          // 默认：本步是完整指令，允许投递中断
    if (opcode == 0x26) { seg_override = cpu.es; seg_override_active = true; cpu_prefix_step = true; cpu.ip++; return; }
    if (opcode == 0x2E) { seg_override = cpu.cs; seg_override_active = true; cpu_prefix_step = true; cpu.ip++; return; }
    if (opcode == 0x36) { seg_override = cpu.ss; seg_override_active = true; cpu_prefix_step = true; cpu.ip++; return; }
    if (opcode == 0x3E) { seg_override = cpu.ds; seg_override_active = true; cpu_prefix_step = true; cpu.ip++; return; }
    if (opcode == 0x64) { seg_override = cpu.fs; seg_override_active = true; cpu_prefix_step = true; cpu.ip++; return; }   // FS:
    if (opcode == 0x65) { seg_override = cpu.gs; seg_override_active = true; cpu_prefix_step = true; cpu.ip++; return; }   // GS:
    if (opcode == 0x66) { cpu.prefix_66 = true; cpu_prefix_step = true; cpu.ip++; return; }
  if (opcode == 0x67) { cpu.prefix_67 = true; cpu_prefix_step = true; cpu.ip++; return; }
  if (opcode == 0xF0) { cpu_prefix_step = true; cpu.ip++; return; }

    // REP/REPE/REPNE 前缀
    if (opcode == 0xF2 || opcode == 0xF3) {
        uint8_t next = MEM(addr + 1);
        bool rep_ov = false;          // REP 与字符串指令之间是否夹了段覆盖前缀
        uint16_t rep_ov_seg = 0;

        // 允许 REP 与字符串指令之间夹一个段覆盖前缀（如 F3 2E A4）
        if (next == 0x26 || next == 0x2E || next == 0x36 || next == 0x3E) {
            rep_ov_seg = (next == 0x26) ? cpu.es :
                         (next == 0x2E) ? cpu.cs :
                         (next == 0x36) ? cpu.ss : cpu.ds;
            rep_ov = true;
            next = MEM(addr + 2);
        }

        if (next == 0xA4 || next == 0xA5 || next == 0xA6 || next == 0xA7 ||
            next == 0xAA || next == 0xAB || next == 0xAC || next == 0xAD ||
            next == 0xAE || next == 0xAF) {

            cpu.ip++;                       // 跳过 F2/F3 前缀
            if (rep_ov) cpu.ip++;           // 跳过夹在中间的段前缀
            uint32_t base_ip = cpu.ip;      // 指向字符串指令

            // 若 REP 之前已有段覆盖（如 2E F3 A4），保留它供每轮使用
            if (!rep_ov && seg_override_active) {
                rep_ov = true;
                rep_ov_seg = seg_override;
            }

            if (cpu.cx == 0) {
                // CX=0：REP 不执行任何数据搬运，只把 IP 推到指令之后
                cpu.ip = base_ip + 1;
                seg_override_active = false;   // ★ 前缀到此为止，不许留给下一条指令
                return;
            }

            while (cpu.cx != 0) {
                // 字符串指令会经 get_seg() 一次性消费掉段覆盖，这里每轮恢复
                if (rep_ov) { seg_override = rep_ov_seg; seg_override_active = true; }
                cpu.ip = base_ip;
                cpu_execute_instruction();
                cpu.cx--;

                if (opcode == 0xF2 && next == 0xA6) { if (get_flag(FLAG_ZF)) break; }
                if (opcode == 0xF2 && next == 0xA7) { if (get_flag(FLAG_ZF)) break; }
                if (opcode == 0xF2 && next == 0xAE) { if (get_flag(FLAG_ZF)) break; }
                if (opcode == 0xF2 && next == 0xAF) { if (get_flag(FLAG_ZF)) break; }
                if (opcode == 0xF3 && next == 0xA6) { if (!get_flag(FLAG_ZF)) break; }
                if (opcode == 0xF3 && next == 0xA7) { if (!get_flag(FLAG_ZF)) break; }
                if (opcode == 0xF3 && next == 0xAE) { if (!get_flag(FLAG_ZF)) break; }
                if (opcode == 0xF3 && next == 0xAF) { if (!get_flag(FLAG_ZF)) break; }
            }
            return;
        }
        cpu_prefix_step = true;   // 孤立的 REP 前缀：下一步才是真正要执行的指令
        cpu.ip++;
        return;
    }

    switch (opcode) {
        // MOV r8, imm8
        case 0xB0: case 0xB1: case 0xB2: case 0xB3:
        case 0xB4: case 0xB5: case 0xB6: case 0xB7: {
            uint8_t imm = MEM(addr + 1);
            set_reg8_val(opcode & 0x07, imm);
            cpu.ip += 2;
            break;
        }

        // MOV r16, imm16
        case 0xB8: case 0xB9: case 0xBA: case 0xBB:
        case 0xBC: case 0xBD: case 0xBE: case 0xBF: {
            *reg16_table[opcode & 0x07] = read_word(cpu.cs, cpu.ip + 1);
            cpu.ip += 3;
            break;
        }

        // ADD AL, imm8
        case 0x04: {
            uint8_t imm = MEM(addr + 1);
            uint8_t a = cpu.ax & 0xFF;
            uint8_t result = a + imm;
            update_arith_flags_8(result, a, imm, false);
            cpu.ax = (cpu.ax & 0xFF00) | result;
            cpu.ip += 2;
            break;
        }

        // ADD AX, imm16
        case 0x05: {
            uint16_t imm = read_word(cpu.cs, cpu.ip + 1);
            uint16_t result = cpu.ax + imm;
            update_arith_flags(result, cpu.ax, imm, false);
            cpu.ax = result;
            cpu.ip += 3;
            break;
        }

        // DAA
        case 0x27: {
            uint8_t al = cpu.ax & 0xFF;
            uint8_t old_al = al;
            bool old_cf = get_flag(FLAG_CF);
            bool old_af = get_flag(FLAG_AF);
            if ((al & 0x0F) > 9 || old_af) {
                al += 6;
                set_flag(FLAG_AF);
            } else {
                clear_flag(FLAG_AF);
            }
            if (old_al > 0x99 || old_cf) {
                al += 0x60;
                set_flag(FLAG_CF);
            } else {
                clear_flag(FLAG_CF);
            }
            cpu.ax = (cpu.ax & 0xFF00) | al;
            update_logic_flags8(al);
            if (get_flag(FLAG_CF)) set_flag(FLAG_CF);
            if (get_flag(FLAG_AF)) set_flag(FLAG_AF);
            cpu.ip += 1;
            break;
        }

        // DAS
        case 0x2F: {
            uint8_t al = cpu.ax & 0xFF;
            uint8_t old_al = al;
            bool old_cf = get_flag(FLAG_CF);
            bool old_af = get_flag(FLAG_AF);
            if ((al & 0x0F) > 9 || old_af) {
                al -= 6;
                set_flag(FLAG_AF);
            } else {
                clear_flag(FLAG_AF);
            }
            if (old_al > 0x99 || old_cf) {
                al -= 0x60;
                set_flag(FLAG_CF);
            } else {
                clear_flag(FLAG_CF);
            }
            cpu.ax = (cpu.ax & 0xFF00) | al;
            update_logic_flags8(al);
            if (get_flag(FLAG_CF)) set_flag(FLAG_CF);
            if (get_flag(FLAG_AF)) set_flag(FLAG_AF);
            cpu.ip += 1;
            break;
        }

        // AAA
        case 0x37: {
            uint8_t al = cpu.ax & 0xFF;
            if ((al & 0x0F) > 9 || get_flag(FLAG_AF)) {
                cpu.ax = (cpu.ax & 0xFF00) | ((al + 6) & 0x0F);
                cpu.ax += 0x100;
                set_flag(FLAG_AF);
                set_flag(FLAG_CF);
            } else {
                cpu.ax = (cpu.ax & 0xFF00) | (al & 0x0F);
                clear_flag(FLAG_AF);
                clear_flag(FLAG_CF);
            }
            cpu.ip += 1;
            break;
        }

        // AAS
        case 0x3F: {
            uint8_t al = cpu.ax & 0xFF;
            if ((al & 0x0F) > 9 || get_flag(FLAG_AF)) {
                cpu.ax = (cpu.ax & 0xFF00) | ((al - 6) & 0x0F);
                cpu.ax -= 0x100;
                set_flag(FLAG_AF);
                set_flag(FLAG_CF);
            } else {
                cpu.ax = (cpu.ax & 0xFF00) | (al & 0x0F);
                clear_flag(FLAG_AF);
                clear_flag(FLAG_CF);
            }
            cpu.ip += 1;
            break;
        }

        // SUB AL, imm8
        case 0x2C: {
            uint8_t imm = MEM(addr + 1);
            uint8_t a = cpu.ax & 0xFF;
            uint8_t result = a - imm;
            update_arith_flags_8(result, a, imm, true);
            cpu.ax = (cpu.ax & 0xFF00) | result;
            cpu.ip += 2;
            break;
        }

        // SUB AX, imm16
        case 0x2D: {
            uint16_t imm = read_word(cpu.cs, cpu.ip + 1);
            uint16_t result = cpu.ax - imm;
            update_arith_flags(result, cpu.ax, imm, true);
            cpu.ax = result;
            cpu.ip += 3;
            break;
        }

        // INC r16
        case 0x40: case 0x41: case 0x42: case 0x43:
        case 0x44: case 0x45: case 0x46: case 0x47: {
            uint16_t* r = reg16_table[opcode & 0x07];
            uint16_t old = *r;
            uint16_t result = old + 1;
            // INC 不影响 CF
            bool cf = get_flag(FLAG_CF);
            update_arith_flags(result, old, 1, false);
            if (cf) set_flag(FLAG_CF); else clear_flag(FLAG_CF);
            *r = result;
            cpu.ip += 1;
            break;
        }

        // DEC r16
        case 0x48: case 0x49: case 0x4A: case 0x4B:
        case 0x4C: case 0x4D: case 0x4E: case 0x4F: {
            uint16_t* r = reg16_table[opcode & 0x07];
            uint16_t old = *r;
            uint16_t result = old - 1;
            bool cf = get_flag(FLAG_CF);
            update_arith_flags(result, old, 1, true);
            if (cf) set_flag(FLAG_CF); else clear_flag(FLAG_CF);
            *r = result;
            cpu.ip += 1;
            break;
        }

        // PUSH r16
        case 0x50: case 0x51: case 0x52: case 0x53:
        case 0x55: case 0x56: case 0x57: {
            push(*reg16_table[opcode & 0x07]);
            cpu.ip += 1;
            break;
        }
        // PUSH SP (0x54)：286 压"减 2 之后"的 SP（8086 压减之前的，186+ 起改为新的）
        case 0x54: {
            cpu.sp -= 2;
            write_word(cpu.ss, cpu.sp, cpu.sp);
            cpu.ip += 1;
            break;
        }

        // POP r16
        case 0x58: case 0x59: case 0x5A: case 0x5B:
        case 0x5C: case 0x5D: case 0x5E: case 0x5F: {
            *reg16_table[opcode & 0x07] = pop();
            cpu.ip += 1;
            break;
        }

        // PUSH ES/CS/SS/DS
        case 0x06: push(cpu.es); cpu.ip += 1; break;
        case 0x0E: push(cpu.cs); cpu.ip += 1; break;
        case 0x16: push(cpu.ss); cpu.ip += 1; break;
        case 0x1E: push(cpu.ds); cpu.ip += 1; break;

        // POP ES/SS/DS —— 保护模式必须走 cpu_load_seg_reg 做权限校验并填基址缓存
        case 0x07: { uint16_t s = pop(); cpu_load_seg_reg(SEG_ES, s); cpu.ip += 1; break; }
        case 0x17: { uint16_t s = pop(); cpu_load_seg_reg(SEG_SS, s); cpu.ip += 1; break; }
        case 0x1F: { uint16_t s = pop(); cpu_load_seg_reg(SEG_DS, s); cpu.ip += 1; break; }

        // JMP short
        case 0xEB: cpu.ip += 2 + (int8_t)cpu_mem_read(addr + 1); break;
	// JMP far ptr16:16 (0xEA)
        case 0xEA: {
			uint16_t new_ip = read_word(cpu.cs, cpu.ip + 1);
			uint16_t new_cs = read_word(cpu.cs, cpu.ip + 3);
			if (protect.pe) { protect_far_jmp(new_cs, new_ip); break; }
			cpu.cs = new_cs;
			cpu.ip = new_ip;
			break;
		}

        // JMP near
        case 0xE9: cpu.ip += 3 + (int16_t)read_word(cpu.cs, cpu.ip + 1); break;

        // Jcc short
        case 0x70: cpu.ip += 2 + (get_flag(FLAG_OF) ? (int8_t)cpu_mem_read(addr + 1) : 0); break;
        case 0x71: cpu.ip += 2 + (!get_flag(FLAG_OF) ? (int8_t)cpu_mem_read(addr + 1) : 0); break;
        case 0x72: cpu.ip += 2 + (get_flag(FLAG_CF) ? (int8_t)cpu_mem_read(addr + 1) : 0); break;
        case 0x73: cpu.ip += 2 + (!get_flag(FLAG_CF) ? (int8_t)cpu_mem_read(addr + 1) : 0); break;
        case 0x74: cpu.ip += 2 + (get_flag(FLAG_ZF) ? (int8_t)cpu_mem_read(addr + 1) : 0); break;
        case 0x75: cpu.ip += 2 + (!get_flag(FLAG_ZF) ? (int8_t)cpu_mem_read(addr + 1) : 0); break;
        case 0x76: cpu.ip += 2 + ((get_flag(FLAG_CF) || get_flag(FLAG_ZF)) ? (int8_t)cpu_mem_read(addr + 1) : 0); break;
        case 0x77: cpu.ip += 2 + ((!get_flag(FLAG_CF) && !get_flag(FLAG_ZF)) ? (int8_t)cpu_mem_read(addr + 1) : 0); break;
        case 0x78: cpu.ip += 2 + (get_flag(FLAG_SF) ? (int8_t)cpu_mem_read(addr + 1) : 0); break;
        case 0x79: cpu.ip += 2 + (!get_flag(FLAG_SF) ? (int8_t)cpu_mem_read(addr + 1) : 0); break;
        case 0x7A: cpu.ip += 2 + (get_flag(FLAG_PF) ? (int8_t)cpu_mem_read(addr + 1) : 0); break;
        case 0x7B: cpu.ip += 2 + (!get_flag(FLAG_PF) ? (int8_t)cpu_mem_read(addr + 1) : 0); break;
        case 0x7C: cpu.ip += 2 + ((get_flag(FLAG_SF) != get_flag(FLAG_OF)) ? (int8_t)cpu_mem_read(addr + 1) : 0); break;
        case 0x7D: cpu.ip += 2 + ((get_flag(FLAG_SF) == get_flag(FLAG_OF)) ? (int8_t)cpu_mem_read(addr + 1) : 0); break;
        case 0x7E: cpu.ip += 2 + ((get_flag(FLAG_ZF) || (get_flag(FLAG_SF) != get_flag(FLAG_OF))) ? (int8_t)cpu_mem_read(addr + 1) : 0); break;
        case 0x7F: cpu.ip += 2 + ((!get_flag(FLAG_ZF) && (get_flag(FLAG_SF) == get_flag(FLAG_OF))) ? (int8_t)cpu_mem_read(addr + 1) : 0); break;

        // LOOP/LOOPZ/LOOPNZ/JCXZ
        case 0xE2: cpu.cx--; if (cpu.cx != 0) cpu.ip += 2 + (int8_t)cpu_mem_read(addr + 1); else cpu.ip += 2; break;
        case 0xE1: cpu.cx--; if (cpu.cx != 0 && get_flag(FLAG_ZF)) cpu.ip += 2 + (int8_t)cpu_mem_read(addr + 1); else cpu.ip += 2; break;
        case 0xE0: cpu.cx--; if (cpu.cx != 0 && !get_flag(FLAG_ZF)) cpu.ip += 2 + (int8_t)cpu_mem_read(addr + 1); else cpu.ip += 2; break;
        case 0xE3: if (cpu.cx == 0) cpu.ip += 2 + (int8_t)cpu_mem_read(addr + 1); else cpu.ip += 2; break;

        // CALL near
        case 0xE8: {
			int16_t offset = (int16_t)read_word(cpu.cs, cpu.ip + 1);
			
			push(cpu.ip + 3);
			cpu.ip += 3 + offset;
			break;
		}
	// LES r16, m16:16 (0xC4)
	case 0xC4: {
    		uint8_t modrm = cpu_mem_read(addr + 1);
    		uint8_t reg = (modrm >> 3) & 0x07;
    		uint32_t ea = decode_modrm_addr(modrm, NULL);
   		uint16_t off = cpu_mem_read(ea) | (cpu_mem_read(ea + 1) << 8);
    		uint16_t seg = cpu_mem_read(ea + 2) | (cpu_mem_read(ea + 3) << 8);
    		*reg16_table[reg] = off;
    		cpu_load_seg_reg(SEG_ES, seg);
    		cpu.ip += 2; handle_modrm_ip(modrm);
    		break;
	}

	// LDS r16, m16:16 (0xC5)
	case 0xC5: {
    		uint8_t modrm = cpu_mem_read(addr + 1);
    		uint8_t reg = (modrm >> 3) & 0x07;
    		uint32_t ea = decode_modrm_addr(modrm, NULL);
    		uint16_t off = cpu_mem_read(ea) | (cpu_mem_read(ea + 1) << 8);
    		uint16_t seg = cpu_mem_read(ea + 2) | (cpu_mem_read(ea + 3) << 8);
    		*reg16_table[reg] = off;
    		cpu_load_seg_reg(SEG_DS, seg);
    		cpu.ip += 2; handle_modrm_ip(modrm);
    		break;
	}
        // RET
	case 0xC3: {
		cpu.ip = pop();
		break;
	}
        // RET imm16 —— 先读 imm，再 pop
        case 0xC2: {
            uint16_t imm = read_word(cpu.cs, cpu.ip + 1);
            cpu.ip = pop();
            cpu.sp += imm;
            break;
        }

        // INT3
        case 0xCC: cpu.ip += 1; cpu_interrupt(3); break;

        // INT imm8
        case 0xCD: {
            uint8_t int_num = MEM(addr + 1);
            cpu.ip += 2;
            // INT 13h：硬盘服务由模拟器内置（BIOS 只认软盘）。首次调用时装向量桩，
            // 之后请求经桩 → INT 0F1h 回到 C 代码（见 ide.c）。
            if (int_num == 0x13) ide_int13_prepare();
            // INT 0F1h：桩专用的硬件中断号，直接由 C 处理，不再查 IVT
            if (int_num == 0xF1) { ide_int13_handle(); break; }
            cpu_interrupt(int_num);
            break;
        }

        // INTO
        case 0xCE: {
            cpu.ip += 1;
            if (get_flag(FLAG_OF)) cpu_interrupt(4);
            break;
        }

        // IRET
        case 0xCF:
            if (protect.pe) { protect_far_ret(true, 0); in_fault = 0; break; }
            cpu.ip = pop();
            cpu_load_seg_reg(SEG_CS, pop());
            cpu.flags = (uint16_t)((pop() & 0x7FFF) | 0x0002);
            in_fault = 0;
            break;

        // String ops
        case 0xA4: {
            write_byte(cpu.es, cpu.di, read_byte(get_seg(), cpu.si));
            if (get_flag(FLAG_DF)) { cpu.si--; cpu.di--; } else { cpu.si++; cpu.di++; }
            cpu.ip += 1; break;
        }
        case 0xA5: {
            write_word(cpu.es, cpu.di, read_word(get_seg(), cpu.si));
            if (get_flag(FLAG_DF)) { cpu.si -= 2; cpu.di -= 2; } else { cpu.si += 2; cpu.di += 2; }
            cpu.ip += 1; break;
        }
        case 0xA6: {
            uint8_t a = read_byte(get_seg(), cpu.si), b = read_byte(cpu.es, cpu.di);
            update_arith_flags_8(a - b, a, b, true);
            if (get_flag(FLAG_DF)) { cpu.si--; cpu.di--; } else { cpu.si++; cpu.di++; }
            cpu.ip += 1; break;
        }
        case 0xA7: {
            uint16_t a = read_word(get_seg(), cpu.si), b = read_word(cpu.es, cpu.di);
            update_arith_flags(a - b, a, b, true);
            if (get_flag(FLAG_DF)) { cpu.si -= 2; cpu.di -= 2; } else { cpu.si += 2; cpu.di += 2; }
            cpu.ip += 1; break;
        }
        case 0xAA: {
            (void)get_seg();   // 目标固定 ES:DI，前缀在此消耗掉，不能漏到下一条指令
            write_byte(cpu.es, cpu.di, cpu.ax & 0xFF);
            if (get_flag(FLAG_DF)) cpu.di--; else cpu.di++;
            cpu.ip += 1; break;
        }
        case 0xAB: {
            (void)get_seg();
            write_word(cpu.es, cpu.di, cpu.ax);
            if (get_flag(FLAG_DF)) cpu.di -= 2; else cpu.di += 2;
            cpu.ip += 1; break;
        }
        case 0xAC: {
			uint16_t seg = get_seg();
			cpu.ax = (cpu.ax & 0xFF00) | read_byte(seg, cpu.si);
			if (get_flag(FLAG_DF)) cpu.si--; else cpu.si++;
			cpu.ip += 1; break;
		}
        case 0xAD: {
            cpu.ax = read_word(get_seg(), cpu.si);
            if (get_flag(FLAG_DF)) cpu.si -= 2; else cpu.si += 2;
            cpu.ip += 1; break;
        }
        case 0xAE: {
            (void)get_seg();
            uint8_t a = cpu.ax & 0xFF, b = read_byte(cpu.es, cpu.di);
            update_arith_flags_8(a - b, a, b, true);
            if (get_flag(FLAG_DF)) cpu.di--; else cpu.di++;
            cpu.ip += 1; break;
        }
        case 0xAF: {
            (void)get_seg();
            uint16_t a = cpu.ax, b = read_word(cpu.es, cpu.di);
            update_arith_flags(a - b, a, b, true);
            if (get_flag(FLAG_DF)) cpu.di -= 2; else cpu.di += 2;
            cpu.ip += 1; break;
        }

        // NOP
        case 0x90: cpu.ip += 1; break;

        // XCHG AX, r16
        case 0x91: case 0x92: case 0x93:
        case 0x94: case 0x95: case 0x96: case 0x97: {
            uint16_t tmp = cpu.ax;
            cpu.ax = *reg16_table[opcode & 0x07];
            *reg16_table[opcode & 0x07] = tmp;
            cpu.ip += 1; break;
        }
	// XLAT
        case 0xD7: {
            uint16_t seg = get_seg();
            cpu.ax = (cpu.ax & 0xFF00) |
                     read_byte(seg, (cpu.bx + (cpu.ax & 0xFF)) & 0xFFFF);
            cpu.ip += 1;
            break;
        }
		// x87 FPU ESC (0xD8 ~ 0xDF)
	case 0xD8: case 0xD9: case 0xDA: case 0xDB:
	case 0xDC: case 0xDD: case 0xDE: case 0xDF: {
			uint8_t modrm = cpu_mem_read(addr + 1);
			uint8_t mod = (modrm >> 6) & 0x03;
			uint32_t ea = 0;
			if (mod != 3) {
				ea = decode_modrm_addr(modrm, NULL);
			}
			fpu_escape(opcode, modrm, ea);
			cpu.ip += 2;
			if (mod != 3) {
				handle_modrm_ip(modrm);
			}
			break;
		}
        // CBW
        case 0x98: cpu.ax = (cpu.ax & 0x80) ? (cpu.ax | 0xFF00) : (cpu.ax & 0x00FF); cpu.ip += 1; break;

        // CWD
        case 0x99: cpu.dx = (cpu.ax & 0x8000) ? 0xFFFF : 0x0000; cpu.ip += 1; break;
	
	case 0x9A: {
		uint16_t new_ip = read_word(cpu.cs, cpu.ip + 1);
		uint16_t new_cs = read_word(cpu.cs, cpu.ip + 3);
		if (protect.pe) { protect_far_call(new_cs, new_ip); break; }
		push(cpu.cs);
		push(cpu.ip + 5);
		cpu.cs = new_cs;
		cpu.ip = new_ip;
		break;
	}
        // WAIT
        case 0x9B: cpu.ip += 1; break;

        // RETF
	case 0xCB: {
		if (protect.pe) { protect_far_ret(false, 0); break; }
		cpu.ip = pop();
		cpu_load_seg_reg(SEG_CS, pop());
		break;
	}

        // RETF imm16
        case 0xCA: {
            uint16_t imm = read_word(cpu.cs, cpu.ip + 1);
            if (protect.pe) { protect_far_ret(false, imm); break; }
            cpu.ip = pop();
            cpu_load_seg_reg(SEG_CS, pop());
            cpu.sp += imm;
            break;
        }

        // ENTER
        case 0xC8: {
            uint16_t alloc = read_word(cpu.cs, cpu.ip + 1);
            uint8_t level = MEM(addr + 3) & 0x1F;
            push(cpu.bp);
            uint16_t frame = cpu.sp;
            if (level > 0) {
                for (int i = 1; i < level; i++) {
                    cpu.bp -= 2;
                    push(read_word(cpu.ss, cpu.bp));
                }
                push(frame);
            }
            cpu.bp = frame;
            cpu.sp -= alloc;
            cpu.ip += 4;
            break;
        }

        // LEAVE
        case 0xC9: {
            cpu.sp = cpu.bp;
            cpu.bp = pop();
            cpu.ip += 1;
            break;
        }

        // PUSHF/POPF/SAHF/LAHF
        // 286：PUSHF 压入的 FLAGS bit15 恒为 0；POPF 只能装 bit0~14（bit1 恒 1）。
        //   保护模式下 IOPL 位需要 CPL=0 才能改（此处只在 CPL=0 下运行高特权代码，
        //   DOS/Windows 的 CPL0 段因此可正常设置 IOPL）。
        case 0x9C: push(cpu.flags); cpu.ip += 1; break;
        case 0x9D: cpu.flags = (uint16_t)((pop() & 0x7FFF) | 0x0002); cpu.ip += 1; break;
        // 9E = SAHF：把 AH 装进 SF/ZF/AF/PF/CF（bit1 恒为 1，OF/DF/IF/TF 不动）
        case 0x9E: cpu.flags = (cpu.flags & 0xFF02) | ((cpu.ax >> 8) & 0xD5) | 0x0002; cpu.ip += 1; break;
        // 9F = LAHF：把标志位装进 AH（bit1 恒为 1）
        case 0x9F: cpu.ax = (cpu.ax & 0x00FF) | (((cpu.flags & 0xD5) | 0x02) << 8); cpu.ip += 1; break;

        // MOV AL/AX, moffs
        case 0xA0: cpu.ax = (cpu.ax & 0xFF00) | read_byte(get_seg(), read_word(cpu.cs, cpu.ip + 1)); cpu.ip += 3; break;
        case 0xA1: cpu.ax = read_word(get_seg(), read_word(cpu.cs, cpu.ip + 1)); cpu.ip += 3; break;
        case 0xA2: write_byte(get_seg(), read_word(cpu.cs, cpu.ip + 1), cpu.ax & 0xFF); cpu.ip += 3; break;
        case 0xA3: write_word(get_seg(), read_word(cpu.cs, cpu.ip + 1), cpu.ax); cpu.ip += 3; break;

        // TEST AL/AX, imm
        case 0xA8: {
            uint8_t result = (cpu.ax & 0xFF) & cpu_mem_read(addr + 1);
            update_logic_flags8(result);
            cpu.ip += 2; break;
        }
        case 0xA9: {
            uint16_t result = cpu.ax & read_word(cpu.cs, cpu.ip + 1);
            update_logic_flags16(result);
            cpu.ip += 3; break;
        }

        case 0xF4:
			cpu_halted = true;
			hlt_count++;
			cpu.ip += 1;
			break;

        // INT1 / ICEBP (0xF1)：产生 1 号中断（调试异常）
        case 0xF1:
            cpu.ip += 1;
            cpu_interrupt(1);
            break;

        // CLC/STC/CLI/STI/CLD/STD/CMC
        case 0xF8: clear_flag(FLAG_CF); cpu.ip += 1; break;
        case 0xF9: set_flag(FLAG_CF); cpu.ip += 1; break;
        case 0xFA: clear_flag(FLAG_IF); cpu.ip += 1; break;
        case 0xFB: set_flag(FLAG_IF); cpu.ip += 1; break;
        case 0xFC: clear_flag(FLAG_DF); cpu.ip += 1; break;
        case 0xFD: set_flag(FLAG_DF); cpu.ip += 1; break;
        case 0xF5: if (get_flag(FLAG_CF)) clear_flag(FLAG_CF); else set_flag(FLAG_CF); cpu.ip += 1; break;

        // ============ ModR/M Instructions ============

        // MOV r/m8, r8 (0x88)
        case 0x88: {
            uint8_t modrm = cpu_mem_read(addr + 1);
            write_modrm8(modrm, get_reg8_val((modrm >> 3) & 0x07));
            cpu.ip += 2; handle_modrm_ip(modrm);
            break;
        }

        // MOV r/m16, r16 (0x89)
        case 0x89: {
            uint8_t modrm = cpu_mem_read(addr + 1);
            write_modrm16(modrm, *reg16_table[(modrm >> 3) & 0x07]);
            cpu.ip += 2; handle_modrm_ip(modrm);
            break;
        }

        // MOV r8, r/m8 (0x8A)
        case 0x8A: {
            uint8_t modrm = cpu_mem_read(addr + 1);
            set_reg8_val((modrm >> 3) & 0x07, read_modrm8(modrm));
            cpu.ip += 2; handle_modrm_ip(modrm);
            break;
        }

        // MOV r16, r/m16 (0x8B)
        case 0x8B: {
            uint8_t modrm = cpu_mem_read(addr + 1);
            *reg16_table[(modrm >> 3) & 0x07] = read_modrm16(modrm);
            cpu.ip += 2; handle_modrm_ip(modrm);
            break;
        }

        // MOV r/m16, Sreg (0x8C)
        case 0x8C: {
            uint8_t modrm = cpu_mem_read(addr + 1);
            write_modrm16(modrm, *seg_table[(modrm >> 3) & 0x07]);
            cpu.ip += 2; handle_modrm_ip(modrm);
            break;
        }

        // MOV Sreg, r/m16 (0x8E) —— 保护模式下必须按描述符校验并填基址缓存
        case 0x8E: {
            uint8_t modrm = cpu_mem_read(addr + 1);
            uint16_t sel = read_modrm16(modrm);
            cpu_load_seg_reg(seg_index_tbl[(modrm >> 3) & 0x07], sel);
            cpu.ip += 2; handle_modrm_ip(modrm);
            break;
        }

        // POP r/m16 (0x8F)
        case 0x8F: {
            uint8_t modrm = cpu_mem_read(addr + 1);
            write_modrm16(modrm, pop());
            cpu.ip += 2; handle_modrm_ip(modrm);
            break;
        }

        // PUSH imm16 (0x68)
        case 0x68: {
            push(read_word(cpu.cs, cpu.ip + 1));
            cpu.ip += 3; break;
        }

        // IMUL r16, r/m16, imm16 (0x69)
        case 0x69: {
            uint8_t modrm = cpu_mem_read(addr + 1);
            uint8_t reg = (modrm >> 3) & 0x07;
            int imm_off = 2 + modrm_imm_offset(modrm);
            uint16_t a = read_modrm16(modrm);
            uint16_t imm = read_word(cpu.cs, cpu.ip + imm_off);
            int32_t result = (int16_t)a * (int16_t)imm;
            *reg16_table[reg] = result & 0xFFFF;
            if (result < -32768 || result > 32767) { set_flag(FLAG_CF); set_flag(FLAG_OF); }
            else { clear_flag(FLAG_CF); clear_flag(FLAG_OF); }
            cpu.ip += 2; handle_modrm_ip(modrm);
            cpu.ip += 2;
            break;
        }

        // PUSH imm8 (0x6A)
        case 0x6A: {
            push((int16_t)(int8_t)MEM(addr + 1));
            cpu.ip += 2; break;
        }

        // IMUL r16, r/m16, imm8 (0x6B)
        case 0x6B: {
            uint8_t modrm = cpu_mem_read(addr + 1);
            uint8_t reg = (modrm >> 3) & 0x07;
            int imm_off = 2 + modrm_imm_offset(modrm);
            uint16_t a = read_modrm16(modrm);
            int16_t imm = (int8_t)cpu_mem_read(addr + imm_off);
            int32_t result = (int16_t)a * imm;
            *reg16_table[reg] = result & 0xFFFF;
            if (result < -32768 || result > 32767) { set_flag(FLAG_CF); set_flag(FLAG_OF); }
            else { clear_flag(FLAG_CF); clear_flag(FLAG_OF); }
            cpu.ip += 2; handle_modrm_ip(modrm);
            cpu.ip += 1;
            break;
        }

        // LEA r16, m (0x8D)
        case 0x8D: {
            uint8_t modrm = cpu_mem_read(addr + 1);
            uint8_t rm = modrm & 0x07;
            uint8_t mod = (modrm >> 6) & 0x03;
            uint16_t ea = 0;
            switch (rm) {
                case 0: ea = cpu.bx + cpu.si; break;
                case 1: ea = cpu.bx + cpu.di; break;
                case 2: ea = cpu.bp + cpu.si; break;
                case 3: ea = cpu.bp + cpu.di; break;
                case 4: ea = cpu.si; break;
                case 5: ea = cpu.di; break;
                case 6: ea = (mod == 0) ? read_word(cpu.cs, cpu.ip + 2) : cpu.bp; break;
                case 7: ea = cpu.bx; break;
            }
            if (mod == 1) ea += (int8_t)cpu_mem_read(addr + 2);
            else if (mod == 2) ea += read_word(cpu.cs, cpu.ip + 2);
            *reg16_table[(modrm >> 3) & 0x07] = ea;
            cpu.ip += 2;
            if (mod == 0 && rm == 6) cpu.ip += 2;
            else if (mod == 1) cpu.ip += 1;
            else if (mod == 2) cpu.ip += 2;
            break;
        }

        // XCHG r8, r/m8 (0x86)
        case 0x86: {
            uint8_t modrm = cpu_mem_read(addr + 1);
            uint8_t reg = (modrm >> 3) & 0x07;
            uint8_t a = get_reg8_val(reg);
            uint8_t b = read_modrm8(modrm);
            write_modrm8(modrm, a);
            set_reg8_val(reg, b);
            cpu.ip += 2; handle_modrm_ip(modrm);
            break;
        }

        // XCHG r16, r/m16 (0x87)
        case 0x87: {
            uint8_t modrm = cpu_mem_read(addr + 1);
            uint8_t reg = (modrm >> 3) & 0x07;
            uint16_t a = *reg16_table[reg];
            uint16_t b = read_modrm16(modrm);
            write_modrm16(modrm, a);
            *reg16_table[reg] = b;
            cpu.ip += 2; handle_modrm_ip(modrm);
            break;
        }

        // CMP r/m8, r8 (0x38)
        case 0x38: {
            uint8_t modrm = cpu_mem_read(addr + 1);
            uint8_t a = read_modrm8(modrm);
            uint8_t b = get_reg8_val((modrm >> 3) & 0x07);
            update_arith_flags_8(a - b, a, b, true);
            cpu.ip += 2; handle_modrm_ip(modrm);
            break;
        }

        // CMP r/m16, r16 (0x39)
        case 0x39: {
            uint8_t modrm = cpu_mem_read(addr + 1);
            uint16_t a = read_modrm16(modrm);
            uint16_t b = *reg16_table[(modrm >> 3) & 0x07];
            update_arith_flags(a - b, a, b, true);
            cpu.ip += 2; handle_modrm_ip(modrm);
            break;
        }

        // CMP r8, r/m8 (0x3A)
        case 0x3A: {
            uint8_t modrm = cpu_mem_read(addr + 1);
            uint8_t a = get_reg8_val((modrm >> 3) & 0x07);
            uint8_t b = read_modrm8(modrm);
            update_arith_flags_8(a - b, a, b, true);
            cpu.ip += 2; handle_modrm_ip(modrm);
            break;
        }

        // CMP r16, r/m16 (0x3B)
        case 0x3B: {
            uint8_t modrm = cpu_mem_read(addr + 1);
            uint16_t a = *reg16_table[(modrm >> 3) & 0x07];
            uint16_t b = read_modrm16(modrm);
            update_arith_flags(a - b, a, b, true);
            cpu.ip += 2; handle_modrm_ip(modrm);
            break;
        }

        // CMP AL, imm8
        case 0x3C: {
            uint8_t a = cpu.ax & 0xFF;
            update_arith_flags_8(a - cpu_mem_read(addr + 1), a, cpu_mem_read(addr + 1), true);
            cpu.ip += 2; break;
        }

        // CMP AX, imm16
        case 0x3D: {
            uint16_t imm = read_word(cpu.cs, cpu.ip + 1);
            update_arith_flags(cpu.ax - imm, cpu.ax, imm, true);
            cpu.ip += 3; break;
        }

        // TEST r/m8, r8 (0x84)
        case 0x84: {
            uint8_t modrm = cpu_mem_read(addr + 1);
            uint8_t result = read_modrm8(modrm) & get_reg8_val((modrm >> 3) & 0x07);
            update_logic_flags8(result);
            cpu.ip += 2; handle_modrm_ip(modrm);
            break;
        }

        // TEST r/m16, r16 (0x85)
        case 0x85: {
            uint8_t modrm = cpu_mem_read(addr + 1);
            uint16_t result = read_modrm16(modrm) & *reg16_table[(modrm >> 3) & 0x07];
            update_logic_flags16(result);
            cpu.ip += 2; handle_modrm_ip(modrm);
            break;
        }

        // AND r/m8, r8 (0x20)
        case 0x20: {
            uint8_t modrm = cpu_mem_read(addr + 1);
            uint8_t result = read_modrm8(modrm) & get_reg8_val((modrm >> 3) & 0x07);
            update_logic_flags8(result);
            write_modrm8(modrm, result);
            cpu.ip += 2; handle_modrm_ip(modrm);
            break;
        }

        // AND r/m16, r16 (0x21)
        case 0x21: {
            uint8_t modrm = cpu_mem_read(addr + 1);
            uint16_t result = read_modrm16(modrm) & *reg16_table[(modrm >> 3) & 0x07];
            update_logic_flags16(result);
            write_modrm16(modrm, result);
            cpu.ip += 2; handle_modrm_ip(modrm);
            break;
        }

        // AND r8, r/m8 (0x22)
        case 0x22: {
            uint8_t modrm = cpu_mem_read(addr + 1);
            uint8_t result = get_reg8_val((modrm >> 3) & 0x07) & read_modrm8(modrm);
            update_logic_flags8(result);
            set_reg8_val((modrm >> 3) & 0x07, result);
            cpu.ip += 2; handle_modrm_ip(modrm);
            break;
        }

        // AND r16, r/m16 (0x23)
        case 0x23: {
            uint8_t modrm = cpu_mem_read(addr + 1);
            uint16_t result = *reg16_table[(modrm >> 3) & 0x07] & read_modrm16(modrm);
            update_logic_flags16(result);
            *reg16_table[(modrm >> 3) & 0x07] = result;
            cpu.ip += 2; handle_modrm_ip(modrm);
            break;
        }

        // AND AL, imm8
        case 0x24: {
            uint8_t result = (cpu.ax & 0xFF) & cpu_mem_read(addr + 1);
            update_logic_flags8(result);
            cpu.ax = (cpu.ax & 0xFF00) | result;
            cpu.ip += 2; break;
        }

        // AND AX, imm16
        case 0x25: {
            uint16_t result = cpu.ax & read_word(cpu.cs, cpu.ip + 1);
            update_logic_flags16(result);
            cpu.ax = result;
            cpu.ip += 3; break;
        }

        // OR r/m8, r8 (0x08)
        case 0x08: {
            uint8_t modrm = cpu_mem_read(addr + 1);
            uint8_t result = read_modrm8(modrm) | get_reg8_val((modrm >> 3) & 0x07);
            update_logic_flags8(result);
            write_modrm8(modrm, result);
            cpu.ip += 2; handle_modrm_ip(modrm);
            break;
        }

        // OR r/m16, r16 (0x09)
        case 0x09: {
            uint8_t modrm = cpu_mem_read(addr + 1);
            uint16_t result = read_modrm16(modrm) | *reg16_table[(modrm >> 3) & 0x07];
            update_logic_flags16(result);
            write_modrm16(modrm, result);
            cpu.ip += 2; handle_modrm_ip(modrm);
            break;
        }

        // OR r8, r/m8 (0x0A)
        case 0x0A: {
            uint8_t modrm = cpu_mem_read(addr + 1);
            uint8_t result = get_reg8_val((modrm >> 3) & 0x07) | read_modrm8(modrm);
            update_logic_flags8(result);
            set_reg8_val((modrm >> 3) & 0x07, result);
            cpu.ip += 2; handle_modrm_ip(modrm);
            break;
        }

        // OR r16, r/m16 (0x0B)
        case 0x0B: {
            uint8_t modrm = cpu_mem_read(addr + 1);
            uint16_t result = *reg16_table[(modrm >> 3) & 0x07] | read_modrm16(modrm);
            update_logic_flags16(result);
            *reg16_table[(modrm >> 3) & 0x07] = result;
            cpu.ip += 2; handle_modrm_ip(modrm);
            break;
        }

        // OR AL, imm8
        case 0x0C: {
            uint8_t result = (cpu.ax & 0xFF) | cpu_mem_read(addr + 1);
            update_logic_flags8(result);
            cpu.ax = (cpu.ax & 0xFF00) | result;
            cpu.ip += 2; break;
        }

        // OR AX, imm16
        case 0x0D: {
            uint16_t result = cpu.ax | read_word(cpu.cs, cpu.ip + 1);
            update_logic_flags16(result);
            cpu.ax = result;
            cpu.ip += 3; break;
        }

        // XOR r/m8, r8 (0x30)
        case 0x30: {
            uint8_t modrm = cpu_mem_read(addr + 1);
            uint8_t result = read_modrm8(modrm) ^ get_reg8_val((modrm >> 3) & 0x07);
            update_logic_flags8(result);
            write_modrm8(modrm, result);
            cpu.ip += 2; handle_modrm_ip(modrm);
            break;
        }

        // XOR r/m16, r16 (0x31)
        case 0x31: {
            uint8_t modrm = cpu_mem_read(addr + 1);
            uint16_t result = read_modrm16(modrm) ^ *reg16_table[(modrm >> 3) & 0x07];
            update_logic_flags16(result);
            write_modrm16(modrm, result);
            cpu.ip += 2; handle_modrm_ip(modrm);
            break;
        }

        // XOR r8, r/m8 (0x32)
        case 0x32: {
            uint8_t modrm = cpu_mem_read(addr + 1);
            uint8_t result = get_reg8_val((modrm >> 3) & 0x07) ^ read_modrm8(modrm);
            update_logic_flags8(result);
            set_reg8_val((modrm >> 3) & 0x07, result);
            cpu.ip += 2; handle_modrm_ip(modrm);
            break;
        }

        // XOR r16, r/m16 (0x33)
        case 0x33: {
            uint8_t modrm = cpu_mem_read(addr + 1);
            uint16_t result = *reg16_table[(modrm >> 3) & 0x07] ^ read_modrm16(modrm);
            update_logic_flags16(result);
            *reg16_table[(modrm >> 3) & 0x07] = result;
            cpu.ip += 2; handle_modrm_ip(modrm);
            break;
        }

        // XOR AL, imm8
        case 0x34: {
            uint8_t result = (cpu.ax & 0xFF) ^ cpu_mem_read(addr + 1);
            update_logic_flags8(result);
            cpu.ax = (cpu.ax & 0xFF00) | result;
            cpu.ip += 2; break;
        }

        // XOR AX, imm16
        case 0x35: {
            uint16_t result = cpu.ax ^ read_word(cpu.cs, cpu.ip + 1);
            update_logic_flags16(result);
            cpu.ax = result;
            cpu.ip += 3; break;
        }

        // ADD r/m8, r8 (0x00)
        case 0x00: {
            uint8_t modrm = cpu_mem_read(addr + 1);
            uint8_t a = read_modrm8(modrm);
            uint8_t b = get_reg8_val((modrm >> 3) & 0x07);
            uint8_t result = a + b;
            update_arith_flags_8(result, a, b, false);
            write_modrm8(modrm, result);
            cpu.ip += 2; handle_modrm_ip(modrm);
            break;
        }

        // ADD r/m16, r16 (0x01)
        case 0x01: {
            uint8_t modrm = cpu_mem_read(addr + 1);
            uint16_t a = read_modrm16(modrm);
            uint16_t b = *reg16_table[(modrm >> 3) & 0x07];
            uint16_t result = a + b;
            update_arith_flags(result, a, b, false);
            write_modrm16(modrm, result);
            cpu.ip += 2; handle_modrm_ip(modrm);
            break;
        }

        // ADD r8, r/m8 (0x02)
        case 0x02: {
            uint8_t modrm = cpu_mem_read(addr + 1);
            uint8_t a = get_reg8_val((modrm >> 3) & 0x07);
            uint8_t b = read_modrm8(modrm);
            uint8_t result = a + b;
            update_arith_flags_8(result, a, b, false);
            set_reg8_val((modrm >> 3) & 0x07, result);
            cpu.ip += 2; handle_modrm_ip(modrm);
            break;
        }

        // ADD r16, r/m16 (0x03)
        case 0x03: {
            uint8_t modrm = cpu_mem_read(addr + 1);
            uint16_t a = *reg16_table[(modrm >> 3) & 0x07];
            uint16_t b = read_modrm16(modrm);
            uint16_t result = a + b;
            update_arith_flags(result, a, b, false);
            *reg16_table[(modrm >> 3) & 0x07] = result;
            cpu.ip += 2; handle_modrm_ip(modrm);
            break;
        }

        // SUB r/m8, r8 (0x28)
        case 0x28: {
            uint8_t modrm = cpu_mem_read(addr + 1);
            uint8_t a = read_modrm8(modrm);
            uint8_t b = get_reg8_val((modrm >> 3) & 0x07);
            uint8_t result = a - b;
            update_arith_flags_8(result, a, b, true);
            write_modrm8(modrm, result);
            cpu.ip += 2; handle_modrm_ip(modrm);
            break;
        }

        // SUB r/m16, r16 (0x29)
        case 0x29: {
            uint8_t modrm = cpu_mem_read(addr + 1);
            uint16_t a = read_modrm16(modrm);
            uint16_t b = *reg16_table[(modrm >> 3) & 0x07];
            uint16_t result = a - b;
            update_arith_flags(result, a, b, true);
            write_modrm16(modrm, result);
            cpu.ip += 2; handle_modrm_ip(modrm);
            break;
        }

        // SUB r8, r/m8 (0x2A)
        case 0x2A: {
            uint8_t modrm = cpu_mem_read(addr + 1);
            uint8_t a = get_reg8_val((modrm >> 3) & 0x07);
            uint8_t b = read_modrm8(modrm);
            uint8_t result = a - b;
            update_arith_flags_8(result, a, b, true);
            set_reg8_val((modrm >> 3) & 0x07, result);
            cpu.ip += 2; handle_modrm_ip(modrm);
            break;
        }

        // SUB r16, r/m16 (0x2B)
        case 0x2B: {
            uint8_t modrm = cpu_mem_read(addr + 1);
            uint16_t a = *reg16_table[(modrm >> 3) & 0x07];
            uint16_t b = read_modrm16(modrm);
            uint16_t result = a - b;
            update_arith_flags(result, a, b, true);
            *reg16_table[(modrm >> 3) & 0x07] = result;
            cpu.ip += 2; handle_modrm_ip(modrm);
            break;
        }

        // ADC r/m8, r8 (0x10)
        case 0x10: {
            uint8_t modrm = cpu_mem_read(addr + 1);
            uint8_t a = read_modrm8(modrm);
            uint8_t b = get_reg8_val((modrm >> 3) & 0x07);
            uint8_t c = get_flag(FLAG_CF) ? 1 : 0;
            uint8_t result = a + b + c;
            update_arith_flags_8(result, a, b + c, false);
            write_modrm8(modrm, result);
            cpu.ip += 2; handle_modrm_ip(modrm);
            break;
        }

        // ADC r/m16, r16 (0x11)
        case 0x11: {
            uint8_t modrm = cpu_mem_read(addr + 1);
            uint16_t a = read_modrm16(modrm);
            uint16_t b = *reg16_table[(modrm >> 3) & 0x07];
            uint16_t c = get_flag(FLAG_CF) ? 1 : 0;
            uint16_t result = a + b + c;
            update_arith_flags(result, a, b + c, false);
            write_modrm16(modrm, result);
            cpu.ip += 2; handle_modrm_ip(modrm);
            break;
        }

        // ADC r8, r/m8 (0x12)
        case 0x12: {
            uint8_t modrm = cpu_mem_read(addr + 1);
            uint8_t a = get_reg8_val((modrm >> 3) & 0x07);
            uint8_t b = read_modrm8(modrm);
            uint8_t c = get_flag(FLAG_CF) ? 1 : 0;
            uint8_t result = a + b + c;
            update_arith_flags_8(result, a, b + c, false);
            set_reg8_val((modrm >> 3) & 0x07, result);
            cpu.ip += 2; handle_modrm_ip(modrm);
            break;
        }

        // ADC r16, r/m16 (0x13)
        case 0x13: {
            uint8_t modrm = cpu_mem_read(addr + 1);
            uint16_t a = *reg16_table[(modrm >> 3) & 0x07];
            uint16_t b = read_modrm16(modrm);
            uint16_t c = get_flag(FLAG_CF) ? 1 : 0;
            uint16_t result = a + b + c;
            update_arith_flags(result, a, b + c, false);
            *reg16_table[(modrm >> 3) & 0x07] = result;
            cpu.ip += 2; handle_modrm_ip(modrm);
            break;
        }

        // SBB r/m8, r8 (0x18)
        case 0x18: {
            uint8_t modrm = cpu_mem_read(addr + 1);
            uint8_t a = read_modrm8(modrm);
            uint8_t b = get_reg8_val((modrm >> 3) & 0x07);
            uint8_t c = get_flag(FLAG_CF) ? 1 : 0;
            uint8_t result = a - b - c;
            update_arith_flags_8(result, a, b + c, true);
            write_modrm8(modrm, result);
            cpu.ip += 2; handle_modrm_ip(modrm);
            break;
        }

        // SBB r/m16, r16 (0x19)
        case 0x19: {
            uint8_t modrm = cpu_mem_read(addr + 1);
            uint16_t a = read_modrm16(modrm);
            uint16_t b = *reg16_table[(modrm >> 3) & 0x07];
            uint16_t c = get_flag(FLAG_CF) ? 1 : 0;
            uint16_t result = a - b - c;
            update_arith_flags(result, a, b + c, true);
            write_modrm16(modrm, result);
            cpu.ip += 2; handle_modrm_ip(modrm);
            break;
        }

        // SBB r8, r/m8 (0x1A)
        case 0x1A: {
            uint8_t modrm = cpu_mem_read(addr + 1);
            uint8_t a = get_reg8_val((modrm >> 3) & 0x07);
            uint8_t b = read_modrm8(modrm);
            uint8_t c = get_flag(FLAG_CF) ? 1 : 0;
            uint8_t result = a - b - c;
            update_arith_flags_8(result, a, b + c, true);
            set_reg8_val((modrm >> 3) & 0x07, result);
            cpu.ip += 2; handle_modrm_ip(modrm);
            break;
        }

        // SBB r16, r/m16 (0x1B)
        case 0x1B: {
            uint8_t modrm = cpu_mem_read(addr + 1);
            uint16_t a = *reg16_table[(modrm >> 3) & 0x07];
            uint16_t b = read_modrm16(modrm);
            uint16_t c = get_flag(FLAG_CF) ? 1 : 0;
            uint16_t result = a - b - c;
            update_arith_flags(result, a, b + c, true);
            *reg16_table[(modrm >> 3) & 0x07] = result;
            cpu.ip += 2; handle_modrm_ip(modrm);
            break;
        }

        // ADC AL, imm8
        case 0x14: {
            uint8_t a = cpu.ax & 0xFF;
            uint8_t b = cpu_mem_read(addr + 1);
            uint8_t c = get_flag(FLAG_CF) ? 1 : 0;
            uint8_t result = a + b + c;
            update_arith_flags_8(result, a, b + c, false);
            cpu.ax = (cpu.ax & 0xFF00) | result;
            cpu.ip += 2; break;
        }

        // ADC AX, imm16
        case 0x15: {
            uint16_t imm = read_word(cpu.cs, cpu.ip + 1);
            uint16_t c = get_flag(FLAG_CF) ? 1 : 0;
            uint16_t result = cpu.ax + imm + c;
            update_arith_flags(result, cpu.ax, imm + c, false);
            cpu.ax = result;
            cpu.ip += 3; break;
        }

        // SBB AL, imm8
        case 0x1C: {
            uint8_t a = cpu.ax & 0xFF;
            uint8_t b = cpu_mem_read(addr + 1);
            uint8_t c = get_flag(FLAG_CF) ? 1 : 0;
            uint8_t result = a - b - c;
            update_arith_flags_8(result, a, b + c, true);
            cpu.ax = (cpu.ax & 0xFF00) | result;
            cpu.ip += 2; break;
        }

        // SBB AX, imm16
        case 0x1D: {
            uint16_t imm = read_word(cpu.cs, cpu.ip + 1);
            uint16_t c = get_flag(FLAG_CF) ? 1 : 0;
            uint16_t result = cpu.ax - imm - c;
            update_arith_flags(result, cpu.ax, imm + c, true);
            cpu.ax = result;
            cpu.ip += 3; break;
        }

        // ============ GRP Instructions ============

        	// GRP1 r/m8, imm8 (0x80 / 0x82 别名)
	case 0x80: case 0x82: {
           uint8_t modrm = MEM(addr + 1);
           uint8_t op = (modrm >> 3) & 0x07;
           uint8_t imm = MEM(addr + 2 + modrm_imm_offset(modrm));
		uint8_t a = read_modrm8(modrm);
		uint8_t result = 0;
		switch (op) {
			case 0: result = a + imm; update_arith_flags_8(result, a, imm, false); break;
			case 1: result = a | imm; update_logic_flags8(result); break;
			case 2: result = a + imm + (get_flag(FLAG_CF) ? 1 : 0); update_arith_flags_8(result, a, imm + (get_flag(FLAG_CF) ? 1 : 0), false); break;
			case 3: result = a - imm - (get_flag(FLAG_CF) ? 1 : 0); update_arith_flags_8(result, a, imm + (get_flag(FLAG_CF) ? 1 : 0), true); break;
			case 4: result = a & imm; update_logic_flags8(result); break;
			case 5: result = a - imm; update_arith_flags_8(result, a, imm, true); break;
			case 6: result = a ^ imm; update_logic_flags8(result); break;
			case 7: update_arith_flags_8(a - imm, a, imm, true); break;
		}
		if (op != 7) write_modrm8(modrm, result);
		cpu.ip += 3; handle_modrm_ip(modrm);
		break;
	}

        	// GRP1 r/m16, imm16 (0x81)
	case 0x81: {
		uint8_t modrm = cpu_mem_read(addr + 1);
		uint8_t op = (modrm >> 3) & 0x07;
		uint16_t imm = read_word(cpu.cs, cpu.ip + 2 + modrm_imm_offset(modrm));
		uint16_t a = read_modrm16(modrm);
		uint16_t result = 0;
		switch (op) {
			case 0: result = a + imm; update_arith_flags(result, a, imm, false); break;
			case 1: result = a | imm; update_logic_flags16(result); break;
			case 2: result = a + imm + (get_flag(FLAG_CF) ? 1 : 0); update_arith_flags(result, a, imm + (get_flag(FLAG_CF) ? 1 : 0), false); break;
			case 3: result = a - imm - (get_flag(FLAG_CF) ? 1 : 0); update_arith_flags(result, a, imm + (get_flag(FLAG_CF) ? 1 : 0), true); break;
			case 4: result = a & imm; update_logic_flags16(result); break;
			case 5: result = a - imm; update_arith_flags(result, a, imm, true); break;
			case 6: result = a ^ imm; update_logic_flags16(result); break;
			case 7: update_arith_flags(a - imm, a, imm, true); break;
		}
		if (op != 7) write_modrm16(modrm, result);
		cpu.ip += 4; handle_modrm_ip(modrm);
		break;
	}

        	// GRP1 r/m16, imm8 (0x83)
	case 0x83: {
		uint8_t modrm = MEM(addr + 1);
		uint8_t op = (modrm >> 3) & 0x07;
		int16_t imm = (int8_t)MEM(addr + 2 + modrm_imm_offset(modrm));
		uint16_t a = read_modrm16(modrm);
		uint16_t result = 0;
		switch (op) {
			case 0: result = a + imm; update_arith_flags(result, a, imm, false); break;
			case 1: result = a | imm; update_logic_flags16(result); break;
			case 2: result = a + imm + (get_flag(FLAG_CF) ? 1 : 0); update_arith_flags(result, a, imm + (get_flag(FLAG_CF) ? 1 : 0), false); break;
			case 3: result = a - imm - (get_flag(FLAG_CF) ? 1 : 0); update_arith_flags(result, a, imm + (get_flag(FLAG_CF) ? 1 : 0), true); break;
			case 4: result = a & imm; update_logic_flags16(result); break;
			case 5: result = a - imm; update_arith_flags(result, a, imm, true); break;
			case 6: result = a ^ imm; update_logic_flags16(result); break;
			case 7: update_arith_flags(a - imm, a, imm, true); break;
		}
		if (op != 7) write_modrm16(modrm, result);
		cpu.ip += 3; handle_modrm_ip(modrm);
		break;
	}

        // MOV r/m8, imm8 (0xC6)
case 0xC6: {
  uint8_t modrm = MEM(addr + 1);
  uint8_t mod = (modrm >> 6) & 0x03;
  uint8_t rm = modrm & 0x07;
  int disp_len = 0;
  if (mod == 0 && rm == 6) disp_len = 2;
  else if (mod == 1) disp_len = 1;
  else if (mod == 2) disp_len = 2;
  uint8_t imm = MEM(addr + 2 + disp_len);
  write_modrm8(modrm, imm);
  cpu.ip += 2 + disp_len + 1;
  break;
}

        // MOV r/m16, imm16 (0xC7)
        case 0xC7: {
            uint8_t modrm = cpu_mem_read(addr + 1);
            write_modrm16(modrm, read_word(cpu.cs, cpu.ip + 2 + modrm_imm_offset(modrm)));
            cpu.ip += 4; handle_modrm_ip(modrm);
            break;
        }

        // GRP2 r/m8, 1 (0xD0)
        case 0xD0: {
            uint8_t modrm = cpu_mem_read(addr + 1);
            uint8_t op = (modrm >> 3) & 0x07;
            uint8_t a = read_modrm8(modrm);
            uint8_t result = a;
            bool new_cf = false;
            switch (op) {
                case 0: new_cf = (a & 0x80) != 0; result = (a << 1) | (new_cf ? 1 : 0); break;
                case 1: new_cf = (a & 0x01) != 0; result = (a >> 1) | (new_cf ? 0x80 : 0); break;
                case 2: new_cf = (a & 0x80) != 0; result = (a << 1) | (get_flag(FLAG_CF) ? 1 : 0); break;
                case 3: new_cf = (a & 0x01) != 0; result = (a >> 1) | (get_flag(FLAG_CF) ? 0x80 : 0); break;
                case 4: case 6: new_cf = (a & 0x80) != 0; result = a << 1; break;
                case 5: new_cf = (a & 0x01) != 0; result = a >> 1; break;
                case 7: new_cf = (a & 0x01) != 0; result = (a >> 1) | (a & 0x80); break;
            }
            write_modrm8(modrm, result);
            update_logic_flags8(result);
            if (new_cf) set_flag(FLAG_CF); else clear_flag(FLAG_CF);
            // SHL/SAL：OF = 结果最高位 XOR CF（最高位发生了变化就置 1）
            if (op == 4 || op == 6) {
                if (((result & 0x80) != 0) != new_cf) set_flag(FLAG_OF); else clear_flag(FLAG_OF);
            }
            cpu.ip += 2; handle_modrm_ip(modrm);
            break;
        }

        // GRP2 r/m16, 1 (0xD1)
        case 0xD1: {
            uint8_t modrm = cpu_mem_read(addr + 1);
            uint8_t op = (modrm >> 3) & 0x07;
            uint16_t a = read_modrm16(modrm);
            uint16_t result = a;
            bool new_cf = false;
            switch (op) {
                case 0: new_cf = (a & 0x8000) != 0; result = (a << 1) | (new_cf ? 1 : 0); break;
                case 1: new_cf = (a & 0x0001) != 0; result = (a >> 1) | (new_cf ? 0x8000 : 0); break;
                case 2: new_cf = (a & 0x8000) != 0; result = (a << 1) | (get_flag(FLAG_CF) ? 1 : 0); break;
                case 3: new_cf = (a & 0x0001) != 0; result = (a >> 1) | (get_flag(FLAG_CF) ? 0x8000 : 0); break;
                case 4: case 6: new_cf = (a & 0x8000) != 0; result = a << 1; break;
                case 5: new_cf = (a & 0x0001) != 0; result = a >> 1; break;
                case 7: new_cf = (a & 0x0001) != 0; result = (a >> 1) | (a & 0x8000); break;
            }
            write_modrm16(modrm, result);
            update_logic_flags16(result);
            if (new_cf) set_flag(FLAG_CF); else clear_flag(FLAG_CF);
            // SHL/SAL：OF = 结果最高位 XOR CF
            if (op == 4 || op == 6) {
                if (((result & 0x8000) != 0) != new_cf) set_flag(FLAG_OF); else clear_flag(FLAG_OF);
            }
            cpu.ip += 2; handle_modrm_ip(modrm);
            break;
        }

        // GRP2 r/m8, CL (0xD2)
        case 0xD2: {
            uint8_t modrm = cpu_mem_read(addr + 1);
            uint8_t op = (modrm >> 3) & 0x07;
            uint8_t a = read_modrm8(modrm);
            uint8_t count = cpu.cx & 0x1F;   // 286：移位计数掩到 5 位
            uint8_t result = a;
            bool new_cf = false;
            if (count == 0) { cpu.ip += 2; handle_modrm_ip(modrm); break; }
            for (int i = 0; i < count; i++) {
                switch (op) {
                    case 0: new_cf = (result & 0x80) != 0; result = (result << 1) | (new_cf ? 1 : 0); break;
                    case 1: new_cf = (result & 0x01) != 0; result = (result >> 1) | (new_cf ? 0x80 : 0); break;
                    case 2: { bool cf = get_flag(FLAG_CF); new_cf = (result & 0x80) != 0; result = (result << 1) | (cf ? 1 : 0); break; }
                    case 3: { bool cf = get_flag(FLAG_CF); new_cf = (result & 0x01) != 0; result = (result >> 1) | (cf ? 0x80 : 0); break; }
                    case 4: case 6: new_cf = (result & 0x80) != 0; result = result << 1; break;
                    case 5: new_cf = (result & 0x01) != 0; result = result >> 1; break;
                    case 7: new_cf = (result & 0x01) != 0; result = (result >> 1) | (result & 0x80); break;
                }
                if (new_cf) set_flag(FLAG_CF); else clear_flag(FLAG_CF);
            }
            write_modrm8(modrm, result);
            update_logic_flags8(result);
            if (new_cf) set_flag(FLAG_CF); else clear_flag(FLAG_CF);
            // SHL/SAL：OF = 结果最高位 XOR CF
            if (op == 4 || op == 6) {
                if (((result & 0x80) != 0) != new_cf) set_flag(FLAG_OF); else clear_flag(FLAG_OF);
            }
            cpu.ip += 2; handle_modrm_ip(modrm);
            break;
        }

        // GRP2 r/m16, CL (0xD3)
        case 0xD3: {
            uint8_t modrm = cpu_mem_read(addr + 1);
            uint8_t op = (modrm >> 3) & 0x07;
            uint16_t a = read_modrm16(modrm);
            uint8_t count = cpu.cx & 0x1F;   // 286：移位计数掩到 5 位
            uint16_t result = a;
            bool new_cf = false;
            if (count == 0) { cpu.ip += 2; handle_modrm_ip(modrm); break; }
            for (int i = 0; i < count; i++) {
                switch (op) {
                    case 0: new_cf = (result & 0x8000) != 0; result = (result << 1) | (new_cf ? 1 : 0); break;
                    case 1: new_cf = (result & 0x0001) != 0; result = (result >> 1) | (new_cf ? 0x8000 : 0); break;
                    case 2: { bool cf = get_flag(FLAG_CF); new_cf = (result & 0x8000) != 0; result = (result << 1) | (cf ? 1 : 0); break; }
                    case 3: { bool cf = get_flag(FLAG_CF); new_cf = (result & 0x0001) != 0; result = (result >> 1) | (cf ? 0x8000 : 0); break; }
                    case 4: case 6: new_cf = (result & 0x8000) != 0; result = result << 1; break;
                    case 5: new_cf = (result & 0x0001) != 0; result = result >> 1; break;
                    case 7: new_cf = (result & 0x0001) != 0; result = (result >> 1) | (result & 0x8000); break;
                }
                if (new_cf) set_flag(FLAG_CF); else clear_flag(FLAG_CF);
            }
            write_modrm16(modrm, result);
            update_logic_flags16(result);
            if (new_cf) set_flag(FLAG_CF); else clear_flag(FLAG_CF);
            // SHL/SAL：OF = 结果最高位 XOR CF
            if (op == 4 || op == 6) {
                if (((result & 0x8000) != 0) != new_cf) set_flag(FLAG_OF); else clear_flag(FLAG_OF);
            }
            cpu.ip += 2; handle_modrm_ip(modrm);
            break;
        }

        // AAM (0xD4)
        case 0xD4: {
            uint8_t base = cpu_mem_read(addr + 1);
            if (base == 0) { cpu.ip += 2; cpu_exception(0, "AAM 基数为 0（等同于除零）"); break; }
            uint8_t al = cpu.ax & 0xFF;
            cpu.ax = ((al / base) << 8) | (al % base);
            update_logic_flags8(cpu.ax & 0xFF);
            cpu.ip += 2; break;
        }

        // AAD (0xD5)
        case 0xD5: {
            uint8_t base = cpu_mem_read(addr + 1);
            uint8_t al = cpu.ax & 0xFF;
            uint8_t ah = (cpu.ax >> 8) & 0xFF;
            uint8_t result = (ah * base + al) & 0xFF;
            cpu.ax = result;
            update_logic_flags8(result);
            cpu.ip += 2; break;
        }

        // SALC / SETALC (0xD6，未公开指令)：AL = CF ? 0xFF : 0x00
        case 0xD6: {
            cpu.ax = (cpu.ax & 0xFF00) | (get_flag(FLAG_CF) ? 0xFF : 0x00);
            cpu.ip += 1; break;
        }

        // GRP3 r/m8 (0xF6)
        case 0xF6: {
            uint8_t modrm = cpu_mem_read(addr + 1);
            uint8_t op = (modrm >> 3) & 0x07;
            uint8_t a = read_modrm8(modrm);
            cpu.ip += 2; handle_modrm_ip(modrm);
            switch (op) {
                case 0: {
                    uint8_t imm = read_byte(cpu.cs, cpu.ip);   // ★ 立即数在 CS:IP，不是物理地址
                    uint8_t result = a & imm;
                    update_logic_flags8(result);
                    cpu.ip += 1; break;
                }
                case 2: write_modrm8(modrm, ~a); break;
                case 3: {
                    uint8_t result = -a;
                    update_arith_flags_8(result, 0, a, true);
                    write_modrm8(modrm, result); break;
                }
                case 4: {
                    uint16_t result = (cpu.ax & 0xFF) * a;
                    cpu.ax = result; break;
                }
                case 5: {
                    int16_t result = (int8_t)(cpu.ax & 0xFF) * (int8_t)a;
                    cpu.ax = result; break;
                }
                case 6: {
                    // DIV r/m8：AX / r/m8 → 商 AL，余数 AH
                    uint16_t al = cpu.ax;
                    if (a == 0) { cpu_exception(0, "DIV r/m8 除数为 0"); break; }
                    uint16_t q = al / a;
                    if (q > 0xFF) { cpu_exception(0, "DIV r/m8 商溢出（>0xFF）"); break; }
                    cpu.ax = ((al % a) << 8) | (q & 0xFF); break;
                }
                case 7: {
                    // IDIV r/m8：AX / r/m8（有符号）→ 商 AL，余数 AH
                    int16_t al = (int16_t)cpu.ax;
                    if (a == 0) { cpu_exception(0, "IDIV r/m8 除数为 0"); break; }
                    int16_t q = al / (int8_t)a;
                    if (q > 127 || q < -128) { cpu_exception(0, "IDIV r/m8 商溢出"); break; }
                    cpu.ax = (((al % (int8_t)a) & 0xFF) << 8) | (q & 0xFF); break;
                }
            }
            break;
        }

        // GRP3 r/m16 (0xF7)
        case 0xF7: {
            uint8_t modrm = cpu_mem_read(addr + 1);
            uint8_t op = (modrm >> 3) & 0x07;
            uint16_t a = read_modrm16(modrm);
            cpu.ip += 2; handle_modrm_ip(modrm);
            switch (op) {
                case 0: {
                    uint16_t imm = read_word(cpu.cs, cpu.ip);
                    uint16_t result = a & imm;
                    update_logic_flags16(result);
                    cpu.ip += 2; break;
                }
                case 2: write_modrm16(modrm, ~a); break;
                case 3: {
                    uint16_t result = -a;
                    update_arith_flags(result, 0, a, true);
                    write_modrm16(modrm, result); break;
                }
                case 4: {
                    uint32_t result = cpu.ax * a;
                    cpu.ax = result & 0xFFFF;
                    cpu.dx = result >> 16; break;
                }
                case 5: {
                    int32_t result = (int16_t)cpu.ax * (int16_t)a;
                    cpu.ax = result & 0xFFFF;
                    cpu.dx = (result >> 16) & 0xFFFF; break;
                }
                case 6: {
                    // DIV r/m16：DX:AX / r/m16 → 商 AX，余数 DX
                    uint32_t dividend = ((uint32_t)cpu.dx << 16) | cpu.ax;
                    if (a == 0) { cpu_exception(0, "DIV r/m16 除数为 0"); break; }
                    uint32_t q = dividend / a;
                    if (q > 0xFFFF) { cpu_exception(0, "DIV r/m16 商溢出（>0xFFFF）"); break; }
                    cpu.ax = q & 0xFFFF;
                    cpu.dx = dividend % a; break;
                }
                case 7: {
                    // IDIV r/m16：DX:AX / r/m16（有符号）→ 商 AX，余数 DX
                    int32_t dividend = (int32_t)(((uint32_t)cpu.dx << 16) | cpu.ax);
                    if (a == 0) { cpu_exception(0, "IDIV r/m16 除数为 0"); break; }
                    int32_t q = dividend / (int16_t)a;
                    if (q > 32767 || q < -32768) { cpu_exception(0, "IDIV r/m16 商溢出"); break; }
                    cpu.ax = q & 0xFFFF;
                    cpu.dx = (dividend % (int16_t)a) & 0xFFFF; break;
                }
            }
            break;
        }

        		// GRP4 r/m8 (0xFE) - INC/DEC
		case 0xFE: {
			uint8_t modrm = cpu_mem_read(addr + 1);
			uint8_t op = (modrm >> 3) & 0x07;
			uint8_t a = read_modrm8(modrm);
			bool cf = get_flag(FLAG_CF);
			if (op == 0) {
				uint8_t result = a + 1;
				update_arith_flags_8(result, a, 1, false);
				if (cf) set_flag(FLAG_CF); else clear_flag(FLAG_CF);
				write_modrm8(modrm, result);
			} else if (op == 1) {
				uint8_t result = a - 1;
				update_arith_flags_8(result, a, 1, true);
				if (cf) set_flag(FLAG_CF); else clear_flag(FLAG_CF);
				write_modrm8(modrm, result);
			}
			cpu.ip += 2; handle_modrm_ip(modrm);
			break;
		}

        // GRP5 r/m16 (0xFF) - INC/DEC/CALL/JMP/PUSH
        case 0xFF: {
            uint8_t modrm = cpu_mem_read(addr + 1);
            uint8_t op = (modrm >> 3) & 0x07;
            uint8_t gmod = (modrm >> 6) & 0x03;
            uint8_t grm = modrm & 0x07;
            // ★ 顺序很重要：
            //   1) decode_modrm_addr 会"消费"段覆盖前缀，一条指令里只能调用一次，
            //      所以必须先算 ea，再从 ea 取值（不能再调 read_modrm16）。
            //   2) 必须在推进 IP 之前算，因为 decode_modrm_addr 用 cpu.ip+2 取 disp16。
            uint32_t ea = (gmod == 3) ? 0 : decode_modrm_addr(modrm, NULL);
            uint16_t a = (gmod == 3) ? *reg16_table[grm]
                                     : (cpu_mem_read(ea) | (cpu_mem_read(ea + 1) << 8));
            cpu.ip += 2; handle_modrm_ip(modrm);
            switch (op) {
                case 0: {
                    uint16_t result = a + 1;
                    bool cf = get_flag(FLAG_CF);
                    update_arith_flags(result, a, 1, false);
                    if (cf) set_flag(FLAG_CF); else clear_flag(FLAG_CF);
                    if (gmod == 3) *reg16_table[grm] = result;
                    else write_word_at(ea, result);
                    break;
                }
                case 1: {
                    uint16_t result = a - 1;
                    bool cf = get_flag(FLAG_CF);
                    update_arith_flags(result, a, 1, true);
                    if (cf) set_flag(FLAG_CF); else clear_flag(FLAG_CF);
                    if (gmod == 3) *reg16_table[grm] = result;
                    else write_word_at(ea, result);
                    break;
                }
                case 2:  // CALL near r/m16（/2，段内间接调用，不换 CS）
                    push(cpu.ip);
                    cpu.ip = a;
                    break;
                case 3: {  // CALL far
                    uint16_t off, seg;
                    if (gmod == 3) {
                        off = *reg16_table[grm];
                        seg = cpu.cs;
                    } else {
                        off = cpu_mem_read(ea) | (cpu_mem_read(ea + 1) << 8);
                        seg = cpu_mem_read(ea + 2) | (cpu_mem_read(ea + 3) << 8);
                    }
                    push(cpu.cs);
                    push(cpu.ip);
                    cpu.cs = seg;
                    cpu.ip = off;
                    break;
                }
                case 4: cpu.ip = a; break;          // JMP near
                case 5: {  // JMP far
                    uint16_t off, seg;
                    if (gmod == 3) { off = *reg16_table[grm]; seg = cpu.cs; }
                    else {
                        off = cpu_mem_read(ea) | (cpu_mem_read(ea + 1) << 8);
                        seg = cpu_mem_read(ea + 2) | (cpu_mem_read(ea + 3) << 8);
                    }
                    cpu.cs = seg;
                    cpu.ip = off;
                    break;
                }
                case 6: push(a); break;             // PUSH r/m16
            }
            break;
        }

        // GRP2 r/m8, imm8 (0xC0)
        case 0xC0: {
            uint8_t modrm = cpu_mem_read(addr + 1);
            uint8_t op = (modrm >> 3) & 0x07;
            uint8_t imm = cpu_mem_read(addr + 2 + modrm_imm_offset(modrm)) & 0x1F;
            uint8_t a = read_modrm8(modrm);
            uint8_t result = a;
            bool new_cf = false;
            cpu.ip += 3;
            handle_modrm_ip(modrm);
            if (imm == 0) break;
            for (int i = 0; i < imm; i++) {
                switch (op) {
                    case 0: new_cf = (result & 0x80) != 0; result = (result << 1) | (new_cf ? 1 : 0); break;
                    case 1: new_cf = (result & 0x01) != 0; result = (result >> 1) | (new_cf ? 0x80 : 0); break;
                    case 2: { bool cf = get_flag(FLAG_CF); new_cf = (result & 0x80) != 0; result = (result << 1) | (cf ? 1 : 0); break; }
                    case 3: { bool cf = get_flag(FLAG_CF); new_cf = (result & 0x01) != 0; result = (result >> 1) | (cf ? 0x80 : 0); break; }
                    case 4: case 6: new_cf = (result & 0x80) != 0; result = result << 1; break;
                    case 5: new_cf = (result & 0x01) != 0; result = result >> 1; break;
                    case 7: new_cf = (result & 0x01) != 0; result = (result >> 1) | (result & 0x80); break;
                }
                if (new_cf) set_flag(FLAG_CF); else clear_flag(FLAG_CF);
            }
            write_modrm8(modrm, result);
            update_logic_flags8(result);
            if (new_cf) set_flag(FLAG_CF); else clear_flag(FLAG_CF);
            break;
        }

        // GRP2 r/m16, imm8 (0xC1)
        case 0xC1: {
            uint8_t modrm = cpu_mem_read(addr + 1);
            uint8_t op = (modrm >> 3) & 0x07;
            uint8_t imm = cpu_mem_read(addr + 2 + modrm_imm_offset(modrm)) & 0x1F;
            uint16_t a = read_modrm16(modrm);
            uint16_t result = a;
            bool new_cf = false;
            cpu.ip += 3;
            handle_modrm_ip(modrm);
            if (imm == 0) break;
            for (int i = 0; i < imm; i++) {
                switch (op) {
                    case 0: new_cf = (result & 0x8000) != 0; result = (result << 1) | (new_cf ? 1 : 0); break;
                    case 1: new_cf = (result & 0x0001) != 0; result = (result >> 1) | (new_cf ? 0x8000 : 0); break;
                    case 2: { bool cf = get_flag(FLAG_CF); new_cf = (result & 0x8000) != 0; result = (result << 1) | (cf ? 1 : 0); break; }
                    case 3: { bool cf = get_flag(FLAG_CF); new_cf = (result & 0x0001) != 0; result = (result >> 1) | (cf ? 0x8000 : 0); break; }
                    case 4: case 6: new_cf = (result & 0x8000) != 0; result = result << 1; break;
                    case 5: new_cf = (result & 0x0001) != 0; result = result >> 1; break;
                    case 7: new_cf = (result & 0x0001) != 0; result = (result >> 1) | (result & 0x8000); break;
                }
                if (new_cf) set_flag(FLAG_CF); else clear_flag(FLAG_CF);
            }
            write_modrm16(modrm, result);
            update_logic_flags16(result);
            if (new_cf) set_flag(FLAG_CF); else clear_flag(FLAG_CF);
            break;
        }

        // IN AL, imm8 (0xE4)
        case 0xE4: {
            uint8_t port = MEM(addr + 1);
            cpu.ax = (cpu.ax & 0xFF00) | io_read_port(port);
            cpu.ip += 2;
            break;
        }

        // IN AX, imm8 (0xE5)
        case 0xE5: {
            uint8_t port = MEM(addr + 1);
            cpu.ax = io_read_port16(port);
            cpu.ip += 2;
            break;
        }

        // OUT imm8, AL (0xE6)
        case 0xE6: {
            uint8_t port = MEM(addr + 1);
            io_write_port(port, cpu.ax & 0xFF);
            cpu.ip += 2;
            break;
        }

        // OUT imm8, AX (0xE7)
        case 0xE7: {
            uint8_t port = MEM(addr + 1);
            io_write_port16(port, cpu.ax);
            cpu.ip += 2;
            break;
        }

        // IN AL, DX (0xEC)
        case 0xEC: {
            cpu.ax = (cpu.ax & 0xFF00) | io_read_port(cpu.dx);
            cpu.ip++;
            break;
        }

        // IN AX, DX (0xED)
        case 0xED: {
            cpu.ax = io_read_port16(cpu.dx);
            cpu.ip++;
            break;
        }

        // OUT DX, AL (0xEE)
        case 0xEE: {
            io_write_port(cpu.dx, cpu.ax & 0xFF);
            cpu.ip++;
            break;
        }

        // OUT DX, AX (0xEF)
        case 0xEF: {
            io_write_port16(cpu.dx, cpu.ax);
            cpu.ip++;
            break;
        }

        case 0x0F: {
			cpu386_execute_0f();
			break;
		}

        default:
			cpu386_execute_opcode(opcode);
			break;
    }

    // 段覆盖前缀只对紧随其后的一条指令有效：若该指令未访问内存，
    // 这里必须清掉，否则会泄漏到后续指令造成错误的段寻址。
    seg_override = 0;
    seg_override_active = false;

    // 数据访问越限/越权已投递 #GP(13)：译码随后推进的 cpu.ip 会覆盖处理程序入口，
    // 这里恢复成异常处理程序的 CS:IP（见 seg_access_check 的说明）。
    bool faulted = seg_fault_abort;
    if (faulted) {
        seg_fault_abort = false;
        cpu.cs = seg_fault_cs;
        cpu.ip = seg_fault_ip;
    }

    // 单步陷阱：TF 置位时，每条指令执行完都要产生 INT 1（调试异常）。
    // cpu_interrupt 投递时会清 TF 并压入带 TF 的 FLAGS，IRET 恢复后继续单步。
    if (cpu_running && !faulted && get_flag(FLAG_TF)) {
        cpu_exception(1, "单步陷阱 (single step)");
    }
}

// ============================================================
// 异常 / 故障报告（除零、无效指令、通用保护、双重/三重故障）
// ============================================================
static const char* exc_name(uint8_t v) {
	switch (v) {
		case 0:  return "除零错误 (divide error)";
		case 1:  return "单步调试 (debug)";
		case 2:  return "非屏蔽中断 (NMI)";
		case 3:  return "断点 (breakpoint)";
		case 4:  return "溢出 (overflow)";
		case 5:  return "边界越界 (bound range)";
		case 6:  return "无效操作码 (invalid opcode)";
		case 7:  return "设备不可用 (device not available)";
		case 8:  return "双重故障 (double fault)";
		case 10: return "无效 TSS";
		case 11: return "段不存在 (segment not present)";
		case 12: return "栈段错误 (stack fault)";
		case 13: return "通用保护 (general protection)";
		case 14: return "缺页 (page fault)";
		case 16: return "浮点错误 (FPU error)";
		case 17: return "对齐检查 (alignment check)";
		default: return "未知异常";
	}
}

// 打印故障现场：类型/原因、CS:IP、通用与段寄存器、出错指令附近字节、栈顶
void cpu_fault_dump(const char* kind, uint8_t vector, const char* why) {
	fprintf(stderr, "\n[FAULT] ========== %s ==========\n", kind);
	fprintf(stderr, "[FAULT] 类型：%s (向量 %02Xh)", exc_name(vector), vector);
	if (why && why[0]) fprintf(stderr, "  原因：%s", why);
	fprintf(stderr, "\n");
	fprintf(stderr, "[FAULT] CS:IP=%04X:%04X  SS:SP=%04X:%04X  DS=%04X ES=%04X\n",
	        cpu.cs, cpu.ip, cpu.ss, cpu.sp, cpu.ds, cpu.es);
	fprintf(stderr, "[FAULT] AX=%04X BX=%04X CX=%04X DX=%04X SI=%04X DI=%04X BP=%04X FL=%04X\n",
	        cpu.ax, cpu.bx, cpu.cx, cpu.dx, cpu.si, cpu.di, cpu.bp, cpu.flags);
	// 出错指令附近字节：F6/F7 等已把 IP 推进到下一条，故从 IP-6 起打 8 字节，
	// 并把 IP 所指字节用方括号标出
	// ★ 诊断读必须绕开保护模式限长检查（seg_lin）：栈 dump 会读到偏移 0xFFFF，
	//   若走限长检查会再触发一次 #GP，形成"报告故障时又出故障"的递归。
	{
		uint16_t start = (uint16_t)(cpu.ip - 6);
		fprintf(stderr, "[FAULT] 代码 @%04X:%04X:", cpu.cs, start);
		for (int i = 0; i < 8; i++) {
			uint8_t b = cpu_mem_read(cpu_seg_base(cpu.cs) + (uint16_t)(start + i));
			if (i == 6) fprintf(stderr, " [%02X]", b);
			else        fprintf(stderr, " %02X", b);
		}
		fprintf(stderr, "\n");
	}
	// 栈顶 8 个字
	fprintf(stderr, "[FAULT] 栈 @%04X:%04X:", cpu.ss, cpu.sp);
	for (int i = 0; i < 8; i++) {
		uint32_t a = cpu_seg_base(cpu.ss) + (uint16_t)(cpu.sp + i * 2);
		fprintf(stderr, " %04X", cpu_mem_read(a) | (cpu_mem_read(a + 1) << 8));
	}
	fprintf(stderr, "\n");
	fflush(stderr);
}

// 投递异常：按实模式 IVT 走 8086 逻辑。
// 若异常处理程序内部再出错（in_fault 重入）或向量为空 → 双重故障；
// 双重故障也投不出去 → 三重故障 → 停机。
void cpu_exception(uint8_t vector, const char* why) {
	// 保护模式下由 protect.c 按 IDT 门投递（含错误码、栈切换、TSS 任务门）
	if (protect.pe) {
		protect_exception(vector, 0);
		return;
	}
	if (in_fault) {
		cpu_fault_dump("双重故障 (double fault)", 8, why);
		uint16_t o = read_word(0, 8 * 4), s = read_word(0, 8 * 4 + 2);
		if ((o | s) == 0) {
			fprintf(stderr, "[FAULT] 双重故障向量 08h 也为空 → 三重故障 (triple fault) → 停机\n");
		} else {
			fprintf(stderr, "[FAULT] 异常处理中再次出错 → 双重故障 → 停机\n");
		}
		fflush(stderr);
		in_fault = 0;
		cpu_running = false;
		return;
	}

	if (debug_mode) cpu_fault_dump("CPU 异常", vector, why);

	uint32_t va = (uint32_t)vector * 4;
	uint16_t o = read_word(0, va), s = read_word(0, va + 2);
	if ((o | s) == 0) {
		fprintf(stderr, "[FAULT] 向量 %02Xh 为空，异常无法投递 → 双重故障 → 停机\n", vector);
		fflush(stderr);
		cpu_running = false;
		return;
	}

	in_fault = 1;
	cpu_interrupt(vector);
	// 处理程序返回到此之前保持 in_fault=1；IRET 会清零。
}

// 无效操作码：8086 无法继续解码 → 报告现场并停机
void cpu_invalid_opcode(uint8_t op) {
	char why[48];
	snprintf(why, sizeof(why), "操作码 %02Xh", op);
	cpu_fault_dump("无效操作码 (invalid opcode)", 6, why);
	fprintf(stderr, "[FAULT] 无效指令无法继续执行 → 停机\n");
	fflush(stderr);
	in_fault = 0;
	cpu_running = false;
}

void cpu_interrupt(uint8_t int_num) {
	cpu_halted = false;
	// ★ 时序：硬件/软中断本身约 51 周期（压栈 + 查向量表 + 跳转），
	//   不记的话中断密集时虚拟时间会偏慢
	cpu_last_cycles = 51;
	cpu_cycles += cpu_last_cycles;
	// 保护模式：按 IDT 门投递（含 DPL 检查、栈切换、任务门）
	if (protect.pe) { protect_interrupt(int_num, false, 0, false); return; }
	push(cpu.flags);
	push(cpu.cs);
	push(cpu.ip);
	// ★ 8086：INT n / 硬件中断 压栈后必须清 IF（压入的 FLAGS 保留原值，IRET 恢复）。
	//   不清的话中断处理程序在 IF=1 下运行，处理程序内部还会被再次投递中断，
	//   导致压栈帧嵌套、BIOS 键盘缓冲区/DOS 临界区被破坏 → 随机"输入卡死"、
	//   "Data error reading drive A"/"Invalid drive in search path" 等怪现象。
	clear_flag(FLAG_IF);

	uint32_t vector_addr = int_num * 4;
	uint16_t new_ip = read_word(0, vector_addr);
	uint16_t new_cs = read_word(0, vector_addr + 2);

	if (new_cs == 0 && new_ip == 0) {
		if (in_fault) {
			fprintf(stderr,
			        "[FAULT] 异常投递失败：向量 %02Xh 为空（%s）→ 三重故障 (triple fault) → 停机\n",
			        int_num, exc_name(int_num));
		} else {
			fprintf(stderr,
			        "[FAULT] 向量 %02Xh 为空（%s）@ CS:%04X IP:%04X → 无法投递 → 停机\n",
			        int_num, exc_name(int_num), cpu.cs, (uint16_t)(cpu.ip - 1));
		}
		cpu.ip = pop();
		cpu.cs = pop();
		cpu.flags = pop();
		in_fault = 0;
		cpu_running = false;
		return;
	}

	cpu.ip = new_ip;
	cpu.cs = new_cs;
	clear_flag(FLAG_IF);
	clear_flag(FLAG_TF);
}