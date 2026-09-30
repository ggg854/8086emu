// fpu.c - x87 FPU 模拟（8087 / 80287 / 80387 指令集）
//
// 覆盖 D8~DF 全部 ModR/M 形式：
//   - 内存形式：m32real / m64real / m80real / m32int / m16int / m64int / m80bcd
//   - 寄存器形式：算术、比较、传送、常量、超越函数、控制/状态/环境存取
#include "fpu.h"
#include "cpu.h"
#include <stdio.h>
#include <string.h>
#include <math.h>

FPU fpu;

// ---- 状态字位定义 ----
#define SW_IE  0x0001   // 无效操作
#define SW_DE  0x0002   // 未规格化操作数
#define SW_ZE  0x0004   // 除零
#define SW_OE  0x0008   // 溢出
#define SW_UE  0x0010   // 下溢
#define SW_PE  0x0020   // 精度
#define SW_SF  0x0040   // 栈错误
#define SW_ES  0x0080   // 错误汇总
#define SW_C0  0x0100   // 条件码 C0
#define SW_C1  0x0200   // 条件码 C1
#define SW_C2  0x0400   // 条件码 C2
#define SW_TOP 0x3800   // 栈顶指针 TOP（位 11~13）
#define SW_C3  0x4000   // 条件码 C3
#define SW_B   0x8000   // 忙标志

#define DEFAULT_CW 0x037F   // 扩展精度 / 就近舍入 / 屏蔽全部异常

// 内存访问（1MB 掩码，与 cpu.c 保持一致）
#define MEMB(a) memory[(uint32_t)(a) & 0xFFFFF]

// ============================================================
// 初始化 / 复位
// ============================================================
void fpu_init(void) {
    fpu_reset();
}

void fpu_reset(void) {
    memset(&fpu, 0, sizeof(fpu));
    fpu.cw = DEFAULT_CW;
    fpu.sw = 0;
    fpu.tw = 0xFFFF;   // 全部标记为空
    fpu.top = 0;
}

// ============================================================
// 栈操作
// ============================================================
static int st_index(int n) {
    return (fpu.top + n) & 7;
}

static void fpu_set_top(int t) {
    fpu.top = t & 7;
    fpu.sw = (uint16_t)((fpu.sw & ~SW_TOP) | ((fpu.top & 7) << 11));
}

static void fpu_push(long double v) {
    fpu_set_top((fpu.top - 1) & 7);
    fpu.st[fpu.top] = v;
    fpu.tw &= (uint16_t)~(1 << fpu.top);
}

static long double fpu_pop(void) {
    long double v = fpu.st[fpu.top];
    fpu.tw |= (uint16_t)(1 << fpu.top);
    fpu_set_top((fpu.top + 1) & 7);
    return v;
}

static long double fpu_st0(void) {
    return fpu.st[fpu.top];
}

static void fpu_set_st0(long double v) {
    fpu.st[fpu.top] = v;
}

static void fpu_set_cc(int c3, int c2, int c0) {
    fpu.sw &= (uint16_t)~(SW_C3 | SW_C2 | SW_C0);
    if (c3) fpu.sw |= SW_C3;
    if (c2) fpu.sw |= SW_C2;
    if (c0) fpu.sw |= SW_C0;
}

// FCOM / FUCOM / FCOMPP 等：设置 C3/C2/C0
static void fpu_compare(long double a, long double b) {
    if (isnan(a) || isnan(b)) {
        fpu_set_cc(1, 1, 1);          // 无序
    } else if (a > b) {
        fpu_set_cc(0, 0, 0);
    } else if (a < b) {
        fpu_set_cc(0, 0, 1);
    } else {
        fpu_set_cc(1, 0, 0);          // 相等
    }
}

// FCOMI / FUCOMI：设置 EFLAGS 的 ZF/PF/CF
static void fpu_comi_flags(long double a, long double b) {
    if (isnan(a) || isnan(b)) {
        set_flag(FLAG_ZF); set_flag(FLAG_PF); set_flag(FLAG_CF);
    } else if (a > b) {
        clear_flag(FLAG_ZF); clear_flag(FLAG_PF); clear_flag(FLAG_CF);
    } else if (a < b) {
        clear_flag(FLAG_ZF); clear_flag(FLAG_PF); set_flag(FLAG_CF);
    } else {
        set_flag(FLAG_ZF); clear_flag(FLAG_PF); clear_flag(FLAG_CF);
    }
    clear_flag(FLAG_OF); clear_flag(FLAG_SF); clear_flag(FLAG_AF);
}

// ============================================================
// 整数转换（按当前舍入方式取整，越界返回饱和值）
// ============================================================
static int16_t fpu_to_i16(long double v) {
    if (isnan(v)) return (int16_t)0x8000;
    long double r = rintl(v);
    if (r >= 32767.0L) return (int16_t)0x7FFF;
    if (r <= -32768.0L) return (int16_t)0x8000;
    return (int16_t)r;
}

static int32_t fpu_to_i32(long double v) {
    if (isnan(v)) return (int32_t)0x80000000;
    long double r = rintl(v);
    if (r >= 2147483647.0L) return (int32_t)0x7FFFFFFF;
    if (r <= -2147483648.0L) return (int32_t)0x80000000;
    return (int32_t)r;
}

static int64_t fpu_to_i64(long double v) {
    if (isnan(v)) return (int64_t)0x8000000000000000LL;
    long double r = rintl(v);
    if (r >= 9223372036854775807.0L) return (int64_t)0x7FFFFFFFFFFFFFFFLL;
    if (r <= -9223372036854775808.0L) return (int64_t)0x8000000000000000LL;
    return (int64_t)r;
}

// ============================================================
// 内存操作数读取 / 写入
// ============================================================
static float read_mem_f32(uint32_t addr) {
    uint32_t bits = 0;
    for (int i = 0; i < 4; i++) bits |= (uint32_t)MEMB(addr + i) << (i * 8);
    float f;
    memcpy(&f, &bits, 4);
    return f;
}

static double read_mem_f64(uint32_t addr) {
    uint64_t bits = 0;
    for (int i = 0; i < 8; i++) bits |= (uint64_t)MEMB(addr + i) << (i * 8);
    double d;
    memcpy(&d, &bits, 8);
    return d;
}

// 80 位扩展精度：64 位尾数 + 16 位符号/指数
static long double read_mem_f80(uint32_t addr) {
    uint64_t m = 0;
    for (int i = 0; i < 8; i++) m |= (uint64_t)MEMB(addr + i) << (i * 8);
    uint16_t se = (uint16_t)(MEMB(addr + 8) | (MEMB(addr + 9) << 8));
    int sign = (se >> 15) & 1;
    int e = se & 0x7FFF;

    long double v;
    if (e == 0 && m == 0) {
        v = 0.0L;
    } else if (e == 0x7FFF) {
        v = (m & 0x7FFFFFFFFFFFFFFFULL) ? (long double)NAN : (long double)INFINITY;
    } else if (e == 0) {
        v = ldexpl((long double)m, -16382 - 63);        // 非规格化数
    } else {
        v = ldexpl((long double)m, e - 16383 - 63);
    }
    return sign ? -v : v;
}

static void write_mem_f80(uint32_t addr, long double v) {
    uint64_t m = 0;
    uint16_t se = 0;

    if (isnan(v)) {
        se = 0x7FFF;
        m = 0xC000000000000000ULL;
    } else if (isinf(v)) {
        se = 0x7FFF;
        m = 0x8000000000000000ULL;
    } else if (v != 0.0L) {
        int e = 0;
        long double fr = frexpl(fabsl(v), &e);          // v = fr * 2^e, fr ∈ [0.5,1)
        m = (uint64_t)ldexpl(fr, 64);                   // 显式整数位在 bit63
        int E = e + 16382;
        if (E <= 0) { m = 0; se = 0; }                  // 下溢 → 0（简化）
        else if (E >= 0x7FFF) { se = 0x7FFF; m = 0x8000000000000000ULL; }
        else se = (uint16_t)E;
    }
    if (signbit(v)) se |= 0x8000;

    for (int i = 0; i < 8; i++) MEMB(addr + i) = (uint8_t)(m >> (i * 8));
    MEMB(addr + 8) = (uint8_t)(se & 0xFF);
    MEMB(addr + 9) = (uint8_t)(se >> 8);
}

static int16_t read_mem_i16(uint32_t addr) {
    return (int16_t)(MEMB(addr) | (MEMB(addr + 1) << 8));
}

static int32_t read_mem_i32(uint32_t addr) {
    uint32_t v = 0;
    for (int i = 0; i < 4; i++) v |= (uint32_t)MEMB(addr + i) << (i * 8);
    return (int32_t)v;
}

static int64_t read_mem_i64(uint32_t addr) {
    uint64_t v = 0;
    for (int i = 0; i < 8; i++) v |= (uint64_t)MEMB(addr + i) << (i * 8);
    return (int64_t)v;
}

static void write_mem_f32(uint32_t addr, long double v) {
    float f = (float)v;
    uint32_t bits;
    memcpy(&bits, &f, 4);
    for (int i = 0; i < 4; i++) MEMB(addr + i) = (uint8_t)(bits >> (i * 8));
}

static void write_mem_f64(uint32_t addr, long double v) {
    double d = (double)v;
    uint64_t bits;
    memcpy(&bits, &d, 8);
    for (int i = 0; i < 8; i++) MEMB(addr + i) = (uint8_t)(bits >> (i * 8));
}

static void write_mem_i16(uint32_t addr, int16_t v) {
    MEMB(addr) = (uint8_t)(v & 0xFF);
    MEMB(addr + 1) = (uint8_t)((v >> 8) & 0xFF);
}

static void write_mem_i32(uint32_t addr, int32_t v) {
    uint32_t u = (uint32_t)v;
    for (int i = 0; i < 4; i++) MEMB(addr + i) = (uint8_t)(u >> (i * 8));
}

static void write_mem_i64(uint32_t addr, int64_t v) {
    uint64_t u = (uint64_t)v;
    for (int i = 0; i < 8; i++) MEMB(addr + i) = (uint8_t)(u >> (i * 8));
}

// 80 位压缩 BCD（18 位十进制 + 符号）
static long double read_mem_bcd(uint32_t addr) {
    long double v = 0.0L;
    for (int i = 9; i >= 0; i--) {
        uint8_t b = MEMB(addr + i);
        uint8_t hi = (uint8_t)((b >> 4) & 0x0F);
        uint8_t lo = (uint8_t)(b & 0x0F);
        if (i == 9) v = (long double)lo;                     // 最高位数字
        else v = v * 100.0L + (long double)(hi * 10 + lo);
    }
    uint8_t sgn = (uint8_t)((MEMB(addr + 9) >> 4) & 0x0F);
    if (sgn == 0x0D || sgn == 0x0B || sgn == 0x0F) v = -v;
    return v;
}

static void write_mem_bcd(uint32_t addr, long double v) {
    bool neg = signbit(v);
    long double x = floorl(fabsl(v) + 0.5L);
    uint8_t buf[10];
    memset(buf, 0, sizeof(buf));
    for (int i = 0; i < 9; i++) {
        int d1 = (int)fmodl(x, 10.0L); x = floorl(x / 10.0L);
        int d2 = (int)fmodl(x, 10.0L); x = floorl(x / 10.0L);
        buf[i] = (uint8_t)((d2 << 4) | d1);
    }
    int d = (int)fmodl(x, 10.0L);
    buf[9] = (uint8_t)(((neg ? 0x0D : 0x00) << 4) | d);
    for (int i = 0; i < 10; i++) MEMB(addr + i) = buf[i];
}

// ============================================================
// 环境 / 状态存取（实模式 14 字节环境，94 字节状态）
// ============================================================
static void fpu_store_env(uint32_t addr) {
    write_mem_i16(addr + 0, (int16_t)fpu.cw);
    write_mem_i16(addr + 2, (int16_t)fpu.sw);
    write_mem_i16(addr + 4, (int16_t)fpu.tw);
    write_mem_i16(addr + 6, 0);    // 指令指针
    write_mem_i16(addr + 8, 0);    // CS
    write_mem_i16(addr + 10, 0);   // 操作数指针
    write_mem_i16(addr + 12, 0);   // DS
}

static void fpu_load_env(uint32_t addr) {
    fpu.cw = (uint16_t)read_mem_i16(addr + 0);
    fpu.sw = (uint16_t)read_mem_i16(addr + 2);
    fpu.tw = (uint16_t)read_mem_i16(addr + 4);
    fpu.top = (fpu.sw >> 11) & 7;
}

static void fpu_fnsave(uint32_t addr) {
    fpu_store_env(addr);
    for (int i = 0; i < 8; i++) {
        write_mem_f80(addr + 14 + (uint32_t)i * 10, fpu.st[st_index(i)]);
    }
    fpu_reset();   // FNSAVE 执行后 FPU 复位
}

static void fpu_frstor(uint32_t addr) {
    fpu_load_env(addr);
    for (int i = 0; i < 8; i++) {
        fpu.st[st_index(i)] = read_mem_f80(addr + 14 + (uint32_t)i * 10);
    }
}

// ============================================================
// 超越函数
// ============================================================
static void fpu_f2xm1(void) {
    fpu_set_st0(powl(2.0L, fpu_st0()) - 1.0L);
}

static void fpu_fyl2x(void) {
    long double x = fpu_st0();
    long double y = fpu.st[st_index(1)];
    fpu_pop();
    fpu_set_st0(y * log2l(x));
}

static void fpu_fyl2xp1(void) {
    long double x = fpu_st0();
    long double y = fpu.st[st_index(1)];
    fpu_pop();
    fpu_set_st0(y * log2l(x + 1.0L));
}

static void fpu_fptan(void) {
    long double x = fpu_st0();
    fpu_set_st0(tanl(x));
    fpu_push(1.0L);
}

static void fpu_fpatan(void) {
    long double x = fpu_st0();                 // ST(0)
    long double y = fpu.st[st_index(1)];       // ST(1)
    fpu_pop();
    fpu_set_st0(atan2l(y, x));
}

static void fpu_fxtract(void) {
    long double x = fpu_st0();
    if (x == 0.0L) {
        fpu_set_st0(0.0L);
        fpu_push(-INFINITY);
        return;
    }
    int e = 0;
    long double fr = frexpl(x, &e);            // x = fr * 2^e, fr ∈ [0.5,1)
    fpu_set_st0((long double)e);               // 指数
    fpu_push(fr * 2.0L);                       // 尾数 ∈ [1,2)
}

static void fpu_fprem(bool ieee) {
    long double a = fpu_st0();
    long double b = fpu.st[st_index(1)];
    if (b == 0.0L || isinf(a) || isnan(a) || isnan(b)) {
        fpu.sw |= SW_IE | SW_ES;
        return;
    }
    long double q = ieee ? nearbyintl(a / b) : truncl(a / b);
    long double r = a - q * b;
    fpu_set_st0(r);

    long long qi = (long long)q;
    fpu.sw &= (uint16_t)~(SW_C0 | SW_C1 | SW_C2 | SW_C3);
    if (qi & 1) fpu.sw |= SW_C0;
    if (qi & 2) fpu.sw |= SW_C3;
    if (qi & 4) fpu.sw |= SW_C1;
    // C2 = 0：约减已完成
}

static void fpu_fscale(void) {
    long double x = fpu_st0();
    long double n = truncl(fpu.st[st_index(1)]);
    if (n > 32767.0L) n = 32767.0L;
    if (n < -32768.0L) n = -32768.0L;
    fpu_set_st0(ldexpl(x, (int)n));
}

static void fpu_fsin(void) {
    fpu_set_st0(sinl(fpu_st0()));
}

static void fpu_fcos(void) {
    fpu_set_st0(cosl(fpu_st0()));
}

static void fpu_fsincos(void) {
    long double x = fpu_st0();
    fpu_set_st0(sinl(x));
    fpu_push(cosl(x));
}

static void fpu_frndint(void) {
    fpu_set_st0(rintl(fpu_st0()));
}

static void fpu_fsqrt(void) {
    long double x = fpu_st0();
    if (x < 0.0L) { fpu.sw |= SW_IE | SW_ES; return; }
    fpu_set_st0(sqrtl(x));
}

static void fpu_ftst(void) {
    fpu_compare(fpu_st0(), 0.0L);
}

static void fpu_fxam(void) {
    long double x = fpu_st0();
    int empty = (fpu.tw & (1 << fpu.top)) != 0;
    fpu.sw &= (uint16_t)~(SW_C0 | SW_C2 | SW_C3);
    if (empty) {
        fpu.sw |= SW_C3 | SW_C0;                       // 空
    } else if (isnan(x)) {
        fpu.sw |= SW_C0;                               // NaN
    } else if (isinf(x)) {
        fpu.sw |= SW_C2 | SW_C0;                       // 无穷
    } else if (x == 0.0L) {
        fpu.sw |= SW_C3;                               // 零
    } else {
        fpu.sw |= SW_C2;                               // 规格化有限数
    }
    if (signbit(x)) fpu.sw |= SW_C1;                   // 符号
    else fpu.sw &= (uint16_t)~SW_C1;
}

// ============================================================
// FCMOV 条件判断
// ============================================================
static bool fcmov_cond(int reg) {
    switch (reg) {
        case 0: return get_flag(FLAG_CF);                       // FCMOVB
        case 1: return get_flag(FLAG_ZF);                       // FCMOVE
        case 2: return get_flag(FLAG_CF) || get_flag(FLAG_ZF);  // FCMOVBE
        case 3: return get_flag(FLAG_PF);                       // FCMOVU
        case 4: return !get_flag(FLAG_CF);                      // FCMOVNB
        case 5: return !get_flag(FLAG_ZF);                      // FCMOVNE
        case 6: return !get_flag(FLAG_CF) && !get_flag(FLAG_ZF); // FCMOVNBE
        case 7: return !get_flag(FLAG_PF);                      // FCMOVNU
    }
    return false;
}

// ============================================================
// mod != 3：内存操作数形式
// ============================================================
static void fpu_mem_form(uint8_t op, int reg, uint32_t addr) {
    switch (op) {
    case 0xD8:  // 单精度实数
        switch (reg) {
            case 0: fpu_set_st0(fpu_st0() + read_mem_f32(addr)); break;              // FADD
            case 1: fpu_set_st0(fpu_st0() * read_mem_f32(addr)); break;              // FMUL
            case 2: fpu_compare(fpu_st0(), read_mem_f32(addr)); break;               // FCOM
            case 3: fpu_compare(fpu_st0(), read_mem_f32(addr)); fpu_pop(); break;    // FCOMP
            case 4: fpu_set_st0(fpu_st0() - read_mem_f32(addr)); break;              // FSUB
            case 5: fpu_set_st0(read_mem_f32(addr) - fpu_st0()); break;              // FSUBR
            case 6: fpu_set_st0(fpu_st0() / read_mem_f32(addr)); break;              // FDIV
            case 7: fpu_set_st0(read_mem_f32(addr) / fpu_st0()); break;              // FDIVR
        }
        break;

    case 0xD9:
        switch (reg) {
            case 0: fpu_push(read_mem_f32(addr)); break;                             // FLD m32real
            case 1: fpu_push(read_mem_f32(addr)); break;                             // (未公开)
            case 2: write_mem_f32(addr, fpu_st0()); break;                           // FST m32real
            case 3: write_mem_f32(addr, fpu_st0()); fpu_pop(); break;                // FSTP m32real
            case 4: fpu_load_env(addr); break;                                       // FLDENV
            case 5: fpu.cw = (uint16_t)read_mem_i16(addr); break;                     // FLDCW
            case 6: fpu_store_env(addr); break;                                      // FNSTENV
            case 7: write_mem_i16(addr, (int16_t)fpu.cw); break;                      // FNSTCW
        }
        break;

    case 0xDA:  // 32 位整数
        switch (reg) {
            case 0: fpu_set_st0(fpu_st0() + (long double)read_mem_i32(addr)); break;            // FIADD
            case 1: fpu_set_st0(fpu_st0() * (long double)read_mem_i32(addr)); break;            // FIMUL
            case 2: fpu_compare(fpu_st0(), (long double)read_mem_i32(addr)); break;             // FICOM
            case 3: fpu_compare(fpu_st0(), (long double)read_mem_i32(addr)); fpu_pop(); break;  // FICOMP
            case 4: fpu_set_st0(fpu_st0() - (long double)read_mem_i32(addr)); break;            // FISUB
            case 5: fpu_set_st0((long double)read_mem_i32(addr) - fpu_st0()); break;            // FISUBR
            case 6: fpu_set_st0(fpu_st0() / (long double)read_mem_i32(addr)); break;            // FIDIV
            case 7: fpu_set_st0((long double)read_mem_i32(addr) / fpu_st0()); break;            // FIDIVR
        }
        break;

    case 0xDB:
        switch (reg) {
            case 0: fpu_push((long double)read_mem_i32(addr)); break;                // FILD m32int
            case 1: fpu_push((long double)read_mem_i32(addr)); break;                // FISTTP m32int
            case 2: write_mem_i32(addr, fpu_to_i32(fpu_st0())); break;               // FIST m32int
            case 3: write_mem_i32(addr, fpu_to_i32(fpu_st0())); fpu_pop(); break;    // FISTP m32int
            case 5: fpu_push(read_mem_f80(addr)); break;                             // FLD m80real
            case 7: write_mem_f80(addr, fpu_st0()); fpu_pop(); break;                // FSTP m80real
            default: break;
        }
        break;

    case 0xDC:  // 双精度实数
        switch (reg) {
            case 0: fpu_set_st0(fpu_st0() + read_mem_f64(addr)); break;              // FADD
            case 1: fpu_set_st0(fpu_st0() * read_mem_f64(addr)); break;              // FMUL
            case 2: fpu_compare(fpu_st0(), read_mem_f64(addr)); break;               // FCOM
            case 3: fpu_compare(fpu_st0(), read_mem_f64(addr)); fpu_pop(); break;    // FCOMP
            case 4: fpu_set_st0(fpu_st0() - read_mem_f64(addr)); break;              // FSUB
            case 5: fpu_set_st0(read_mem_f64(addr) - fpu_st0()); break;              // FSUBR
            case 6: fpu_set_st0(fpu_st0() / read_mem_f64(addr)); break;              // FDIV
            case 7: fpu_set_st0(read_mem_f64(addr) / fpu_st0()); break;              // FDIVR
        }
        break;

    case 0xDD:
        switch (reg) {
            case 0: fpu_push(read_mem_f64(addr)); break;                             // FLD m64real
            case 1: fpu_push(read_mem_f64(addr)); break;                             // FISTTP m64int
            case 2: write_mem_f64(addr, fpu_st0()); break;                           // FST m64real
            case 3: write_mem_f64(addr, fpu_st0()); fpu_pop(); break;                // FSTP m64real
            case 4: fpu_frstor(addr); break;                                         // FRSTOR
            case 6: fpu_fnsave(addr); break;                                         // FNSAVE
            case 7: write_mem_i16(addr, (int16_t)fpu.sw); break;                      // FNSTSW m16
            default: break;
        }
        break;

    case 0xDE:  // 16 位整数
        switch (reg) {
            case 0: fpu_set_st0(fpu_st0() + (long double)read_mem_i16(addr)); break;            // FIADD
            case 1: fpu_set_st0(fpu_st0() * (long double)read_mem_i16(addr)); break;            // FIMUL
            case 2: fpu_compare(fpu_st0(), (long double)read_mem_i16(addr)); break;             // FICOM
            case 3: fpu_compare(fpu_st0(), (long double)read_mem_i16(addr)); fpu_pop(); break;  // FICOMP
            case 4: fpu_set_st0(fpu_st0() - (long double)read_mem_i16(addr)); break;            // FISUB
            case 5: fpu_set_st0((long double)read_mem_i16(addr) - fpu_st0()); break;            // FISUBR
            case 6: fpu_set_st0(fpu_st0() / (long double)read_mem_i16(addr)); break;            // FIDIV
            case 7: fpu_set_st0((long double)read_mem_i16(addr) / fpu_st0()); break;            // FIDIVR
        }
        break;

    case 0xDF:
        switch (reg) {
            case 0: fpu_push((long double)read_mem_i16(addr)); break;                // FILD m16int
            case 1: fpu_push((long double)read_mem_i16(addr)); break;                // FISTTP m16int
            case 2: write_mem_i16(addr, fpu_to_i16(fpu_st0())); break;               // FIST m16int
            case 3: write_mem_i16(addr, fpu_to_i16(fpu_st0())); fpu_pop(); break;    // FISTP m16int
            case 4: fpu_push(read_mem_bcd(addr)); break;                             // FBLD m80bcd
            case 5: fpu_push((long double)read_mem_i64(addr)); break;                // FILD m64int
            case 6: write_mem_bcd(addr, fpu_st0()); fpu_pop(); break;                // FBSTP m80bcd
            case 7: write_mem_i64(addr, fpu_to_i64(fpu_st0())); fpu_pop(); break;    // FISTP m64int
        }
        break;
    }
}

// ============================================================
// mod == 3：寄存器形式
// ============================================================
static void fpu_reg_form(uint8_t op, uint8_t modrm, int reg, int i) {
    int idx = st_index(i);

    switch (op) {
    case 0xD8:   // ST(0) op= ST(i)
        switch (reg) {
            case 0: fpu_set_st0(fpu_st0() + fpu.st[idx]); break;
            case 1: fpu_set_st0(fpu_st0() * fpu.st[idx]); break;
            case 2: fpu_compare(fpu_st0(), fpu.st[idx]); break;
            case 3: fpu_compare(fpu_st0(), fpu.st[idx]); fpu_pop(); break;
            case 4: fpu_set_st0(fpu_st0() - fpu.st[idx]); break;
            case 5: fpu_set_st0(fpu.st[idx] - fpu_st0()); break;
            case 6: fpu_set_st0(fpu_st0() / fpu.st[idx]); break;
            case 7: fpu_set_st0(fpu.st[idx] / fpu_st0()); break;
        }
        break;

    case 0xD9:
        switch (reg) {
            case 0: fpu_push(fpu.st[idx]); break;                    // FLD ST(i)
            case 1: {                                                // FXCH ST(i)
                long double t = fpu_st0();
                fpu_set_st0(fpu.st[idx]);
                fpu.st[idx] = t;
                break;
            }
            case 2: fpu.st[idx] = fpu_st0(); break;                  // FST ST(i)
            case 3: fpu.st[idx] = fpu_st0(); fpu_pop(); break;       // FSTP ST(i)
            case 4:
                switch (i) {
                    case 0: fpu_set_st0(-fpu_st0()); break;          // FCHS
                    case 1: fpu_set_st0(fabsl(fpu_st0())); break;    // FABS
                    case 4: fpu_ftst(); break;                       // FTST
                    case 5: fpu_fxam(); break;                       // FXAM
                    default: break;
                }
                break;
            case 5:
                switch (i) {
                    case 0: fpu_push(1.0L); break;                              // FLD1
                    case 1: fpu_push(3.32192809488736234787L); break;           // FLDL2T
                    case 2: fpu_push(1.44269504088896340736L); break;           // FLDL2E
                    case 3: fpu_push(3.14159265358979323846L); break;           // FLDPI
                    case 4: fpu_push(0.30102999566398119521L); break;           // FLDLG2
                    case 5: fpu_push(0.69314718055994530942L); break;           // FLDLN2
                    case 6: fpu_push(0.0L); break;                              // FLDZ
                    default: break;
                }
                break;
            case 6:
                switch (i) {
                    case 0: fpu_f2xm1(); break;                     // F2XM1
                    case 1: fpu_fyl2x(); break;                     // FYL2X
                    case 2: fpu_fptan(); break;                     // FPTAN
                    case 3: fpu_fpatan(); break;                    // FPATAN
                    case 4: fpu_fxtract(); break;                   // FXTRACT
                    case 5: fpu_fprem(true); break;                 // FPREM1
                    case 6: fpu_set_top((fpu.top - 1) & 7); break;  // FDECSTP
                    case 7: fpu_set_top((fpu.top + 1) & 7); break;  // FINCSTP
                }
                break;
            case 7:
                switch (i) {
                    case 0: fpu_fprem(false); break;                // FPREM
                    case 1: fpu_fyl2xp1(); break;                   // FYL2XP1
                    case 2: fpu_fsqrt(); break;                     // FSQRT
                    case 3: fpu_fsincos(); break;                   // FSINCOS
                    case 4: fpu_frndint(); break;                   // FRNDINT
                    case 5: fpu_fscale(); break;                    // FSCALE
                    case 6: fpu_fsin(); break;                      // FSIN
                    case 7: fpu_fcos(); break;                      // FCOS
                }
                break;
        }
        break;

    case 0xDA:
        if (modrm == 0xE9) {                                        // FUCOMPP
            fpu_compare(fpu_st0(), fpu.st[idx]);
            fpu_pop();
            fpu_pop();
        } else if (reg <= 3) {
            if (fcmov_cond(reg)) fpu_set_st0(fpu.st[idx]);           // FCMOVB/E/BE/U
        }
        break;

    case 0xDB:
        if (reg <= 3) {
            if (fcmov_cond(reg + 4)) fpu_set_st0(fpu.st[idx]);       // FCMOVNB/E/NBE/NU
        } else if (reg == 4) {
            switch (i) {
                case 2: fpu.sw &= (uint16_t)~0x00FF; break;          // FCLEX
                case 3: fpu_reset(); break;                          // FINIT
                default: break;                                      // FENI/FDISI/FSETPM 等空操作
            }
        } else if (reg == 5) {                                       // FUCOMI ST,ST(i)
            fpu_comi_flags(fpu_st0(), fpu.st[idx]);
        } else if (reg == 6) {                                       // FCOMI ST,ST(i)
            fpu_comi_flags(fpu_st0(), fpu.st[idx]);
        }
        break;

    case 0xDC:   // ST(i) op= ST(0)
        switch (reg) {
            case 0: fpu.st[idx] = fpu.st[idx] + fpu_st0(); break;    // FADD
            case 1: fpu.st[idx] = fpu.st[idx] * fpu_st0(); break;    // FMUL
            case 4: fpu.st[idx] = fpu_st0() - fpu.st[idx]; break;    // FSUBR
            case 5: fpu.st[idx] = fpu.st[idx] - fpu_st0(); break;    // FSUB
            case 6: fpu.st[idx] = fpu_st0() / fpu.st[idx]; break;    // FDIVR
            case 7: fpu.st[idx] = fpu.st[idx] / fpu_st0(); break;    // FDIV
            default: break;                                         // FCOM2/FCOMP3 在 387 上空操作
        }
        break;

    case 0xDD:
        switch (reg) {
            case 0: fpu.tw |= (uint16_t)(1 << idx); break;            // FFREE ST(i)
            case 1: break;                                           // FXCH4（空操作）
            case 2: fpu.st[idx] = fpu_st0(); break;                  // FST ST(i)
            case 3: fpu.st[idx] = fpu_st0(); fpu_pop(); break;       // FSTP ST(i)
            case 4: fpu_compare(fpu_st0(), fpu.st[idx]); break;      // FUCOM ST(i)
            case 5: fpu_compare(fpu_st0(), fpu.st[idx]); fpu_pop(); break;  // FUCOMP ST(i)
            default: break;
        }
        break;

    case 0xDE:
        if (modrm == 0xD9) {                                         // FCOMPP
            fpu_compare(fpu_st0(), fpu.st[idx]);
            fpu_pop();
            fpu_pop();
        } else {
            switch (reg) {
                case 0: fpu.st[idx] = fpu.st[idx] + fpu_st0(); fpu_pop(); break;  // FADDP
                case 1: fpu.st[idx] = fpu.st[idx] * fpu_st0(); fpu_pop(); break;  // FMULP
                case 4: fpu.st[idx] = fpu_st0() - fpu.st[idx]; fpu_pop(); break;  // FSUBRP
                case 5: fpu.st[idx] = fpu.st[idx] - fpu_st0(); fpu_pop(); break;  // FSUBP
                case 6: fpu.st[idx] = fpu_st0() / fpu.st[idx]; fpu_pop(); break;  // FDIVRP
                case 7: fpu.st[idx] = fpu.st[idx] / fpu_st0(); fpu_pop(); break;  // FDIVP
                default: break;
            }
        }
        break;

    case 0xDF:
        switch (reg) {
            case 0: fpu.tw |= (uint16_t)(1 << idx); fpu_pop(); break;  // FFREEP ST(i)
            case 1: case 2: case 3: break;                            // 空操作
            case 4: cpu.ax = fpu.sw; break;                           // FNSTSW AX
            case 5: fpu_comi_flags(fpu_st0(), fpu.st[idx]); fpu_pop(); break;  // FUCOMIP
            case 6: fpu_comi_flags(fpu_st0(), fpu.st[idx]); fpu_pop(); break;  // FCOMIP
            default: break;
        }
        break;
    }
}

// ============================================================
// ESC 指令入口
// ============================================================
void fpu_escape(uint8_t opcode, uint8_t modrm, uint32_t addr) {
    int mod = (modrm >> 6) & 3;
    int reg = (modrm >> 3) & 7;
    int rm  = modrm & 7;

    if (mod != 3) {
        fpu_mem_form(opcode, reg, addr);
    } else {
        fpu_reg_form(opcode, modrm, reg, rm);
    }
}