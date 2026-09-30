// fpu.h - 8087 FPU Emulation
#ifndef FPU_H
#define FPU_H

#include <stdint.h>
#include <stdbool.h>

// 浮点寄存器栈
typedef struct {
    long double st[8];
    uint16_t sw;      // 状态字
    uint16_t cw;      // 控制字
    uint16_t tw;      // 标记字
    int top;          // 栈顶索引 0~7
} FPU;

extern FPU fpu;

void fpu_init(void);
void fpu_reset(void);

// 执行 ESC 指令
// opcode: 0xD8~0xDF
// modrm: ModR/M 字节
// addr: 内存地址（如果 mod != 3）
void fpu_escape(uint8_t opcode, uint8_t modrm, uint32_t addr);

#endif