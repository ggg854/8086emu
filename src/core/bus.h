#ifndef CORE_BUS_H
#define CORE_BUS_H

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ------------------------------------------------------------------
 * 总线（Bus）抽象层
 *
 * CPU 执行 IN/OUT 时只与 Bus 打交道：bus_read_port() / bus_write_port()。
 * Bus 按端口号把请求路由给已注册的设备（每个设备认领一段连续端口范围）。
 * 后注册者优先，因此machine层可以先注册通用默认处理，再让具体设备覆盖。
 *
 * 设计原则（遵循架构路线图「不要过度设计」）：裸 C + struct + 函数指针，
 * 不引入虚函数/复杂继承。设备只需提供 read/write 两个回调即可挂上总线。
 * ------------------------------------------------------------------ */

/* 设备读写回调：ctx 为设备私有数据；port 为绝对端口号。 */
typedef uint8_t (*bus_read_fn)(void *ctx, uint16_t port);
typedef void    (*bus_write_fn)(void *ctx, uint16_t port, uint8_t val);

typedef struct Bus Bus;

/* 注册一个设备，认领 [start, start+len) 端口区间。后者优先（覆盖前面同区间）。 */
void bus_register(Bus *bus, const char *name, void *ctx,
                  bus_read_fn rd, bus_write_fn wr,
                  uint32_t start, uint32_t len);

/* 未命中任何设备时的默认处理（通常读返回 0xFF / 写忽略）。 */
void bus_set_default(Bus *bus, bus_read_fn rd, bus_write_fn wr, void *ctx);

/* 清空总线（设备表 + 默认处理）。机器重启/重新装配前调用，保证注册幂等。 */
void bus_reset(Bus *bus);

uint8_t bus_read_port(Bus *bus, uint16_t port);
void    bus_write_port(Bus *bus, uint16_t port, uint8_t val);

/* 全局单例总线，供 cpu.c 等设备无关代码直接取用。 */
Bus *bus_get_global(void);

#ifdef __cplusplus
}
#endif

#endif /* CORE_BUS_H */
