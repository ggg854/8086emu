#include "bus.h"
#include <string.h>

/* 设备表上限：本模拟器端口设备数量有限，固定数组足够，避免动态分配。 */
#define BUS_MAX_DEVS 64

typedef struct {
    const char  *name;     /* 调试用设备名            */
    void        *ctx;      /* 设备私有数据            */
    bus_read_fn  rd;       /* 读端口回调              */
    bus_write_fn wr;       /* 写端口回调              */
    uint16_t     start;    /* 认领区间起点            */
    uint16_t     len;      /* 区间长度（端口数）      */
} BusDev;

struct Bus {
    BusDev   devs[BUS_MAX_DEVS];
    int      ndevs;
    bus_read_fn  def_rd;   /* 默认读（未命中设备时）  */
    bus_write_fn def_wr;   /* 默认写                  */
    void    *def_ctx;
};

/* 全局单例：所有设备/CPU 共享同一根总线。 */
static Bus g_bus;

Bus *bus_get_global(void) { return &g_bus; }

void bus_register(Bus *bus, const char *name, void *ctx,
                  bus_read_fn rd, bus_write_fn wr,
                  uint16_t start, uint16_t len) {
    if (!bus || bus->ndevs >= BUS_MAX_DEVS) return;
    BusDev *d = &bus->devs[bus->ndevs++];
    d->name  = name;
    d->ctx   = ctx;
    d->rd    = rd;
    d->wr    = wr;
    d->start = start;
    d->len   = len;
}

void bus_set_default(Bus *bus, bus_read_fn rd, bus_write_fn wr, void *ctx) {
    bus->def_rd  = rd;
    bus->def_wr  = wr;
    bus->def_ctx = ctx;
}

void bus_reset(Bus *bus) {
    bus->ndevs = 0;
    bus->def_rd = NULL;
    bus->def_wr = NULL;
    bus->def_ctx = NULL;
}

uint8_t bus_read_port(Bus *bus, uint16_t port) {
    /* 后注册者优先：从表尾向前找第一个覆盖该端口的设备。 */
    for (int i = bus->ndevs - 1; i >= 0; i--) {
        BusDev *d = &bus->devs[i];
        if (port >= d->start && port < d->start + d->len)
            return d->rd(d->ctx, port);
    }
    if (bus->def_rd) return bus->def_rd(bus->def_ctx, port);
    return 0xFF;   /* 无设备认领：等价于浮空总线，读回 0xFF */
}

void bus_write_port(Bus *bus, uint16_t port, uint8_t val) {
    for (int i = bus->ndevs - 1; i >= 0; i--) {
        BusDev *d = &bus->devs[i];
        if (port >= d->start && port < d->start + d->len) {
            d->wr(d->ctx, port, val);
            return;
        }
    }
    if (bus->def_wr) bus->def_wr(bus->def_ctx, port, val);
}
