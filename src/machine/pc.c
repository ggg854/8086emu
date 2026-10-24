#include "pc.h"
#include "bus.h"
#include "io.h"
#include "dma.h"

/* ------------------------------------------------------------------
 * 把 I/O 设备装配到总线上。init_emulator() 在重启时也会重复调用，
 * 因此先用 bus_reset() 清空设备表，保证多次装配幂等。
 *
 * 顺序很关键：
 *   1) io_register_bus  —— legacy-io 兜底设备，覆盖全部 64K 端口；
 *   2) dma_register_bus —— 8237 认领 0x00-0x0F / 0x80-0x8F / 0xC0-0xDF。
 * 后注册者优先，故 DMA 会接管自己的区间，legacy 里的对应分支随之失效（后续清理）。
 * ------------------------------------------------------------------ */
void machine_io_init(void) {
    Bus* bus = bus_get_global();
    bus_reset(bus);

    io_register_bus(bus);      // legacy 兜底（最先）
    dma_register_bus(bus);     // 8237 DMA（覆盖其端口区间）
}
