/*
 * dcf77_io.h — platform abstraction for 32-bit AXI4-Lite reads/writes.
 *
 * Three build configurations:
 *
 *   1. Default (bare-metal Microblaze / any platform with mmap'd MMIO):
 *      uses a `volatile uint32_t *` cast on the base+offset address.
 *
 *   2. -DDCF77_USE_XIL_IO: redirects to Xil_In32/Xil_Out32 from the
 *      Xilinx BSP. Required when the BSP's data-cache policy needs the
 *      Xilinx macros (rare on Microblaze, common on Zynq).
 *
 *   3. -DDCF77_HOST_TEST: redirects to function pointers populated by
 *      the unit-test harness. No actual MMIO.
 */
#ifndef DCF77_IO_H_
#define DCF77_IO_H_

#include <stdint.h>

#ifdef DCF77_HOST_TEST
/* Host-test mode: tests register a fake register file. */
extern uint32_t (*dcf77_test_read32)(uintptr_t addr);
extern void     (*dcf77_test_write32)(uintptr_t addr, uint32_t v);

static inline uint32_t dcf77_read32(uintptr_t addr)
{
    return dcf77_test_read32(addr);
}
static inline void dcf77_write32(uintptr_t addr, uint32_t v)
{
    dcf77_test_write32(addr, v);
}

#elif defined(DCF77_USE_XIL_IO)
#include "xil_io.h"
static inline uint32_t dcf77_read32(uintptr_t addr)
{
    return Xil_In32((UINTPTR)addr);
}
static inline void dcf77_write32(uintptr_t addr, uint32_t v)
{
    Xil_Out32((UINTPTR)addr, v);
}

#else
static inline uint32_t dcf77_read32(uintptr_t addr)
{
    return *(volatile uint32_t *)addr;
}
static inline void dcf77_write32(uintptr_t addr, uint32_t v)
{
    *(volatile uint32_t *)addr = v;
}
#endif

#endif /* DCF77_IO_H_ */
