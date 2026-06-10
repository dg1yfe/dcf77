# DCF77 host driver

A portable C library wrapping the `dcf77_axi_lite` AXI4-Lite slave.
Default target: bare-metal Microblaze. Trivially re-targetable to
Linux/Zynq by redefining the read32/write32 helpers in `dcf77_io.h`.

For the AXI register layout and bit fields the driver mirrors, see
[`doc/dcf77_architecture.md`](../../doc/dcf77_architecture.md) §4.

## Quick start

```sh
# Host-side unit tests (no HW, no Xilinx tooling).
make test

# Build the static library for the host (links against libdcf77.a).
make lib

# Cross-build for Microblaze.
make lib CROSS=mb-
```

## API at a glance

```c
#include "dcf77.h"

dcf77_handle_t h;
dcf77_init(&h, XPAR_DCF77_AXI_LITE_0_BASEADDR);

if (h.pm_present) {
    /* Optional: poll a few seconds of soft metrics. */
    if (dcf77_second_tick(&h)) {
        dcf77_soft_metrics_t s;
        dcf77_read_soft(&h, &s);
        dcf77_clear_second_tick(&h);
        /* s.am_soft, s.pm_soft, s.sec_index, s.pm_valid, ... */
    }
}

dcf77_time_t t;
if (dcf77_read_time(&h, &t)) {
    /* t.year/month/day/hour/minute/weekday + flags + source */
}
```

The hardware exposes a level-sensitive `irq` line driven by
`OR(IRQ_FLAG & IRQ_MASK)`. Set up the mask once and either wire
`irq` to the CPU's interrupt controller or poll
`dcf77_irq_read_flags`:

```c
/* Unmask the per-second tick + a few state-change events. */
dcf77_irq_set_mask(&h, DCF77_IRQ_SECOND_TICK_BIT
                     | DCF77_IRQ_PLL_LOCK_RISE_BIT
                     | DCF77_IRQ_AM_TIME_VALID_BIT);

for (;;) {
    uint32_t flags = dcf77_irq_read_flags(&h);
    if (!flags) continue;
    dcf77_irq_clear(&h, flags);
    /* dispatch on flag bits ... */
}
```

## Capability detection

`dcf77_init` reads `STATUS` once and stores two bits:

- `h.pm_present`       — `WITH_PM`       was true at synthesis.
- `h.pm_debug_present` — `WITH_PM_DEBUG` was true at synthesis.

The PM-only and PLL-debug functions early-return `false` when their
gate is off, so caller code can be unconditional.

## Vitis / XilSDK integration

This is a "static-library" component. For a Vitis project:

1. **Add the library**: in your application project's properties →
   *C/C++ Build* → *Settings* → *Libraries*, add the path to
   `libdcf77.a` and the include path to this directory.

2. **Optional `DCF77_USE_XIL_IO`**: define this in your Vitis project
   if the BSP needs `Xil_In32`/`Xil_Out32` (data-cache policy, Zynq
   PS-side, etc.). Microblaze on uncached MMIO works fine with the
   default `volatile uint32_t *` access.

3. **Optional `DCF77_WANT_FLOAT`**: define this to enable
   `dcf77_snr_db()`. Pulls in `log10f` from libm — keep undefined for
   the smallest binary.

A minimal `main.c` consuming the driver lives in
[`../examples/dcf77_demo.c`](../examples/dcf77_demo.c).

## Footprint

Host-side `text+rodata+bss` (gcc -Os, x86_64):

```
$ size libdcf77.a
   text    data     bss
   1840       0       0   dcf77.o
```

Microblaze build is comparable. Adding `-DDCF77_WANT_FLOAT` brings in
`log10f` (~1 kB).

## Build flavours

| Flag                  | Effect                                              |
| --------------------- | --------------------------------------------------- |
| (none)                | Default MMIO via `volatile uint32_t *`              |
| `-DDCF77_USE_XIL_IO`  | Redirect to `Xil_In32`/`Xil_Out32`                  |
| `-DDCF77_HOST_TEST`   | Redirect to function pointers (unit tests only)     |
| `-DDCF77_WANT_FLOAT`  | Enable `dcf77_snr_db()` (libm required)             |
