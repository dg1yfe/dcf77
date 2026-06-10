# DCF77 demo (bare-metal Microblaze)

A minimal example that polls the `dcf77_axi_lite` slave once per
second and prints decoded time over the BSP UART, using the driver
+ MLD libraries from [`../driver/`](../driver/) and [`../mld/`](../mld/).

## Build

Two paths.

### Plain Makefile (command line)

Requires the Xilinx `mb-gcc` toolchain on PATH and a generated BSP
with `xparameters.h` + `libxil.a`.

```sh
# Cross-build the supporting libraries first.
make libs CROSS=mb-

# Link the demo against them + your BSP.
make CROSS=mb- BSP_INC=/path/to/bsp/include LDFLAGS=-L/path/to/bsp/lib
```

Produces `dcf77_demo.elf`. Load via Vitis or `xsct`'s `dow` command.

### Vitis application project

The recommended workflow when you already have a Vivado design
exporting `dcf77_axi_lite`:

1. **Create a new Application Project** in Vitis, targeting your
   Microblaze BSP.
2. **Add the driver + MLD sources** as filesystem links or copies:
   - `../driver/dcf77.c` + `dcf77.h` + `dcf77_io.h`
   - `../mld/dcf77_mld.c` + `dcf77_mld.h`
3. **Add `dcf77_demo.c`** as the project's `main`.
4. **Include paths**: in *C/C++ Build → Settings → Compiler →
   Includes*, add the driver and mld source directories.
5. **Optional**: define `DCF77_USE_XIL_IO` if your BSP requires
   `Xil_In32` / `Xil_Out32` for MMIO.
6. **Build** the project — Vitis generates the linker script + BSP
   wrappers automatically.

## Expected output

```
--- DCF77 demo ---
base=0x44a00000  pm_present=1  pm_debug_present=0

[wait] minute boundary  am_sync=0 pm_sync=1 pm_qual=2731 rssi=27812 env_min=4031 noise=842
[wait] minute boundary  am_sync=1 pm_sync=2 pm_qual=18411 rssi=28140 env_min=3917 noise=618
[PM ] 2026-06-10 (wd 3) 14:37  dst=1 ann=0 leap=0
[MLD] 2026-06-10 (wd 3) 14:38  dst=1 ann=0 leap=0
[MLD] 2026-06-10 (wd 3) 14:39  dst=1 ann=0 leap=0
```

The `[wait]` lines surface signal-quality fields once per minute
while neither path has decoded yet. `[PM]` / `[AM]` lines mean the
hardware decoder produced a time. `[MLD]` lines come from the
soft-combining decoder (combines AM + PM LLRs and applies
parity-driven single-bit-flip recovery — typically locks earliest).

## Tuning

The demo uses default MLD weights. To adjust:

```c
dcf77_mld_init(&mld);
mld.alpha       = 0x60;   /* downweight noisy AM */
mld.allow_flip  = true;
mld.require_sik = false;
```

If your hardware build has `WITH_PM=false`, the driver's
`pm_present` cache bit is zero and the MLD silently degrades to
AM-only.

## Caveats

- The demo busy-polls. For low-power deployments, drive the loop
  body from a 1 ms `XTmrCtr` interrupt and `xil_idle` between
  polls.
- Year is reported as `20YY`; replace the prefix if you're using a
  non-2000 century.
- The MLD's per-minute β scaling reads `pm_lock_qual` at the minute
  boundary. If your design needs faster adaptation (e.g. lossy
  fading), poll `dcf77_read_quality` more frequently.
