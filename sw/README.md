# DCF77 software

Host-side code for the DCF77 receiver. Default target is bare-metal
Microblaze; the libraries are portable to Linux/Zynq via a one-file
abstraction layer ([`driver/dcf77_io.h`](driver/dcf77_io.h)).

## Layout

| Directory                 | Purpose                                                              |
| ------------------------- | -------------------------------------------------------------------- |
| [`driver/`](driver/)      | `libdcf77.a` — typed C wrapper for the AXI4-Lite register file       |
| [`mld/`](mld/)            | `libdcf77_mld.a` — example Maximum-Likelihood Decoder                 |
| [`examples/`](examples/)  | `dcf77_demo.elf` — bare-metal Microblaze main loop                    |

A Python mirror of the MLD lives at
[`../simulation/scripts/dcf77_mld.py`](../simulation/scripts/dcf77_mld.py)
for bit-identical cross-validation against captured LLR streams.

## Quick start

```sh
# Host-side unit tests (no FPGA, no Xilinx tooling).
make -C driver test
make -C mld    test

# Cross-build for Microblaze.
make -C driver lib CROSS=mb-
make -C mld    lib CROSS=mb-
make -C examples   CROSS=mb-
```

## Cross-validation

The MLD's C and Python implementations are kept in step via a shared
CSV I/O format. After capturing per-second `(sec_index, am_soft,
am_valid, pm_soft, pm_valid)` from a sim TB or hardware ILA into a
CSV, both implementations must produce bit-identical output:

```sh
python3 ../simulation/scripts/dcf77_mld.py --csv corpus.csv > py.out
./mld/dcf77_mld_test --corpus < corpus.csv > c.out
diff py.out c.out                  # must be empty
```

## Configuration / build flavours

| Flag                  | Where         | Effect                                              |
| --------------------- | ------------- | --------------------------------------------------- |
| (none)                | default       | MMIO via `volatile uint32_t *`                      |
| `-DDCF77_USE_XIL_IO`  | host project  | redirect MMIO to `Xil_In32`/`Xil_Out32`             |
| `-DDCF77_HOST_TEST`   | unit tests    | redirect MMIO to function pointers (host tests)     |
| `-DDCF77_WANT_FLOAT`  | host project  | enable `dcf77_snr_db()` (libm required)             |

## See also

- [`../doc/dcf77_architecture.md`](../doc/dcf77_architecture.md) —
  receiver architecture, AXI register map, performance, host driver
  examples.
- [`../doc/dcf77_receiver_reference.md`](../doc/dcf77_receiver_reference.md)
  — per-bit AM register tables.
