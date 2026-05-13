# DCF77 Receiver/Decoder

VHDL-2008 implementation of a DCF77 (77.5 kHz longwave time-signal) AM
receiver and frame decoder, packaged as an AXI4-Lite peripheral.  Target is
AMD/Xilinx Artix-7 200 at a 125 MHz system clock; the ADC stream is 14-bit
signed samples at 125 MS/s.

## Architecture

Signal-processing chain — decimate first, mix later.  Only one carry chain
runs at full 125 MHz timing; the inner CIC stages live behind
`set_multicycle_path` constraints.

```
adc[14b, 125 MS/s]
   ▼
CIC₀  (N=3, R=2,  M=1)        17 b integrators @ 125 MHz, single-cycle
   ▼  62.5 MS/s
CIC₁  (N=3, R=16, M=1)        29 b integ / 28 b comb (Hogenauer pruned)
   ▼  3.91 MS/s
CIC₂  (N=3, R=16, M=1)        31 b integ / 26 b comb
   ▼  244.140 kS/s, 22 b signed
sign-LO mixer ◀── NCO (24 b phase acc, 77.5 kHz, ±1.8 mHz)
   ▼ I, Q (22 b each)
power = I² + Q² (1 × DSP48E1, MAC time-shared)
   ▼
adaptive threshold (slow IIRs for env_max, half-peak threshold, hysteresis)
   ▼ digital pulse
pulse-width FSM @ 244 kS/s:  100 ms±10 ms → bit 0,  200 ms±10 ms → bit 1,
                             no absenkung in expected window → minute mark
   ▼
59-bit frame buffer
   ▼
BCD + parity + plausibility checks  (per doc/dcf77_format_spec.md)
   ▼
AXI4-Lite slave  (8 registers)
```

The post-mixer I/Q is also exposed as a `dcf77_iq_tap_type` port for a
future PM (BPSK / m-sequence) correlator.

Design decisions are described in
`/Users/eligs/.claude/plans/this-project-aims-to-toasty-pelican.md`; CIC
internal widths come from the Hogenauer analysis emitted by
`simulation/scripts/cic_pruning.py`.

## Repository layout

```
inc/                shared packages (compiled first)
  dcf77_pkg.vhd            types, register map, pack/unpack helpers
  dcf77_cic_widths_pkg.vhd CIC widths (generated, see below)
  axi4lite_interface_pkg.vhd channel record types

src/dcf77/          RTL
  dcf77_cic_dec.vhd        generic 3-stage CIC building block (×3 in cascade)
  dcf77_nco.vhd            24 b phase-accumulator DDS, sign-only I/Q outputs
  dcf77_mixer.vhd          22 b cond-negate I/Q
  dcf77_env.vhd            I²+Q² with time-shared DSP48E1 MAC
  dcf77_bit_det.vhd        adaptive threshold + pulse-width FSM + sync state
  dcf77_frame_dec.vhd      59-bit buffer + BCD/parity decoder
  dcf77_core.vhd           top-level RTL wiring (no AXI here)
  axi_readmux.vhd          AXI4-Lite read-channel decoder
  axi_writemux.vhd         AXI4-Lite write-channel decoder
  dcf77_axi_lite.vhd       top-level shell: core + readmux + writemux + flat AXI ports

constraints/
  dcf77.xdc                set_multicycle_path for CIC₁ (×2) / CIC₂ (×32)
                           and false_path on the slow IIRs

simulation/scripts/
  run.py                   VUnit runner (also registers TB configurations)
  cic_pruning.py           Hogenauer width / SNR analysis (see "Tools" below)

simulation/tb/             VUnit testbenches (00..10_)
doc/                       protocol spec + Gaisler papers + AXI reference
```

## Coding style

Follows the Gaisler structured two-process method (see `doc/structdes.pdf`
and `doc/vhdl2proc.pdf`):

* one combinational and one sequential process per entity;
* one register record (`r`, `rin`) holding all flip-flops; ports use
  records grouped by direction;
* sequential statements (loops, case, if/elsif) inside the combinational
  process — no concurrent dataflow tangle;
* signal-processing nets are `ieee.fixed_pkg` `sfixed` / `ufixed` with
  explicit `(high downto low)` ranges so LSB scale is encoded in the type;
* counters and bit-vectors stay on plain `unsigned` / `std_logic_vector`.

## Running the test suite

```sh
# Activate a venv with `vunit_hdl` installed first.
python3 simulation/scripts/run.py            # run all 31 tests
python3 simulation/scripts/run.py --list     # list tests without running
python3 simulation/scripts/run.py "*nco*"    # filter by name
python3 simulation/scripts/run.py --clean    # rebuild from scratch
```

Tests run under GHDL with `--std=08 -fsynopsys -frelaxed`.  Build
artefacts land in `vunit_out/` (gitignored).  Total runtime ≈ 35 s.

## AXI4-Lite register map

| Offset | RW | Name        | Layout                                                    |
| ------ | -- | ----------- | --------------------------------------------------------- |
| 0x00   | RW | CTRL        | bit 0 enable (sticky), bit 1 soft_reset (1-clock pulse)   |
| 0x04   | R  | STATUS      | 1..0 sync_state, 2 time_valid, 13..8 bit_index, 31..16 RSSI |
| 0x08   | R  | TIME_HMS    | 5..0 minute, 12..8 hour                                   |
| 0x0C   | R  | DATE        | 7..0 year, 11..8 month, 20..16 day, 26..24 weekday        |
| 0x10   | R  | FLAGS       | 0 dst_active, 1 dst_announce, 2 leap_announce             |
| 0x14   | R  | RSSI        | 31..16 magnitude (top 16 b of env_max)                    |
| 0x18   | R  | LAST_FRAME0 | raw frame bits 0..31                                      |
| 0x1C   | R  | LAST_FRAME1 | raw frame bits 32..58                                     |

`sync_state` encoding: `00` = search, `01` = lock, `10` = hold.  After
`T_HOLD_SEC` (default 120 s) without a minute mark the state falls from
`lock` to `hold`; after `T_SEARCH_SEC` (default 300 s) it falls to
`search`.  Reads of addresses ≥ 0x20 return DECERR; writes to any
read-only offset return SLVERR.

## Tools — `cic_pruning.py`

This script computed the per-stage CIC widths in
`inc/dcf77_cic_widths_pkg.vhd` from the design parameters (R=2/16/16,
N=3, M=1, input 14 b, output 22 b) using a Hogenauer-style L2-norm noise
analysis.  The output is **checked into the repository**; the script is
kept for reproducibility but does not need to run on every build.

Re-run it only when the CIC topology changes:

```sh
python3 simulation/scripts/cic_pruning.py
```

The script has no external dependencies (pure Python) and emits both the
VHDL constants package and a plain-text width / SNR report under
`simulation/derived/`.

## Integration notes

* The slave is in the `s_axi_aclk` clock domain.  `dcf77_core` assumes
  `adc_valid` and `adc_sample` are synchronous to the same clock.  If the
  ADC has its own clock, add a CDC FIFO upstream of this entity.
* `set_multicycle_path` in `constraints/dcf77.xdc` uses wildcards on the
  CIC instance names (`*u_cic1*`, `*u_cic2*`); review after the first
  Vivado place-and-route to confirm the hierarchical paths resolve.
* The host's `soft_reset` bit and the external `!aresetn` are OR-combined
  inside `dcf77_axi_lite` to drive the synchronous reset of
  `dcf77_core` — no separate reset distribution.

## Known limitations

* AM only.  PM (BPSK / m-sequence) is out of scope; the I/Q tap port is
  exposed so a future PM correlator can attach there.
* End-to-end functional verification is a single scaled-down bit;
  a full-minute synthetic test would require either a long sim run or
  drastically scaled bit_det generics.
* No interrupt line is generated.  The host polls STATUS for time_valid.
* Sticky enable, pulsing soft_reset.  No watchdog on dropouts beyond the
  sync-state demotion sequence (`lock → hold → search`).
