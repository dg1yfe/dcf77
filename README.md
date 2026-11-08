# DCF77 Receiver/Decoder

VHDL-2008 implementation of a DCF77 (77.5 kHz longwave time-signal) receiver
and frame decoder, exposed as an AXI4-Lite peripheral. It decodes both the
amplitude-modulated (AM) second marks and the phase-modulated (PM / PSK)
signal. Target is AMD/Xilinx Artix-7 at a 125 MHz system clock; the ADC stream
is 14-bit signed samples at 125 MS/s.

## Architecture

Signal-processing chain — decimate first, mix later. Only the first CIC carry
chain runs at full 125 MHz timing; the inner stages advance on clock-enable
strobes and live behind `set_multicycle_path` constraints. One 125 MHz clock
domain throughout.

```
adc[14b] --125MS/s--> CIC0(R=2) -> CIC1(R=10) -> CIC2(R=25) --250kS/s--> mixer --+--> I
                                                                  NCO ---/         +--> Q
                                                                   ^
                                                  phase_inc_offset (carrier loop, below)

I -> i_boxcar(R=125) --+
Q -> q_boxcar(R=125) --+--2kS/s--> CORDIC (I/Q -> polar) --> mag[19b], phase[12b]

  mag   --> mag_iir(20Hz) --> env_dec(/10) --200S/s--> env_tracker --> bit_det
  phase -+-> carrier_pll(PI, KP=10/KI=23) --> phase_inc_offset --> NCO
         +-> phase_quality --> pll_lock
         +-> pm_correlator --> pm_frame_dec ------------------+  (only when WITH_PM)
                                                              |
       bit_det --(gated by pll_lock)--> frame_dec (AM) --> status_out <--+ (PM)
```

- **Pre-mixer** (`CIC0..CIC2`, real-valued): a 3-stage cascaded CIC decimator
  (uniform N=3, M=1) drops 125 MS/s to 250 kS/s. Widths follow a Hogenauer
  pruning analysis (see Tools below).
- **Sign-LO mixer**: a cond-negate mixer driven by the NCO folds the residual
  carrier to DC, splitting the real stream into I/Q at 250 kS/s.
- **Post-mixer** (per channel): a single-stage boxcar (`dcf77_boxcar`, R=125)
  decimates each of I/Q to 2 kS/s — flat and cheap, replacing the earlier
  multi-stage complex CIC cascade.
- **CORDIC**: a time-shared vectoring CORDIC converts I/Q to magnitude
  (≈ K·√(I²+Q²), 19 b) and phase (atan2, 12 b) at 2 kS/s.
- **AM path**: magnitude → 20 Hz IIR → /10 decimator → 200 S/s envelope →
  `env_tracker` (env_max / env_min / noise-MAD + falling-edge detect) →
  symbol-structural `bit_det` → `frame_dec` (BCD + parity + plausibility).
- **Carrier loop**: CORDIC phase → `carrier_pll` (2nd-order PI, KP=10/KI=23) →
  `phase_inc_offset` → NCO, closing the mix-down loop. `phase_quality` derives
  `pll_lock` from the same phase, which gates the AM outputs into `frame_dec`.
- **PM path** (`WITH_PM`): CORDIC phase → `pm_correlator` (BPSK/SIK matched
  filter) → a second `frame_dec` in PM mode → PM time fields.

`bit_det` runs at 200 S/s over a free-running modulo-`BIT_PERIOD` (=200) counter,
anchored on the first falling edge:

- W1 `[0..20)` (0–100 ms): minute-mark candidate — `acc_W1 > 7·env_max` → mm.
- W2 `[20..40)` (100–200 ms): bit discriminator — `acc_W2 < 0.578·iir_high` → bit-1.
- W3 `[40..200)`: carrier reference, feeds the `iir_high` tracker.
- Drops the anchor after 3 consecutive mm candidates (wrong-phase recovery).

CIC internal widths come from the Hogenauer analysis emitted by
`simulation/scripts/cic_pruning.py`; design rationale is documented in
[`doc/dcf77_architecture.md`](doc/dcf77_architecture.md).

## Repository layout

```
inc/                shared packages (compiled first)
  dcf77_pkg.vhd              types, register map, pack/unpack helpers
  dcf77_cic_widths_pkg.vhd   CIC0..CIC2 (+ TB-only CIC_ENV) widths (generated)
  axi4lite_interface_pkg.vhd AXI4-Lite channel record types

src/dcf77/          RTL
  dcf77_cic_dec.vhd        generic CIC building block (CIC0..CIC2, pre-mixer)
  dcf77_nco.vhd            phase-accumulator DDS, signed phase_inc_offset port
  dcf77_mixer.vhd          sign-LO cond-negate I/Q mixer
  dcf77_boxcar.vhd         1-stage accumulate-and-dump decimator (I/Q + env)
  dcf77_cordic_atan2.vhd   vectoring CORDIC: I/Q -> magnitude + phase
  dcf77_iir_lpf.vhd        generic 1st-order IIR (y += (x-y)>>SHIFT)
  dcf77_carrier_pll.vhd    2nd-order PI carrier-tracking loop
  dcf77_phase_quality.vhd  ±1 hysteresis lock indicator on CORDIC phase
  dcf77_env_tracker.vhd    envelope stats (max/min/noise) + falling-edge detect
  dcf77_bit_det.vhd        symbol-structural AM bit detector @ 200 S/s
  dcf77_frame_dec.vhd      59-bit buffer + BCD/parity decoder (AM and PM modes)
  dcf77_pm_correlator.vhd  PM (BPSK/SIK) matched-filter correlator + DLL
  dcf77_irq_gen.vhd        edge-detecting interrupt aggregator (W1C flags + mask)
  dcf77_core.vhd           top-level RTL wiring (no AXI here)
  axi_readmux.vhd          AXI4-Lite read-channel arbiter + register decode
  axi_writemux.vhd         AXI4-Lite write-channel arbiter + register decode
  dcf77_axi_lite.vhd       top-level shell: core + muxes + irq_gen; record-
                            bundled AXI ports (canonical non-IP top)

constraints/
  dcf77.xdc                set_multicycle_path for CIC1 (×2) / CIC2 (×20) /
                           boxcars (×500) / mag_iir + env_dec (×62500);
                           false_path on the slow env_tracker / bit_det IIRs

simulation/scripts/
  run.py                   VUnit runner (also registers TB configurations)
  cic_pruning.py           Hogenauer width / SNR analysis (see Tools below)
  dcf77_soft_decode.py     reference software AM decoder (replay ground truth)
  wav_to_iq.py / wav_to_if.py / synth_dcf77_wav.py   replay stimulus tooling

simulation/tb/             VUnit testbenches (block + replay + PM)
doc/                       architecture + protocol spec + AXI interface + papers
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

The AXI4-Lite slave structure and this coding style are described in detail in
[`doc/axi4lite_interface.md`](doc/axi4lite_interface.md).

## Running the test suite

```sh
# Activate a venv with `vunit_hdl` installed first.
python3 simulation/scripts/run.py            # build + run the suite
python3 simulation/scripts/run.py --list     # list tests without running
python3 simulation/scripts/run.py "*nco*"    # filter by name
python3 simulation/scripts/run.py -p 8        # parallelise (recommended)
python3 simulation/scripts/run.py --clean    # rebuild from scratch
```

Tests run under GHDL with `--std=08 -fsynopsys -frelaxed`. Build artefacts land
in `vunit_out/` (gitignored). The suite covers per-block testbenches (CIC, NCO,
CORDIC, bit_det, frame_dec, AXI-Lite) plus baseband/IF/PM replay benches; the
replay benches register per WAV/capture found in `samples/` (see below).

## Documentation

The README is developer-facing. For the end-to-end architecture (AM + PM
signal chains, edge-protected T=1 SIK detection, AXI register map, synthesis
generics, corpus performance numbers, host driver examples) see
[**`doc/dcf77_architecture.md`**](doc/dcf77_architecture.md).

The AXI4-Lite slave structure, channel records, register pack/unpack pattern,
and the two-process coding style are in
[`doc/axi4lite_interface.md`](doc/axi4lite_interface.md).

For the integrator-oriented per-bit register tables and software pseudo-code see
[`doc/dcf77_receiver_reference.md`](doc/dcf77_receiver_reference.md).

The DCF77 protocol itself (AM frame layout, BCD encoding, parity rules, PM
hints) is documented in [`doc/dcf77_format_spec.md`](doc/dcf77_format_spec.md).

## AXI4-Lite register map

32-bit registers on an 8-bit byte address (word index = offset / 4). The PM
block (`0x30`, `0x38`–`0x4C`) is present only when `WITH_PM`; the PLL telemetry
block (`0x50`/`0x54`/`0x58`) only when `WITH_PM_DEBUG` — otherwise those offsets
return DECERR. Offsets `0x10` and `0x44` are reserved (DECERR).

| Offset | RW | Name | Layout |
| ------ | -- | ---- | ------ |
| 0x00 | RW | CTRL | 0 enable (sticky), 1 soft_reset (1-clk pulse), 2 env_mode |
| 0x04 | R | STATUS | 1..0 sync_state, 2 time_valid, 3 pm_present, 4 pm_debug_present, 5 pll_lock, 6 anchor_held, 13..8 bit_index, 31..16 RSSI |
| 0x08 | R | TIME_HMS | 5..0 minute, 12..8 hour |
| 0x0C | R | DATE | 7..0 year, 11..8 month, 20..16 day, 26..24 weekday, 27 dst_active, 28 dst_announce, 29 leap_announce |
| 0x14 | R | RSSI | 15..0 magnitude (top 16 b of env_max) |
| 0x18 | R | LAST_FRAME0 | raw frame bits 0..31 |
| 0x1C | R | LAST_FRAME1 | raw frame bits 32..58 |
| 0x20 | R | ENV_MIN | 15..0 magnitude (top 16 b of env_min trough tracker) |
| 0x24 | R | NOISE | 15..0 magnitude (top 16 b of noise-MAD IIR) |
| 0x28 | R | SYNC_DIAG | 5..0 sync_quality, 7..6 mm_quality, 13..8 cycle_in_min |
| 0x2C | R | SOFT_AM | 11..0 am_soft, 23..12 mm_soft, 29..24 bit_index, 30 am_soft_valid, 31 minute_mark |
| 0x30 | R | SOFT_PM | 5..0 pm_sec_index, 17..6 pm_soft, 30 pm_valid, 31 pm_minute_mark |
| 0x34 | R/W1C | IRQ_FLAG | 0 second_tick, 1 am_minute_mark, 2 pm_minute_mark, 3 am_time_valid, 4 pm_time_valid, 5 am_sync_lost, 6 pm_sync_lost, 7 pll_lock_rise, 8 pll_lock_fall |
| 0x38 | R | PM_STATUS | 5..0 pm_bit_index, 7..6 pm_sync_state, 8 pm_time_valid, 31..16 pm_lock_qual |
| 0x3C | R | PM_TIME_HMS | 5..0 minute, 12..8 hour (PM-decoded) |
| 0x40 | R | PM_DATE | as DATE, PM-decoded |
| 0x48 | R | PM_LAST_FRAME0 | PM raw frame bits 0..31 |
| 0x4C | R | PM_LAST_FRAME1 | PM raw frame bits 32..58 |
| 0x50 | R | PLL_PHASE_FILT | signed 29-b carrier-PLL LPF state |
| 0x54 | RW | PLL_I_STATE | signed 28-b PI integrator; write jams it |
| 0x58 | RW | PLL_OFFSET | signed 24-b NCO phase_inc_offset; write overrides |
| 0x5C | RW | IRQ_MASK | per-bit IRQ enable (same layout as IRQ_FLAG) |
| 0x60 | R | SHADOW | secondary of an atomic register pair (see below) |

`SNR_dB ≈ 20·log10((RSSI − ENV_MIN) / NOISE)` — the host reads the three fields
and computes the dB value.

**Atomic pairs**: reading a "primary" register latches its companion into
`SHADOW` on the same cycle, so a follow-up `SHADOW` read is coherent with the
primary (`TIME_HMS`→DATE, `SOFT_PM`→paired SOFT_AM, `PM_TIME_HMS`→PM_DATE).

`sync_state` encoding: `00` = search, `01` = lock, `10` = hold. It is forced to
search while `pll_lock` = 0; once locked it follows `bit_det` (lock while the
minute-mark candidate counter is non-zero, else hold). Writes to a read-only
offset return SLVERR/DECERR.

An interrupt line (`irq`) is asserted when any unmasked `IRQ_FLAG` bit is set;
`dcf77_irq_gen` sources second-tick, minute-mark, time-valid, sync-lost and
PLL-lock edge events. Flags are cleared write-1-to-clear.

## Replay test — decoding a real recording

The AM back end can be exercised against real DCF77 recordings. The baseband
replay testbench (`06_dcf77_rx_replay_tb`) injects pre-decimated complex I/Q
**at the 1 kS/s boundary**: it reconstructs the envelope locally
(`|I| + 3/8·|Q|`) and feeds the real `env_tracker` / `bit_det` / `frame_dec`
back end, bypassing the NCO, mixer, CICs and CORDIC — so a 4-minute clip decodes
in seconds. Full-chain and PM coverage come from the IF-replay benches
(`07_…if_replay`, `14_…pm_if_replay`) and the standalone PM correlator/capture
benches (`15_…pm_correlator_replay`, `16_…phase_err_capture`).

![Replay pipeline](doc/dcf77-replay-pipeline.drawio.svg)

The recordings are too large for this repository and are published as a
tarball with a pinned SHA-256. Fetch them, drop in a WAV of your own, run the
preprocessor, run the regression:

```sh
# Download the reference recordings into samples/ (see samples/PROVENANCE.md).
python3 simulation/scripts/fetch_samples.py

# Drop in your own WAV (mono real-valued, ≥ 12 kHz, DCF77 at 5 kHz IF).
cp my_dcf77_recording.wav samples/

# (Optional) synthetic recording for self-testing.
python3 simulation/scripts/synth_dcf77_wav.py    # writes samples/synth.wav

# Preprocess every WAV in samples/ into simulation/derived/<stem>.{s16,meta}.
# Idempotent: skips WAVs whose outputs are newer.  Use --force to re-run.
python3 simulation/scripts/wav_to_iq.py --all

# Run the replay testbench (registered once per prepared WAV).
python3 simulation/scripts/run.py "*rx_replay*"
```

The preprocessor:

1. Mixes the WAV down with a complex LO at `--if-hz` (default 5000 Hz)
   so the DCF77 carrier lands at DC.
2. Low-passes (60 Hz) and decimates to **1 kS/s**.
3. Writes interleaved little-endian int16 I/Q pairs to `*.s16`.
4. Runs its **own software DCF77 decoder** (`dcf77_soft_decode.py`,
   see [§ Soft decoder internals](#soft-decoder-internals)) on the
   same I/Q stream and caches the recovered timestamp in `*.meta`.

Two replay tests are registered per WAV when its `.s16`/`.meta` exist:

* **`smoke`** — feeds the first ~60 s of the recording.  If the soft
  decoder locked (`expect_full_decode=1`), asserts `bits_seen ≥ 30` and
  `minutes_seen ≥ 1`; otherwise just verifies the pipeline runs end-to-end.
* **`full_decode`** — feeds the entire recording, asserts the decoded
  fields match the reference time the Python decoder recovered from
  the same audio.  Only registered when the preprocessor locked
  (`expect_full_decode=1` in the meta) **and** the
  `DCF77_FULL_REPLAY=1` environment variable is set.

  ```sh
  DCF77_FULL_REPLAY=1 python3 simulation/scripts/run.py "*rx_replay*full*"
  ```

WAVs that the Python decoder couldn't decode (weak / noisy recordings)
still get a smoke config — only the full-decode validation is skipped
for them.  When `expect_full_decode=0` in the .meta the smoke config
drops the `bits_seen >= 30` / `minutes_seen >= 1` assertion so the
pipeline still runs end-to-end without the regression failing on the
receiver's decode limits.  The TB log line
`EXPECT_DECODE_PROGRESS=false; ...` makes the relaxed mode explicit.

### Soft decoder internals

`dcf77_soft_decode.py` recovers the embedded timestamp via a three-template
matched filter on the same 1 kS/s I/Q stream the testbench will inject.
Two pulse-shape templates (`t_bit0`, `t_bit1`) score "how dip-like is this
second's envelope?", a third constant template (`t_mm`) confirms "envelope
sat above the local mean = no dip = minute mark".  Phase lock for the
1 Hz second grid falls out of a coherent sum across the whole recording;
per-second decisions are then assembled into 59-bit frames and validated
by framing + parity + plausibility checks.

![Matched-filter detector](doc/dcf77-matched-filter.drawio.svg)

The bit templates are linearly independent but not orthogonal (inner
product ≈ 0.667, ~48° angle), so `argmax(c_0, c_1)` discriminates by
linear-discriminant margin rather than by orthogonal projection — about
5 dB shy of the optimal orthogonalised pair.  A single parity-good,
plausibility-passing frame is enough to lock; the matched filter's
implicit coherent averaging makes false locks vanishingly unlikely.

### Verbose decode logging

Pass `-v` to `run.py` (forwarded to VUnit) to surface the per-bit /
per-minute info() log lines the testbench emits:

```sh
python3 simulation/scripts/run.py -v "*rx_replay*smoke*"
```

Sample (truncated):

```
INFO - sync_state SEARCH -> LOCK at 1234567 ns
INFO - bit 0 = '0' sync=LOCK (#1)
INFO - bit 1 = '1' sync=LOCK (#2)
...
INFO - minute_mark (#1, bits_seen=59)
INFO - time_valid: 26-5-13 (Wed) 11:42 dst='1'
```

### Synthetic test signal

`synth_dcf77_wav.py` writes to `samples/synth.wav` by default and also drops a
`samples/synth.expect` sidecar carrying the ground-truth time fields.
`wav_to_iq.py --all` picks the sidecar up automatically and cross-checks its
Python decoder against the truth, surfacing a loud warning if they ever
disagree — a regression alarm for the soft-decoder logic.

The replay tests are silently skipped if `samples/` is empty or no `.s16`/`.meta`
artefacts exist for the WAVs found there, so the core suite remains runnable
without them.

## Tools — `cic_pruning.py`

This script computes the per-stage CIC widths in
`inc/dcf77_cic_widths_pkg.vhd` from the design parameters (pre-mixer chain
R=2/10/25, N=3, M=1, input 14 b) using a Hogenauer-style L2-norm noise
analysis. It models the `CIC0..CIC2` pre-mixer chain plus the `CIC_ENV`
decimator used only by the baseband replay TB; the post-mixer I/Q path is a
plain `dcf77_boxcar` sized inline in `dcf77_core`. The output is **checked into
the repository**; the script is kept for reproducibility but does not need to
run on every build.

Re-run it only when the CIC topology changes:

```sh
python3 simulation/scripts/cic_pruning.py
```

The script has no external dependencies (pure Python) and emits both the VHDL
constants package and a plain-text width / SNR report under
`simulation/derived/`.

## Integration notes

* The slave is in the `s_axi_aclk` clock domain. `dcf77_core` assumes
  `adc_valid` and `adc_sample` are synchronous to the same clock. If the ADC has
  its own clock, add a CDC FIFO upstream of this entity.
* `set_multicycle_path` in `constraints/dcf77.xdc` uses wildcards on the CIC /
  boxcar / IIR instance names; review after the first Vivado place-and-route to
  confirm the hierarchical paths resolve.
* The host's `soft_reset` bit and the external `!aresetn` are OR-combined inside
  `dcf77_axi_lite` to drive the synchronous reset of `dcf77_core` — no separate
  reset distribution.
* `WITH_PM` / `WITH_PM_DEBUG` generics gate the PM decode stack and the PLL
  telemetry/override registers respectively, trading LUTs/BRAM/flops for
  features.

## Known limitations

* End-to-end functional verification leans on replay against captured/synthetic
  recordings; a full synthetic minute through the entire RTL chain is expensive
  to simulate, so most block coverage is per-entity.
* No dropout watchdog beyond the sync-state demotion sequence
  (`lock → hold → search`); `enable` is sticky, `soft_reset` is a 1-clock pulse.
* The PM stack assumes the AM carrier PLL has locked (it consumes the same
  CORDIC phase); PM decode does not run standalone without AM carrier lock.
