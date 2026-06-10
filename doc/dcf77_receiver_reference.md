# DCF77 Receiver — Integrator Reference

This document describes the **dcf77_axi_lite** receiver module from an
integrator's point of view: what it does, what data flows through it,
what every AXI4-Lite register contains, and what the host should do to
read the decoded time and signal-quality information.

> **Note** — this reference covers the original AM-only receiver
> (registers 0x00..0x24) for historical continuity. For the current
> design including the PM (PSK/SIK) decoder stack, the PM AXI register
> block (0x30, 0x38–0x4C), the carrier-PLL telemetry block
> (0x50–0x58), the `WITH_PM` / `WITH_PM_DEBUG` synthesis generics, and
> corpus performance numbers, see
> [`dcf77_architecture.md`](dcf77_architecture.md).

For protocol details refer to `doc/dcf77_format_spec.md`.  For project
layout, test infrastructure, and developer notes see `README.md` at the
repo root.

---

## 1. What this module is

A self-contained AM receiver and decoder for the DCF77 longwave
time-signal (77.5 kHz, Mainflingen, Germany).  Takes a raw 14-bit
signed ADC stream at 125 MS/s on one side and presents the decoded
date/time plus signal-quality information through an **AXI4-Lite slave**
interface on the other side.  The post-mixer I/Q is also exposed (at
25 kS/s) for a future PM (phase-modulation) correlator.

Top-level entity is `dcf77_axi_lite` (see
`src/dcf77/dcf77_axi_lite.vhd`).  It is a thin wrapper that
instantiates `dcf77_core` (the actual receiver) plus the
`axi_readmux` / `axi_writemux` slave decoders.

---

## 2. Block diagram and data flow

![DCF77 data flow](dcf77-dataflow.drawio.svg)

Diagram source: `doc/dcf77-dataflow.drawio.svg`.  The SVG embeds the
draw.io XML — open it directly in draw.io / diagrams.net to edit.

```
  ADC                       (14 b signed, 125 MS/s, two's complement)
    │
    ▼
  ┌─────────────────────────────────────────────────┐
  │ CIC₀  R=2,  N=3   17 b integ, single-cycle      │  125 MHz
  ├─────────────────────────────────────────────────┤
  │ CIC₁  R=10, N=3   29 b integ, multicycle 2      │  62.5 MS/s
  ├─────────────────────────────────────────────────┤
  │ CIC₂  R=25, N=3   33 b integ, multicycle 20     │  6.25 MS/s
  └─────────────────────────────────────────────────┘
                       │
                       │  real-valued, 22 b, 250 kS/s
                       │  (carrier still at 77.5 kHz)
                       ▼
              ┌─────────────────────┐         ┌──────────────────┐
              │ sign-LO mixer (I,Q) │ ◀──┐    │ NCO 25-b acc     │
              └─────────────────────┘    └────│ K=10_399_018     │
                       │                      │ + phase_inc_offset│
                       │  I, Q (each 22 b,    └──────────────────┘
                       │  250 kS/s)                       ▲
        ┌──────────────┼──────────────┐                   │
        ▼              │              ▼                   │
  CIC₃_I R=10        (PM hook       CIC₃_Q R=10           │
  CIC₄_I R=25         iq_tap_out    Q-boxcar  R=25  ──────┤
   ↓ I @ 22 b,        @ 25 kS/s)     ↓ 27 b signed         │
   ↓ 1 kS/s                          ↓ 1 kS/s              │
   ▼                                 ▼                     │
  ┌──────────────────┐       ┌─────────────────────┐       │
  │ env (magnitude)  │       │ dcf77_carrier_pll    │       │
  │ |I|+(3/8)|Q|     │       │  2nd-order PI loop   │ ──────┘
  │ (no DSP)         │       │  Kp=7/1024,Ki=3/131k │  phase_inc_offset
  └──────────────────┘       │  lock_state          │
        │                    └─────────────────────┘
        │  26 b unsigned, 1 kS/s              │
        ▼                                     │
  ┌──────────────────────────────┐            │
  │ CIC_ENV  R=10, N=3            │            │
  └──────────────────────────────┘            │
        │  env @ 100 S/s                       │
        ▼                                     │
  ┌─────────────────────────────────────────────────┐
  │ symbol-structural bit_det @ 100 S/s              │
  │   - anchor on first falling edge, free-running   │
  │     modulo-BIT_PERIOD (=100) sample counter      │
  │   - W1 [0..10) acc_W1 > 7·env_max → mm candidate │
  │   - W2 [10..20) acc_W2 < 0.578·iir_high → bit-1   │
  │   - W3 [20..100) feeds iir_high carrier reference│
  │   - 3-in-a-row mm candidates → drop anchor       │
  │   - mm_counter (0..3) gates mm acceptance        │
  │ env_max / env_min / noise IIRs feed SNR registers │
  │ output gating: AND with carrier_pll.lock_state ──┘
  └─────────────────────────────────────────────────┘
                       │
                       ▼
  ┌─────────────────────────────────────────────────┐
  │ 59-bit frame buffer + BCD + parity decoder       │
  │ Plausibility checks (per spec §1.4)              │
  └─────────────────────────────────────────────────┘
                       │
                       ▼
  ┌─────────────────────────────────────────────────┐
  │ AXI4-Lite slave (10 registers, 32-b each)        │
  └─────────────────────────────────────────────────┘
```

**Total decimation** from ADC (125 MS/s) to AM bit detector (100 S/s) is
**1 250 000** = 500 (pre-mixer) × 250 (post-mixer per channel) × 10
(envelope CIC).

The entire pipeline lives in a single 125 MHz clock domain; inner CIC
stages advance on clock-enable strobes, and `set_multicycle_path` XDC
constraints (factors 2, 20, 500, 5000 — see
`constraints/dcf77.xdc`) tell Vivado to relax timing on those stages.

---

## 3. Top-level interface

Entity `dcf77_axi_lite` exposes the conventional flat AXI4-Lite slave
ports plus an ADC stream port:

| Port                       | Direction | Width | Description                                          |
| -------------------------- | --------- | ----- | ---------------------------------------------------- |
| `s_axi_aclk`               | in        | 1     | AXI clock = receiver clock = 125 MHz                  |
| `s_axi_aresetn`            | in        | 1     | active-low synchronous reset                          |
| `s_axi_awaddr`             | in        | 8     | write address                                        |
| `s_axi_awprot`             | in        | 3     | write protection (unused, ignored)                    |
| `s_axi_awvalid` / `awready`| in/out    | 1     | write-address handshake                              |
| `s_axi_wdata`              | in        | 32    | write data                                           |
| `s_axi_wstrb`              | in        | 4     | per-byte write enables                                |
| `s_axi_wvalid` / `wready`  | in/out    | 1     | write-data handshake                                 |
| `s_axi_bresp` / `bvalid` / `bready` | out/out/in | 2/1/1 | write response (OKAY/SLVERR)                |
| `s_axi_araddr`             | in        | 8     | read address                                         |
| `s_axi_arprot`             | in        | 3     | read protection (unused)                              |
| `s_axi_arvalid` / `arready`| in/out    | 1     | read-address handshake                               |
| `s_axi_rdata`              | out       | 32    | read data                                            |
| `s_axi_rresp`              | out       | 2     | read response (OKAY/DECERR)                          |
| `s_axi_rvalid` / `rready`  | out/in    | 1     | read-data handshake                                  |
| `adc_sample`               | in        | sfixed(13 downto 0) | signed ADC sample (two's complement) |
| `adc_valid`                | in        | 1     | sample-valid strobe (typically tied high)             |

The 8-bit address space gives 64 × 32-b registers; only the first 10
are used.  Reads at offsets ≥ `0x28` return `DECERR`.

The `adc_sample`/`adc_valid` pair is assumed synchronous to
`s_axi_aclk`.  If the ADC has its own clock, add a CDC FIFO upstream.

---

## 4. Generic parameters

The module ships with silicon-faithful defaults.  All generics are for
integration-time overrides (and for testbench scaling).

| Generic           | Default     | Meaning                                              |
| ----------------- | ----------- | ---------------------------------------------------- |
| `K_ATK`           | 6           | env_max / env_min attack shift (1/2^K per sample)    |
| `K_DEC`           | 12          | env_max / env_min slow-decay shift (~4 s @ 1 kHz)    |
| `K_NOISE`         | 12          | noise-MAD IIR shift (~4 s @ 1 kHz)                    |
| `T_LOW_MIN`       | 90          | minimum low-pulse width, in 1 kS/s ticks (90 ms)      |
| `T_BIT0_MAX`      | 110         | upper bound for bit-0 low pulse (110 ms)              |
| `T_BIT1_MIN`      | 190         | lower bound for bit-1 low pulse (190 ms)              |
| `T_BIT1_MAX`      | 210         | upper bound for bit-1 low pulse (210 ms)              |
| `T_GAP_MINUTE`    | 1 500       | minute-mark gap threshold (1500 ms)                   |
| `SAMPLE_RATE_HZ`  | 1 000       | env_valid rate; used to convert seconds → ticks      |
| `T_HOLD_SEC`      | 120         | sync_lock → sync_hold timeout (seconds)              |
| `T_SEARCH_SEC`    | 300         | sync_lock|hold → sync_search timeout (seconds)       |
| `NCO_PHASE_INCR`  | 81 245      | 18-b NCO increment for 77.5 kHz at 250 kS/s tick     |

---

## 5. AXI4-Lite register map

![DCF77 register map](dcf77-registers.drawio.svg)

Diagram source: `doc/dcf77-registers.drawio.svg`.  The SVG embeds the
draw.io XML — open it directly in draw.io / diagrams.net to edit.

All registers are 32 bits.  Byte addresses are word-aligned (low two
bits must be zero).

| Offset | RW  | Name        | Content                                  |
|--------|-----|-------------|------------------------------------------|
| 0x00   | RW  | CTRL        | enable, soft_reset                       |
| 0x04   | RO  | STATUS      | sync_state, time_valid, bit_index, RSSI  |
| 0x08   | RO  | TIME_HMS    | minute, hour                             |
| 0x0C   | RO  | DATE        | year, month, day, weekday                |
| 0x10   | RO  | FLAGS       | DST flags                                |
| 0x14   | RO  | RSSI        | env_max top 16 b                         |
| 0x18   | RO  | LAST_FRAME0 | raw frame bits 0..31                     |
| 0x1C   | RO  | LAST_FRAME1 | raw frame bits 32..58                    |
| 0x20   | RO  | ENV_MIN     | env_min top 16 b                         |
| 0x24   | RO  | NOISE       | noise-MAD IIR top 16 b                   |

Writes to any read-only offset return **SLVERR**.  Reads at offsets ≥
`0x28` return **DECERR**.

---

## 6. Per-register bit definitions

Bits not described below are reserved and read as zero.

### 6.1 `CTRL` — offset 0x00 — read/write

| Bits   | Field        | RW  | Description                                                      |
| ------ | ------------ | --- | ---------------------------------------------------------------- |
| 0      | `enable`     | RW  | `1` (default) lets ADC samples flow into the pipeline; `0` gates the chain, freezes state. |
| 1      | `soft_reset` | W   | Writing `1` resets the core for exactly one clock; reads back as `0` (self-clearing pulse). |
| 31..2  | reserved     | —   | Read as 0.  Writes ignored.                                       |

The default `CTRL` value after `aresetn` is `0x0000_0001` (enable on,
soft_reset clear).

### 6.2 `STATUS` — offset 0x04 — read-only

| Bits    | Field        | Description                                                  |
| ------- | ------------ | ------------------------------------------------------------ |
| 1..0    | `sync_state` | `00`=SEARCH, `01`=LOCK, `10`=HOLD.  See §7.4.                |
| 2       | `time_valid` | `1` when the *most recently completed* minute frame passed parity and plausibility checks. |
| 7..3    | reserved     | 0.                                                           |
| 13..8   | `bit_index`  | Current position within the minute, 0..58.  Value 59 means the frame is full and the decoder is waiting for the next minute mark. |
| 15..14  | reserved     | 0.                                                           |
| 31..16  | `rssi`       | Top 16 bits of the `env_max` IIR (carrier-on amplitude proxy). |

### 6.3 `TIME_HMS` — offset 0x08 — read-only

The decoded time-of-day from the most recently *successfully* parsed
minute frame.

| Bits    | Field    | Description                          |
| ------- | -------- | ------------------------------------ |
| 5..0    | `minute` | 0..59                                 |
| 7..6    | reserved | 0                                     |
| 12..8   | `hour`   | 0..23                                 |
| 31..13  | reserved | 0                                     |

### 6.4 `DATE` — offset 0x0C — read-only

| Bits    | Field     | Description                          |
| ------- | --------- | ------------------------------------ |
| 7..0    | `year`    | last two digits, 0..99 (BCD-decoded). Add the century in software. |
| 11..8   | `month`   | 1..12                                |
| 15..12  | reserved  | 0                                     |
| 20..16  | `day`     | 1..31                                |
| 23..21  | reserved  | 0                                     |
| 26..24  | `weekday` | 1=Mon, 2=Tue, …, 7=Sun                |
| 31..27  | reserved  | 0                                     |

### 6.5 `FLAGS` — offset 0x10 — read-only

| Bits    | Field           | Description                                                    |
| ------- | --------------- | -------------------------------------------------------------- |
| 0       | `dst_active`    | `1` while CEST (MESZ, UTC+2) is in effect; `0` for CET (UTC+1). |
| 1       | `dst_announce`  | Set in the hour before a DST switch.                            |
| 2       | `leap_announce` | Set in the hour before a leap-second insertion.                 |
| 31..3   | reserved        | 0.                                                             |

### 6.6 `RSSI` — offset 0x14 — read-only

| Bits   | Field   | Description                                                       |
| ------ | ------- | ----------------------------------------------------------------- |
| 15..0  | `rssi`  | Top 16 b of the adaptive `env_max` peak detector (linear power-domain magnitude). |
| 31..16 | reserved | 0.                                                               |

This is the same value the `STATUS` register exposes in bits 31..16,
duplicated in a dedicated register so the host can read it without
masking.

### 6.7 `LAST_FRAME0` — offset 0x18 — read-only

| Bits   | Field           | Description                                            |
| ------ | --------------- | ------------------------------------------------------ |
| i (0..31) | bit i of frame | The raw bit value (0 or 1) received at second i of the most recently completed minute. |

### 6.8 `LAST_FRAME1` — offset 0x1C — read-only

| Bits     | Field             | Description                                  |
| -------- | ----------------- | -------------------------------------------- |
| i (0..26) | bit (32+i) of frame | Raw bit at second 32+i.                    |
| 31..27   | reserved          | 0 (frame is 59 bits, second 59 is the minute mark and has no bit). |

Useful for debugging or for software that wants to apply its own
parity/plausibility logic.

### 6.9 `ENV_MIN` — offset 0x20 — read-only

| Bits   | Field      | Description                                                |
| ------ | ---------- | ---------------------------------------------------------- |
| 15..0  | `env_min`  | Top 16 b of the adaptive trough tracker (absenkung-level amplitude). |
| 31..16 | reserved   | 0.                                                         |

### 6.10 `NOISE` — offset 0x24 — read-only

| Bits   | Field   | Description                                                 |
| ------ | ------- | ----------------------------------------------------------- |
| 15..0  | `noise` | Top 16 b of the running mean-absolute-deviation of `env_pwr` from the currently-tracked level (env_max while carrier='1', env_min while carrier='0'). |
| 31..16 | reserved | 0.                                                         |

---

## 7. Operational notes

### 7.1 Reading the decoded time

1. Poll `STATUS` until `time_valid` (bit 2) is `1`.
2. Read `TIME_HMS`, `DATE`, and `FLAGS` in any order.

The decoded values correspond to the **most recently completed** minute
(i.e., the frame that ended at the most recent minute mark).  The
DCF77 protocol broadcasts the time of the **upcoming** minute during
the preceding 60 seconds — so the value the host reads at second N of
some minute M is the time of minute M itself (because the frame that
encoded minute M finished decoding at the start of minute M).

`time_valid` clears at the next minute mark and is re-asserted only if
the new frame passes parity and plausibility checks.  Between minute
marks the register values are stable.

### 7.2 Reset and enable semantics

* `s_axi_aresetn` resets the whole module (AXI slave + receiver core).
* `CTRL.soft_reset` (bit 1, write-only pulse) issues a one-clock
  synchronous reset to just the receiver core — the AXI slave keeps
  responding.  The bit reads back as `0` because the writemux
  self-clears it.
* `CTRL.enable` (bit 0, sticky) gates `adc_valid` into the pipeline.
  When cleared, the pipeline freezes its state; when re-asserted, it
  resumes from where it was.  Useful for pausing the decoder without a
  reset, e.g. while reconfiguring an upstream antenna multiplexer.

### 7.3 SNR estimation (host-side)

The host computes signal-to-noise ratio from three registers:

```
SNR_dB ≈ 20 · log10( (RSSI − ENV_MIN) / NOISE )
```

* `(RSSI − ENV_MIN)` is the modulation depth — the difference between
  the carrier-on and absenkung levels.  For a clean DCF77 signal this
  ratio is roughly `1 : 0.15`, i.e. modulation depth ≈ 85 % of the
  carrier amplitude.
* `NOISE` is the running mean absolute deviation of the envelope from
  whichever level the carrier hysteretic comparator currently believes
  it should be at.  A clean signal yields a very small `NOISE` value;
  a weak or interferer-corrupted signal makes it grow.

The estimator is updated continuously at the 1 kS/s env_valid rate, so
the registers are eventually-consistent.  Wait a few seconds after
gain or antenna changes for the IIRs to settle.

### 7.4 Sync-state machine

| `STATUS[1:0]` | Name    | Meaning                                                                                      |
| ------------- | ------- | -------------------------------------------------------------------------------------------- |
| `00`          | SEARCH  | No minute mark has been seen yet (or it has been more than `T_SEARCH_SEC` since the last one). The decoder is hunting for synchronisation. |
| `01`          | LOCK    | A minute mark has fired within the last `T_HOLD_SEC` seconds; the frame buffer is filling in step with broadcast seconds. |
| `10`          | HOLD    | The decoder was in LOCK but more than `T_HOLD_SEC` seconds have passed without a new minute mark.  Frames are still being decoded but their second-alignment is no longer guaranteed. |
| `11`          | reserved (not generated)                                                                                |

The state machine demotes monotonically (LOCK → HOLD → SEARCH) when
minute marks stop arriving and snaps back to LOCK on any successful
minute mark.

---

## 8. Resource and timing notes

The receiver fits comfortably under 1 % of an Artix-7 200:

| Resource | Approximate usage |
| -------- | ----------------- |
| LUTs     | ~1500             |
| FFs      | ~1600             |
| DSP48E1  | 1 (time-shared MAC for I² + Q²) |
| BRAM     | 0                  |

Only the 17-bit CIC₀ integrators sit on the unconstrained 125 MHz
single-cycle path.  All inner CIC stages and the bit_det / noise IIRs
get multicycle or false-path relief via `constraints/dcf77.xdc`.

The post-mixer adders (CIC₃ R=10, CIC₄ R=25) run at 25 kS/s and 1 kS/s
effective rates respectively; their wide (~30–33 b) integrators have
trivial timing margin.

---

## 9. Integration tips

* Compile order: `inc/` packages first
  (`dcf77_pkg.vhd`, `dcf77_cic_widths_pkg.vhd`, `axi4lite_interface_pkg.vhd`),
  then `src/dcf77/` in any order.  All files are VHDL-2008.
* Always apply `constraints/dcf77.xdc` (or its equivalent in your
  flow); without it Vivado will treat the inner-CIC adders as
  single-cycle paths and over-pessimise.
* The `iq_tap_out` port is an *output* of `dcf77_core`.  When wrapping
  inside your top-level you can either expose it as a separate set of
  ports (for a future PM correlator) or tie it off.  The signal is
  registered, so leaving it dangling is fine; the synthesis tool will
  remove unused logic.
* Re-running `simulation/scripts/cic_pruning.py` is only necessary
  when changing the CIC topology (R values, input width, output
  width).  The output `inc/dcf77_cic_widths_pkg.vhd` is checked in.
* For end-to-end validation against a real shortwave recording, see
  the **Replay test** section of `README.md` — it preprocesses a
  USB-demodulated baseband WAV (tuned to 72.5 kHz) into a binary
  stimulus file that drives the receiver back end.

---

## 10. Software pseudo-code

A minimal host driver looks roughly like:

```c
#define DCF77_BASE   /* mapping of the slave */

#define CTRL         (DCF77_BASE + 0x00)
#define STATUS       (DCF77_BASE + 0x04)
#define TIME_HMS     (DCF77_BASE + 0x08)
#define DATE         (DCF77_BASE + 0x0C)
#define FLAGS        (DCF77_BASE + 0x10)
#define RSSI         (DCF77_BASE + 0x14)
#define LAST_FRAME0  (DCF77_BASE + 0x18)
#define LAST_FRAME1  (DCF77_BASE + 0x1C)
#define ENV_MIN      (DCF77_BASE + 0x20)
#define NOISE        (DCF77_BASE + 0x24)

void dcf77_soft_reset(void)        { write(CTRL, 0x3); /* pulse */ }
void dcf77_enable(int en)          { write(CTRL, en & 1); }

int dcf77_time_valid(void)         { return (read(STATUS) >> 2) & 1; }
int dcf77_sync_state(void)         { return read(STATUS) & 0x3; }
int dcf77_bit_index(void)          { return (read(STATUS) >> 8) & 0x3F; }

struct dcf77_time {
    uint8_t year, month, day, weekday, hour, minute;
    uint8_t dst_active, dst_announce, leap_announce;
};

void dcf77_read_time(struct dcf77_time *t)
{
    uint32_t hms   = read(TIME_HMS);
    uint32_t date  = read(DATE);
    uint32_t flags = read(FLAGS);
    t->minute       =  hms       & 0x3F;
    t->hour         = (hms >> 8) & 0x1F;
    t->year         =  date        & 0xFF;
    t->month        = (date >> 8)  & 0x0F;
    t->day          = (date >> 16) & 0x1F;
    t->weekday      = (date >> 24) & 0x07;
    t->dst_active   =  flags       & 1;
    t->dst_announce = (flags >> 1) & 1;
    t->leap_announce= (flags >> 2) & 1;
}

double dcf77_snr_db(void)
{
    uint32_t r  = read(RSSI)    & 0xFFFF;
    uint32_t mn = read(ENV_MIN) & 0xFFFF;
    uint32_t n  = read(NOISE)   & 0xFFFF;
    if (n == 0 || r <= mn) return -INFINITY;
    return 20.0 * log10((double)(r - mn) / (double)n);
}
```

---

*Last updated to match the 5-stage CIC cascade (R=2·10·25 pre-mixer,
R=10·25 per-channel post-mixer, 1 kS/s AM stage) and the env_min /
noise-MAD SNR estimator.*
