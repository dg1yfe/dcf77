#!/usr/bin/env python3
"""Hogenauer width / SNR computation for the DCF77 receiver's 5-stage CIC.

Emits:
  inc/dcf77_cic_widths_pkg.vhd   — VHDL-2008 constants package consumed by RTL.
  simulation/derived/cic_widths.txt — human-readable report.

The five CIC stages (uniform N=3, M=1) are:
    CIC0  R=2   @125 MHz    -- only stage at full clock rate
    CIC1  R=10  @62.5 MHz   -- clock-enabled (multicycle 2)
    CIC2  R=25  @6.25 MHz   -- clock-enabled (multicycle 20)
        [ sign-LO mixer here -- splits real into I and Q ]
    CIC3  R=10  @250 kHz    -- clock-enabled (multicycle 500), per channel
    CIC4  R=25  @25 kHz     -- clock-enabled (multicycle 5000), per channel

The complex post-mixer stages (CIC3 / CIC4) are emitted once each — the
generated package's CIC3_* and CIC4_* constants are used for BOTH the
I-channel and Q-channel instances inside dcf77_core.

For each stage we compute the L2 error-gain F_j of every truncation point
(integrators 1..N, combs 1..N, plus a stage output truncation) following
Hogenauer (1981).  Bits dropped at stage j are then

    B_j = floor( log2(sigma_out_target) - log2(F_j) - 0.5*log2(2N+1) )

where sigma_out_target = 1 stage-output LSB (so the kept stage output is
unit-LSB faithful).  The cascade is treated stage-by-stage with
INTER_STAGE_MARGIN_BITS extra headroom at each inter-stage truncation; with
margin=0 (the current setting) every intermediate stage output is 22 b,
exactly matching the existing iq_word_t.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path


def convolve(a: list[float], b: list[float]) -> list[float]:
    """Plain 1-D linear convolution -- no numpy needed for these tiny vectors."""
    out = [0.0] * (len(a) + len(b) - 1)
    for i, ai in enumerate(a):
        if ai == 0.0:
            continue
        for j, bj in enumerate(b):
            out[i + j] += ai * bj
    return out


def l2_norm(xs: list[float]) -> float:
    return math.sqrt(sum(x * x for x in xs))


# --------------------------------------------------------------------------- #
# Configuration                                                                #
# --------------------------------------------------------------------------- #

INPUT_WIDTH = 14    # signed ADC sample width (LSB position 0)
OUTPUT_WIDTH = 22   # signed output width of the whole cascade
N_STAGES_FILT = 3   # integrators / combs per CIC
M = 1               # comb differential delay
INTER_STAGE_MARGIN_BITS = 0   # 0 = intermediate outputs collapse to OUTPUT_WIDTH
                               # (matches iq_word_t = sfixed(21 downto 0))

ROOT = Path(__file__).resolve().parent.parent.parent
INC_PATH = ROOT / "inc" / "dcf77_cic_widths_pkg.vhd"
REPORT_PATH = ROOT / "simulation" / "derived" / "cic_widths.txt"


@dataclass
class StageSpec:
    name: str
    R: int
    in_rate_hz: float
    in_width_override: int | None = None   # None ⇒ inherit from previous stage
    out_width_override: int | None = None  # None ⇒ default sizing rule


STAGES: list[StageSpec] = [
    # Real-valued pre-mixer chain: 125 MHz -> 250 kS/s.
    StageSpec("cic0", R=2,  in_rate_hz=125_000_000),
    StageSpec("cic1", R=10, in_rate_hz= 62_500_000),
    StageSpec("cic2", R=25, in_rate_hz=  6_250_000),
    # Complex post-mixer chain (instantiated twice, once per I/Q channel):
    # 250 kS/s -> 1 kS/s.
    StageSpec("cic3", R=10, in_rate_hz=    250_000),
    StageSpec("cic4", R=25, in_rate_hz=     25_000),
    # Envelope decimator (after I^2 + Q^2 in dcf77_env):
    # 1 kS/s -> 100 S/s.  Input width = env_word_t total bits (26).
    StageSpec("cic_env", R=10, in_rate_hz=1_000,
              in_width_override=26, out_width_override=26),
]


# --------------------------------------------------------------------------- #
# Hogenauer math                                                               #
# --------------------------------------------------------------------------- #

def compute_Fj(N: int, R: int, M: int) -> list[float]:
    """L2 norm of the impulse response from truncation point j to the stage
    output, for j = 1..2N+1.  Indexing:
        j = 1..N        -> truncation at integrator j (input rate)
        j = N+1..2N     -> truncation at comb (j-N) (output rate)
        j = 2N+1        -> stage output truncation (no further filtering)
    """
    F: list[float] = []
    box = [1.0] * (R * M)

    # Integrators 1..N: noise propagates through (N-j+1) more integrators,
    # then decimation by R, then N combs.  Removing (j-1) integrators from the
    # head of the cascade gives a CIC with (N-j+1) integrators and the same R,
    # M.  Its input-rate impulse response is the (N-j+1)-fold convolution of an
    # (R*M)-box; we keep every R'th sample to model the decimation.
    for j in range(1, N + 1):
        h: list[float] = [1.0]
        for _ in range(N - j + 1):
            h = convolve(h, box)
        h_out = h[::R]
        F.append(l2_norm(h_out))

    # Combs (j-N) for j=N+1..2N: noise propagates through (2N-j+1) more combs
    # at the OUTPUT rate.  Each comb has impulse response [1, 0..0, -1] with
    # (M-1) zeros in between (length M+1).
    comb_h = [0.0] * (M + 1)
    comb_h[0] = 1.0
    comb_h[-1] = -1.0
    for jk in range(1, N + 1):
        remaining = N - jk + 1
        h = [1.0]
        for _ in range(remaining):
            h = convolve(h, comb_h)
        F.append(l2_norm(h))

    # Output truncation: no further filtering.
    F.append(1.0)
    return F


def stage_widths(input_width: int, N: int, R: int, M: int,
                 stage_out_width: int) -> dict:
    """Hogenauer pruning for a single CIC stage.

    The stage takes `input_width` bits at its input LSB and produces
    `stage_out_width` bits at its output.  Hogenauer's allowance at truncation
    point j is the largest LSB position B_j (measured from the input LSB) such
    that this truncation alone contributes at most 1/sqrt(2N+1) of one stage-
    output LSB of noise sigma:

        B_j  <= B_out - log2(F_j) - 0.5*log2(2N+1)

    where B_out is the LSB position of the kept output (= natural_width -
    stage_out_width).
    """
    F = compute_Fj(N, R, M)
    natural_width = input_width + N * int(math.ceil(math.log2(R * M)))
    B_out = natural_width - stage_out_width
    norm = 0.5 * math.log2(2 * N + 1)

    drops: list[int] = []
    for fj in F:
        bj_real = B_out - math.log2(max(fj, 1e-12)) - norm
        bj = math.floor(bj_real)
        drops.append(max(bj, 0))

    # Per-truncation-point kept widths (LSB position B_j -> kept = natural - B_j).
    integrator_kept = [natural_width - drops[j] for j in range(N)]
    comb_kept       = [natural_width - drops[N + j] for j in range(N)]
    output_kept     = stage_out_width

    return {
        "F": F,
        "B_out": B_out,
        "natural_width": natural_width,
        "drops": drops,
        "integrator_kept": integrator_kept,
        "comb_kept": comb_kept,
        "output_kept": output_kept,
    }


# --------------------------------------------------------------------------- #
# Cascade analysis                                                             #
# --------------------------------------------------------------------------- #

def analyse_cascade() -> dict:
    """Walk the cascade stage-by-stage, sizing per-stage widths so the final
    output cleanly carries OUTPUT_WIDTH signed bits with truncation noise well
    below the ADC quantisation noise referred to the output."""

    total_R = 1
    for s in STAGES:
        total_R *= s.R
    full_output_width = INPUT_WIDTH + N_STAGES_FILT * int(math.ceil(math.log2(total_R * M)))

    # Per-stage target output widths.  Stage 0 passes its full natural width on
    # (no point throwing away the modest 3 b of growth).  Stages 1..N-1 narrow
    # the path back down to OUTPUT_WIDTH, with at least INTER_STAGE_MARGIN_BITS
    # of headroom so accumulated truncation noise at the cascade output is well
    # below 1 LSB at OUTPUT_WIDTH.
    stage_out_widths: list[int] = []
    last_idx = len(STAGES) - 1
    for idx, spec in enumerate(STAGES):
        if spec.in_width_override is not None:
            in_width = spec.in_width_override
        elif idx == 0:
            # Stage 0 has only 3 b of growth from a 14 b input -> 17 b.  Keep all.
            in_width = INPUT_WIDTH
        else:
            in_width = stage_out_widths[-1]
        natural = in_width + N_STAGES_FILT * int(math.ceil(math.log2(spec.R * M)))
        if spec.out_width_override is not None:
            out_w = min(spec.out_width_override, natural)
        elif idx == last_idx:
            out_w = OUTPUT_WIDTH
        elif idx == 0:
            out_w = natural   # carry all bits to stage 1
        else:
            out_w = OUTPUT_WIDTH + INTER_STAGE_MARGIN_BITS
            out_w = min(out_w, natural)
        stage_out_widths.append(out_w)

    stage_in_width = INPUT_WIDTH
    stages_info = []
    for idx, spec in enumerate(STAGES):
        if spec.in_width_override is not None:
            stage_in_width = spec.in_width_override
        stage_full = stage_in_width + N_STAGES_FILT * int(math.ceil(math.log2(spec.R * M)))
        out_target = stage_out_widths[idx]

        widths = stage_widths(stage_in_width, N_STAGES_FILT, spec.R, M, out_target)

        int_w  = widths["integrator_kept"]
        comb_w = widths["comb_kept"]
        out_w  = widths["output_kept"]

        # Uniform per-stage widths: take the MAX across truncation points so the
        # widest (most-MSB-sensitive) stage in that group governs the actual
        # bit count.  RTL stays cleanly uniform per stage.
        int_uniform  = max(int_w)
        comb_uniform = max(comb_w)

        stages_info.append({
            "name": spec.name,
            "R": spec.R,
            "natural_width": stage_full,
            "in_width": stage_in_width,
            "integrator_widths": int_w,
            "comb_widths": comb_w,
            "integrator_uniform": int_uniform,
            "comb_uniform": comb_uniform,
            "out_width": out_w,
            "Fj": widths["F"],
            "drops": widths["drops"],
            "B_out": widths["B_out"],
        })

        stage_in_width = out_w

    return {
        "stages": stages_info,
        "total_R": total_R,
        "full_output_width": full_output_width,
        "final_lsb_pos": full_output_width - OUTPUT_WIDTH,
    }


# --------------------------------------------------------------------------- #
# Code emission                                                                #
# --------------------------------------------------------------------------- #

VHDL_HEADER = """-- =========================================================================
-- dcf77_cic_widths_pkg.vhd
--
-- AUTO-GENERATED by simulation/scripts/cic_pruning.py -- DO NOT EDIT BY HAND.
--
-- Per-stage CIC integrator / comb / output widths for the cascaded decimator.
-- Widths follow a Hogenauer-style pruning analysis; see the generator script
-- for the math.  All widths are signed (sfixed) and refer to the total bit
-- count of each carrier word.
--
-- CIC0..CIC2 are real-valued (pre-mixer); CIC3..CIC4 are complex (instantiated
-- twice each, once per I/Q channel after the sign-LO mixer).
-- =========================================================================

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

package dcf77_cic_widths_pkg is

"""

VHDL_FOOTER = "\nend package dcf77_cic_widths_pkg;\n"


def emit_vhdl(report: dict) -> str:
    out = [VHDL_HEADER]
    out.append(f"    constant CIC_INPUT_WIDTH  : natural := {INPUT_WIDTH};\n")
    out.append(f"    constant CIC_OUTPUT_WIDTH : natural := {OUTPUT_WIDTH};\n")
    out.append(f"    constant CIC_N            : natural := {N_STAGES_FILT};\n")
    out.append(f"    constant CIC_M            : natural := {M};\n")
    out.append(f"    constant CIC_TOTAL_R      : natural := {report['total_R']};\n")
    out.append(f"    constant CIC_FULL_OUT_W   : natural := {report['full_output_width']};\n")
    out.append("\n")

    for s in report["stages"]:
        prefix = s["name"].upper()
        out.append(f"    -- {s['name']}: R = {s['R']}\n")
        out.append(f"    constant {prefix}_R           : natural := {s['R']};\n")
        out.append(f"    constant {prefix}_IN_W        : natural := {s['in_width']};\n")
        out.append(f"    constant {prefix}_INT_W       : natural := {s['integrator_uniform']};\n")
        out.append(f"    constant {prefix}_COMB_W      : natural := {s['comb_uniform']};\n")
        out.append(f"    constant {prefix}_OUT_W       : natural := {s['out_width']};\n")
        out.append("\n")

    out.append(VHDL_FOOTER)
    return "".join(out)


def emit_report(report: dict) -> str:
    lines = []
    lines.append("DCF77 CIC cascade — Hogenauer width report")
    lines.append("=" * 60)
    lines.append(f"Total decimation : {report['total_R']}")
    lines.append(f"Input width      : {INPUT_WIDTH} b signed")
    lines.append(f"Output width     : {OUTPUT_WIDTH} b signed")
    lines.append(f"Full nat. output : {report['full_output_width']} b")
    lines.append(f"Final LSB pos.   : 2^{report['final_lsb_pos']} below cascade FS")
    lines.append("")
    for s in report["stages"]:
        lines.append(f"--- {s['name']} (R={s['R']}) ---")
        lines.append(f"  natural width      : {s['natural_width']} b")
        lines.append(f"  input width        : {s['in_width']} b")
        lines.append(f"  integrator widths  : {s['integrator_widths']}  -> uniform {s['integrator_uniform']} b")
        lines.append(f"  comb widths        : {s['comb_widths']}  -> uniform {s['comb_uniform']} b")
        lines.append(f"  output width       : {s['out_width']} b")
        lines.append(f"  F_j (1..2N+1)      : " +
                     ", ".join(f"{fj:.3g}" for fj in s["Fj"]))
        lines.append(f"  drops              : {s['drops']}")
        lines.append("")
    return "\n".join(lines) + "\n"


# --------------------------------------------------------------------------- #
# Entry point                                                                  #
# --------------------------------------------------------------------------- #

def main() -> int:
    report = analyse_cascade()
    INC_PATH.parent.mkdir(parents=True, exist_ok=True)
    REPORT_PATH.parent.mkdir(parents=True, exist_ok=True)
    INC_PATH.write_text(emit_vhdl(report))
    REPORT_PATH.write_text(emit_report(report))
    print(f"wrote {INC_PATH}")
    print(f"wrote {REPORT_PATH}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
