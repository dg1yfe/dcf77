#!/usr/bin/env python3
"""Replay the PI loop in Python from a captured .pe.bin to expose
the integrator's time trajectory.

The HW PLL is deterministic: given the cordic_phase sequence, the
internal phase_filt / lock_iir / i_state / offset can be
reconstructed bit-exact in Python.

Outputs a PNG per stem with five panels:
    - cordic_phase (LSB, raw 12-b signed)
    - phase_filt (LSB, after ×2^11 and 20 Hz LPF)
    - lock_iir   (slow IIR of |phase_filt|, log-scale, with thresholds)
    - NCO offset (Hz)
    - lock_state (HW)

Usage:
    python3 simulation/scripts/analyze_pll_dynamics.py [--tmax T] <stem> [<stem>...]

    --tmax T   restrict x-axis to first T seconds (default: full capture)
"""
from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
import matplotlib.pyplot as plt

DERIVED = Path(__file__).resolve().parents[1] / "derived"

# HW constants (match dcf77_carrier_pll.vhd defaults from dcf77_core).
FS              = 2000.0
KP_SHIFT        = 10
KI_SHIFT        = 23
I_FRAC_BITS     = 10
LOOP_IIR_SHIFT  = 4
LOCK_IIR_SHIFT  = 14
ADAPTER_SHIFT   = 11   # phase_for_pll = -cordic_phase << 11
NCO_PHASE_W     = 25
OFFSET_W        = 24
ISTATE_W        = OFFSET_W + I_FRAC_BITS + 4

K_PRE           = KI_SHIFT - I_FRAC_BITS   # 13
NCO_FS_HZ       = 12000.0   # IF sample rate the NCO runs at
OFFSET_LSB_HZ   = NCO_FS_HZ / (1 << NCO_PHASE_W)  # ≈ 3.58e-4 Hz/LSB

# dcf77_phase_quality: saturating up/down counter on the low/high
# bin split at |cordic_phase| = 2^(PHASE_W-1) = 1024 LSB.
PQ_COUNTER_W   = 12
PQ_CMAX_POS    = (1 << (PQ_COUNTER_W - 1)) - 1
PQ_CMAX_NEG    = -(1 << (PQ_COUNTER_W - 1))
PQ_LOW_BOUND   = 1 << 10   # |cordic_phase| < PQ_LOW_BOUND → +1 step


def arith_shr(x: np.ndarray, n: int) -> np.ndarray:
    """Arithmetic right shift (floor division by 2^n)."""
    return np.floor_divide(x.astype(np.int64), 1 << n)


def sat(x: np.ndarray | int, width: int) -> np.ndarray | int:
    """Symmetric saturation to `width`-bit signed."""
    hi = (1 << (width - 1)) - 1
    lo = -(1 << (width - 1))
    return np.clip(x, lo, hi)


def read_pe_bin(path: Path) -> tuple[np.ndarray, np.ndarray]:
    raw = np.fromfile(path, dtype="<i2")
    phase = (raw & 0x0FFF).astype(np.int64)
    phase[phase >= 0x800] -= 0x1000
    lock = ((raw >> 15) & 1).astype(bool)
    return phase, lock


def replay_pll(cordic_phase: np.ndarray) -> dict:
    """Bit-exact Python replay of dcf77_carrier_pll's internal state."""
    n = len(cordic_phase)
    phase_for_pll = (-cordic_phase.astype(np.int64)) << ADAPTER_SHIFT

    phase_filt = np.zeros(n, dtype=np.int64)
    lock_iir   = np.zeros(n, dtype=np.int64)
    i_state    = np.zeros(n, dtype=np.int64)
    offset     = np.zeros(n, dtype=np.int64)

    pf = 0
    i  = 0
    # Phase-quality saturating ±1 counter (separate module, fed by
    # |cordic_phase|).  Resets to 0; +1 when |p|<PQ_LOW_BOUND else -1.
    pq_counter = 0
    for k in range(n):
        # 20 Hz LPF (carrier_pll, on phase_for_pll = -cordic_phase<<11)
        diff = phase_for_pll[k] - pf
        step = arith_shr(np.array(diff), LOOP_IIR_SHIFT).item()
        pf = pf + step
        phase_filt[k] = pf

        # Integral path
        i_step = arith_shr(np.array(pf), K_PRE).item()
        i = sat(i + i_step, ISTATE_W)
        i_state[k] = i

        # Output: (i_state >> I_FRAC_BITS) + (phase_filt >> KP_SHIFT)
        i_part = arith_shr(np.array(i), I_FRAC_BITS).item()
        p_path = arith_shr(np.array(pf), KP_SHIFT).item()
        offset[k] = sat(i_part + p_path, OFFSET_W)

        # Phase-quality module: ±1 saturating counter on the low/high
        # bin split.  See dcf77_phase_quality.vhd.
        if int(abs(cordic_phase[k])) < PQ_LOW_BOUND:
            if pq_counter < PQ_CMAX_POS:
                pq_counter += 1
        else:
            if pq_counter > PQ_CMAX_NEG:
                pq_counter -= 1
        lock_iir[k] = pq_counter

    nco_df_hz = offset * OFFSET_LSB_HZ

    return {
        "phase_filt": phase_filt,
        "lock_iir":   lock_iir,
        "i_state":    i_state,
        "offset":     offset,
        "nco_df_hz":  nco_df_hz,
    }


def plot_stem(stem: str, ax_path: Path, tmax: float | None) -> None:
    pe = DERIVED / f"{stem}.pe.bin"
    phase, lock_hw = read_pe_bin(pe)
    state = replay_pll(phase)

    t = np.arange(len(phase)) / FS

    fig, axes = plt.subplots(5, 1, figsize=(13, 11), sharex=True)

    axes[0].plot(t, phase, lw=0.4, color="C0")
    axes[0].set_ylabel("cordic_phase\n(LSB, ±2048 = ±π)")
    axes[0].axhline( 177, color="grey", ls=":", lw=0.6,
                    label="±PSK (±15.6°)")
    axes[0].axhline(-177, color="grey", ls=":", lw=0.6)
    axes[0].axhline(0,    color="black", lw=0.4)
    axes[0].legend(loc="upper right", fontsize=8)
    axes[0].grid(True, alpha=0.3)

    axes[1].plot(t, state["phase_filt"], lw=0.5, color="C1")
    axes[1].axhline(0, color="black", lw=0.4)
    axes[1].set_ylabel("phase_filt\n(LSB, 20 Hz LPF)")
    axes[1].grid(True, alpha=0.3)

    axes[2].plot(t, state["lock_iir"], lw=0.7, color="C5")
    axes[2].axhline(+1024, color="green", ls="--", lw=0.8, label="LOCK_IN (+1024)")
    axes[2].axhline(-512,  color="red",   ls="--", lw=0.8, label="LOCK_OUT (-512)")
    axes[2].axhline(0, color="black", lw=0.4)
    axes[2].set_ylabel("pq counter (sat ±2047)\nlow-bin vs high-bin balance")
    axes[2].set_ylim(PQ_CMAX_NEG * 1.1, PQ_CMAX_POS * 1.1)
    axes[2].legend(loc="upper right", fontsize=8)
    axes[2].grid(True, alpha=0.3)

    axes[3].plot(t, state["nco_df_hz"], lw=0.7, color="C3")
    axes[3].set_ylabel("NCO offset\n(Hz, fs_NCO×offset/2^25)")
    axes[3].axhline(0, color="black", lw=0.4)
    axes[3].grid(True, alpha=0.3)

    axes[4].plot(t, lock_hw.astype(int), lw=0.7, color="C4",
                 drawstyle="steps-post")
    axes[4].set_ylabel("pll_lock\n(HW)")
    axes[4].set_ylim(-0.1, 1.1)
    axes[4].set_xlabel("time (s)")
    axes[4].grid(True, alpha=0.3)

    if tmax is not None:
        axes[-1].set_xlim(0, tmax)

    sigma  = float(phase.std())
    second_half = state["nco_df_hz"][int(len(phase) * 0.5):]
    mean_df  = float(second_half.mean())
    li_final = float(state["lock_iir"][-1])
    fig.suptitle(
        f"{stem}\n"
        f"σ(cordic_phase) = {sigma:.0f} LSB    "
        f"NCO Δf (2nd-half mean) = {mean_df:+.3f} Hz    "
        f"pq counter final = {li_final:+.0f}",
        fontsize=11)
    fig.tight_layout()
    fig.savefig(ax_path, dpi=120)
    plt.close(fig)
    print(f"[{stem}] σ={sigma:.0f}  Δf={mean_df:+.3f} Hz  "
          f"pq counter={li_final:+.0f}  -> {ax_path}")


def main(argv: list[str]) -> int:
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument("--tmax", type=float, default=None,
                    help="restrict x-axis to first T seconds")
    ap.add_argument("stems", nargs="+")
    args = ap.parse_args(argv)

    suffix = f".pll_dyn_{int(args.tmax)}s.png" if args.tmax else ".pll_dyn.png"
    for stem in args.stems:
        pe = DERIVED / f"{stem}.pe.bin"
        if not pe.exists():
            print(f"missing: {pe}", file=sys.stderr)
            continue
        plot_stem(stem, DERIVED / f"{stem}{suffix}", args.tmax)
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
