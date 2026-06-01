#!/usr/bin/env python3
"""Render SVG plots of captured phase_err streams (.pe.bin files).

File format (from 16_dcf77_phase_err_capture_tb): 2 bytes int16 little-
endian per baseband sample (= 2 kS/s).
    bits 11:0  : signed cordic_phase (12 b)
    bits 14:12 : reserved (zero)
    bit  15    : pll_lock

For each .pe.bin in simulation/derived/, write
simulation/derived/<stem>.pe.svg containing:
  * phase_err time-series over the recording (subsampled if long)
  * a 5-second zoom panel
  * pll_lock state strip

Usage:
    python3 simulation/scripts/plot_phase_err.py [stem ...]

With no args, plot every .pe.bin in simulation/derived/.
"""
from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

BASEBAND_HZ = 2000.0
DERIVED = Path(__file__).resolve().parents[1] / "derived"


def read_pe_bin(path: Path) -> tuple[np.ndarray, np.ndarray]:
    raw = np.fromfile(path, dtype="<i2")
    # phase_err = bits 11:0 sign-extended; pll_lock = bit 15
    phase = (raw & 0x0FFF).astype(np.int16)
    phase[phase >= 0x800] -= 0x1000        # sign-extend 12-bit
    lock = ((raw >> 15) & 1).astype(np.uint8)
    return phase, lock


def plot_one(stem: str, phase: np.ndarray, lock: np.ndarray, out: Path) -> None:
    n = len(phase)
    t = np.arange(n) / BASEBAND_HZ   # broadcast-time seconds

    fig, ax = plt.subplots(2, 1, figsize=(16, 8),
                           gridspec_kw=dict(height_ratios=[6, 0.5]),
                           sharex=True)

    # Full-resolution phase_err -- every sample, no subsampling.  Zoom
    # in arbitrarily in the SVG to see chip transitions.
    ax[0].plot(t, phase, lw=0.3, color="#1f77b4")
    ax[0].set_title(f"{stem} -- phase_err  ({n:,} samples, "
                    f"{n/BASEBAND_HZ:.1f} broadcast-s @ {BASEBAND_HZ/1000:.1f} kS/s)")
    ax[0].set_ylabel("phase_err [LSB, 2π / 2¹²]")
    ax[0].grid(alpha=0.3)
    ax[0].set_xlim(0, t[-1])

    # pll_lock strip across the full recording
    ax[1].fill_between(t, 0, lock.astype(float), step="post",
                       color="#2ca02c", alpha=0.7, linewidth=0)
    ax[1].set_ylim(-0.1, 1.1)
    ax[1].set_yticks([0, 1])
    ax[1].set_yticklabels(["unlk", "lock"])
    ax[1].set_xlabel("broadcast time [s]")
    ax[1].set_xlim(0, t[-1])
    ax[1].grid(alpha=0.3)

    # Quick stats annotation
    locked_frac = float(lock.mean())
    p_min, p_max = int(phase.min()), int(phase.max())
    p_std = float(phase.std())
    fig.text(0.99, 0.985,
             f"samples = {n:,}   |   "
             f"pll_lock fraction = {locked_frac:.3f}   |   "
             f"phase_err [min, max] = [{p_min}, {p_max}]   |   "
             f"σ = {p_std:.1f}",
             ha="right", va="top", fontsize=9, family="monospace")

    plt.tight_layout(rect=[0, 0, 1, 0.97])
    plt.savefig(out, format="svg")
    plt.close(fig)


def main(argv: list[str]) -> int:
    targets: list[Path]
    if argv:
        targets = [DERIVED / f"{s}.pe.bin" for s in argv]
        missing = [p for p in targets if not p.exists()]
        if missing:
            for p in missing:
                print(f"missing: {p}", file=sys.stderr)
            return 1
    else:
        targets = sorted(DERIVED.glob("*.pe.bin"))
        if not targets:
            print("no .pe.bin files in simulation/derived/", file=sys.stderr)
            return 1

    for path in targets:
        stem = path.name[:-len(".pe.bin")]
        out = path.with_suffix("").with_suffix(".pe.svg")
        phase, lock = read_pe_bin(path)
        plot_one(stem, phase, lock, out)
        print(f"{out.name}  ({len(phase):,} samples, "
              f"pll_lock {lock.mean()*100:.1f}%)")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
