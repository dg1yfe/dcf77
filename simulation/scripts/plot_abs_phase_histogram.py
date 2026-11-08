#!/usr/bin/env python3
"""Per-recording histogram of |cordic_phase| (N-bin, equal-width).

Each recording's |cordic_phase| time series (post 4 s warmup) is binned
into N equal-width bins on [0, 2048] LSB.  One subplot per recording.

Usage:
    plot_abs_phase_histogram.py [--nbins N] [<stem> ...]
"""
from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
import matplotlib.pyplot as plt

DERIVED = Path(__file__).resolve().parents[1] / "derived"
FS_BB    = 2000.0
WARMUP_S = 4.0
RANGE_LSB = 2048   # |cordic_phase| ∈ [0, 2048] = [0, π] on 12-b CORDIC


def read_pe_bin(path: Path) -> np.ndarray:
    raw = np.fromfile(path, dtype="<i2")
    phase = (raw & 0x0FFF).astype(np.int64)
    phase[phase >= 0x800] -= 0x1000
    return phase


def plot_corpus(stems: list[str], out_path: Path, nbins: int) -> None:
    n = len(stems)
    fig, axes = plt.subplots(n, 1, figsize=(11, 1.7 * n),
                             sharex=True, constrained_layout=True)
    if n == 1:
        axes = [axes]

    for ax, stem in zip(axes, stems):
        p = DERIVED / f"{stem}.pe.bin"
        if not p.exists():
            ax.text(0.5, 0.5, f"missing: {stem}",
                    transform=ax.transAxes, ha="center")
            continue
        phase = read_pe_bin(p)
        skip  = int(WARMUP_S * FS_BB)
        ap    = np.abs(phase[skip:])
        counts, edges = np.histogram(np.clip(ap, 0, RANGE_LSB),
                                     bins=nbins, range=(0, RANGE_LSB))
        centers = 0.5 * (edges[:-1] + edges[1:])
        widths  = np.diff(edges)
        # Noise-reference recordings get a distinct colour.
        color = "tab:red" if "noise" in stem else "tab:blue"
        ax.bar(centers, counts, width=widths * 0.92,
               color=color, edgecolor="black", linewidth=0.4)

        sigma = float(phase[skip:].std())
        mp    = float(ap.mean())
        ax.set_ylabel(f"{stem[:25]}\n"
                      f"σ={sigma:.0f} |p|={mp:.0f}", fontsize=8)
        ax.set_xlim(0, RANGE_LSB)
        ax.grid(True, alpha=0.3, axis="y")
        ax.tick_params(labelsize=8)

    axes[-1].set_xlabel("|cordic_phase|  (LSB, ±2048 = ±π)")
    fig.suptitle(f"|cordic_phase| histogram per recording ({nbins} bins)",
                 fontsize=11)
    fig.savefig(out_path, dpi=120)
    plt.close(fig)
    print(f"wrote {out_path}")


def main(argv: list[str]) -> int:
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument("--nbins", type=int, default=20,
                    help="number of equal-width bins (default 20)")
    ap.add_argument("stems", nargs="*")
    args = ap.parse_args(argv)

    if args.stems:
        stems = args.stems
    else:
        stems = [
            "dcf_strong",
            "synth",
            "dcf_3_1",
            "dcf_3_6",
            "dcf_3_4",
            "dcf_3_3",
            "dcf_3_7",
            "dcf_3_5",
            # Noise reference: dcf_3_5 mixed against 1750 Hz NCO (signal-
            # free part of the IF spectrum).  Distribution should be
            # essentially uniform on [0, 2048].
            "dcf_3_5.noise1750",
        ]
    out = DERIVED / f"abs_phase_histogram_n{args.nbins}.png"
    plot_corpus(stems, out, args.nbins)
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
