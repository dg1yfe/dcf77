#!/usr/bin/env python3
"""One-line-per-recording summary of phase_err captures.

Reads every .pe.bin in simulation/derived/ and prints:

    recording                              lock%   σ    mean|p|   Δf(Hz)  wraps/s

Δf is derived from the wrap count: each phase wrap is one full
±2π = 2^12 LSB rotation in 1/fs seconds; signed by majority direction.
Useful for spotting the loop wandering at a wrong-frequency equilibrium.
"""
from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

DERIVED = Path(__file__).resolve().parents[1] / "derived"
FS = 2000.0   # 2 kS/s baseband sample rate
LSB_FULL = 4096  # 12-b CORDIC phase range


def read_pe_bin(path: Path) -> tuple[np.ndarray, np.ndarray]:
    raw = np.fromfile(path, dtype="<i2")
    phase = (raw & 0x0FFF).astype(np.int32)
    phase[phase >= 0x800] -= 0x1000
    lock = ((raw >> 15) & 1).astype(bool)
    return phase, lock


def stats(path: Path, stem: str) -> str:
    phase, lock = read_pe_bin(path)
    if len(phase) < 100:
        return f"{stem:55s}  short capture ({len(phase)} samples)"

    # Drop first 4 s to skip the startup transient (LPF / integrator warmup).
    warm = min(8000, len(phase) // 4)
    p = phase[warm:]

    lock_frac = float(lock[warm:].mean())
    sigma = float(p.std())
    mean_signed = float(p.mean())
    mean_abs = float(np.abs(p).mean())

    # Δf from cordic_phase alone is unreliable when σ is comparable to
    # π — both np.unwrap and wrap-corrected derivatives mistake noise
    # for cycle slips.  Use analyze_pll_dynamics.py for true NCO Δf.

    return (f"{stem:55s}  "
            f"{lock_frac*100:5.1f}%  "
            f"σ={sigma:5.0f}  "
            f"|p|={mean_abs:5.0f}  "
            f"mean={mean_signed:+6.1f}")


def main(argv: list[str]) -> int:
    if argv:
        targets = [DERIVED / f"{s}.pe.bin" for s in argv]
    else:
        targets = sorted(DERIVED.glob("*.pe.bin"))

    if not targets:
        print("no .pe.bin files found", file=sys.stderr)
        return 1

    print(f"{'recording':55s}  lock%   σ       mean|p|  mean(p)")
    print("-" * 100)
    for p in targets:
        if not p.exists():
            print(f"missing: {p}", file=sys.stderr)
            continue
        stem = p.name[:-len(".pe.bin")]
        print(stats(p, stem))
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
