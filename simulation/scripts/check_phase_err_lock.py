#!/usr/bin/env python3
"""Sanity check: does pll_lock actually mean the carrier PLL has locked?

When the carrier PLL is genuinely tracking, the CORDIC phase output
should be quasi-stationary:
    mean ≈ 0
    σ    ≈ PSK_deviation (±177 LSB) + thermal noise  ≲ 400 LSB

A phase signal that is *uniformly distributed* over [-π, π] has
    σ = 2^12 / √12 ≈ 1183 LSB

So a locked-only-σ near or above 600 is a strong sign that the
"lock" indicator is mis-asserting -- the loop is wrapping or
random-walking rather than tracking.

This script reads every .pe.bin in simulation/derived/ (one row per
recording) and asserts:

    1. pll_lock fraction >= MIN_LOCK_FRACTION
    2. when pll_lock=1, σ(cordic_phase) < MAX_LOCKED_STD
    3. when pll_lock=1, |mean(cordic_phase)| < MAX_LOCKED_MEAN

If any check fails, exit 1 with a printed report.  Intended for CI
integration so a regression that breaks the PLL lock detector or
loop tuning is caught the next time we run the capture pass.

Thresholds are deliberately permissive to start (no false alarms on
the current corpus *once the PLL is actually fixed*) -- tighten as
the design matures.
"""
from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

DERIVED = Path(__file__).resolve().parents[1] / "derived"

# Thresholds.  Tune as the design improves.
MIN_LOCK_FRACTION = 0.70    # >= 70% of samples must be in pll_lock='1'
MAX_LOCKED_STD    = 400     # σ of cordic_phase when locked must be < 400 LSB
MAX_LOCKED_MEAN   = 200     # |mean| of cordic_phase when locked < 200 LSB
PSK_DEV_LSB       = 177     # ±15.6° at 2π = 2^12 LSB
UNIFORM_STD       = 1183    # σ if phase is uniformly distributed (sanity ref)


def read_pe_bin(path: Path) -> tuple[np.ndarray, np.ndarray]:
    raw = np.fromfile(path, dtype="<i2")
    phase = (raw & 0x0FFF).astype(np.int32)
    phase[phase >= 0x800] -= 0x1000
    lock = ((raw >> 15) & 1).astype(bool)
    return phase, lock


def check_one(path: Path) -> tuple[bool, list[str]]:
    """Return (ok, list_of_failure_messages)."""
    phase, lock = read_pe_bin(path)
    stem = path.name[:-len(".pe.bin")]
    errs: list[str] = []

    lock_frac = float(lock.mean())
    if lock_frac < MIN_LOCK_FRACTION:
        errs.append(f"pll_lock fraction {lock_frac:.3f} "
                    f"< {MIN_LOCK_FRACTION} (carrier never settled)")

    if lock.any():
        p_locked = phase[lock]
        s = float(p_locked.std())
        m = float(abs(p_locked.mean()))

        if s >= MAX_LOCKED_STD:
            ratio = s / UNIFORM_STD
            hint = (f"σ/uniform_σ = {ratio:.2f} -- "
                    + ("loop wraps or random-walks (lock indicator is bogus)"
                       if ratio > 0.5
                       else "loop noisy, but not full-range"))
            errs.append(
                f"locked-phase σ {s:.0f} >= {MAX_LOCKED_STD} LSB ({hint})")

        if m >= MAX_LOCKED_MEAN:
            errs.append(f"locked-phase |mean| {m:.0f} >= {MAX_LOCKED_MEAN} LSB "
                        f"(static phase offset -- NCO bias?)")

    return (len(errs) == 0), errs


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

    print(f"thresholds: lock_frac >= {MIN_LOCK_FRACTION}   "
          f"locked σ < {MAX_LOCKED_STD}   "
          f"|locked mean| < {MAX_LOCKED_MEAN}")
    print(f"references: PSK_deviation = ±{PSK_DEV_LSB} LSB   "
          f"uniform [-π,π] σ = {UNIFORM_STD}")
    print()

    failed: list[str] = []
    for path in targets:
        stem = path.name[:-len(".pe.bin")]
        ok, errs = check_one(path)
        status = "PASS" if ok else "FAIL"
        print(f"[{status}] {stem}")
        for e in errs:
            print(f"          - {e}")
        if not ok:
            failed.append(stem)

    print()
    if failed:
        print(f"FAILED: {len(failed)} of {len(targets)} recordings")
        return 1
    print(f"PASS: all {len(targets)} recordings")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
