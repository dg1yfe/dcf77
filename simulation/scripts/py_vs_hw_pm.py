#!/usr/bin/env python3
"""Compare Python-side PM decoder vs HW PM correlator on a recording.

Loads a .if.s16, mixes to complex baseband at the recording's IF,
takes the phase, runs the dcf77_pm_decode.pm_soft_per_second() at the
sweep-optimum chip0 offset, then computes the SIK signature match
against the same template the HW would use.

This isolates whether the ~25-30 % per-bit error ceiling we see on
the new corpus is intrinsic to the recording (= Python also struggles)
or a HW chain artefact (= Python decodes cleanly).
"""
from __future__ import annotations

import argparse
import re
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from dcf77_pm_decode import (
    pm_soft_per_second, find_chip0_offset,
    CHIP0_OFFSET_NOMINAL, SECOND_SAMPLES,
)


def load_if(path: Path):
    raw = np.fromfile(path, dtype=np.int16).astype(np.float64)
    meta_path = path.with_suffix(".meta")
    meta = {}
    if meta_path.exists():
        for line in meta_path.read_text().splitlines():
            if "=" in line:
                k, v = line.split("=", 1)
                meta[k.strip()] = v.strip()
    fs   = float(meta.get("sample_rate_hz", 12_000))
    f_if = float(meta.get("if_hz", 5_000))
    return raw, fs, f_if


def mix_and_phase(raw, fs, f_if, fs_out=2000.0):
    """Mix to baseband and decimate to fs_out, return wrapped phase."""
    n = np.arange(len(raw), dtype=np.float64)
    lo = np.exp(-1j * 2 * np.pi * f_if * n / fs)
    bb = raw * lo
    decim = int(round(fs / fs_out))
    n_out = len(bb) // decim
    bb = bb[: n_out * decim].reshape(n_out, decim).mean(axis=1)
    # CORDIC-equivalent: wrapped atan2.  No de-trend or PLL — keeps it
    # comparable to what the HW CORDIC sees.
    phase = np.angle(bb)
    return phase, fs_out


def best_sik_score(signs):
    """Return (best_pct, sign_flip, offset, signature_score, total)."""
    obs = np.array(signs)
    best = (-1.0, +1, 0, 0, 0)
    for flip in (+1, -1):
        for off in range(60):
            arr = np.zeros(len(obs), dtype=int)
            for k in range(len(obs)):
                sec = (off + k) % 60
                if sec <= 9:                       arr[k] = +1
                elif sec <= 14 or sec == 59:       arr[k] = -1
            mask = arr != 0
            sc  = int(np.sum(obs[mask] == flip * arr[mask]))
            tot = int(np.sum(mask))
            frac = sc / max(tot, 1)
            if frac > best[0]:
                best = (frac, flip, off, sc, tot)
    return best


def main():
    p = argparse.ArgumentParser()
    p.add_argument("if_s16", type=Path)
    p.add_argument("--chip0", type=int, default=None,
                   help="force chip0 offset (default: auto-sweep)")
    p.add_argument("--max-secs", type=int, default=200,
                   help="limit to first N broadcast seconds")
    args = p.parse_args()

    raw, fs, f_if = load_if(args.if_s16)
    print(f"loaded {len(raw)} samples @ {fs:.0f} Hz, IF={f_if:.0f} Hz, "
          f"{len(raw)/fs:.1f} s")

    phase, fs_bb = mix_and_phase(raw, fs, f_if)
    print(f"baseband: {len(phase)} samples @ {fs_bb:.0f} Hz")

    # Convert to the HW's 9-b LSB scale (2π = 2^9) so pm_soft values are
    # in the same order of magnitude as the HW correlator's output.
    phase_lsb = phase * (512.0 / (2.0 * np.pi))

    if args.chip0 is None:
        # Use the first ~30s for offset search.
        srch_n = min(30 * SECOND_SAMPLES, len(phase_lsb))
        best_off, best_score = find_chip0_offset(phase_lsb[:srch_n])
        print(f"auto chip0 offset: {best_off} (score {best_score:.0f})")
        chip0 = best_off
    else:
        chip0 = args.chip0
        print(f"using chip0 = {chip0}")

    pm_soft = pm_soft_per_second(phase_lsb, chip0_offset=chip0)
    if args.max_secs and len(pm_soft) > args.max_secs:
        pm_soft = pm_soft[: args.max_secs]
    signs = np.where(pm_soft > 0, +1, -1)
    pct, flip, off, sc, tot = best_sik_score(signs)
    print(f"\nPython decoder over {len(signs)} seconds:")
    print(f"  best SIK match: {sc}/{tot} = {pct*100:.1f}% "
          f"(flip={flip:+d}, offset={off})")
    print(f"  pm_soft stats: mean={pm_soft.mean():.0f} "
          f"std={pm_soft.std():.0f} "
          f"min={pm_soft.min():.0f} max={pm_soft.max():.0f}")

    # Sweep chip0 by ±5 samples around the chosen one to see how sharp
    # the optimum is.
    print(f"\nSensitivity around chip0={chip0}:")
    for d in range(-5, 6):
        c = chip0 + d
        ps = pm_soft_per_second(phase_lsb, chip0_offset=c)
        if args.max_secs:
            ps = ps[: args.max_secs]
        sg = np.where(ps > 0, +1, -1)
        pct, *_ = best_sik_score(sg)
        marker = "  ←" if d == 0 else ""
        print(f"  chip0={c:5d}: SIK match {pct*100:5.1f}%{marker}")


if __name__ == "__main__":
    raise SystemExit(main())
