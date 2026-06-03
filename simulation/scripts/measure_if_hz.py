#!/usr/bin/env python3
"""Measure the carrier-frequency position in each .if.s16 recording.

Each `.if.s16` is a real-valued int16 stream, sample_rate = 12 kS/s,
nominally centred around `if_hz` (= where wav_to_iq.py downconverted
the original WAV).  The actual SDR LO offset means the carrier line
is at `if_hz + ΔLO` for some ΔLO depending on the source (a few Hz
for a well-calibrated KiwiSDR, up to ~200 Hz for the worst captures
in this corpus).

This script measures the carrier position by FFTing a window of
samples around the nominal IF and parabolically interpolating the
peak.  Result is written back into the corresponding `.if.meta`:

    measured_if_hz=3047.32   # Hz, parabolic-fitted carrier position
    carrier_offset_hz=+47.32 # Hz, = measured_if_hz - if_hz (convenience)
    carrier_snr_db=43.2      # dB, peak / median in the search window

Existing tooling that uses `if_hz` keeps working unchanged.  New
consumers (e.g. run.py) prefer `measured_if_hz` when available.

Usage:
    python3 simulation/scripts/measure_if_hz.py [--all | <stem> ...]
"""
from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

import numpy as np

DERIVED = Path(__file__).resolve().parents[1] / "derived"

NFFT          = 65536          # bin width ≈ 0.183 Hz at fs=12 kS/s
WIN_SECONDS   = 8.0            # 8 s of samples = 96 K -> spans 2 FFT frames
SEARCH_HZ     = 500.0          # search ±500 Hz around nominal if_hz
MIN_SNR_DB    = 6.0            # below this the measurement is considered unreliable


def read_meta(path: Path) -> dict:
    out = {}
    for line in path.read_text().splitlines():
        line = line.strip()
        if not line or line.startswith("#") or "=" not in line:
            continue
        k, v = line.split("=", 1)
        out[k.strip()] = v.strip()
    return out


def write_meta(path: Path, kv: dict) -> None:
    lines = [f"{k}={v}" for k, v in kv.items()]
    path.write_text("\n".join(lines) + "\n")


def parabolic_peak(y_minus1: float, y0: float, y_plus1: float) -> float:
    """Sub-bin peak position offset for a parabolic fit through 3 bins.

    Returns the offset (in bin units) of the peak from the centre bin.
    Range: (-0.5, +0.5).  Caller adds the integer centre-bin index.
    """
    denom = (y_minus1 - 2.0 * y0 + y_plus1)
    if denom == 0.0:
        return 0.0
    return 0.5 * (y_minus1 - y_plus1) / denom


def measure_one(if_s16_path: Path, meta_kv: dict) -> dict | None:
    """Run carrier measurement, return new meta fields or None on failure."""
    fs = int(meta_kv["sample_rate_hz"])
    nominal_if = float(meta_kv["if_hz"])
    sample_count = int(meta_kv["sample_count"])

    n_target = int(WIN_SECONDS * fs)
    n_read   = min(n_target, sample_count)

    # Read first n_read int16 samples (real-valued).
    with open(if_s16_path, "rb") as fh:
        buf = np.frombuffer(fh.read(n_read * 2), dtype="<i2")
    x = buf.astype(np.float64)

    if len(x) < NFFT:
        # Pad to nfft with zeros so we can still FFT.
        x = np.concatenate([x, np.zeros(NFFT - len(x))])
    else:
        # Use the first nfft samples (one window).
        x = x[:NFFT] * np.hanning(NFFT)

    X = np.fft.rfft(x)
    P = np.abs(X) ** 2

    bin_hz   = fs / NFFT
    search_lo = max(0, int(math.floor((nominal_if - SEARCH_HZ) / bin_hz)))
    search_hi = min(len(P) - 1, int(math.ceil((nominal_if + SEARCH_HZ) / bin_hz)))

    window = P[search_lo:search_hi + 1]
    if len(window) == 0:
        return None

    rel_peak_idx = int(np.argmax(window))
    abs_peak_idx = search_lo + rel_peak_idx
    peak_p       = float(P[abs_peak_idx])

    # Parabolic interpolation around the peak (need 1 bin on each side).
    if 1 <= abs_peak_idx < len(P) - 1:
        offset_bins = parabolic_peak(
            float(P[abs_peak_idx - 1]),
            float(P[abs_peak_idx]),
            float(P[abs_peak_idx + 1]),
        )
    else:
        offset_bins = 0.0

    measured_hz = (abs_peak_idx + offset_bins) * bin_hz

    # SNR: peak vs median of search window (rough but robust).
    noise_floor = float(np.median(window))
    snr_db = 10.0 * math.log10(peak_p / max(noise_floor, 1e-30))

    return {
        "measured_if_hz":    f"{measured_hz:.3f}",
        "carrier_offset_hz": f"{measured_hz - nominal_if:+.3f}",
        "carrier_snr_db":    f"{snr_db:.1f}",
    }


def process_meta(meta_path: Path) -> tuple[str, str]:
    """Measure carrier for one .if.meta file.  Returns (status, message)."""
    stem = meta_path.name.replace(".if.meta", "")
    if_s16 = meta_path.with_suffix("").with_suffix(".if.s16")

    if not if_s16.exists():
        return "skip", f"no .if.s16"

    kv = read_meta(meta_path)
    if kv.get("signal_present", "1") == "0":
        return "skip", "signal_present=0"

    if "if_hz" not in kv or "sample_rate_hz" not in kv or "sample_count" not in kv:
        return "skip", "missing required fields"

    new_fields = measure_one(if_s16, kv)
    if new_fields is None:
        return "fail", "FFT search returned no peak"

    snr_db = float(new_fields["carrier_snr_db"])
    weak   = snr_db < MIN_SNR_DB

    kv.update(new_fields)
    write_meta(meta_path, kv)

    msg = (f"if_hz={kv['if_hz']}  "
           f"measured={new_fields['measured_if_hz']}  "
           f"Δ={new_fields['carrier_offset_hz']}  "
           f"SNR={new_fields['carrier_snr_db']} dB"
           + ("  [WEAK]" if weak else ""))
    return ("warn" if weak else "ok"), msg


def main(argv: list[str]) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--all", action="store_true",
                    help="process every .if.meta in simulation/derived/")
    ap.add_argument("stems", nargs="*",
                    help="specific recording stems to process")
    args = ap.parse_args(argv)

    targets: list[Path]
    if args.all:
        targets = sorted(DERIVED.glob("*.if.meta"))
    elif args.stems:
        targets = [DERIVED / f"{s}.if.meta" for s in args.stems]
    else:
        ap.print_help()
        return 1

    if not targets:
        print("no .if.meta found", file=sys.stderr)
        return 1

    for meta in targets:
        if not meta.exists():
            print(f"  missing: {meta}", file=sys.stderr)
            continue
        status, msg = process_meta(meta)
        stem = meta.name.replace(".if.meta", "")
        tag  = {"ok": "  OK", "warn": "WARN", "skip": "skip", "fail": "FAIL"}[status]
        print(f"[{tag}] {stem:50s}  {msg}")

    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
