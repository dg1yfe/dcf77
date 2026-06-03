#!/usr/bin/env python3
"""Evaluate candidate lock-indicator metrics on the .if.s16 corpus.

Mixes each recording through a Python equivalent of the HW front-end:
  IF (12 kS/s real) ──► NCO mixer (measured_if_hz) ──► I/Q
  I/Q ──► boxcar /6 ──► I/Q at 2 kS/s (post-boxcar, matches HW)

Then reports for each recording:
  mean(I)            -- DC carrier amplitude through the lock
  mean(|I|)          -- |I| envelope (≈ A for locked clean PSK)
  mean(|Q|)          -- |Q| envelope (≈ 0.27 A for locked clean PSK)
  mean(|I|)/mean(|Q|) -- I/Q asymmetry (locked clean ≈ 3.6, unlocked ≈ 1)
  mean(I)/mean(mag)  -- carrier coherence ratio (∈ [0, 1])
  mean(mag)          -- received signal strength
  mean(|cordic|)     -- current HW metric, for reference

A 4 s warmup is skipped at the start of each recording.
"""
from __future__ import annotations

import sys
import math
from pathlib import Path

import numpy as np

DERIVED = Path(__file__).resolve().parents[1] / "derived"
FS_IF   = 12000.0    # IF sample rate
FS_BB   = 2000.0     # post-boxcar baseband sample rate
DEC     = 6          # boxcar decimation factor
WARMUP_S = 4.0


def read_meta(path: Path) -> dict:
    out = {}
    for line in path.read_text().splitlines():
        line = line.strip()
        if not line or line.startswith("#") or "=" not in line:
            continue
        k, v = line.split("=", 1)
        out[k.strip()] = v.strip()
    return out


def refine_carrier_hz(x: np.ndarray, f_init_hz: float) -> float:
    """Refine f_carrier estimate using a long FFT of the *full* recording.

    The .if.meta `measured_if_hz` is estimated from the first 8 s; on a
    60+ s recording the residual is ~20 mHz at best, which open-loop
    integrates to ~1 cycle of phase drift.  A full-recording FFT
    (NFFT = next-pow-2(N) up to 1M) has bin width ≈ fs/NFFT down to
    ~10 mHz, with parabolic interpolation getting us within a couple
    of mHz — enough that I/Q metrics stay clean over 60 s.
    """
    n = len(x)
    # Cap NFFT at 1M to avoid blowing memory; fs/1M = 11.4 mHz bin.
    nfft = min(1 << 20, 1 << (int(np.ceil(np.log2(max(n, 1))))))
    if nfft < 4096:
        return f_init_hz
    # If recording is longer than nfft, just take the first nfft samples
    # (still ~87 s at fs=12 kS/s — bin width 11 mHz is the precision
    # limit anyway).
    x_clip = x[:nfft] if len(x) >= nfft else x
    x_pad = np.zeros(nfft)
    x_pad[:len(x_clip)] = x_clip * np.hanning(len(x_clip))
    X = np.fft.rfft(x_pad)
    P = (X.real**2 + X.imag**2)
    bin_hz = FS_IF / nfft
    center = int(round(f_init_hz / bin_hz))
    search = max(8, int(round(50.0 / bin_hz)))   # ±50 Hz search
    lo, hi = max(1, center - search), min(len(P) - 2, center + search)
    rel = int(np.argmax(P[lo:hi + 1]))
    k = lo + rel
    # Parabolic peak interp.
    a, b, c = float(P[k - 1]), float(P[k]), float(P[k + 1])
    denom = (a - 2 * b + c)
    delta = 0.5 * (a - c) / denom if denom != 0 else 0.0
    return (k + delta) * bin_hz


def mix_and_decimate(s16_path: Path, f_carrier_hz: float,
                     max_seconds: float | None = None
                     ) -> tuple[np.ndarray, np.ndarray, float]:
    """Read .if.s16, mix to baseband at f_carrier_hz (refined on the
    full recording), boxcar /6.

    Returns (I, Q, f_used) as float arrays at 2 kS/s plus the actual
    carrier frequency used.
    """
    with open(s16_path, "rb") as fh:
        x = np.frombuffer(fh.read(), dtype="<i2").astype(np.float64)
    if max_seconds is not None:
        x = x[: int(max_seconds * FS_IF)]

    f_refined = refine_carrier_hz(x, f_carrier_hz)

    n = len(x)
    t = np.arange(n) / FS_IF
    lo = np.exp(-1j * 2 * np.pi * f_refined * t)
    bb = x * lo

    n_out = n // DEC
    bb = bb[: n_out * DEC]
    bb = bb.reshape(n_out, DEC).sum(axis=1)

    return bb.real, bb.imag, f_refined


def metrics(I: np.ndarray, Q: np.ndarray) -> dict:
    """Per-recording lock-indicator candidates.

    The mixer LO has an arbitrary initial phase, so to compare apples
    with apples with the HW PLL (which by construction keeps the
    carrier on the I axis) we de-rotate the baseband so that the
    complex mean(I + jQ) sits on the real axis.  After de-rotation
    the "Q" axis carries the same modulation + noise the HW PLL
    routes to its phase detector.
    """
    z   = I + 1j * Q
    dc  = z.mean()
    # De-rotate: align mean(I+jQ) onto the real axis (== what the PLL does).
    if abs(dc) > 0:
        z_r = z * np.exp(-1j * np.angle(dc))
    else:
        z_r = z
    I_r = z_r.real
    Q_r = z_r.imag
    mag = np.abs(z_r)

    # CORDIC-equivalent phase, in 12-b LSB (±2048 = ±π), of the
    # de-rotated baseband.  Should match HW cordic_phase σ closely.
    phase_lsb = np.arctan2(Q_r, I_r) * 2048.0 / math.pi

    return {
        "mean_I":         float(I_r.mean()),            # = |DC| after de-rot
        "mean_abs_I":     float(np.abs(I_r).mean()),
        "mean_abs_Q":     float(np.abs(Q_r).mean()),
        "mean_mag":       float(mag.mean()),
        # Rotation-invariant SNR proxy: |⟨z⟩| / ⟨|z|⟩.  ∈ [0, 1].
        # 1 = pure carrier, 0.96 = clean PSK only, lower with noise.
        "coh":            float(abs(dc) / max(mag.mean(), 1e-9)),
        # I/Q amplitude ratio after de-rotation.
        "abs_I_over_Q":   float(np.abs(I_r).mean() / max(np.abs(Q_r).mean(), 1)),
        # Carrier amplitude normalized to envelope.
        "I_over_mag":     float(I_r.mean() / max(mag.mean(), 1)),
        "mean_abs_phase": float(np.abs(phase_lsb).mean()),
        "sigma_phase":    float(phase_lsb.std()),
    }


def main(argv: list[str]) -> int:
    metas = sorted(DERIVED.glob("*.if.meta"))
    if not metas:
        print("no .if.meta files found", file=sys.stderr)
        return 1

    rows = []
    for meta_path in metas:
        kv = read_meta(meta_path)
        if kv.get("signal_present", "1") == "0":
            continue
        if_s16 = meta_path.with_suffix("").with_suffix(".if.s16")
        if not if_s16.exists():
            continue
        f_carrier = float(kv.get("measured_if_hz", kv["if_hz"]))

        I, Q, f_used = mix_and_decimate(if_s16, f_carrier, max_seconds=60.0)
        # Skip warmup.
        skip = int(WARMUP_S * FS_BB)
        I, Q = I[skip:], Q[skip:]
        if len(I) < 100:
            continue

        m = metrics(I, Q)
        m["stem"] = meta_path.name[:-len(".if.meta")][:35]
        m["snr_db"] = float(kv.get("carrier_snr_db", "nan"))
        rows.append(m)

    # Sort by mean_abs_phase (the current HW metric) so the order
    # matches the existing corpus tables.
    rows.sort(key=lambda r: r["mean_abs_phase"])

    print(f"{'recording':35s}  {'σ(p)':>5s} {'|p|':>5s}  "
          f"{'|DC|':>7s} {'⟨|z|⟩':>7s}  "
          f"{'coh':>5s}  {'|I|/|Q|':>7s}  {'SNR_dB':>6s}")
    print("-" * 95)
    for r in rows:
        print(f"{r['stem']:35s}  "
              f"{r['sigma_phase']:5.0f} {r['mean_abs_phase']:5.0f}  "
              f"{r['mean_I']:7.0f} {r['mean_mag']:7.0f}  "
              f"{r['coh']:5.3f}  {r['abs_I_over_Q']:7.2f}  "
              f"{r['snr_db']:6.1f}")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
