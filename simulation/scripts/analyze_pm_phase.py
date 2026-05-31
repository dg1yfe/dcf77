#!/usr/bin/env python3
"""Analyze full-rate CORDIC phase telemetry to diagnose PSK SNR loss.

Reads a 14_pm_if_replay_tb CSV captured with TELEM_PERIOD_US=500 (= one
row per CORDIC sample at 2 kS/s) and produces:

  * Power-spectrum plot of cordic_phase (locked window) showing where
    the PSK chip-rate energy (645.83 Hz) sits vs noise.
  * SIK signal-to-noise breakdown: in-band power at chip rate vs noise
    band on either side.
  * Per-second chip-aligned phase reshape (1586 samples × N seconds),
    averaged to see chip-rate template clarity.

Usage:
    DCF77_PLL_TELEM_FAST=1 ... DCF77_PM_PLL_SWEEP="10;22" \
        python3 simulation/scripts/run.py "*pm_if_replay_tb.dcf_med2*"
    python3 simulation/scripts/analyze_pm_phase.py \
        simulation/derived/dcf_med2.pm.kp10_ki22.fast.telem.csv
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np

FS_BB         = 2_000.0           # baseband sample rate
F_CHIP        = 77_500.0 / 120.0  # ≈ 645.833 Hz
PRBS_SAMPLES  = 1586              # 1 PRBS cycle = 793 ms × 2 kHz
SAMPLES_PER_S = int(FS_BB)


def load(path: Path):
    data = np.genfromtxt(path, delimiter=",", names=True, dtype=None)
    t_us = data["t_us"].astype(np.float64)
    return data, t_us / 1e6   # seconds


def panel_spectrum(ax, phase, fs, label):
    """Plot the power spectrum (one-sided)."""
    n = len(phase)
    win = np.hanning(n)
    p = phase * win
    fft = np.fft.rfft(p)
    freqs = np.fft.rfftfreq(n, d=1.0/fs)
    pow_db = 20 * np.log10(np.maximum(np.abs(fft), 1e-9))
    ax.plot(freqs, pow_db, linewidth=0.6, label=label)
    ax.axvline(F_CHIP, color="r", linestyle="--", alpha=0.5,
               label=f"chip rate ≈ {F_CHIP:.1f} Hz")
    # also mark expected PSK harmonics (chip-rate × N) since chip transitions
    # produce odd-order energy too
    for k in (3, 5, 7):
        ax.axvline(k * F_CHIP, color="r", linestyle=":", alpha=0.2)


def main() -> int:
    p = argparse.ArgumentParser()
    p.add_argument("csv", type=Path)
    p.add_argument("--start-s", type=float, default=40.0,
                   help="skip this many seconds of PLL acquisition")
    p.add_argument("--duration-s", type=float, default=60.0,
                   help="analyze window of this length")
    args = p.parse_args()

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    data, t_s = load(args.csv)
    print(f"loaded {len(t_s)} rows = {t_s[-1]:.1f} s of broadcast")
    cols = data.dtype.names
    if "cordic_phase" not in cols:
        print("CSV has no cordic_phase column", file=sys.stderr)
        return 1

    mask = (t_s >= args.start_s) & (t_s < args.start_s + args.duration_s)
    phase = data["cordic_phase"][mask].astype(np.float64)
    if len(phase) < 2 * PRBS_SAMPLES:
        print(f"window too short ({len(phase)} samples)", file=sys.stderr)
        return 1
    pll_lock_window = data["pll_lock"][mask]
    pm_lock_qual = data["pm_lock_qual"][mask]
    print(f"window: {args.start_s:.1f}–{args.start_s + args.duration_s:.1f} s, "
          f"{len(phase)} samples, "
          f"pll_lock fraction = {np.mean(pll_lock_window):.3f}, "
          f"pm_lock_qual range = [{pm_lock_qual.min()}, {pm_lock_qual.max()}]")

    # Center / detrend (remove mean — PLL should already have done this)
    phase_dc = float(phase.mean())
    phase_std = float(phase.std())
    print(f"cordic_phase: mean={phase_dc:.2f} LSB, std={phase_std:.2f} LSB "
          f"(2π = 4096 LSB → std={phase_std * 360 / 4096:.2f}°)")

    # Spectrum.
    n = len(phase)
    win = np.hanning(n)
    p_w = (phase - phase_dc) * win
    fft = np.fft.rfft(p_w)
    freqs = np.fft.rfftfreq(n, d=1.0/FS_BB)
    pow_db = 20 * np.log10(np.maximum(np.abs(fft), 1e-9))

    # Find chip-rate bin and compute SNR vs noise band ±50 Hz around it.
    bin_chip = int(round(F_CHIP * n / FS_BB))
    in_band  = np.abs(np.abs(fft[max(0, bin_chip-2):bin_chip+3])).max()
    # Noise band: 200 Hz on either side of chip rate, excluding chip ±10 Hz
    nb_lo = int(round((F_CHIP - 200) * n / FS_BB))
    nb_hi = int(round((F_CHIP + 200) * n / FS_BB))
    excl  = (np.arange(nb_lo, nb_hi) >= bin_chip - 5) & \
            (np.arange(nb_lo, nb_hi) <= bin_chip + 5)
    noise = np.abs(fft[nb_lo:nb_hi])[~excl]
    noise_med = float(np.median(noise))
    snr_db = 20 * np.log10(in_band / max(noise_med, 1e-9))
    print(f"chip-rate bin power: {20*np.log10(in_band):.1f} dB")
    print(f"surrounding-band noise median: {20*np.log10(noise_med):.1f} dB")
    print(f"chip-rate SNR (peak / median noise): {snr_db:.1f} dB")

    # Per-cycle reshape: chop window into 1586-sample slabs (PRBS cycles).
    # Average over cycles to see chip-rate-aligned template emerging.
    n_cycles = len(phase) // PRBS_SAMPLES
    slab = phase[: n_cycles * PRBS_SAMPLES].reshape(n_cycles, PRBS_SAMPLES)
    slab_mean = slab.mean(axis=0)
    slab_var  = slab.var(axis=0)
    print(f"{n_cycles} PRBS cycles in window — "
          f"per-sample mean swing: ±{np.abs(slab_mean).max():.1f} LSB, "
          f"per-sample var mean: {slab_var.mean():.1f}")

    # Plot.
    fig, axes = plt.subplots(3, 1, figsize=(12, 10), constrained_layout=True)
    ax = axes[0]
    sub = slice(0, min(len(phase), 4000))   # show first 2 s
    ax.plot(np.arange(sub.stop) / FS_BB, phase[sub], linewidth=0.4)
    ax.set_xlabel("time (s)")
    ax.set_ylabel("cordic_phase (LSB)")
    ax.set_title(f"cordic_phase trace ({args.csv.name})")
    ax.grid(alpha=0.3)

    ax = axes[1]
    ax.plot(freqs, pow_db, linewidth=0.6)
    ax.axvline(F_CHIP, color="r", linestyle="--", alpha=0.5,
               label=f"chip rate {F_CHIP:.0f} Hz")
    for k in (3, 5):
        ax.axvline(k * F_CHIP % (FS_BB/2),
                   color="r", linestyle=":", alpha=0.3)
    ax.set_xlim(0, FS_BB/2)
    ax.set_xlabel("frequency (Hz)")
    ax.set_ylabel("power (dB)")
    ax.set_title(f"phase spectrum  (chip-rate SNR = {snr_db:.1f} dB)")
    ax.legend()
    ax.grid(alpha=0.3)

    ax = axes[2]
    ax.plot(np.arange(PRBS_SAMPLES), slab_mean, linewidth=0.5,
            label=f"mean of {n_cycles} PRBS cycles")
    ax.fill_between(np.arange(PRBS_SAMPLES),
                    slab_mean - np.sqrt(slab_var),
                    slab_mean + np.sqrt(slab_var),
                    alpha=0.2, label="±1 σ")
    ax.set_xlabel("sample in PRBS cycle")
    ax.set_ylabel("cordic_phase (LSB)")
    ax.set_title("Per-cycle averaged template (should show chip transitions "
                 "if chip alignment is stable)")
    ax.legend()
    ax.grid(alpha=0.3)

    out = args.csv.with_suffix(".analyze.png")
    fig.savefig(out, dpi=120)
    plt.close(fig)
    print(f"wrote {out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
