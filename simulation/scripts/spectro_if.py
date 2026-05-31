#!/usr/bin/env python3
"""Power-spectrum + spectrogram analysis of .if.s16 recordings.

For each recording produces a PNG with:
  * Top row: full-recording averaged power spectrum
  * Middle row: spectrogram (time on x, freq on y)
  * Bottom row: zoomed baseband spectrogram (= mixed to DC, ±1 kHz band)

This lets us see, at a glance, whether the carrier is a clean line,
how the noise floor varies over time, whether there are bursts of
interference, and how the AM modulation appears in the baseband band.

Usage:
    python3 simulation/scripts/spectro_if.py                # all .if.s16
    python3 simulation/scripts/spectro_if.py simulation/derived/dcf_med2.if.s16
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np
from scipy import signal


def load_if(path: Path):
    """Load real-valued int16 IF stream + the .meta sidecar."""
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
    return raw, fs, f_if, meta


def render(path: Path) -> None:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    raw, fs, f_if, meta = load_if(path)
    n = len(raw)
    dur_s = n / fs
    print(f"{path.name}: {n} samples, {dur_s:.1f} s, fs={fs:.0f} Hz, "
          f"IF={f_if:.0f} Hz")

    # Smaller FFTs to keep time-resolution without losing frequency
    # detail at the carrier — at fs=12 kHz, nperseg=1024 gives bin
    # width = 11.7 Hz and time resolution = 85 ms per slice.
    NPERSEG = 1024

    # Whole-recording averaged power spectrum (Welch).
    f, Pxx = signal.welch(raw, fs=fs, nperseg=NPERSEG,
                          noverlap=NPERSEG // 2,
                          window="hann", scaling="density")
    Pxx_db = 10 * np.log10(np.maximum(Pxx, 1e-12))

    # Wideband spectrogram (real spectrum).
    f_spec, t_spec, S = signal.spectrogram(
        raw, fs=fs, nperseg=NPERSEG, noverlap=NPERSEG // 2,
        window="hann", scaling="density", mode="psd")
    S_db = 10 * np.log10(np.maximum(S, 1e-12))

    # Mix to baseband and look at ±1 kHz around DC = ±1 kHz around carrier.
    nco = np.exp(-1j * 2 * np.pi * f_if * np.arange(n) / fs)
    bb  = raw * nco
    f_bb, t_bb, S_bb = signal.spectrogram(
        bb, fs=fs, nperseg=NPERSEG, noverlap=NPERSEG // 2,
        window="hann", scaling="density", mode="psd",
        return_onesided=False)
    # Sort by frequency for nicer plot
    order = np.argsort(f_bb)
    f_bb = f_bb[order]
    S_bb = S_bb[order]
    S_bb_db = 10 * np.log10(np.maximum(S_bb, 1e-12))

    fig, axes = plt.subplots(3, 1, figsize=(14, 11), constrained_layout=True)

    # --- Panel 1: averaged power spectrum --------------------------------
    ax = axes[0]
    ax.plot(f, Pxx_db, linewidth=0.6)
    ax.axvline(f_if, color="r", linestyle="--", alpha=0.5,
               label=f"IF = {f_if:.0f} Hz")
    # Also mark IF ± chip_rate so we can spot PSK sidebands
    for d in (-2000, -1000, -645.83, 645.83, 1000, 2000):
        ax.axvline(f_if + d, color="orange", linestyle=":", alpha=0.3)
    ax.set_xlabel("frequency (Hz)")
    ax.set_ylabel("PSD (dB/Hz)")
    ax.set_title(f"{path.name} — averaged power spectrum")
    ax.set_xlim(0, fs / 2)
    ax.grid(alpha=0.3)
    ax.legend()

    # Percentile-based color limits so the carrier line shows up even
    # when noise floor is high (auto-scaling otherwise spans the whole
    # dynamic range and washes out the peak).
    def clim(arr):
        return float(np.percentile(arr, 5)), float(np.percentile(arr, 99.9))

    # --- Panel 2: wideband spectrogram -----------------------------------
    ax = axes[1]
    vmin, vmax = clim(S_db)
    im = ax.pcolormesh(t_spec, f_spec, S_db, shading="auto", cmap="viridis",
                       vmin=vmin, vmax=vmax)
    ax.axhline(f_if, color="r", linestyle="--", alpha=0.5)
    ax.set_xlabel("time (s)")
    ax.set_ylabel("frequency (Hz)")
    ax.set_title(f"Wideband spectrogram  (color: {vmin:.0f}..{vmax:.0f} dB/Hz)")
    fig.colorbar(im, ax=ax, label="dB/Hz")

    # --- Panel 3: baseband spectrogram, zoomed to ±1.5 kHz --------------
    ax = axes[2]
    mask = np.abs(f_bb) <= 1500
    S_bb_masked = S_bb_db[mask, :]
    vmin, vmax = clim(S_bb_masked)
    im = ax.pcolormesh(t_bb, f_bb[mask], S_bb_masked,
                       shading="auto", cmap="viridis",
                       vmin=vmin, vmax=vmax)
    for d in (-645.83, 645.83):
        ax.axhline(d, color="orange", linestyle=":", alpha=0.5,
                   label="±chip rate" if d > 0 else None)
    ax.set_xlabel("time (s)")
    ax.set_ylabel("freq from carrier (Hz)")
    ax.set_title(f"Baseband spectrogram (±1.5 kHz around carrier, "
                 f"color: {vmin:.0f}..{vmax:.0f} dB/Hz)")
    ax.legend()
    fig.colorbar(im, ax=ax, label="dB/Hz")

    out = path.with_suffix(".spectro.png")
    fig.savefig(out, dpi=110)
    plt.close(fig)
    print(f"  wrote {out}")


def main() -> int:
    p = argparse.ArgumentParser()
    p.add_argument("paths", nargs="*", type=Path,
                   help="default: every .if.s16 in simulation/derived")
    args = p.parse_args()

    if args.paths:
        paths = args.paths
    else:
        root = Path(__file__).resolve().parent.parent / "derived"
        paths = sorted(root.glob("*.if.s16"))
        if not paths:
            print("no .if.s16 found", file=sys.stderr)
            return 1

    for p in paths:
        render(p)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
