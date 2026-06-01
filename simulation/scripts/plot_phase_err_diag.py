#!/usr/bin/env python3
"""Diagnostic plots for captured phase_err streams.

For each .pe.bin in simulation/derived/, write <stem>.pe.diag.png with:
  * spectrogram of cordic_phase (reveals frequency offset / wrapping)
  * histogram (locked-only vs all)
  * rolling 1-second σ + pll_lock strip
  * stats annotation

Raster (PNG) instead of SVG -- the spectrogram and histogram benefit
from raster, and file sizes stay reasonable (~500 KB).
"""
from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from scipy import signal as sps

BASEBAND_HZ = 2000.0
DERIVED = Path(__file__).resolve().parents[1] / "derived"


def read_pe_bin(path: Path) -> tuple[np.ndarray, np.ndarray]:
    raw = np.fromfile(path, dtype="<i2")
    phase = (raw & 0x0FFF).astype(np.int32)
    phase[phase >= 0x800] -= 0x1000
    lock = ((raw >> 15) & 1).astype(np.uint8)
    return phase, lock


def plot_one(stem: str, phase: np.ndarray, lock: np.ndarray,
             out: Path) -> None:
    n = len(phase)
    t = np.arange(n) / BASEBAND_HZ

    fig = plt.figure(figsize=(14, 11))
    gs = fig.add_gridspec(4, 2, height_ratios=[2.5, 1.2, 1.5, 0.4],
                          width_ratios=[3, 1.2], hspace=0.5, wspace=0.25)

    # (1) Spectrogram of cordic_phase
    ax_spec = fig.add_subplot(gs[0, :])
    n_fft  = 512
    hop    = 256
    f, tt, Sxx = sps.spectrogram(
        phase.astype(float), fs=BASEBAND_HZ, nperseg=n_fft, noverlap=n_fft - hop,
        window="hann", scaling="density", mode="psd")
    Sxx_dB = 10 * np.log10(np.maximum(Sxx, 1e-3))
    im = ax_spec.pcolormesh(tt, f, Sxx_dB, shading="auto",
                            cmap="viridis", vmin=Sxx_dB.max() - 60,
                            vmax=Sxx_dB.max())
    ax_spec.set_ylabel("frequency [Hz]")
    ax_spec.set_xlabel("broadcast time [s]")
    ax_spec.set_title(f"{stem} -- spectrogram of cordic_phase  "
                      f"(N_FFT={n_fft}, fs={BASEBAND_HZ:.0f} Hz)")
    cax = fig.add_axes([0.92, 0.74, 0.012, 0.18])
    fig.colorbar(im, cax=cax, label="dB")

    # (2) Rolling 1-second σ of cordic_phase (= 2000-sample window)
    ax_rms = fig.add_subplot(gs[1, :])
    win = int(1.0 * BASEBAND_HZ)
    sq  = phase.astype(np.int64) ** 2
    # Cumulative sum trick for rolling variance
    csum  = np.concatenate([[0], np.cumsum(sq)])
    csum_lin = np.concatenate([[0], np.cumsum(phase)])
    rolling_mean = (csum_lin[win:] - csum_lin[:-win]) / win
    rolling_meansq = (csum[win:] - csum[:-win]) / win
    rolling_var = rolling_meansq - rolling_mean ** 2
    rolling_std = np.sqrt(np.maximum(rolling_var, 0))
    t_rms = (np.arange(len(rolling_std)) + win) / BASEBAND_HZ
    ax_rms.plot(t_rms, rolling_std, lw=0.5, color="#d62728")
    ax_rms.axhline(177, ls=":", color="grey", lw=1,
                   label="PSK deviation (±177 LSB)")
    ax_rms.axhline(1183, ls=":", color="black", lw=1,
                   label="uniform [-π, π] (1183)")
    ax_rms.set_ylabel("rolling σ (1 s) [LSB]")
    ax_rms.set_xlabel("broadcast time [s]")
    ax_rms.grid(alpha=0.3)
    ax_rms.legend(loc="upper right", fontsize=8)
    ax_rms.set_xlim(0, t[-1])

    # (3a) Histogram of cordic_phase, locked vs unlocked
    ax_hist = fig.add_subplot(gs[2, 0])
    bins = np.linspace(-2048, 2048, 81)
    ax_hist.hist(phase[lock == 1], bins=bins, alpha=0.7,
                 color="#2ca02c", label=f"pll_lock=1 ({(lock==1).sum():,})",
                 density=True)
    if (lock == 0).any():
        ax_hist.hist(phase[lock == 0], bins=bins, alpha=0.55,
                     color="#d62728",
                     label=f"pll_lock=0 ({(lock==0).sum():,})", density=True)
    # Reference: PSK ±177 lines
    ax_hist.axvline(-177, ls=":", color="grey", lw=1)
    ax_hist.axvline(+177, ls=":", color="grey", lw=1)
    ax_hist.set_xlabel("cordic_phase [LSB]")
    ax_hist.set_ylabel("density")
    ax_hist.set_title("phase distribution (vertical lines = ±PSK deviation)")
    ax_hist.legend(loc="upper right", fontsize=8)
    ax_hist.grid(alpha=0.3)

    # (3b) Stats panel (text)
    ax_stats = fig.add_subplot(gs[2, 1])
    ax_stats.axis("off")
    locked = phase[lock == 1] if (lock == 1).any() else np.array([])
    stats = [
        f"samples       : {n:,}",
        f"duration      : {n / BASEBAND_HZ:.1f} s",
        f"pll_lock      : {lock.mean()*100:.1f} %",
        "",
        "all-samples phase:",
        f"  mean ± std  : {phase.mean():+.1f} ± {phase.std():.1f}",
        f"  min, max    : {phase.min()}, {phase.max()}",
        "",
        "locked-only phase:",
        f"  mean ± std  : {locked.mean():+.1f} ± {locked.std():.1f}"
            if locked.size else "  (no locked samples)",
        f"  min, max    : {locked.min()}, {locked.max()}"
            if locked.size else "",
        "",
        "expected if locked:",
        "  σ ≈ 200-300  (PSK + noise)",
        "  σ ≈ 1183     (uniform [-π,π])",
    ]
    ax_stats.text(0.0, 1.0, "\n".join(stats),
                  va="top", family="monospace", fontsize=10)

    # (4) pll_lock strip
    ax_lock = fig.add_subplot(gs[3, :])
    ax_lock.fill_between(t, 0, lock.astype(float), step="post",
                         color="#2ca02c", alpha=0.7, linewidth=0)
    ax_lock.set_ylim(-0.1, 1.1)
    ax_lock.set_yticks([0, 1])
    ax_lock.set_yticklabels(["unlk", "lock"])
    ax_lock.set_xlabel("broadcast time [s]")
    ax_lock.set_xlim(0, t[-1])
    ax_lock.grid(alpha=0.3)

    plt.savefig(out, format="png", dpi=120, bbox_inches="tight")
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
        out = path.with_suffix("").with_suffix(".pe.diag.png")
        phase, lock = read_pe_bin(path)
        plot_one(stem, phase, lock, out)
        print(f"{out.name}  ({len(phase):,} samples)")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
