#!/usr/bin/env python3
"""Plot PLL telemetry CSVs emitted by the IF-replay testbenches.

The 07 and 14 IF-replay testbenches each accept a TELEM_CSV generic
that, when non-empty, writes one row per TELEM_PERIOD_MS of broadcast
time.  Rows from 07 cover NCO offset / filtered phase error / lock
state; 14 adds PM correlator status (pm_soft, pm_lock_qual, sync_state).
This script renders the CSV with matplotlib -- one subplot per group
(frequency, phase, envelope, PM status if present).

Usage:
    DCF77_PLL_TELEM=1 python3 simulation/scripts/run.py "*.acquire.*"
    python3 simulation/scripts/plot_pll_telem.py simulation/derived/dcf_strong.telem.csv

If invoked without an argument, plots every *.telem.csv in
simulation/derived/.
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np


# Conversion factors for the IF-replay generic defaults.  NCO offset is
# signed N_b at 2^N_b corresponds to fs_mixer cycles/sample -- expressed
# in Hz on the IF/baseband mixer.
NCO_PHASE_W = 25
NCO_OFFSET_W = 24
# Default 07 chain: IF_HZ=5000, fs_in=12000.  Mixer Fs at the boxcar
# tick rate = 12 kHz (one mixer tick per IF sample).  NCO_PHASE_W=25
# corresponds to 2π = 2^25 phase units per cycle.  An offset of `o` LSB
# shifts the synthesized NCO frequency by  o * fs_in / 2^NCO_PHASE_W  Hz.
def nco_lsb_to_hz(offset_lsb: int, fs_in: float = 12_000.0) -> float:
    return offset_lsb * fs_in / (2 ** NCO_PHASE_W)


# Filtered phase error in 29-b at 2π = 2^23 (= 12-b CORDIC shifted left
# by 11, but the 07 chain's pll_qfilt is sign(I_iir)*Q_box, scale set
# by post-boxcar magnitude).  Just plot the raw signed value; calibrate
# to degrees only if the dump comes from 14 (CORDIC-derived).
def lsb_to_deg(x_lsb: int, lsb_per_2pi: float) -> float:
    return x_lsb * 360.0 / lsb_per_2pi


def plot_one(csv_path: Path) -> None:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    data = np.genfromtxt(csv_path, delimiter=",", names=True, dtype=None)
    if data.size == 0:
        print(f"{csv_path}: empty")
        return

    if "t_ms" in data.dtype.names:
        t_s = data["t_ms"] / 1000.0
    elif "t_us" in data.dtype.names:
        t_s = data["t_us"] / 1_000_000.0
    else:
        raise RuntimeError("CSV has no t_ms or t_us column")

    cols = list(data.dtype.names)
    has_pm = "pm_soft" in cols or "pm_lock_qual" in cols
    n_panels = 4 if has_pm else 3
    fig, axes = plt.subplots(n_panels, 1, figsize=(11, 1.7 + 1.4 * n_panels),
                             sharex=True, constrained_layout=True)

    # Panel 1: NCO offset / frequency error
    ax = axes[0]
    f_hz = np.array([nco_lsb_to_hz(int(v)) for v in data["nco_offset"]])
    ax.plot(t_s, f_hz, color="tab:blue", linewidth=0.7)
    ax.axhline(0.0, color="k", linewidth=0.4, alpha=0.5)
    ax.set_ylabel("NCO offset (Hz)")
    ax.set_title(f"PLL telemetry — {csv_path.name}")
    ax.grid(True, alpha=0.3)

    # Panel 2: filtered phase error (q_filt) and slow lock IIR (q_lock)
    ax = axes[1]
    ax.plot(t_s, data["pll_qfilt"], color="tab:red",   linewidth=0.6,
            label="pll_qfilt")
    ax.plot(t_s, data["pll_qlock"], color="tab:orange", linewidth=0.6,
            label="pll_qlock", alpha=0.8)
    ax.axhline(0.0, color="k", linewidth=0.4, alpha=0.5)
    ax.set_ylabel("PLL filt err (LSB)")
    ax.legend(loc="upper right", fontsize=8)
    ax.grid(True, alpha=0.3)

    # Panel 3: lock state, with env_pwr or cordic_phase on twin axis.
    ax = axes[2]
    ax.fill_between(t_s, 0, data["pll_lock"], color="tab:green",
                    alpha=0.25, step="post", label="pll_lock")
    ax.set_ylabel("pll_lock", color="tab:green")
    ax.set_ylim(-0.05, 1.05)
    ax.grid(True, alpha=0.3)
    ax2 = ax.twinx()
    if "env_pwr" in cols:
        ax2.plot(t_s, data["env_pwr"], color="tab:purple", linewidth=0.4)
        ax2.set_ylabel("env_pwr (LSB)", color="tab:purple")
    elif "cordic_phase" in cols:
        ax2.plot(t_s, data["cordic_phase"], color="tab:purple", linewidth=0.4)
        ax2.set_ylabel("cordic_phase (LSB)", color="tab:purple")

    if has_pm:
        ax = axes[3]
        if "pm_lock_qual" in cols:
            ax.plot(t_s, data["pm_lock_qual"], color="tab:cyan",
                    linewidth=0.7, label="pm_lock_qual")
        if "pm_soft" in cols:
            ax.plot(t_s, data["pm_soft"], color="tab:brown",
                    linewidth=0.4, alpha=0.6, label="pm_soft")
        ax.axhline(0.0, color="k", linewidth=0.4, alpha=0.5)
        ax.set_ylabel("PM status")
        ax.legend(loc="upper right", fontsize=8)
        ax.grid(True, alpha=0.3)

    axes[-1].set_xlabel("broadcast time (s)")
    out = csv_path.with_suffix(".png")
    fig.savefig(out, dpi=120)
    plt.close(fig)
    print(f"wrote {out}")


def main() -> int:
    p = argparse.ArgumentParser(description="Render PLL-telemetry CSV files.")
    p.add_argument("csv", nargs="*", type=Path,
                   help="CSV files; default = simulation/derived/*.telem.csv")
    args = p.parse_args()

    paths = list(args.csv)
    if not paths:
        derived = Path(__file__).resolve().parent.parent / "derived"
        paths = sorted(derived.glob("*.telem.csv"))
        if not paths:
            print(f"no *.telem.csv in {derived}", file=sys.stderr)
            return 1

    for p in paths:
        plot_one(p)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
