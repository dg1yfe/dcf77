#!/usr/bin/env python3
"""Score PLL-gain sweep results from dcf77_pm_if_replay_tb telemetry.

After running, e.g.,
    DCF77_PM_PLL_SWEEP="7,10,12,15;20,23,25,28" \
        python3 simulation/scripts/run.py -p 8 "*pm_if_replay*dcf_strong*"

this script reads every dcf_strong.pm.kp*_ki*.telem.csv in
simulation/derived/ and reports per-combo metrics:

    nco_std     — standard deviation of the NCO frequency error (Hz);
                  big = oscillating PLL.
    nco_p99     — 99th-percentile |NCO offset| (Hz).
    sat_pct     — % of samples where |cordic_phase| ≥ 2000 (= ±π);
                  high = CORDIC saturating = PLL not tracking.
    pm_lq_end   — final pm_lock_qual value (higher = better PM lock).
    pm_lq_avg   — average pm_lock_qual after first 20 s.
    sik_score   — best SIK signature match across all (offset, polarity)
                  combinations (read from the per-bit log file if
                  available; otherwise NaN).

Rows are sorted by the composite score so the best combo is on top.
"""
from __future__ import annotations

import re
import sys
from pathlib import Path

import numpy as np


# Same NCO conversion as in plot_pll_telem.py.
NCO_PHASE_W = 25
def nco_lsb_to_hz(offset_lsb, fs_in=12_000.0):
    return offset_lsb * fs_in / (2 ** NCO_PHASE_W)


def find_test_log(csv_path: Path, kp: int, ki: int) -> Path | None:
    """Locate VUnit test_output/.../output.txt matching this CSV run.

    The CSV filename embeds the signal stem (e.g. dcf_strong.pm.kp10_ki23
    .telem.csv); we need to match BOTH the signal stem AND the kp/ki
    suffix when picking the corresponding test-output directory, or a
    plain "kp10_ki23" search would collide across signals.
    """
    root = csv_path.resolve().parents[2] / "vunit_out" / "test_output"
    if not root.exists():
        return None
    # Strip ".pm.kpN_kiM.telem.csv" off the CSV path to recover the stem.
    stem = csv_path.name.split(".pm.kp")[0]   # e.g. "dcf_strong"
    needle = f".{stem}.kp{kp}_ki{ki}.pm_smoke"
    for d in root.iterdir():
        if needle in d.name:
            f = d / "output.txt"
            if f.exists():
                return f
    return None


def sik_score_from_log(log_path: Path) -> float | None:
    """Parse pm_bit log lines and compute the best SIK signature match
    across (offset, polarity) combinations.  Returns the fraction of
    signature bits that agree (0.0..1.0) or None if no bits found."""
    bits = []
    pat = re.compile(r"pm_bit #\d+ sign=([01])")
    for line in log_path.read_text().splitlines():
        m = pat.search(line)
        if m:
            bits.append(1 if m.group(1) == "1" else -1)
    if not bits:
        return None
    obs = np.array(bits)
    best = -1.0
    for sign_flip in (+1, -1):
        for off in range(60):
            exp = np.zeros(len(obs), dtype=int)
            for k in range(len(obs)):
                sec = (off + k) % 60
                if sec <= 9:
                    exp[k] = +1
                elif sec <= 14 or sec == 59:
                    exp[k] = -1
            mask = exp != 0
            if not mask.any():
                continue
            score = float(np.sum(obs[mask] == sign_flip * exp[mask]))
            total = float(np.sum(mask))
            frac = score / total
            if frac > best:
                best = frac
    return best


def score_one(csv_path: Path) -> dict:
    data = np.genfromtxt(csv_path, delimiter=",", names=True, dtype=None)
    if data.size == 0:
        return {}
    cols = list(data.dtype.names)
    nco_hz   = nco_lsb_to_hz(data["nco_offset"].astype(float))
    sat_frac = float(np.mean(np.abs(data["cordic_phase"]) >= 2000)) \
        if "cordic_phase" in cols else float("nan")

    pm_lq_end = int(data["pm_lock_qual"][-1]) if "pm_lock_qual" in cols else 0
    if "t_ms" in cols:
        t_ms = data["t_ms"].astype(int)
    else:
        t_ms = (data["t_us"].astype(int)) // 1000
    late = t_ms > 20_000
    pm_lq_avg = float(np.mean(data["pm_lock_qual"][late])) \
        if late.any() and "pm_lock_qual" in cols else 0.0

    return dict(
        path      = csv_path,
        nco_std   = float(np.std(nco_hz)),
        nco_p99   = float(np.percentile(np.abs(nco_hz), 99)),
        sat_pct   = sat_frac * 100.0,
        pm_lq_end = pm_lq_end,
        pm_lq_avg = pm_lq_avg,
    )


def parse_kp_ki(path: Path) -> tuple[int, int] | None:
    m = re.search(r"kp(\d+)_ki(\d+)", path.name)
    if not m:
        return None
    return int(m.group(1)), int(m.group(2))


def main() -> int:
    if len(sys.argv) > 1:
        files = [Path(p) for p in sys.argv[1:]]
    else:
        derived = Path(__file__).resolve().parent.parent / "derived"
        files = sorted(derived.glob("*.pm.kp*_ki*.telem.csv"))

    if not files:
        print("no sweep CSVs found", file=sys.stderr)
        return 1

    rows = []
    for f in files:
        kpki = parse_kp_ki(f)
        if not kpki:
            continue
        s = score_one(f)
        if not s:
            continue
        s["kp"], s["ki"] = kpki
        log = find_test_log(f, *kpki)
        s["sik"] = sik_score_from_log(log) if log else None
        rows.append(s)

    # Sort by SIK match (most direct quality metric); break ties by
    # smaller NCO oscillation.
    rows.sort(key=lambda r: (-(r["sik"] or 0.0), r["nco_std"]))

    print(f"{'KP':>3} {'KI':>3} | "
          f"{'nco_std_Hz':>11} {'nco_p99_Hz':>11} {'sat_%':>7} "
          f"{'pm_lq_avg':>10} {'pm_lq_end':>10} {'sik_%':>7}")
    print("-" * 84)
    for r in rows:
        sik_str = f"{r['sik']*100:.1f}" if r["sik"] is not None else "n/a"
        print(f"{r['kp']:>3} {r['ki']:>3} | "
              f"{r['nco_std']:>11.3f} {r['nco_p99']:>11.3f} "
              f"{r['sat_pct']:>7.2f} "
              f"{r['pm_lq_avg']:>10.0f} {r['pm_lq_end']:>10d} "
              f"{sik_str:>7}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
