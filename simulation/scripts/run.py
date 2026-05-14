#!/usr/bin/env python3
"""VUnit runner for the DCF77 receiver/decoder.

Usage:
    python3 simulation/scripts/run.py             # run the full suite
    python3 simulation/scripts/run.py --clean     # rebuild from scratch
    python3 simulation/scripts/run.py --list      # list tests only

Env vars:
    DCF77_FULL_REPLAY=1     also register the full_decode replay test for
                            every WAV whose Python soft-decoder locked.
                            Off by default.
"""

import os
from pathlib import Path
import sys

from vunit import VUnit


ROOT = Path(__file__).resolve().parent.parent.parent
INC_DIR = ROOT / "inc"
SRC_DIR = ROOT / "src" / "dcf77"
TB_DIR = ROOT / "simulation" / "tb"
SAMPLES_DIR = ROOT / "samples"
DERIVED_DIR = ROOT / "simulation" / "derived"


def _add_sources(lib, path: Path) -> None:
    """Add every *.vhd / *.vhdl file under `path` to the given library."""
    if not path.is_dir():
        return
    for pattern in ("*.vhd", "*.vhdl"):
        for f in sorted(path.glob(pattern)):
            lib.add_source_file(str(f))


def build() -> VUnit:
    vu = VUnit.from_argv(
        argv=sys.argv[1:],
        compile_builtins=False,
    )
    vu.add_vhdl_builtins()
    vu.add_verification_components()

    lib = vu.add_library("dcf77")

    _add_sources(lib, INC_DIR)
    _add_sources(lib, SRC_DIR)
    _add_sources(lib, TB_DIR)

    # Apply compile/sim flags with allow_empty so this works on partial trees.
    have_sources = bool(
        list(SRC_DIR.glob("*.vhd")) or list(INC_DIR.glob("*.vhd"))
    )
    have_tbs = bool(list(TB_DIR.glob("*.vhd")))
    if have_sources:
        lib.set_compile_option(
            "ghdl.a_flags", ["-fsynopsys", "--std=08", "-frelaxed"]
        )
    if have_tbs:
        lib.set_sim_option(
            "ghdl.elab_flags", ["-fsynopsys", "--std=08", "-frelaxed"]
        )

    _configure_testbenches(lib)

    DERIVED_DIR.mkdir(parents=True, exist_ok=True)

    return vu


def _configure_testbenches(lib) -> None:
    """Register per-testbench configurations (e.g., multiple CIC widths)."""
    try:
        cic_tb = lib.test_bench("dcf77_cic_dec_tb")
    except KeyError:
        return

    # CIC0 in the cascade (R=2, no growth past the input width).
    cic_tb.add_config(
        name="r2",
        generics=dict(
            DEC_R=2,
            N_STAGES=3,
            IN_WIDTH=14,
            INT_WIDTH=17,
            COMB_WIDTH=17,
            OUT_WIDTH=17,
        ),
    )
    # CIC1 / CIC3 (R=10), widths per cic_pruning.py with margin=0.
    cic_tb.add_config(
        name="r10",
        generics=dict(
            DEC_R=10,
            N_STAGES=3,
            IN_WIDTH=17,
            INT_WIDTH=29,
            COMB_WIDTH=26,
            OUT_WIDTH=22,
        ),
    )
    # CIC2 / CIC4 (R=25), widths per cic_pruning.py with margin=0.
    cic_tb.add_config(
        name="r25",
        generics=dict(
            DEC_R=25,
            N_STAGES=3,
            IN_WIDTH=22,
            INT_WIDTH=33,
            COMB_WIDTH=26,
            OUT_WIDTH=22,
        ),
    )

    _configure_replay_testbench(lib)


def _configure_replay_testbench(lib) -> None:
    """Register one replay-TB configuration per WAV in samples/.

    For each samples/<stem>.wav the runner looks for matching artefacts
    derived/<stem>.s16 + .meta produced by wav_to_iq.py.  If the .meta
    has expect_full_decode=1 (i.e. the Python soft decoder locked onto
    two consecutive frames), a full_decode config is also registered
    -- gated on DCF77_FULL_REPLAY=1 so the slower test is opt-in.

    Run `python3 simulation/scripts/wav_to_iq.py --all` once to populate
    derived/ after dropping new WAVs into samples/.
    """
    try:
        tb = lib.test_bench("dcf77_rx_replay_tb")
    except KeyError:
        return

    if not SAMPLES_DIR.is_dir():
        return

    want_full = os.environ.get("DCF77_FULL_REPLAY") == "1"

    for wav in sorted(SAMPLES_DIR.glob("*.wav")):
        stem = wav.stem
        s16  = DERIVED_DIR / f"{stem}.s16"
        meta = DERIVED_DIR / f"{stem}.meta"
        if not (s16.exists() and meta.exists()):
            print(f"[replay] {wav.name}: no derived/.s16/.meta -- run "
                  f"`python3 simulation/scripts/wav_to_iq.py --all`")
            continue

        kv         = _read_meta(meta)
        rate       = int(kv.get("sample_rate_hz", 1000))
        pairs      = int(kv["sample_count"])
        has_signal = int(kv.get("signal_present", 1)) == 1
        has_exp    = int(kv.get("expect_full_decode", 0)) == 1

        if not has_signal:
            print(f"[replay] {wav.name}: signal_present=0 in meta -- skipped")
            continue

        common = dict(
            REPLAY_FILE   = str(s16),
            SAMPLE_COUNT  = pairs,
            EXPECT_YEAR    = int(kv.get("year",    0)),
            EXPECT_MONTH   = int(kv.get("month",   0)),
            EXPECT_DAY     = int(kv.get("day",     0)),
            EXPECT_WEEKDAY = int(kv.get("weekday", 0)),
            EXPECT_HOUR    = int(kv.get("hour",    0)),
            EXPECT_MINUTE  = int(kv.get("minute",  0)),
            EXPECT_DST     = int(kv.get("dst_active", 0)),
        )

        # >=120 s of feed so the second minute mark fires (the first one
        # delimits an empty frame_buf; the second carries the first full
        # 59-bit frame and triggers time_valid).
        smoke = dict(common,
                     MAX_SAMPLES = 130 * rate,
                     EXPECT_FULL_FRAME    = False,
                     EXPECT_VALIDATE_TIME = False)
        tb.test("smoke").add_config(name=stem, generics=smoke)

        if has_exp and want_full:
            full = dict(common,
                        MAX_SAMPLES = 0,
                        EXPECT_FULL_FRAME    = True,
                        EXPECT_VALIDATE_TIME = True)
            tb.test("full_decode").add_config(name=stem, generics=full)


def _read_meta(path) -> dict:
    out = {}
    for line in path.read_text().splitlines():
        line = line.strip()
        if not line or line.startswith("#") or "=" not in line:
            continue
        k, v = line.split("=", 1)
        out[k.strip()] = v.strip()
    return out


def main() -> int:
    vu = build()
    vu.main()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
