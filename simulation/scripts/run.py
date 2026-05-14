#!/usr/bin/env python3
"""VUnit runner for the DCF77 receiver/decoder.

Usage:
    python3 simulation/scripts/run.py             # run the full suite
    python3 simulation/scripts/run.py --clean     # rebuild from scratch
    python3 simulation/scripts/run.py --list      # list tests only

Env vars:
    DCF77_FULL_REPLAY=1     also register the slow full-minute replay test
                            (~30 min wall time).  Off by default.
"""

import os
from pathlib import Path
import sys

from vunit import VUnit


ROOT = Path(__file__).resolve().parent.parent.parent
INC_DIR = ROOT / "inc"
SRC_DIR = ROOT / "src" / "dcf77"
TB_DIR = ROOT / "simulation" / "tb"
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
    """Register the binary-file replay testbench if its inputs are available.

    The replay test reads `simulation/derived/synth.s16` (or any user-supplied
    .s16) plus a sidecar `simulation/derived/synth.meta` describing the
    expected decoded time.  The .s16 and .meta files are produced by:
        python3 simulation/scripts/synth_dcf77_wav.py --out simulation/derived/synth.wav --duration 65
        python3 simulation/scripts/wav_to_postcic.py  --in simulation/derived/synth.wav --out simulation/derived/synth.s16 --meta simulation/derived/synth.meta \\
            --year 26 --month 5 --day 13 --weekday 3 --hour 11 --minute 42 --dst 1
    If either file is missing, the replay testbench is silently skipped so
    the rest of the suite remains runnable without those artefacts.
    """
    try:
        tb = lib.test_bench("dcf77_rx_replay_tb")
    except KeyError:
        return

    s16  = DERIVED_DIR / "synth.s16"
    meta = DERIVED_DIR / "synth.meta"
    if not (s16.exists() and meta.exists()):
        return

    meta_kv = _read_meta(meta)

    sample_rate = int(meta_kv.get("sample_rate_hz", 250000))
    total_samples = int(meta_kv["sample_count"])

    common = dict(
        REPLAY_FILE  = str(s16),
        SAMPLE_COUNT = total_samples,
        EXPECT_YEAR    = int(meta_kv["year"]),
        EXPECT_MONTH   = int(meta_kv["month"]),
        EXPECT_DAY     = int(meta_kv["day"]),
        EXPECT_WEEKDAY = int(meta_kv["weekday"]),
        EXPECT_HOUR    = int(meta_kv["hour"]),
        EXPECT_MINUTE  = int(meta_kv["minute"]),
        EXPECT_DST     = int(meta_kv["dst_active"]),
    )

    # Smoke: feed only the first ~5 s of the recording (enough to detect
    # carrier and at least one bit pulse) so the test stays fast.
    smoke_generics = dict(common,
                          MAX_SAMPLES = 5 * sample_rate,
                          EXPECT_FULL_FRAME = False)

    # Full decode: feed the entire recording.  Required to span a minute
    # mark and trigger frame_dec's time_valid.
    full_generics = dict(common,
                         MAX_SAMPLES = 0,            # 0 = full file
                         EXPECT_FULL_FRAME = True)

    # Attach each config to ONE specific test case so the cross-product of
    # configs and tests doesn't produce nonsensical pairings (e.g. running
    # the full_decode assertions over a 5 s clip).
    tb.test("smoke").add_config(name="synth", generics=smoke_generics)
    # The full-decode test is slow (~30 min wall) so it is opt-in.
    if os.environ.get("DCF77_FULL_REPLAY") == "1":
        tb.test("full_decode").add_config(name="synth", generics=full_generics)


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
