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
    _configure_if_replay_testbench(lib)
    _configure_pm_if_replay_testbench(lib)


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
    full_configured = False

    for wav in sorted(SAMPLES_DIR.glob("*.wav")):
        stem = wav.stem
        s16  = DERIVED_DIR / f"{stem}.s16"
        meta = DERIVED_DIR / f"{stem}.meta"
        if not (s16.exists() and meta.exists()):
            print(f"[replay] {wav.name}: no derived/.s16/.meta -- run "
                  f"`python3 simulation/scripts/wav_to_iq.py --all`")
            continue

        kv      = _read_meta(meta)
        rate    = int(kv.get("sample_rate_hz", 1000))
        pairs   = int(kv["sample_count"])
        has_exp = int(kv.get("expect_full_decode", 0)) == 1

        if not has_exp:
            print(f"[replay] {wav.name}: Python decoder did not lock -- "
                  f"smoke runs without decode-progress assertion")

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
                     EXPECT_FULL_FRAME      = False,
                     EXPECT_VALIDATE_TIME   = False,
                     EXPECT_DECODE_PROGRESS = has_exp)
        tb.test("smoke").add_config(name=stem, generics=smoke)

        if has_exp and want_full:
            full = dict(common,
                        MAX_SAMPLES = 0,
                        EXPECT_FULL_FRAME    = True,
                        EXPECT_VALIDATE_TIME = True)
            tb.test("full_decode").add_config(name=stem, generics=full)
            full_configured = True

    # VUnit runs every `run(...)` in the testbench, so if nobody registered
    # a real full_decode config it would otherwise execute once with default
    # generics and emit a "no REPLAY_FILE" info() line.  Drop in an
    # explicitly-named placeholder so the result is labelled clearly.  We
    # omit REPLAY_FILE from the generics so the entity default ("") kicks
    # in -- ghdl refuses an empty-string generic on the command line.
    if not full_configured:
        reason = ("set_DCF77_FULL_REPLAY_to_1"
                  if not want_full
                  else "no_WAV_with_expect_full_decode")
        tb.test("full_decode").add_config(
            name=reason,
            generics=dict(EXPECT_FULL_FRAME=False))


def _configure_if_replay_testbench(lib) -> None:
    """Register IF-replay configurations per WAV with .if.s16/.if.meta.

    Two tests per WAV:
      acquire -- NCO at nominal 5 kHz; verify the carrier_pll captures
                 the recording's residual IF offset.  On lock, writes
                 the converged phase_inc_offset to <stem>.if.lock.
      decode  -- NCO still at nominal 5 kHz, but the TB reads the
                 seed from <stem>.if.lock at startup and adds it to
                 the live PLL output.  This makes decode independent
                 of acquisition dynamics (PLL only fine-tracks from
                 a known-good seed).

    Test names are alphabetically ordered so `acquire` runs before
    `decode` -- VUnit walks configs that way by default.

    Run `python3 simulation/scripts/wav_to_if.py --all` to populate
    the .if.s16/.if.meta artefacts after dropping WAVs into samples/.
    """
    try:
        tb = lib.test_bench("dcf77_if_replay_tb")
    except KeyError:
        return

    if not SAMPLES_DIR.is_dir():
        return

    want_full = os.environ.get("DCF77_FULL_REPLAY") == "1"
    decode_configured = False

    for wav in sorted(SAMPLES_DIR.glob("*.wav")):
        stem = wav.stem
        if_s16  = DERIVED_DIR / f"{stem}.if.s16"
        if_meta = DERIVED_DIR / f"{stem}.if.meta"
        if_lock = DERIVED_DIR / f"{stem}.if.lock"
        if not (if_s16.exists() and if_meta.exists()):
            print(f"[if-replay] {wav.name}: no derived/.if.s16/.if.meta -- "
                  f"run `python3 simulation/scripts/wav_to_if.py --all`")
            continue

        kv      = _read_meta(if_meta)
        rate    = int(kv.get("sample_rate_hz", 12000))
        if_hz   = int(float(kv.get("if_hz", 5000)))
        samples = int(kv["sample_count"])
        has_exp = int(kv.get("expect_full_decode", 0)) == 1

        common = dict(
            REPLAY_FILE    = str(if_s16),
            SAMPLE_COUNT   = samples,
            SAMPLE_RATE_HZ = rate,
            IF_HZ          = if_hz,
            PE_LOOP        = os.environ.get("DCF77_PE_LOOP") == "1",
            EXPECT_YEAR    = int(kv.get("year",    0)),
            EXPECT_MONTH   = int(kv.get("month",   0)),
            EXPECT_DAY     = int(kv.get("day",     0)),
            EXPECT_WEEKDAY = int(kv.get("weekday", 0)),
            EXPECT_HOUR    = int(kv.get("hour",    0)),
            EXPECT_MINUTE  = int(kv.get("minute",  0)),
            EXPECT_DST     = int(kv.get("dst_active", 0)),
        )

        # acquire: feeds the WHOLE recording from a clean NCO (no
        # seed read).  Writes the final converged phase_inc_offset to
        # LOCK_FILE_OUT for decode to pick up.
        acq_cfg = dict(common,
                       LOCK_FILE_OUT          = str(if_lock),
                       MAX_SAMPLES            = 0,
                       EXPECT_FULL_FRAME      = False,
                       EXPECT_VALIDATE_TIME   = False,
                       EXPECT_DECODE_PROGRESS = has_exp)
        # Opt-in dense PLL telemetry dump (DCF77_PLL_TELEM=1 enables).
        # Writes <stem>.telem.csv next to the .if.s16 with one row per
        # 10 ms of broadcast time -- render via plot_pll_telem.py.
        if os.environ.get("DCF77_PLL_TELEM") == "1":
            acq_cfg["TELEM_CSV"] = str(DERIVED_DIR / f"{stem}.telem.csv")
        tb.test("acquire").add_config(name=stem, generics=acq_cfg)

        if has_exp and want_full:
            dec_cfg = dict(common,
                           LOCK_FILE_IN         = str(if_lock),
                           MAX_SAMPLES          = 0,
                           EXPECT_FULL_FRAME    = True,
                           EXPECT_VALIDATE_TIME = True)
            # Opt-in env_pwr time-series dump (set DCF77_ENV_DUMP=1 to enable).
            if os.environ.get("DCF77_ENV_DUMP") == "1":
                dec_cfg["ENV_DUMP_FILE"] = str(DERIVED_DIR / f"{stem}.env.tsv")
            tb.test("decode").add_config(name=stem, generics=dec_cfg)
            decode_configured = True

    if not decode_configured:
        reason = ("set_DCF77_FULL_REPLAY_to_1"
                  if not want_full
                  else "no_WAV_with_expect_full_decode")
        tb.test("decode").add_config(
            name=reason,
            generics=dict(EXPECT_FULL_FRAME=False))


def _configure_pm_if_replay_testbench(lib) -> None:
    """Register PM IF-replay configurations per .if.s16 recording.

    For each `.if.s16` in derived/, register one `pm_smoke` configuration
    that runs the PM correlator through the full recording and checks
    that pm_sync_state has advanced past FILL and that pm_valid pulses
    have fired (= the PM chain is wired and running).  Decoded time
    validation is not (yet) asserted — needs cached PM truth.
    """
    try:
        tb = lib.test_bench("dcf77_pm_if_replay_tb")
    except KeyError:
        return

    if not SAMPLES_DIR.is_dir():
        return

    for wav in sorted(SAMPLES_DIR.glob("*.wav")):
        stem    = wav.stem
        if_s16  = DERIVED_DIR / f"{stem}.if.s16"
        if_meta = DERIVED_DIR / f"{stem}.if.meta"
        if not (if_s16.exists() and if_meta.exists()):
            continue

        kv      = _read_meta(if_meta)
        rate    = int(kv.get("sample_rate_hz", 12000))
        if_hz   = int(float(kv.get("if_hz", 5000)))
        samples = int(kv["sample_count"])

        cfg = dict(
            REPLAY_FILE         = str(if_s16),
            SAMPLE_COUNT        = samples,
            SAMPLE_RATE_HZ      = rate,
            IF_HZ               = if_hz,
            EXPECT_PM_VALID_MIN = 30,
        )
        if os.environ.get("DCF77_PLL_TELEM") == "1":
            cfg["TELEM_CSV"] = str(DERIVED_DIR / f"{stem}.pm.telem.csv")
        tb.test("pm_smoke").add_config(name=stem, generics=cfg)


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
