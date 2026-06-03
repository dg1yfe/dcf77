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
    _configure_phase_err_capture_testbench(lib)
    _configure_pm_corr_replay_testbench(lib)


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
        if_hz   = _effective_if_hz(kv)
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
        if_hz   = _effective_if_hz(kv)
        samples = int(kv["sample_count"])

        base_cfg = dict(
            REPLAY_FILE         = str(if_s16),
            SAMPLE_COUNT        = samples,
            SAMPLE_RATE_HZ      = rate,
            IF_HZ               = if_hz,
            EXPECT_PM_VALID_MIN = 30,
        )

        # Optional PLL-gain sweep: enumerate one config per (KP, KI) pair.
        # Activated by DCF77_PM_PLL_SWEEP="kp1,kp2,...;ki1,ki2,..." (two
        # comma lists separated by ';').  Each combo writes a uniquely-
        # named telemetry CSV so the analysis script can compare them.
        sweep = os.environ.get("DCF77_PM_PLL_SWEEP")
        if sweep:
            kp_str, ki_str = sweep.split(";", 1)
            kp_list = [int(s) for s in kp_str.split(",") if s.strip()]
            ki_list = [int(s) for s in ki_str.split(",") if s.strip()]
            for kp in kp_list:
                for ki in ki_list:
                    cfg = dict(base_cfg)
                    cfg["PLL_KP_SHIFT"] = kp
                    cfg["PLL_KI_SHIFT"] = ki
                    suffix = f"kp{kp}_ki{ki}"
                    if os.environ.get("DCF77_PLL_TELEM_FAST") == "1":
                        cfg["TELEM_PERIOD_US"] = 500
                        cfg["TELEM_CSV"] = str(
                            DERIVED_DIR
                            / f"{stem}.pm.{suffix}.fast.telem.csv")
                    else:
                        cfg["TELEM_CSV"] = str(
                            DERIVED_DIR
                            / f"{stem}.pm.{suffix}.telem.csv")
                    tb.test("pm_smoke").add_config(
                        name=f"{stem}.{suffix}", generics=cfg)
            continue

        cfg = dict(base_cfg)
        if os.environ.get("DCF77_PLL_TELEM") == "1":
            cfg["TELEM_CSV"] = str(DERIVED_DIR / f"{stem}.pm.telem.csv")
            # DCF77_PLL_TELEM_FAST=1 → dump every CORDIC sample (2 kHz)
            # so the chip-rate (645 Hz) is properly resolved.  Default
            # is 10 ms (100 Hz), heavily aliased for chip-rate analysis.
            if os.environ.get("DCF77_PLL_TELEM_FAST") == "1":
                cfg["TELEM_PERIOD_US"] = 500
                cfg["TELEM_CSV"] = str(
                    DERIVED_DIR / f"{stem}.pm.fast.telem.csv")
        # DCF77_PM_ACQUIRE_LOG=1 → write per-hypothesis ACQUIRE
        # autocorrelation profile to <stem>.pm.acquire.csv (one row per
        # hypothesis evaluated: t_us, hyp_idx, abs_acc, sec_phase).
        if os.environ.get("DCF77_PM_ACQUIRE_LOG") == "1":
            cfg["ACQUIRE_LOG_CSV"] = str(
                DERIVED_DIR / f"{stem}.pm.acquire.csv")
        tb.test("pm_smoke").add_config(name=stem, generics=cfg)


def _effective_if_hz(kv: dict) -> int:
    """Return the carrier-centred IF for the testbench's NCO (integer Hz).

    Prefers the per-recording `measured_if_hz` (= the Python pre-
    acquisition output of `measure_if_hz.py`) rounded to nearest int;
    falls back to the nominal `if_hz`.  For sub-Hz precision use
    `_nco_phase_incr(kv, sample_rate, nco_phase_w)` to compute an
    integer NCO_PHASE_INCR_VAL directly.
    """
    measured = kv.get("measured_if_hz")
    if measured is not None:
        try:
            return int(round(float(measured)))
        except ValueError:
            pass
    return int(float(kv["if_hz"]))


def _nco_phase_incr(kv: dict, sample_rate_hz: int = 12_000,
                    nco_phase_w: int = 25) -> int:
    """Compute the precise NCO_PHASE_INCR_VAL from .if.meta.

    Uses `measured_if_hz` when available (sub-Hz precision), else
    `if_hz`.  Returns the rounded integer.  At fs=12 kS/s and
    NCO_PHASE_W=25 the LSB resolution is fs/2^25 ≈ 358 µHz -- well
    below any residual offset we care about.  Pass this to the
    testbench's NCO_PHASE_INCR_OVERRIDE generic to bypass the
    integer-Hz-only IF_HZ path.
    """
    measured = kv.get("measured_if_hz")
    src = float(measured) if measured is not None else float(kv["if_hz"])
    return int(round(src * (1 << nco_phase_w) / sample_rate_hz))


def _configure_phase_err_capture_testbench(lib) -> None:
    """Register phase_err capture configurations per .if.s16 recording.

    For each `.if.s16` in derived/, register one `capture` configuration
    on dcf77_phase_err_capture_tb.  The capture writes a small
    int16-LE binary file (one sample per cordic_v pulse) to
    `<stem>.pe.bin` for replay by dcf77_pm_correlator_replay_tb.

    The capture TB instantiates only the upstream IF chain (NCO,
    mixer, boxcars, CORDIC, carrier PLL) -- no correlator, no
    frame_dec -- so it runs at upstream-only wall-clock cost.
    """
    try:
        tb = lib.test_bench("dcf77_phase_err_capture_tb")
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
        if_hz   = _effective_if_hz(kv)
        samples = int(kv["sample_count"])

        base = dict(
            REPLAY_FILE    = str(if_s16),
            SAMPLE_COUNT   = samples,
            SAMPLE_RATE_HZ = rate,
            IF_HZ          = if_hz,
            NCO_PHASE_INCR_OVERRIDE = _nco_phase_incr(kv, rate),
        )
        # DCF77_CAPTURE_CPT overrides CLOCKS_PER_TICK (default 8).
        cpt = os.environ.get("DCF77_CAPTURE_CPT")
        if cpt:
            base["CLOCKS_PER_TICK"] = int(cpt)
        # DCF77_CAPTURE_MAX_SEC truncates each capture to N broadcast
        # seconds.  Useful when sweeping PLL gains -- 30 s is plenty to
        # see lock/no-lock and the spectral peak, at ~30 s wall per
        # capture vs ~270 s for the full recording.
        max_sec = os.environ.get("DCF77_CAPTURE_MAX_SEC")
        if max_sec:
            base["MAX_SAMPLES"] = int(max_sec) * rate

        # DCF77_CAPTURE_PLL_SWEEP enumerates one capture per tuple
        # "KP:KI[:IIR[:INV]]" (space-separated list).  INV is 0/1.
        # Output files: <stem>.kp{KP}_ki{KI}[_iir{IIR}][_inv].pe.bin.
        sweep = os.environ.get("DCF77_CAPTURE_PLL_SWEEP", "").strip()
        if sweep:
            for tup in sweep.split():
                parts = tup.split(":")
                kp = int(parts[0]); ki = int(parts[1])
                iir = int(parts[2]) if len(parts) >= 3 else None
                inv = int(parts[3]) if len(parts) >= 4 else 0
                suffix = f"kp{kp}_ki{ki}"
                extra  = dict()
                if iir is not None:
                    suffix += f"_iir{iir}"
                    extra["PLL_LOOP_IIR_SHIFT"] = iir
                if inv:
                    suffix += "_inv"
                    extra["PLL_INVERT_PHASE"] = True
                cfg = dict(base,
                           PLL_KP_SHIFT  = kp,
                           PLL_KI_SHIFT  = ki,
                           PHASE_ERR_DUMP = str(
                               DERIVED_DIR / f"{stem}.{suffix}.pe.bin"),
                           **extra)
                tb.test("capture").add_config(
                    name=f"{stem}.{suffix}", generics=cfg)
        else:
            cfg = dict(base,
                       PHASE_ERR_DUMP = str(DERIVED_DIR / f"{stem}.pe.bin"))
            tb.test("capture").add_config(name=stem, generics=cfg)

        # DCF77_CAPTURE_NOISE_REF=1 registers a noise-reference capture
        # using this recording but with the NCO de-tuned to a signal-
        # free part of the IF spectrum.  Default 1750 Hz; overridable
        # via DCF77_CAPTURE_NOISE_HZ.  Files: <stem>.noise<HZ>.pe.bin.
        if os.environ.get("DCF77_CAPTURE_NOISE_REF") and stem.startswith("dcf_3_5"):
            noise_hz = int(os.environ.get("DCF77_CAPTURE_NOISE_HZ", "1750"))
            noise_npi = int(round(noise_hz * (1 << 25) / rate))
            ncfg = dict(base,
                        NCO_PHASE_INCR_OVERRIDE = noise_npi,
                        PHASE_ERR_DUMP = str(
                            DERIVED_DIR / f"{stem}.noise{noise_hz}.pe.bin"))
            tb.test("capture").add_config(
                name=f"{stem}.noise{noise_hz}", generics=ncfg)


def _configure_pm_corr_replay_testbench(lib) -> None:
    """Register PM-correlator standalone replay configurations.

    For each `<stem>.pe.bin` in derived/ (produced by 14_pm with
    PHASE_ERR_DUMP=1), register one `pm_smoke` configuration on
    dcf77_pm_correlator_replay_tb.  Sweepable PHASE_ERR_V_CLOCKS via
    DCF77_PM_PERIOD_SWEEP="N1,N2,..." (default 48 = current TB rate).
    """
    try:
        tb = lib.test_bench("dcf77_pm_correlator_replay_tb")
    except KeyError:
        return

    if not DERIVED_DIR.is_dir():
        return

    sweep = os.environ.get("DCF77_PM_PERIOD_SWEEP", "")
    if sweep:
        period_list = [int(s) for s in sweep.split(",") if s.strip()]
    else:
        period_list = [48]

    # DCF77_PM_REPLAY_LIMIT caps the number of baseband samples replayed.
    # Useful for the slow-rate sweep points where the full file would be
    # infeasible (e.g. PHASE_ERR_V_CLOCKS=31740 means 8x more sim time
    # per sample than the default 48).  Set to e.g. 240000 (= 120
    # broadcast-sec = 2 full SIK minute cycles) for the buffer-race test.
    limit_str = os.environ.get("DCF77_PM_REPLAY_LIMIT", "")
    sample_limit = int(limit_str) if limit_str else 0

    for pe_bin in sorted(DERIVED_DIR.glob("*.pe.bin")):
        stem    = pe_bin.name[:-len(".pe.bin")]
        size_b  = pe_bin.stat().st_size
        n_samp  = size_b // 2
        if sample_limit and sample_limit < n_samp:
            n_samp = sample_limit

        for period in period_list:
            cfg = dict(
                REPLAY_FILE         = str(pe_bin),
                SAMPLE_COUNT        = n_samp,
                PHASE_ERR_V_CLOCKS  = period,
                EXPECT_PM_VALID_MIN = 30,
            )
            if os.environ.get("DCF77_PM_ACQUIRE_LOG") == "1":
                cfg["ACQUIRE_LOG_CSV"] = str(
                    DERIVED_DIR / f"{stem}.pmreplay.p{period}.acquire.csv")
            name = stem if len(period_list) == 1 else f"{stem}.p{period}"
            tb.test("pm_smoke").add_config(name=name, generics=cfg)


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
