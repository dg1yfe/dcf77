#!/usr/bin/env python3
"""Convert a real-valued WAV recording to single-channel IF samples at 12 kS/s.

The IF replay testbench (simulation/tb/07_dcf77_if_replay_tb.vhd) feeds
these samples directly into the receiver's mixer with the NCO tuned to
the WAV's IF frequency at 12 kS/s.  This exercises the full carrier-PLL
loop closure -- something the baseband replay (simulation/tb/06_*) can't
do because its input has already been mixed down to DC at preprocessor
time.

Pipeline (per WAV):
    1. read mono 16-b PCM WAV at any sample rate
    2. resample (rational up/down via resample_poly) to FS_IF_HZ = 12_000
       so the DCF77 IF survives in-band (fs_in must satisfy
       fs_in >= 2 * if_hz for valid Nyquist coverage of the IF)
    3. joint peak-normalize the real-valued stream to INT16_PEAK = 16384
       (6 dB headroom)
    4. ALSO derive a 1 kS/s baseband I/Q stream (same path as
       wav_to_iq.py) and run dcf77_soft_decode on it to recover the
       expected timestamp for the testbench .meta sidecar
    5. write the real-valued int16 LE samples to .if.s16
    6. write a .if.meta carrying stream parameters and decoded timestamp

Idempotency: --all skips a WAV whose .if.s16 and .if.meta are both newer
than the source.  --force overrides.
"""
from __future__ import annotations

import argparse
import math
import wave
from fractions import Fraction
from pathlib import Path

import numpy as np
from scipy.signal import resample_poly

from dcf77_soft_decode import decode_iq, summarize

# Re-use the baseband helpers from wav_to_iq.py to keep the soft-decode
# path identical between the two flows.
from wav_to_iq import (
    INT16_PEAK,
    F_IF_DEFAULT_HZ,
    read_wav_mono,
    downconvert,
    lpf,
    decimate_to_1k,
    read_expect,
    maybe_warn_mismatch,
)


FS_IF_HZ = 12_000          # output rate for the IF replay TB


def resample_to_if(x: np.ndarray, fs_in: int,
                   fs_out: int = FS_IF_HZ) -> np.ndarray:
    """Rational resample fs_in -> fs_out.  Validates Nyquist for the IF."""
    if fs_in == fs_out:
        return x.copy()
    ratio = Fraction(fs_out, fs_in).limit_denominator(1024)
    up, dn = ratio.numerator, ratio.denominator
    if up * fs_in != dn * fs_out:
        print(f"  WARNING: rational approx {up}/{dn} != {fs_out}/{fs_in} "
              f"exactly; effective rate "
              f"{up * fs_in / dn:.3f} Hz")
    return resample_poly(x, up=up, down=dn)


def to_int16_real(x: np.ndarray) -> np.ndarray:
    peak = float(np.max(np.abs(x))) or 1.0
    return np.clip(x / peak * INT16_PEAK, -32_768, 32_767).astype(np.int16)


def write_if_meta(path: Path, *,
                  sample_count: int,
                  source_wav: str,
                  fs_in_hz: int,
                  if_hz: float,
                  decoded: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    expect_full = 1 if decoded["locked"] else 0
    sig_present = 1 if decoded["signal_present"] else 0
    lines = [
        f"sample_count={sample_count}",
        f"sample_rate_hz={FS_IF_HZ}",
        f"format=int16_le_real",
        f"if_hz={if_hz:g}",
        f"fs_in_hz={fs_in_hz}",
        f"source_wav={source_wav}",
        f"signal_present={sig_present}",
        f"expect_full_decode={expect_full}",
        f"year={decoded['year']}",
        f"month={decoded['month']}",
        f"day={decoded['day']}",
        f"weekday={decoded['weekday']}",
        f"hour={decoded['hour']}",
        f"minute={decoded['minute']}",
        f"dst_active={decoded['dst_active']}",
    ]
    path.write_text("\n".join(lines) + "\n")


def up_to_date(wav: Path, if_s16: Path, if_meta: Path) -> bool:
    if not (if_s16.exists() and if_meta.exists()):
        return False
    return min(if_s16.stat().st_mtime,
               if_meta.stat().st_mtime) >= wav.stat().st_mtime


def process_wav(wav: Path, if_s16: Path, if_meta: Path, *,
                if_hz: float, expect_sidecar: Path | None) -> None:
    print(f"[wav_to_if] {wav.name}")
    x_in, fs_in = read_wav_mono(wav)
    print(f"  read {len(x_in)} samples @ {fs_in} Hz "
          f"({len(x_in) / fs_in:.2f} s)")

    if fs_in < 2 * if_hz:
        raise ValueError(f"fs_in={fs_in} below Nyquist for IF={if_hz}")

    x_if = resample_to_if(x_in, fs_in)
    print(f"  resampled to {FS_IF_HZ} Hz: {len(x_if)} samples "
          f"({len(x_if) / FS_IF_HZ:.2f} s)")

    # Soft-decode via the existing baseband path to recover expected time.
    x_bb  = downconvert(x_in, fs_in, if_hz)
    x_lpf = lpf(x_bb, fs_in)
    iq    = decimate_to_1k(x_lpf, fs_in)
    decoded = decode_iq(iq)
    print(f"  {summarize(decoded)}")
    maybe_warn_mismatch(decoded, expect_sidecar)

    packed = to_int16_real(x_if)
    if_s16.parent.mkdir(parents=True, exist_ok=True)
    packed.tofile(if_s16)
    write_if_meta(if_meta, sample_count=len(x_if),
                  source_wav=wav.name, fs_in_hz=fs_in,
                  if_hz=if_hz, decoded=decoded)
    print(f"  wrote {if_s16.name} ({if_s16.stat().st_size} B) "
          f"and {if_meta.name}")


def process_all(samples_dir: Path, derived_dir: Path, *,
                if_hz: float, force: bool) -> None:
    derived_dir.mkdir(parents=True, exist_ok=True)
    wavs = sorted(samples_dir.glob("*.wav"))
    if not wavs:
        print(f"[wav_to_if] no WAV files in {samples_dir}")
        return
    for wav in wavs:
        if_s16  = derived_dir / f"{wav.stem}.if.s16"
        if_meta = derived_dir / f"{wav.stem}.if.meta"
        if not force and up_to_date(wav, if_s16, if_meta):
            print(f"[wav_to_if] {wav.name}: up-to-date, skipping")
            continue
        expect = samples_dir / f"{wav.stem}.expect"
        process_wav(wav, if_s16, if_meta, if_hz=if_hz,
                    expect_sidecar=expect if expect.exists() else None)


def main() -> int:
    p = argparse.ArgumentParser(
        description="WAV -> 12 kS/s IF samples for the DCF77 IF replay TB.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    g = p.add_argument_group("single-file mode")
    g.add_argument("--in", dest="wav_in", type=Path)
    g.add_argument("--out", type=Path,
                   help="output .if.s16 file")
    g.add_argument("--meta", type=Path,
                   help="output .if.meta sidecar")
    g.add_argument("--expect", type=Path,
                   help=".expect sidecar to cross-check the decoder against")

    g2 = p.add_argument_group("batch mode")
    g2.add_argument("--all", action="store_true",
                    help="process every samples/*.wav into simulation/derived/")
    g2.add_argument("--samples-dir", type=Path,
                    default=Path(__file__).resolve().parent.parent.parent
                    / "samples")
    g2.add_argument("--derived-dir", type=Path,
                    default=Path(__file__).resolve().parent.parent
                    / "derived")

    p.add_argument("--if-hz", type=float, default=F_IF_DEFAULT_HZ,
                   help=f"IF position in the input WAV (default {F_IF_DEFAULT_HZ:g})")
    p.add_argument("--force", action="store_true",
                   help="re-process even if outputs are up-to-date")
    args = p.parse_args()

    if args.all:
        process_all(args.samples_dir, args.derived_dir,
                    if_hz=args.if_hz, force=args.force)
        return 0

    if not (args.wav_in and args.out and args.meta):
        p.error("single-file mode requires --in/--out/--meta")

    if not args.force and up_to_date(args.wav_in, args.out, args.meta):
        print(f"[wav_to_if] {args.wav_in.name}: up-to-date, skipping "
              f"(use --force to override)")
        return 0

    process_wav(args.wav_in, args.out, args.meta,
                if_hz=args.if_hz, expect_sidecar=args.expect)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
