#!/usr/bin/env python3
"""Convert a real-valued WAV recording to complex baseband I/Q at 1 kS/s.

Input  : mono 16-b PCM WAV with the DCF77 signal at a known IF
         (default 5 kHz; matches typical USB-baseband recordings).
Output : little-endian interleaved int16 I,Q pairs at 1 kS/s, in a .s16
         file consumed by simulation/tb/06_dcf77_rx_replay_tb.vhd, plus a
         .meta sidecar carrying stream parameters and (when the decoder
         locked) the recovered timestamp.

Pipeline (per WAV):
    1. read mono int16  -> float64 in [-1, 1)
    2. multiply by complex LO exp(-j*2pi*f_if*t)
    3. FIR lowpass on each of I and Q  (60 Hz cutoff)
    4. decimate to 1000 Hz with resample_poly(up=1, down=fs_in/1000)
    5. joint peak-normalize I,Q to INT16_PEAK = 16384  (6 dB headroom)
    6. invoke dcf77_soft_decode to recover ground truth
    7. interleave I,Q -> int16 LE  -> .s16
    8. write .meta

Tolerance: as long as the residual LO error after step 2 is below the
60 Hz LPF cutoff, envelope magnitude is preserved (|I+jQ| is rotation-
invariant).  Real-receiver drifts of a few Hz are absorbed.

Idempotency: --all skips a WAV whose .s16 and .meta are both newer than
the source.  --force overrides.

Usage:
    # single-file
    python3 simulation/scripts/wav_to_iq.py \\
        --in  samples/dcf_strong.wav \\
        --out simulation/derived/dcf_strong.s16 \\
        --meta simulation/derived/dcf_strong.meta

    # process every samples/*.wav
    python3 simulation/scripts/wav_to_iq.py --all
"""
from __future__ import annotations

import argparse
import math
import wave
from pathlib import Path

import numpy as np
from scipy.signal import firwin, fftconvolve, resample_poly

from dcf77_soft_decode import decode_iq, summarize


FS_IQ_HZ        = 1_000          # output rate
F_IF_DEFAULT_HZ = 5_000.0        # carrier position in the input baseband
LPF_CUTOFF_HZ   = 60.0           # post-mix lowpass, well below f_if image
LPF_NUMTAPS     = 201
INT16_PEAK      = 16_384


def read_wav_mono(path: Path) -> tuple[np.ndarray, int]:
    with wave.open(str(path), "rb") as w:
        if w.getnchannels() != 1:
            raise ValueError(f"{path}: expected mono WAV, got "
                             f"{w.getnchannels()} channels")
        if w.getsampwidth() != 2:
            raise ValueError(f"{path}: expected 16-b WAV, got "
                             f"{w.getsampwidth() * 8} b")
        fs = w.getframerate()
        n  = w.getnframes()
        raw = w.readframes(n)
    x = np.frombuffer(raw, dtype=np.int16).astype(np.float64) / 32_768.0
    return x, fs


def downconvert(x: np.ndarray, fs: int, f_if_hz: float) -> np.ndarray:
    t = np.arange(len(x), dtype=np.float64) / fs
    lo = np.exp(-1j * 2.0 * math.pi * f_if_hz * t)
    return x.astype(np.complex128) * lo


def lpf(x: np.ndarray, fs: int) -> np.ndarray:
    taps = firwin(LPF_NUMTAPS, cutoff=LPF_CUTOFF_HZ / (fs / 2.0),
                  window="hamming")
    return fftconvolve(x.real, taps, mode="same") \
        + 1j * fftconvolve(x.imag, taps, mode="same")


def decimate_to_1k(x: np.ndarray, fs_in: int) -> np.ndarray:
    """Resample to FS_IQ_HZ.  Integer-multiple inputs use a clean
    decimation; non-multiple rates fall back to a rational approximation
    (limit_denominator=2048) and accept up to 2 % rate error."""
    if fs_in == FS_IQ_HZ:
        return x
    if fs_in % FS_IQ_HZ == 0:
        down = fs_in // FS_IQ_HZ
        return resample_poly(x, up=1, down=down)
    from fractions import Fraction
    ratio = Fraction(FS_IQ_HZ, fs_in).limit_denominator(2048)
    up, dn = ratio.numerator, ratio.denominator
    err = abs(up * fs_in - dn * FS_IQ_HZ) / (dn * FS_IQ_HZ)
    if err > 0.02:
        raise ValueError(
            f"decimate_to_1k: rational approx {up}/{dn} for "
            f"{FS_IQ_HZ}/{fs_in} is {err*100:.2f} % off (>2 % limit)")
    return resample_poly(x, up=up, down=dn)


def to_int16_pair(iq: np.ndarray) -> np.ndarray:
    peak = float(max(np.max(np.abs(iq.real)), np.max(np.abs(iq.imag)))) or 1.0
    i16 = np.clip(iq.real / peak * INT16_PEAK, -32_768, 32_767).astype(np.int16)
    q16 = np.clip(iq.imag / peak * INT16_PEAK, -32_768, 32_767).astype(np.int16)
    interleaved = np.empty(2 * len(iq), dtype=np.int16)
    interleaved[0::2] = i16
    interleaved[1::2] = q16
    return interleaved


def write_meta(path: Path, *,
               sample_count: int, source_wav: str, if_hz: float,
               decoded: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    expect_full = 1 if decoded["locked"] else 0
    sig_present = 1 if decoded["signal_present"] else 0
    lines = [
        f"sample_count={sample_count}",
        f"sample_rate_hz={FS_IQ_HZ}",
        f"iq_format=interleaved_le_int16",
        f"if_hz={if_hz:g}",
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


def read_expect(path: Path) -> dict:
    out = {}
    for line in path.read_text().splitlines():
        line = line.strip()
        if not line or line.startswith("#") or "=" not in line:
            continue
        k, v = line.split("=", 1)
        out[k.strip()] = v.strip()
    return out


def maybe_warn_mismatch(decoded: dict, expect_sidecar: Path | None) -> None:
    if expect_sidecar is None or not expect_sidecar.exists():
        return
    if not decoded["locked"]:
        print(f"  WARNING: decoder did not lock but {expect_sidecar.name} exists")
        return
    expect = read_expect(expect_sidecar)
    for key in ("year", "month", "day", "weekday", "hour", "minute", "dst_active"):
        if int(expect.get(key, -1)) != decoded[key]:
            print(f"  WARNING: decoder/{expect_sidecar.name} mismatch on "
                  f"{key}: decoded={decoded[key]} expected={expect.get(key)}")


def up_to_date(wav: Path, s16: Path, meta: Path) -> bool:
    if not (s16.exists() and meta.exists()):
        return False
    return min(s16.stat().st_mtime, meta.stat().st_mtime) >= wav.stat().st_mtime


def process_wav(wav: Path, s16: Path, meta: Path, *,
                if_hz: float, expect_sidecar: Path | None) -> None:
    print(f"[wav_to_iq] {wav.name}")
    x_in, fs_in = read_wav_mono(wav)
    print(f"  read {len(x_in)} samples @ {fs_in} Hz ({len(x_in) / fs_in:.2f} s)")

    x_bb  = downconvert(x_in, fs_in, if_hz)
    x_lpf = lpf(x_bb, fs_in)
    iq    = decimate_to_1k(x_lpf, fs_in)
    print(f"  decimated to {FS_IQ_HZ} Hz: {len(iq)} I/Q pairs")

    decoded = decode_iq(iq)
    print(f"  {summarize(decoded)}")
    maybe_warn_mismatch(decoded, expect_sidecar)

    packed = to_int16_pair(iq)
    s16.parent.mkdir(parents=True, exist_ok=True)
    packed.tofile(s16)
    write_meta(meta, sample_count=len(iq),
               source_wav=wav.name, if_hz=if_hz, decoded=decoded)
    print(f"  wrote {s16.name} ({s16.stat().st_size} B) and {meta.name}")


def process_all(samples_dir: Path, derived_dir: Path, *,
                if_hz: float, force: bool) -> None:
    derived_dir.mkdir(parents=True, exist_ok=True)
    wavs = sorted(samples_dir.glob("*.wav"))
    if not wavs:
        print(f"[wav_to_iq] no WAV files in {samples_dir}")
        return
    for wav in wavs:
        s16  = derived_dir / f"{wav.stem}.s16"
        meta = derived_dir / f"{wav.stem}.meta"
        if not force and up_to_date(wav, s16, meta):
            print(f"[wav_to_iq] {wav.name}: up-to-date, skipping")
            continue
        expect = samples_dir / f"{wav.stem}.expect"
        process_wav(wav, s16, meta, if_hz=if_hz,
                    expect_sidecar=expect if expect.exists() else None)


def main() -> int:
    p = argparse.ArgumentParser(
        description="WAV -> complex I/Q 1 kS/s for the DCF77 replay TB.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    g = p.add_argument_group("single-file mode")
    g.add_argument("--in", dest="wav_in", type=Path)
    g.add_argument("--out", type=Path)
    g.add_argument("--meta", type=Path)
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
        print(f"[wav_to_iq] {args.wav_in.name}: up-to-date, skipping "
              f"(use --force to override)")
        return 0

    process_wav(args.wav_in, args.out, args.meta,
                if_hz=args.if_hz, expect_sidecar=args.expect)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
