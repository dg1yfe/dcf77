#!/usr/bin/env python3
"""Convert a DCF77 USB-baseband WAV recording to the post-CIC injection format.

Pipeline:
    WAV (mono, real-valued)                  fs_in  Hz
        -> resample to 244 140 Hz            fs_out
        -> frequency shift +72.5 kHz (cos mixer)
        -> bandpass around 77.5 kHz (suppress the 67.5 kHz image)
        -> scale to int16 with 6 dB headroom
        -> little-endian binary

The output `*.s16` file is consumed by simulation/tb/06_dcf77_rx_replay_tb.vhd
which feeds it through the mixer (the recording is real-valued; the mixer
+ NCO drop it to baseband I/Q).  A companion `*.meta` text sidecar
documents the expected decoded time -- the VHDL testbench currently gets
the expected fields via VUnit generics, but the sidecar is human-readable
and serves as the canonical record of what the recording should decode to.

Usage:
    python3 simulation/scripts/wav_to_postcic.py \\
        --in  simulation/derived/synth.wav \\
        --out simulation/derived/synth.s16 \\
        --meta simulation/derived/synth.meta \\
        --year 26 --month 5 --day 13 --weekday 3 \\
        --hour 11 --minute 42 --dst 1
"""
from __future__ import annotations

import argparse
import math
import wave
from pathlib import Path

import numpy as np
from scipy.signal import firwin, fftconvolve, resample_poly


FS_OUT_HZ      = 250_000        # post-CIC tick rate = 125 MHz / 500
F_DCF77_HZ     = 77_500         # DCF77 carrier
F_BB_CARRIER   = 5_000          # carrier position in USB baseband (LO 72.5 kHz)
F_SHIFT_HZ     = F_DCF77_HZ - F_BB_CARRIER   # = 72 500 Hz
INT16_PEAK     = 16_384         # ~6 dB headroom under +/-32 768


def read_wav_mono(path: Path) -> tuple[np.ndarray, int]:
    """Return (samples_as_float64, sample_rate)."""
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


def resample_to(x: np.ndarray, fs_in: int, fs_out: int) -> np.ndarray:
    """Polyphase resampler.  Computes up/down from gcd(fs_in, fs_out)."""
    g = math.gcd(fs_in, fs_out)
    up   = fs_out // g
    down = fs_in  // g
    return resample_poly(x, up, down)


def frequency_shift(x: np.ndarray, fs: int, f_shift_hz: float) -> np.ndarray:
    """Real-valued frequency shift via cos-multiplication.

    Produces sum and difference components; the caller must bandpass-filter
    to keep only the desired sideband (77.5 kHz in our case).
    """
    n = len(x)
    t = np.arange(n) / fs
    return x * np.cos(2.0 * math.pi * f_shift_hz * t)


def bandpass_77_5(x: np.ndarray, fs: int) -> np.ndarray:
    """FIR bandpass with passband 74.5..80.5 kHz (6 kHz around 77.5 kHz)."""
    nyq = fs / 2.0
    taps = firwin(
        numtaps=201,
        cutoff=[74_500.0 / nyq, 80_500.0 / nyq],
        pass_zero=False,
        window="hamming",
    )
    return fftconvolve(x, taps, mode="same")


def to_int16(x: np.ndarray) -> np.ndarray:
    """Scale to int16 with 6 dB headroom (peak ~= +/-16384)."""
    peak = float(np.max(np.abs(x))) or 1.0
    scaled = x / peak * INT16_PEAK
    return np.clip(scaled, -32_768, 32_767).astype(np.int16)


def write_meta(path: Path, sample_count: int, args: argparse.Namespace) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        f"sample_count={sample_count}\n"
        f"sample_rate_hz={FS_OUT_HZ}\n"
        f"year={args.year}\n"
        f"month={args.month}\n"
        f"day={args.day}\n"
        f"weekday={args.weekday}\n"
        f"hour={args.hour}\n"
        f"minute={args.minute}\n"
        f"dst_active={args.dst}\n"
    )


def main() -> int:
    p = argparse.ArgumentParser(
        description="WAV -> post-CIC int16 stream for the DCF77 replay TB."
    )
    p.add_argument("--in",  dest="wav_in",  required=True, type=Path)
    p.add_argument("--out",                  required=True, type=Path)
    p.add_argument("--meta",                 required=True, type=Path)
    p.add_argument("--year",    type=int, required=True)
    p.add_argument("--month",   type=int, required=True)
    p.add_argument("--day",     type=int, required=True)
    p.add_argument("--weekday", type=int, required=True)
    p.add_argument("--hour",    type=int, required=True)
    p.add_argument("--minute",  type=int, required=True)
    p.add_argument("--dst",     type=int, required=True)
    args = p.parse_args()

    x_in, fs_in = read_wav_mono(args.wav_in)
    print(f"read {args.wav_in}: {len(x_in)} samples @ {fs_in} Hz "
          f"({len(x_in) / fs_in:.2f} s)")

    x_rs = resample_to(x_in, fs_in, FS_OUT_HZ)
    print(f"resampled to {FS_OUT_HZ} Hz: {len(x_rs)} samples")

    x_shifted = frequency_shift(x_rs, FS_OUT_HZ, F_SHIFT_HZ)
    x_bp      = bandpass_77_5(x_shifted, FS_OUT_HZ)
    x_i16     = to_int16(x_bp)

    args.out.parent.mkdir(parents=True, exist_ok=True)
    x_i16.tofile(args.out)
    write_meta(args.meta, len(x_i16), args)

    print(f"wrote {args.out}: {len(x_i16)} samples ({args.out.stat().st_size} B)")
    print(f"wrote {args.meta}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
