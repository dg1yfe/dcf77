#!/usr/bin/env python3
"""Synthesise a DCF77 USB-baseband WAV file for the replay testbench.

The output mimics a shortwave receiver tuned to 72.5 kHz with ~6 kHz
bandwidth: the DCF77 carrier (77.5 kHz) appears at 5 kHz in the real-
valued mono baseband.  Used as a permanent self-test fixture for the
replay pipeline -- the real-recording test case feeds whatever WAV the
user drops in, but this script lets us validate everything without a
shortwave receiver in the loop.

The amplitude modulation matches doc/dcf77_format_spec.md:
* full carrier from second N + pulse_width to second N+1
* absenkung from second N to second N + (0.1 s for bit 0, 0.2 s for bit 1)
* no absenkung at second 59 (minute marker)

Usage:
    python3 simulation/scripts/synth_dcf77_wav.py \\
        --out simulation/derived/synth.wav \\
        --year 26 --month 5 --day 13 --weekday 3 \\
        --hour 11 --minute 42 --dst 1 --duration 65
"""
from __future__ import annotations

import argparse
import math
import wave
from pathlib import Path

import numpy as np


# DCF77 BCD bit weights, LSB-first (per doc/dcf77_format_spec.md §1.3).
W_MIN = [1, 2, 4, 8, 10, 20, 40]
W_HR  = [1, 2, 4, 8, 10, 20]
W_DAY = [1, 2, 4, 8, 10, 20]
W_WD  = [1, 2, 4]
W_MON = [1, 2, 4, 8, 10]
W_YR  = [1, 2, 4, 8, 10, 20, 40, 80]


def bcd_encode(value: int, weights: list[int]) -> list[int]:
    """Encode `value` LSB-first using DCF77 bit-weights.

    Greedy from the highest weight downwards; the DCF77 representation is
    unique inside the valid ranges in §1.4 of the spec.
    """
    bits = [0] * len(weights)
    remaining = value
    for k in range(len(weights) - 1, -1, -1):
        if remaining >= weights[k]:
            bits[k] = 1
            remaining -= weights[k]
    return bits


def build_frame(year: int, month: int, day: int, weekday: int,
                hour: int, minute: int, dst_active: bool) -> list[int]:
    """Return the 59-bit DCF77 frame for the given minute as a list of 0/1."""
    f = [0] * 59
    f[0]  = 0
    f[15] = 0                       # call bit
    f[16] = 0                       # DST announce
    f[17] = 1 if dst_active else 0  # MESZ
    f[18] = 0 if dst_active else 1  # MEZ (invariant: 17 XOR 18 = 1)
    f[19] = 0                       # no leap second
    f[20] = 1                       # start of time info

    enc_min = bcd_encode(minute, W_MIN)
    for i, b in enumerate(enc_min):
        f[21 + i] = b
    f[28] = sum(enc_min) % 2

    enc_hr = bcd_encode(hour, W_HR)
    for i, b in enumerate(enc_hr):
        f[29 + i] = b
    f[35] = sum(enc_hr) % 2

    enc_day = bcd_encode(day, W_DAY)
    for i, b in enumerate(enc_day):
        f[36 + i] = b
    enc_wd = bcd_encode(weekday, W_WD)
    for i, b in enumerate(enc_wd):
        f[42 + i] = b
    enc_mon = bcd_encode(month, W_MON)
    for i, b in enumerate(enc_mon):
        f[45 + i] = b
    enc_yr = bcd_encode(year, W_YR)
    for i, b in enumerate(enc_yr):
        f[50 + i] = b
    f[58] = sum(f[36:58]) % 2       # even parity over bits 36..58

    return f


def synth_baseband(frame: list[int],
                   duration_s: float,
                   fs: int,
                   f_carrier: float = 5_000.0,
                   amp_high: float = 0.80,
                   amp_low: float  = 0.12) -> np.ndarray:
    """Build a real-valued USB-baseband signal of length `duration_s`.

    Frame replay begins at t = 0 (second 0 of the minute the frame
    represents).  If the recording goes past t = 60 s, a bit-0-style 100 ms
    absenkung is added at t = 60 s as the first pulse of the NEXT minute
    (so the decoder sees the minute-mark gap close out into a fresh pulse).
    """
    n = int(round(duration_s * fs))
    t = np.arange(n) / fs

    # Default amplitude is the carrier-on level.
    amp = np.full(n, amp_high, dtype=np.float64)

    # 59 second-aligned pulses per minute; index 59 is the minute marker.
    for sec, bit in enumerate(frame):
        start = int(round(sec * fs))
        if start >= n:
            break
        pulse_s = 0.1 if bit == 0 else 0.2
        end = min(start + int(round(pulse_s * fs)), n)
        amp[start:end] = amp_low

    # First pulse of the next minute (bit 0 of that minute, treat as 0).
    if duration_s > 60.0:
        start = int(round(60.0 * fs))
        if start < n:
            end = min(start + int(round(0.1 * fs)), n)
            amp[start:end] = amp_low

    # Modulate a sinusoid at the baseband carrier (DCF77 = LO + 5 kHz).
    return amp * np.cos(2.0 * math.pi * f_carrier * t)


def main() -> int:
    p = argparse.ArgumentParser(
        description="Synthesise a DCF77 USB-baseband WAV file."
    )
    p.add_argument("--out", required=True, type=Path)
    p.add_argument("--fs", type=int, default=16_000,
                   help="WAV sample rate (Hz); default 16 000")
    p.add_argument("--duration", type=float, default=65.0,
                   help="Recording length in seconds")
    p.add_argument("--year",    type=int, default=26)
    p.add_argument("--month",   type=int, default=5)
    p.add_argument("--day",     type=int, default=13)
    p.add_argument("--weekday", type=int, default=3, help="1=Mon..7=Sun")
    p.add_argument("--hour",    type=int, default=11)
    p.add_argument("--minute",  type=int, default=42)
    p.add_argument("--dst",     type=int, default=1,
                   help="1 = MESZ summer, 0 = MEZ winter")
    args = p.parse_args()

    frame = build_frame(args.year, args.month, args.day, args.weekday,
                        args.hour, args.minute, bool(args.dst))
    sig = synth_baseband(frame, args.duration, args.fs)

    peak = float(np.max(np.abs(sig))) or 1.0
    sig_i16 = np.clip(sig / peak * 16_384.0, -32_768, 32_767).astype(np.int16)

    args.out.parent.mkdir(parents=True, exist_ok=True)
    with wave.open(str(args.out), "wb") as w:
        w.setnchannels(1)
        w.setsampwidth(2)
        w.setframerate(args.fs)
        w.writeframes(sig_i16.tobytes())

    print(f"wrote {args.out}: {len(sig_i16)} samples at {args.fs} Hz "
          f"({args.duration:.2f} s)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
