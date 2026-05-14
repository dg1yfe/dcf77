#!/usr/bin/env python3
"""Software DCF77 AM decoder consuming complex baseband at 1 kS/s.

Mirrors the FPGA bit_det + frame_dec logic so the preprocessor can
recover the timestamp embedded in a WAV recording.  The result is the
ground-truth reference the VHDL replay testbench validates against;
caching it in the .meta file means the decoder runs once per WAV, not
every test invocation.

Algorithm:
    1. envelope = |I + jQ|   (rotation-invariant: tolerates LO offset)
    2. adaptive peak (fast attack, slow decay) -> threshold at half peak
    3. measure low-pulse widths in samples (= ms at 1 kS/s)
    4. 70..130 ms -> bit 0;  170..230 ms -> bit 1;  >=1500 ms gap -> minute mark
    5. once 59 valid bits since the last minute mark, BCD-decode the
       fields per doc/dcf77_format_spec.md and check the three parity bits
    6. require two consecutive valid frames with minute incrementing
       by one (mod 60) before declaring "decoded" -- filters single-bit
       errors on weak recordings

Return value of decode_iq() is a dict with the time fields plus the
boolean `locked` (True when two-frame agreement was reached).
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable

import numpy as np


SAMPLE_RATE_HZ      = 1000
T_LOW_MIN_MS        = 70
T_BIT0_MAX_MS       = 130
T_BIT1_MIN_MS       = 170
T_BIT1_MAX_MS       = 230
T_GAP_MINUTE_MS     = 1500

PEAK_ATTACK_SHIFT   = 3       # alpha ~ 1/8 -- fast follow up
PEAK_DECAY_SHIFT    = 10      # alpha ~ 1/1024 -- slow trickle down (~1 s)
THRESHOLD_FRAC      = 0.5     # threshold = peak * 0.5


W_MIN = [1, 2, 4, 8, 10, 20, 40]
W_HR  = [1, 2, 4, 8, 10, 20]
W_DAY = [1, 2, 4, 8, 10, 20]
W_WD  = [1, 2, 4]
W_MON = [1, 2, 4, 8, 10]
W_YR  = [1, 2, 4, 8, 10, 20, 40, 80]


def _bcd_decode(bits: list[int], weights: list[int]) -> int:
    return sum(b * w for b, w in zip(bits, weights))


@dataclass
class FrameDecode:
    year: int
    month: int
    day: int
    weekday: int
    hour: int
    minute: int
    dst_active: int
    parity_ok: bool

    def plausible(self) -> bool:
        return (
            self.parity_ok
            and 0 <= self.minute <= 59
            and 0 <= self.hour <= 23
            and 1 <= self.day <= 31
            and 1 <= self.month <= 12
            and 0 <= self.year <= 99
            and 1 <= self.weekday <= 7
            and self.dst_active in (0, 1)
        )


def _decode_frame(bits: list[int]) -> FrameDecode | None:
    if len(bits) != 59:
        return None
    if bits[0] != 0 or bits[20] != 1:
        return None
    if (bits[17] ^ bits[18]) != 1:
        return None

    p_min  = (sum(bits[21:28]) + bits[28]) % 2 == 0
    p_hour = (sum(bits[29:35]) + bits[35]) % 2 == 0
    p_date = (sum(bits[36:58]) + bits[58]) % 2 == 0

    minute  = _bcd_decode(bits[21:28], W_MIN)
    hour    = _bcd_decode(bits[29:35], W_HR)
    day     = _bcd_decode(bits[36:42], W_DAY)
    weekday = _bcd_decode(bits[42:45], W_WD)
    month   = _bcd_decode(bits[45:50], W_MON)
    year    = _bcd_decode(bits[50:58], W_YR)
    dst     = bits[17]

    return FrameDecode(
        year=year, month=month, day=day, weekday=weekday,
        hour=hour, minute=minute, dst_active=dst,
        parity_ok=(p_min and p_hour and p_date),
    )


def _detect_pulses(env: np.ndarray) -> list[tuple[int, int]]:
    """Return (low_start_idx, low_length_samples) pairs.

    Threshold = THRESHOLD_FRAC * adaptive peak; peak is a fast-attack /
    slow-decay tracker rebuilt by running the entire stream in one pass.
    """
    n = len(env)
    peak = float(env[: min(1000, n)].max()) if n else 0.0
    if peak <= 0:
        return []

    atk = 1.0 / (1 << PEAK_ATTACK_SHIFT)
    dec = 1.0 / (1 << PEAK_DECAY_SHIFT)

    pulses: list[tuple[int, int]] = []
    state_low = False
    low_start = 0

    for i in range(n):
        sample = float(env[i])
        if sample > peak:
            peak += atk * (sample - peak)
        else:
            peak -= dec * peak
        thresh = THRESHOLD_FRAC * peak
        is_low = sample < thresh

        if is_low and not state_low:
            state_low = True
            low_start = i
        elif not is_low and state_low:
            state_low = False
            pulses.append((low_start, i - low_start))

    if state_low:
        pulses.append((low_start, n - low_start))

    return pulses


def _classify(low_len_samples: int) -> int | None:
    ms = low_len_samples * 1000 // SAMPLE_RATE_HZ
    if T_LOW_MIN_MS <= ms <= T_BIT0_MAX_MS:
        return 0
    if T_BIT1_MIN_MS <= ms <= T_BIT1_MAX_MS:
        return 1
    return None


def decode_iq(iq: np.ndarray) -> dict:
    """Decode complex baseband samples at 1 kS/s.

    Returns a dict with year/month/day/weekday/hour/minute/dst_active and
    a `locked` boolean (True iff two consecutive frames with minute n,
    n+1 mod 60 were decoded).  Time fields are zero when not locked.
    """
    env = np.abs(iq).astype(np.float64)
    raw_pulses = _detect_pulses(env)

    # Count pulses whose width falls in a valid bit window.  A clean
    # minute carries 59 such pulses (~30 bit-0 + ~29 bit-1).  If the
    # recording is essentially noise the classifier finds none and we
    # flag the WAV as signal-absent so the test runner can skip it.
    valid_pulses = sum(1 for _, l in raw_pulses if _classify(l) is not None)
    duration_min = max(1.0, len(iq) / (60.0 * SAMPLE_RATE_HZ))
    signal_present = valid_pulses >= 20 * duration_min

    bits: list[int] = []
    last_acc_start = None
    last_acc_end   = None
    frames: list[FrameDecode] = []
    MIN_GAP_TO_LAST_ACC_MS = 700

    for start, length in raw_pulses:
        bit = _classify(length)
        if bit is None:
            continue

        if last_acc_start is not None:
            gap_acc_ms = start - last_acc_start
            if gap_acc_ms < MIN_GAP_TO_LAST_ACC_MS:
                continue
            gap_end_ms = start - last_acc_end
            if gap_end_ms >= T_GAP_MINUTE_MS:
                if len(bits) == 59:
                    fd = _decode_frame(bits)
                    if fd is not None and fd.plausible():
                        frames.append(fd)
                bits = []
        bits.append(bit)
        if len(bits) > 59:
            bits = []
        last_acc_start = start
        last_acc_end   = start + length

    if len(bits) == 59:
        fd = _decode_frame(bits)
        if fd is not None and fd.plausible():
            frames.append(fd)

    locked = _find_consecutive(frames)
    if locked is None:
        return _empty_result(signal_present=signal_present,
                              valid_pulses=valid_pulses)

    return {
        "year": locked.year,
        "month": locked.month,
        "day": locked.day,
        "weekday": locked.weekday,
        "hour": locked.hour,
        "minute": locked.minute,
        "dst_active": locked.dst_active,
        "locked": True,
        "frames_decoded": len(frames),
        "signal_present": True,
        "valid_pulses": valid_pulses,
    }


def _empty_result(signal_present: bool = False, valid_pulses: int = 0) -> dict:
    return {"year": 0, "month": 0, "day": 0, "weekday": 0,
            "hour": 0, "minute": 0, "dst_active": 0,
            "locked": False, "frames_decoded": 0,
            "signal_present": signal_present,
            "valid_pulses": valid_pulses}


def _find_consecutive(frames: list[FrameDecode]) -> FrameDecode | None:
    """Return the LATEST frame `b` of a (n, n+1 mod 60) pair `(a, b)`, if any.

    We return `b` (not `a`) so the reported timestamp matches what the FPGA's
    frame_dec holds at end-of-stream: the most recent successfully decoded
    minute.  The (a, b) consistency check filters single-bit errors before
    the value is published.
    """
    last_match: FrameDecode | None = None
    for a, b in zip(frames, frames[1:]):
        if (a.minute + 1) % 60 == b.minute \
           and a.hour == b.hour - (1 if b.minute == 0 else 0):
            last_match = b
    return last_match


def summarize(result: dict) -> str:
    if not result["signal_present"]:
        return (f"decoder: no signal detected "
                f"({result['valid_pulses']} valid pulses found)")
    if not result["locked"]:
        return (f"decoder: signal present but no lock "
                f"({result['frames_decoded']} candidate frames, "
                f"{result['valid_pulses']} valid pulses)")
    return (f"decoder: {result['year']:02d}-{result['month']:02d}-{result['day']:02d} "
            f"wd={result['weekday']} {result['hour']:02d}:{result['minute']:02d} "
            f"dst={result['dst_active']} (locked over 2 frames, "
            f"{result['valid_pulses']} valid pulses)")


if __name__ == "__main__":
    import sys
    if len(sys.argv) != 2:
        print("usage: dcf77_soft_decode.py <iq.s16>", file=sys.stderr)
        raise SystemExit(2)
    raw = np.fromfile(sys.argv[1], dtype=np.int16)
    iq = (raw[0::2].astype(np.float32) + 1j * raw[1::2].astype(np.float32))
    print(summarize(decode_iq(iq)))
