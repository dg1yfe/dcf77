#!/usr/bin/env python3
"""Software DCF77 AM decoder consuming complex baseband at 1 kS/s.

Mirrors the FPGA bit_det + frame_dec logic so the preprocessor can
recover the timestamp embedded in a WAV recording.  The result is the
ground-truth reference the VHDL replay testbench validates against;
caching it in the .meta file means the decoder runs once per WAV, not
every test invocation.

Algorithm (matched filter, 3 unit-energy templates per second):
    1. envelope = |I + jQ|          (rotation-invariant: tolerates LO offset)
    2. baseline = uniform_filter1d(env, 2 s)   tracks slow RSSI drift
       e_d      = env - baseline   (zero-mean deviation)
    3. templates (all length T = 1000 = 1 s @ 1 kS/s, unit energy):
         t_bit0 : -1 for [0:100],  +1/9 for [100:1000]  (zero mean)
         t_bit1 : -1 for [0:200],  +1/4 for [200:1000]  (zero mean)
         t_mm   : +1/sqrt(T) for the whole second        (constant)
    4. cross-correlate e_d with each template -> c_0, c_1, c_mm
    5. phase-lock the 1 Hz second grid: pick phi maximising
       sum_k max(c_0[phi+k*T], c_1[phi+k*T])
    6. at each grid point label the second as:
         BIT_0 / BIT_1   if max(c_0, c_1) > PULSE_PRESENT_FRAC * median
         MINUTE_MARK     if not a pulse AND |c_mm| < FLAT_FRAC * median
         ambiguous       otherwise (dropped from the bit buffer)
    7. walk labels: bits accumulate, MINUTE_MARK flushes the buffer
       (decoding a 59-bit frame if parity + plausibility pass).
    8. lock on the last decoded frame -- a single 59-bit frame that
       (a) starts with bit[0]=0 and bit[20]=1, (b) passes all three
       even-parity checks, and (c) is plausible (ranges of every
       field), is considered ground truth.  Matches the FPGA's
       frame_dec semantics: time_valid drops only when a fresh
       minute fails the checks.

Return value of decode_iq() is a dict with the time fields plus the
boolean `locked` (True when at least one such frame was decoded).
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from scipy.signal import fftconvolve
from scipy.ndimage import uniform_filter1d


SAMPLE_RATE_HZ      = 1000
T_PER_SECOND        = SAMPLE_RATE_HZ
T_BIT0_LOW          = 100        # samples (= ms at 1 kS/s)
T_BIT1_LOW          = 200
BASELINE_WIN        = 2 * SAMPLE_RATE_HZ
MIN_DURATION_SEC    = 120        # need at least two minutes for a lock

PULSE_ABSENT_FRAC   = 0.2        # bit_score below this fraction of the
                                  # recording's median ⇒ no pulse this second
MIN_SIGNAL_SCORE    = 1e-6       # below this the recording is just noise

LABEL_BIT0          = 0
LABEL_BIT1          = 1
LABEL_MM            = 2
LABEL_AMBIGUOUS     = -1


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


def _make_bit_template(low_samples: int) -> np.ndarray:
    """Zero-mean unit-energy template: -1 during the dip, +k during the
    carrier-on portion (k chosen so the template's mean is zero)."""
    t = np.empty(T_PER_SECOND, dtype=np.float64)
    high = low_samples / (T_PER_SECOND - low_samples)
    t[:low_samples] = -1.0
    t[low_samples:] = high
    return t / np.linalg.norm(t)


def _make_mm_template() -> np.ndarray:
    """Unit-energy constant template: 1/sqrt(T) everywhere."""
    return np.full(T_PER_SECOND, 1.0 / np.sqrt(T_PER_SECOND))


T_BIT0   = _make_bit_template(T_BIT0_LOW)
T_BIT1   = _make_bit_template(T_BIT1_LOW)
T_MM     = _make_mm_template()


def _correlate(e_d: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Cross-correlate the detrended envelope with the three templates.

    Returns (c_0, c_1, c_mm), each of length len(e_d) - T + 1, indexed by
    the template's start-of-window position.
    """
    c_0 = fftconvolve(e_d, T_BIT0[::-1], mode="valid")
    c_1 = fftconvolve(e_d, T_BIT1[::-1], mode="valid")
    # c_mm[n] = sum(e_d[n:n+T]) / sqrt(T)  (unit-energy constant template).
    # Build via cumsum so the indexing exactly matches fftconvolve('valid').
    n = len(e_d)
    csum = np.concatenate(([0.0], np.cumsum(e_d, dtype=np.float64)))
    sliding_sum = csum[T_PER_SECOND:] - csum[: n - T_PER_SECOND + 1]
    c_mm = sliding_sum / np.sqrt(T_PER_SECOND)
    return c_0, c_1, c_mm


def _find_phase(c_0: np.ndarray, c_1: np.ndarray) -> int:
    """Return the second-grid phase phi in [0, T) that maximises the
    sum of max(c_0, c_1) over all phi+k*T grid points."""
    bit_score = np.maximum(c_0, c_1)
    full = (len(bit_score) // T_PER_SECOND) * T_PER_SECOND
    if full == 0:
        return 0
    reshaped = bit_score[:full].reshape(-1, T_PER_SECOND)
    phase_scores = reshaped.sum(axis=0)
    return int(np.argmax(phase_scores))


def _decide_seconds(c_0: np.ndarray, c_1: np.ndarray, c_mm: np.ndarray,
                    phi: int) -> np.ndarray:
    """Per-second labelling.  Returns a 1-D int array of labels indexed
    by second-of-recording, drawn from {0, 1, 2}.

    Decision rule (no ambiguous category — parity + plausibility checks
    at the back filter incorrectly classified bits):
      * MINUTE_MARK if max(c_0, c_1) < PULSE_ABSENT_FRAC * its median
        AND c_mm > 0 (no pulse this second AND envelope sat above the
        local 2-s mean -- the characteristic "carrier on, no dip" gap
        at second 59).
      * BIT_x = argmax(c_0, c_1) otherwise (every other second is
        treated as a bit, with the value taken from whichever template
        scored higher).
    """
    idx = np.arange(phi, len(c_0), T_PER_SECOND)
    s0, s1, sm = c_0[idx], c_1[idx], c_mm[idx]
    bit_score = np.maximum(s0, s1)
    bit_value = (s1 > s0).astype(int)

    if bit_score.size == 0:
        return np.array([], dtype=int)

    absent_thr = PULSE_ABSENT_FRAC * np.median(bit_score)

    no_pulse = bit_score < absent_thr
    mm_here  = no_pulse & (sm > 0)

    label = bit_value.copy()
    label[mm_here] = LABEL_MM
    return label


def _walk_labels(labels: np.ndarray) -> list[FrameDecode]:
    """Reduce per-second labels to a list of plausible decoded frames."""
    frames: list[FrameDecode] = []
    buf: list[int] = []
    for lab in labels:
        if lab == LABEL_BIT0 or lab == LABEL_BIT1:
            buf.append(int(lab))
            if len(buf) > 59:
                buf = []
        elif lab == LABEL_MM:
            if len(buf) == 59:
                fd = _decode_frame(buf)
                if fd is not None and fd.plausible():
                    frames.append(fd)
            buf = []
        # LABEL_AMBIGUOUS: drop the second silently, keep accumulating.
    # End-of-stream: try the final buffer in case a clip ends mid-minute
    if len(buf) == 59:
        fd = _decode_frame(buf)
        if fd is not None and fd.plausible():
            frames.append(fd)
    return frames


def _latest_frame(frames: list[FrameDecode]) -> FrameDecode | None:
    """Return the most recent decoded frame, or None if the list is empty.

    Every entry in `frames` already passed `_decode_frame`'s framing check
    (bit[0]==0, bit[20]==1), the three even-parity checks, and the
    plausibility ranges in `FrameDecode.plausible`, so it is a valid
    ground-truth candidate on its own.  Reporting the latest entry
    matches what the FPGA's frame_dec holds at end-of-stream.
    """
    return frames[-1] if frames else None


def decode_iq(iq: np.ndarray) -> dict:
    """Decode complex baseband samples at 1 kS/s using a matched-filter
    bit detector.

    Returns a dict with year/month/day/weekday/hour/minute/dst_active and
    a `locked` boolean (True iff two consecutive frames with minute n,
    n+1 mod 60 were decoded).  Time fields are zero when not locked.
    """
    if len(iq) < MIN_DURATION_SEC * SAMPLE_RATE_HZ:
        return _empty_result()

    env = np.abs(iq).astype(np.float64)
    baseline = uniform_filter1d(env, size=BASELINE_WIN, mode="nearest")
    e_d = env - baseline

    c_0, c_1, c_mm = _correlate(e_d)
    phi = _find_phase(c_0, c_1)
    labels = _decide_seconds(c_0, c_1, c_mm, phi)

    valid_pulses = int(np.sum((labels == LABEL_BIT0) | (labels == LABEL_BIT1)))
    # Median bit score (positive when there's any DCF77 modulation,
    # essentially zero on pure noise).
    bit_scores = np.maximum(c_0[phi::T_PER_SECOND],
                            c_1[phi::T_PER_SECOND])
    median_score = float(np.median(bit_scores)) if bit_scores.size else 0.0
    signal_present = median_score > MIN_SIGNAL_SCORE and valid_pulses >= 10

    frames = _walk_labels(labels)
    locked = _latest_frame(frames)
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
            f"dst={result['dst_active']} (locked on {result['frames_decoded']} "
            f"valid frame{'s' if result['frames_decoded'] != 1 else ''}, "
            f"{result['valid_pulses']} valid pulses)")


if __name__ == "__main__":
    import sys
    if len(sys.argv) != 2:
        print("usage: dcf77_soft_decode.py <iq.s16>", file=sys.stderr)
        raise SystemExit(2)
    raw = np.fromfile(sys.argv[1], dtype=np.int16)
    iq = (raw[0::2].astype(np.float32) + 1j * raw[1::2].astype(np.float32))
    print(summarize(decode_iq(iq)))
