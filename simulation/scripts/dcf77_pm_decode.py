#!/usr/bin/env python3
"""Software DCF77 PM (PSK / SIK) reference decoder.

Matches the FPGA PM-correlator chain: PRBS-9 with taps 5 & 9 initialised
to "000010001", chip rate f_T = 77500/120 ≈ 645.83 Hz
= 31/96 of the 2 kS/s sample rate (exact rational), 1586 samples per
1-second integration window starting 200 ms after each second boundary.

Per Hetzel (1988, EFTF), the PSK uses Sequence Inversion Keying: PRBS
template is fixed, polarity is flipped by the data bit (s_k='1' → PNS̄,
s_k='0' → PNS unchanged).  The first 15 seconds of each minute carry the
deterministic signature 1111111111 00000 (= ten inverted, five unchanged)
which acts as a minute-mark marker.

Public API:
    make_prbs512()                 → 512-chip ±1 numpy array (sign template)
    expand_prbs_to_samples(prbs)   → 1586 sign-per-sample template @ 2 kS/s
    pm_soft_per_second(phase_err)  → list[float] per second, sliding correlator
    sik_polarity(sec)              → +1 / -1 expected SIK polarity per second

CLI: invoke as a self-test against a synthetic stimulus.
"""
from __future__ import annotations

import numpy as np

SAMPLE_RATE_HZ      = 2000             # PM correlator sample rate
CHIP_RATE_NUM       = 31                # 31/96 ratio between chip and sample rates
CHIP_RATE_DEN       = 96
PRBS_CHIPS          = 512               # PRBS-9 with force-1 = 2^9 = 512 chips
PRBS_SAMPLES        = 1586              # ceil(512 * 96/31) samples per PRBS cycle
SECOND_SAMPLES      = SAMPLE_RATE_HZ    # 2000 samples per second
CHIP0_OFFSET_NOMINAL = 400              # 200 ms × 2 kS/s

LFSR_TAPS           = (4, 8)            # 0-indexed = stages 5 and 9
LFSR_INIT           = 0x011             # 17 = 0b000010001 (stages 1 & 5 set)


def make_prbs512() -> np.ndarray:
    """Returns 512 chips of the DCF77 SIK PRBS as a ±1 int8 array.

    Per external broadcast analysis:
      * Fibonacci LFSR, polynomial x^9 + x^5 + 1, taps 5 & 9 feeding back
        into bit 0 on a left shift.
      * Initial seed = 0x011 = 17.
      * Output bit = MSB (bit 8) of state, taken BEFORE the shift.
      * Period = 511 chips; one '0' chip is PREPENDED to balance to 512
        chips with exactly 256 ones and 256 zeros (DC-balanced).

    Returned as +1 for chip='0' (PNS, phase = +Δφ) and -1 for chip='1'
    (PNS̄, phase = -Δφ), so Σ phase_err · chip_sign  is positive for
    the SIK '0' bit (matching the HW pm_soft convention).
    """
    state = LFSR_INIT
    chips = np.empty(PRBS_CHIPS, dtype=np.int8)
    chips[0] = +1                          # prepended force-1 chip = '0'
    for n in range(1, PRBS_CHIPS):
        bit = (state >> 8) & 1             # MSB output BEFORE shift
        chips[n] = 1 - 2 * bit             # '0' → +1, '1' → -1
        new_bit = ((state >> LFSR_TAPS[0]) & 1) ^ ((state >> LFSR_TAPS[1]) & 1)
        state = ((state << 1) | new_bit) & 0x1FF
    return chips


def expand_prbs_to_samples(prbs: np.ndarray) -> np.ndarray:
    """Map the 512-chip PRBS to a 1586-sample sign template at 2 kS/s.

    sample-to-chip mapping follows the FPGA's mod-96 Bresenham:
        chip(n) = floor((n * 31) / 96)
    so each chip spans either 3 or 4 samples (mean = 96/31 ≈ 3.097).
    """
    n = np.arange(PRBS_SAMPLES)
    chip_idx = (n * CHIP_RATE_NUM) // CHIP_RATE_DEN
    return prbs[chip_idx].astype(np.float64)


def sik_polarity(sec: int, sik_data: list[int] | None = None) -> int:
    """Return +1 (SIK '0') or -1 (SIK '1') for the given second-in-minute.

    For seconds 0..9: SIK '1' (signature) → -1
    For seconds 10..14, 59: SIK '0' (signature) → +1
    For seconds 15..58: depends on the data bit (caller provides sik_data
    indexed by second-in-minute).  Returns 0 if sik_data is None.
    """
    if 0 <= sec <= 9:
        return -1
    if 10 <= sec <= 14 or sec == 59:
        return +1
    if sik_data is not None:
        bit = sik_data[sec]
        return -1 if bit else +1
    return 0


def pm_soft_per_second(phase_err: np.ndarray,
                       chip0_offset: int = CHIP0_OFFSET_NOMINAL
                       ) -> np.ndarray:
    """Compute pm_soft per second from a phase_err stream at 2 kS/s.

    For each second boundary k in [0, len(phase_err) / 2000):
        window  = phase_err[k*2000 + chip0_offset : k*2000 + chip0_offset + 1586]
        pm_soft = Σ window[n] · template[n]      (template = ±1 per sample)

    Returns a 1-D float array with one pm_soft per complete second.  The
    sign matches the SIK encoding: positive → SIK '0' (PNS unchanged) →
    data bit '0' favoured.
    """
    prbs     = make_prbs512()
    template = expand_prbs_to_samples(prbs)

    n_seconds = len(phase_err) // SECOND_SAMPLES
    out = np.zeros(n_seconds, dtype=np.float64)
    for k in range(n_seconds):
        start = k * SECOND_SAMPLES + chip0_offset
        end   = start + PRBS_SAMPLES
        if end > len(phase_err):
            break
        window = phase_err[start:end]
        out[k] = float(np.dot(window, template))
    return out


def find_chip0_offset(phase_err: np.ndarray,
                      search_range_samples: int = 2000
                      ) -> tuple[int, float]:
    """Sweep chip0_offset over [0, search_range_samples) and return the
    (offset, mean |pm_soft|) pair that maximises the per-second correlation.

    The peak of the triangular auto-correlation (Hetzel Bild 9) identifies
    the broadcast's chip-0 sample position.  Used by the corpus pre-pass
    to find PRBS_OFFSET_SAMPLES for each recording.
    """
    prbs     = make_prbs512()
    template = expand_prbs_to_samples(prbs)
    n_seconds = (len(phase_err) - search_range_samples - PRBS_SAMPLES) // SECOND_SAMPLES
    if n_seconds < 5:
        return CHIP0_OFFSET_NOMINAL, 0.0

    best_offset = 0
    best_score  = -np.inf
    for off in range(0, search_range_samples, 4):
        acc = 0.0
        for k in range(n_seconds):
            start = k * SECOND_SAMPLES + off
            window = phase_err[start:start + PRBS_SAMPLES]
            acc += abs(float(np.dot(window, template)))
        mean_abs = acc / n_seconds
        if mean_abs > best_score:
            best_score = mean_abs
            best_offset = off
    return best_offset, best_score


def _self_test() -> int:
    """Synthesise a known phase_err stream and verify the decoder recovers
    the expected per-second SIK bits."""
    rng = np.random.default_rng(42)
    n_secs = 30
    delta_phi_lsb = 27_000.0
    snr_db = 12.0

    prbs     = make_prbs512()
    template = expand_prbs_to_samples(prbs)

    # Build a deterministic SIK bit pattern.  Seconds 0..9 = '1' (signature),
    # 10..14 = '0' (signature), 15..58 = alternating data, 59 = '0' (forced).
    sik_bits = [1] * 10 + [0] * 5 + [(i & 1) for i in range(15, 59)] + [0]
    polarities = np.array([sik_polarity(s, sik_data=sik_bits)
                           for s in range(n_secs)], dtype=np.float64)

    # Synthesise phase_err.  Active 1586 samples per second starting at chip0.
    chip0 = CHIP0_OFFSET_NOMINAL
    phase_err = np.zeros(n_secs * SECOND_SAMPLES, dtype=np.float64)
    for k in range(n_secs):
        s = k * SECOND_SAMPLES + chip0
        phase_err[s:s + PRBS_SAMPLES] = delta_phi_lsb * polarities[k] * template

    # AWGN scaled to match SNR.  Signal power per sample = delta_phi_lsb^2.
    sig_power_per_sample = delta_phi_lsb ** 2
    noise_power_per_sample = sig_power_per_sample / (10 ** (snr_db / 10))
    noise_std = float(np.sqrt(noise_power_per_sample))
    phase_err += rng.normal(0.0, noise_std, size=phase_err.shape)

    pm_soft = pm_soft_per_second(phase_err, chip0_offset=chip0)
    expected_sign = polarities
    actual_sign   = np.sign(pm_soft)
    matches = int(np.sum(actual_sign == expected_sign))
    print(f"PM self-test: {matches}/{n_secs} per-second signs match "
          f"(SNR={snr_db} dB)")
    print(f"  pm_soft (signed): {[round(v / delta_phi_lsb, 2) for v in pm_soft[:15]]}...")

    # Search the chip0 offset to verify the peak-finder
    offset_found, score = find_chip0_offset(phase_err)
    print(f"  chip0 search: offset={offset_found} (true={chip0}), "
          f"score={score:.1f}")

    return 0 if matches == n_secs else 1


if __name__ == "__main__":
    raise SystemExit(_self_test())
