#!/usr/bin/env python3
"""Unit tests for dcf77_pm_decode.

Synthesises a phase_err stream from a known SIK bit pattern + AWGN and
verifies the decoder:
    1. recovers the per-second SIK sign at SNR ≥ 0 dB,
    2. produces a triangular ACF peaking at the true chip0 offset,
    3. handles signal-loss / zero-input cleanly.

Run via `python3 -m pytest simulation/scripts/test_pm_decode.py` or
directly as `python3 simulation/scripts/test_pm_decode.py`.
"""
from __future__ import annotations

import numpy as np

from dcf77_pm_decode import (
    CHIP0_OFFSET_NOMINAL,
    PRBS_SAMPLES,
    SECOND_SAMPLES,
    expand_prbs_to_samples,
    find_chip0_offset,
    make_prbs512,
    pm_soft_per_second,
    sik_polarity,
)


def synth_phase_err(sik_bits: list[int],
                    delta_phi_lsb: float = 27_000.0,
                    snr_db: float = 12.0,
                    chip0_offset: int = CHIP0_OFFSET_NOMINAL,
                    seed: int = 0) -> np.ndarray:
    n_secs = len(sik_bits)
    template = expand_prbs_to_samples(make_prbs512())
    polarities = np.array([sik_polarity(s, sik_data=sik_bits)
                           for s in range(n_secs)], dtype=np.float64)
    phase_err = np.zeros(n_secs * SECOND_SAMPLES, dtype=np.float64)
    for k in range(n_secs):
        s = k * SECOND_SAMPLES + chip0_offset
        phase_err[s:s + PRBS_SAMPLES] = delta_phi_lsb * polarities[k] * template
    if snr_db < 200:
        rng = np.random.default_rng(seed)
        sig_power = delta_phi_lsb ** 2
        noise_power = sig_power / (10 ** (snr_db / 10))
        phase_err += rng.normal(0.0, np.sqrt(noise_power), phase_err.shape)
    return phase_err


def _make_full_minute() -> list[int]:
    """One full DCF77 minute of SIK bits: signature in s0..s14, s59,
    plus an arbitrary deterministic data pattern in s15..s58."""
    return ([1] * 10 + [0] * 5 +
            [(i % 7) & 1 for i in range(15, 59)] +
            [0])


def test_lock_clean_signs() -> None:
    sik = _make_full_minute()
    phase_err = synth_phase_err(sik, snr_db=12.0)
    pm_soft = pm_soft_per_second(phase_err)
    expected = np.array([sik_polarity(s, sik_data=sik)
                         for s in range(len(pm_soft))])
    actual = np.sign(pm_soft).astype(int)
    matches = int(np.sum(actual == expected))
    assert matches == len(sik), \
        f"clean signal: expected 60/60 signs to match, got {matches}/60"


def test_lock_noisy_signs() -> None:
    sik = _make_full_minute()
    phase_err = synth_phase_err(sik, snr_db=0.0)
    pm_soft = pm_soft_per_second(phase_err)
    expected = np.array([sik_polarity(s, sik_data=sik)
                         for s in range(len(pm_soft))])
    actual = np.sign(pm_soft).astype(int)
    matches = int(np.sum(actual == expected))
    assert matches >= 55, \
        f"noisy signal: expected ≥55/60 signs to match, got {matches}/60"


def test_chip0_offset_sweep_peaks_at_truth() -> None:
    # Repeat the minute three times so the chip0 search has enough seconds.
    sik = _make_full_minute() * 3
    truth_offset = 400
    phase_err = synth_phase_err(sik, chip0_offset=truth_offset,
                                snr_db=200)   # near-noiseless
    offset_found, score = find_chip0_offset(phase_err,
                                            search_range_samples=2000)
    assert abs(offset_found - truth_offset) <= 4, \
        f"chip0 search: expected offset near {truth_offset}, got {offset_found}"
    assert score > 0


def test_pm_soft_zero_on_silence() -> None:
    phase_err = np.zeros(60 * SECOND_SAMPLES, dtype=np.float64)
    pm_soft = pm_soft_per_second(phase_err)
    assert np.all(pm_soft == 0.0)


def test_template_is_balanced() -> None:
    """The 512-chip PRBS has 256 zeros and 256 ones (Hetzel's force-1
    padding); the ±1 chip values sum to ~0 over the full sequence."""
    prbs = make_prbs512()
    assert np.sum(prbs) == 0, \
        f"PRBS chip sum should be 0 (256 each ±1), got {np.sum(prbs)}"


if __name__ == "__main__":
    test_template_is_balanced(); print("ok: template_is_balanced")
    test_lock_clean_signs(); print("ok: lock_clean_signs")
    test_lock_noisy_signs(); print("ok: lock_noisy_signs")
    test_chip0_offset_sweep_peaks_at_truth(); print("ok: chip0_offset_sweep")
    test_pm_soft_zero_on_silence(); print("ok: pm_soft_zero_on_silence")
    print("all PM decoder tests passed")
