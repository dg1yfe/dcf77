# DCF77 test results

Replay-corpus results for the receiver, moved out of
[`dcf77_architecture.md`](dcf77_architecture.md). See its §9.5 and §9.6 for
how to run the regression and add a recording.

> **Status:** measured on 2026-11-08 with a full
> `python3 simulation/scripts/run.py --clean -p 8` run (99 tests, 99 pass,
> 41 min wall). `07_dcf77_if_replay_tb` closes the carrier loop exactly as
> `dcf77_core` does (CORDIC phase, registered `-phase << 11`,
> `dcf77_carrier_pll` KP=10/KI=23, offset scaled by 250000/Fs for the
> 12 kS/s NCO) and checks `pll_lock` from `dcf77_phase_quality`. All 10
> `acquire` configs lock (`synth`, `dcf_strong`, `dcf_3_1` ... `dcf_3_7long`),
> first lock after 5-15 s of recording, lock held for 97-100 % of the
> 5 s telemetry samples. The converged NCO offset is within about 1.5 Hz
> of nominal, but the instantaneous offset jitters by a few hundred LSB
> (proportional term on a noisy phase), so it is not a precise estimate of
> the residual 0.24-0.49 Hz carrier offset. The figures below use the current
> fixtures: the synthesizer prepends the '0' chip and emits the RF phase
> with the sign the receiver chain and the real recordings use. The
> recordings are fetched with `simulation/scripts/fetch_samples.py`; their
> origin is listed in `samples/PROVENANCE.md`. The duration is the length
> of the WAV file. With the default `run.py` settings the replay covers the
> first 60 s of each capture (`DCF77_PM_REPLAY_LIMIT`), except `synth`,
> which is replayed whole and checked against `samples/synth.expect`.
>
> Test count: still 99, the same set as before the recordings were renamed
> to their short stems (one config per recording and test, plus the
> `dcf_3_5.noise1750` reference in the PM-correlator regression).

## PM-correlator regression

`15_dcf77_pm_correlator_replay_tb`, PHASE_ERR_V_CLOCKS = 62500 (live HW
rate), 11/11 pass:

| Recording (duration)                | pm_lock_qual | pm_valid | pm_mm  | Notes                          |
| ----------------------------------- | ------------ | -------- | ------ | ------------------------------ |
| `dcf_strong` (223 s)                | 31992        | 57       | **1**  | reference: clean signal        |
| `synth` (130 s, synthetic)          | 29683        | 128      | **1**  | whole capture; PM frame decodes to 26-05-13 11:43, = `synth.expect` |
| `dcf_3_3` (200 s)                   | 15868        | 55       | **1**  | strong real signal             |
| `dcf_3_6` (198 s)                   | 23483        | 57       | **1**  | strong real signal             |
| `dcf_3_4` (257 s)                   | 2967         | 54       | 0      | marginal                       |
| `dcf_3_5` (312 s)                   | 4790         | 50       | 0      | marginal                       |
| `dcf_3_7` (544 s)                   | 4066         | 47       | 0      | marginal                       |
| `dcf_3_1` (398 s)                   | 2187         | 56       | 0      | marginal                       |
| `dcf_3_5long` (**26 min**)          | 3779         | 46       | 0      | marginal SNR 20 dB, first 60 s only |
| `dcf_3_7long` (**23 min**)          | 4091         | 54       | 0      | marginal SNR 28 dB, first 60 s only |
| `dcf_3_5.noise1750` (312 s)         | 0            | 0        | 0      | noise reference, stays in FILL (pass = rejected) |

The strong recordings clear pm_mm within their first available minute.
The marginal recordings need many minutes to roll a binomially-clean
SIK signature; in an earlier full-length replay
(`DCF77_PM_REPLAY_LIMIT=0`) `dcf_3_5long` rolled one in 26 min,
confirming that edge-protected T=1 is the right operating point for
real-world marginal reception. The default 60 s window shown above is
too short for that.

Each config takes ~48 s of wall time at live HW rate (`synth`, replayed
whole: 57 s); inside the full `-p 8` run they overlap with the other
benches.
