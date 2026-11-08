# DCF77 test results

Replay-corpus results for the receiver, moved out of
[`dcf77_architecture.md`](dcf77_architecture.md). See its §9.5 and §9.6 for
how to run the regression and add a recording.

> **Status:** these figures predate the PM sequence fixes of November 2026
> (the synthesizer now prepends the '0' chip, matching the decoder and RTL)
> and have not been re-run since. The `synth` row used an AM-only fixture,
> generated before PM was added to the synthesizer. The duration is the
> length of the WAV file, which is also the length of its replay run. The
> recordings are fetched with `simulation/scripts/fetch_samples.py`; their
> origin is listed in `samples/PROVENANCE.md`.

## PM-correlator regression

`15_dcf77_pm_correlator_replay_tb`, PHASE_ERR_V_CLOCKS = 62500 (live HW
rate), 11/11 pass:

| Recording (duration)                | pm_lock_qual | pm_valid | pm_mm  | Notes                          |
| ----------------------------------- | ------------ | -------- | ------ | ------------------------------ |
| `dcf_strong` (223 s)                 | 31992        | 57       | **1**  | reference: clean signal        |
| `synth` (synthetic)                | ~29000       | 58       | 0      | AM-only fixture, no PM in the file |
| `dcf_3_3` (200 s)                   | 15868        | 55       | **1**  | strong real signal             |
| `dcf_3_6` (198 s)                   | 23483        | 57       | **1**  | strong real signal             |
| `dcf_3_4` (257 s)                   | 2967         | 54       | 0      | marginal                       |
| `dcf_3_5` (312 s)                   | 4790         | 50       | 0      | marginal                       |
| `dcf_3_7` (544 s)                   | 4066         | 47       | 0      | marginal                       |
| `dcf_3_1` (398 s)                   | 2187         | 56       | 0      | marginal                       |
| `dcf_3_5long` (**26 min**)          | 4732         | 1552     | **1**  | marginal SNR 20 dB, T=1 fired  |
| `dcf_3_7long` (**23 min**)        | 4442         | 1401     | 0      | marginal SNR 28 dB, T=1 not yet |

The strong recordings clear pm_mm within their first available minute.
The marginal recordings need many minutes to roll a binomially-clean
SIK signature; `dcf_3_5long` rolled one in 26 min, confirming that
edge-protected T=1 is the right operating point for real-world
marginal reception.

Total wall time for the 11-config corpus at live HW rate: **~118 sec**
(2 min) using 8 parallel workers.
