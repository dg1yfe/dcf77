# Hetzel 1988 (EFTF): DCF77 pseudo-random phase-shift keying, summary

**Citation:** P. Hetzel (Physikalisch-Technische Bundesanstalt, Braunschweig),
"Time dissemination via the LF transmitter DCF77 using a pseudo-random
phase-shift keying of the carrier", *Proc. 2nd European Frequency and Time
Forum (EFTF)*, Neuchâtel, 16–18 March 1988, Session TF-05, paper no. 3,
pp. 351–364.

> This is a paraphrased summary written for this project. It is not a
> transcription, and it leaves out the figures and the exact wording. For
> the authoritative text, get the original from the EFTF proceedings or
> from PTB. The paper also cites the author's dissertation (Univ. Stuttgart,
> 1987) as the detailed reference for the spectrum analysis.

## 1. Motivation and overview

- AM second markers received in a narrow band give poor timing resolution.
  A wide band brings in interference, and phase-synchronous interference
  inside the AM band cannot be separated from the marker itself.
- Cross-correlation against a known pseudo-random noise (PRN) signal rejects
  uncorrelated interference. It also gives a correlation peak that is about
  2 chips wide, which is much sharper than an AM edge. This means better
  arrival-time resolution and better use of the available bandwidth.
- DCF77 therefore carries pseudo-random phase-shift keying (PRPSK) on top of
  the existing AM. The phase swing is small, and its mean is zero, so the
  carrier still works as a frequency standard.
- Data are added with **Sequence Inversion Keying (SIK)**: each second sends
  either the sequence or its complement. The paper cites Ward (1976) for SIK.

## 2. Modulation parameters

| Parameter | Value in the paper | Notes |
|---|---|---|
| Carrier | 77.5 kHz (period 12.9 µs) | |
| Phase swing Δφ | **±10°** around the mean phase φm | Equals about ±0.4 µs of phase time |
| Phase mapping | output **low → φm + Δφ**, output **high → φm − Δφ** | Applies after the SIK XOR (Fig. 3) |
| Chip clock f_T | **77 500 / 120 = 645.83 Hz** | Exact subharmonic of the carrier, so 120 carrier cycles per chip. Chosen near the upper rate the antenna can follow |
| Chip duration T_T | 1/f_T ≈ **1.548 ms** | |
| Shift register | 9 stages, feedback = **XOR of stages 5 and 9**, fed into stage 1 | Read from Fig. 2. Polynomial is x⁹ + x⁵ + 1 in this stage numbering |
| Sequence length N | **2⁹ = 512 chips** | 511-chip m-sequence plus the all-zero state (see below). 256 zeros and 256 ones, so DC-balanced |
| Noise cycle T_K | N·T_T ≈ **793 ms** | |
| Start of cycle | **200 ms after the start of each second** (the AM falling edge) | Enable/clear pulse |
| End of cycle | about **7 ms before the next second marker** | Generator then stops until the next second |
| Switching instants | All phase steps and AM edges happen at **positive zero crossings** of the carrier | PN cycles, AM markers and carrier stay phase-locked |
| Bits per second | 1 (one SIK bit per noise cycle) | |

**Generator start-up (Fig. 2):** each second the enable/clear pulse resets
the register to all zeros. A flip-flop (FF) is set at cycle start and
OR-injects a '1' into stage 1 on the first clock, then clears. After that the
register runs freely on its 5⊕9 feedback. The PN output (PNS) appears to be
taken from the feedback XOR output, before the FF injection. *This tap point
is our reading of the schematic, not stated in the text.* Read that way, the
512-chip sequence starts with one '0' (the all-zero state) and then runs the
511-chip m-sequence.

**SIK data:** an XOR after the register adds the data bit modulo 2. Data 0
sends PNS unchanged, and data 1 sends the inverted sequence. The data bits
are the same as the AM pulse-width bits, with one exception at the minute
mark. Second 59 is *not* left empty as in AM, and the **ten seconds 0–9 all
carry the inverted sequence**. The paper gives no other per-chip time coding.
The whole sequence is either PNS or its complement.

## 3. Spectrum

- The envelope follows (sin x)/x with nulls at multiples of f_T (645.83 Hz
  spacing). It is made of discrete lines spaced f_T/N ≈ 1.26 Hz apart.
- The PN line amplitudes scale with sin Δφ, and the residual carrier scales
  with cos Δφ.
- A measurement with 100 Hz resolution bandwidth at the transmitter showed
  the first sidelobes roughly 40 dB below the carrier peak. *This value was
  read approximately from the plot.*

## 4. PTB reference receiver (Fig. 5/6)

- **Carrier PLL:** a 6.2 MHz VCXO divided by 80 gives the 77.5 kHz
  reference. A phase detector compares it with the received signal after a
  band-pass of width **Δf = 4·f_T (≈ 2.6 kHz)** and an amplifier, and a PI
  controller steers the VCXO. Because the PN phase averages to zero, the
  locked detector output averages to zero. The fast residual is the received
  chip sequence, which becomes the input to the correlator.
- **AM path:** a second detector driven by the 90°-shifted reference acts as
  a synchronous envelope demodulator. A comparator follows it and recovers
  the 0.1 s and 0.2 s second pulses.
- **Delay-lock loop (DLL):** a local PN generator is clocked from the VCXO
  through a phase shifter, ÷100 to 62 kHz and ÷96 to 645.83 Hz. It produces
  three copies of the sequence: early (+T_T/2), punctual and late (−T_T/2).
  Early and late are each multiplied with the received sequence and
  integrated, and their difference steers the phase shifter. This gives an
  S-curve built from the triangular 2-chip autocorrelation.
- **Data readout:** the punctual product is integrated over the cycle. At the
  end of each cycle its sign gives the bit: correlation means 0, and
  anticorrelation means 1. The detected bit also flips the polarity of the
  early-minus-late sum, so tracking keeps the correct sign whatever the data.
- **Timing:** 1 Hz pulses (F) and a /512 counter (P) enable and clear the
  local generator, reset the integrators before each cycle and trigger
  readout at the end. The rising edge of F defines the received
  "UTC(DCF77)" epoch.
- No explicit SNR or BER figures are given. Bit recognition was reported as
  reliable, and the advantage over AM was clearest during strong lightning
  activity.

## 5. Measurement results

UTC(PTB) − UTC(DCF77) + K was measured daily at 11 h and 23 h UTC over
about 200 days in 1987, at Braunschweig, 273 km from the transmitter. Night
values scatter more because of the sky wave. Daytime scatter grows from
about mid-October as the sky-wave share increases. Typical standard
deviations at about 300 km:

| Season | Single value vs daily mean (10 000 × 1 s values) | Single value vs nightly mean (1000 × 1 min values) | Daily mean vs monthly mean |
|---|---|---|---|
| Summer | 2 µs | 14 µs | 4 µs |
| Winter | 9 µs | 22 µs | 10 µs |

The summary section states that daytime spread outside winter was below half
a carrier period (< 6.5 µs).

## 6. Caveats

- These are the **1988 parameters**. The paper gives Δφ = ±10°; the
  transmitter may use up to about ±15° today (this repo's synthesizer uses
  15.6°). Treat Δφ as a calibration parameter.
- The chips cover only the part of each second where the carrier is not
  power-reduced (+200 ms to about +993 ms), which is why a chip lasts
  1.548 ms rather than 1 s / 512.
- The schematic gives the register taps but not the output tap point or the
  shift direction in words. The tap point in §2 comes from the figure and was
  checked against the RTL and the Python models.
- Second 59 is transmitted. The paper does not say what bit it carries, and
  it does not cover seconds 10–14.

## Consistency with this repo

`doc/dcf77_format_spec.md` Part 2 was corrected to match this paper (chip rate
645.83 Hz, taps 5 and 9, prepended '0' chip, one SIK bit per second, PM on air
since the 1980s). The RTL (`dcf77_pm_correlator.vhd`) and
`simulation/scripts/dcf77_pm_decode.py` agree with it. Checks done:

- Taps 5 and 9 with seed `000010001` cycle through all 511 non-zero states.
  The cyclic autocorrelation is 511 at lag 0 and −1 elsewhere.
- With the '0' chip prepended there are 256 zeros and 256 ones, so the phase
  sum is zero.
- The reciprocal polynomial x⁹ + x⁴ + 1 gives the time-reversed sequence,
  whose maximum cross-correlation with the real one is only 45 of 511.
- `simulation/scripts/synth_dcf77_wav.py` builds the same m-sequence but
  *appends* the forced '0' chip and starts one state earlier. It differs from
  the decoder and RTL in two of 512 chips (a cyclic shift by one). It should
  be changed to prepend the '0' chip.
