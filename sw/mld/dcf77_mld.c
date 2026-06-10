/*
 * dcf77_mld.c — see dcf77_mld.h.
 *
 * Algorithm summary
 * -----------------
 * 1. feed():  store am_soft[sec_index] / pm_soft[sec_index]. When
 *             sec_index wraps (e.g. last=59, new=0), trigger decode
 *             of the just-completed minute.
 * 2. combine: for each second, compute  comb = α·am_soft + β·pm_soft
 *             with α,β disabled where the respective LLR isn't valid.
 * 3. hard:    bit[i] = (comb[i] < 0) ? 1 : 0  (matches AM and SIK
 *             sign conventions: both vote +ve for bit '0').
 * 4. parity:  three even-parity checks over bits 21..28, 29..35,
 *             36..58. If exactly one fails AND allow_flip is true,
 *             pick the lowest-|comb| bit in the failing group and
 *             flip it.
 * 5. BCD + plausibility: standard DCF77 fields. On success the
 *             decoded fields are written to *t.
 */
#include "dcf77_mld.h"

#include <string.h>
#include <stdlib.h>

/* ---------------------------------------------------------------- */
/* DCF77 frame layout (bit indices 0..58).                            */
/* ---------------------------------------------------------------- */

/* Field windows (start, count). */
#define DCF77_DST_ANNOUNCE_BIT   16
#define DCF77_DST_ACTIVE_BIT     17
#define DCF77_LEAP_ANNOUNCE_BIT  19
#define DCF77_TIME_START_BIT     20  /* always '1' */
#define DCF77_MIN_START          21
#define DCF77_MIN_LEN            7
#define DCF77_MIN_PARITY_BIT     28
#define DCF77_HOUR_START         29
#define DCF77_HOUR_LEN           6
#define DCF77_HOUR_PARITY_BIT    35
#define DCF77_DAY_START          36
#define DCF77_DAY_LEN            6
#define DCF77_WEEKDAY_START      42
#define DCF77_WEEKDAY_LEN        3
#define DCF77_MONTH_START        45
#define DCF77_MONTH_LEN          5
#define DCF77_YEAR_START         50
#define DCF77_YEAR_LEN           8
#define DCF77_DATE_PARITY_BIT    58

static const uint8_t W_MIN[7]  = {1, 2, 4, 8, 10, 20, 40};
static const uint8_t W_HR[6]   = {1, 2, 4, 8, 10, 20};
static const uint8_t W_DAY[6]  = {1, 2, 4, 8, 10, 20};
static const uint8_t W_WD[3]   = {1, 2, 4};
static const uint8_t W_MON[5]  = {1, 2, 4, 8, 10};
static const uint8_t W_YR[8]   = {1, 2, 4, 8, 10, 20, 40, 80};

static uint8_t bcd_decode(const uint8_t *bits, const uint8_t *weights,
                          uint8_t n)
{
    uint16_t sum = 0;
    for (uint8_t i = 0; i < n; i++) {
        sum = (uint16_t)(sum + bits[i] * weights[i]);
    }
    return (uint8_t)sum;
}

uint8_t dcf77_mld_parity_even(const uint8_t *bits, uint8_t n)
{
    uint8_t s = 0;
    for (uint8_t i = 0; i < n; i++) {
        s = (uint8_t)(s ^ (bits[i] & 1u));
    }
    /* "even" = 0 → return 1 (good); odd → return 0 (bad) */
    return (uint8_t)(s == 0u ? 1u : 0u);
}

/* ---------------------------------------------------------------- */
/* Lifecycle.                                                        */
/* ---------------------------------------------------------------- */

void dcf77_mld_init(dcf77_mld_t *mld)
{
    memset(mld, 0, sizeof *mld);
    mld->alpha       = DCF77_MLD_ALPHA_DEFAULT;
    mld->beta        = DCF77_MLD_BETA_DEFAULT;
    mld->allow_flip  = true;
    mld->require_sik = false;   /* HW T=1 detector already anchors */
    mld->last_sec    = 0xFFu;   /* sentinel for "no feed yet" */
}

void dcf77_mld_reset(dcf77_mld_t *mld)
{
    /* Zero buffers, keep config + counters. */
    memset(mld->am_soft,  0, sizeof mld->am_soft);
    memset(mld->pm_soft,  0, sizeof mld->pm_soft);
    memset(mld->am_valid, 0, sizeof mld->am_valid);
    memset(mld->pm_valid, 0, sizeof mld->pm_valid);
    mld->last_sec = 0xFFu;
    mld->have_any = false;
}

void dcf77_mld_set_beta(dcf77_mld_t *mld, uint16_t pm_lock_qual)
{
    /* pm_lock_qual is a 16-b unsigned IIR; >> 7 normalises to Q1.7
     * with saturation at 0x80 (= 1.0). Anything <128 is treated as
     * "not locked" and zeros β so the MLD falls back to AM-only. */
    uint16_t b = (uint16_t)(pm_lock_qual >> 7);
    if (b > 0x80u) b = 0x80u;
    if (b < 0x10u) b = 0u;     /* deadband: PM contribution < ~12% drop */
    mld->beta = (int16_t)b;
}

/* ---------------------------------------------------------------- */
/* Combine + hard-decode.                                            */
/* ---------------------------------------------------------------- */

int16_t dcf77_mld_combine_one(const dcf77_mld_t *mld, uint8_t sec)
{
    int32_t sum = 0;
    if (mld->am_valid[sec]) {
        sum += (int32_t)mld->alpha * (int32_t)mld->am_soft[sec];
    }
    if (mld->pm_valid[sec]) {
        sum += (int32_t)mld->beta  * (int32_t)mld->pm_soft[sec];
    }
    /* Q1.7 → shift down 7. Saturate to int16. */
    sum >>= 7;
    if (sum >  32767) sum =  32767;
    if (sum < -32768) sum = -32768;
    return (int16_t)sum;
}

void dcf77_mld_hard_decode(const dcf77_mld_t *mld, uint8_t bits[59])
{
    for (uint8_t i = 0; i < 59; i++) {
        int16_t c = dcf77_mld_combine_one(mld, i);
        /* Sign convention: am_soft > 0 ⇒ bit '0'.
         * Use < 0 (strict) so the neutral 0 LLR maps to '0'. */
        bits[i] = (c < 0) ? 1u : 0u;
    }
}

/* ---------------------------------------------------------------- */
/* SIK signature check (PM seconds 0..14).                            */
/* ---------------------------------------------------------------- */

uint8_t dcf77_mld_sik_score(const dcf77_mld_t *mld)
{
    /* Expected SIK polarity: seconds 0..9 transmit PNS̄ (pm_soft < 0),
     * seconds 10..14 transmit PNS  (pm_soft >= 0). */
    uint8_t hits = 0;
    for (uint8_t i = 0; i < 15; i++) {
        if (!mld->pm_valid[i]) continue;
        if (i < 10) {
            if (mld->pm_soft[i] < 0) hits++;
        } else {
            if (mld->pm_soft[i] >= 0) hits++;
        }
    }
    return hits;
}

/* ---------------------------------------------------------------- */
/* BCD + parity + plausibility.                                      */
/* ---------------------------------------------------------------- */

bool dcf77_mld_validate(const uint8_t bits[59], dcf77_time_t *t)
{
    /* Framing markers. */
    if (bits[0] != 0u) return false;
    if (bits[DCF77_TIME_START_BIT] != 1u) return false;

    /* DST mutex: bits 17 and 18 must differ. */
    if ((bits[DCF77_DST_ACTIVE_BIT] ^ bits[18]) != 1u) return false;

    /* Even parities. */
    if (!dcf77_mld_parity_even(&bits[DCF77_MIN_START],  DCF77_MIN_LEN + 1))  return false;
    if (!dcf77_mld_parity_even(&bits[DCF77_HOUR_START], DCF77_HOUR_LEN + 1)) return false;
    /* Date parity spans bits 36..58 inclusive (23 bits). */
    if (!dcf77_mld_parity_even(&bits[DCF77_DAY_START],
                                (uint8_t)(DCF77_DATE_PARITY_BIT - DCF77_DAY_START + 1)))
        return false;

    /* BCD decode. */
    uint8_t minute  = bcd_decode(&bits[DCF77_MIN_START],     W_MIN, DCF77_MIN_LEN);
    uint8_t hour    = bcd_decode(&bits[DCF77_HOUR_START],    W_HR,  DCF77_HOUR_LEN);
    uint8_t day     = bcd_decode(&bits[DCF77_DAY_START],     W_DAY, DCF77_DAY_LEN);
    uint8_t weekday = bcd_decode(&bits[DCF77_WEEKDAY_START], W_WD,  DCF77_WEEKDAY_LEN);
    uint8_t month   = bcd_decode(&bits[DCF77_MONTH_START],   W_MON, DCF77_MONTH_LEN);
    uint8_t year    = bcd_decode(&bits[DCF77_YEAR_START],    W_YR,  DCF77_YEAR_LEN);

    /* Plausibility ranges. */
    if (minute  > 59u) return false;
    if (hour    > 23u) return false;
    if (day     <  1u || day   > 31u) return false;
    if (month   <  1u || month > 12u) return false;
    if (weekday <  1u || weekday > 7u) return false;
    /* year is 0..99 — every 8-bit BCD-decoded value fits */

    if (t) {
        t->minute        = minute;
        t->hour          = hour;
        t->day           = day;
        t->weekday       = weekday;
        t->month         = month;
        t->year          = year;
        t->dst_active    = (bits[DCF77_DST_ACTIVE_BIT] != 0u);
        t->dst_announce  = (bits[DCF77_DST_ANNOUNCE_BIT] != 0u);
        t->leap_announce = (bits[DCF77_LEAP_ANNOUNCE_BIT] != 0u);
        t->source        = DCF77_SOURCE_MLD;
    }
    return true;
}

/* ---------------------------------------------------------------- */
/* Parity-driven single-bit-flip recovery.                            */
/* ---------------------------------------------------------------- */

static uint8_t abs_i16(int16_t v)
{
    /* Map to uint8 saturated; we only need the relative magnitude. */
    int32_t a = (v < 0) ? -(int32_t)v : (int32_t)v;
    if (a > 255) a = 255;
    return (uint8_t)a;
}

/*
 * Identify the bit in [start, start+count) with the smallest |comb|
 * (lowest confidence) and flip it. Returns true if a flip was made.
 */
static bool flip_weakest(const dcf77_mld_t *mld,
                         uint8_t bits[59],
                         uint8_t start, uint8_t count)
{
    uint8_t weakest = start;
    uint8_t weakest_mag = 0xFFu;
    bool found = false;

    for (uint8_t i = start; i < start + count; i++) {
        uint8_t mag = abs_i16(dcf77_mld_combine_one(mld, i));
        if (!found || mag < weakest_mag) {
            weakest = i;
            weakest_mag = mag;
            found = true;
        }
    }
    if (found) {
        bits[weakest] ^= 1u;
    }
    return found;
}

/* Try to fix the frame in-place. Up to one flip per failing parity
 * group. Returns true if all three parities are satisfied after flips. */
static bool try_flip_recovery(const dcf77_mld_t *mld, uint8_t bits[59])
{
    uint8_t p_min  = dcf77_mld_parity_even(&bits[DCF77_MIN_START],  DCF77_MIN_LEN + 1);
    uint8_t p_hr   = dcf77_mld_parity_even(&bits[DCF77_HOUR_START], DCF77_HOUR_LEN + 1);
    uint8_t p_dt   = dcf77_mld_parity_even(&bits[DCF77_DAY_START],
                                            (uint8_t)(DCF77_DATE_PARITY_BIT
                                                       - DCF77_DAY_START + 1));

    uint8_t failed = (uint8_t)(!p_min) + (uint8_t)(!p_hr) + (uint8_t)(!p_dt);
    if (failed == 0u) return true;
    if (failed > 1u)  return false; /* two-group flip is out of scope */

    if (!p_min) {
        flip_weakest(mld, bits, DCF77_MIN_START, DCF77_MIN_LEN + 1);
    } else if (!p_hr) {
        flip_weakest(mld, bits, DCF77_HOUR_START, DCF77_HOUR_LEN + 1);
    } else {
        flip_weakest(mld, bits, DCF77_DAY_START,
                     (uint8_t)(DCF77_DATE_PARITY_BIT - DCF77_DAY_START + 1));
    }

    /* Re-check. A flip can fix the targeted parity but should not
     * break the others (they're disjoint groups). */
    p_min = dcf77_mld_parity_even(&bits[DCF77_MIN_START],  DCF77_MIN_LEN + 1);
    p_hr  = dcf77_mld_parity_even(&bits[DCF77_HOUR_START], DCF77_HOUR_LEN + 1);
    p_dt  = dcf77_mld_parity_even(&bits[DCF77_DAY_START],
                                   (uint8_t)(DCF77_DATE_PARITY_BIT
                                              - DCF77_DAY_START + 1));
    return (p_min && p_hr && p_dt);
}

/* ---------------------------------------------------------------- */
/* Top-level decode.                                                  */
/* ---------------------------------------------------------------- */

bool dcf77_mld_decode_now(dcf77_mld_t *mld, dcf77_time_t *t)
{
    /* Optional SIK signature check — anchors the minute when PM is
     * fully locked. Skipped when require_sik is false (HW T=1 already
     * anchored us). */
    if (mld->require_sik &&
        dcf77_mld_sik_score(mld) < DCF77_MLD_SIK_THRESHOLD) {
        mld->minutes_failed++;
        return false;
    }

    uint8_t bits[59];
    dcf77_mld_hard_decode(mld, bits);

    if (!dcf77_mld_validate(bits, t)) {
        if (mld->allow_flip && try_flip_recovery(mld, bits)) {
            if (dcf77_mld_validate(bits, t)) {
                mld->minutes_decoded++;
                return true;
            }
        }
        mld->minutes_failed++;
        return false;
    }
    mld->minutes_decoded++;
    return true;
}

/* ---------------------------------------------------------------- */
/* Per-second feed.                                                   */
/* ---------------------------------------------------------------- */

bool dcf77_mld_feed(dcf77_mld_t *mld,
                    const dcf77_soft_metrics_t *s,
                    dcf77_time_t *t)
{
    bool minute_boundary = false;

    if (mld->have_any) {
        /* Wrap detector: sec_index strictly less than last_sec means
         * we crossed a minute boundary. (Equal sec_index means a
         * second was double-fed; treat as no boundary, overwrite.) */
        if (s->sec_index < mld->last_sec) {
            minute_boundary = true;
        }
    } else {
        mld->have_any = true;
    }

    bool decoded_ok = false;
    if (minute_boundary) {
        decoded_ok = dcf77_mld_decode_now(mld, t);
        dcf77_mld_reset(mld);
        mld->have_any = true;
    }

    /* Store the new second's LLRs. */
    if (s->sec_index < 60u) {
        if (s->am_valid) {
            mld->am_soft[s->sec_index]  = s->am_soft;
            mld->am_valid[s->sec_index] = 1u;
        }
        if (s->pm_valid) {
            mld->pm_soft[s->sec_index]  = s->pm_soft;
            mld->pm_valid[s->sec_index] = 1u;
        }
        mld->last_sec = s->sec_index;
    }

    return decoded_ok;
}
