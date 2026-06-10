/*
 * dcf77_mld.h — example Maximum-Likelihood Decoder for the DCF77
 * receiver's per-second am_soft / pm_soft outputs.
 *
 * The MLD combines AM and PM bit LLRs into a hard-decoded 60-bit
 * frame, validates via BCD + even-parity + plausibility, and falls
 * back to a single-bit-flip search when one parity check fails.
 *
 * This is the "example" implementation — bit-by-bit + parity flip,
 * about 3 dB shy of the theoretically optimal MLD that would
 * enumerate (year, month, day, hour, minute) hypotheses. The
 * example runs in <1 ms on a Microblaze with ~2 kB code + ~300 B
 * per-minute buffer.
 */
#ifndef DCF77_MLD_H_
#define DCF77_MLD_H_

#include "dcf77.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ---------------------------------------------------------------- */
/* Tuning knobs (compile-time defaults; override in code at runtime). */
/* ---------------------------------------------------------------- */

/* α = AM weight in Q1.7 (0x80 = 1.0). Default 1.0 — the AM bit LLR
 * is the cheaper of the two and always available. */
#define DCF77_MLD_ALPHA_DEFAULT     0x80

/* β = PM weight in Q1.7 (0x80 = 1.0). Set per-minute from
 * pm_lock_qual: β = min(0x80, pm_lock_qual >> 7). */
#define DCF77_MLD_BETA_DEFAULT      0x80

/* SIK signature check: how many of seconds 0..14 must match the
 * expected polarity pattern. 12/15 is a good default (allows up to
 * 3 PM bit errors in the signature, beyond which the minute is
 * dropped). */
#define DCF77_MLD_SIK_THRESHOLD     12

/* ---------------------------------------------------------------- */
/* MLD state. Caller-supplied; no internal allocation. */
/* ---------------------------------------------------------------- */

typedef struct {
    /* Configuration. */
    int16_t  alpha;        /* Q1.7 AM weight */
    int16_t  beta;         /* Q1.7 PM weight */
    bool     allow_flip;   /* enable single-bit-flip parity recovery */
    bool     require_sik;  /* require SIK signature match for PM-anchored decode */

    /* Per-minute LLR ring, indexed 0..59 by sec_index. */
    int16_t  am_soft[60];
    int16_t  pm_soft[60];
    uint8_t  am_valid[60]; /* 0/1 — which seconds have a fresh AM LLR */
    uint8_t  pm_valid[60]; /* 0/1 — which seconds have a fresh PM LLR */

    /* Tracking. */
    uint8_t  last_sec;     /* sec_index of the most recent feed */
    bool     have_any;     /* false until first feed() call */
    uint16_t minutes_decoded;  /* monotonic count of successful decodes */
    uint16_t minutes_failed;   /* monotonic count of completed but failed minutes */
} dcf77_mld_t;

/* ---------------------------------------------------------------- */
/* API.                                                              */
/* ---------------------------------------------------------------- */

void dcf77_mld_init(dcf77_mld_t *mld);
void dcf77_mld_reset(dcf77_mld_t *mld);

/*
 * Per-minute β adjustment. Call once at minute boundary with the
 * pm_lock_qual from PM_STATUS[31:16]. Mapping: β = min(0x80, ql >> 7).
 * Lock quality below ~256 (PM not really locked) zeroes β, degrading
 * the MLD to AM-only.
 */
void dcf77_mld_set_beta(dcf77_mld_t *mld, uint16_t pm_lock_qual);

/*
 * Feed one second of soft metrics. If this feed completes a minute
 * (detected via sec_index wrap), the previous minute's 60-bit frame
 * is attempted and, on success, populates *t and returns true.
 *
 * `t` may be NULL when the caller doesn't need the decoded output
 * (e.g. while still collecting initial minutes).
 */
bool dcf77_mld_feed(dcf77_mld_t *mld,
                    const dcf77_soft_metrics_t *s,
                    dcf77_time_t *t);

/*
 * One-shot decode helper. Combines + hard-decodes + parity-checks
 * + (optionally) flip-recovers the per-minute LLR buffer in `mld`
 * and writes the decoded time to *t on success. Returns true iff
 * the resulting frame passes parity + plausibility.
 *
 * Exposed publicly so the Python mirror can drive the same code
 * path from CSV-loaded buffers.
 */
bool dcf77_mld_decode_now(dcf77_mld_t *mld, dcf77_time_t *t);

/* ---------------------------------------------------------------- */
/* Helpers (exposed for unit testing).                               */
/* ---------------------------------------------------------------- */

/*
 * Combine α·am_soft + β·pm_soft (Q1.7) into a signed 16-b LLR,
 * saturated to int16 range. β is zeroed for seconds where pm_valid
 * is false; α is zeroed for seconds where am_valid is false.
 */
int16_t dcf77_mld_combine_one(const dcf77_mld_t *mld, uint8_t sec);

/*
 * Hard-decode the 59-bit frame buffer (seconds 0..58) from the
 * MLD's per-second combined LLRs. bits[i] is 0/1.
 */
void dcf77_mld_hard_decode(const dcf77_mld_t *mld, uint8_t bits[59]);

/*
 * Run BCD + parity + plausibility on a candidate bit array. Writes
 * to *t on success. Returns true iff the frame is internally valid.
 */
bool dcf77_mld_validate(const uint8_t bits[59], dcf77_time_t *t);

/*
 * Even-parity check: 1 if even (good), 0 if odd (bad).
 */
uint8_t dcf77_mld_parity_even(const uint8_t *bits, uint8_t n);

/*
 * Check the SIK signature in pm_soft[0..14]. Returns the number of
 * signs that matched the expected pattern (1111111111 00000, with
 * '1' meaning pm_soft < 0 = inverted-PNS, '0' meaning pm_soft >= 0).
 * Range 0..15. >= DCF77_MLD_SIK_THRESHOLD means the minute is anchored.
 */
uint8_t dcf77_mld_sik_score(const dcf77_mld_t *mld);

#ifdef __cplusplus
}
#endif

#endif /* DCF77_MLD_H_ */
