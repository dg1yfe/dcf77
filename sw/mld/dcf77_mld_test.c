/*
 * dcf77_mld_test.c — host unit tests for the MLD.
 *
 * Builds a known-good DCF77 frame in software, drops it through the
 * MLD bit-by-bit, and asserts the decoded fields match. Then tests:
 *   - parity-driven single-bit-flip recovery on a corrupted bit
 *   - SIK signature scoring
 *   - per-second feed sequencing with sec_index wrap
 *
 *     gcc -DDCF77_HOST_TEST -Wall -Wextra -std=c11 \
 *         dcf77_mld_test.c dcf77_mld.c ../driver/dcf77.c \
 *         -I. -I../driver -o dcf77_mld_test && ./dcf77_mld_test
 */
#include "dcf77_mld.h"
#include "../driver/dcf77_io.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* dcf77_io.h pulls in DCF77_HOST_TEST fakes; tests don't touch MMIO
 * (the MLD has no register access), but the driver helper functions
 * we re-use need symbols defined. */
static uint32_t dummy_read32(uintptr_t a) { (void)a; return 0; }
static void     dummy_write32(uintptr_t a, uint32_t v) { (void)a; (void)v; }
uint32_t (*dcf77_test_read32)(uintptr_t)            = dummy_read32;
void     (*dcf77_test_write32)(uintptr_t, uint32_t) = dummy_write32;

/* ---------------------------------------------------------------- */
/* Test harness.                                                     */
/* ---------------------------------------------------------------- */

static int fails = 0;

#define EXPECT(cond) do {                                           \
    if (!(cond)) {                                                  \
        fprintf(stderr, "  FAIL %s:%d  %s\n",                       \
                __FILE__, __LINE__, #cond);                         \
        fails++;                                                    \
    }                                                               \
} while (0)

#define EXPECT_EQ_INT(a, b) do {                                    \
    long _a = (long)(a), _b = (long)(b);                            \
    if (_a != _b) {                                                 \
        fprintf(stderr, "  FAIL %s:%d  %s (%ld) == %s (%ld)\n",     \
                __FILE__, __LINE__, #a, _a, #b, _b);                \
        fails++;                                                    \
    }                                                               \
} while (0)

/* ---------------------------------------------------------------- */
/* Build a known-good DCF77 frame.                                    */
/* ---------------------------------------------------------------- */

/* Encode value into LSB-first BCD over `n` bits with the given
 * weight table. Weights must accommodate the value exactly. */
static void put_bcd(uint8_t *bits, uint8_t start, const uint8_t *weights,
                    uint8_t n, uint8_t value)
{
    /* Simple greedy from MSB-weight down. Works for the canonical
     * DCF77 weight tables (1..80, sums to >= max value). */
    int v = value;
    for (int i = n - 1; i >= 0; i--) {
        if (v >= weights[i]) {
            bits[start + i] = 1u;
            v -= weights[i];
        } else {
            bits[start + i] = 0u;
        }
    }
}

static uint8_t parity_even_bit(const uint8_t *bits, uint8_t n)
{
    uint8_t s = 0;
    for (uint8_t i = 0; i < n; i++) s ^= bits[i];
    return s;   /* the parity bit must equal this so XOR-sum is 0 */
}

static const uint8_t W_MIN_T[7] = {1, 2, 4, 8, 10, 20, 40};
static const uint8_t W_HR_T[6]  = {1, 2, 4, 8, 10, 20};
static const uint8_t W_DAY_T[6] = {1, 2, 4, 8, 10, 20};
static const uint8_t W_WD_T[3]  = {1, 2, 4};
static const uint8_t W_MON_T[5] = {1, 2, 4, 8, 10};
static const uint8_t W_YR_T[8]  = {1, 2, 4, 8, 10, 20, 40, 80};

static void build_frame(uint8_t bits[59],
                        uint8_t minute, uint8_t hour,
                        uint8_t day, uint8_t weekday,
                        uint8_t month, uint8_t year,
                        bool dst_active, bool dst_announce, bool leap_announce)
{
    memset(bits, 0, 59);
    bits[0]  = 0;                         /* start of minute */
    bits[16] = dst_announce  ? 1u : 0u;
    bits[17] = dst_active    ? 1u : 0u;
    bits[18] = dst_active    ? 0u : 1u;   /* mutex with bit 17 */
    bits[19] = leap_announce ? 1u : 0u;
    bits[20] = 1u;                        /* start-of-time */

    put_bcd(bits, 21, W_MIN_T, 7, minute);
    bits[28] = parity_even_bit(&bits[21], 7);

    put_bcd(bits, 29, W_HR_T, 6, hour);
    bits[35] = parity_even_bit(&bits[29], 6);

    put_bcd(bits, 36, W_DAY_T, 6, day);
    put_bcd(bits, 42, W_WD_T,  3, weekday);
    put_bcd(bits, 45, W_MON_T, 5, month);
    put_bcd(bits, 50, W_YR_T,  8, year);
    bits[58] = parity_even_bit(&bits[36], 22);
}

/* Push the bits into MLD as soft LLRs. mag controls confidence
 * (typical 100 = high confidence). Both AM and PM agree by default. */
static void feed_frame_to_mld(dcf77_mld_t *mld,
                              const uint8_t bits[59],
                              int16_t mag)
{
    for (uint8_t sec = 0; sec < 59; sec++) {
        dcf77_soft_metrics_t s = {0};
        s.am_sec_index = sec;
        s.pm_sec_index = sec;        /* HW slaving keeps these aligned */
        s.am_valid     = true;
        s.pm_valid     = true;
        /* sign convention: positive ⇒ '0', negative ⇒ '1' */
        s.am_soft      = (int16_t)(bits[sec] ? -mag : mag);
        s.pm_soft      = (int16_t)(bits[sec] ? -mag : mag);
        dcf77_mld_feed(mld, &s, NULL);
    }
    /* Sec 59: SIK '0' (pm_soft positive), no AM bit. */
    dcf77_soft_metrics_t s = {0};
    s.am_sec_index = 59;
    s.pm_sec_index = 59;
    s.am_valid     = false;
    s.pm_valid     = true;
    s.pm_soft      = mag;
    dcf77_mld_feed(mld, &s, NULL);
}

/* Trigger a minute boundary: feed sec 0 with no data of interest. */
static bool trigger_minute_boundary(dcf77_mld_t *mld, dcf77_time_t *t)
{
    dcf77_soft_metrics_t s = {0};
    s.am_sec_index = 0;
    s.pm_sec_index = 0;
    s.am_valid     = false;
    s.pm_valid     = false;
    return dcf77_mld_feed(mld, &s, t);
}

/* ---------------------------------------------------------------- */
/* Individual tests.                                                 */
/* ---------------------------------------------------------------- */

static void test_parity_helper(void)
{
    printf("test_parity_helper\n");
    uint8_t a[] = {0, 0, 0, 0};
    EXPECT_EQ_INT(dcf77_mld_parity_even(a, 4), 1); /* all zero → even */
    uint8_t b[] = {1, 0, 0, 0};
    EXPECT_EQ_INT(dcf77_mld_parity_even(b, 4), 0);
    uint8_t c[] = {1, 1, 0, 0};
    EXPECT_EQ_INT(dcf77_mld_parity_even(c, 4), 1);
    uint8_t d[] = {1, 1, 1, 0};
    EXPECT_EQ_INT(dcf77_mld_parity_even(d, 4), 0);
}

static void test_validate_clean_frame(void)
{
    printf("test_validate_clean_frame\n");
    uint8_t bits[59];
    /* 2026-06-10, Wed, 14:37 CEST */
    build_frame(bits, 37, 14, 10, 3, 6, 26, true, false, false);
    dcf77_time_t t = {0};
    EXPECT(dcf77_mld_validate(bits, &t));
    EXPECT_EQ_INT(t.minute, 37);
    EXPECT_EQ_INT(t.hour, 14);
    EXPECT_EQ_INT(t.day, 10);
    EXPECT_EQ_INT(t.weekday, 3);
    EXPECT_EQ_INT(t.month, 6);
    EXPECT_EQ_INT(t.year, 26);
    EXPECT(t.dst_active);
    EXPECT(!t.dst_announce);
    EXPECT(!t.leap_announce);
    EXPECT(t.source == DCF77_SOURCE_MLD);
}

static void test_validate_broken_framing(void)
{
    printf("test_validate_broken_framing\n");
    uint8_t bits[59];
    build_frame(bits, 37, 14, 10, 3, 6, 26, true, false, false);

    /* bit[0] = 1 → framing fails */
    bits[0] = 1u;
    EXPECT(!dcf77_mld_validate(bits, NULL));
    bits[0] = 0u;

    /* bit[20] = 0 → time-start marker fails */
    bits[20] = 0u;
    EXPECT(!dcf77_mld_validate(bits, NULL));
    bits[20] = 1u;

    /* DST mutex broken (both bits 17,18 set) */
    bits[18] = bits[17];
    EXPECT(!dcf77_mld_validate(bits, NULL));
}

static void test_validate_implausible(void)
{
    printf("test_validate_implausible\n");
    uint8_t bits[59];
    /* minute=60 → out of range */
    build_frame(bits, 0, 14, 10, 3, 6, 26, true, false, false);
    /* hand-poke minute=60 via BCD weights: 4+10+40+... actually
     * direct: set bits to encode 60 then update parity */
    memset(&bits[21], 0, 7);
    bits[21+1] = 1; bits[21+4] = 1; bits[21+5] = 1; /* = 2+10+20 = 32, hmm */
    /* Simpler: set BCD for ones=10 (impossible value) via bits */
    bits[21+0] = 0; bits[21+1] = 1; /* 2 */
    bits[21+2] = 0; bits[21+3] = 1; /* +8 = 10 in ones nibble => impossible BCD */
    bits[21+4] = 0; bits[21+5] = 0; bits[21+6] = 0;
    bits[28] = parity_even_bit(&bits[21], 7);
    /* decoded as 10, which is plausible; let me do something clearly
     * out-of-range instead: minute = 75 (40+20+10+4+1) */
    memset(&bits[21], 0, 7);
    bits[21+0] = 1; bits[21+2] = 1; bits[21+4] = 1;
    bits[21+5] = 1; bits[21+6] = 1;
    bits[28] = parity_even_bit(&bits[21], 7);
    EXPECT(!dcf77_mld_validate(bits, NULL));
}

static void test_flip_recovery_minute(void)
{
    printf("test_flip_recovery_minute\n");
    uint8_t bits[59];
    build_frame(bits, 37, 14, 10, 3, 6, 26, true, false, false);

    /* Corrupt one minute bit; parity should now fail; flip-recovery
     * should fix it given the lowest-confidence bit is the corrupt one. */
    bits[23] ^= 1u;
    dcf77_time_t t = {0};
    EXPECT(!dcf77_mld_validate(bits, &t));   /* baseline parity fail */

    dcf77_mld_t mld;
    dcf77_mld_init(&mld);
    /* Stuff the corrupted bits as LLRs, with bit 23 at low confidence. */
    for (uint8_t i = 0; i < 59; i++) {
        mld.am_valid[i] = 1u;
        mld.pm_valid[i] = 1u;
        int16_t mag = (i == 23) ? (int16_t)5 : (int16_t)100;
        mld.am_soft[i] = (int16_t)(bits[i] ? -mag : mag);
        mld.pm_soft[i] = mld.am_soft[i];
    }
    /* PM seconds 0..14 must match SIK signature for require_sik; the
     * default initialiser sets require_sik=false so we skip that. */

    EXPECT(dcf77_mld_decode_now(&mld, &t));
    EXPECT_EQ_INT(t.minute, 37);
    EXPECT_EQ_INT(t.hour,   14);
    EXPECT_EQ_INT(mld.minutes_decoded, 1);
}

static void test_no_recovery_when_disabled(void)
{
    printf("test_no_recovery_when_disabled\n");
    uint8_t bits[59];
    build_frame(bits, 37, 14, 10, 3, 6, 26, true, false, false);
    bits[23] ^= 1u;

    dcf77_mld_t mld;
    dcf77_mld_init(&mld);
    mld.allow_flip = false;
    for (uint8_t i = 0; i < 59; i++) {
        mld.am_valid[i] = 1u;
        mld.pm_valid[i] = 1u;
        int16_t mag = (i == 23) ? (int16_t)5 : (int16_t)100;
        mld.am_soft[i] = (int16_t)(bits[i] ? -mag : mag);
        mld.pm_soft[i] = mld.am_soft[i];
    }
    dcf77_time_t t = {0};
    EXPECT(!dcf77_mld_decode_now(&mld, &t));
    EXPECT_EQ_INT(mld.minutes_failed, 1);
}

static void test_two_failed_parities_unrecoverable(void)
{
    printf("test_two_failed_parities_unrecoverable\n");
    uint8_t bits[59];
    build_frame(bits, 37, 14, 10, 3, 6, 26, true, false, false);
    bits[23] ^= 1u;   /* breaks minute parity */
    bits[40] ^= 1u;   /* breaks date parity */

    dcf77_mld_t mld;
    dcf77_mld_init(&mld);
    for (uint8_t i = 0; i < 59; i++) {
        mld.am_valid[i] = 1u;
        mld.pm_valid[i] = 1u;
        int16_t mag = 100;
        mld.am_soft[i] = (int16_t)(bits[i] ? -mag : mag);
        mld.pm_soft[i] = mld.am_soft[i];
    }
    dcf77_time_t t = {0};
    EXPECT(!dcf77_mld_decode_now(&mld, &t));
}

static void test_sik_score(void)
{
    printf("test_sik_score\n");
    dcf77_mld_t mld;
    dcf77_mld_init(&mld);
    /* Perfect SIK: seconds 0..9 → pm_soft<0, seconds 10..14 → pm_soft>=0 */
    for (uint8_t i = 0; i < 15; i++) {
        mld.pm_valid[i] = 1u;
        mld.pm_soft[i]  = (i < 10) ? (int16_t)-100 : (int16_t)100;
    }
    EXPECT_EQ_INT(dcf77_mld_sik_score(&mld), 15);

    /* Flip one PM sign → score 14 */
    mld.pm_soft[5] = 100;
    EXPECT_EQ_INT(dcf77_mld_sik_score(&mld), 14);

    /* Invalidate some seconds — they don't count */
    mld.pm_valid[0] = 0u;
    EXPECT_EQ_INT(dcf77_mld_sik_score(&mld), 13);
}

static void test_feed_minute_boundary(void)
{
    printf("test_feed_minute_boundary\n");
    dcf77_mld_t mld;
    dcf77_mld_init(&mld);

    uint8_t bits[59];
    build_frame(bits, 8, 9, 10, 3, 6, 26, true, false, false);
    feed_frame_to_mld(&mld, bits, 100);

    /* End-of-minute: next feed at sec_index 0 should trigger decode. */
    dcf77_time_t t = {0};
    EXPECT(trigger_minute_boundary(&mld, &t));
    EXPECT_EQ_INT(t.minute, 8);
    EXPECT_EQ_INT(t.hour,   9);
    EXPECT(t.source == DCF77_SOURCE_MLD);
}

static void test_beta_scheduling(void)
{
    printf("test_beta_scheduling\n");
    dcf77_mld_t mld;
    dcf77_mld_init(&mld);

    /* High PM lock quality → β saturates at 0x80 (= 1.0 Q1.7) */
    dcf77_mld_set_beta(&mld, 0xFFFFu);
    EXPECT_EQ_INT(mld.beta, 0x80);

    /* Moderate (8000 → 8000 >> 7 = 62, in deadband region but >= 0x10) */
    dcf77_mld_set_beta(&mld, 8000);
    EXPECT_EQ_INT(mld.beta, 62);

    /* Below deadband — zeros out */
    dcf77_mld_set_beta(&mld, 1000);
    EXPECT_EQ_INT(mld.beta, 0);

    dcf77_mld_set_beta(&mld, 0);
    EXPECT_EQ_INT(mld.beta, 0);
}

/* ---------------------------------------------------------------- */
/* Corpus driver: read sec_index,am_soft,am_valid,pm_soft,pm_valid   */
/* CSV from stdin and emit one decoded line per minute.  Used for     */
/* bit-identical cross-validation against simulation/scripts/         */
/* dcf77_mld.py.                                                     */
/* ---------------------------------------------------------------- */

static int corpus_mode(void)
{
    dcf77_mld_t mld;
    dcf77_mld_init(&mld);

    char line[256];
    while (fgets(line, sizeof line, stdin)) {
        /* Skip blank / comment lines. */
        char *p = line;
        while (*p == ' ' || *p == '\t') p++;
        if (*p == '\0' || *p == '\n' || *p == '#') continue;

        int sec, am, av, pm, pv;
        if (sscanf(p, "%d , %d , %d , %d , %d", &sec, &am, &av, &pm, &pv) != 5) {
            continue;
        }
        dcf77_soft_metrics_t s = {0};
        /* CSV format predates the split; treat sec as both AM and PM
         * sec_index.  The Python mirror does the same. */
        s.am_sec_index = (uint8_t)sec;
        s.pm_sec_index = (uint8_t)sec;
        s.am_soft      = (int16_t)am;
        s.am_valid     = av != 0;
        s.pm_soft      = (int16_t)pm;
        s.pm_valid     = pv != 0;

        dcf77_time_t t = {0};
        if (dcf77_mld_feed(&mld, &s, &t)) {
            printf("minute=%02u hour=%02u date=20%02u-%02u-%02u "
                   "weekday=%u dst=%u announce=%u leap=%u\n",
                   t.minute, t.hour,
                   t.year, t.month, t.day,
                   t.weekday,
                   (unsigned)t.dst_active,
                   (unsigned)t.dst_announce,
                   (unsigned)t.leap_announce);
        }
    }
    fprintf(stderr, "# %u minutes decoded, %u failed\n",
            mld.minutes_decoded, mld.minutes_failed);
    return 0;
}

/* ---------------------------------------------------------------- */

int main(int argc, char **argv)
{
    if (argc >= 2 && (strcmp(argv[1], "--corpus") == 0
                      || strcmp(argv[1], "-c") == 0)) {
        return corpus_mode();
    }

    test_parity_helper();
    test_validate_clean_frame();
    test_validate_broken_framing();
    test_validate_implausible();
    test_flip_recovery_minute();
    test_no_recovery_when_disabled();
    test_two_failed_parities_unrecoverable();
    test_sik_score();
    test_feed_minute_boundary();
    test_beta_scheduling();

    if (fails == 0) {
        printf("\nAll MLD unit tests passed.\n");
        return 0;
    } else {
        printf("\n%d MLD unit test failure(s).\n", fails);
        return 1;
    }
}
