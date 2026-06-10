/*
 * test_dcf77.c — host-side unit tests for the dcf77 driver.
 *
 * Compile with -DDCF77_HOST_TEST. The fake register file replaces
 * MMIO with a flat 256-byte array; tests poke values into the array
 * and assert the unpack helpers + accessor functions return the
 * right answers. No FPGA needed.
 *
 *     gcc -DDCF77_HOST_TEST -Wall -Wextra -std=c11 \
 *         test_dcf77.c dcf77.c -o test_dcf77 && ./test_dcf77
 */
#include "dcf77.h"
#include "dcf77_io.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* ---------------------------------------------------------------- */
/* Fake register file.                                               */
/* ---------------------------------------------------------------- */

#define FAKE_BASE   ((uintptr_t)0x40000000u)
#define FAKE_SIZE   256u

static uint32_t fake_regs[FAKE_SIZE / 4];

static uint32_t host_read32(uintptr_t addr)
{
    uintptr_t off = addr - FAKE_BASE;
    if (off >= FAKE_SIZE) {
        fprintf(stderr, "test: out-of-range read @%lx\n", (unsigned long)addr);
        exit(1);
    }
    return fake_regs[off / 4];
}

static void host_write32(uintptr_t addr, uint32_t v)
{
    uintptr_t off = addr - FAKE_BASE;
    if (off >= FAKE_SIZE) {
        fprintf(stderr, "test: out-of-range write @%lx\n", (unsigned long)addr);
        exit(1);
    }
    fake_regs[off / 4] = v;
}

uint32_t (*dcf77_test_read32)(uintptr_t addr)            = host_read32;
void     (*dcf77_test_write32)(uintptr_t addr, uint32_t) = host_write32;

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

static void clear_regs(void)
{
    memset(fake_regs, 0, sizeof fake_regs);
}

/* ---------------------------------------------------------------- */
/* Individual tests.                                                 */
/* ---------------------------------------------------------------- */

static void test_sext(void)
{
    printf("test_sext\n");
    EXPECT_EQ_INT(dcf77_sext(0x7FFu, 12), 2047);
    EXPECT_EQ_INT(dcf77_sext(0x800u, 12), -2048);
    EXPECT_EQ_INT(dcf77_sext(0xFFFu, 12), -1);
    EXPECT_EQ_INT(dcf77_sext(0x000u, 12), 0);
    EXPECT_EQ_INT(dcf77_sext(0x7FFFFFFu, 28), 134217727);
    EXPECT_EQ_INT(dcf77_sext(0x8000000u, 28), -134217728);
    EXPECT_EQ_INT(dcf77_sext(0x0FFFFFFu, 24), -1);
    /* width == 32 — no-op cast */
    EXPECT_EQ_INT(dcf77_sext(0xFFFFFFFFu, 32), -1);
}

static void test_init_caps(void)
{
    printf("test_init_caps\n");
    clear_regs();
    fake_regs[DCF77_REG_STATUS / 4] =
        DCF77_STATUS_PM_PRESENT_BIT | DCF77_STATUS_PM_DEBUG_PRESENT_BIT;

    dcf77_handle_t h;
    dcf77_init(&h, FAKE_BASE);
    EXPECT(h.base == FAKE_BASE);
    EXPECT(h.pm_present);
    EXPECT(h.pm_debug_present);

    /* AM-only build */
    clear_regs();
    fake_regs[DCF77_REG_STATUS / 4] = 0u;
    dcf77_init(&h, FAKE_BASE);
    EXPECT(!h.pm_present);
    EXPECT(!h.pm_debug_present);

    /* PM but no debug */
    clear_regs();
    fake_regs[DCF77_REG_STATUS / 4] = DCF77_STATUS_PM_PRESENT_BIT;
    dcf77_init(&h, FAKE_BASE);
    EXPECT(h.pm_present);
    EXPECT(!h.pm_debug_present);
}

static void test_ctrl(void)
{
    printf("test_ctrl\n");
    clear_regs();
    dcf77_handle_t h = { .base = FAKE_BASE };

    dcf77_enable(&h, true);
    EXPECT_EQ_INT(fake_regs[0] & DCF77_CTRL_ENABLE_BIT, 1);

    dcf77_enable(&h, false);
    EXPECT_EQ_INT(fake_regs[0] & DCF77_CTRL_ENABLE_BIT, 0);

    fake_regs[0] = DCF77_CTRL_ENABLE_BIT;
    dcf77_soft_reset(&h);
    EXPECT(fake_regs[0] & DCF77_CTRL_SOFT_RESET_BIT);
    EXPECT(fake_regs[0] & DCF77_CTRL_ENABLE_BIT);
}

static void test_unpack_time(void)
{
    printf("test_unpack_time\n");
    /* TIME_HMS bit layout: [5:0] minute, [12:8] hour                 */
    /* DATE    : [7:0] year, [11:8] month, [20:16] day, [26:24] wday  */
    /* FLAGS   : [0] dst, [1] dst_ann, [2] leap_ann                    */
    uint32_t time_hms = (0x3u << 8) | 0x2Au;            /* hour=3, minute=42 */
    uint32_t date     = (0x5u << 24) | (0x15u << 16) | (0x6u << 8) | 0x18u;
    uint32_t flags    = 0x5u;                            /* dst + leap_ann */
    dcf77_time_t t;
    dcf77_unpack_time(time_hms, date, flags, DCF77_SOURCE_PM, &t);
    EXPECT_EQ_INT(t.minute, 42);
    EXPECT_EQ_INT(t.hour, 3);
    EXPECT_EQ_INT(t.year, 0x18);
    EXPECT_EQ_INT(t.month, 6);
    EXPECT_EQ_INT(t.day, 0x15);
    EXPECT_EQ_INT(t.weekday, 5);
    EXPECT(t.dst_active);
    EXPECT(!t.dst_announce);
    EXPECT(t.leap_announce);
    EXPECT(t.source == DCF77_SOURCE_PM);
}

static void test_unpack_soft_pm(void)
{
    printf("test_unpack_soft_pm\n");
    /* SOFT_PM: [5:0] sec_index, [17:6] pm_soft (12 b signed),
     *          [30] pm_valid, [31] pm_minute_mark.                  */
    uint32_t w = 0u;
    w |= (uint32_t)17u;                    /* sec_index = 17        */
    w |= ((uint32_t)0xFFFu & 0xFFFu) << 6; /* pm_soft = -1 (signed) */
    w |= 1u << 30;                          /* pm_valid              */
    w |= 1u << 31;                          /* minute_mark           */
    dcf77_soft_metrics_t s = {0};
    dcf77_unpack_soft_pm(w, &s);
    EXPECT_EQ_INT(s.sec_index, 17);
    EXPECT_EQ_INT(s.pm_soft, -1);
    EXPECT(s.pm_valid);
    EXPECT(s.pm_minute_mark);

    /* Positive max */
    w = (uint32_t)0x7FFu << 6;
    w |= 1u << 30;
    dcf77_unpack_soft_pm(w, &s);
    EXPECT_EQ_INT(s.sec_index, 0);
    EXPECT_EQ_INT(s.pm_soft, 2047);
    EXPECT(s.pm_valid);
    EXPECT(!s.pm_minute_mark);

    /* Negative max */
    w = (uint32_t)0x800u << 6;
    dcf77_unpack_soft_pm(w, &s);
    EXPECT_EQ_INT(s.pm_soft, -2048);
}

static void test_unpack_soft_am(void)
{
    printf("test_unpack_soft_am\n");
    /* SOFT_AM: [11:0] am_soft (12 b signed),
     *          [23:12] mm_soft, [29:24] bit_index,
     *          [30] am_soft_valid, [31] minute_mark.                */
    uint32_t w = 0u;
    w |= 0xFFFu;                            /* am_soft = -1          */
    w |= (uint32_t)42u << 24;               /* bit_index = 42        */
    w |= 1u << 30;
    dcf77_soft_metrics_t s = {0};
    dcf77_unpack_soft_am(w, &s);
    EXPECT_EQ_INT(s.am_soft, -1);
    EXPECT(s.am_valid);
    EXPECT(!s.am_minute_mark);
}

static void test_read_quality(void)
{
    printf("test_read_quality\n");
    clear_regs();
    /* STATUS[31..16] = rssi = 0x4321, sync = LOCK */
    fake_regs[DCF77_REG_STATUS / 4] =
        (0x4321u << 16) | DCF77_SYNC_LOCK | DCF77_STATUS_PM_PRESENT_BIT;
    fake_regs[DCF77_REG_ENV_MIN / 4] = 0x0080u;
    fake_regs[DCF77_REG_NOISE   / 4] = 0x0010u;
    /* PM_STATUS: pm_time_valid=1, pm_sync_state=LOCK, pm_lock_qual=0xBEEF */
    fake_regs[DCF77_REG_PM_STATUS / 4] =
        (0xBEEFu << 16) | (1u << 8) | (DCF77_PM_SYNC_LOCK << 6) | 0x0u;

    dcf77_handle_t h;
    dcf77_init(&h, FAKE_BASE);

    dcf77_quality_t q;
    dcf77_read_quality(&h, &q);
    EXPECT_EQ_INT(q.rssi,    0x4321);
    EXPECT_EQ_INT(q.env_min, 0x0080);
    EXPECT_EQ_INT(q.noise,   0x0010);
    EXPECT(q.am_sync_state == DCF77_SYNC_LOCK);
    EXPECT(q.pm_sync_state == DCF77_PM_SYNC_LOCK);
    EXPECT(q.pm_time_valid);
    EXPECT_EQ_INT(q.pm_lock_qual, 0xBEEF);
}

static void test_read_soft_pair(void)
{
    printf("test_read_soft_pair\n");
    clear_regs();
    fake_regs[DCF77_REG_STATUS / 4] = DCF77_STATUS_PM_PRESENT_BIT;
    /* SOFT_PM: sec=20, pm_soft=+100, valid */
    fake_regs[DCF77_REG_SOFT_PM / 4] =
        20u | ((uint32_t)100u << 6) | (1u << 30);
    /* SOFT_AM: am_soft=-50, valid */
    fake_regs[DCF77_REG_SOFT_AM / 4] =
        (uint32_t)(0xFFFu & (uint32_t)-50) | (1u << 30);

    dcf77_handle_t h;
    dcf77_init(&h, FAKE_BASE);

    dcf77_soft_metrics_t s = {0};
    bool ok = dcf77_read_soft(&h, &s);
    EXPECT(ok);
    EXPECT_EQ_INT(s.sec_index, 20);
    EXPECT_EQ_INT(s.pm_soft, 100);
    EXPECT_EQ_INT(s.am_soft, -50);
    EXPECT(s.pm_valid);
    EXPECT(s.am_valid);
}

static void test_am_only_fallback(void)
{
    printf("test_am_only_fallback\n");
    clear_regs();
    /* PM absent → sec_index taken from SOFT_AM[29:24]. */
    fake_regs[DCF77_REG_STATUS / 4] = 0u;
    fake_regs[DCF77_REG_SOFT_AM / 4] =
        ((uint32_t)33u << 24) | (1u << 30) | 0x12u;

    dcf77_handle_t h;
    dcf77_init(&h, FAKE_BASE);
    EXPECT(!h.pm_present);

    dcf77_soft_metrics_t s = {0};
    bool ok = dcf77_read_soft(&h, &s);
    EXPECT(ok);
    EXPECT_EQ_INT(s.sec_index, 33);
    EXPECT_EQ_INT(s.am_soft, 0x12);
    EXPECT(!s.pm_valid);
}

static void test_pll_gating(void)
{
    printf("test_pll_gating\n");
    clear_regs();
    fake_regs[DCF77_REG_STATUS / 4] = 0u;             /* PM_DEBUG off */

    dcf77_handle_t h;
    dcf77_init(&h, FAKE_BASE);
    EXPECT(!h.pm_debug_present);

    dcf77_pll_telem_t p = { .i_state = 99 };
    EXPECT(!dcf77_pll_read(&h, &p));
    EXPECT_EQ_INT(p.i_state, 0);                      /* zeroed out */
    EXPECT(!dcf77_pll_jam_offset(&h, 1234));
    EXPECT(!dcf77_pll_jam_i_state(&h, -77));

    /* Now enable WITH_PM_DEBUG, plant some values */
    clear_regs();
    fake_regs[DCF77_REG_STATUS / 4] =
        DCF77_STATUS_PM_PRESENT_BIT | DCF77_STATUS_PM_DEBUG_PRESENT_BIT;
    fake_regs[DCF77_REG_PLL_PHASE_FILT / 4] = 0x1FFFFFFFu;      /* 29-b -1 */
    fake_regs[DCF77_REG_PLL_I_STATE    / 4] = 0x08000000u;      /* 28-b min */
    fake_regs[DCF77_REG_PLL_OFFSET     / 4] = 0x00800000u;      /* 24-b min */

    dcf77_init(&h, FAKE_BASE);
    EXPECT(h.pm_debug_present);
    EXPECT(dcf77_pll_read(&h, &p));
    EXPECT_EQ_INT(p.phase_filt, -1);
    EXPECT_EQ_INT(p.i_state, -134217728);
    EXPECT_EQ_INT(p.offset, -8388608);

    EXPECT(dcf77_pll_jam_offset(&h, -1));
    EXPECT_EQ_INT(fake_regs[DCF77_REG_PLL_OFFSET / 4], 0x00FFFFFFu);
    EXPECT(dcf77_pll_jam_i_state(&h, -1));
    EXPECT_EQ_INT(fake_regs[DCF77_REG_PLL_I_STATE / 4], 0x0FFFFFFFu);
}

static void test_read_am_frame(void)
{
    printf("test_read_am_frame\n");
    clear_regs();
    fake_regs[DCF77_REG_LAST_FRAME0 / 4] = 0xDEADBEEFu;
    fake_regs[DCF77_REG_LAST_FRAME1 / 4] = 0x07FFFFFFu;  /* 27 bits of '1' */
    dcf77_handle_t h = { .base = FAKE_BASE };
    uint64_t f = 0u;
    dcf77_read_am_frame(&h, &f);
    EXPECT_EQ_INT(f & 0xFFFFFFFFull, 0xDEADBEEF);
    EXPECT_EQ_INT((f >> 32) & 0x07FFFFFFull, 0x07FFFFFF);
    /* Bits 59..63 must read as zero. */
    EXPECT_EQ_INT((f >> 59) & 0x1Full, 0);
}

static void test_irq_access(void)
{
    printf("test_irq_access\n");
    clear_regs();
    dcf77_handle_t h = { .base = FAKE_BASE };

    /* Reset state: flag = 0, mask = 0, second_tick = false. */
    EXPECT_EQ_INT(dcf77_irq_read_flags(&h), 0);
    EXPECT_EQ_INT(dcf77_irq_get_mask(&h), 0);
    EXPECT(!dcf77_second_tick(&h));

    /* Plant a few sticky flag bits + a mask. The fake register file
     * is direct-mapped, so reads/writes hit the same backing word. */
    fake_regs[DCF77_REG_IRQ_FLAG / 4] =
        DCF77_IRQ_SECOND_TICK_BIT | DCF77_IRQ_PLL_LOCK_RISE_BIT;
    fake_regs[DCF77_REG_IRQ_MASK / 4] = DCF77_IRQ_SECOND_TICK_BIT;

    uint32_t flags = dcf77_irq_read_flags(&h);
    EXPECT(flags & DCF77_IRQ_SECOND_TICK_BIT);
    EXPECT(flags & DCF77_IRQ_PLL_LOCK_RISE_BIT);
    EXPECT(dcf77_second_tick(&h));

    /* dcf77_irq_clear performs a W1C write at the AXI level. The
     * fake register file just stores the write (it doesn't model the
     * sticky / W1C HW behaviour), so we can only verify that the
     * write went through. */
    dcf77_irq_clear(&h, DCF77_IRQ_SECOND_TICK_BIT);
    EXPECT_EQ_INT(fake_regs[DCF77_REG_IRQ_FLAG / 4],
                  DCF77_IRQ_SECOND_TICK_BIT);  /* last write */

    dcf77_irq_set_mask(&h, 0xDEADBEEFu);
    EXPECT_EQ_INT(fake_regs[DCF77_REG_IRQ_MASK / 4], 0xDEADBEEFu);
    EXPECT_EQ_INT(dcf77_irq_get_mask(&h), 0xDEADBEEFu);

    /* clear_second_tick is the bit-0 helper. */
    fake_regs[DCF77_REG_IRQ_FLAG / 4] = 0u;
    dcf77_clear_second_tick(&h);
    EXPECT_EQ_INT(fake_regs[DCF77_REG_IRQ_FLAG / 4],
                  DCF77_IRQ_SECOND_TICK_BIT);
}

/* ---------------------------------------------------------------- */

int main(void)
{
    test_sext();
    test_init_caps();
    test_ctrl();
    test_unpack_time();
    test_unpack_soft_pm();
    test_unpack_soft_am();
    test_read_quality();
    test_read_soft_pair();
    test_am_only_fallback();
    test_pll_gating();
    test_read_am_frame();
    test_irq_access();

    if (fails == 0) {
        printf("\nAll driver unit tests passed.\n");
        return 0;
    } else {
        printf("\n%d driver unit test failure(s).\n", fails);
        return 1;
    }
}
