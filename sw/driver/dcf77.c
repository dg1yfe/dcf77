/*
 * dcf77.c — bare-metal driver implementation. See dcf77.h for the API.
 *
 * Reads volatile MMIO via dcf77_io.h helpers. Sign-extension done
 * centrally in dcf77_sext(). All capability gating happens via the
 * cached pm_present / pm_debug_present bits captured at dcf77_init().
 */
#include "dcf77.h"
#include "dcf77_io.h"

#ifdef DCF77_WANT_FLOAT
#include <math.h>
#endif

static inline uint32_t reg_read(const dcf77_handle_t *h, uintptr_t off)
{
    return dcf77_read32(h->base + off);
}

static inline void reg_write(const dcf77_handle_t *h, uintptr_t off, uint32_t v)
{
    dcf77_write32(h->base + off, v);
}

/* ---------------------------------------------------------------- */

int32_t dcf77_sext(uint32_t v, unsigned width)
{
    if (width == 0u || width >= 32u) {
        return (int32_t)v;
    }
    uint32_t mask = (1u << width) - 1u;
    uint32_t sign = 1u << (width - 1u);
    v &= mask;
    if (v & sign) {
        v |= ~mask;
    }
    return (int32_t)v;
}

/* ---------------------------------------------------------------- */

void dcf77_init(dcf77_handle_t *h, uintptr_t base)
{
    h->base = base;
    uint32_t status = dcf77_read32(base + DCF77_REG_STATUS);
    h->pm_present       = (status & DCF77_STATUS_PM_PRESENT_BIT) != 0u;
    h->pm_debug_present = (status & DCF77_STATUS_PM_DEBUG_PRESENT_BIT) != 0u;
}

void dcf77_soft_reset(const dcf77_handle_t *h)
{
    /* CTRL.soft_reset is self-clearing — one write pulses for one clk. */
    uint32_t ctrl = reg_read(h, DCF77_REG_CTRL);
    reg_write(h, DCF77_REG_CTRL, ctrl | DCF77_CTRL_SOFT_RESET_BIT);
}

void dcf77_enable(const dcf77_handle_t *h, bool en)
{
    uint32_t ctrl = reg_read(h, DCF77_REG_CTRL);
    if (en) {
        ctrl |= DCF77_CTRL_ENABLE_BIT;
    } else {
        ctrl &= ~DCF77_CTRL_ENABLE_BIT;
    }
    reg_write(h, DCF77_REG_CTRL, ctrl);
}

/* ---------------------------------------------------------------- */

void dcf77_unpack_time(uint32_t time_hms,
                       uint32_t date,
                       dcf77_source_t src,
                       dcf77_time_t *t)
{
    t->minute        = (uint8_t)( time_hms        & 0x3Fu);
    t->hour          = (uint8_t)((time_hms >>  8) & 0x1Fu);
    t->year          = (uint8_t)( date            & 0xFFu);
    t->month         = (uint8_t)((date     >>  8) & 0x0Fu);
    t->day           = (uint8_t)((date     >> 16) & 0x1Fu);
    t->weekday       = (uint8_t)((date     >> 24) & 0x07u);
    t->dst_active    = (date & DCF77_DATE_DST_ACTIVE_BIT)    != 0u;
    t->dst_announce  = (date & DCF77_DATE_DST_ANNOUNCE_BIT)  != 0u;
    t->leap_announce = (date & DCF77_DATE_LEAP_ANNOUNCE_BIT) != 0u;
    t->source        = src;
}

bool dcf77_read_am_time(const dcf77_handle_t *h, dcf77_time_t *t)
{
    uint32_t status = reg_read(h, DCF77_REG_STATUS);
    if ((status & DCF77_STATUS_TIME_VALID_BIT) == 0u) {
        return false;
    }
    /* Two non-atomic reads.  The race window between them is tiny
     * (TIME and DATE only update on the same frame_dec decode trigger,
     * once per minute) but callers needing strict atomicity should
     * use dcf77_read_am_time_atomic(). */
    dcf77_unpack_time(reg_read(h, DCF77_REG_TIME_HMS),
                      reg_read(h, DCF77_REG_DATE),
                      DCF77_SOURCE_AM, t);
    return true;
}

bool dcf77_read_am_time_atomic(const dcf77_handle_t *h, dcf77_time_t *t)
{
    uint32_t status = reg_read(h, DCF77_REG_STATUS);
    if ((status & DCF77_STATUS_TIME_VALID_BIT) == 0u) {
        return false;
    }
    /* TIME_HMS read latches the shared shadow with DATE from the same
     * readmux comb cycle; the SHADOW read returns the paired value. */
    uint32_t time_hms = reg_read(h, DCF77_REG_TIME_HMS);
    uint32_t date     = reg_read(h, DCF77_REG_SHADOW);
    dcf77_unpack_time(time_hms, date, DCF77_SOURCE_AM, t);
    return true;
}

bool dcf77_read_pm_time(const dcf77_handle_t *h, dcf77_time_t *t)
{
    if (!h->pm_present) {
        return false;
    }
    uint32_t pm_status = reg_read(h, DCF77_REG_PM_STATUS);
    if ((pm_status & (1u << 8)) == 0u) {           /* pm_time_valid */
        return false;
    }
    dcf77_unpack_time(reg_read(h, DCF77_REG_PM_TIME_HMS),
                      reg_read(h, DCF77_REG_PM_DATE),
                      DCF77_SOURCE_PM, t);
    return true;
}

bool dcf77_read_pm_time_atomic(const dcf77_handle_t *h, dcf77_time_t *t)
{
    if (!h->pm_present) {
        return false;
    }
    uint32_t pm_status = reg_read(h, DCF77_REG_PM_STATUS);
    if ((pm_status & (1u << 8)) == 0u) {
        return false;
    }
    uint32_t pm_time = reg_read(h, DCF77_REG_PM_TIME_HMS);
    uint32_t pm_date = reg_read(h, DCF77_REG_SHADOW);
    dcf77_unpack_time(pm_time, pm_date, DCF77_SOURCE_PM, t);
    return true;
}

bool dcf77_read_time(const dcf77_handle_t *h, dcf77_time_t *t)
{
    if (dcf77_read_pm_time(h, t)) {
        return true;
    }
    return dcf77_read_am_time(h, t);
}

/* ---------------------------------------------------------------- */

void dcf77_read_quality(const dcf77_handle_t *h, dcf77_quality_t *q)
{
    uint32_t status = reg_read(h, DCF77_REG_STATUS);
    q->rssi    = (uint16_t)((status >> 16) & 0xFFFFu);
    q->env_min = (uint16_t)( reg_read(h, DCF77_REG_ENV_MIN) & 0xFFFFu);
    q->noise   = (uint16_t)( reg_read(h, DCF77_REG_NOISE)   & 0xFFFFu);
    q->am_sync_state = (dcf77_sync_state_t)(status & DCF77_STATUS_SYNC_MASK);

    if (h->pm_present) {
        uint32_t pm = reg_read(h, DCF77_REG_PM_STATUS);
        q->pm_sync_state  = (dcf77_pm_sync_state_t)((pm >> 6) & 0x3u);
        q->pm_time_valid  = ((pm >> 8) & 1u) != 0u;
        q->pm_lock_qual   = (uint16_t)((pm >> 16) & 0xFFFFu);
    } else {
        q->pm_sync_state = DCF77_PM_SYNC_FILL;
        q->pm_time_valid = false;
        q->pm_lock_qual  = 0u;
    }
}

#ifdef DCF77_WANT_FLOAT
float dcf77_snr_db(const dcf77_handle_t *h)
{
    dcf77_quality_t q;
    dcf77_read_quality(h, &q);
    if (q.noise == 0u || q.rssi <= q.env_min) {
        return -INFINITY;
    }
    float num = (float)(q.rssi - q.env_min);
    return 20.0f * log10f(num / (float)q.noise);
}
#endif

/* ---------------------------------------------------------------- */

bool dcf77_second_tick(const dcf77_handle_t *h)
{
    return (reg_read(h, DCF77_REG_IRQ_FLAG) & DCF77_IRQ_SECOND_TICK_BIT) != 0u;
}

void dcf77_clear_second_tick(const dcf77_handle_t *h)
{
    /* IRQ_FLAG is per-bit W1C; clear just the second_tick. */
    reg_write(h, DCF77_REG_IRQ_FLAG, DCF77_IRQ_SECOND_TICK_BIT);
}

uint32_t dcf77_irq_read_flags(const dcf77_handle_t *h)
{
    return reg_read(h, DCF77_REG_IRQ_FLAG);
}

void dcf77_irq_clear(const dcf77_handle_t *h, uint32_t mask)
{
    reg_write(h, DCF77_REG_IRQ_FLAG, mask);
}

uint32_t dcf77_irq_get_mask(const dcf77_handle_t *h)
{
    return reg_read(h, DCF77_REG_IRQ_MASK);
}

void dcf77_irq_set_mask(const dcf77_handle_t *h, uint32_t mask)
{
    reg_write(h, DCF77_REG_IRQ_MASK, mask);
}

bool dcf77_read_soft_atomic(const dcf77_handle_t *h,
                            dcf77_soft_metrics_t *s)
{
    if (!h->pm_present) {
        return false;
    }
    /* SOFT_PM read latches SOFT_AM into the shared shadow on the same
     * readmux comb cycle.  Subsequent SHADOW read returns the paired
     * SOFT_AM value. */
    uint32_t soft_pm = reg_read(h, DCF77_REG_SOFT_PM);
    uint32_t soft_am = reg_read(h, DCF77_REG_SHADOW);
    dcf77_unpack_soft_pm(soft_pm, s);
    dcf77_unpack_soft_am(soft_am, s);
    return s->pm_valid || s->am_valid;
}

void dcf77_unpack_soft_pm(uint32_t soft_pm, dcf77_soft_metrics_t *s)
{
    s->sec_index      = (uint8_t)(soft_pm & 0x3Fu);
    s->pm_soft        = (int16_t)dcf77_sext((soft_pm >> 6) & 0xFFFu, 12);
    s->pm_valid       = ((soft_pm >> 30) & 1u) != 0u;
    s->pm_minute_mark = ((soft_pm >> 31) & 1u) != 0u;
}

void dcf77_unpack_soft_am(uint32_t soft_am, dcf77_soft_metrics_t *s)
{
    s->am_soft        = (int16_t)dcf77_sext(soft_am & 0xFFFu, 12);
    /* sec_index is taken from SOFT_PM when PM is present; SOFT_AM's
     * own bit_index field (bits 29..24) is filled below only as a
     * fallback for AM-only builds. */
    s->am_valid        = ((soft_am >> 30) & 1u) != 0u;
    s->am_minute_mark  = ((soft_am >> 31) & 1u) != 0u;
}

bool dcf77_read_soft(const dcf77_handle_t *h, dcf77_soft_metrics_t *s)
{
    /* Read PM first (atomic with sec_index), then AM. The combined
     * sample is "valid for this second" if either side has a fresh
     * datum. */
    uint32_t soft_pm = 0u;
    if (h->pm_present) {
        soft_pm = reg_read(h, DCF77_REG_SOFT_PM);
        dcf77_unpack_soft_pm(soft_pm, s);
    } else {
        s->sec_index      = 0u;
        s->pm_soft        = 0;
        s->pm_valid       = false;
        s->pm_minute_mark = false;
    }

    uint32_t soft_am = reg_read(h, DCF77_REG_SOFT_AM);
    dcf77_unpack_soft_am(soft_am, s);

    /* When PM is absent, fall back to the AM bit_index field
     * (SOFT_AM bits 29..24) for sec_index. */
    if (!h->pm_present) {
        s->sec_index = (uint8_t)((soft_am >> 24) & 0x3Fu);
    }

    return s->pm_valid || s->am_valid;
}

/* ---------------------------------------------------------------- */

void dcf77_read_am_frame(const dcf77_handle_t *h, uint64_t *frame)
{
    uint64_t lo = (uint64_t)reg_read(h, DCF77_REG_LAST_FRAME0);
    uint64_t hi = (uint64_t)reg_read(h, DCF77_REG_LAST_FRAME1);
    /* Bits 0..31 in LAST_FRAME0, bits 32..58 in LAST_FRAME1[26..0]. */
    *frame = lo | ((hi & 0x07FFFFFFull) << 32);
}

bool dcf77_read_pm_frame(const dcf77_handle_t *h, uint64_t *frame)
{
    if (!h->pm_present) {
        *frame = 0u;
        return false;
    }
    uint64_t lo = (uint64_t)reg_read(h, DCF77_REG_PM_LAST_FRAME0);
    uint64_t hi = (uint64_t)reg_read(h, DCF77_REG_PM_LAST_FRAME1);
    *frame = lo | ((hi & 0x07FFFFFFull) << 32);
    return true;
}

/* ---------------------------------------------------------------- */

bool dcf77_pll_read(const dcf77_handle_t *h, dcf77_pll_telem_t *p)
{
    if (!h->pm_debug_present) {
        p->phase_filt = 0;
        p->i_state    = 0;
        p->offset     = 0;
        return false;
    }
    p->phase_filt = dcf77_sext(reg_read(h, DCF77_REG_PLL_PHASE_FILT), 29);
    p->i_state    = dcf77_sext(reg_read(h, DCF77_REG_PLL_I_STATE),    28);
    p->offset     = dcf77_sext(reg_read(h, DCF77_REG_PLL_OFFSET),     24);
    return true;
}

bool dcf77_pll_jam_i_state(const dcf77_handle_t *h, int32_t i_state)
{
    if (!h->pm_debug_present) {
        return false;
    }
    /* The HW stores only the low 28 bits; write is one-clock pulse. */
    reg_write(h, DCF77_REG_PLL_I_STATE, (uint32_t)i_state & 0x0FFFFFFFu);
    return true;
}

bool dcf77_pll_jam_offset(const dcf77_handle_t *h, int32_t offset)
{
    if (!h->pm_debug_present) {
        return false;
    }
    reg_write(h, DCF77_REG_PLL_OFFSET, (uint32_t)offset & 0x00FFFFFFu);
    return true;
}
