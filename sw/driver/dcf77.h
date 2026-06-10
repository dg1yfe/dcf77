/*
 * dcf77.h — bare-metal host driver for the dcf77_axi_lite slave.
 *
 * Target: AMD/Xilinx Microblaze (bare-metal C). Portable to Linux/Zynq
 * by redefining the read32/write32 helpers in dcf77_io.h.
 *
 * The slave's full register map and bit layouts are documented in
 * doc/dcf77_architecture.md §4. This header mirrors those bit fields
 * one-for-one.
 */
#ifndef DCF77_H_
#define DCF77_H_

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ---------------------------------------------------------------- */
/* Register offsets (matches inc/dcf77_pkg.vhd REG_* constants).    */
/* ---------------------------------------------------------------- */
#define DCF77_REG_CTRL              0x00u
#define DCF77_REG_STATUS            0x04u
#define DCF77_REG_TIME_HMS          0x08u   /* read latches SHADOW = DATE */
#define DCF77_REG_DATE              0x0Cu   /* year/month/day/weekday + flags bits 29..27 */
/* 0x10 reserved (formerly FLAGS; flags are now packed into DATE).         */
#define DCF77_REG_RSSI              0x14u
#define DCF77_REG_LAST_FRAME0       0x18u
#define DCF77_REG_LAST_FRAME1       0x1Cu
#define DCF77_REG_ENV_MIN           0x20u
#define DCF77_REG_NOISE             0x24u
#define DCF77_REG_SYNC_DIAG         0x28u
#define DCF77_REG_SOFT_AM           0x2Cu
#define DCF77_REG_SOFT_PM           0x30u   /* WITH_PM; read latches SHADOW = SOFT_AM */
#define DCF77_REG_IRQ_FLAG          0x34u   /* W1C sticky interrupt sources */
#define DCF77_REG_PM_STATUS         0x38u   /* WITH_PM */
#define DCF77_REG_PM_TIME_HMS       0x3Cu   /* WITH_PM; read latches SHADOW = PM_DATE */
#define DCF77_REG_PM_DATE           0x40u   /* WITH_PM; PM_DATE + flags bits 29..27 */
/* 0x44 reserved (formerly PM_FLAGS; flags packed into PM_DATE).           */
#define DCF77_REG_PM_LAST_FRAME0    0x48u   /* WITH_PM */
#define DCF77_REG_PM_LAST_FRAME1    0x4Cu   /* WITH_PM */
#define DCF77_REG_PLL_PHASE_FILT    0x50u   /* WITH_PM_DEBUG */
#define DCF77_REG_PLL_I_STATE       0x54u   /* WITH_PM_DEBUG, R/W */
#define DCF77_REG_PLL_OFFSET        0x58u   /* WITH_PM_DEBUG, R/W */
#define DCF77_REG_IRQ_MASK          0x5Cu   /* R/W per-source enables */
#define DCF77_REG_SHADOW            0x60u   /* shared atomic-pair shadow */

/* DATE / PM_DATE bit positions for the packed flags (replaces the
 * standalone FLAGS register). */
#define DCF77_DATE_DST_ACTIVE_BIT    (1u << 27)
#define DCF77_DATE_DST_ANNOUNCE_BIT  (1u << 28)
#define DCF77_DATE_LEAP_ANNOUNCE_BIT (1u << 29)

/* CTRL bits */
#define DCF77_CTRL_ENABLE_BIT       (1u << 0)
#define DCF77_CTRL_SOFT_RESET_BIT   (1u << 1)
#define DCF77_CTRL_ENV_MODE_BIT     (1u << 2)

/* STATUS bits */
#define DCF77_STATUS_SYNC_MASK            0x3u
#define DCF77_STATUS_TIME_VALID_BIT       (1u << 2)
#define DCF77_STATUS_PM_PRESENT_BIT       (1u << 3)
#define DCF77_STATUS_PM_DEBUG_PRESENT_BIT (1u << 4)
#define DCF77_STATUS_PLL_LOCK_BIT         (1u << 5)
#define DCF77_STATUS_ANCHOR_HELD_BIT      (1u << 6)

/* Interrupt source bit positions (apply to both IRQ_FLAG and IRQ_MASK). */
#define DCF77_IRQ_SECOND_TICK_BIT     (1u << 0)
#define DCF77_IRQ_AM_MINUTE_MARK_BIT  (1u << 1)
#define DCF77_IRQ_PM_MINUTE_MARK_BIT  (1u << 2)
#define DCF77_IRQ_AM_TIME_VALID_BIT   (1u << 3)
#define DCF77_IRQ_PM_TIME_VALID_BIT   (1u << 4)
#define DCF77_IRQ_AM_SYNC_LOST_BIT    (1u << 5)
#define DCF77_IRQ_PM_SYNC_LOST_BIT    (1u << 6)
#define DCF77_IRQ_PLL_LOCK_RISE_BIT   (1u << 7)
#define DCF77_IRQ_PLL_LOCK_FALL_BIT   (1u << 8)

/* ---------------------------------------------------------------- */
/* Public types.                                                     */
/* ---------------------------------------------------------------- */

typedef enum {
    DCF77_SYNC_SEARCH = 0,
    DCF77_SYNC_LOCK   = 1,
    DCF77_SYNC_HOLD   = 2
} dcf77_sync_state_t;

typedef enum {
    DCF77_PM_SYNC_FILL    = 0,
    DCF77_PM_SYNC_ACQUIRE = 1,
    DCF77_PM_SYNC_LOCK    = 2,
    DCF77_PM_SYNC_HOLD    = 3
} dcf77_pm_sync_state_t;

typedef enum {
    DCF77_SOURCE_NONE = 0,
    DCF77_SOURCE_AM   = 1,
    DCF77_SOURCE_PM   = 2,
    DCF77_SOURCE_MLD  = 3
} dcf77_source_t;

typedef struct {
    uint8_t  year;         /* 0..99 (add century in software) */
    uint8_t  month;        /* 1..12 */
    uint8_t  day;          /* 1..31 */
    uint8_t  weekday;      /* 1=Mon .. 7=Sun */
    uint8_t  hour;         /* 0..23 */
    uint8_t  minute;       /* 0..59 */
    bool     dst_active;
    bool     dst_announce;
    bool     leap_announce;
    dcf77_source_t source; /* which path produced this snapshot */
} dcf77_time_t;

typedef struct {
    uint16_t rssi;              /* env_max top 16 b */
    uint16_t env_min;
    uint16_t noise;
    uint16_t pm_lock_qual;      /* 0 if PM not present / not locked */
    dcf77_sync_state_t    am_sync_state;
    dcf77_pm_sync_state_t pm_sync_state;
    bool     pm_time_valid;
} dcf77_quality_t;

typedef struct {
    int16_t  am_soft;           /* signed 12 b → sign-extended to int16 */
    int16_t  pm_soft;           /* signed 12 b → sign-extended to int16 */
    uint8_t  am_sec_index;      /* SOFT_AM[29:24] (PM-slaved when PM in LOCK) */
    uint8_t  pm_sec_index;      /* SOFT_PM[5:0]   (atomic with pm_soft)        */
    bool     pm_valid;          /* SOFT_PM[30] */
    bool     pm_minute_mark;    /* SOFT_PM[31], HW SIK signature edge */
    bool     am_valid;          /* SOFT_AM[30] */
    bool     am_minute_mark;    /* SOFT_AM[31] */
} dcf77_soft_metrics_t;

typedef struct {
    int32_t phase_filt;         /* signed 29 b → int32 */
    int32_t i_state;            /* signed 28 b → int32 */
    int32_t offset;             /* signed 24 b → int32 */
} dcf77_pll_telem_t;

typedef struct {
    uintptr_t base;             /* slave base address (from xparameters.h) */
    bool      pm_present;       /* cached STATUS[3] at init */
    bool      pm_debug_present; /* cached STATUS[4] at init */
} dcf77_handle_t;

/* ---------------------------------------------------------------- */
/* Lifecycle.                                                        */
/* ---------------------------------------------------------------- */

/* Populate a handle. Reads STATUS once to cache capability bits. */
void dcf77_init(dcf77_handle_t *h, uintptr_t base);

/* Soft-reset the receiver core (CTRL.soft_reset pulse). */
void dcf77_soft_reset(const dcf77_handle_t *h);

/* Gate ADC samples into the pipeline (sticky). */
void dcf77_enable(const dcf77_handle_t *h, bool en);

/* ---------------------------------------------------------------- */
/* Time read-out.                                                    */
/* ---------------------------------------------------------------- */

/*
 * Read decoded time. Prefers PM when present and pm_time_valid;
 * falls back to AM time_valid. Returns false if neither path has a
 * valid frame yet.
 */
bool dcf77_read_time(const dcf77_handle_t *h, dcf77_time_t *t);

bool dcf77_read_am_time(const dcf77_handle_t *h, dcf77_time_t *t);
bool dcf77_read_pm_time(const dcf77_handle_t *h, dcf77_time_t *t);

/* ---------------------------------------------------------------- */
/* Signal quality.                                                   */
/* ---------------------------------------------------------------- */

void dcf77_read_quality(const dcf77_handle_t *h, dcf77_quality_t *q);

#ifdef DCF77_WANT_FLOAT
/* SNR_dB = 20·log10((RSSI - ENV_MIN) / NOISE). Returns -HUGE_VAL on
 * degenerate inputs. Pulls in libm log10f. */
float dcf77_snr_db(const dcf77_handle_t *h);
#endif

/* ---------------------------------------------------------------- */
/* Per-second soft-metric polling.                                   */
/* ---------------------------------------------------------------- */

/*
 * Check whether the second-tick sticky bit (IRQ_FLAG[0]) is set.
 * Returns true if there's fresh per-second data to consume.
 *
 * Equivalent to (dcf77_irq_read_flags() & DCF77_IRQ_SECOND_TICK_BIT)
 * but doesn't surface the rest of the flag word.
 */
bool dcf77_second_tick(const dcf77_handle_t *h);

/*
 * Read the atomically-sampled per-second LLR pair. SOFT_PM is read
 * first (carries pm_sec_index snapshot); SOFT_AM is then matched
 * against the same second_index via a single retry. Returns false
 * if neither side has a valid datum this second.
 */
bool dcf77_read_soft(const dcf77_handle_t *h, dcf77_soft_metrics_t *s);

/* W1C just the second-tick bit. Equivalent to
 * `dcf77_irq_clear(h, DCF77_IRQ_SECOND_TICK_BIT)`. Caller pattern:
 *    if (dcf77_second_tick(&h)) {
 *        dcf77_read_soft(&h, &s);
 *        dcf77_clear_second_tick(&h);
 *        ...
 *    }
 */
void dcf77_clear_second_tick(const dcf77_handle_t *h);

/* ---------------------------------------------------------------- */
/* Interrupts (IRQ_FLAG / IRQ_MASK at 0x34 / 0x5C).                  */
/* ---------------------------------------------------------------- */

/*
 * Read the live sticky-flag word. Each set bit corresponds to a
 * source event the hardware has captured since the last W1C clear.
 * Bit positions are the DCF77_IRQ_*_BIT macros.
 */
uint32_t dcf77_irq_read_flags(const dcf77_handle_t *h);

/*
 * W1C the specified flag bits. Pass a bitmask of DCF77_IRQ_*_BIT
 * values; for each set bit in `mask`, the corresponding sticky
 * flag is cleared. Bits already 0 are unaffected.
 */
void dcf77_irq_clear(const dcf77_handle_t *h, uint32_t mask);

/*
 * Read / write the per-source interrupt mask. The `irq` output of
 * the dcf77_axi_lite entity is high while any unmasked flag bit
 * is set; bits cleared in IRQ_MASK are still latched into
 * IRQ_FLAG but don't drive the output line.
 *
 * Reset value of IRQ_MASK is 0 — the host opts in by writing each
 * desired source.
 */
uint32_t dcf77_irq_get_mask(const dcf77_handle_t *h);
void     dcf77_irq_set_mask(const dcf77_handle_t *h, uint32_t mask);

/* ---------------------------------------------------------------- */
/* Atomic-pair reads via the shared SHADOW register (0x60).          */
/* ---------------------------------------------------------------- */

/*
 * Read SOFT_PM and SOFT_AM atomically.  The SOFT_PM read latches the
 * shared shadow with SOFT_AM from the same readmux comb cycle; this
 * function then reads the shadow.  Returns false if WITH_PM is not
 * synthesised (no shadow latch fires for the PM path).
 *
 * Use this when consuming per-second AM+PM LLR pairs in the MLD; the
 * non-atomic alternative (two separate reads) can straddle a status
 * update and emit a pair from different broadcast seconds.
 */
bool dcf77_read_soft_atomic(const dcf77_handle_t *h,
                            dcf77_soft_metrics_t *s);

/*
 * Read TIME_HMS + DATE atomically (DATE comes from the shadow latched
 * by the TIME_HMS read).  Equivalent to dcf77_read_am_time() but with
 * the AM time triple captured in a single readmux comb cycle.  Returns
 * false if STATUS.time_valid is clear.
 */
bool dcf77_read_am_time_atomic(const dcf77_handle_t *h, dcf77_time_t *t);

/*
 * PM equivalent: read PM_TIME_HMS + PM_DATE atomically via the
 * shadow latch.  Returns false if PM is absent or pm_time_valid is
 * clear.
 */
bool dcf77_read_pm_time_atomic(const dcf77_handle_t *h, dcf77_time_t *t);

/* ---------------------------------------------------------------- */
/* Raw frame access (debug).                                         */
/* ---------------------------------------------------------------- */

/* Concatenates LAST_FRAME0[0..31] + LAST_FRAME1[0..26] into bits
 * 0..58 of *frame; AM frame is 59 bits. */
void dcf77_read_am_frame(const dcf77_handle_t *h, uint64_t *frame);

/* PM frame is 59 bits (positions 0..58); bit 59 is forced '0' in the
 * SIK signature so it's not stored. Returns false if PM not present. */
bool dcf77_read_pm_frame(const dcf77_handle_t *h, uint64_t *frame);

/* ---------------------------------------------------------------- */
/* Carrier-PLL telemetry (WITH_PM_DEBUG only).                       */
/* ---------------------------------------------------------------- */

/*
 * Read all three telemetry snapshots. Returns false if WITH_PM_DEBUG
 * is not synthesised (h->pm_debug_present == false).
 */
bool dcf77_pll_read(const dcf77_handle_t *h, dcf77_pll_telem_t *p);

/*
 * Jam the carrier-PLL integrator state with a known value. One-clock
 * pulse; subsequent reads return the live HW value. Returns false if
 * WITH_PM_DEBUG is not synthesised.
 */
bool dcf77_pll_jam_i_state(const dcf77_handle_t *h, int32_t i_state);

/*
 * Jam the NCO phase_inc_offset (units: LSB of the 24-b offset).
 * Returns false if WITH_PM_DEBUG is not synthesised.
 */
bool dcf77_pll_jam_offset(const dcf77_handle_t *h, int32_t offset);

/* ---------------------------------------------------------------- */
/* Internal helpers (exposed for unit tests).                        */
/* ---------------------------------------------------------------- */

/*
 * Sign-extend the low `width` bits of v to int32_t. Width 1..32.
 * Wraps around correctly for the LLR / PLL-telemetry fields.
 */
int32_t dcf77_sext(uint32_t v, unsigned width);

/* Unpack the SOFT_PM word into a metrics struct (PM fields only). */
void dcf77_unpack_soft_pm(uint32_t soft_pm, dcf77_soft_metrics_t *s);

/* Unpack the SOFT_AM word into a metrics struct (AM fields only). */
void dcf77_unpack_soft_am(uint32_t soft_am, dcf77_soft_metrics_t *s);

/* Decode the TIME_HMS + DATE register pair into a time struct.  The
 * DST / leap-announce flags are unpacked from DATE bits 29..27 (since
 * the dedicated FLAGS register was removed in favour of packing them
 * into DATE).  Sets t->source to the caller-supplied tag.
 *
 * For atomic reads, pair this with dcf77_read_time_atomic() (or
 * compose your own from TIME_HMS + SHADOW). */
void dcf77_unpack_time(uint32_t time_hms,
                       uint32_t date,
                       dcf77_source_t src,
                       dcf77_time_t *t);

#ifdef __cplusplus
}
#endif

#endif /* DCF77_H_ */
