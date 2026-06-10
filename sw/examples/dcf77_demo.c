/*
 * dcf77_demo.c — bare-metal Microblaze example.
 *
 * Polls the dcf77_axi_lite slave's IRQ_FLAG register and prints
 * decoded time over UART. Uses the MLD to combine AM + PM soft
 * metrics for earlier lock on marginal signals.
 *
 * The hardware exposes a single level-sensitive `irq` line that is
 * high while any unmasked IRQ_FLAG bit is set. This demo enables a
 * useful subset of sources and dispatches per-source handlers from
 * the main loop. A production design would wire `irq` to the
 * Microblaze interrupt controller and replace the polling loop
 * with an ISR — the source-dispatch logic stays identical.
 *
 * Wiring assumptions:
 *   - The slave's base address is exposed via xparameters.h as
 *     XPAR_DCF77_AXI_LITE_0_BASEADDR.
 *   - Standard Xilinx UARTLite mapped through xil_printf().
 */
#include <xil_io.h>
#include <xil_printf.h>
#include <xparameters.h>

#include "dcf77.h"
#include "dcf77_mld.h"

#ifndef XPAR_DCF77_AXI_LITE_0_BASEADDR
#error "xparameters.h must define XPAR_DCF77_AXI_LITE_0_BASEADDR"
#endif

static const char *src_str(dcf77_source_t s)
{
    switch (s) {
    case DCF77_SOURCE_AM:  return "AM ";
    case DCF77_SOURCE_PM:  return "PM ";
    case DCF77_SOURCE_MLD: return "MLD";
    default:               return "---";
    }
}

static void print_time(const dcf77_time_t *t, dcf77_source_t mark)
{
    xil_printf("[%s] 20%02u-%02u-%02u (wd %u) %02u:%02u  "
               "dst=%u ann=%u leap=%u\r\n",
               src_str(mark),
               (unsigned)t->year, (unsigned)t->month, (unsigned)t->day,
               (unsigned)t->weekday,
               (unsigned)t->hour, (unsigned)t->minute,
               (unsigned)t->dst_active,
               (unsigned)t->dst_announce,
               (unsigned)t->leap_announce);
}

/* Sources to wake up on. SECOND_TICK drives the per-second MLD feed;
 * the *_RISE / *_LOST events surface state changes the user cares
 * about even when soft metrics are quiet. Add more bits to taste. */
#define DEMO_IRQ_MASK   (DCF77_IRQ_SECOND_TICK_BIT      \
                       | DCF77_IRQ_AM_MINUTE_MARK_BIT   \
                       | DCF77_IRQ_PM_MINUTE_MARK_BIT   \
                       | DCF77_IRQ_AM_TIME_VALID_BIT    \
                       | DCF77_IRQ_PM_TIME_VALID_BIT    \
                       | DCF77_IRQ_PLL_LOCK_RISE_BIT    \
                       | DCF77_IRQ_PLL_LOCK_FALL_BIT    \
                       | DCF77_IRQ_AM_SYNC_LOST_BIT     \
                       | DCF77_IRQ_PM_SYNC_LOST_BIT)

static void handle_second_tick(dcf77_handle_t *dcf, dcf77_mld_t *mld,
                               uint8_t *last_sec)
{
    dcf77_soft_metrics_t s;
    if (!dcf77_read_soft(dcf, &s)) {
        return;
    }

    /* Re-scale PM weight at the minute boundary (sec_index wrap). */
    if (*last_sec != 0xFFu && s.sec_index < *last_sec) {
        dcf77_quality_t q;
        dcf77_read_quality(dcf, &q);
        dcf77_mld_set_beta(mld, q.pm_lock_qual);
    }
    *last_sec = s.sec_index;

    dcf77_time_t t;
    if (dcf77_mld_feed(mld, &s, &t)) {
        print_time(&t, DCF77_SOURCE_MLD);
    }
}

int main(void)
{
    dcf77_handle_t dcf;
    dcf77_mld_t    mld;

    dcf77_init(&dcf, (uintptr_t)XPAR_DCF77_AXI_LITE_0_BASEADDR);
    dcf77_mld_init(&mld);

    xil_printf("\r\n--- DCF77 demo ---\r\n"
               "base=0x%08lx  pm_present=%u  pm_debug_present=%u\r\n",
               (unsigned long)dcf.base,
               (unsigned)dcf.pm_present,
               (unsigned)dcf.pm_debug_present);

    /* Enable the receiver and unmask the IRQ sources we care about. */
    dcf77_enable(&dcf, true);
    dcf77_irq_set_mask(&dcf, DEMO_IRQ_MASK);

    uint8_t last_sec = 0xFF;

    for (;;) {
        /* Poll the IRQ_FLAG register. A production design would
         * block in an ISR triggered by the `irq` line; the dispatch
         * below is identical in both forms. */
        uint32_t flags = dcf77_irq_read_flags(&dcf);
        if ((flags & DEMO_IRQ_MASK) == 0u) {
            continue;
        }

        /* W1C the flags we're about to service. Clearing first means
         * any source that re-fires during our handlers will set a
         * fresh flag and we'll see it next loop iteration. */
        dcf77_irq_clear(&dcf, flags & DEMO_IRQ_MASK);

        if (flags & DCF77_IRQ_PLL_LOCK_RISE_BIT) {
            xil_printf("[pll ] carrier-PLL locked\r\n");
        }
        if (flags & DCF77_IRQ_PLL_LOCK_FALL_BIT) {
            xil_printf("[pll ] carrier-PLL lost lock\r\n");
        }
        if (flags & DCF77_IRQ_AM_SYNC_LOST_BIT) {
            xil_printf("[am  ] sync lost\r\n");
        }
        if (flags & DCF77_IRQ_PM_SYNC_LOST_BIT) {
            xil_printf("[pm  ] sync lost\r\n");
        }

        if (flags & DCF77_IRQ_SECOND_TICK_BIT) {
            handle_second_tick(&dcf, &mld, &last_sec);
        }

        /* Surface HW-decoded times when they pass parity. */
        if (flags & DCF77_IRQ_PM_TIME_VALID_BIT) {
            dcf77_time_t ht;
            if (dcf77_read_pm_time(&dcf, &ht)) {
                print_time(&ht, DCF77_SOURCE_PM);
            }
        }
        if (flags & DCF77_IRQ_AM_TIME_VALID_BIT) {
            dcf77_time_t ht;
            if (dcf77_read_am_time(&dcf, &ht)) {
                print_time(&ht, DCF77_SOURCE_AM);
            }
        }
    }

    /* unreachable */
    return 0;
}
