/*
 * dcf77_demo.c — bare-metal Microblaze example.
 *
 * Polls the dcf77_axi_lite slave once per second and prints decoded
 * time over UART. Uses the MLD to combine AM + PM soft metrics for
 * earlier lock on marginal signals.
 *
 * Wiring assumptions:
 *   - The slave's base address is exposed via xparameters.h as
 *     XPAR_DCF77_AXI_LITE_0_BASEADDR.
 *   - Standard Xilinx UARTLite mapped through xil_printf().
 *
 * Polls run in a busy loop. For a real product, drive the poll from
 * a 1 ms timer interrupt and call this loop body from the ISR.
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

    /* Make sure the receiver is enabled (sticky bit, default '1' after
     * aresetn — but explicit is fine). */
    dcf77_enable(&dcf, true);

    /* Track which seconds we've already logged so we don't repeat. */
    uint8_t last_logged_sec = 0xFF;

    /* Re-pull pm_lock_qual every minute boundary to scale β. */
    uint8_t last_sec = 0xFF;

    for (;;) {
        if (!dcf77_second_tick(&dcf)) {
            continue;
        }

        dcf77_soft_metrics_t s;
        bool any = dcf77_read_soft(&dcf, &s);
        dcf77_clear_second_tick(&dcf);

        if (!any) {
            continue;
        }

        /* Re-scale PM weight at the minute boundary. */
        if (last_sec != 0xFFu && s.sec_index < last_sec) {
            dcf77_quality_t q;
            dcf77_read_quality(&dcf, &q);
            dcf77_mld_set_beta(&mld, q.pm_lock_qual);
        }
        last_sec = s.sec_index;

        /* Feed the MLD; on success, print the decoded time. */
        dcf77_time_t t;
        if (dcf77_mld_feed(&mld, &s, &t)) {
            print_time(&t, DCF77_SOURCE_MLD);
            continue;
        }

        /* Periodically also surface AM/PM HW-decoded times so the
         * user can see whether either path has locked yet. Avoid
         * spamming — once per minute (at sec_index == 0) is enough. */
        if (s.sec_index == 0u && last_logged_sec != 0u) {
            dcf77_time_t ht;
            if (dcf77_read_pm_time(&dcf, &ht)) {
                print_time(&ht, DCF77_SOURCE_PM);
            } else if (dcf77_read_am_time(&dcf, &ht)) {
                print_time(&ht, DCF77_SOURCE_AM);
            } else {
                dcf77_quality_t q;
                dcf77_read_quality(&dcf, &q);
                xil_printf("[wait] minute boundary  "
                           "am_sync=%u pm_sync=%u pm_qual=%u "
                           "rssi=%u env_min=%u noise=%u\r\n",
                           (unsigned)q.am_sync_state,
                           (unsigned)q.pm_sync_state,
                           (unsigned)q.pm_lock_qual,
                           (unsigned)q.rssi,
                           (unsigned)q.env_min,
                           (unsigned)q.noise);
            }
        }
        last_logged_sec = s.sec_index;
    }

    /* unreachable */
    return 0;
}
