# DCF77 example MLD

A bit-by-bit soft-decision decoder that combines per-second `am_soft`
and `pm_soft` LLRs into a hard-decoded 60-bit DCF77 frame, with
single-bit-flip parity recovery. Builds against the driver from
[`../driver/`](../driver/).

For the algorithm overview see
[`../../doc/dcf77_architecture.md`](../../doc/dcf77_architecture.md);
this README documents the API and the tunable parameters.

## API at a glance

```c
#include "dcf77_mld.h"

dcf77_mld_t mld;
dcf77_mld_init(&mld);

/* Per minute boundary: scale β from the live lock quality. */
dcf77_quality_t q;
dcf77_read_quality(&dcf, &q);
dcf77_mld_set_beta(&mld, q.pm_lock_qual);

/* Per second: read the AXI snapshot, feed the MLD, check decode. */
if (dcf77_second_tick(&dcf)) {
    dcf77_soft_metrics_t s;
    dcf77_read_soft(&dcf, &s);
    dcf77_clear_second_tick(&dcf);

    dcf77_time_t t;
    if (dcf77_mld_feed(&mld, &s, &t)) {
        /* Successful decode of the just-completed minute. */
    }
}
```

`dcf77_mld_feed` returns `true` only on the minute-boundary feed that
ends a successfully-decoded frame. All other seconds (intermediate
data accumulation, failed parity / plausibility, etc.) return `false`.

## Tunables

- **`alpha`** (Q1.7, default 0x80 = 1.0) — AM weight. Lowered when
  the AM path is known noisy (e.g. heavy industrial interference).
- **`beta`** (Q1.7, set per-minute) — PM weight. Default mapping
  `β = clamp(pm_lock_qual >> 7, 0, 0x80)` with a deadband below
  ~12.5 % of full lock that zeros β (degrades to AM-only).
- **`allow_flip`** (default `true`) — enable single-bit-flip parity
  recovery. Disable for absolutely-conservative decoding.
- **`require_sik`** (default `false`) — also gate decode on a 12/15
  SIK signature match in PM seconds 0..14. The hardware T=1
  edge-protected detector already anchors the minute boundary, so
  this is normally redundant; turn on for AM-disabled / PM-only
  reception modes.

## Footprint

`size libdcf77_mld.a` (gcc -Os, x86_64): ~1.8 kB code, 0 BSS.
Per-handle state is 244 B (mostly the 4× 60-byte LLR / valid
arrays). Microblaze build is comparable.

## Cross-validation

A bit-identical Python mirror at
[`simulation/scripts/dcf77_mld.py`](../../simulation/scripts/dcf77_mld.py)
reads a CSV of per-second LLR tuples (`sec_index, am_soft, am_valid,
pm_soft, pm_valid`) and emits the same decoded fields as this C
library. The two are used to cross-check the algorithm against
sim-TB or hardware-captured LLR streams without re-running the FPGA.

```sh
# After capturing LLRs into corpus_soft.csv:
python3 simulation/scripts/dcf77_mld.py < corpus_soft.csv > py.out
./sw/mld/dcf77_mld_test --corpus corpus_soft.csv > c.out
diff py.out c.out                  # must be bit-identical
```

(The `--corpus` mode of the test binary is a thin batch-driver; see
`dcf77_mld_test.c` for the entry point.)

## Limits

- **Two simultaneous parity failures** are out of scope. Recovery
  would require enumerating both groups' flip candidates — possible
  but not done in this example.
- **Sub-second drift** isn't modelled. The MLD assumes `sec_index`
  is monotone 0..59 within a minute; a glitch (e.g. signal loss
  mid-minute) drops the in-progress minute and resumes at the next
  full minute boundary.
- **Year-century resolution** is left to software. DCF77's BCD year
  is 00..99; add 2000 or use a sliding window relative to the local
  RTC.
