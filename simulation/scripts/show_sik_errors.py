#!/usr/bin/env python3
"""Show SIK-signature bit-errors with LLRs for each minute in a PM-decode run.

Reads the pm_bit_log lines from a VUnit test output file, locates the
best (offset, polarity) alignment of the bit stream against the
broadcast SIK pattern, then prints one block per broadcast minute:

    expected: 1     1     1     1     1   ...
    decoded:  1     1     1     1     1   ...
    LLR:    -812 -1244  -987  -533  -291  ...

with the LLR (pm_soft) value column-aligned under the corresponding bit.

Usage:
    show_sik_errors.py <path-to-output.txt>            # one specific test
    show_sik_errors.py --glob "*kp10_ki23*"            # any pattern
    show_sik_errors.py                                 # default = kp10_ki23
"""
from __future__ import annotations

import argparse
import re
import sys
from pathlib import Path

import numpy as np

PAT = re.compile(r"pm_bit #(\d+) sign=([01]) soft=(-?\d+) bit_ix=(\d+)")


def load(path: Path):
    signs, softs = [], []
    for line in path.read_text().splitlines():
        m = PAT.search(line)
        if m:
            signs.append(1 if m.group(2) == "1" else -1)
            softs.append(int(m.group(3)))
    return np.array(signs), np.array(softs)


def find_alignment(signs):
    """Return (sec_of_first_bit, polarity_flip).  polarity_flip is +1 if
    SIK '1' → sign +1 (= bit value 1 in our convention), -1 if inverted.
    """
    best = (-1, None, None)
    for flip in (+1, -1):
        for off in range(60):
            expected = np.zeros(len(signs), dtype=int)
            for k in range(len(signs)):
                sec = (off + k) % 60
                if sec <= 9:
                    expected[k] = +1
                elif sec <= 14 or sec == 59:
                    expected[k] = -1
            mask = expected != 0
            score = int(np.sum(signs[mask] == flip * expected[mask]))
            if score > best[0]:
                best = (score, off, flip)
    return best


def render_block(expected_bits, observed_bits, softs, header=""):
    """Render a single 15-bit signature block: expected, decoded, LLR."""
    cell = 6  # chars per bit cell — wide enough for "-2047 "
    def fmt_bits(bits):
        return "".join(f"{b:^{cell}d}" for b in bits)
    def fmt_softs(s):
        return "".join(f"{v:^{cell}d}" for v in s)
    diff = "".join((" " * cell) if e == o else f"{'^':^{cell}}"
                   for e, o in zip(expected_bits, observed_bits))
    if header:
        print(f"{header}")
    print(f"  expect: {fmt_bits(expected_bits)}")
    print(f"  decoded:{fmt_bits(observed_bits)}")
    print(f"  errors: {diff}")
    print(f"  LLR:    {fmt_softs(softs)}")


def main() -> int:
    p = argparse.ArgumentParser()
    p.add_argument("path", nargs="?", type=Path,
                   help="VUnit output.txt (default: any kp10_ki23 run)")
    p.add_argument("--glob", default="*kp10_ki23*",
                   help="glob to locate a test_output dir when no path given")
    args = p.parse_args()

    if args.path:
        log = args.path
    else:
        root = Path(__file__).resolve().parents[2] / "vunit_out" / "test_output"
        candidates = sorted(root.glob(f"{args.glob}/output.txt"))
        if not candidates:
            print(f"no match for glob {args.glob} under {root}", file=sys.stderr)
            return 1
        log = candidates[0]
        print(f"# Using {log}")

    signs, softs = load(log)
    if len(signs) == 0:
        print("no pm_bit_log lines found", file=sys.stderr)
        return 1
    print(f"# {len(signs)} bits in stream")

    score, off, flip = find_alignment(signs)
    print(f"# alignment: sec_of_first_bit={off}, polarity_flip={flip:+d}, "
          f"signature_score={score}")
    print()

    # Iterate signature windows.  Each minute, sec 0..14 form the signature.
    # The bit index k where sec==0 is k = (60 - off) % 60, then every +60.
    first_sig_start = (60 - off) % 60
    n_minutes = (len(signs) - first_sig_start) // 60

    n_zero_err = 0
    n_one_err  = 0
    n_two_err  = 0
    err_hist = {}
    for m in range(n_minutes):
        start = first_sig_start + m * 60
        sig_signs = signs[start:start + 15]
        sig_softs = softs[start:start + 15]
        # Convert observed sign back to bit value: with flip applied,
        # the "expected" SIK polarity is +1 for sec 0..9 ('1') and -1
        # for sec 10..14 ('0').  Observed bit_value = (flip * sign) > 0.
        # In our HW convention (post-correlator-swap), bit_value=1 for
        # SIK '1' and bit_value=0 for SIK '0'.  Express both as 0/1.
        if flip == +1:
            observed_bits = [1 if s == +1 else 0 for s in sig_signs]
            expected_bits = [1]*10 + [0]*5
        else:
            observed_bits = [0 if s == +1 else 1 for s in sig_signs]
            expected_bits = [1]*10 + [0]*5

        errs = sum(1 for e, o in zip(expected_bits, observed_bits) if e != o)
        err_hist[errs] = err_hist.get(errs, 0) + 1

        render_block(expected_bits, observed_bits, sig_softs.tolist(),
                     header=f"# minute {m+1} (bits {start}..{start+14}, "
                            f"{errs} bit error{'s' if errs != 1 else ''})")
        print()

    print()
    print(f"# HD histogram across {n_minutes} minutes:")
    for hd in sorted(err_hist):
        print(f"  HD={hd}: {err_hist[hd]} minute(s)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
