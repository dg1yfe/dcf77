#!/usr/bin/env python3
"""Python mirror of sw/mld/dcf77_mld.c.

Implements the same bit-by-bit + single-bit-flip MLD against a CSV of
per-second LLR tuples. Used to cross-validate the C library against
the existing simulation corpus without an FPGA in the loop.

Input format (stdin or --csv path), one second per line:

    sec_index, am_soft, am_valid, pm_soft, pm_valid

`am_soft` and `pm_soft` are signed integers (int16 range);
`am_valid`/`pm_valid` are 0 or 1. Lines starting with `#` are ignored.

Output (stdout): one line per decoded minute.

    minute=MM hour=HH date=YYYY-MM-DD weekday=W dst=B announce=B leap=B

Lines preceded by `# failed` indicate a minute that didn't decode.

The sign convention matches the hardware:
    am_soft > 0  ⇒  bit '0'  (no AM dip)
    pm_soft > 0  ⇒  bit '0'  (SIK unchanged-PNS)
"""
from __future__ import annotations

import argparse
import sys
from dataclasses import dataclass, field
from pathlib import Path

W_MIN  = (1, 2, 4, 8, 10, 20, 40)
W_HR   = (1, 2, 4, 8, 10, 20)
W_DAY  = (1, 2, 4, 8, 10, 20)
W_WD   = (1, 2, 4)
W_MON  = (1, 2, 4, 8, 10)
W_YR   = (1, 2, 4, 8, 10, 20, 40, 80)


@dataclass
class Soft:
    sec_index: int
    am_soft:   int
    am_valid:  int
    pm_soft:   int
    pm_valid:  int


@dataclass
class MLD:
    alpha:       int = 0x80
    beta:        int = 0x80
    allow_flip:  bool = True
    require_sik: bool = False

    am_soft:  list[int] = field(default_factory=lambda: [0] * 60)
    pm_soft:  list[int] = field(default_factory=lambda: [0] * 60)
    am_valid: list[int] = field(default_factory=lambda: [0] * 60)
    pm_valid: list[int] = field(default_factory=lambda: [0] * 60)

    last_sec: int  = -1
    have_any: bool = False

    minutes_decoded: int = 0
    minutes_failed:  int = 0

    # ----- helpers -----
    def reset_buffers(self) -> None:
        for i in range(60):
            self.am_soft[i]  = 0
            self.pm_soft[i]  = 0
            self.am_valid[i] = 0
            self.pm_valid[i] = 0
        self.last_sec = -1
        self.have_any = False

    def set_beta(self, pm_lock_qual: int) -> None:
        b = pm_lock_qual >> 7
        if b > 0x80:
            b = 0x80
        if b < 0x10:
            b = 0
        self.beta = b

    def combine_one(self, sec: int) -> int:
        s = 0
        if self.am_valid[sec]:
            s += self.alpha * self.am_soft[sec]
        if self.pm_valid[sec]:
            s += self.beta  * self.pm_soft[sec]
        s >>= 7
        return max(-32768, min(32767, s))

    def hard_decode(self) -> list[int]:
        bits = [0] * 59
        for i in range(59):
            c = self.combine_one(i)
            bits[i] = 1 if c < 0 else 0
        return bits

    def sik_score(self) -> int:
        hits = 0
        for i in range(15):
            if not self.pm_valid[i]:
                continue
            if i < 10:
                hits += int(self.pm_soft[i] < 0)
            else:
                hits += int(self.pm_soft[i] >= 0)
        return hits

    @staticmethod
    def parity_even(bits: list[int]) -> bool:
        return (sum(bits) % 2) == 0

    def validate(self, bits: list[int]):
        if bits[0] != 0 or bits[20] != 1:
            return None
        if (bits[17] ^ bits[18]) != 1:
            return None
        if not self.parity_even(bits[21:29]):
            return None
        if not self.parity_even(bits[29:36]):
            return None
        if not self.parity_even(bits[36:59]):
            return None

        def bcd(window: list[int], w: tuple[int, ...]) -> int:
            return sum(b * c for b, c in zip(window, w))

        minute  = bcd(bits[21:28], W_MIN)
        hour    = bcd(bits[29:35], W_HR)
        day     = bcd(bits[36:42], W_DAY)
        weekday = bcd(bits[42:45], W_WD)
        month   = bcd(bits[45:50], W_MON)
        year    = bcd(bits[50:58], W_YR)

        if minute > 59 or hour > 23:
            return None
        if not (1 <= day <= 31 and 1 <= month <= 12 and 1 <= weekday <= 7):
            return None
        return {
            "minute": minute, "hour": hour, "day": day,
            "weekday": weekday, "month": month, "year": year,
            "dst_active":    bits[17],
            "dst_announce":  bits[16],
            "leap_announce": bits[19],
        }

    def flip_weakest(self, bits: list[int], start: int, count: int) -> None:
        weakest, weakest_mag = -1, 1 << 30
        for i in range(start, start + count):
            m = abs(self.combine_one(i))
            if m < weakest_mag:
                weakest_mag = m
                weakest = i
        if weakest >= 0:
            bits[weakest] ^= 1

    def try_flip(self, bits: list[int]) -> bool:
        p_min = self.parity_even(bits[21:29])
        p_hr  = self.parity_even(bits[29:36])
        p_dt  = self.parity_even(bits[36:59])
        failed = int(not p_min) + int(not p_hr) + int(not p_dt)
        if failed == 0:
            return True
        if failed > 1:
            return False
        if not p_min:
            self.flip_weakest(bits, 21, 8)
        elif not p_hr:
            self.flip_weakest(bits, 29, 7)
        else:
            self.flip_weakest(bits, 36, 23)
        return (self.parity_even(bits[21:29])
                and self.parity_even(bits[29:36])
                and self.parity_even(bits[36:59]))

    def decode_now(self):
        if self.require_sik and self.sik_score() < 12:
            self.minutes_failed += 1
            return None
        bits = self.hard_decode()
        decoded = self.validate(bits)
        if decoded is None and self.allow_flip and self.try_flip(bits):
            decoded = self.validate(bits)
        if decoded is None:
            self.minutes_failed += 1
            return None
        self.minutes_decoded += 1
        return decoded

    def feed(self, s: Soft):
        decoded = None
        if self.have_any:
            if s.sec_index < self.last_sec:
                decoded = self.decode_now()
                self.reset_buffers()
                self.have_any = True
        else:
            self.have_any = True

        if 0 <= s.sec_index < 60:
            if s.am_valid:
                self.am_soft[s.sec_index]  = s.am_soft
                self.am_valid[s.sec_index] = 1
            if s.pm_valid:
                self.pm_soft[s.sec_index]  = s.pm_soft
                self.pm_valid[s.sec_index] = 1
            self.last_sec = s.sec_index
        return decoded


def parse_csv(stream) -> list[Soft]:
    out: list[Soft] = []
    for line in stream:
        line = line.strip()
        if not line or line.startswith("#"):
            continue
        parts = [p.strip() for p in line.split(",")]
        if len(parts) < 5:
            continue
        out.append(Soft(
            sec_index = int(parts[0]),
            am_soft   = int(parts[1]),
            am_valid  = int(parts[2]),
            pm_soft   = int(parts[3]),
            pm_valid  = int(parts[4]),
        ))
    return out


def main(argv: list[str]) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--csv", type=Path,
                    help="input CSV (default stdin)")
    ap.add_argument("--alpha", type=lambda s: int(s, 0), default=0x80,
                    help="AM weight (Q1.7)")
    ap.add_argument("--beta", type=lambda s: int(s, 0), default=0x80,
                    help="PM weight (Q1.7) — overrides per-minute scaling")
    ap.add_argument("--no-flip", action="store_true",
                    help="disable single-bit-flip parity recovery")
    ap.add_argument("--require-sik", action="store_true",
                    help="require 12/15 SIK signature match")
    args = ap.parse_args(argv)

    mld = MLD(alpha=args.alpha, beta=args.beta,
              allow_flip=not args.no_flip,
              require_sik=args.require_sik)

    stream = args.csv.open() if args.csv else sys.stdin
    samples = parse_csv(stream)

    for s in samples:
        decoded = mld.feed(s)
        if decoded is not None:
            print(
                f"minute={decoded['minute']:02d} hour={decoded['hour']:02d} "
                f"date=20{decoded['year']:02d}-{decoded['month']:02d}-{decoded['day']:02d} "
                f"weekday={decoded['weekday']} dst={decoded['dst_active']} "
                f"announce={decoded['dst_announce']} leap={decoded['leap_announce']}"
            )

    print(f"# {mld.minutes_decoded} minutes decoded, "
          f"{mld.minutes_failed} failed",
          file=sys.stderr)
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
