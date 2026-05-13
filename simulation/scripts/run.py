#!/usr/bin/env python3
"""VUnit runner for the DCF77 receiver/decoder.

Usage:
    python3 simulation/scripts/run.py             # run the full suite
    python3 simulation/scripts/run.py --clean     # rebuild from scratch
    python3 simulation/scripts/run.py --list      # list tests only
"""

from pathlib import Path
import sys

from vunit import VUnit


ROOT = Path(__file__).resolve().parent.parent.parent
INC_DIR = ROOT / "inc"
SRC_DIR = ROOT / "src" / "dcf77"
TB_DIR = ROOT / "simulation" / "tb"
DERIVED_DIR = ROOT / "simulation" / "derived"


def _add_sources(lib, path: Path) -> None:
    """Add every *.vhd / *.vhdl file under `path` to the given library."""
    if not path.is_dir():
        return
    for pattern in ("*.vhd", "*.vhdl"):
        for f in sorted(path.glob(pattern)):
            lib.add_source_file(str(f))


def build() -> VUnit:
    vu = VUnit.from_argv(
        argv=sys.argv[1:],
        compile_builtins=False,
    )
    vu.add_vhdl_builtins()
    vu.add_verification_components()

    lib = vu.add_library("dcf77")

    _add_sources(lib, INC_DIR)
    _add_sources(lib, SRC_DIR)
    _add_sources(lib, TB_DIR)

    # Apply compile/sim flags with allow_empty so this works on partial trees.
    have_sources = bool(
        list(SRC_DIR.glob("*.vhd")) or list(INC_DIR.glob("*.vhd"))
    )
    have_tbs = bool(list(TB_DIR.glob("*.vhd")))
    if have_sources:
        lib.set_compile_option(
            "ghdl.a_flags", ["-fsynopsys", "--std=08", "-frelaxed"]
        )
    if have_tbs:
        lib.set_sim_option(
            "ghdl.elab_flags", ["-fsynopsys", "--std=08", "-frelaxed"]
        )

    _configure_testbenches(lib)

    DERIVED_DIR.mkdir(parents=True, exist_ok=True)

    return vu


def _configure_testbenches(lib) -> None:
    """Register per-testbench configurations (e.g., multiple CIC widths)."""
    try:
        cic_tb = lib.test_bench("dcf77_cic_dec_tb")
    except KeyError:
        return

    # CIC0 in the cascade: R=2, no growth past the input width.
    cic_tb.add_config(
        name="r2",
        generics=dict(
            DEC_R=2,
            N_STAGES=3,
            IN_WIDTH=14,
            INT_WIDTH=17,
            COMB_WIDTH=17,
            OUT_WIDTH=17,
        ),
    )
    # CIC1: matches the widths emitted by cic_pruning.py for R=16 stage.
    cic_tb.add_config(
        name="r16",
        generics=dict(
            DEC_R=16,
            N_STAGES=3,
            IN_WIDTH=17,
            INT_WIDTH=29,
            COMB_WIDTH=28,
            OUT_WIDTH=24,
        ),
    )


def main() -> int:
    vu = build()
    vu.main()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
