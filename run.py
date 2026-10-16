#!/usr/bin/env python3
"""VUnit regression runner for every testbench in tb/.

    python run.py                       every testbench, in parallel with -p N
    python run.py "*closed_loop*"       one testbench or one test case
    python run.py --list                what would run

The testbenches were written self-checking before VUnit was added and they still
are: each counts its own comparisons and fails on any mismatch or on none at all.
VUnit adds discovery, one simulation per test case, parallel runs, a pass/fail
table and an xunit report CI can keep. It does not replace a single check.

The vector files are generated from focmodel by scripts/gen_vectors.py before
this runs (make sim does both). Their paths are handed to each testbench as an
absolute generic, because VUnit runs every simulation in its own output folder
and a relative path would no longer point at sim/vectors.
"""

from __future__ import annotations

import pathlib
import sys

from vunit import VUnit

ROOT = pathlib.Path(__file__).resolve().parent
VECTORS = ROOT / "sim" / "vectors"

# Dependency order does not matter to VUnit, which works it out, but the
# lists are kept explicit so a stray file in rtl/ or tb/ is not silently pulled
# into the build.
RTL = [
    "foc_pkg", "cordic_atan_pkg", "cordic", "clarke", "pi_axis", "svm",
    "current_loop", "pwm3ph", "qei_decoder", "axi4lite_regs", "foc_top",
]
TB = [
    "tb_pkg", "machine_pkg",
    "tb_cordic_rot", "tb_cordic_vec", "tb_clarke", "tb_pi_axis", "tb_svm",
    "tb_current_loop", "tb_pwm3ph", "tb_qei_decoder", "tb_axi4lite_regs",
    "tb_foc_closed_loop",
]

# Testbenches that read a generated vector file, and which file.
VECTOR_FILES = {
    "tb_cordic_rot": "cordic_rot.txt",
    "tb_cordic_vec": "cordic_vec.txt",
    "tb_clarke": "clarke.txt",
    "tb_pi_axis": "pi_axis.txt",
    "tb_svm": "svm.txt",
    "tb_current_loop": "current_loop.txt",
}


def main() -> None:
    missing = [f for f in VECTOR_FILES.values() if not (VECTORS / f).is_file()]
    if missing and "--list" not in sys.argv:
        sys.exit("vector files missing, run `make vectors` first: " + ", ".join(missing))

    vu = VUnit.from_argv(compile_builtins=False)
    vu.add_vhdl_builtins()

    lib = vu.add_library("foc_lib", vhdl_standard="2008")
    for name in RTL:
        lib.add_source_file(ROOT / "rtl" / f"{name}.vhd")
    for name in TB:
        lib.add_source_file(ROOT / "tb" / f"{name}.vhd")

    for tb, fname in VECTOR_FILES.items():
        lib.test_bench(tb).set_generic("G_VECTORS", (VECTORS / fname).as_posix())

    vu.set_compile_option("ghdl.a_flags", ["-frelaxed"])
    vu.set_sim_option("ghdl.elab_flags", ["-frelaxed"])

    # The testbenches accumulate errors and decide at the end, so a check that
    # fails reports at severity error and the run carries on to count the rest.
    # Only failure, raised by tb_pkg.finish_report or a timeout, stops it.
    vu.set_sim_option("vhdl_assert_stop_level", "failure")

    vu.main()


if __name__ == "__main__":
    main()
