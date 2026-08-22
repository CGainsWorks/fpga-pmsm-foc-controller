"""The RTL constants and the model constants must be the same constants.

focmodel generates the testbench vectors, so if a word length were changed in one
place and not the other the vectors would simply encode the wrong arithmetic and
the simulation would agree with them.  This test is the thing that stops that:
it reads the numbers out of the VHDL and compares them with the Python.
"""

from __future__ import annotations

import pathlib
import re

import pytest

from focmodel import params as P

ROOT = pathlib.Path(__file__).resolve().parents[1]
PKG = ROOT / "rtl" / "foc_pkg.vhd"

CONST_RE = re.compile(
    r"^\s*constant\s+(\w+)\s*:\s*\w+\s*:=\s*([^;]+);", re.MULTILINE
)


def vhdl_constants() -> dict[str, str]:
    text = PKG.read_text(encoding="utf-8")
    return {m.group(1).upper(): m.group(2).strip() for m in CONST_RE.finditer(text)}


@pytest.fixture(scope="module")
def consts() -> dict[str, str]:
    return vhdl_constants()


@pytest.mark.parametrize(
    ("name", "expected"),
    [
        ("C_W_DATA", P.W_DATA),
        ("C_Q_DATA", P.Q_DATA),
        ("C_W_ANG", P.W_ANG),
        ("C_W_XY", P.W_XY),
        ("C_Q_XY", P.Q_XY),
        ("C_W_Z", P.W_Z),
        ("C_N_STAGES", P.N_STAGES),
        ("C_Q_KINV", P.Q_KINV),
        ("C_KINV", P.KINV_Q),
        ("C_Q_TRIG", P.Q_TRIG),
        ("C_INV_SQRT3", P.INV_SQRT3_Q),
        ("C_SQRT3_2", P.SQRT3_2_Q),
        ("C_Q_KP", P.Q_KP),
        ("C_Q_KI", P.Q_KI),
        ("C_W_ACC", P.W_ACC),
    ],
)
def test_scalar_constant_matches(consts: dict[str, str], name: str,
                                 expected: int) -> None:
    assert name in consts, f"{name} is not declared in {PKG.name}"
    assert int(consts[name]) == expected


def test_derived_constants_are_derived(consts: dict[str, str]) -> None:
    """The guard and shift counts are expressions in the VHDL, not literals.

    They are checked by re-deriving them rather than by parsing the expression,
    which is the same thing the hardware does at elaboration.
    """
    assert consts["C_GUARD"] == "C_Q_XY - C_Q_DATA"
    assert consts["C_Z_SHIFT"] == "C_W_Z - C_W_ANG"
    assert P.GUARD_BITS == P.Q_XY - P.Q_DATA
    assert P.Q_Z_SHIFT == P.W_Z - P.W_ANG


def test_kinv_width_holds_the_constant(consts: dict[str, str]) -> None:
    """1/K must fit the multiplier operand the RTL declares for it."""
    width = int(consts["C_W_KINV"])
    assert (1 << (width - 1)) > P.KINV_Q
    # ... and be wide enough to be worth having: a narrower constant would put a
    # systematic scale error into every rotation.
    assert abs(P.KINV_Q / (1 << P.Q_KINV) * P.K_CORDIC - 1.0) < 1e-5


def test_internal_format_cannot_overflow() -> None:
    """No pair of representable inputs can overflow the CORDIC datapath.

    The largest vector the input format can present is (max, max), of magnitude
    sqrt(2) * 4 per unit, and the algorithm multiplies magnitude by at most K.
    The internal format has to hold the result, and this is the calculation that
    fixed C_W_XY at 24 bits.
    """
    max_in = (2 ** (P.W_DATA - 1) - 1) / float(1 << P.Q_DATA)
    worst = max_in * (2 ** 0.5) * P.K_CORDIC
    headroom = (2 ** (P.W_XY - 1) - 1) / float(1 << P.Q_XY)
    assert worst < headroom
    # And that the choice is not extravagant: one bit narrower would not do.
    assert worst > headroom / 2.0
