"""Bit-accurate fixed-point primitives.

Every function here has a one-for-one counterpart in ``rtl/foc_pkg.vhd``.  The
models in this package are not "approximately what the hardware does", they are
the same integer operations in the same order, so a mismatch between a model and
the RTL is always a bug in one of them and never a modelling artefact.

Notation
--------
A signed fixed-point number in format ``Qm.n`` occupies ``1 + m + n`` bits: one
sign bit, ``m`` integer bits and ``n`` fractional bits.  Its value is
``i * 2**-n`` where ``i`` is the two's-complement integer stored in the register,
so the representable range is ``[-2**m, 2**m - 2**-n]`` and the resolution is
``2**-n``.

Python's ``>>`` on a negative integer is an arithmetic floor shift, which is
exactly what ``ieee.numeric_std.shift_right`` does to a ``signed``, so the two
implementations agree bit for bit on negative operands as well.
"""

from __future__ import annotations


def q_max(width: int) -> int:
    """Largest integer representable in a ``width``-bit signed register."""
    return (1 << (width - 1)) - 1


def q_min(width: int) -> int:
    """Smallest integer representable in a ``width``-bit signed register."""
    return -(1 << (width - 1))


def saturates(value: int, width: int) -> bool:
    """True when ``value`` does not fit in a ``width``-bit signed register."""
    return value > q_max(width) or value < q_min(width)


def sat(value: int, width: int) -> int:
    """Clamp ``value`` into a ``width``-bit signed register.

    Saturation rather than wrapping is used everywhere in the datapath.  A wrap
    turns a small overcurrent into a full-scale sign reversal, which a current
    controller then acts on as if it were a real measurement; saturation turns
    the same event into a bounded error the integrator can still work against.
    """
    if value > q_max(width):
        return q_max(width)
    if value < q_min(width):
        return q_min(width)
    return value


def wrap(value: int, width: int) -> int:
    """Reduce ``value`` modulo ``2**width`` into a ``width``-bit signed range.

    Used only for angles, where wrapping is not an error but the arithmetic we
    want: the binary angular measure below is a residue system modulo one turn.
    """
    m = 1 << width
    value &= m - 1
    return value - m if value >= (m >> 1) else value


def round_shr(value: int, shift: int) -> int:
    """Arithmetic right shift with round-half-up.

    ``(value + 2**(shift-1)) >> shift`` costs one adder in hardware and gives an
    error in ``(-0.5, +0.5]`` LSB, against ``[-1, 0]`` LSB for plain truncation.
    The bias matters: truncation is a systematic offset that a following
    integrator would accumulate, whereas round-half-up is unbiased except on the
    measure-zero set of exact halves.
    """
    if shift == 0:
        return value
    return (value + (1 << (shift - 1))) >> shift


def mul_shr_sat(a: int, b: int, shift: int, width: int) -> int:
    """``a * b`` requantised by ``shift`` fractional bits and saturated."""
    return sat(round_shr(a * b, shift), width)


def to_q(x: float, frac: int, width: int) -> int:
    """Convert a real number to a ``Q`` integer, rounding then saturating."""
    scaled = x * (1 << frac)
    # round-half-away-from-zero, so that -0.5 LSB and +0.5 LSB are symmetric
    i = int(scaled + 0.5) if scaled >= 0 else -int(-scaled + 0.5)
    return sat(i, width)


def from_q(i: int, frac: int) -> float:
    """Convert a ``Q`` integer back to a real number."""
    return i / float(1 << frac)
