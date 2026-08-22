"""The fixed-point primitives, which every other number in the repository rests on."""

from __future__ import annotations

import random

import pytest

from focmodel.fixed import (
    from_q,
    q_max,
    q_min,
    round_shr,
    sat,
    saturates,
    to_q,
    wrap,
)


def test_round_shr_matches_the_definition() -> None:
    """(v + 2**(s-1)) >> s, with a floor shift, on both signs."""
    for v in range(-40, 41):
        for s in range(0, 5):
            assert round_shr(v, s) == (v if s == 0 else (v + (1 << (s - 1))) >> s)


def test_round_shr_error_is_inside_half_an_lsb() -> None:
    """Round-half-up, so the error lies in (-0.5, +0.5] and not in [-1, 0]."""
    for v in range(-1000, 1001):
        for s in (1, 3, 8):
            err = round_shr(v, s) - v / float(1 << s)
            assert -0.5 < err <= 0.5


def test_round_shr_is_unbiased_away_from_exact_halves() -> None:
    """The mean error over a full residue cycle is the half-LSB tie alone.

    Truncation would give a mean of -0.5 LSB, which an integrator downstream
    would accumulate into a real offset.  This is the reason the extra adder is
    worth spending.
    """
    s = 4
    n = 1 << s
    total = sum(round_shr(v, s) - v / float(n) for v in range(0, 4 * n))
    assert abs(total / (4 * n)) <= 0.5 / n + 1e-12


def test_saturation_clamps_and_reports() -> None:
    assert sat(1 << 20, 16) == q_max(16)
    assert sat(-(1 << 20), 16) == q_min(16)
    assert sat(123, 16) == 123
    assert saturates(1 << 20, 16)
    assert not saturates(123, 16)


def test_saturation_never_changes_sign() -> None:
    """The property that makes saturation the right choice over wrapping."""
    rng = random.Random(2)
    for _ in range(10000):
        v = rng.randint(-(1 << 24), 1 << 24)
        s = sat(v, 16)
        if v != 0:
            assert (s >= 0) == (v >= 0)


def test_wrap_is_arithmetic_modulo_a_turn() -> None:
    assert wrap(40000, 16) == 40000 - 65536
    assert wrap(-40000, 16) == 65536 - 40000
    assert wrap(32768, 16) == -32768
    assert wrap(-32768, 16) == -32768
    for v in range(-70000, 70000, 37):
        assert (wrap(v, 16) - v) % 65536 == 0
        assert -32768 <= wrap(v, 16) <= 32767


def test_wrap_makes_angle_addition_associative() -> None:
    """Angles form a residue system, so a plain adder is the correct hardware."""
    rng = random.Random(4)
    for _ in range(5000):
        a = rng.randint(-32768, 32767)
        b = rng.randint(-32768, 32767)
        c = rng.randint(-32768, 32767)
        assert wrap(wrap(a + b, 16) + c, 16) == wrap(a + wrap(b + c, 16), 16)


def test_negating_an_angle_is_exact_at_the_end_point() -> None:
    """-(-pi) is +pi and +pi and -pi are the same angle."""
    assert wrap(0 - (-32768), 16) == -32768


@pytest.mark.parametrize("frac", [12, 13, 14])
def test_q_round_trip(frac: int) -> None:
    """Round trip within half an LSB, over the range the format can hold."""
    limit = float(1 << (15 - frac))
    for x in (-0.9 * limit, -1.0, -0.001, 0.0, 0.001, 1.0, 0.9 * limit):
        i = to_q(x, frac, 16)
        assert abs(from_q(i, frac) - x) <= 0.5 / (1 << frac) + 1e-12


def test_to_q_saturates_rather_than_wrapping() -> None:
    assert to_q(100.0, 13, 16) == q_max(16)
    assert to_q(-100.0, 13, 16) == q_min(16)
