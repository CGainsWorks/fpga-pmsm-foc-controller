"""Properties of the CORDIC and the error budget the testbenches rely on.

The bound checked here is the same one tb/tb_cordic_rot.vhd checks against the
RTL.  Having it in both places is deliberate: this test says the algorithm meets
the bound, the testbench says the hardware implements the algorithm.
"""

from __future__ import annotations

import math
import random

import pytest

from focmodel import params as P
from focmodel.cordic import ROTATE, VECTOR, cordic

SCALE = 1 << P.Q_DATA
HALF_TURN = 1 << (P.W_ANG - 1)


def test_gain_matches_the_infinite_product() -> None:
    """K_n converges quickly; at 18 stages it is the limit to eleven digits."""
    limit = 1.6467602581210656
    # 18 stages leave 1.6e-11 between K_n and its limit, which is 2**-36 of the
    # gain and therefore invisible in a 16-bit result.  Stage count stopped
    # mattering to the gain long before it stopped mattering to the angle.
    assert abs(P.K_CORDIC - limit) < 3e-11


def test_convergence_region_covers_a_quadrant() -> None:
    """The reach of the angle set must exceed pi/2 after the quadrant fold.

    Sum of atan(2**-i) over the implemented stages is 1.7433 rad.  The fold
    leaves at most pi/2 = 1.5708 rad to resolve, so there is 0.17 rad of margin
    and convergence does not depend on the exact stage count.
    """
    reach = sum(math.atan(2.0 ** -i) for i in range(P.N_STAGES))
    assert reach > math.pi / 2.0
    assert reach - math.pi / 2.0 > 0.15


def test_every_step_is_reachable_from_the_ones_after_it() -> None:
    """The sufficient condition for a binary angle set to converge."""
    a = [math.atan(2.0 ** -i) for i in range(P.N_STAGES)]
    for i in range(P.N_STAGES - 1):
        assert a[i] <= sum(a[i + 1:]) + a[-1]


def test_residual_is_below_the_input_angle_resolution() -> None:
    """The unresolved angle must be smaller than one LSB of the angle input.

    Otherwise the stage count and not the interface, would be what limits the
    accuracy of the transform.
    """
    residual = math.atan(2.0 ** -(P.N_STAGES - 1))
    lsb = 2.0 * math.pi / (1 << P.W_ANG)
    assert residual < 0.5 * lsb


@pytest.mark.parametrize("seed", [1, 2, 3])
def test_rotation_error_is_inside_the_published_bound(seed: int) -> None:
    """Worst-case rotation error over the specified input range.

    The specified range is each component within +/- 2 per unit, which covers
    twice the rated peak current.  tb_cordic_rot.vhd allows 2 LSB against real
    arithmetic; the measured figure here is what justifies that number.
    """
    rng = random.Random(seed)
    worst = 0.0
    for _ in range(4000):
        xi = rng.randint(-2 * SCALE, 2 * SCALE)
        yi = rng.randint(-2 * SCALE, 2 * SCALE)
        ai = rng.randint(-HALF_TURN, HALF_TURN - 1)
        xo, yo, _res, _s = cordic(ROTATE, xi, yi, ai)
        ang = ai * math.pi / HALF_TURN
        rx = xi * math.cos(ang) - yi * math.sin(ang)
        ry = xi * math.sin(ang) + yi * math.cos(ang)
        worst = max(worst, abs(xo - rx), abs(yo - ry))
    assert worst <= 2.0
    # The margin should be real but not enormous; if this ever fails low, the
    # word lengths grew and something is being paid for that is not needed.
    assert worst > 0.25


def test_rotation_of_the_axes_is_exact() -> None:
    """Quarter turns come out of the fold alone and must be exact."""
    one = SCALE
    quarter = HALF_TURN // 2
    assert cordic(ROTATE, one, 0, 0)[:2] == (one, 0)
    assert cordic(ROTATE, one, 0, quarter)[:2] == (0, one)
    assert cordic(ROTATE, one, 0, -quarter)[:2] == (0, -one)
    assert cordic(ROTATE, one, 0, -HALF_TURN)[:2] == (-one, 0)


def test_rotation_preserves_length() -> None:
    """A rotation is an isometry and the compensated output must show it."""
    rng = random.Random(11)
    for _ in range(500):
        xi = rng.randint(-SCALE, SCALE)
        yi = rng.randint(-SCALE, SCALE)
        ai = rng.randint(-HALF_TURN, HALF_TURN - 1)
        xo, yo, _r, _s = cordic(ROTATE, xi, yi, ai)
        before = math.hypot(xi, yi)
        after = math.hypot(xo, yo)
        assert abs(after - before) <= 2.0


@pytest.mark.parametrize("seed", [4, 5])
def test_vectoring_magnitude_and_argument(seed: int) -> None:
    rng = random.Random(seed)
    worst_m = 0.0
    worst_a = 0.0
    for _ in range(4000):
        xi = rng.randint(-2 * SCALE, 2 * SCALE)
        yi = rng.randint(-2 * SCALE, 2 * SCALE)
        mag, _z, ang, _s = cordic(VECTOR, xi, yi, 0)
        worst_m = max(worst_m, abs(mag - math.hypot(xi, yi)))
        if xi or yi:
            want = math.atan2(yi, xi) / math.pi * HALF_TURN
            d = (ang - want + HALF_TURN) % (2 * HALF_TURN) - HALF_TURN
            worst_a = max(worst_a, abs(d))
    assert worst_m <= 2.0
    assert worst_a <= 2.0


def test_vectoring_never_returns_a_negative_magnitude() -> None:
    """The voltage limiter compares the magnitude against a positive limit."""
    rng = random.Random(7)
    for _ in range(2000):
        xi = rng.randint(-2 * SCALE, 2 * SCALE)
        yi = rng.randint(-2 * SCALE, 2 * SCALE)
        assert cordic(VECTOR, xi, yi, 0)[0] >= 0


def test_rotation_residual_stays_inside_one_angle_lsb() -> None:
    """The reported residual is a free self-check on the fold and the table."""
    rng = random.Random(9)
    for _ in range(2000):
        ai = rng.randint(-HALF_TURN, HALF_TURN - 1)
        _x, _y, res, _s = cordic(ROTATE, SCALE, 0, ai)
        assert abs(res) <= 1
