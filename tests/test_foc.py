"""Properties of the transforms, the controller and the modulator."""

from __future__ import annotations

import math
import random

import pytest

from focmodel import params as P
from focmodel.fixed import to_q
from focmodel.foc import PiAxis, clarke, inv_clarke, min_max_offset, svm

SCALE = 1 << P.Q_DATA


def balanced(amp: float, theta: float) -> tuple[int, int]:
    va = amp * math.cos(theta)
    vb = amp * math.cos(theta - 2.0 * math.pi / 3.0)
    alpha = va
    beta = (va + 2.0 * vb) / math.sqrt(3.0)
    return to_q(alpha, P.Q_DATA, P.W_DATA), to_q(beta, P.Q_DATA, P.W_DATA)


# ---------------------------------------------------------------------------
# Clarke
# ---------------------------------------------------------------------------

def test_clarke_is_amplitude_invariant() -> None:
    """A balanced set of peak amplitude A must give a vector of magnitude A.

    This is the whole reason for the 2/3 in front of the transform.  Every
    per-unit limit downstream, the voltage limit above all, is stated as a space
    vector magnitude, and it would mean something 1.5 times different if the
    power-invariant scaling had been used instead.
    """
    for amp in (0.25, 0.5, 1.0, 2.0):
        for k in range(0, 360, 7):
            th = math.radians(k)
            ia = to_q(amp * math.cos(th), P.Q_DATA, P.W_DATA)
            ib = to_q(amp * math.cos(th - 2.0 * math.pi / 3.0),
                      P.Q_DATA, P.W_DATA)
            al, be, _s = clarke(ia, ib)
            assert abs(math.hypot(al, be) / SCALE - amp) < 3e-3


def test_clarke_matches_the_three_input_form() -> None:
    """Substituting i_c = -i_a - i_b must not change the answer."""
    rng = random.Random(3)
    for _ in range(1000):
        ia = rng.randint(-SCALE, SCALE)
        ib = rng.randint(-SCALE, SCALE)
        ic = -ia - ib
        al, be, _s = clarke(ia, ib)
        want_beta = (ib - ic) / math.sqrt(3.0)
        assert al == ia
        assert abs(be - want_beta) <= 1.0


def test_clarke_saturates_rather_than_wrapping() -> None:
    """An input pair outside the physical set must clip, not change sign."""
    _al, be, sat = clarke(3 * SCALE, 3 * SCALE)
    assert sat is True
    assert be == (1 << (P.W_DATA - 1)) - 1


def test_inverse_clarke_undoes_the_forward_map() -> None:
    rng = random.Random(5)
    for _ in range(1000):
        al = rng.randint(-SCALE, SCALE)
        be = rng.randint(-SCALE, SCALE)
        va, vb, vc = inv_clarke(al, be)
        assert va + vb + vc == pytest.approx(0.0, abs=2.0)
        back_al, back_be, _s = clarke(va, vb)
        assert abs(back_al - al) <= 1
        assert abs(back_be - be) <= 2


# ---------------------------------------------------------------------------
# Modulator
# ---------------------------------------------------------------------------

def test_common_mode_injection_changes_no_line_voltage() -> None:
    """The offset is free precisely because it is common to all three legs."""
    rng = random.Random(8)
    for _ in range(1000):
        al = rng.randint(-SCALE, SCALE)
        be = rng.randint(-SCALE, SCALE)
        va, vb, vc = inv_clarke(al, be)
        off = min_max_offset(va, vb, vc)
        assert (va + off) - (vb + off) == va - vb
        assert (vb + off) - (vc + off) == vb - vc


def test_linear_range_ends_at_exactly_one_per_unit() -> None:
    """The duty must span the full rail at 1.0 pu and no further.

    With V_base = V_dc/sqrt(3) the space-vector linear limit is 1.0 per unit by
    construction, so this is the check that the base and the modulator agree.
    """
    lo, hi, over = SCALE * 10, -1, False
    for k in range(0, 3600):
        al, be = balanced(1.0, k * math.pi / 1800.0)
        cmps, o = svm(al, be, P.PWM_TOP)
        lo = min(lo, min(cmps))
        hi = max(hi, max(cmps))
        over = over or o
    assert lo == 0
    assert hi == P.PWM_TOP
    assert not over


def test_above_the_linear_range_the_flag_is_raised() -> None:
    seen = False
    for k in range(0, 360, 3):
        al, be = balanced(1.05, math.radians(k))
        _c, o = svm(al, be, P.PWM_TOP)
        seen = seen or o
    assert seen


def test_injection_buys_the_expected_headroom() -> None:
    """Without injection the linear range would end at sqrt(3)/2 = 0.866.

    The ratio of the two is 2/sqrt(3) = 1.1547, the 15.47 per cent the README
    claims.  Here it is measured rather than asserted: the largest amplitude
    whose plain sinusoidal duty stays inside the rail against the largest whose
    injected duty does.
    """
    def plain_fits(amp: float) -> bool:
        for k in range(0, 360, 3):
            th = math.radians(k)
            for ph in (0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0):
                d = 0.5 + amp * math.cos(th - ph) / math.sqrt(3.0)
                if d < 0.0 or d > 1.0:
                    return False
        return True

    def injected_fits(amp: float) -> bool:
        for k in range(0, 360, 3):
            al, be = balanced(amp, math.radians(k))
            _c, o = svm(al, be, P.PWM_TOP)
            if o:
                return False
        return True

    plain = max(a / 1000.0 for a in range(1, 1200) if plain_fits(a / 1000.0))
    inject = max(a / 1000.0 for a in range(1, 1200) if injected_fits(a / 1000.0))
    assert plain == pytest.approx(math.sqrt(3.0) / 2.0, abs=3e-3)
    assert inject == pytest.approx(1.0, abs=3e-3)
    assert inject / plain == pytest.approx(2.0 / math.sqrt(3.0), rel=5e-3)


def test_duties_are_centred_in_the_rail() -> None:
    """max + min = top is what makes this modulator space-vector PWM."""
    rng = random.Random(13)
    for _ in range(2000):
        al = rng.randint(-SCALE, SCALE)
        be = rng.randint(-SCALE, SCALE)
        cmps, over = svm(al, be, P.PWM_TOP)
        if not over:
            assert abs(max(cmps) + min(cmps) - P.PWM_TOP) <= 1


# ---------------------------------------------------------------------------
# PI axis
# ---------------------------------------------------------------------------

def test_integrator_has_no_dead_zone() -> None:
    """A one-LSB error must still move the integrator.

    Holding the state in the output format instead of in the format of the
    Ki times e product is the classic way to get this wrong, and it leaves a
    permanent offset the loop cannot remove.
    """
    pi = PiAxis(P.KP_Q, P.KI_Q, P.V_MAX_Q)
    before = pi.acc
    pi.step(1, 0)
    assert pi.acc > before
    assert pi.acc - before == P.KI_Q


def test_integrator_reaches_the_reference_from_a_one_lsb_error() -> None:
    """With a plant of unit gain the loop must remove a one-LSB error."""
    pi = PiAxis(P.KP_Q, P.KI_Q, P.V_MAX_Q)
    meas = 0
    for _ in range(5000):
        u = pi.step(1, meas)
        meas = max(-1, min(1, u))
    assert meas == 1


def test_no_windup_while_saturated() -> None:
    """The state must not grow while the output is already at its limit."""
    pi = PiAxis(P.KP_Q, P.KI_Q, P.V_MAX_Q)
    for _ in range(50):
        pi.step(2 * SCALE, 0)
    assert pi.sat
    held = pi.acc
    for _ in range(500):
        pi.step(2 * SCALE, 0)
    assert pi.acc == held


def test_recovery_from_saturation_is_immediate() -> None:
    """Coming off the limit must not wait for an integrator to unwind."""
    pi = PiAxis(P.KP_Q, P.KI_Q, P.V_MAX_Q)
    for _ in range(1000):
        pi.step(2 * SCALE, 0)
    assert pi.step(2 * SCALE, 0) == P.V_MAX_Q
    # Reverse the error: the proportional term alone must take the output off
    # the positive limit on the very next sample.
    assert pi.step(-2 * SCALE, 0) < P.V_MAX_Q


def test_output_never_leaves_the_limit() -> None:
    rng = random.Random(21)
    pi = PiAxis(P.KP_Q, P.KI_Q, P.V_MAX_Q)
    for _ in range(20000):
        u = pi.step(rng.randint(-4 * SCALE, 4 * SCALE),
                    rng.randint(-4 * SCALE, 4 * SCALE))
        assert -P.V_MAX_Q <= u <= P.V_MAX_Q
