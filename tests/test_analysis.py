"""The tuning, checked against the analysis it was derived from.

The gains are not tuned by hand and they are not tuned by simulation.  They come
from one design rule and these tests re-derive the rule and then confirm that
the closed loop the bit-accurate model produces actually behaves the way the rule
says it will.  A change to the sample rate, the machine or the word lengths has
to survive both.
"""

from __future__ import annotations

import math

import pytest

from focmodel import params as P
from focmodel.analysis import (
    closed_loop_poles,
    margins,
    plant_pole,
    quantised_gains,
    rise_time_continuous,
    rise_time_discrete,
    summary,
)
from focmodel.pmsm import run_step, step_metrics


def test_gains_follow_the_internal_model_rule() -> None:
    """Kp = wc L and Ki = Kp R / L, with the sample rate fixing wc.

    Cancelling the plant pole with the PI zero leaves L(s) = Kp/(sL), a pure
    integrator, whose crossover is Kp/L.  There is then exactly one free
    parameter and it belongs to the sample rate, not to the machine.
    """
    assert pytest.approx(P.OMEGA_C * P.L_PU, rel=1e-12) == P.KP
    assert pytest.approx(P.KP * (P.R_S / P.L_D) * P.T_S, rel=1e-12) == P.KI_TS
    assert pytest.approx(0.1, rel=1e-12) == P.OMEGA_C * P.T_S


def test_quantised_gains_are_close_to_the_ideal() -> None:
    """Quantising the gains must not move the design.

    A 0.1 per cent gain error moves the closed-loop pole by 0.1 per cent, which
    is two orders of magnitude below the temperature drift of a copper winding.
    """
    kp, ki = quantised_gains()
    assert abs(kp / P.KP - 1.0) < 2e-3
    assert abs(ki / P.KI_TS - 1.0) < 2e-3


def test_the_discrete_zero_lands_on_the_discrete_pole() -> None:
    """The continuous cancellation survives the discretisation."""
    kp, ki = quantised_gains()
    zero = kp / (kp + ki)
    assert abs(zero - plant_pole()) < 1e-4


def test_closed_loop_is_stable_with_the_computation_delay() -> None:
    poles = closed_loop_poles(1)
    assert all(abs(z) < 1.0 for z in poles)
    dominant = max(abs(z) for z in poles)
    assert 0.85 < dominant < 0.90


def test_margins_are_generous() -> None:
    """Phase margin above 60 degrees and gain margin above 12 dB.

    Both are the usual industrial thresholds.  The margins are this large
    because wc*Ts was chosen at 0.1; pushing the bandwidth up would spend them.
    """
    m = margins(1)
    assert m["phase_margin_deg"] > 60.0
    assert m["gain_margin_db"] > 12.0
    assert abs(m["w_gc"] / P.OMEGA_C - 1.0) < 0.05


def test_the_delay_costs_phase_margin_and_usable_gain() -> None:
    """What the sample-to-apply delay actually costs.

    It is not rise time.  With the cancellation in place the loop is a pure
    integrator, a/(z**d (z-1)) and adding a sample of delay moves the dominant
    pole slightly faster while introducing a second, fast one, so the 10 to 90
    per cent rise is marginally shorter rather than longer.  What the delay takes
    is margin and headroom: the sampled loop is stable for a < 2 without the
    delay and only for a < 1 with it, so the gain the loop can carry is halved
    and the phase margin at the design point drops with it.  That is the reason
    the design sits at a = 0.1 rather than anywhere near the limit.
    """
    assert margins(0)["phase_margin_deg"] > margins(1)["phase_margin_deg"]
    assert rise_time_discrete(1) < rise_time_continuous()


def test_predicted_rise_times_bracket_each_other() -> None:
    assert rise_time_continuous() == pytest.approx(1.0986e-3, rel=1e-3)
    assert rise_time_discrete(1) == pytest.approx(0.9133e-3, rel=1e-3)


@pytest.mark.parametrize("target", [0.1, 0.3, 0.5])
def test_simulated_step_matches_the_discrete_prediction(target: float) -> None:
    """The bit-accurate loop against the analysis, at standstill.

    The measurement grid is one sample, 50 us on a 912 us figure, so agreement
    is only ever meaningful to about six per cent.  Ten per cent is allowed.
    """
    trace = run_step(target, 0.0, periods=400)
    m = step_metrics(trace, target)
    assert m["t_rise"] is not None
    assert abs(m["t_rise"] - rise_time_discrete(1)) <= 0.10 * rise_time_discrete(1)
    assert abs(m["steady_error"]) < 1e-3
    assert m["overshoot"] < 0.05


def test_disturbance_rejection_runs_at_the_open_loop_time_constant() -> None:
    """Pole-zero cancellation speeds the reference response, not the disturbance.

    The cancelled plant pole is still there; it simply does not appear in the
    reference transfer function.  A back-EMF step therefore takes L/R to reject,
    five times longer than a reference step takes to follow and that is the
    whole argument for adding feedforward decoupling.  The test pins the effect
    down so that the roadmap item stays honest.
    """
    trace = run_step(0.3, 500.0, periods=1000)
    iq = [r[2] for r in trace]
    fast = round(4.0 * rise_time_discrete(1) / P.T_S)
    slow = round(3.0 * P.TAU_E / P.T_S)
    # Four reference rise times in, a loop with nothing to reject would be
    # inside a per cent of the target.  With the back-EMF to reject it is not.
    assert iq[fast] < 0.90 * 0.3
    # Three electrical time constants in, it has all but arrived and the tail
    # that remains decays at L/R rather than at the closed-loop bandwidth.
    assert abs(iq[slow] - 0.3) < 0.01
    assert abs(iq[-1] - 0.3) < 2e-3


def test_summary_is_self_consistent() -> None:
    s = summary()
    assert s["cancellation_residual"] < 1e-4
    assert s["rise_time_discrete_ms"] < s["rise_time_continuous_ms"]
    assert s["phase_margin_deg"] > 60.0


def test_sampling_bound_is_respected() -> None:
    """wc*Ts must stay well below the delay-limited stability bound.

    With one sample of computation delay the loop is a/(z(z-1)); it leaves the
    unit circle at a = 1 and a = (Kp+Ki)(1-p)/R_pu here.  The design sits at a
    tenth of that.
    """
    kp, ki = quantised_gains()
    a = (kp + ki) * (1.0 - plant_pole()) / P.R_PU
    assert a < 0.15
    assert math.isclose(a, 0.1006, rel_tol=2e-2)
