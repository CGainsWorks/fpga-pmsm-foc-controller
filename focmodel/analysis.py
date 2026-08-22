"""Closed-loop analysis of the current loop, in continuous and in discrete time.

The controller is designed in continuous time because that is where the machine
lives and where the design intent is legible, then checked in discrete time
because that is where it runs.  Both results are reported: the continuous design
predicts a 10 to 90 per cent rise time of 1.10 ms and the exact discrete analysis
of the same gains, including the one-sample computation delay, predicts 0.91 ms.
The second number is the one the testbenches are allowed to check against.

Continuous design (docs/05-current-loop-tuning.md)
--------------------------------------------------
The per-axis plant is a first-order lag ``G(s) = 1 / (R + s L)``.  Placing the PI
zero on the plant pole, ``Ti = L / R``, cancels it and leaves

    L(s) = Kp / (s L)      so      wc = Kp / L

which fixes ``Kp = wc L`` and ``Ki = Kp R / L``.  There is exactly one free
parameter left, the bandwidth, and it is set by the sample rate rather than by
the machine.

Discrete check
--------------
Zero-order hold on the plant gives

    G(z) = (1 - p) / R * 1 / (z - p),      p = exp(-Ts R / L)

and the implemented controller, whose integrator uses the current error and so is
a backward-Euler integrator, is

    C(z) = Kp + Ki z / (z - 1) = ((Kp + Ki) z - Kp) / (z - 1)

with a zero at ``z = Kp / (Kp + Ki)``.  That is the discrete image of the same
pole-zero cancellation, accurate to first order in ``Ts R / L``.  The residual
mismatch is reported by :func:`summary` and is four parts in a hundred thousand
at the reference tuning, far below the tolerance of any real winding resistance.
"""

from __future__ import annotations

import cmath
import math

from .params import (
    KI_Q,
    KP_Q,
    L_D,
    OMEGA_C,
    Q_KI,
    Q_KP,
    R_PU,
    R_S,
    T_S,
)


def quantised_gains() -> tuple[float, float]:
    """The gains the hardware actually applies, after quantisation."""
    return KP_Q / float(1 << Q_KP), KI_Q / float(1 << Q_KI)


def plant_pole() -> float:
    """Zero-order-hold pole of the per-axis plant."""
    return math.exp(-T_S * R_S / L_D)


def closed_loop_poles(delay_samples: int = 1) -> list[complex]:
    """Poles of the sampled loop with ``delay_samples`` of computation delay.

    With the pole-zero cancellation in place the loop reduces to an integrator
    behind the delay, so the characteristic polynomial is ``z**(1+d) - z**d + a``
    with ``a = (Kp + Ki) (1 - p) / R_pu``.
    """
    kp, ki = quantised_gains()
    p = plant_pole()
    a = (kp + ki) * (1.0 - p) / R_PU
    if delay_samples == 0:
        return [complex(1.0 - a, 0.0)]
    if delay_samples == 1:
        disc = cmath.sqrt(complex(1.0 - 4.0 * a))
        return [(1.0 + disc) / 2.0, (1.0 - disc) / 2.0]
    raise ValueError("only zero or one sample of delay is modelled")


def step_response(delay_samples: int = 1, n: int = 4000) -> list[float]:
    """Unit step response of the sampled closed loop.

    With the cancellation in place the loop is an integrator behind the delay,
    so the closed loop is a / (z**(1+d) - z**d + a) and the response follows
    from its difference equation directly.  This is used rather than the
    dominant pole because for one sample of delay the second pole is not
    negligible: it is slow enough to hold the first ten per cent of the rise
    back, and quoting the dominant pole alone would predict a rise time about
    four per cent shorter than the loop actually gives.
    """
    kp, ki = quantised_gains()
    p = plant_pole()
    a = (kp + ki) * (1.0 - p) / R_PU
    d = delay_samples
    y = [0.0] * (n + d + 2)
    for k in range(d + 1, n):
        # y[k] = y[k-1] - a y[k-1-d] + a u[k-1-d], with u a unit step from 0
        y[k] = y[k - 1] - a * y[k - 1 - d] + a
    return y[:n]


def rise_time_discrete(delay_samples: int = 1) -> float:
    """10 to 90 per cent rise time of the sampled closed loop.

    Measured off the exact step response with linear interpolation between
    samples, so the answer is not quantised to the sample grid the way a
    simulation's is.
    """
    y = step_response(delay_samples)

    def cross(level: float) -> float:
        for k in range(1, len(y)):
            if y[k] >= level:
                prev = y[k - 1]
                frac = 0.0 if y[k] == prev else (level - prev) / (y[k] - prev)
                return (k - 1 + frac) * T_S
        raise RuntimeError("step response never reached the level")

    return cross(0.9) - cross(0.1)


def rise_time_continuous() -> float:
    """10 to 90 per cent rise time of the continuous first-order design."""
    return math.log(9.0) / OMEGA_C


def margins(delay_samples: int = 1) -> dict:
    """Gain and phase margin of the sampled open loop, by direct search."""
    kp, ki = quantised_gains()
    p = plant_pole()
    a = (kp + ki) * (1.0 - p) / R_PU

    def loop(w: float) -> complex:
        z = cmath.exp(1j * w * T_S)
        return a / (z ** delay_samples * (z - 1.0))

    lo, hi = 1.0, math.pi / T_S
    for _ in range(200):
        mid = math.sqrt(lo * hi)
        if abs(loop(mid)) > 1.0:
            lo = mid
        else:
            hi = mid
    w_gc = math.sqrt(lo * hi)
    pm = 180.0 + math.degrees(cmath.phase(loop(w_gc)))

    # Gain crossover of the phase, for the gain margin
    lo, hi = w_gc, math.pi / T_S
    for _ in range(200):
        mid = 0.5 * (lo + hi)
        if cmath.phase(loop(mid)) > -math.pi:
            lo = mid
        else:
            hi = mid
    gm = -20.0 * math.log10(abs(loop(0.5 * (lo + hi))))
    return {"w_gc": w_gc, "phase_margin_deg": pm, "gain_margin_db": gm}


def summary() -> dict:
    kp, ki = quantised_gains()
    p = plant_pole()
    zero = kp / (kp + ki)
    out = {
        "kp": kp,
        "ki": ki,
        "plant_pole": p,
        "controller_zero": zero,
        "cancellation_residual": abs(zero - p),
        "closed_loop_poles": [abs(z) for z in closed_loop_poles(1)],
        "rise_time_continuous_ms": rise_time_continuous() * 1e3,
        "rise_time_discrete_ms": rise_time_discrete(1) * 1e3,
    }
    out.update(margins(1))
    return out


if __name__ == "__main__":       # pragma: no cover
    for k, v in summary().items():
        print(f"{k:28s} {v}")
