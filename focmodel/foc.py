"""Bit-accurate models of the control datapath.

Each function mirrors one RTL entity.  The arithmetic is written the way the
hardware does it, including the intermediate word lengths, so that the vectors
this module generates are a specification for the RTL rather than a comparison
point with a tolerance attached.
"""

from __future__ import annotations

from dataclasses import dataclass, field

from .cordic import ROTATE, VECTOR, cordic
from .fixed import round_shr, sat, saturates, wrap
from .params import (
    INV_SQRT3_Q,
    Q_DATA,
    Q_KI,
    Q_KP,
    Q_TRIG,
    SQRT3_2_Q,
    W_ACC,
    W_ANG,
    W_DATA,
)

HALF_Q = 1 << (Q_DATA - 1)      # 0.5 in Q2.13


# ---------------------------------------------------------------------------
# Clarke transform.  docs/04-clarke-park.md
# ---------------------------------------------------------------------------

def clarke(ia: int, ib: int) -> tuple[int, int, bool]:
    """Amplitude-invariant three-to-two phase map for an isolated neutral.

        i_alpha = i_a
        i_beta  = (i_a + 2 i_b) / sqrt(3)

    The third phase current is never measured.  With an isolated star point the
    three phase currents sum to zero by Kirchhoff, so a third sensor carries no
    information the first two do not already have and leaving it out removes a
    sensor, a shunt, an amplifier and an ADC channel from every drive built on
    this block.
    """
    t = ia + 2 * ib                       # Q4.13 in 18 bits, no rounding yet
    beta = round_shr(t * INV_SQRT3_Q, Q_TRIG)
    return sat(ia, W_DATA), sat(beta, W_DATA), saturates(beta, W_DATA)


def inv_clarke(valpha: int, vbeta: int) -> tuple[int, int, int]:
    """Two-to-three phase map, the transpose of the Clarke matrix.

        v_a = v_alpha
        v_b = -v_alpha/2 + (sqrt(3)/2) v_beta
        v_c = -v_alpha/2 - (sqrt(3)/2) v_beta

    Written as one rounded shift each so that the model and the RTL round in the
    same place; splitting it into two roundings would cost half an LSB.
    """
    half = -valpha << (Q_TRIG - 1)
    vb = round_shr(half + vbeta * SQRT3_2_Q, Q_TRIG)
    vc = round_shr(half - vbeta * SQRT3_2_Q, Q_TRIG)
    return valpha, vb, vc


# ---------------------------------------------------------------------------
# PI current controller.  docs/05-current-loop-tuning.md
# ---------------------------------------------------------------------------

@dataclass
class PiAxis:
    """Discrete PI with conditional-integration anti-windup.

        e[k] = ref - meas
        I[k] = I[k-1] + Ki e[k]      (frozen while the output is saturated into
                                      the same sign as the error)
        u[k] = clamp(Kp e[k] + I[k], +/- Umax)

    The integrator is kept in the natural format of the Ki times e product,
    Q?.27 and only requantised on the way out.  That is not decoration: at the
    reference tuning Ki is 0.0180, so a one-LSB error produces 296 accumulator
    counts and the smallest error the ADC can resolve still moves the integrator
    by nearly 300 counts.  Holding the integrator in the output format instead
    would need Ki times e above 2**-13, that is an error of 68 LSB and the loop
    would sit with a permanent offset it could not remove.
    """

    kp: int
    ki: int
    umax: int
    acc: int = 0
    sat: bool = field(default=False, init=False)

    def reset(self) -> None:
        self.acc = 0
        self.sat = False

    def step(self, ref: int, meas: int) -> int:
        e = sat(ref - meas, W_DATA)
        p_term = round_shr(self.kp * e, Q_KP)          # Q2.13, wider than 16 bits
        acc_next = self.acc + self.ki * e              # Q?.27
        i_term = round_shr(acc_next, Q_KI)             # Q2.13
        u_un = p_term + i_term

        acc_lim = self.umax << Q_KI
        if u_un > self.umax:
            u = self.umax
            self.sat = True
            if e > 0:
                acc_next = self.acc                    # freeze, do not wind up
        elif u_un < -self.umax:
            u = -self.umax
            self.sat = True
            if e < 0:
                acc_next = self.acc
        else:
            u = u_un
            self.sat = False

        self.acc = max(-acc_lim, min(acc_lim, acc_next))
        assert not saturates(self.acc, W_ACC)
        return u


# ---------------------------------------------------------------------------
# Modulator.  docs/06-svpwm-and-pwm.md
# ---------------------------------------------------------------------------

def min_max_offset(va: int, vb: int, vc: int) -> int:
    """The common-mode injection that makes this modulator space-vector PWM.

        v_off = -(max(va, vb, vc) + min(va, vb, vc)) / 2

    Adding the same offset to all three legs cannot change any line-to-line
    voltage, so it cannot change the current in an isolated-neutral machine: it
    is free.  Choosing it to centre the three references inside the available
    rail is exactly what placing the two zero vectors symmetrically inside the
    switching period does, which is why this three-comparator circuit produces
    the same duty cycles as sector-and-dwell-time space-vector PWM.
    """
    return -round_shr(max(va, vb, vc) + min(va, vb, vc), 1)


def svm(valpha: int, vbeta: int, pwm_top: int) -> tuple[list[int], bool]:
    """Voltage vector in the stationary frame to three compare values."""
    va, vb, vc = inv_clarke(valpha, vbeta)
    voff = min_max_offset(va, vb, vc)
    cmps: list[int] = []
    over = False
    for v in (va, vb, vc):
        # d = 1/2 + (v + v_off)/sqrt(3), the sqrt(3) being V_base over V_dc
        d = round_shr((v + voff) * INV_SQRT3_Q, Q_TRIG) + HALF_Q
        if d < 0 or d > (1 << Q_DATA):
            over = True
        c = round_shr(d * pwm_top, Q_DATA)
        cmps.append(max(0, min(pwm_top, c)))
    return cmps, over


# ---------------------------------------------------------------------------
# The whole current loop.  docs/09-verification.md
# ---------------------------------------------------------------------------

@dataclass
class CurrentLoop:
    kp: int
    ki: int
    vmax: int
    pwm_top: int

    def __post_init__(self) -> None:
        self.pi_d = PiAxis(self.kp, self.ki, self.vmax)
        self.pi_q = PiAxis(self.kp, self.ki, self.vmax)

    def reset(self) -> None:
        self.pi_d.reset()
        self.pi_q.reset()

    def step(self, ia: int, ib: int, theta: int, id_ref: int, iq_ref: int) -> dict:
        """One control update.

        Returns every internal node the testbenches check, not only the outputs,
        so that a mismatch is localised to one stage instead of to the loop.
        """
        ialpha, ibeta, clarke_sat = clarke(ia, ib)

        # Park: rotate the measured current vector by -theta into the rotor frame
        idm, iqm, _res, _s1 = cordic(ROTATE, ialpha, ibeta, wrap(-theta, W_ANG))

        vd = sat(self.pi_d.step(id_ref, idm), W_DATA)
        vq = sat(self.pi_q.step(iq_ref, iqm), W_DATA)

        # Voltage limiting in polar form.  Vectoring gives magnitude and angle;
        # the magnitude is clipped to the linear limit and the direction is kept,
        # which is the limiting strategy that leaves the applied voltage angle
        # unchanged and so does not rotate the current vector while limiting.
        mag, _zero, phi, _s2 = cordic(VECTOR, vd, vq, 0)
        mag_lim = min(mag, self.vmax)

        # Inverse Park folded into the limiter: R(theta) R(phi) [m; 0]
        # = R(theta + phi) [m; 0], so one rotation does the work of two.
        psi = wrap(theta + phi, W_ANG)
        valpha, vbeta, _res3, _s3 = cordic(ROTATE, mag_lim, 0, psi)

        cmps, over = svm(valpha, vbeta, self.pwm_top)
        return {
            "ialpha": ialpha, "ibeta": ibeta,
            "id": idm, "iq": iqm,
            "vd": vd, "vq": vq,
            "mag": mag, "phi": phi, "mag_lim": mag_lim, "psi": psi,
            "valpha": valpha, "vbeta": vbeta,
            "cmp": cmps,
            "sat": clarke_sat or self.pi_d.sat or self.pi_q.sat or over,
        }
