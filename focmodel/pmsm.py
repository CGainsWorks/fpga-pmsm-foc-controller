"""Averaged model of a surface-mount PMSM fed from a two-level inverter.

The machine in the rotor frame, in SI units, is

    v_d = R i_d + L_d d(i_d)/dt - w L_q i_q
    v_q = R i_q + L_q d(i_q)/dt + w L_d i_d + w psi_f

with ``w`` the electrical angular velocity.  Dividing through by the voltage
base and using ``L_pu = L I_base / V_base`` puts it in per unit without changing
its shape, which is why the controller carries no machine constants: the same
gains describe any machine once its own bases are used.

The inverter is represented by its average over one PWM period.  That is exact
for the fundamental and drops only the switching ripple, whose lowest component
sits at 20 kHz, sixty-three times above the 318 Hz closed-loop bandwidth, and is
attenuated by the machine inductance by the same factor.  The switching waveform
itself, including dead time, is checked separately in ``tb/tb_pwm3ph.vhd``; the
two testbenches together cover what a single switching-resolution closed-loop
simulation would, at a fraction of the run time.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from .fixed import sat, to_q, wrap
from .foc import CurrentLoop
from .params import (
    ADC_BITS,
    ADC_FULL_SCALE_PU,
    I_BASE,
    KI_Q,
    KP_Q,
    L_D,
    L_PU,
    L_Q,
    OMEGA_C,
    POLE_PAIRS,
    PSI_F,
    PWM_TOP,
    Q_DATA,
    R_PU,
    R_S,
    T_S,
    V_BASE,
    V_DC,
    V_MAX_Q,
    W_ANG,
    W_DATA,
)


def adc(value_pu: float) -> int:
    """Quantise a per-unit current the way the analogue front end does.

    A bipolar ADC of ``ADC_BITS`` spanning ``+/- ADC_FULL_SCALE_PU`` has an LSB of
    ``2 * ADC_FULL_SCALE_PU / 2**ADC_BITS`` = 9.77e-4 pu, eight times coarser than
    the Q2.13 datapath LSB.  The datapath is therefore not what limits current
    resolution, which is the result the word-length choice was made to produce.
    """
    lsb = 2.0 * ADC_FULL_SCALE_PU / (1 << ADC_BITS)
    q = round(value_pu / lsb) * lsb
    q = max(-ADC_FULL_SCALE_PU, min(ADC_FULL_SCALE_PU - lsb, q))
    return to_q(q, Q_DATA, W_DATA)


def theta_to_bam(theta_rad: float) -> int:
    """Electrical angle in radians to the 16-bit binary angular measure."""
    return wrap(round(theta_rad / math.pi * (1 << (W_ANG - 1))), W_ANG)


@dataclass
class Machine:
    """State of the machine in per-unit rotor-frame currents."""

    id_pu: float = 0.0
    iq_pu: float = 0.0
    theta: float = 0.0          # electrical angle, radians
    omega: float = 0.0          # electrical angular velocity, rad/s

    def advance(self, vd_pu: float, vq_pu: float, dt: float) -> None:
        """One forward-Euler step.

        The step is stable while ``dt < 2 tau_e``; the caller uses
        ``T_S / 50 = 1 us`` against ``tau_e = 4 ms``, four thousand times inside
        the bound, so the integration error is not what any measured number here
        depends on.
        """
        emf = self.omega * PSI_F / V_BASE
        did = (vd_pu - R_PU * self.id_pu + self.omega * L_PU * self.iq_pu) / L_PU
        diq = (vq_pu - R_PU * self.iq_pu - self.omega * L_PU * self.id_pu - emf) / L_PU
        self.id_pu += did * dt
        self.iq_pu += diq * dt
        self.theta += self.omega * dt

    def phase_currents(self) -> tuple[float, float]:
        """Rotor-frame currents back to the two measured phase currents."""
        c, s = math.cos(self.theta), math.sin(self.theta)
        ialpha = c * self.id_pu - s * self.iq_pu
        ibeta = s * self.id_pu + c * self.iq_pu
        ia = ialpha
        ib = (-ialpha + math.sqrt(3.0) * ibeta) / 2.0
        return ia, ib


def duties_to_dq(cmp_abc: list[int], theta: float) -> tuple[float, float]:
    """Compare values to the per-unit rotor-frame voltage actually applied."""
    v = [(c / PWM_TOP - 0.5) * V_DC for c in cmp_abc]
    valpha = (2.0 * v[0] - v[1] - v[2]) / 3.0
    vbeta = (v[1] - v[2]) / math.sqrt(3.0)
    valpha /= V_BASE
    vbeta /= V_BASE
    c, s = math.cos(theta), math.sin(theta)
    return c * valpha + s * vbeta, -s * valpha + c * vbeta


def run_step(iq_ref_pu: float, speed_rpm: float, periods: int = 200,
             substeps: int = 50, kp_q: int = KP_Q, ki_q: int = KI_Q):
    """Closed-loop step response through the bit-accurate controller.

    The duty cycles computed from the sample taken at the start of period k are
    applied during period k+1, which is what the hardware does and what makes the
    loop delay one and a half sample periods rather than half of one.
    """
    m = Machine()
    m.omega = speed_rpm / 60.0 * 2.0 * math.pi * POLE_PAIRS
    loop = CurrentLoop(kp_q, ki_q, V_MAX_Q, PWM_TOP)
    ref = to_q(iq_ref_pu, Q_DATA, W_DATA)
    applied = [PWM_TOP // 2] * 3
    trace = []
    dt = T_S / substeps
    for k in range(periods):
        ia, ib = m.phase_currents()
        out = loop.step(adc(ia), adc(ib), theta_to_bam(m.theta), 0, ref)
        for _ in range(substeps):
            vd, vq = duties_to_dq(applied, m.theta)
            m.advance(vd, vq, dt)
        applied = out["cmp"]
        trace.append((k * T_S, m.id_pu, m.iq_pu, out["iq"], out["vd"], out["vq"]))
    return trace


def step_metrics(trace, target_pu: float):
    """Rise time, overshoot and steady-state error of a step response."""
    t = [r[0] for r in trace]
    iq = [r[2] for r in trace]
    t10 = t90 = None
    for i, v in enumerate(iq):
        if t10 is None and v >= 0.1 * target_pu:
            t10 = t[i]
        if t90 is None and v >= 0.9 * target_pu:
            t90 = t[i]
            break
    peak = max(iq)
    tail = iq[int(0.8 * len(iq)):]
    settled = sum(tail) / len(tail)
    return {
        "t_rise": None if (t10 is None or t90 is None) else t90 - t10,
        "overshoot": peak / target_pu - 1.0,
        "steady_error": settled - target_pu,
    }


__all__ = [
    "I_BASE",
    "L_D",
    "L_Q",
    "OMEGA_C",
    "R_S",
    "Machine",
    "adc",
    "duties_to_dq",
    "run_step",
    "sat",
    "step_metrics",
    "theta_to_bam",
]
