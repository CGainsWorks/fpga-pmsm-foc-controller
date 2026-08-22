"""Word lengths, fixed-point formats and the reference machine.

Nothing in this file is a free choice: every number is derived in the document
named beside it and ``tests/test_error_budget.py`` re-derives the important ones
from first principles so that a hand edit here fails the test suite.
"""

from __future__ import annotations

import math

# ---------------------------------------------------------------------------
# Word lengths and fixed-point formats.  See docs/02-fixed-point.md.
# ---------------------------------------------------------------------------

W_DATA = 16           # per-unit currents and voltages
Q_DATA = 13           # ... in Q2.13, range [-4, +4), resolution 2**-13
W_ANG = 16            # binary angular measure, 2**16 counts to the turn
W_XY = 24             # CORDIC x/y datapath
Q_XY = 19             # ... in Q4.19, range [-16, +16)
W_Z = 22              # CORDIC angle accumulator, 2**21 counts to the half turn
Q_Z_SHIFT = W_Z - W_ANG   # left shift taking a BAM16 angle into the Z format
N_STAGES = 18         # CORDIC iterations
W_KP = 16
Q_KP = 12             # proportional gain in Q3.12, range [-8, +8)
W_KI = 16
Q_KI = 14             # integral gain in Q1.14, range [-2, +2)
W_ACC = 32            # PI integrator accumulator
Q_ACC = Q_DATA + Q_KI  # = 27, the natural format of the Ki * e product

# Guard bits carried through the CORDIC over the input format.
GUARD_BITS = Q_XY - Q_DATA        # 6

# ---------------------------------------------------------------------------
# Derived constants.  See docs/03-cordic.md.
# ---------------------------------------------------------------------------

def cordic_gain(n: int = N_STAGES) -> float:
    """K_n = prod_{i=0}^{n-1} sqrt(1 + 2**(-2i)), the CORDIC processing gain."""
    k = 1.0
    for i in range(n):
        k *= math.sqrt(1.0 + 2.0 ** (-2 * i))
    return k


def atan_table(n: int = N_STAGES) -> list[int]:
    """atan(2**-i) expressed in the Z format, 2**(W_Z-1) counts to the half turn."""
    full = 1 << (W_Z - 1)
    return [round(math.atan(2.0 ** -i) / math.pi * full) for i in range(n)]


K_CORDIC = cordic_gain()
K_INV = 1.0 / K_CORDIC

# 1/K in Q0.15.  Every CORDIC instance compensates its own gain, so each one
# presents a plain Q2.13 interface and the gain never appears in the system
# equations.  The alternative, carrying K through the voltage path and undoing
# K**2 once at the end, saves one multiplier and is noted in
# docs/10-limitations-and-roadmap.md.
Q_KINV = 17
KINV_Q = round(K_INV * (1 << Q_KINV))

# 1/sqrt(3) and sqrt(3)/2 in Q0.15, used by the Clarke and inverse Clarke maps.
Q_TRIG = 15
INV_SQRT3_Q = round((1.0 / math.sqrt(3.0)) * (1 << Q_TRIG))
SQRT3_2_Q = round((math.sqrt(3.0) / 2.0) * (1 << Q_TRIG))

# ---------------------------------------------------------------------------
# Reference machine and drive.  See docs/01-system-and-requirements.md.
# A 48 V surface-mount PMSM of the size used in an AGV wheel drive or a small
# collaborative-robot joint.  These numbers exist so that every claim in the
# repository is checkable against one concrete machine; the RTL itself carries
# no machine constants, they all arrive through registers.
# ---------------------------------------------------------------------------

POLE_PAIRS = 4
R_S = 0.5             # ohm, per phase, line to neutral
L_D = 2.0e-3          # henry
L_Q = 2.0e-3          # henry, surface magnets so L_d = L_q
PSI_F = 0.02          # weber, rotor flux linkage
V_DC = 48.0           # volt
I_BASE = 10.0         # ampere, rated peak phase current
V_BASE = V_DC / math.sqrt(3.0)   # volt, peak phase voltage at the linear limit

F_CLK = 100.0e6       # hertz
F_PWM = 20.0e3        # hertz
PWM_TOP = round(F_CLK / (2.0 * F_PWM))          # 2500, up-down counter top
T_S = 1.0 / F_PWM     # second, one control update per PWM period
DEADTIME_NS = 1000.0
DEADTIME_CYC = round(DEADTIME_NS * 1e-9 * F_CLK)

ADC_BITS = 12
ADC_FULL_SCALE_PU = 2.0   # the ADC spans +/- 2 pu of phase current

# Per-unit machine constants: v_pu = R_pu * i_pu + L_pu * d(i_pu)/dt
R_PU = R_S * I_BASE / V_BASE
L_PU = L_D * I_BASE / V_BASE          # seconds
TAU_E = L_D / R_S                     # electrical time constant, seconds

# Current-loop bandwidth.  See docs/05-current-loop-tuning.md for the phase
# margin argument that fixes OMEGA_C * T_S at 0.1.
OMEGA_C_TS = 0.1
OMEGA_C = OMEGA_C_TS / T_S            # rad/s
KP = OMEGA_C * L_PU                   # per-unit proportional gain
KI_TS = KP * (R_S / L_D) * T_S        # per-unit integral gain per sample

KP_Q = round(KP * (1 << Q_KP))
KI_Q = round(KI_TS * (1 << Q_KI))

V_MAX_PU = 1.0        # the space-vector linear limit is exactly 1 pu by choice
                      # of V_BASE = V_DC / sqrt(3); see docs/06-svpwm-and-pwm.md
V_MAX_Q = round(V_MAX_PU * (1 << Q_DATA))
