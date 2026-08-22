#!/usr/bin/env python3
"""Recompute every number the README quotes and check the text still says it.

A repository that documents its own derivations acquires a specific failure mode:
the derivation moves and the prose does not.  Every figure in README.md is
produced here from focmodel and looked up in the text, so a word length or a gain
that changes without the documentation following it fails CI rather than quietly
leaving the README wrong.

Numbers that come out of simulation rather than out of the models are not checked
here; the testbenches print those on every run.
"""

from __future__ import annotations

import math
import pathlib
import random
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))

from focmodel import params as P
from focmodel.analysis import (
    plant_pole,
    quantised_gains,
    rise_time_continuous,
    rise_time_discrete,
    summary,
)
from focmodel.cordic import ROTATE, cordic

ROOT = pathlib.Path(__file__).resolve().parents[1]
HALF_TURN = 1 << (P.W_ANG - 1)
ONE = 1 << P.Q_DATA


def worst_rotation_error() -> float:
    """Worst error against real arithmetic over the committed vector set.

    Reproduces the first block of scripts/gen_vectors.py exactly: same seed, same
    cases, same order, so the figure is the one the testbench will report.
    """
    rng = random.Random(20260822)
    cases = [(0, 0, 0), (ONE, 0, 0), (ONE, 0, HALF_TURN // 2),
             (ONE, 0, -HALF_TURN // 2), (ONE, 0, HALF_TURN - 1),
             (ONE, 0, -HALF_TURN), (0, ONE, HALF_TURN // 4),
             (2 * ONE, -2 * ONE, HALF_TURN - 1),
             (-2 * ONE, 2 * ONE, -HALF_TURN)]
    for _ in range(2000 - len(cases)):
        cases.append((rng.randint(-2 * ONE, 2 * ONE),
                      rng.randint(-2 * ONE, 2 * ONE),
                      rng.randint(-HALF_TURN, HALF_TURN - 1)))
    worst = 0.0
    for x, y, a in cases:
        xo, yo, _r, _s = cordic(ROTATE, x, y, a)
        ang = a * math.pi / HALF_TURN
        rx = x * math.cos(ang) - y * math.sin(ang)
        ry = x * math.sin(ang) + y * math.cos(ang)
        worst = max(worst, abs(xo - rx), abs(yo - ry))
    return worst


def sci(x: float, digits: int = 2) -> str:
    """Scientific notation the way the prose writes it, without the padded zero."""
    return f"{x:.{digits}e}".replace("e-0", "e-").replace("e+0", "e")


def cordic_error_bound() -> float:
    """The four-term worst case from docs/03, over the specified input range.

    The specification is each input component within +/- 2 per unit, so the
    vector magnitude is at most 2 sqrt(2) = 2.83 and not the 5.66 the format
    could hold.  Quoting the bound over the format rather than over the
    specification would inflate it by a factor of two for no reason.
    """
    mag = math.sqrt(2.0) * 2.0
    residual = math.atan(2.0 ** -(P.N_STAGES - 1)) * mag
    table = P.N_STAGES * 0.5 * math.pi / (1 << (P.W_Z - 1)) * mag
    trunc = P.N_STAGES * math.sqrt(2.0) * 2.0 ** -P.Q_XY * P.K_CORDIC
    lsb = 2.0 ** -P.Q_DATA
    return (residual + table + trunc) / lsb + 0.5


def build_claims() -> list[tuple[str, str]]:
    s = summary()
    kp, ki = quantised_gains()
    a = (kp + ki) * (1.0 - plant_pole()) / P.R_PU
    adc_lsb = 2.0 * P.ADC_FULL_SCALE_PU / (1 << P.ADC_BITS)
    latency = 66
    lat_s = latency / P.F_CLK
    omega_e_500 = 500.0 / 60.0 * 2.0 * math.pi * P.POLE_PAIRS
    base_rpm = P.V_BASE / P.PSI_F / P.POLE_PAIRS * 60.0 / (2.0 * math.pi)
    max_in = (2 ** (P.W_DATA - 1) - 1) / float(1 << P.Q_DATA)

    return [
        ("ADC LSB", sci(adc_lsb, 3)),
        ("CORDIC gain", f"{P.K_CORDIC:.4f}"),
        ("worst internal magnitude", f"{max_in * math.sqrt(2.0) * P.K_CORDIC:.2f}"),
        ("internal range", str(1 << (P.W_XY - 1 - P.Q_XY))),
        ("residual angle", sci(math.atan(2.0 ** -(P.N_STAGES - 1)))),
        ("iteration reach",
         f"{sum(math.atan(2.0 ** -i) for i in range(P.N_STAGES)):.4f}"),
        ("fold margin",
         f"{sum(math.atan(2.0 ** -i) for i in range(P.N_STAGES)) - math.pi / 2:.2f}"),
        ("worst measured rotation error", f"{worst_rotation_error():.2f}"),
        ("derived rotation bound", f"{cordic_error_bound():.2f}"),
        ("phase margin", f"{s['phase_margin_deg']:.1f}"),
        ("gain margin", f"{s['gain_margin_db']:.1f}"),
        ("continuous rise", f"{rise_time_continuous() * 1e3:.3f}"),
        ("discrete rise", f"{rise_time_discrete(1) * 1e3:.3f}"),
        ("loop gain at the design point", f"{a:.1f}"),
        ("space-vector headroom", f"{(2.0 / math.sqrt(3.0) - 1.0) * 100:.2f}"),
        ("sinusoidal limit", f"{math.sqrt(3.0) / 2.0:.3f}"),
        ("datapath latency", f"{latency} clocks"),
        ("datapath latency in us", f"{lat_s * 1e6:.2f} us"),
        ("datapath phase lag",
         f"{math.degrees(P.OMEGA_C * lat_s):.3f} degrees"),
        ("bandwidth in hertz", f"{P.OMEGA_C / (2.0 * math.pi):.0f} Hz"),
        ("sample-to-apply delay", f"{1.5 * P.T_S * 1e6:.0f} us"),
        ("sample-to-apply phase lag",
         f"{math.degrees(P.OMEGA_C * 1.5 * P.T_S):.1f} degrees"),
        ("back-EMF at 500 rpm",
         f"{omega_e_500 * P.PSI_F / P.V_BASE:.3f}"),
        ("cross-coupling at 500 rpm",
         f"{omega_e_500 * P.L_PU * 0.3:.3f}"),
        ("bandwidth", f"{P.OMEGA_C:.0f} rad/s"),
        ("electrical time constant", f"{P.TAU_E * 1e3:.1f} ms"),
        ("base speed", f"{base_rpm:.0f} rpm"),
    ]


def main() -> int:
    readme = (ROOT / "README.md").read_text(encoding="utf-8")
    claims = build_claims()
    width = max(len(name) for name, _ in claims)
    missing = []
    for name, text in claims:
        ok = text in readme
        print(f"  {'ok ' if ok else 'MISS'}  {name:<{width}}  {text}")
        if not ok:
            missing.append((name, text))
    if missing:
        print("\nREADME.md does not contain:", file=sys.stderr)
        for name, text in missing:
            print(f"  {name}: {text}", file=sys.stderr)
        return 1
    print(f"\nall {len(claims)} figures in README.md come from the models")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
