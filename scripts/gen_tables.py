#!/usr/bin/env python3
"""Emit rtl/cordic_atan_pkg.vhd from the same constants the models use.

The table could be built inside VHDL with ieee.math_real at elaboration time.
It is generated here instead for two reasons: synthesis support for real
arithmetic in constant functions is uneven across vendors and generating it
makes the table reviewable in the diff of any change to the word lengths.  CI
regenerates the file and fails if it differs from the committed one, so the two
cannot drift apart.
"""

from __future__ import annotations

import argparse
import math
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))

from focmodel import params as P
from focmodel.analysis import rise_time_continuous, rise_time_discrete
from focmodel.params import N_STAGES, W_Z, atan_table

HEADER = """\
-- -----------------------------------------------------------------------------
-- cordic_atan_pkg  -- GENERATED FILE, DO NOT EDIT
--
-- Regenerate with:  python scripts/gen_tables.py
--
-- Entry i is atan(2**-i) expressed in the CORDIC angle format, in which
-- 2**{half} counts make a half turn, so one count is pi/2**{half} = {res:.4e} rad.
--
-- Convergence.  A CORDIC step can only ever subtract the angle it is given, so
-- the algorithm resolves an angle theta if and only if every partial remainder
-- can still be reached by the steps that are left.  The sufficient condition is
--
--     atan(2**-i) <= sum_{{j>i}} atan(2**-j) + atan(2**-(n-1))
--
-- which the binary angle set satisfies for every i and the total reach is
--
--     sum_{{i=0}}^{{{last}}} atan(2**-i) = {total} counts = {total_rad:.6f} rad
--
-- against the pi/2 = {half_pi} counts that survive the quadrant fold in
-- rtl/cordic.vhd.  The margin is {margin:.3f} rad, so convergence is not marginal.
--
-- Residual.  After n steps the unresolved angle is at most atan(2**-(n-1)),
-- here atan(2**-{nm1}) = {residual:.3e} rad, which is {residual_lsb:.3f} of one LSB of
-- the 16-bit input angle.  The arithmetic therefore contributes less angle error
-- than the interface it is given.
-- -----------------------------------------------------------------------------

library ieee;
  use ieee.std_logic_1164.all;
  use ieee.numeric_std.all;

library work;
  use work.foc_pkg.all;

package cordic_atan_pkg is

  constant C_ATAN : atan_table_t(0 to C_N_STAGES - 1) := (
"""

FOOTER = """  );

end package cordic_atan_pkg;
"""


def render() -> str:
    tab = atan_table()
    half = W_Z - 1
    total = sum(tab)
    total_rad = total / float(1 << half) * math.pi
    half_pi = 1 << (W_Z - 2)
    body = HEADER.format(
        half=half,
        res=math.pi / (1 << half),
        last=N_STAGES - 1,
        total=total,
        total_rad=total_rad,
        half_pi=half_pi,
        margin=total_rad - math.pi / 2.0,
        nm1=N_STAGES - 1,
        residual=math.atan(2.0 ** -(N_STAGES - 1)),
        residual_lsb=math.atan(2.0 ** -(N_STAGES - 1)) / (2.0 * math.pi / 65536.0),
    )
    lines = []
    for i, v in enumerate(tab):
        comma = "," if i < len(tab) - 1 else ""
        rad = math.atan(2.0 ** -i)
        lines.append(
            f"    {i:2d} => to_signed({v:7d}, C_W_Z){comma}"
            f"   -- atan(2**-{i:<2d}) = {rad:.9f} rad"
        )
    return body + "\n".join(lines) + "\n" + FOOTER


MACHINE = """
-- -----------------------------------------------------------------------------
-- machine_pkg  -- GENERATED FILE, DO NOT EDIT
--
-- Regenerate with:  python scripts/gen_tables.py
--
-- The reference machine, its per-unit constants and the tuning derived from
-- them, for the closed-loop testbench.  These are generated rather than typed so
-- that the machine the testbench simulates and the machine focmodel analyses are
-- the same machine.  A hand-copied constant here would let the simulation agree
-- with a prediction neither of them was about.
--
-- Per unit: v_pu = R_pu i_pu + L_pu d(i_pu)/dt, with the current base the rated
-- peak phase current and the voltage base V_dc/sqrt(3), which is the space-vector
-- linear limit and therefore exactly 1.0 per unit.
-- -----------------------------------------------------------------------------

package machine_pkg is

  constant C_POLE_PAIRS : natural := {pole_pairs};
  constant C_R_PU       : real    := {r_pu:.12e};   -- per unit
  constant C_L_PU       : real    := {l_pu:.12e};   -- seconds
  constant C_PSI_PU     : real    := {psi_pu:.12e};   -- per unit per rad/s
  constant C_TS         : real    := {ts:.12e};   -- seconds
  constant C_SQRT3      : real    := 1.732050807569e+00;
  constant C_ADC_LSB    : real    := {adc_lsb:.12e};   -- per unit
  constant C_ADC_FS     : real    := {adc_fs:.12e};   -- per unit
  constant C_Q_SCALE    : real    := {q_scale:.1f};             -- 2**Q_DATA

  constant C_PWM_TOP  : natural := {pwm_top};
  constant C_DEADTIME : natural := {deadtime};
  constant C_KP_Q     : integer := {kp_q};
  constant C_KI_Q     : integer := {ki_q};
  constant C_VMAX_Q   : integer := {vmax_q};

  -- The encoder: lines per revolution, counts per revolution after x4
  -- decoding and the QEI_SCALE register value that maps a count to angle.
  constant C_ENC_LINES : natural := {enc_lines};
  constant C_QEI_CPR   : natural := {qei_cpr};
  constant C_QEI_SCALE : natural := {qei_scale};

  -- Predicted 10 to 90 per cent rise time of the closed current loop.  The
  -- continuous design gives the first figure; the exact sampled analysis of the
  -- same gains, including the one-sample computation delay, gives the second,
  -- and that is the one the testbench is allowed to check against.
  constant C_RISE_CONT : real := {rise_cont:.12e};   -- seconds
  constant C_RISE_DISC : real := {rise_disc:.12e};   -- seconds

end package machine_pkg;
"""


def render_machine() -> str:
    return MACHINE.lstrip(chr(10)).format(
        pole_pairs=P.POLE_PAIRS,
        r_pu=P.R_PU,
        l_pu=P.L_PU,
        psi_pu=P.PSI_F / P.V_BASE,
        ts=P.T_S,
        adc_lsb=2.0 * P.ADC_FULL_SCALE_PU / (1 << P.ADC_BITS),
        adc_fs=P.ADC_FULL_SCALE_PU,
        q_scale=float(1 << P.Q_DATA),
        pwm_top=P.PWM_TOP,
        deadtime=P.DEADTIME_CYC,
        kp_q=P.KP_Q,
        ki_q=P.KI_Q,
        vmax_q=P.V_MAX_Q,
        enc_lines=P.ENC_LINES,
        qei_cpr=P.QEI_CPR,
        qei_scale=P.QEI_SCALE,
        rise_cont=rise_time_continuous(),
        rise_disc=rise_time_discrete(1),
    )


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--check", action="store_true",
                    help="exit non-zero if a committed file is out of date")
    args = ap.parse_args()

    targets = [
        (pathlib.Path("rtl/cordic_atan_pkg.vhd"), render()),
        (pathlib.Path("tb/machine_pkg.vhd"), render_machine()),
    ]

    rc = 0
    for out, text in targets:
        if args.check:
            if not out.exists():
                print(f"{out} is missing", file=sys.stderr)
                rc = 1
            elif out.read_text(encoding="utf-8").replace(
                    chr(13) + chr(10), chr(10)) != text:
                print(f"{out} is out of date, run: python scripts/gen_tables.py",
                      file=sys.stderr)
                rc = 1
            else:
                print(f"{out} is up to date")
        else:
            out.write_text(text, encoding="utf-8", newline=chr(10))
            print(f"wrote {out} ({len(text)} bytes)")
    return rc


if __name__ == "__main__":
    raise SystemExit(main())
