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

  constant C_POLE_PAIRS : natural := 4;
  constant C_R_PU       : real    := 1.804219591218e-01;   -- per unit
  constant C_L_PU       : real    := 7.216878364870e-04;   -- seconds
  constant C_PSI_PU     : real    := 7.216878364870e-04;   -- per unit per rad/s
  constant C_TS         : real    := 5.000000000000e-05;   -- seconds
  constant C_SQRT3      : real    := 1.732050807569e+00;
  constant C_ADC_LSB    : real    := 9.765625000000e-04;   -- per unit
  constant C_ADC_FS     : real    := 2.000000000000e+00;   -- per unit
  constant C_Q_SCALE    : real    := 8192.0;             -- 2**Q_DATA

  constant C_PWM_TOP  : natural := 2500;
  constant C_DEADTIME : natural := 100;
  constant C_KP_Q     : integer := 5912;
  constant C_KI_Q     : integer := 296;
  constant C_VMAX_Q   : integer := 8192;

  -- The encoder: lines per revolution, counts per revolution after x4
  -- decoding and the QEI_SCALE register value that maps a count to angle.
  constant C_ENC_LINES : natural := 2500;
  constant C_QEI_CPR   : natural := 10000;
  constant C_QEI_SCALE : natural := 1717987;

  -- Predicted 10 to 90 per cent rise time of the closed current loop.  The
  -- continuous design gives the first figure; the exact sampled analysis of the
  -- same gains, including the one-sample computation delay, gives the second,
  -- and that is the one the testbench is allowed to check against.
  constant C_RISE_CONT : real := 1.098612288668e-03;   -- seconds
  constant C_RISE_DISC : real := 9.133415043774e-04;   -- seconds

end package machine_pkg;
