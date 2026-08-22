-- -----------------------------------------------------------------------------
-- foc_pkg
--
-- Word lengths, fixed-point formats and the rounding and saturation primitives
-- every other entity in this design is built from.  Each constant is derived in
-- docs/02-fixed-point.md and mirrored in focmodel/params.py; tests/test_rtl_
-- constants.py fails if the two ever disagree, so this file cannot drift away
-- from the models that generate the testbench vectors.
--
-- Formats
--   data   Q2.13 in 16 bits, range [-4, +4), resolution 1.22e-4 per unit
--   angle  binary angular measure, 16 bits, 2**16 counts to the electrical turn
--   cordic Q4.19 in 24 bits internally, 22-bit angle accumulator
--
-- The per-unit bases are the rated peak phase current and V_dc/sqrt(3), the
-- latter chosen so that the space-vector linear modulation limit lands on
-- exactly 1.0 per unit.
-- -----------------------------------------------------------------------------

library ieee;
  use ieee.std_logic_1164.all;
  use ieee.numeric_std.all;

package foc_pkg is

  -- Datapath word lengths -----------------------------------------------------
  constant C_W_DATA   : natural := 16;
  constant C_Q_DATA   : natural := 13;
  constant C_W_ANG    : natural := 16;
  constant C_W_XY     : natural := 24;
  constant C_Q_XY     : natural := 19;
  constant C_W_Z      : natural := 22;
  constant C_N_STAGES : natural := 18;

  -- Two growth bits above the input format hold the CORDIC gain, six guard
  -- fraction bits keep the accumulated truncation below a quarter of an output
  -- LSB.  16 + 6 + 2 = 24, which is also the widest operand a Xilinx DSP48 slice
  -- takes on its B port, so the gain-compensation multiply stays in one slice.
  constant C_GUARD   : natural := C_Q_XY - C_Q_DATA;
  constant C_Z_SHIFT : natural := C_W_Z - C_W_ANG;

  -- Constant multipliers ------------------------------------------------------
  constant C_Q_KINV    : natural := 17;
  constant C_KINV      : integer := 79594;      -- 1/K in Q0.17, K = 1.64676026
  constant C_W_KINV    : natural := 18;
  constant C_Q_TRIG    : natural := 15;
  constant C_INV_SQRT3 : integer := 18919;      -- 1/sqrt(3) in Q0.15
  constant C_SQRT3_2   : integer := 28378;      -- sqrt(3)/2 in Q0.15

  -- Controller gain formats ---------------------------------------------------
  constant C_Q_KP  : natural := 12;              -- Kp in Q3.12, range [-8, +8)
  constant C_Q_KI  : natural := 14;              -- Ki in Q1.14, range [-2, +2)
  constant C_W_ACC : natural := 32;              -- integrator, Q?.27

  subtype data_t is signed(C_W_DATA - 1 downto 0);

  subtype angle_t is signed(C_W_ANG - 1 downto 0);

  type cordic_mode_t is (rotate_mode, vector_mode);

  type atan_table_t is array (natural range <>) of signed(C_W_Z - 1 downto 0);

  -- Arithmetic primitives -----------------------------------------------------

  -- Arithmetic right shift with round-half-up.  One adder; error in
  -- (-0.5, +0.5] LSB instead of the [-1, 0] LSB of plain truncation, and
  -- unbiased, which matters wherever the result reaches an integrator.

  function round_shr (v : signed; s : natural) return signed;

  -- Clamp into w bits.  Used everywhere a result leaves a wide internal format,
  -- because wrapping would turn an overcurrent into a sign reversal that the
  -- controller would then act on as if it were a measurement.

  function sat_to (v : signed; w : natural) return signed;

  -- True when sat_to would have clipped.  Drives the status bits.

  function overflows (v : signed; w : natural) return boolean;

  -- Reduce modulo 2**w.  Correct, not lossy, for angles: the binary angular
  -- measure is a residue system modulo one turn, so wrapping is the arithmetic
  -- we want rather than an overflow to be guarded against.

  function wrap_to (v : signed; w : natural) return signed;

end package foc_pkg;

package body foc_pkg is

  function round_shr (v : signed; s : natural) return signed is

    variable t : signed(v'length downto 0);

  begin

    if s = 0 then
      return v;
    end if;

    t := resize(v, v'length + 1) + shift_left(to_signed(1, v'length + 1), s - 1);
    return resize(shift_right(t, s), v'length);

  end function round_shr;

  function sat_to (v : signed; w : natural) return signed is

    variable hi : signed(v'length - 1 downto 0);
    variable lo : signed(v'length - 1 downto 0);

  begin

    if v'length <= w then
      return resize(v, w);
    end if;

    hi := shift_left(to_signed(1, v'length), w - 1) - 1;
    lo := -shift_left(to_signed(1, v'length), w - 1);

    if v > hi then
      return resize(hi, w);
    elsif v < lo then
      return resize(lo, w);
    else
      return resize(v, w);
    end if;

  end function sat_to;

  function overflows (v : signed; w : natural) return boolean is

    variable hi : signed(v'length - 1 downto 0);
    variable lo : signed(v'length - 1 downto 0);

  begin

    if v'length <= w then
      return false;
    end if;

    hi := shift_left(to_signed(1, v'length), w - 1) - 1;
    lo := -shift_left(to_signed(1, v'length), w - 1);
    return v > hi or v < lo;

  end function overflows;

  function wrap_to (v : signed; w : natural) return signed is

    variable a : signed(v'length - 1 downto 0);

  begin

    a := v;
    return a(w - 1 downto 0);

  end function wrap_to;

end package body foc_pkg;
