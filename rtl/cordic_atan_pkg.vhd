-- -----------------------------------------------------------------------------
-- cordic_atan_pkg  -- GENERATED FILE, DO NOT EDIT
--
-- Regenerate with:  python scripts/gen_tables.py
--
-- Entry i is atan(2**-i) expressed in the CORDIC angle format, in which
-- 2**21 counts make a half turn, so one count is pi/2**21 = 1.4980e-06 rad.
--
-- Convergence.  A CORDIC step can only ever subtract the angle it is given, so
-- the algorithm resolves an angle theta if and only if every partial remainder
-- can still be reached by the steps that are left.  The sufficient condition is
--
--     atan(2**-i) <= sum_{j>i} atan(2**-j) + atan(2**-(n-1))
--
-- which the binary angle set satisfies for every i and the total reach is
--
--     sum_{i=0}^{17} atan(2**-i) = 1163715 counts = 1.743278 rad
--
-- against the pi/2 = 1048576 counts that survive the quadrant fold in
-- rtl/cordic.vhd.  The margin is 0.172 rad, so convergence is not marginal.
--
-- Residual.  After n steps the unresolved angle is at most atan(2**-(n-1)),
-- here atan(2**-17) = 7.629e-06 rad, which is 0.080 of one LSB of
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
     0 => to_signed( 524288, C_W_Z),   -- atan(2**-0 ) = 0.785398163 rad
     1 => to_signed( 309505, C_W_Z),   -- atan(2**-1 ) = 0.463647609 rad
     2 => to_signed( 163534, C_W_Z),   -- atan(2**-2 ) = 0.244978663 rad
     3 => to_signed(  83012, C_W_Z),   -- atan(2**-3 ) = 0.124354995 rad
     4 => to_signed(  41667, C_W_Z),   -- atan(2**-4 ) = 0.062418810 rad
     5 => to_signed(  20854, C_W_Z),   -- atan(2**-5 ) = 0.031239833 rad
     6 => to_signed(  10430, C_W_Z),   -- atan(2**-6 ) = 0.015623729 rad
     7 => to_signed(   5215, C_W_Z),   -- atan(2**-7 ) = 0.007812341 rad
     8 => to_signed(   2608, C_W_Z),   -- atan(2**-8 ) = 0.003906230 rad
     9 => to_signed(   1304, C_W_Z),   -- atan(2**-9 ) = 0.001953123 rad
    10 => to_signed(    652, C_W_Z),   -- atan(2**-10) = 0.000976562 rad
    11 => to_signed(    326, C_W_Z),   -- atan(2**-11) = 0.000488281 rad
    12 => to_signed(    163, C_W_Z),   -- atan(2**-12) = 0.000244141 rad
    13 => to_signed(     81, C_W_Z),   -- atan(2**-13) = 0.000122070 rad
    14 => to_signed(     41, C_W_Z),   -- atan(2**-14) = 0.000061035 rad
    15 => to_signed(     20, C_W_Z),   -- atan(2**-15) = 0.000030518 rad
    16 => to_signed(     10, C_W_Z),   -- atan(2**-16) = 0.000015259 rad
    17 => to_signed(      5, C_W_Z)   -- atan(2**-17) = 0.000007629 rad
  );

end package cordic_atan_pkg;
