-- -----------------------------------------------------------------------------
-- clarke
--
-- Amplitude-invariant three-phase to two-phase transform, derived in
-- docs/04-clarke-park.md.  The general map is
--
--     [i_alpha]       [1  -1/2   -1/2  ] [i_a]
--     [       ] = 2/3 [                 ] [i_b]
--     [i_beta ]       [0  s3/2   -s3/2 ] [i_c]
--
-- with the 2/3 chosen so that a balanced set of peak amplitude A produces a
-- space vector of magnitude A rather than 1.5 A, which is what makes every
-- per-unit limit in the design mean what it says.
--
-- With an isolated star point i_a + i_b + i_c = 0 by Kirchhoff, so substituting
-- i_c = -i_a - i_b collapses the map to
--
--     i_alpha = i_a
--     i_beta  = (i_a + 2 i_b) / sqrt(3)
--
-- One multiplier, one adder and a whole current sensor, shunt, amplifier and
-- ADC channel removed from the bill of materials of every drive built on this.
-- The saving is real money per unit shipped, which is the sort of thing that
-- decides whether a control block is used.
--
-- Latency one clock.
-- -----------------------------------------------------------------------------

library ieee;
  use ieee.std_logic_1164.all;
  use ieee.numeric_std.all;

library work;
  use work.foc_pkg.all;

entity clarke is
  port (
    clk       : in    std_logic;
    rst       : in    std_logic;
    in_valid  : in    std_logic;
    ia        : in    signed(C_W_DATA - 1 downto 0); -- Q2.13
    ib        : in    signed(C_W_DATA - 1 downto 0); -- Q2.13
    out_valid : out   std_logic;
    ialpha    : out   signed(C_W_DATA - 1 downto 0); -- Q2.13
    ibeta     : out   signed(C_W_DATA - 1 downto 0); -- Q2.13
    sat_out   : out   std_logic
  );
end entity clarke;

architecture rtl of clarke is

  -- i_a + 2 i_b reaches three times full scale, so the sum is formed two bits
  -- wider than the inputs and only requantised after the multiply.  Rounding
  -- once instead of twice is worth half an LSB and costs nothing.
  constant C_W_SUM  : natural := C_W_DATA + 2;
  constant C_W_PROD : natural := C_W_SUM + 16;

  -- Power-up values on the output register, for the reason given in
  -- rtl/cordic.vhd: the stage is clock-enabled and nothing drives it until the
  -- first sample arrives.
  signal ial_r : signed(C_W_DATA - 1 downto 0) := (others => '0');
  signal ibe_r : signed(C_W_DATA - 1 downto 0) := (others => '0');
  signal sat_r : std_logic                     := '0';

  signal vld : std_logic;

begin

  p_clarke : process (clk) is

    variable t : signed(C_W_SUM - 1 downto 0);
    variable p : signed(C_W_PROD - 1 downto 0);

  begin

    if rising_edge(clk) and in_valid = '1' then
      t := resize(ia, C_W_SUM) + shift_left(resize(ib, C_W_SUM), 1);
      p := round_shr(t * to_signed(C_INV_SQRT3, 16), C_Q_TRIG);

      ial_r <= ia;
      ibe_r <= sat_to(p, C_W_DATA);

      if overflows(p, C_W_DATA) then
        sat_r <= '1';
      else
        sat_r <= '0';
      end if;
    end if;

  end process p_clarke;

  p_valid : process (clk) is
  begin

    if rising_edge(clk) then
      if rst = '1' then
        vld <= '0';
      else
        vld <= in_valid;
      end if;
    end if;

  end process p_valid;

  ialpha    <= ial_r;
  ibeta     <= ibe_r;
  sat_out   <= sat_r;
  out_valid <= vld;

end architecture rtl;
