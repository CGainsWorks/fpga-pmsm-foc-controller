-- -----------------------------------------------------------------------------
-- synth_probe  -- TEMPORARY, not part of the design
--
-- Round five. The cause is now pinned to one function.
--
--   round one, each helper on a 24-bit signal      all three elaborate
--   round two, the clock enable                    both forms elaborate
--   round three, clarke by reduction               only removing the helpers helps
--   round four, each helper wide and on expressions
--       round_shr on a 42-bit product              ok
--       round_shr on a 34-bit signal               ok
--       overflows on either                        ok
--       sat_to on a 34-bit signal                  CRASH
--       sat_to on a 42-bit product                 CRASH
--
-- So sat_to is the only one at fault and width is what provokes it: 24 bits in
-- and 16 out elaborates, 34 bits in and 16 out does not. round_shr and
-- overflows build wide constants the same way and are fine, so it is specific
-- to narrowing the return value from a wide operand.
--
-- Three candidate reformulations, all at 34 bits, all returning the same
-- function of the input. Whichever elaborates goes into rtl/foc_pkg.vhd.
--
-- Delete this file and the synth-probe target once the answer is known.
-- -----------------------------------------------------------------------------

library ieee;
  use ieee.std_logic_1164.all;
  use ieee.numeric_std.all;

-- A. limits built as integer literals rather than by shifting a wide one.
--    Minimal change from the current body.
entity probe_s_literal is
  port (
    clk : in    std_logic;
    d   : in    signed(33 downto 0);
    q   : out   signed(15 downto 0)
  );
end entity probe_s_literal;

architecture rtl of probe_s_literal is

  function sat_a (v : signed; w : natural) return signed is
  begin
    if v'length <= w then
      return resize(v, w);
    end if;
    if v > to_signed(2 ** (w - 1) - 1, v'length) then
      return to_signed(2 ** (w - 1) - 1, w);
    elsif v < to_signed(-(2 ** (w - 1)), v'length) then
      return to_signed(-(2 ** (w - 1)), w);
    else
      return resize(v, w);
    end if;
  end function sat_a;

  signal r : signed(15 downto 0) := (others => '0');

begin

  p : process (clk) is
  begin

    if rising_edge(clk) then
      r <= sat_a(d, 16);
    end if;

  end process p;

  q <= r;

end architecture rtl;

library ieee;
  use ieee.std_logic_1164.all;
  use ieee.numeric_std.all;

-- B. limits built as aggregates at the narrow width, widened by resize for the
--    comparison.
entity probe_s_aggregate is
  port (
    clk : in    std_logic;
    d   : in    signed(33 downto 0);
    q   : out   signed(15 downto 0)
  );
end entity probe_s_aggregate;

architecture rtl of probe_s_aggregate is

  function sat_b (v : signed; w : natural) return signed is
    constant C_HI : signed(w - 1 downto 0) := (w - 1 => '0', others => '1');
    constant C_LO : signed(w - 1 downto 0) := (w - 1 => '1', others => '0');
  begin
    if v'length <= w then
      return resize(v, w);
    end if;
    if v > resize(C_HI, v'length) then
      return C_HI;
    elsif v < resize(C_LO, v'length) then
      return C_LO;
    else
      return resize(v, w);
    end if;
  end function sat_b;

  signal r : signed(15 downto 0) := (others => '0');

begin

  p : process (clk) is
  begin

    if rising_edge(clk) then
      r <= sat_b(d, 16);
    end if;

  end process p;

  q <= r;

end architecture rtl;

library ieee;
  use ieee.std_logic_1164.all;
  use ieee.numeric_std.all;

-- C. the function no longer narrows at all: it clamps at the input width and
--    the caller narrows with resize, which is lossless because the value is
--    already inside the range.
entity probe_s_clampwide is
  port (
    clk : in    std_logic;
    d   : in    signed(33 downto 0);
    q   : out   signed(15 downto 0)
  );
end entity probe_s_clampwide;

architecture rtl of probe_s_clampwide is

  function clamp_c (v : signed; w : natural) return signed is
    constant C_HI : signed(w - 1 downto 0) := (w - 1 => '0', others => '1');
    constant C_LO : signed(w - 1 downto 0) := (w - 1 => '1', others => '0');
  begin
    if v'length <= w then
      return v;
    end if;
    if v > resize(C_HI, v'length) then
      return resize(C_HI, v'length);
    elsif v < resize(C_LO, v'length) then
      return resize(C_LO, v'length);
    else
      return v;
    end if;
  end function clamp_c;

  signal r : signed(15 downto 0) := (others => '0');

begin

  p : process (clk) is
  begin

    if rising_edge(clk) then
      r <= resize(clamp_c(d, 16), 16);
    end if;

  end process p;

  q <= r;

end architecture rtl;

library ieee;
  use ieee.std_logic_1164.all;
  use ieee.numeric_std.all;

library work;
  use work.foc_pkg.all;

-- Control: the current sat_to at the same width, expected to crash.
entity probe_s_current is
  port (
    clk : in    std_logic;
    d   : in    signed(33 downto 0);
    q   : out   signed(15 downto 0)
  );
end entity probe_s_current;

architecture rtl of probe_s_current is

  signal r : signed(15 downto 0) := (others => '0');

begin

  p : process (clk) is
  begin

    if rising_edge(clk) then
      r <= sat_to(d, 16);
    end if;

  end process p;

  q <= r;

end architecture rtl;
