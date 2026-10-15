-- -----------------------------------------------------------------------------
-- synth_probe  -- TEMPORARY, not part of the design
--
-- Round four. What is known so far:
--
--   round one, each construct alone      all ten elaborate
--   round two, the clock enable          both forms elaborate
--   round three, clarke by reduction     the copy crashes; removing the clock
--                                        enable, the process variables or the
--                                        signal initialisers does not help;
--                                        replacing the three arithmetic helpers
--                                        with the same arithmetic inline does
--
-- So a helper is the cause, but not on its own: round one called each of them on
-- a 24-bit port signal and all three elaborated. In clarke they are called on a
-- 34-bit variable and on a composed expression instead.
--
-- This round separates the two axes: which helper, and whether the argument
-- being an expression rather than a signal is what matters.
--
-- Delete this file and the synth-probe target once the answer is known.
-- -----------------------------------------------------------------------------

library ieee;
  use ieee.std_logic_1164.all;
  use ieee.numeric_std.all;

library work;
  use work.foc_pkg.all;

-- 1. round_shr applied to a product rather than to a signal.
entity probe_h_round_expr is
  port (
    clk : in    std_logic;
    d   : in    signed(23 downto 0);
    e   : in    signed(17 downto 0);
    q   : out   signed(41 downto 0)
  );
end entity probe_h_round_expr;

architecture rtl of probe_h_round_expr is

  signal r : signed(41 downto 0) := (others => '0');

begin

  p : process (clk) is
  begin

    if rising_edge(clk) then
      r <= round_shr(d * e, 15);
    end if;

  end process p;

  q <= r;

end architecture rtl;

library ieee;
  use ieee.std_logic_1164.all;
  use ieee.numeric_std.all;

library work;
  use work.foc_pkg.all;

-- 2. sat_to applied to a product.
entity probe_h_sat_expr is
  port (
    clk : in    std_logic;
    d   : in    signed(23 downto 0);
    e   : in    signed(17 downto 0);
    q   : out   signed(15 downto 0)
  );
end entity probe_h_sat_expr;

architecture rtl of probe_h_sat_expr is

  signal r : signed(15 downto 0) := (others => '0');

begin

  p : process (clk) is
  begin

    if rising_edge(clk) then
      r <= sat_to(d * e, 16);
    end if;

  end process p;

  q <= r;

end architecture rtl;

library ieee;
  use ieee.std_logic_1164.all;
  use ieee.numeric_std.all;

library work;
  use work.foc_pkg.all;

-- 3. overflows applied to a product.
entity probe_h_over_expr is
  port (
    clk : in    std_logic;
    d   : in    signed(23 downto 0);
    e   : in    signed(17 downto 0);
    q   : out   std_logic
  );
end entity probe_h_over_expr;

architecture rtl of probe_h_over_expr is

  signal r : std_logic := '0';

begin

  p : process (clk) is
  begin

    if rising_edge(clk) then
      if overflows(d * e, 16) then
        r <= '1';
      else
        r <= '0';
      end if;
    end if;

  end process p;

  q <= r;

end architecture rtl;

library ieee;
  use ieee.std_logic_1164.all;
  use ieee.numeric_std.all;

library work;
  use work.foc_pkg.all;

-- 4. round_shr on a wide signal, to separate width from expression shape.
entity probe_h_round_wide is
  port (
    clk : in    std_logic;
    d   : in    signed(33 downto 0);
    q   : out   signed(33 downto 0)
  );
end entity probe_h_round_wide;

architecture rtl of probe_h_round_wide is

  signal r : signed(33 downto 0) := (others => '0');

begin

  p : process (clk) is
  begin

    if rising_edge(clk) then
      r <= round_shr(d, 15);
    end if;

  end process p;

  q <= r;

end architecture rtl;

library ieee;
  use ieee.std_logic_1164.all;
  use ieee.numeric_std.all;

library work;
  use work.foc_pkg.all;

-- 5. sat_to on a wide signal.
entity probe_h_sat_wide is
  port (
    clk : in    std_logic;
    d   : in    signed(33 downto 0);
    q   : out   signed(15 downto 0)
  );
end entity probe_h_sat_wide;

architecture rtl of probe_h_sat_wide is

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

library ieee;
  use ieee.std_logic_1164.all;
  use ieee.numeric_std.all;

library work;
  use work.foc_pkg.all;

-- 6. overflows on a wide signal.
entity probe_h_over_wide is
  port (
    clk : in    std_logic;
    d   : in    signed(33 downto 0);
    q   : out   std_logic
  );
end entity probe_h_over_wide;

architecture rtl of probe_h_over_wide is

  signal r : std_logic := '0';

begin

  p : process (clk) is
  begin

    if rising_edge(clk) then
      if overflows(d, 16) then
        r <= '1';
      else
        r <= '0';
      end if;
    end if;

  end process p;

  q <= r;

end architecture rtl;
