-- -----------------------------------------------------------------------------
-- synth_probe  -- TEMPORARY, not part of the design
--
-- One minimal entity per construct, to find which of them GHDL 5.0.1 cannot
-- elaborate through its synthesiser.
--
-- Two explanations fit the per-unit results equally well. Every unit that
-- crashes calls an arithmetic helper from rtl/foc_pkg.vhd, and every unit that
-- crashes also reaches ieee.numeric_std.shift_left or shift_right, because the
-- helpers use them internally. The two cannot be separated from the design
-- itself, so they are separated here instead: each probe exercises exactly one
-- construct and nothing else.
--
-- Delete this file and the synth-probe target once the answer is known.
-- -----------------------------------------------------------------------------

library ieee;
  use ieee.std_logic_1164.all;
  use ieee.numeric_std.all;

-- 1. resize: a built-in whose result width comes from a parameter value.
entity probe_resize is
  port (
    clk : in    std_logic;
    d   : in    signed(23 downto 0);
    q   : out   signed(15 downto 0)
  );
end entity probe_resize;

architecture rtl of probe_resize is

  signal r : signed(15 downto 0) := (others => '0');

begin

  p : process (clk) is
  begin

    if rising_edge(clk) then
      r <= resize(d, 16);
    end if;

  end process p;

  q <= r;

end architecture rtl;

library ieee;
  use ieee.std_logic_1164.all;
  use ieee.numeric_std.all;

-- 2. shift_right by a literal.
entity probe_shift_const is
  port (
    clk : in    std_logic;
    d   : in    signed(23 downto 0);
    q   : out   signed(23 downto 0)
  );
end entity probe_shift_const;

architecture rtl of probe_shift_const is

  signal r : signed(23 downto 0) := (others => '0');

begin

  p : process (clk) is
  begin

    if rising_edge(clk) then
      r <= shift_right(d, 3);
    end if;

  end process p;

  q <= r;

end architecture rtl;

library ieee;
  use ieee.std_logic_1164.all;
  use ieee.numeric_std.all;

-- 3. shift_left of a to_signed literal by a generic, which is what the rounding
--    term in round_shr does.
entity probe_shift_gen is
  generic (
    G_S : natural := 5
  );
  port (
    clk : in    std_logic;
    d   : in    signed(23 downto 0);
    q   : out   signed(23 downto 0)
  );
end entity probe_shift_gen;

architecture rtl of probe_shift_gen is

  signal r : signed(23 downto 0) := (others => '0');

begin

  p : process (clk) is
  begin

    if rising_edge(clk) then
      r <= d + shift_left(to_signed(1, 24), G_S - 1);
    end if;

  end process p;

  q <= r;

end architecture rtl;

library ieee;
  use ieee.std_logic_1164.all;
  use ieee.numeric_std.all;

-- 4. a signed multiply, with no helper and no shift.
entity probe_mul is
  port (
    clk : in    std_logic;
    d   : in    signed(23 downto 0);
    e   : in    signed(17 downto 0);
    q   : out   signed(41 downto 0)
  );
end entity probe_mul;

architecture rtl of probe_mul is

  signal r : signed(41 downto 0) := (others => '0');

begin

  p : process (clk) is
  begin

    if rising_edge(clk) then
      r <= d * e;
    end if;

  end process p;

  q <= r;

end architecture rtl;

library ieee;
  use ieee.std_logic_1164.all;
  use ieee.numeric_std.all;

library work;
  use work.foc_pkg.all;

-- 5. round_shr: a helper whose result width comes from an operand's length.
entity probe_round_shr is
  port (
    clk : in    std_logic;
    d   : in    signed(23 downto 0);
    q   : out   signed(23 downto 0)
  );
end entity probe_round_shr;

architecture rtl of probe_round_shr is

  signal r : signed(23 downto 0) := (others => '0');

begin

  p : process (clk) is
  begin

    if rising_edge(clk) then
      r <= round_shr(d, 3);
    end if;

  end process p;

  q <= r;

end architecture rtl;

library ieee;
  use ieee.std_logic_1164.all;
  use ieee.numeric_std.all;

library work;
  use work.foc_pkg.all;

-- 6. sat_to: a helper whose result width comes from a parameter value.
entity probe_sat_to is
  port (
    clk : in    std_logic;
    d   : in    signed(23 downto 0);
    q   : out   signed(15 downto 0)
  );
end entity probe_sat_to;

architecture rtl of probe_sat_to is

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

-- 7. wrap_to: a helper that slices a variable at a parameter-derived index.
entity probe_wrap_to is
  port (
    clk : in    std_logic;
    d   : in    signed(23 downto 0);
    q   : out   signed(15 downto 0)
  );
end entity probe_wrap_to;

architecture rtl of probe_wrap_to is

  signal r : signed(15 downto 0) := (others => '0');

begin

  p : process (clk) is
  begin

    if rising_edge(clk) then
      r <= wrap_to(d, 16);
    end if;

  end process p;

  q <= r;

end architecture rtl;

library ieee;
  use ieee.std_logic_1164.all;
  use ieee.numeric_std.all;

library work;
  use work.foc_pkg.all;

-- 8. overflows: a helper returning a boolean rather than an array.
entity probe_overflows is
  port (
    clk : in    std_logic;
    d   : in    signed(23 downto 0);
    q   : out   std_logic
  );
end entity probe_overflows;

architecture rtl of probe_overflows is

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

library ieee;
  use ieee.std_logic_1164.all;
  use ieee.numeric_std.all;

-- 9. a clock enable written into the edge condition, which is the form the six
--    crashing units use: if rising_edge(clk) and en = '1' then
entity probe_ce_compound is
  port (
    clk : in    std_logic;
    en  : in    std_logic;
    d   : in    signed(23 downto 0);
    q   : out   signed(23 downto 0)
  );
end entity probe_ce_compound;

architecture rtl of probe_ce_compound is

  signal r : signed(23 downto 0) := (others => '0');

begin

  p : process (clk) is
  begin

    if rising_edge(clk) and en = '1' then
      r <= d + 1;
    end if;

  end process p;

  q <= r;

end architecture rtl;

library ieee;
  use ieee.std_logic_1164.all;
  use ieee.numeric_std.all;

-- 10. the same flip-flop with the enable nested inside the edge, which is the
--     canonical template every synthesiser recognises.
entity probe_ce_nested is
  port (
    clk : in    std_logic;
    en  : in    std_logic;
    d   : in    signed(23 downto 0);
    q   : out   signed(23 downto 0)
  );
end entity probe_ce_nested;

architecture rtl of probe_ce_nested is

  signal r : signed(23 downto 0) := (others => '0');

begin

  p : process (clk) is
  begin

    if rising_edge(clk) then
      if en = '1' then
        r <= d + 1;
      end if;
    end if;

  end process p;

  q <= r;

end architecture rtl;
