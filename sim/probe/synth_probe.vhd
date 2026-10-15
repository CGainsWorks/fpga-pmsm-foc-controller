-- -----------------------------------------------------------------------------
-- synth_probe  -- TEMPORARY, not part of the design
--
-- Bisecting the GHDL 5.0.1 synthesis crash by reduction.
--
-- The first round probed each construct on its own: the built-in resize, a
-- shift by a literal, a shift by a generic, a signed multiply and all four
-- arithmetic helpers from rtl/foc_pkg.vhd. The second round added the clock
-- enable written into the edge condition, both ways round. All ten elaborate,
-- so no single construct in the design is the cause and it has to be an
-- interaction.
--
-- So this round takes the smallest crashing unit, clarke, copies its
-- architecture verbatim, and removes one feature per variant. Whichever variant
-- stops crashing names the cause.
--
-- Delete this file and the synth-probe target once the answer is known.
-- -----------------------------------------------------------------------------

library ieee;
  use ieee.std_logic_1164.all;
  use ieee.numeric_std.all;

-- Control from the previous round: a clock enable in the edge condition.
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

library work;
  use work.foc_pkg.all;

-- A. clarke's architecture, verbatim. Expected to crash.
entity probe_c_full is
  port (
    clk       : in    std_logic;
    rst       : in    std_logic;
    in_valid  : in    std_logic;
    ia        : in    signed(C_W_DATA - 1 downto 0);
    ib        : in    signed(C_W_DATA - 1 downto 0);
    out_valid : out   std_logic;
    ialpha    : out   signed(C_W_DATA - 1 downto 0);
    ibeta     : out   signed(C_W_DATA - 1 downto 0);
    sat_out   : out   std_logic
  );
end entity probe_c_full;

architecture rtl of probe_c_full is

  constant C_W_SUM  : natural := C_W_DATA + 2;
  constant C_W_PROD : natural := C_W_SUM + 16;

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

library ieee;
  use ieee.std_logic_1164.all;
  use ieee.numeric_std.all;

library work;
  use work.foc_pkg.all;

-- B. A, with the clock enable taken out of the edge condition.
entity probe_c_noce is
  port (
    clk       : in    std_logic;
    rst       : in    std_logic;
    in_valid  : in    std_logic;
    ia        : in    signed(C_W_DATA - 1 downto 0);
    ib        : in    signed(C_W_DATA - 1 downto 0);
    out_valid : out   std_logic;
    ialpha    : out   signed(C_W_DATA - 1 downto 0);
    ibeta     : out   signed(C_W_DATA - 1 downto 0);
    sat_out   : out   std_logic
  );
end entity probe_c_noce;

architecture rtl of probe_c_noce is

  constant C_W_SUM  : natural := C_W_DATA + 2;
  constant C_W_PROD : natural := C_W_SUM + 16;

  signal ial_r : signed(C_W_DATA - 1 downto 0) := (others => '0');
  signal ibe_r : signed(C_W_DATA - 1 downto 0) := (others => '0');
  signal sat_r : std_logic                     := '0';

  signal vld : std_logic;

begin

  p_clarke : process (clk) is

    variable t : signed(C_W_SUM - 1 downto 0);
    variable p : signed(C_W_PROD - 1 downto 0);

  begin

    if rising_edge(clk) then
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

library ieee;
  use ieee.std_logic_1164.all;
  use ieee.numeric_std.all;

library work;
  use work.foc_pkg.all;

-- C. A, with the helpers replaced by the same arithmetic written inline.
entity probe_c_nohelp is
  port (
    clk       : in    std_logic;
    rst       : in    std_logic;
    in_valid  : in    std_logic;
    ia        : in    signed(C_W_DATA - 1 downto 0);
    ib        : in    signed(C_W_DATA - 1 downto 0);
    out_valid : out   std_logic;
    ialpha    : out   signed(C_W_DATA - 1 downto 0);
    ibeta     : out   signed(C_W_DATA - 1 downto 0);
    sat_out   : out   std_logic
  );
end entity probe_c_nohelp;

architecture rtl of probe_c_nohelp is

  constant C_W_SUM  : natural := C_W_DATA + 2;
  constant C_W_PROD : natural := C_W_SUM + 16;

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
      p := shift_right(t * to_signed(C_INV_SQRT3, 16)
                       + to_signed(2 ** (C_Q_TRIG - 1), C_W_PROD), C_Q_TRIG);

      ial_r <= ia;
      ibe_r <= resize(p, C_W_DATA);

      if p > 32767 or p < -32768 then
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

library ieee;
  use ieee.std_logic_1164.all;
  use ieee.numeric_std.all;

library work;
  use work.foc_pkg.all;

-- D. A, with no process variables: the helpers are applied to expressions.
entity probe_c_novar is
  port (
    clk       : in    std_logic;
    rst       : in    std_logic;
    in_valid  : in    std_logic;
    ia        : in    signed(C_W_DATA - 1 downto 0);
    ib        : in    signed(C_W_DATA - 1 downto 0);
    out_valid : out   std_logic;
    ialpha    : out   signed(C_W_DATA - 1 downto 0);
    ibeta     : out   signed(C_W_DATA - 1 downto 0);
    sat_out   : out   std_logic
  );
end entity probe_c_novar;

architecture rtl of probe_c_novar is

  constant C_W_SUM : natural := C_W_DATA + 2;

  signal ial_r : signed(C_W_DATA - 1 downto 0) := (others => '0');
  signal ibe_r : signed(C_W_DATA - 1 downto 0) := (others => '0');
  signal sat_r : std_logic                     := '0';

  signal vld : std_logic;

begin

  p_clarke : process (clk) is
  begin

    if rising_edge(clk) and in_valid = '1' then
      ial_r <= ia;
      ibe_r <= sat_to(round_shr((resize(ia, C_W_SUM)
                                 + shift_left(resize(ib, C_W_SUM), 1))
                                * to_signed(C_INV_SQRT3, 16), C_Q_TRIG),
                      C_W_DATA);
      sat_r <= '0';
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

library ieee;
  use ieee.std_logic_1164.all;
  use ieee.numeric_std.all;

library work;
  use work.foc_pkg.all;

-- E. A, with no signal initialisers.
entity probe_c_noinit is
  port (
    clk       : in    std_logic;
    rst       : in    std_logic;
    in_valid  : in    std_logic;
    ia        : in    signed(C_W_DATA - 1 downto 0);
    ib        : in    signed(C_W_DATA - 1 downto 0);
    out_valid : out   std_logic;
    ialpha    : out   signed(C_W_DATA - 1 downto 0);
    ibeta     : out   signed(C_W_DATA - 1 downto 0);
    sat_out   : out   std_logic
  );
end entity probe_c_noinit;

architecture rtl of probe_c_noinit is

  constant C_W_SUM  : natural := C_W_DATA + 2;
  constant C_W_PROD : natural := C_W_SUM + 16;

  signal ial_r : signed(C_W_DATA - 1 downto 0);
  signal ibe_r : signed(C_W_DATA - 1 downto 0);
  signal sat_r : std_logic;

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
