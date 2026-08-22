-- -----------------------------------------------------------------------------
-- tb_pwm3ph
--
-- The modulator output stage, checked by measuring the waveform rather than by
-- inspecting it.  Each property is derived in docs/06-svpwm-and-pwm.md and
-- measured over whole periods.
--
-- Period.  Exactly 2*top clocks between sample strobes.  Anything else means the
-- up-down counter turns at the wrong place and every duty in the design is out
-- by a scale factor.
--
-- Duty.  With no dead time a leg is high for exactly 2*cmp - 1 clocks per
-- period and not at all when cmp is zero.  The half-clock shortfall is
-- identical on all three legs, which is what makes it a common-mode term a
-- machine with an isolated star point cannot see.  The check is on the exact
-- clock count rather than on an approximate duty, because a systematic half
-- clock is exactly the sort of thing a percentage hides.
--
-- Dead time.  With the register set to D, a leg whose on and off intervals are
-- both longer than D loses exactly D clocks of conduction and there are exactly
-- two both-off intervals per period, each of exactly D clocks.  The
-- two-transistors-on condition is checked on every clock of the run, not only at
-- the transitions.
--
-- Narrow pulses.  A leg asked for a duty so close to the rail that its off time
-- is shorter than the dead time cannot be given that duty by any inverter: the
-- pulse is swallowed.  The testbench pins this down as an inequality rather than
-- an equality, because the exact loss depends on where the requested edge falls
-- against the clock and it is the reason docs/10-limitations-and-roadmap.md
-- puts minimum-pulse handling next to dead-time compensation.
-- -----------------------------------------------------------------------------

library ieee;
  use ieee.std_logic_1164.all;
  use ieee.numeric_std.all;

library work;
  use work.tb_pkg.all;

entity tb_pwm3ph is
  generic (
    G_PWM_BITS : natural := 13;
    G_DT_BITS  : natural := 10
  );
end entity tb_pwm3ph;

architecture sim of tb_pwm3ph is

  constant C_PERIOD : time    := 10 ns;
  constant C_TOP    : natural := 100;
  constant C_DT     : natural := 7;

  type cmp_arr_t is array (0 to 2) of natural;

  constant C_CMP : cmp_arr_t := (0, 25, 75);

  signal clk      : std_logic                         := '0';
  signal rst      : std_logic                         := '1';
  signal enable   : std_logic                         := '1';
  signal pwm_top  : unsigned(G_PWM_BITS - 1 downto 0) := to_unsigned(C_TOP, G_PWM_BITS);
  signal deadtime : unsigned(G_DT_BITS - 1 downto 0)  := (others => '0');
  signal cmp_a    : unsigned(G_PWM_BITS - 1 downto 0) := to_unsigned(C_CMP(0), G_PWM_BITS);
  signal cmp_b    : unsigned(G_PWM_BITS - 1 downto 0) := to_unsigned(C_CMP(1), G_PWM_BITS);
  signal cmp_c    : unsigned(G_PWM_BITS - 1 downto 0) := to_unsigned(C_CMP(2), G_PWM_BITS);
  signal sample   : std_logic;
  signal hi       : std_logic_vector(2 downto 0);
  signal lo       : std_logic_vector(2 downto 0);

begin

  clk <= not clk after C_PERIOD / 2;

  p_reset : process is
  begin

    rst <= '1';
    wait for 4 * C_PERIOD;
    wait until rising_edge(clk);
    rst <= '0';
    wait;

  end process p_reset;

  u_dut : entity work.pwm3ph
    generic map (
      G_PWM_BITS => G_PWM_BITS,
      G_DT_BITS  => G_DT_BITS
    )
    port map (
      clk      => clk,
      rst      => rst,
      enable   => enable,
      pwm_top  => pwm_top,
      deadtime => deadtime,
      cmp_a    => cmp_a,
      cmp_b    => cmp_b,
      cmp_c    => cmp_c,
      sample   => sample,
      hi       => hi,
      lo       => lo
    );

  p_run : process is

    variable errs   : natural := 0;
    variable checks : natural := 0;
    variable high   : cmp_arr_t;
    variable gaps   : natural;
    variable run    : natural;
    variable maxrun : natural;
    variable len    : natural;
    variable exp    : natural;

  begin

    wait until rst = '0';

    -- --------------------------------------------------------------------
    -- Period length, between two consecutive strobes.
    -- --------------------------------------------------------------------
    wait until rising_edge(clk) and sample = '1';
    len := 0;

    loop

      wait until rising_edge(clk);
      len := len + 1;
      exit when sample = '1';

    end loop;

    check_int("period in clocks", len, 2 * C_TOP, errs);
    checks := checks + 1;

    -- --------------------------------------------------------------------
    -- Duty, first without and then with dead time.  Measurement starts on a
    -- strobe, which is the carrier trough and covers exactly one period.
    -- --------------------------------------------------------------------
    for pass in 0 to 1 loop

      if pass = 1 then
        deadtime <= to_unsigned(C_DT, G_DT_BITS);
        wait until rising_edge(clk) and sample = '1';
        wait until rising_edge(clk) and sample = '1';
      end if;

      wait until rising_edge(clk) and sample = '1';
      high   := (others => 0);
      gaps   := 0;
      run    := 0;
      maxrun := 0;

      for i in 1 to 2 * C_TOP loop

        wait until rising_edge(clk);

        for k in 0 to 2 loop

          if hi(k) = '1' then
            high(k) := high(k) + 1;
          end if;

          check_true("never both transistors on",
                     not (hi(k) = '1' and lo(k) = '1'), errs);

        end loop;

        -- Both-off intervals are counted on leg 1, whose duty is neither zero
        -- nor full and which therefore switches twice a period.
        if hi(1) = '0' and lo(1) = '0' then
          run := run + 1;
        elsif run > 0 then
          gaps := gaps + 1;
          if run > maxrun then
            maxrun := run;
          end if;
          run := 0;
        end if;

      end loop;

      checks := checks + 6 * C_TOP;

      for k in 0 to 2 loop

        if C_CMP(k) = 0 then
          exp := 0;
        else
          exp := 2 * C_CMP(k) - 1;
          if pass = 1 then
            exp := exp - C_DT;
          end if;
        end if;

        check_int("high clocks on leg " & integer'image(k), high(k), exp, errs);
        checks := checks + 1;

      end loop;

      if pass = 1 then
        check_int("both-off intervals per period", gaps, 2, errs);
        check_int("both-off interval length", maxrun, C_DT, errs);
        checks := checks + 2;
      end if;

    end loop;

    -- --------------------------------------------------------------------
    -- Full modulation.  With no dead time the top compare value gives 2*top - 1
    -- clocks, one short of the whole period, which is the same common-mode half
    -- clock the linear cases show.
    -- --------------------------------------------------------------------
    deadtime <= (others => '0');
    cmp_a    <= to_unsigned(C_TOP, G_PWM_BITS);
    wait until rising_edge(clk) and sample = '1';
    wait until rising_edge(clk) and sample = '1';
    wait until rising_edge(clk) and sample = '1';
    high     := (others => 0);

    for i in 1 to 2 * C_TOP loop

      wait until rising_edge(clk);

      if hi(0) = '1' then
        high(0) := high(0) + 1;
      end if;

    end loop;

    check_int("high clocks at full modulation", high(0), 2 * C_TOP - 1, errs);
    checks := checks + 1;

    -- With dead time the same request cannot be met and the leg must lose at
    -- least the dead time rather than silently rounding up to full conduction.
    deadtime <= to_unsigned(C_DT, G_DT_BITS);
    wait until rising_edge(clk) and sample = '1';
    wait until rising_edge(clk) and sample = '1';
    wait until rising_edge(clk) and sample = '1';
    high     := (others => 0);

    for i in 1 to 2 * C_TOP loop

      wait until rising_edge(clk);

      if hi(0) = '1' then
        high(0) := high(0) + 1;
      end if;

    end loop;

    check_true("narrow off pulse loses at least the dead time",
               high(0) <= 2 * C_TOP - 1 - C_DT, errs);
    checks := checks + 1;

    -- --------------------------------------------------------------------
    -- The enable is a hardware interlock, not a request: dropping it must put
    -- every gate low on the next clock and keep it there.
    -- --------------------------------------------------------------------
    enable <= '0';
    wait until rising_edge(clk);
    wait until rising_edge(clk);

    for i in 1 to 4 * C_TOP loop

      wait until rising_edge(clk);
      check_true("all gates off while disabled",
                 hi = "000" and lo = "000", errs);

    end loop;

    checks := checks + 4 * C_TOP;

    finish_report("tb_pwm3ph", errs, checks);
    wait;

  end process p_run;

  p_timeout : process is
  begin

    wait for 5 ms;
    report "tb_pwm3ph: timed out"
      severity failure;

  end process p_timeout;

end architecture sim;
