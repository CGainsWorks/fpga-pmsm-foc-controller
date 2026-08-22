-- -----------------------------------------------------------------------------
-- tb_qei_decoder
--
-- Directed test of the encoder interface.  The configuration is chosen so that
-- the angle mapping can be checked in exact integer arithmetic rather than
-- against a rounded expectation: with 1024 counts per revolution and one pole
-- pair the scale register is 2**32/1024 = 2**22 exactly, so the electrical angle
-- is the position times 64 with no rounding anywhere.  That leaves the mapping
-- either right or wrong, with nothing in between to argue about.
--
-- What is covered
--   counting up and down through all four quadrature states,
--   the modulo-revolution wrap in both directions,
--   the index pulse zeroing the position,
--   both channels moving between two filtered samples raising the sticky error,
--   the error being clearable and not otherwise self-clearing,
--   the speed estimate matching the count rate within its own quantisation.
--
-- What is not covered here is metastability, which no simulation can show: the
-- three-stage synchroniser is justified by the exponential in
-- rtl/qei_decoder.vhd and it is a calculation, not a test.
-- -----------------------------------------------------------------------------

library ieee;
  use ieee.std_logic_1164.all;
  use ieee.numeric_std.all;

library work;
  use work.tb_pkg.all;

entity tb_qei_decoder is
end entity tb_qei_decoder;

architecture sim of tb_qei_decoder is

  constant C_PERIOD : time    := 10 ns;
  constant C_POS_W  : natural := 20;
  constant C_WIN_W  : natural := 24;
  constant C_CPR    : natural := 1024;
  constant C_SCALE  : natural := 4194304; -- 2**32 / 1024
  constant C_DWELL  : natural := 20;      -- clocks per count
  constant C_WIN    : natural := 2000;    -- clocks

  signal clk         : std_logic                      := '0';
  signal rst         : std_logic                      := '1';
  signal clr         : std_logic                      := '0';
  signal a_in        : std_logic                      := '0';
  signal b_in        : std_logic                      := '0';
  signal z_in        : std_logic                      := '0';
  signal cpr         : unsigned(C_POS_W - 1 downto 0) := to_unsigned(C_CPR, C_POS_W);
  signal angle_scale : unsigned(31 downto 0)          := to_unsigned(C_SCALE, 32);
  signal win         : unsigned(C_WIN_W - 1 downto 0) := to_unsigned(C_WIN, C_WIN_W);
  signal use_index   : std_logic                      := '0';
  signal pos         : unsigned(C_POS_W - 1 downto 0);
  signal theta       : signed(15 downto 0);
  signal speed       : signed(15 downto 0);
  signal index_seen  : std_logic;
  signal err         : std_logic;

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

  u_dut : entity work.qei_decoder
    generic map (
      G_POS_BITS => C_POS_W,
      G_FILTER   => 4,
      G_WIN_BITS => C_WIN_W
    )
    port map (
      clk         => clk,
      rst         => rst,
      clr         => clr,
      a_in        => a_in,
      b_in        => b_in,
      z_in        => z_in,
      cpr         => cpr,
      angle_scale => angle_scale,
      win         => win,
      use_index   => use_index,
      pos         => pos,
      theta       => theta,
      speed       => speed,
      index_seen  => index_seen,
      err         => err
    );

  p_run : process is

    variable errs   : natural := 0;
    variable checks : natural := 0;
    variable phase  : natural := 0;

    -- Forward rotation walks A and B through 00, 10, 11, 01.  Exactly one of
    -- the two changes at each step, which is what makes the direction readable
    -- from a single exclusive-or.

    procedure drive (
      p : in natural
    ) is
    begin

      case p is

        when 0 =>

          a_in <= '0';
          b_in <= '0';

        when 1 =>

          a_in <= '1';
          b_in <= '0';

        when 2 =>

          a_in <= '1';
          b_in <= '1';

        when others =>

          a_in <= '0';
          b_in <= '1';

      end case;

    end procedure drive;

    procedure step (
      forward : in boolean
    ) is
    begin

      if forward then
        phase := (phase + 1) mod 4;
      else
        phase := (phase + 3) mod 4;
      end if;

      drive(phase);

      for i in 1 to C_DWELL loop

        wait until rising_edge(clk);

      end loop;

    end procedure step;

  begin

    drive(0);
    wait until rst = '0';

    for i in 1 to 4 * C_DWELL loop

      wait until rising_edge(clk);

    end loop;

    check_int("position after reset", to_integer(pos), 0, errs);
    checks := checks + 1;

    -- 100 counts forward.
    for i in 1 to 100 loop

      step(true);

    end loop;

    check_int("position after 100 forward", to_integer(pos), 100, errs);
    check_int("angle after 100 forward", to_integer(theta), 100 * 64, errs);
    checks := checks + 2;

    -- 30 counts back.
    for i in 1 to 30 loop

      step(false);

    end loop;

    check_int("position after 30 back", to_integer(pos), 70, errs);
    check_int("angle after 30 back", to_integer(theta), 70 * 64, errs);
    checks := checks + 2;

    -- Wrap downwards through zero: 70 back lands on zero, one more must wrap to
    -- the last count of the revolution rather than to a negative position.
    for i in 1 to 71 loop

      step(false);

    end loop;

    check_int("position wraps below zero", to_integer(pos), C_CPR - 1, errs);
    checks := checks + 1;

    -- And upwards through the top.
    step(true);
    check_int("position wraps above the top", to_integer(pos), 0, errs);
    checks := checks + 1;

    -- Speed, over one window.  One count every 20 clocks puts 100 counts in a
    -- 2000 clock window.  The stepping runs for three windows so that at least
    -- one of them lies wholly inside it; the M method quantises to a whole
    -- count and the boundary falls where it falls, so two counts of slack is
    -- the honest tolerance and not a fudge.
    for i in 1 to 300 loop

      step(true);

    end loop;

    check_near("speed estimate", to_integer(speed), C_WIN / C_DWELL, 2, errs);
    checks := checks + 1;

    -- Index.  Enabled, a rising edge on Z must zero the position.
    use_index <= '1';
    wait until rising_edge(clk);
    z_in      <= '1';

    for i in 1 to 8 loop

      wait until rising_edge(clk);

    end loop;

    z_in <= '0';

    for i in 1 to 8 loop

      wait until rising_edge(clk);

    end loop;

    check_int("position zeroed by the index", to_integer(pos), 0, errs);
    check_true("index flag set", index_seen = '1', errs);
    checks    := checks + 2;
    use_index <= '0';

    -- A step of two states at once is not a valid quadrature transition.
    check_true("no error before the fault", err = '0', errs);
    checks := checks + 1;
    phase  := (phase + 2) mod 4;
    drive(phase);

    for i in 1 to 4 * C_DWELL loop

      wait until rising_edge(clk);

    end loop;

    check_true("both channels moving raises the error", err = '1', errs);
    checks := checks + 1;

    -- Sticky until cleared, then clear.
    for i in 1 to 20 loop

      step(true);

    end loop;

    check_true("error stays until it is cleared", err = '1', errs);
    checks := checks + 1;

    clr <= '1';
    wait until rising_edge(clk);
    clr <= '0';

    for i in 1 to 4 loop

      wait until rising_edge(clk);

    end loop;

    check_true("error clears on request", err = '0', errs);
    checks := checks + 1;

    finish_report("tb_qei_decoder", errs, checks);
    wait;

  end process p_run;

  p_timeout : process is
  begin

    wait for 5 ms;
    report "tb_qei_decoder: timed out"
      severity failure;

  end process p_timeout;

end architecture sim;
