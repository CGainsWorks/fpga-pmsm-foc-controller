-- -----------------------------------------------------------------------------
-- tb_pi_axis
--
-- The PI axis against the model, exactly, over a sequence chosen to exercise the
-- two things that are easy to get wrong in a fixed-point controller.
--
-- Windup.  The sequence drives the output hard into its limit and holds it there
-- for two hundred samples, then removes the error.  A controller that kept
-- integrating during the saturation would need roughly as many samples again to
-- unwind and would sit at the limit long after the error changed sign.  The
-- model and the RTL both freeze the integrator, so the output must come off the
-- limit on the sample after the error reverses and the vector comparison holds
-- them to that sample by sample.
--
-- Dead zone.  The random tail includes errors of a single LSB.  If the
-- integrator were held in the output format rather than in the Q?.27 format of
-- the Ki times e product, those samples would produce no state change at all and
-- the recorded outputs would diverge from the model within a few dozen samples.
--
-- Stimulus is applied back to back.  That is not just for speed: it checks that
-- the two-stage pipeline updates its state in the right order when a new sample
-- arrives before the previous one has left.
-- -----------------------------------------------------------------------------

library ieee;
  use ieee.std_logic_1164.all;
  use ieee.numeric_std.all;

library std;
  use std.textio.all;

library vunit_lib;
  context vunit_lib.vunit_context;

library work;
  use work.foc_pkg.all;
  use work.tb_pkg.all;

entity tb_pi_axis is
  generic (
    RUNNER_CFG : string;
    G_VECTORS  : string := "sim/vectors/pi_axis.txt"
  );
end entity tb_pi_axis;

architecture sim of tb_pi_axis is

  constant C_PERIOD : time := 10 ns;

  signal clk       : std_logic                     := '0';
  signal rst       : std_logic                     := '1';
  signal in_valid  : std_logic                     := '0';
  signal ref       : signed(C_W_DATA - 1 downto 0) := (others => '0');
  signal meas      : signed(C_W_DATA - 1 downto 0) := (others => '0');
  signal kp        : signed(C_W_DATA - 1 downto 0) := (others => '0');
  signal ki        : signed(C_W_DATA - 1 downto 0) := (others => '0');
  signal umax      : signed(C_W_DATA - 1 downto 0) := (others => '0');
  signal out_valid : std_logic;
  signal u         : signed(C_W_DATA - 1 downto 0);
  signal sat_out   : std_logic;

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

  u_dut : entity work.pi_axis
    port map (
      clk       => clk,
      rst       => rst,
      clr       => '0',
      in_valid  => in_valid,
      ref       => ref,
      meas      => meas,
      kp        => kp,
      ki        => ki,
      umax      => umax,
      out_valid => out_valid,
      u         => u,
      sat_out   => sat_out
    );

  p_stim : process is

    file     fs : text;
    variable l  : line;
    variable ok : boolean;
    variable a  : integer;
    variable b  : integer;
    variable c  : integer;
    variable v  : integer;

  begin

    wait until rst = '0';
    wait until rising_edge(clk);
    file_open(fs, G_VECTORS, read_mode);

    next_data_line(fs, l, ok);
    assert ok
      report "tb_pi_axis: vector file has no configuration line"
      severity failure;
    read(l, a);
    read(l, b);
    read(l, c);
    deallocate(l);
    kp   <= to_signed(a, C_W_DATA);
    ki   <= to_signed(b, C_W_DATA);
    umax <= to_signed(c, C_W_DATA);
    wait until rising_edge(clk);

    loop

      next_data_line(fs, l, ok);
      exit when not ok;
      read(l, a);
      read(l, b);

      for k in 1 to 2 loop

        read(l, v);

      end loop;

      deallocate(l);
      ref      <= to_signed(a, C_W_DATA);
      meas     <= to_signed(b, C_W_DATA);
      in_valid <= '1';
      wait until rising_edge(clk);

    end loop;

    in_valid <= '0';
    file_close(fs);
    wait;

  end process p_stim;

  p_check : process is

    file     fc   : text;
    variable l    : line;
    variable ok   : boolean;
    variable a    : integer;
    variable b    : integer;
    variable eu   : integer;
    variable es   : integer;
    variable errs : natural := 0;
    variable n    : natural := 0;

  begin

    test_runner_setup(runner, RUNNER_CFG);

    wait until rst = '0';
    file_open(fc, G_VECTORS, read_mode);

    next_data_line(fc, l, ok);
    deallocate(l);

    loop

      next_data_line(fc, l, ok);
      exit when not ok;
      read(l, a);
      read(l, b);
      read(l, eu);
      read(l, es);
      deallocate(l);

      wait until rising_edge(clk) and out_valid = '1';

      check_int("u", to_integer(u), eu, errs);
      check_int("sat", to_int(sat_out), es, errs);
      n := n + 1;

    end loop;

    file_close(fc);
    finish_report("tb_pi_axis", errs, 2 * n);
    test_runner_cleanup(runner);
    wait;

  end process p_check;

  p_timeout : process is
  begin

    wait for 5 ms;
    report "tb_pi_axis: timed out"
      severity failure;

  end process p_timeout;

end architecture sim;
