-- -----------------------------------------------------------------------------
-- tb_current_loop
--
-- The whole datapath, Clarke through to compare values, against the model.
--
-- This is the testbench that catches the mistakes the block-level ones cannot
-- see: the wrong sign on the Park angle, the held angle not being the one the
-- inverse rotation uses, the folded R(theta+phi) identity being folded the wrong
-- way round, a stage started on the wrong valid.  All of those produce perfectly
-- reasonable-looking outputs from every individual block.
--
-- The rows check id, iq, vd, vq and all three compare values, so a mismatch says
-- which stage went wrong rather than only that the answer differs.
--
-- One control update is in flight at a time, which is what the design assumes
-- and what the hardware does: the modulator strobes once per PWM period, 2500
-- clocks apart, against a loop latency of 66.  The testbench holds to that,
-- waiting for each result before presenting the next sample, and the overrun
-- flag in the status word is what would catch a system that did not.
-- -----------------------------------------------------------------------------

library ieee;
  use ieee.std_logic_1164.all;
  use ieee.numeric_std.all;

library std;
  use std.textio.all;

library work;
  use work.foc_pkg.all;
  use work.tb_pkg.all;

entity tb_current_loop is
  generic (
    G_VECTORS  : string  := "sim/vectors/current_loop.txt";
    G_PWM_BITS : natural := 13
  );
end entity tb_current_loop;

architecture sim of tb_current_loop is

  constant C_PERIOD : time := 10 ns;

  signal clk          : std_logic                         := '0';
  signal rst          : std_logic                         := '1';
  signal sample_valid : std_logic                         := '0';
  signal ia           : signed(C_W_DATA - 1 downto 0)     := (others => '0');
  signal ib           : signed(C_W_DATA - 1 downto 0)     := (others => '0');
  signal theta        : signed(C_W_ANG - 1 downto 0)      := (others => '0');
  signal id_ref       : signed(C_W_DATA - 1 downto 0)     := (others => '0');
  signal iq_ref       : signed(C_W_DATA - 1 downto 0)     := (others => '0');
  signal kp           : signed(C_W_DATA - 1 downto 0)     := (others => '0');
  signal ki           : signed(C_W_DATA - 1 downto 0)     := (others => '0');
  signal vmax         : signed(C_W_DATA - 1 downto 0)     := (others => '0');
  signal pwm_top      : unsigned(G_PWM_BITS - 1 downto 0) := (others => '0');
  signal out_valid    : std_logic;
  signal cmp_a        : unsigned(G_PWM_BITS - 1 downto 0);
  signal cmp_b        : unsigned(G_PWM_BITS - 1 downto 0);
  signal cmp_c        : unsigned(G_PWM_BITS - 1 downto 0);
  signal id_meas      : signed(C_W_DATA - 1 downto 0);
  signal iq_meas      : signed(C_W_DATA - 1 downto 0);
  signal vd_out       : signed(C_W_DATA - 1 downto 0);
  signal vq_out       : signed(C_W_DATA - 1 downto 0);
  signal sat_flags    : std_logic_vector(3 downto 0);

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

  u_dut : entity work.current_loop
    generic map (
      G_PWM_BITS => G_PWM_BITS
    )
    port map (
      clk          => clk,
      rst          => rst,
      clr          => '0',
      sample_valid => sample_valid,
      ia           => ia,
      ib           => ib,
      theta        => theta,
      id_ref       => id_ref,
      iq_ref       => iq_ref,
      kp           => kp,
      ki           => ki,
      vmax         => vmax,
      pwm_top      => pwm_top,
      out_valid    => out_valid,
      cmp_a        => cmp_a,
      cmp_b        => cmp_b,
      cmp_c        => cmp_c,
      id_meas      => id_meas,
      iq_meas      => iq_meas,
      vd_out       => vd_out,
      vq_out       => vq_out,
      sat_flags    => sat_flags
    );

  p_run : process is

    file     f    : text;
    variable l    : line;
    variable ok   : boolean;
    variable a    : integer;
    variable b    : integer;
    variable c    : integer;
    variable d    : integer;
    variable sia  : integer;
    variable sib  : integer;
    variable sth  : integer;
    variable sidr : integer;
    variable siqr : integer;
    variable eid  : integer;
    variable eiq  : integer;
    variable evd  : integer;
    variable evq  : integer;
    variable eca  : integer;
    variable ecb  : integer;
    variable ecc  : integer;
    variable errs : natural := 0;
    variable n    : natural := 0;

  begin

    wait until rst = '0';
    wait until rising_edge(clk);
    file_open(f, G_VECTORS, read_mode);

    next_data_line(f, l, ok);
    assert ok
      report "tb_current_loop: vector file has no configuration line"
      severity failure;
    read(l, a);
    read(l, b);
    read(l, c);
    read(l, d);
    deallocate(l);
    kp      <= to_signed(a, C_W_DATA);
    ki      <= to_signed(b, C_W_DATA);
    vmax    <= to_signed(c, C_W_DATA);
    pwm_top <= to_unsigned(d, G_PWM_BITS);
    wait until rising_edge(clk);

    loop

      next_data_line(f, l, ok);
      exit when not ok;
      read(l, sia);
      read(l, sib);
      read(l, sth);
      read(l, sidr);
      read(l, siqr);
      read(l, eid);
      read(l, eiq);
      read(l, evd);
      read(l, evq);
      read(l, eca);
      read(l, ecb);
      read(l, ecc);
      deallocate(l);

      ia           <= to_signed(sia, C_W_DATA);
      ib           <= to_signed(sib, C_W_DATA);
      theta        <= to_signed(sth, C_W_ANG);
      id_ref       <= to_signed(sidr, C_W_DATA);
      iq_ref       <= to_signed(siqr, C_W_DATA);
      sample_valid <= '1';
      wait until rising_edge(clk);
      sample_valid <= '0';

      wait until rising_edge(clk) and out_valid = '1';

      check_int("id", to_integer(id_meas), eid, errs);
      check_int("iq", to_integer(iq_meas), eiq, errs);
      check_int("vd", to_integer(vd_out), evd, errs);
      check_int("vq", to_integer(vq_out), evq, errs);
      check_int("cmp_a", to_integer(cmp_a), eca, errs);
      check_int("cmp_b", to_integer(cmp_b), ecb, errs);
      check_int("cmp_c", to_integer(cmp_c), ecc, errs);
      n := n + 1;

    end loop;

    file_close(f);
    check_true("no overrun was flagged", sat_flags(0) = '0', errs);
    finish_report("tb_current_loop", errs, 7 * n);
    wait;

  end process p_run;

  p_timeout : process is
  begin

    wait for 20 ms;
    report "tb_current_loop: timed out"
      severity failure;

  end process p_timeout;

end architecture sim;
