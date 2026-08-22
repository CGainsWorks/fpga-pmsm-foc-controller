-- -----------------------------------------------------------------------------
-- tb_svm
--
-- The modulator against the model, exactly, plus two properties that are the
-- reason the modulator is built this way at all.
--
-- Centring.  Whenever the reference is inside the linear range, the largest and
-- the smallest of the three compare values must add up to the carrier top, to
-- within the rounding of one halving.  That is the statement that the injection
-- has centred the three references in the rail, which is the same thing as
-- splitting the zero-vector time equally between V0 and V7.  If the offset were
-- computed wrongly or not at all, this is the check that fails.
--
-- Linear range.  The vector file sweeps a balanced set at 0.5, 1.0 and 1.1 per
-- unit.  At 1.0 the duties must reach the rails and the over-modulation flag
-- must still be clear; at 1.1 it must be set.  The boundary landing exactly on
-- 1.0 per unit is not a coincidence, it is the choice of V_base = V_dc/sqrt(3)
-- combined with the 15.47 per cent that the injection buys.
-- -----------------------------------------------------------------------------

library ieee;
  use ieee.std_logic_1164.all;
  use ieee.numeric_std.all;

library std;
  use std.textio.all;

library work;
  use work.foc_pkg.all;
  use work.tb_pkg.all;

entity tb_svm is
  generic (
    G_VECTORS  : string  := "sim/vectors/svm.txt";
    G_PWM_BITS : natural := 13
  );
end entity tb_svm;

architecture sim of tb_svm is

  constant C_PERIOD : time := 10 ns;

  signal clk       : std_logic                         := '0';
  signal rst       : std_logic                         := '1';
  signal in_valid  : std_logic                         := '0';
  signal valpha    : signed(C_W_DATA - 1 downto 0)     := (others => '0');
  signal vbeta     : signed(C_W_DATA - 1 downto 0)     := (others => '0');
  signal pwm_top   : unsigned(G_PWM_BITS - 1 downto 0) := (others => '0');
  signal out_valid : std_logic;
  signal cmp_a     : unsigned(G_PWM_BITS - 1 downto 0);
  signal cmp_b     : unsigned(G_PWM_BITS - 1 downto 0);
  signal cmp_c     : unsigned(G_PWM_BITS - 1 downto 0);
  signal over_mod  : std_logic;

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

  u_dut : entity work.svm
    generic map (
      G_PWM_BITS => G_PWM_BITS
    )
    port map (
      clk       => clk,
      rst       => rst,
      in_valid  => in_valid,
      valpha    => valpha,
      vbeta     => vbeta,
      pwm_top   => pwm_top,
      out_valid => out_valid,
      cmp_a     => cmp_a,
      cmp_b     => cmp_b,
      cmp_c     => cmp_c,
      over_mod  => over_mod
    );

  p_stim : process is

    file     fs  : text;
    variable l   : line;
    variable ok  : boolean;
    variable top : integer;
    variable v   : integer;
    variable al  : integer;
    variable be  : integer;

  begin

    wait until rst = '0';
    wait until rising_edge(clk);
    file_open(fs, G_VECTORS, read_mode);

    next_data_line(fs, l, ok);
    assert ok
      report "tb_svm: vector file has no configuration line"
      severity failure;
    read(l, top);
    deallocate(l);
    pwm_top <= to_unsigned(top, G_PWM_BITS);
    wait until rising_edge(clk);

    loop

      next_data_line(fs, l, ok);
      exit when not ok;
      read(l, al);
      read(l, be);

      for k in 1 to 4 loop

        read(l, v);

      end loop;

      deallocate(l);
      valpha   <= to_signed(al, C_W_DATA);
      vbeta    <= to_signed(be, C_W_DATA);
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
    variable top  : integer;
    variable al   : integer;
    variable be   : integer;
    variable ea   : integer;
    variable eb   : integer;
    variable ec   : integer;
    variable eo   : integer;
    variable errs : natural := 0;
    variable n    : natural := 0;
    variable hi   : integer;
    variable lo   : integer;

  begin

    wait until rst = '0';
    file_open(fc, G_VECTORS, read_mode);

    next_data_line(fc, l, ok);
    read(l, top);
    deallocate(l);

    loop

      next_data_line(fc, l, ok);
      exit when not ok;
      read(l, al);
      read(l, be);
      read(l, ea);
      read(l, eb);
      read(l, ec);
      read(l, eo);
      deallocate(l);

      wait until rising_edge(clk) and out_valid = '1';

      check_int("cmp_a", to_integer(cmp_a), ea, errs);
      check_int("cmp_b", to_integer(cmp_b), eb, errs);
      check_int("cmp_c", to_integer(cmp_c), ec, errs);
      check_int("over_mod", to_int(over_mod), eo, errs);

      if over_mod = '0' then
        hi := to_integer(cmp_a);
        lo := hi;
        if to_integer(cmp_b) > hi then
          hi := to_integer(cmp_b);
        end if;
        if to_integer(cmp_c) > hi then
          hi := to_integer(cmp_c);
        end if;
        if to_integer(cmp_b) < lo then
          lo := to_integer(cmp_b);
        end if;
        if to_integer(cmp_c) < lo then
          lo := to_integer(cmp_c);
        end if;
        check_near("references centred in the rail", hi + lo, top, 2, errs);
      end if;

      n := n + 1;

    end loop;

    file_close(fc);
    finish_report("tb_svm", errs, 4 * n);
    wait;

  end process p_check;

  p_timeout : process is
  begin

    wait for 5 ms;
    report "tb_svm: timed out"
      severity failure;

  end process p_timeout;

end architecture sim;
