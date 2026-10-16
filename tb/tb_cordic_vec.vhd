-- -----------------------------------------------------------------------------
-- tb_cordic_vec
--
-- Vectoring-mode CORDIC, checked against the model exactly and against real
-- arithmetic within a derived bound, the same two-sided arrangement as
-- tb_cordic_rot.
--
-- The angle bound holds for every input including vectors only a few LSB long,
-- where the argument is badly conditioned in the input quantisation and not in
-- the algorithm.  That is worth stating because it is the case the voltage
-- limiter meets whenever the controller is asking for almost no voltage and an
-- implementation that lost the angle there would put the machine's voltage in an
-- arbitrary direction every time it came out of standstill.
-- -----------------------------------------------------------------------------

library ieee;
  use ieee.std_logic_1164.all;
  use ieee.numeric_std.all;
  use ieee.math_real.all;

library std;
  use std.textio.all;

library vunit_lib;
  context vunit_lib.vunit_context;

library work;
  use work.foc_pkg.all;
  use work.tb_pkg.all;

entity tb_cordic_vec is
  generic (
    RUNNER_CFG : string;
    G_VECTORS  : string := "sim/vectors/cordic_vec.txt"
  );
end entity tb_cordic_vec;

architecture sim of tb_cordic_vec is

  constant C_PERIOD  : time    := 10 ns;
  constant C_TOL_MAG : integer := 2; -- LSB
  constant C_TOL_ANG : integer := 2; -- angle counts

  signal clk       : std_logic                     := '0';
  signal rst       : std_logic                     := '1';
  signal in_valid  : std_logic                     := '0';
  signal x_in      : signed(C_W_DATA - 1 downto 0) := (others => '0');
  signal y_in      : signed(C_W_DATA - 1 downto 0) := (others => '0');
  signal out_valid : std_logic;
  signal x_out     : signed(C_W_DATA - 1 downto 0);
  signal y_out     : signed(C_W_DATA - 1 downto 0);
  signal ang_out   : signed(C_W_ANG - 1 downto 0);
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

  u_dut : entity work.cordic
    generic map (
      G_MODE       => vector_mode,
      G_COMPENSATE => true
    )
    port map (
      clk       => clk,
      rst       => rst,
      in_valid  => in_valid,
      x_in      => x_in,
      y_in      => y_in,
      ang_in    => (others => '0'),
      out_valid => out_valid,
      x_out     => x_out,
      y_out     => y_out,
      ang_out   => ang_out,
      sat_out   => sat_out
    );

  p_stim : process is

    file     fs : text;
    variable l  : line;
    variable ok : boolean;
    variable xi : integer;
    variable yi : integer;
    variable em : integer;
    variable ea : integer;

  begin

    wait until rst = '0';
    wait until rising_edge(clk);
    file_open(fs, G_VECTORS, read_mode);

    loop

      next_data_line(fs, l, ok);
      exit when not ok;
      read(l, xi);
      read(l, yi);
      read(l, em);
      read(l, ea);
      deallocate(l);
      x_in     <= to_signed(xi, C_W_DATA);
      y_in     <= to_signed(yi, C_W_DATA);
      in_valid <= '1';
      wait until rising_edge(clk);

    end loop;

    in_valid <= '0';
    file_close(fs);
    wait;

  end process p_stim;

  p_check : process is

    file     fc     : text;
    variable l      : line;
    variable ok     : boolean;
    variable xi     : integer;
    variable yi     : integer;
    variable em     : integer;
    variable ea     : integer;
    variable errs   : natural := 0;
    variable n      : natural := 0;
    variable mag    : real;
    variable ang    : real;
    variable d      : real;
    variable worstm : real    := 0.0;
    variable worsta : real    := 0.0;

  begin

    test_runner_setup(runner, RUNNER_CFG);

    wait until rst = '0';
    file_open(fc, G_VECTORS, read_mode);

    loop

      next_data_line(fc, l, ok);
      exit when not ok;
      read(l, xi);
      read(l, yi);
      read(l, em);
      read(l, ea);
      deallocate(l);

      wait until rising_edge(clk) and out_valid = '1';

      check_int("mag", to_integer(x_out), em, errs);
      check_int("ang_out", to_integer(ang_out), ea, errs);

      mag := sqrt(real(xi) * real(xi) + real(yi) * real(yi));

      if abs (real(to_integer(x_out)) - mag) > worstm then
        worstm := abs (real(to_integer(x_out)) - mag);
      end if;

      check_true("magnitude within the derived bound",
                 abs (real(to_integer(x_out)) - mag) <= real(C_TOL_MAG), errs);

      if xi /= 0 or yi /= 0 then
        ang := arctan(real(yi), real(xi)) / MATH_PI * 32768.0;
        d   := real(to_integer(ang_out)) - ang;

        while d > 32768.0 loop

          d := d - 65536.0;

        end loop;

        while d < -32768.0 loop

          d := d + 65536.0;

        end loop;

        if abs (d) > worsta then
          worsta := abs (d);
        end if;
        check_true("argument within the derived bound",
                   abs (d) <= real(C_TOL_ANG), errs);
      end if;

      n := n + 1;

    end loop;

    file_close(fc);
    report "tb_cordic_vec: worst magnitude error " & real'image(worstm)
           & " LSB, worst argument error " & real'image(worsta) & " counts"
      severity note;
    finish_report("tb_cordic_vec", errs, 3 * n);
    test_runner_cleanup(runner);
    wait;

  end process p_check;

  p_timeout : process is
  begin

    wait for 5 ms;
    report "tb_cordic_vec: timed out"
      severity failure;

  end process p_timeout;

end architecture sim;
