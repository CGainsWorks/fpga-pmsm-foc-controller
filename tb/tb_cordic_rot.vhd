-- -----------------------------------------------------------------------------
-- tb_cordic_rot
--
-- Two independent checks on the rotation-mode CORDIC, run at the same time.
--
-- Against the model.  Every output must equal the value focmodel produced for
-- the same input, exactly.  This is the check that catches an off-by-one in a
-- shift or a rounding term and it is worth nothing if it is given a tolerance.
--
-- Against the mathematics.  Every output is also compared with the true rotation
-- computed in floating point by ieee.math_real, within the bound derived in
-- docs/03-cordic.md.  This is the check that catches the model and the RTL being
-- wrong together, which is exactly what a model-only comparison cannot see.  The
-- error budget there is
--
--   residual angle    atan(2**-17) = 7.63e-6 rad, times |v| <= 2.83 -> 0.18 LSB
--   angle table       18 entries rounded to 2**-21 of a half turn   -> 0.10 LSB
--   shift truncation  18 stages at 2**-19, amplified by K           -> 0.16 LSB
--   output rounding   one half LSB by construction                  -> 0.50 LSB
--                                                              total  0.94 LSB
--
-- so the testbench allows 2 LSB and reports what it actually measured.  If the
-- measured figure ever creeps towards the limit, the word lengths moved.
--
-- Stimulus is applied back to back, one vector per clock, so the pipeline is
-- checked at full throughput rather than one transaction at a time.
-- -----------------------------------------------------------------------------

library ieee;
  use ieee.std_logic_1164.all;
  use ieee.numeric_std.all;
  use ieee.math_real.all;

library std;
  use std.textio.all;

library work;
  use work.foc_pkg.all;
  use work.tb_pkg.all;

entity tb_cordic_rot is
  generic (
    G_VECTORS : string := "sim/vectors/cordic_rot.txt"
  );
end entity tb_cordic_rot;

architecture sim of tb_cordic_rot is

  constant C_PERIOD : time    := 10 ns;
  constant C_TOL    : integer := 2;                        -- LSB, see header

  signal clk       : std_logic                     := '0';
  signal rst       : std_logic                     := '1';
  signal in_valid  : std_logic                     := '0';
  signal x_in      : signed(C_W_DATA - 1 downto 0) := (others => '0');
  signal y_in      : signed(C_W_DATA - 1 downto 0) := (others => '0');
  signal ang_in    : signed(C_W_ANG - 1 downto 0)  := (others => '0');
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
      G_MODE       => rotate_mode,
      G_COMPENSATE => true
    )
    port map (
      clk       => clk,
      rst       => rst,
      in_valid  => in_valid,
      x_in      => x_in,
      y_in      => y_in,
      ang_in    => ang_in,
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
    variable ai : integer;
    variable ex : integer;
    variable ey : integer;
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
      read(l, ai);
      read(l, ex);
      read(l, ey);
      read(l, ea);
      deallocate(l);
      x_in     <= to_signed(xi, C_W_DATA);
      y_in     <= to_signed(yi, C_W_DATA);
      ang_in   <= to_signed(ai, C_W_ANG);
      in_valid <= '1';
      wait until rising_edge(clk);

    end loop;

    in_valid <= '0';
    file_close(fs);
    wait;

  end process p_stim;

  p_check : process is

    file     fc    : text;
    variable l     : line;
    variable ok    : boolean;
    variable xi    : integer;
    variable yi    : integer;
    variable ai    : integer;
    variable ex    : integer;
    variable ey    : integer;
    variable ea    : integer;
    variable errs  : natural := 0;
    variable n     : natural := 0;
    variable ang   : real;
    variable rx    : real;
    variable ry    : real;
    variable err   : real;
    variable worst : real    := 0.0;

  begin

    wait until rst = '0';
    file_open(fc, G_VECTORS, read_mode);

    loop

      next_data_line(fc, l, ok);
      exit when not ok;
      read(l, xi);
      read(l, yi);
      read(l, ai);
      read(l, ex);
      read(l, ey);
      read(l, ea);
      deallocate(l);

      wait until rising_edge(clk) and out_valid = '1';

      check_int("x_out", to_integer(x_out), ex, errs);
      check_int("y_out", to_integer(y_out), ey, errs);
      check_int("ang_out", to_integer(ang_out), ea, errs);

      -- The same result against real arithmetic.  ang_in is a binary angular
      -- measure, so the angle in radians is ang * pi / 2**15.
      ang := real(ai) * MATH_PI / 32768.0;
      rx  := real(xi) * cos(ang) - real(yi) * sin(ang);
      ry  := real(xi) * sin(ang) + real(yi) * cos(ang);
      err := realmax(abs (real(to_integer(x_out)) - rx),
                     abs (real(to_integer(y_out)) - ry));

      if err > worst then
        worst := err;
      end if;

      check_true("rotation within the derived bound", err <= real(C_TOL), errs);

      n := n + 1;

    end loop;

    file_close(fc);
    report "tb_cordic_rot: worst error against real arithmetic "
           & real'image(worst) & " LSB, bound " & integer'image(C_TOL)
      severity note;
    finish_report("tb_cordic_rot", errs, 4 * n);
    wait;

  end process p_check;

  p_timeout : process is
  begin

    wait for 5 ms;
    report "tb_cordic_rot: timed out"
      severity failure;

  end process p_timeout;

end architecture sim;
