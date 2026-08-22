-- -----------------------------------------------------------------------------
-- tb_clarke
--
-- The Clarke transform against the model, exactly, and against real arithmetic
-- within one LSB.  The vector set includes a full electrical turn of a balanced
-- one per unit set, which is what the block sees in service, and random pairs
-- well outside the balanced set, which is what it sees during a fault.
--
-- The comparison against real arithmetic uses the full Clarke expression, so a
-- wrong scaling factor fails it: the amplitude-invariant 2/3 is what puts a
-- balanced set of peak amplitude A on a space vector of magnitude A rather than
-- 1.5 A, and every per-unit limit downstream depends on that being right.
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

entity tb_clarke is
  generic (
    G_VECTORS : string := "sim/vectors/clarke.txt"
  );
end entity tb_clarke;

architecture sim of tb_clarke is

  constant C_PERIOD : time := 10 ns;

  signal clk       : std_logic                     := '0';
  signal rst       : std_logic                     := '1';
  signal in_valid  : std_logic                     := '0';
  signal ia        : signed(C_W_DATA - 1 downto 0) := (others => '0');
  signal ib        : signed(C_W_DATA - 1 downto 0) := (others => '0');
  signal out_valid : std_logic;
  signal ialpha    : signed(C_W_DATA - 1 downto 0);
  signal ibeta     : signed(C_W_DATA - 1 downto 0);
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

  u_dut : entity work.clarke
    port map (
      clk       => clk,
      rst       => rst,
      in_valid  => in_valid,
      ia        => ia,
      ib        => ib,
      out_valid => out_valid,
      ialpha    => ialpha,
      ibeta     => ibeta,
      sat_out   => sat_out
    );

  p_stim : process is

    file     fs : text;
    variable l  : line;
    variable ok : boolean;
    variable v  : integer;
    variable a  : integer;
    variable b  : integer;

  begin

    wait until rst = '0';
    wait until rising_edge(clk);
    file_open(fs, G_VECTORS, read_mode);

    loop

      next_data_line(fs, l, ok);
      exit when not ok;
      read(l, a);
      read(l, b);

      for k in 1 to 3 loop

        read(l, v);

      end loop;

      deallocate(l);
      ia       <= to_signed(a, C_W_DATA);
      ib       <= to_signed(b, C_W_DATA);
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
    variable a     : integer;
    variable b     : integer;
    variable eal   : integer;
    variable ebe   : integer;
    variable est   : integer;
    variable errs  : natural := 0;
    variable n     : natural := 0;
    variable beta  : real;
    variable worst : real    := 0.0;

  begin

    wait until rst = '0';
    file_open(fc, G_VECTORS, read_mode);

    loop

      next_data_line(fc, l, ok);
      exit when not ok;
      read(l, a);
      read(l, b);
      read(l, eal);
      read(l, ebe);
      read(l, est);
      deallocate(l);

      wait until rising_edge(clk) and out_valid = '1';

      check_int("ialpha", to_integer(ialpha), eal, errs);
      check_int("ibeta", to_integer(ibeta), ebe, errs);
      check_int("sat", to_int(sat_out), est, errs);

      -- Against real arithmetic, but only where the true value is inside the
      -- output format; outside it the block is required to saturate and there
      -- is nothing to compare with.
      beta := (real(a) + 2.0 * real(b)) / sqrt(3.0);

      if abs (beta) < 32767.0 then
        if abs (real(to_integer(ibeta)) - beta) > worst then
          worst := abs (real(to_integer(ibeta)) - beta);
        end if;
        check_true("ibeta within one LSB",
                   abs (real(to_integer(ibeta)) - beta) <= 1.0, errs);
      end if;

      n := n + 1;

    end loop;

    file_close(fc);
    report "tb_clarke: worst error against real arithmetic "
           & real'image(worst) & " LSB"
      severity note;
    finish_report("tb_clarke", errs, 4 * n);
    wait;

  end process p_check;

  p_timeout : process is
  begin

    wait for 5 ms;
    report "tb_clarke: timed out"
      severity failure;

  end process p_timeout;

end architecture sim;
