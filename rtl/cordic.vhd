-- -----------------------------------------------------------------------------
-- cordic
--
-- Pipelined circular CORDIC, one entity covering both modes.  Full derivation in
-- docs/03-cordic.md; the short version is that
--
--     x' = x - s 2**-i y        [x']         [ 1      -s 2**-i ] [x]
--     y' = y + s 2**-i x   i.e. [  ] = sqrt(1+4**-i) R(s a_i)   [ ]
--     z' = z - s a_i            [y']                            [y]
--
-- with a_i = atan(2**-i) and s in {-1,+1}.  Each step is a rotation by a fixed
-- angle whose sign is data dependent and a scaling by sqrt(1+4**-i) which is
-- not, so after n steps the gain is the constant K = prod sqrt(1+4**-i) whatever
-- the data did and one constant multiply removes it.  There is no multiplier in
-- the iteration itself: 2**-i is a wired shift.
--
--   rotate_mode  s = sign(z), drives z to zero, so the vector on (x_in, y_in) is
--                rotated by ang_in.  ang_out carries the unresolved residual,
--                which the testbench checks against the derived bound.
--   vector_mode  s = -sign(y), drives y to zero, so x_out is the magnitude of
--                (x_in, y_in) and ang_out is its argument.
--
-- Latency C_N_STAGES + 2 clocks, one result per clock.  The design instantiates
-- three of these and uses each once per 2500 clocks, so a folded single-stage
-- version would be far smaller; see docs/10-limitations-and-roadmap.md.  The
-- pipelined form is kept because it is what lets one instance serve many axes.
--
-- Interfaces are Q2.13 and BAM16 in both directions.  Each instance undoes its
-- own gain, so K never appears in any system-level equation.
--
-- Every pipeline register is clock-enabled by the valid bit travelling with it,
-- so a stage with nothing in it holds instead of shifting.  That is what a
-- flip-flop's enable input is for and it costs nothing on an FPGA.  At one
-- control update per 5000 clocks it is the difference between 54 stages
-- toggling continuously and toggling for 66 clocks in 5000, which is most of
-- the dynamic power of the block.  It also makes the design cheap to simulate,
-- which is what lets the closed-loop testbench afford 27 ms of machine.
-- -----------------------------------------------------------------------------

library ieee;
  use ieee.std_logic_1164.all;
  use ieee.numeric_std.all;

library work;
  use work.foc_pkg.all;
  use work.cordic_atan_pkg.all;

entity cordic is
  generic (
    G_MODE : cordic_mode_t := rotate_mode;
    -- Multiply the result by 1/K.  Turning it off saves one DSP slice and hands
    -- the caller a result scaled by 1.64676026.
    G_COMPENSATE : boolean := true
  );
  port (
    clk       : in    std_logic;
    rst       : in    std_logic;                     -- synchronous, high
    in_valid  : in    std_logic;
    x_in      : in    signed(C_W_DATA - 1 downto 0); -- Q2.13
    y_in      : in    signed(C_W_DATA - 1 downto 0); -- Q2.13
    ang_in    : in    signed(C_W_ANG - 1 downto 0);  -- BAM16, rotate only
    out_valid : out   std_logic;
    x_out     : out   signed(C_W_DATA - 1 downto 0); -- Q2.13
    y_out     : out   signed(C_W_DATA - 1 downto 0); -- Q2.13
    ang_out   : out   signed(C_W_ANG - 1 downto 0);  -- BAM16
    sat_out   : out   std_logic
  );
end entity cordic;

architecture rtl of cordic is

  constant C_HALF_PI : signed(C_W_Z - 1 downto 0) :=
                                                     to_signed(2 ** (C_W_Z - 2), C_W_Z);

  constant C_W_PROD : natural := C_W_XY + C_W_KINV;

  type xy_array_t is array (0 to C_N_STAGES) of signed(C_W_XY - 1 downto 0);

  type z_array_t is array (0 to C_N_STAGES) of signed(C_W_Z - 1 downto 0);

  signal x : xy_array_t := (others => (others => '0'));
  signal y : xy_array_t := (others => (others => '0'));
  signal z : z_array_t  := (others => (others => '0'));

  -- The output register, kept internal so that its power-up value sits on the
  -- flip-flop rather than on the port, which is where a synthesiser will honour
  -- it.  It matters because the stage is clock-enabled: until the first
  -- transaction arrives nothing drives it and anything reading it would
  -- otherwise see an undefined value for as long as the design sits waiting to
  -- be configured.
  signal x_r   : signed(C_W_DATA - 1 downto 0) := (others => '0');
  signal y_r   : signed(C_W_DATA - 1 downto 0) := (others => '0');
  signal ang_r : signed(C_W_ANG - 1 downto 0)  := (others => '0');
  signal sat_r : std_logic                     := '0';

  -- One bit for the input register, C_N_STAGES for the iterations, one for the
  -- output register.
  signal vld : std_logic_vector(C_N_STAGES + 1 downto 0);

begin

  -- ---------------------------------------------------------------------------
  -- Input register and quadrant fold.
  --
  -- The iteration only reaches 1.7433 rad, which covers a quadrant but not the
  -- circle, so the residual angle is first brought inside +/- pi/2 by a quarter
  -- turn.  A quarter turn is an exchange and a negation of the two coordinates:
  -- it is exact, it has unit gain and it costs no arithmetic at all, which is
  -- why the fold is done here rather than by adding more iterations.
  -- ---------------------------------------------------------------------------
  p_input : process (clk) is

    variable xi : signed(C_W_XY - 1 downto 0);
    variable yi : signed(C_W_XY - 1 downto 0);
    variable zi : signed(C_W_Z - 1 downto 0);

  begin

    if rising_edge(clk) and in_valid = '1' then
      xi := shift_left(resize(x_in, C_W_XY), C_GUARD);
      yi := shift_left(resize(y_in, C_W_XY), C_GUARD);

      if G_MODE = rotate_mode then
        zi := shift_left(resize(ang_in, C_W_Z), C_Z_SHIFT);
        if zi > C_HALF_PI then
          x(0) <= -yi;
          y(0) <= xi;
          z(0) <= zi - C_HALF_PI;
        elsif zi < -C_HALF_PI then
          x(0) <= yi;
          y(0) <= -xi;
          z(0) <= zi + C_HALF_PI;
        else
          x(0) <= xi;
          y(0) <= yi;
          z(0) <= zi;
        end if;
      else
        -- Vectoring converges only for x >= 0, the right half plane.  The fold
        -- moves the other half in and seeds the accumulator with the quarter
        -- turn it took out, so the reported argument is still that of the
        -- original vector.
        if xi < 0 then
          if yi >= 0 then
            x(0) <= yi;
            y(0) <= -xi;
            z(0) <= C_HALF_PI;
          else
            x(0) <= -yi;
            y(0) <= xi;
            z(0) <= -C_HALF_PI;
          end if;
        else
          x(0) <= xi;
          y(0) <= yi;
          z(0) <= (others => '0');
        end if;
      end if;
    end if;

  end process p_input;

  -- ---------------------------------------------------------------------------
  -- The iteration.  Shifts truncate; six guard fraction bits hold the total
  -- accumulated truncation below a quarter of an output LSB, which is cheaper
  -- than rounding every stage.
  -- ---------------------------------------------------------------------------

  g_stage : for i in 0 to C_N_STAGES - 1 generate

    p_stage : process (clk) is

      variable neg : boolean;
      variable xs  : signed(C_W_XY - 1 downto 0);
      variable ys  : signed(C_W_XY - 1 downto 0);

    begin

      if rising_edge(clk) and vld(i) = '1' then
        if G_MODE = rotate_mode then
          neg := z(i) < 0;
        else
          neg := y(i) >= 0;
        end if;

        xs := shift_right(x(i), i);
        ys := shift_right(y(i), i);

        if neg then
          x(i + 1) <= x(i) + ys;
          y(i + 1) <= y(i) - xs;
          z(i + 1) <= z(i) + C_ATAN(i);
        else
          x(i + 1) <= x(i) - ys;
          y(i + 1) <= y(i) + xs;
          z(i + 1) <= z(i) - C_ATAN(i);
        end if;
      end if;

    end process p_stage;

  end generate g_stage;

  -- ---------------------------------------------------------------------------
  -- Output register: undo the processing gain, requantise to Q2.13 with rounding
  -- and saturate.  The angle is reduced modulo one turn, which is exact.
  -- ---------------------------------------------------------------------------
  p_output : process (clk) is

    variable rx : signed(C_W_PROD - 1 downto 0);
    variable ry : signed(C_W_PROD - 1 downto 0);

  begin

    if rising_edge(clk) and vld(C_N_STAGES) = '1' then
      if G_COMPENSATE then
        rx := round_shr(x(C_N_STAGES) * to_signed(C_KINV, C_W_KINV),
                        C_Q_KINV + C_GUARD);
        ry := round_shr(y(C_N_STAGES) * to_signed(C_KINV, C_W_KINV),
                        C_Q_KINV + C_GUARD);
      else
        rx := round_shr(resize(x(C_N_STAGES), C_W_PROD), C_GUARD);
        ry := round_shr(resize(y(C_N_STAGES), C_W_PROD), C_GUARD);
      end if;

      x_r   <= sat_to(rx, C_W_DATA);
      y_r   <= sat_to(ry, C_W_DATA);
      ang_r <= wrap_to(round_shr(z(C_N_STAGES), C_Z_SHIFT), C_W_ANG);

      if overflows(rx, C_W_DATA) or overflows(ry, C_W_DATA) then
        sat_r <= '1';
      else
        sat_r <= '0';
      end if;
    end if;

  end process p_output;

  -- ---------------------------------------------------------------------------
  -- Only the valid chain is reset.  The datapath registers are left free
  -- running: their contents are meaningless until a valid arrives with them and
  -- not resetting 1200 flip-flops keeps them out of the global reset fan-out.
  -- They are given a power-up value in their declaration, which every FPGA this
  -- would target honours from the configuration bitstream, so simulation and
  -- hardware agree on what is in them before the first transaction.
  -- ---------------------------------------------------------------------------
  p_valid : process (clk) is
  begin

    if rising_edge(clk) then
      if rst = '1' then
        vld <= (others => '0');
      else
        vld <= vld(vld'high - 1 downto 0) & in_valid;
      end if;
    end if;

  end process p_valid;

  x_out     <= x_r;
  y_out     <= y_r;
  ang_out   <= ang_r;
  sat_out   <= sat_r;
  out_valid <= vld(vld'high);

  -- Simulation-only guard.  In rotation mode the residual is monotonically
  -- driven towards zero and can never leave the folded range, so a value outside
  -- it means the fold or the table is wrong rather than that the input was.
  -- pragma translate_off
  p_assert : process (clk) is
  begin

    if rising_edge(clk) then
      if G_MODE = rotate_mode and vld(0) = '1' then
        assert abs (z(0)) <= C_HALF_PI
          report "cordic: residual angle outside the convergence region"
          severity failure;
      end if;
    end if;

  end process p_assert;

-- pragma translate_on

end architecture rtl;
