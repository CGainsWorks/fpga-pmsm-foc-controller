-- -----------------------------------------------------------------------------
-- pi_axis
--
-- One axis of the current controller.  Derivation in
-- docs/05-current-loop-tuning.md.
--
--     e[k] = ref - meas
--     I[k] = I[k-1] + Ki e[k]        frozen while the output is already hard
--                                    against the limit in the same direction
--     u[k] = clamp(Kp e[k] + I[k], +/- Umax)
--
-- The integrator uses the current error, so it is a backward-Euler integrator
-- and its zero sits at z = Kp/(Kp+Ki).  That is the discrete image of the
-- continuous zero at s = -R/L which cancels the plant pole, and at the reference
-- tuning the two land within 6e-5 of each other.
--
-- Two things here are not cosmetic.
--
-- The integrator is held in the natural Q?.27 format of the Ki times e product
-- and only requantised on the way out.  At the reference tuning a one-LSB error
-- produces 296 accumulator counts, so the smallest error the ADC can resolve
-- still drives the integrator.  Storing the integrator in the Q2.13 output
-- format instead would need an error of 68 LSB before the state moved at all,
-- and the loop would sit with a permanent offset it had no way to remove.  This
-- is the most common way a fixed-point PI is got wrong.
--
-- The anti-windup is conditional integration rather than a clamp on the state
-- alone.  While the actuator is saturated the loop is open, and an integrator
-- that keeps accumulating during that time has to be unwound again before the
-- output can come off the limit, which shows up as a long overshoot after every
-- large transient.  Freezing costs one comparison and removes the effect
-- entirely; the state is additionally clamped to +/- Umax so that a change of
-- limit at run time cannot leave a stale value behind.
--
-- Latency two clocks.  The multiplies are isolated in the first stage so that
-- they map to a DSP slice with its input and output registers used.
-- -----------------------------------------------------------------------------

library ieee;
  use ieee.std_logic_1164.all;
  use ieee.numeric_std.all;

library work;
  use work.foc_pkg.all;

entity pi_axis is
  port (
    clk       : in    std_logic;
    rst       : in    std_logic;
    clr       : in    std_logic;                     -- zero the integrator
    in_valid  : in    std_logic;
    ref       : in    signed(C_W_DATA - 1 downto 0); -- Q2.13
    meas      : in    signed(C_W_DATA - 1 downto 0); -- Q2.13
    kp        : in    signed(C_W_DATA - 1 downto 0); -- Q3.12
    ki        : in    signed(C_W_DATA - 1 downto 0); -- Q1.14
    umax      : in    signed(C_W_DATA - 1 downto 0); -- Q2.13, non-negative
    out_valid : out   std_logic;
    u         : out   signed(C_W_DATA - 1 downto 0); -- Q2.13
    sat_out   : out   std_logic
  );
end entity pi_axis;

architecture rtl of pi_axis is

  constant C_W_PROD : natural := 2 * C_W_DATA; -- 32
  constant C_W_WIDE : natural := C_W_ACC + 2;  -- 34, no overflow

  signal vld1 : std_logic;
  signal vld2 : std_logic;

  signal e_r  : signed(C_W_DATA - 1 downto 0);
  signal p_r  : signed(C_W_PROD - 1 downto 0); -- Kp e, Q2.13
  signal ki_r : signed(C_W_PROD - 1 downto 0); -- Ki e, Q?.27

  signal acc : signed(C_W_ACC - 1 downto 0);

begin

  -- Stage one: the error and the two products.
  p_stage1 : process (clk) is

    variable e : signed(C_W_DATA downto 0);

  begin

    if rising_edge(clk) then
      e    := resize(ref, C_W_DATA + 1) - resize(meas, C_W_DATA + 1);
      e_r  <= sat_to(e, C_W_DATA);
      p_r  <= round_shr(kp * sat_to(e, C_W_DATA), C_Q_KP);
      ki_r <= ki * sat_to(e, C_W_DATA);
    end if;

  end process p_stage1;

  -- Stage two: accumulate, saturate, decide whether to keep the accumulation.
  p_stage2 : process (clk) is

    variable acc_next : signed(C_W_WIDE - 1 downto 0);
    variable i_term   : signed(C_W_WIDE - 1 downto 0);
    variable u_un     : signed(C_W_WIDE - 1 downto 0);
    variable umax_w   : signed(C_W_WIDE - 1 downto 0);
    variable acc_lim  : signed(C_W_WIDE - 1 downto 0);

  begin

    if rising_edge(clk) then
      if rst = '1' or clr = '1' then
        acc     <= (others => '0');
        u       <= (others => '0');
        sat_out <= '0';
      elsif vld1 = '1' then
        umax_w   := resize(umax, C_W_WIDE);
        acc_lim  := shift_left(umax_w, C_Q_KI);
        acc_next := resize(acc, C_W_WIDE) + resize(ki_r, C_W_WIDE);
        i_term   := round_shr(acc_next, C_Q_KI);
        u_un     := resize(p_r, C_W_WIDE) + i_term;

        if u_un > umax_w then
          u       <= umax;
          sat_out <= '1';
          if e_r > 0 then
            acc_next := resize(acc, C_W_WIDE);
          end if;
        elsif u_un < -umax_w then
          u       <= -umax;
          sat_out <= '1';
          if e_r < 0 then
            acc_next := resize(acc, C_W_WIDE);
          end if;
        else
          u       <= sat_to(u_un, C_W_DATA);
          sat_out <= '0';
        end if;

        if acc_next > acc_lim then
          acc <= resize(acc_lim, C_W_ACC);
        elsif acc_next < -acc_lim then
          acc <= resize(-acc_lim, C_W_ACC);
        else
          acc <= resize(acc_next, C_W_ACC);
        end if;
      end if;
    end if;

  end process p_stage2;

  p_valid : process (clk) is
  begin

    if rising_edge(clk) then
      if rst = '1' then
        vld1 <= '0';
        vld2 <= '0';
      else
        vld1 <= in_valid;
        vld2 <= vld1;
      end if;
    end if;

  end process p_valid;

  out_valid <= vld2;

end architecture rtl;
