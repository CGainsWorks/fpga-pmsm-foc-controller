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
-- continuous zero at s = -R/L which cancels the plant pole and at the reference
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
-- alone.  While the actuator is saturated the loop is open and an integrator
-- that keeps accumulating during that time has to be unwound again before the
-- output can come off the limit, which shows up as a long overshoot after every
-- large transient.  Freezing costs one comparison and removes the effect
-- entirely; the state is additionally clamped to +/- Umax so that a change of
-- limit at run time cannot leave a stale value behind.
--
-- Pipelining and why the integrator still updates every clock.  Written
-- directly, one update is an add, a rounding add, a third add, a compare against
-- the limit, a select and a second compare and select for the state clamp, all
-- between one value of the integrator and the next.  The open place-and-route
-- flow in fpga/ put that at 70 MHz.  The arithmetic before and after the
-- integrator pipelines freely; the integrator itself is a recurrence and a
-- register inside it would change the controller.
--
-- So the decisions are moved out of the recurrence instead.  With
-- A = acc + Ki e + 2^13, the rounded integral term is floor(A / 2^14) and for
-- any integer n, floor(A / 2^14) >= n exactly when A >= n 2^14.  Hence
--
--     u > Umax   exactly when  acc >= (Umax - Kp e) 2^14 + 2^13 - Ki e
--     u < -Umax  exactly when  acc <  (-Umax - Kp e - 1) 2^14 + 2^13 - Ki e
--
-- and likewise acc + Ki e > lim exactly when acc > lim - Ki e.  Every right
-- hand side depends only on the current sample, so it is computed in the
-- stages before the integrator and the recurrence is left with one add and
-- four compares in parallel feeding one select.  The equivalence is exact
-- integer arithmetic, not an approximation, which is why tb_pi_axis still
-- compares against focmodel for equality with samples applied back to back.
--
-- Latency nine clocks.  Stages 1 to 5 form the error, the two products and the
-- thresholds, stage 6 is the integrator and stages 7 to 9 form the output.  CLR
-- zeroes the integrator and the output register; a sample already past stage 6
-- when it arrives still emerges, which the current loop never relies on because
-- it holds CLR across a disable rather than pulsing it mid-update.
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
  constant C_W_TH   : natural := C_W_ACC + 8;  -- 40, thresholds
  constant C_N      : natural := 9;            -- stages

  -- 2^13 in the low C_Q_KI bits of a threshold, as a bit pattern so that adding
  -- it to a value whose low bits are zero costs no carry chain.
  constant C_HALF_KI : std_logic_vector(C_Q_KI - 1 downto 0) :=
                                                                (C_Q_KI - 1 => '1', others => '0');

  signal vld : std_logic_vector(1 to C_N);

  -- Stage 1
  signal e1 : signed(C_W_DATA - 1 downto 0);

  -- Stage 2
  signal pm2  : signed(C_W_PROD - 1 downto 0);
  signal ki2  : signed(C_W_PROD - 1 downto 0);
  signal epo2 : std_logic;
  signal eng2 : std_logic;

  -- Stage 3
  signal p3   : signed(C_W_PROD - 1 downto 0);
  signal ki3  : signed(C_W_PROD - 1 downto 0);
  signal epo3 : std_logic;
  signal eng3 : std_logic;

  -- Stage 4
  signal dhi4  : signed(C_W_WIDE - 1 downto 0);
  signal dlo4  : signed(C_W_WIDE - 1 downto 0);
  signal lim4  : signed(C_W_WIDE - 1 downto 0);
  signal nlim4 : signed(C_W_WIDE - 1 downto 0);
  signal umx4  : signed(C_W_DATA - 1 downto 0);
  signal p4    : signed(C_W_PROD - 1 downto 0);
  signal ki4   : signed(C_W_PROD - 1 downto 0);
  signal epo4  : std_logic;
  signal eng4  : std_logic;

  -- Stage 5
  signal th_hi5  : signed(C_W_TH - 1 downto 0);
  signal th_lo5  : signed(C_W_TH - 1 downto 0);
  signal lim_hi5 : signed(C_W_TH - 1 downto 0);
  signal lim_lo5 : signed(C_W_TH - 1 downto 0);
  signal lim5    : signed(C_W_WIDE - 1 downto 0);
  signal umx5    : signed(C_W_DATA - 1 downto 0);
  signal p5      : signed(C_W_PROD - 1 downto 0);
  signal ki5     : signed(C_W_PROD - 1 downto 0);
  signal epo5    : std_logic;
  signal eng5    : std_logic;

  -- Stage 6, the integrator
  signal acc  : signed(C_W_ACC - 1 downto 0);
  signal sum6 : signed(C_W_WIDE - 1 downto 0);
  signal hi6  : std_logic;
  signal lo6  : std_logic;
  signal umx6 : signed(C_W_DATA - 1 downto 0);
  signal p6   : signed(C_W_PROD - 1 downto 0);

  -- Stages 7 and 8
  signal it7  : signed(C_W_WIDE - 1 downto 0);
  signal hi7  : std_logic;
  signal lo7  : std_logic;
  signal umx7 : signed(C_W_DATA - 1 downto 0);
  signal p7   : signed(C_W_PROD - 1 downto 0);
  signal uu8  : signed(C_W_WIDE - 1 downto 0);
  signal hi8  : std_logic;
  signal lo8  : std_logic;
  signal umx8 : signed(C_W_DATA - 1 downto 0);

begin

  -- Stages 1 to 5: the error, the products and the thresholds.
  p_front : process (clk) is

    variable e : signed(C_W_DATA downto 0);

  begin

    if rising_edge(clk) then
      -- 1: the error, saturated to the data width.
      if in_valid = '1' then
        e  := resize(ref, C_W_DATA + 1) - resize(meas, C_W_DATA + 1);
        e1 <= sat_to(e, C_W_DATA);
      end if;

      -- 2: the two products and the sign of the error.
      if vld(1) = '1' then
        pm2 <= kp * e1;
        ki2 <= ki * e1;
        if e1 > 0 then
          epo2 <= '1';
        else
          epo2 <= '0';
        end if;
        if e1 < 0 then
          eng2 <= '1';
        else
          eng2 <= '0';
        end if;
      end if;

      -- 3: the proportional term back to Q2.13.
      if vld(2) = '1' then
        p3   <= round_shr(pm2, C_Q_KP);
        ki3  <= ki2;
        epo3 <= epo2;
        eng3 <= eng2;
      end if;

      -- 4: Umax - Kp e, -Umax - Kp e - 1 (not Umax is -Umax - 1) and the
      -- integrator limit.  The limit is taken here, with the sample, so that the
      -- whole update sees one value of it.
      if vld(3) = '1' then
        dhi4  <= resize(umax, C_W_WIDE) - resize(p3, C_W_WIDE);
        dlo4  <= resize(not umax, C_W_WIDE) - resize(p3, C_W_WIDE);
        lim4  <= shift_left(resize(umax, C_W_WIDE), C_Q_KI);
        nlim4 <= -shift_left(resize(umax, C_W_WIDE), C_Q_KI);
        umx4  <= umax;
        p4    <= p3;
        ki4   <= ki3;
        epo4  <= epo3;
        eng4  <= eng3;
      end if;

      -- 5: the four thresholds the integrator will compare against.
      if vld(4) = '1' then
        th_hi5  <= signed(std_logic_vector(resize(dhi4, C_W_TH - C_Q_KI)) & C_HALF_KI)
                   - resize(ki4, C_W_TH);
        th_lo5  <= signed(std_logic_vector(resize(dlo4, C_W_TH - C_Q_KI)) & C_HALF_KI)
                   - resize(ki4, C_W_TH);
        lim_hi5 <= resize(lim4, C_W_TH) - resize(ki4, C_W_TH);
        lim_lo5 <= resize(nlim4, C_W_TH) - resize(ki4, C_W_TH);
        lim5    <= lim4;
        umx5    <= umx4;
        p5      <= p4;
        ki5     <= ki4;
        epo5    <= epo4;
        eng5    <= eng4;
      end if;
    end if;

  end process p_front;

  -- Stage 6: the integrator.  One add and four compares, all against acc and
  -- all in parallel, then one select.
  p_integrate : process (clk) is

    variable a      : signed(C_W_TH - 1 downto 0);
    variable hi     : boolean;
    variable lo     : boolean;
    variable freeze : boolean;

  begin

    if rising_edge(clk) then
      if rst = '1' or clr = '1' then
        acc <= (others => '0');
      elsif vld(5) = '1' then
        a      := resize(acc, C_W_TH);
        hi     := a >= th_hi5;
        lo     := a < th_lo5;
        freeze := (hi and epo5 = '1') or ((not hi) and lo and eng5 = '1');

        if freeze then
          if a > resize(lim5, C_W_TH) then
            acc <= resize(lim5, C_W_ACC);
          elsif a < -resize(lim5, C_W_TH) then
            acc <= resize(-lim5, C_W_ACC);
          end if;
        else
          if a > lim_hi5 then
            acc <= resize(lim5, C_W_ACC);
          elsif a < lim_lo5 then
            acc <= resize(-lim5, C_W_ACC);
          else
            acc <= acc + resize(ki5, C_W_ACC);
          end if;
        end if;

        sum6 <= resize(acc, C_W_WIDE) + resize(ki5, C_W_WIDE);
        if hi then
          hi6 <= '1';
        else
          hi6 <= '0';
        end if;
        if lo then
          lo6 <= '1';
        else
          lo6 <= '0';
        end if;
        umx6 <= umx5;
        p6   <= p5;
      end if;
    end if;

  end process p_integrate;

  -- Stages 7 to 9: the output, from the unfrozen sum exactly as before.
  p_output : process (clk) is
  begin

    if rising_edge(clk) then
      if vld(6) = '1' then
        it7  <= round_shr(sum6, C_Q_KI);
        hi7  <= hi6;
        lo7  <= lo6;
        umx7 <= umx6;
        p7   <= p6;
      end if;

      if vld(7) = '1' then
        uu8  <= resize(p7, C_W_WIDE) + it7;
        hi8  <= hi7;
        lo8  <= lo7;
        umx8 <= umx7;
      end if;

      if rst = '1' or clr = '1' then
        u       <= (others => '0');
        sat_out <= '0';
      elsif vld(8) = '1' then
        if hi8 = '1' then
          u       <= umx8;
          sat_out <= '1';
        elsif lo8 = '1' then
          u       <= -umx8;
          sat_out <= '1';
        else
          u       <= sat_to(uu8, C_W_DATA);
          sat_out <= '0';
        end if;
      end if;
    end if;

  end process p_output;

  p_valid : process (clk) is
  begin

    if rising_edge(clk) then
      if rst = '1' then
        vld <= (others => '0');
      else
        vld <= in_valid & vld(1 to C_N - 1);
      end if;
    end if;

  end process p_valid;

  out_valid <= vld(C_N);

end architecture rtl;
