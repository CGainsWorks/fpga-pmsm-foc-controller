-- -----------------------------------------------------------------------------
-- svm
--
-- Stationary-frame voltage vector to three compare values, by inverse Clarke and
-- min-max common-mode injection.  Full derivation in docs/06-svpwm-and-pwm.md.
--
-- Inverse Clarke, the transpose of the amplitude-invariant forward map:
--
--     v_a = v_alpha
--     v_b = -v_alpha/2 + (sqrt(3)/2) v_beta
--     v_c = -v_alpha/2 - (sqrt(3)/2) v_beta
--
-- Injection.  Adding the same offset to all three legs changes no line-to-line
-- voltage, so in a machine whose star point is isolated it changes no current
-- and is free.  Spending it on
--
--     v_off = -(max(v_a,v_b,v_c) + min(v_a,v_b,v_c)) / 2
--
-- centres the three references in the rail.  This is not an approximation to
-- space-vector modulation, it is the same modulator: centring the references is
-- what splitting the zero-vector time equally between V0 and V7 does to the duty
-- cycles and the two computations agree exactly.  What it costs is three
-- comparators instead of a sector decode, two dwell-time multiplies and a
-- per-sector duty table.
--
-- Why it is worth doing at all.  Without injection a balanced set of peak
-- amplitude A needs a duty excursion of A/sqrt(3) about the half-rail, so the
-- linear range ends at A = sqrt(3)/2 = 0.866.  With it the peak of v + v_off is
-- (sqrt(3)/2) A, so the excursion is A/2 and the linear range ends at A = 1.
-- That is a factor 2/sqrt(3) = 1.1547 or 15.47 per cent more fundamental
-- voltage from the same DC link, which on a motor drive is 15.47 per cent more
-- speed before field weakening starts and roughly 13 per cent lower conduction
-- loss at a given power.
--
-- Duty, with V_base = V_dc/sqrt(3):  d = 1/2 + (v + v_off)/sqrt(3), so the
-- linear limit sits at exactly 1.0 per unit and d covers exactly [0, 1].
--
-- Pipelining.  Every stage below does one multiply, or one carry chain, or one
-- compare and select, and nothing more.  The arithmetic is the same operations
-- in the same order as the original two-stage form, so the result is bit for bit
-- what focmodel computes; only where the registers sit has changed.  It was the
-- open place-and-route flow in fpga/ that showed the two-stage form, with two
-- multiplies, four adds and three compares between registers, closing at under
-- 40 MHz.  Latency 13 clocks.
-- -----------------------------------------------------------------------------

library ieee;
  use ieee.std_logic_1164.all;
  use ieee.numeric_std.all;

library work;
  use work.foc_pkg.all;

entity svm is
  generic (
    G_PWM_BITS : natural := 13
  );
  port (
    clk       : in    std_logic;
    rst       : in    std_logic;
    in_valid  : in    std_logic;
    valpha    : in    signed(C_W_DATA - 1 downto 0); -- Q2.13
    vbeta     : in    signed(C_W_DATA - 1 downto 0); -- Q2.13
    pwm_top   : in    unsigned(G_PWM_BITS - 1 downto 0);
    out_valid : out   std_logic;
    cmp_a     : out   unsigned(G_PWM_BITS - 1 downto 0);
    cmp_b     : out   unsigned(G_PWM_BITS - 1 downto 0);
    cmp_c     : out   unsigned(G_PWM_BITS - 1 downto 0);
    over_mod  : out   std_logic
  );
end entity svm;

architecture rtl of svm is

  -- -v_alpha/2 -/+ (sqrt(3)/2) v_beta reaches 1.366 times full scale, so the
  -- phase references are carried two bits wider than the input format.  They are
  -- never saturated here: at the modulation limit they are 1.0 per unit and the
  -- headroom exists only so that a nonsense input cannot wrap into a valid duty.
  constant C_W_V   : natural := C_W_DATA + 2;        -- 18
  constant C_W_OFF : natural := C_W_V + 1;           -- 19
  constant C_W_T   : natural := C_W_V + 2;           -- 20
  constant C_W_P   : natural := C_W_DATA + 16;       -- 32
  constant C_W_MUL : natural := C_W_T + 16;          -- 36
  constant C_W_CMP : natural := C_W_T + G_PWM_BITS + 1;
  constant C_HALF  : integer := 2 ** (C_Q_DATA - 1); -- 0.5 in Q2.13
  constant C_UNITY : integer := 2 ** C_Q_DATA;       -- 1.0 in Q2.13
  constant C_N     : natural := 13;                  -- stages

  type v3_t is array (0 to 2) of signed(C_W_V - 1 downto 0);

  type t3_t is array (0 to 2) of signed(C_W_T - 1 downto 0);

  type m3_t is array (0 to 2) of signed(C_W_MUL - 1 downto 0);

  type c3_t is array (0 to 2) of signed(C_W_CMP - 1 downto 0);

  type cmp3_t is array (0 to 2) of unsigned(G_PWM_BITS - 1 downto 0);

  type top_t is array (1 to C_N - 1) of unsigned(G_PWM_BITS - 1 downto 0);

  signal vld : std_logic_vector(1 to C_N);

  -- The carrier top travels with the sample, so that the compare values are
  -- scaled by the top that was current when the sample arrived, exactly as in
  -- the unpipelined form.
  signal top : top_t;

  signal prod_r : signed(C_W_P - 1 downto 0);
  signal half_r : signed(C_W_P - 1 downto 0);
  signal va_r   : signed(C_W_V - 1 downto 0);
  signal sp_r   : signed(C_W_P - 1 downto 0);
  signal sm_r   : signed(C_W_P - 1 downto 0);
  signal va2_r  : signed(C_W_V - 1 downto 0);
  signal v3_r   : v3_t;
  signal v4_r   : v3_t;
  signal vmax_r : signed(C_W_V - 1 downto 0);
  signal vmin_r : signed(C_W_V - 1 downto 0);
  signal v5_r   : v3_t;
  signal sum_r  : signed(C_W_OFF - 1 downto 0);
  signal v6_r   : v3_t;
  signal off_r  : signed(C_W_OFF - 1 downto 0);
  signal t_r    : t3_t;
  signal m_r    : m3_t;
  signal d_r    : m3_t;
  signal dn_r   : t3_t;
  signal ovf10  : std_logic;
  signal cm_r   : c3_t;
  signal ovf11  : std_logic;
  signal c_r    : c3_t;
  signal ovf12  : std_logic;

  -- The compare values are registered internally with a power-up value rather
  -- than driven straight from the clock-enabled process.  The modulator holds
  -- them across the whole PWM period and reads them at every trough, including
  -- the troughs before the loop has produced its first result, so they have to
  -- mean something from configuration onwards.  A half-rail duty would look
  -- more neutral than zero and is not: zero is all three legs on the same rail,
  -- which is a zero vector and applies no voltage to the machine at all.
  signal cmp_r  : cmp3_t    := (others => (others => '0'));
  signal over_r : std_logic := '0';

begin

  -- Inverse Clarke and the injection offset, stages 1 to 6.
  p_offset : process (clk) is

    variable vmax : signed(C_W_V - 1 downto 0);
    variable vmin : signed(C_W_V - 1 downto 0);

  begin

    if rising_edge(clk) then
      -- 1: the one multiply and the halved alpha.
      if in_valid = '1' then
        prod_r <= vbeta * to_signed(C_SQRT3_2, 16);
        half_r <= shift_left(resize(-valpha, C_W_P), C_Q_TRIG - 1);
        va_r   <= resize(valpha, C_W_V);
        top(1) <= pwm_top;
      end if;

      -- 2: the two sums.
      if vld(1) = '1' then
        sp_r   <= half_r + prod_r;
        sm_r   <= half_r - prod_r;
        va2_r  <= va_r;
        top(2) <= top(1);
      end if;

      -- 3: back to Q2.13 with rounding, saturated to the reference width.
      if vld(2) = '1' then
        v3_r(0) <= va2_r;
        v3_r(1) <= sat_to(round_shr(sp_r, C_Q_TRIG), C_W_V);
        v3_r(2) <= sat_to(round_shr(sm_r, C_Q_TRIG), C_W_V);
        top(3)  <= top(2);
      end if;

      -- 4: the largest and smallest of the three.
      if vld(3) = '1' then
        vmax := v3_r(0);
        vmin := v3_r(0);

        for k in 1 to 2 loop

          if v3_r(k) > vmax then
            vmax := v3_r(k);
          end if;
          if v3_r(k) < vmin then
            vmin := v3_r(k);
          end if;

        end loop;

        vmax_r <= vmax;
        vmin_r <= vmin;
        v4_r   <= v3_r;
        top(4) <= top(3);
      end if;

      -- 5 and 6: v_off = -(max + min) / 2, rounded.
      if vld(4) = '1' then
        sum_r  <= resize(vmax_r, C_W_OFF) + resize(vmin_r, C_W_OFF);
        v5_r   <= v4_r;
        top(5) <= top(4);
      end if;

      if vld(5) = '1' then
        off_r  <= -round_shr(sum_r, 1);
        v6_r   <= v5_r;
        top(6) <= top(5);
      end if;
    end if;

  end process p_offset;

  -- Duty and compare value, stages 7 to 13, the same for each leg.
  p_duty : process (clk) is

    variable tops : signed(G_PWM_BITS downto 0);
    variable ovf  : std_logic;

  begin

    if rising_edge(clk) then
      -- 7: add the offset.
      if vld(6) = '1' then

        for k in 0 to 2 loop

          t_r(k) <= resize(v6_r(k), C_W_T) + resize(off_r, C_W_T);

        end loop;

        top(7) <= top(6);
      end if;

      -- 8: scale by 1/sqrt(3).
      if vld(7) = '1' then

        for k in 0 to 2 loop

          m_r(k) <= t_r(k) * to_signed(C_INV_SQRT3, 16);

        end loop;

        top(8) <= top(7);
      end if;

      -- 9: round and add the half-rail offset.
      if vld(8) = '1' then

        for k in 0 to 2 loop

          d_r(k) <= round_shr(m_r(k), C_Q_TRIG) + to_signed(C_HALF, C_W_MUL);

        end loop;

        top(9) <= top(8);
      end if;

      -- 10: the over-modulation flag and the duty saturated to [0, 1].
      if vld(9) = '1' then
        ovf := '0';

        for k in 0 to 2 loop

          if d_r(k) < 0 or d_r(k) > to_signed(C_UNITY, C_W_MUL) then
            ovf := '1';
          end if;
          dn_r(k) <= sat_to(d_r(k), C_W_T);

        end loop;

        ovf10   <= ovf;
        top(10) <= top(9);
      end if;

      -- 11: duty times the carrier top.
      if vld(10) = '1' then
        tops := signed(resize(top(10), G_PWM_BITS + 1));

        for k in 0 to 2 loop

          cm_r(k) <= dn_r(k) * tops;

        end loop;

        ovf11   <= ovf10;
        top(11) <= top(10);
      end if;

      -- 12: round back to clocks.
      if vld(11) = '1' then

        for k in 0 to 2 loop

          c_r(k) <= round_shr(cm_r(k), C_Q_DATA);

        end loop;

        ovf12   <= ovf11;
        top(12) <= top(11);
      end if;

      -- 13: clamp to the carrier.
      if vld(12) = '1' then
        tops := signed(resize(top(12), G_PWM_BITS + 1));

        for k in 0 to 2 loop

          if c_r(k) < 0 then
            cmp_r(k) <= (others => '0');
          elsif c_r(k) > resize(tops, C_W_CMP) then
            cmp_r(k) <= top(12);
          else
            cmp_r(k) <= unsigned(resize(c_r(k), G_PWM_BITS));
          end if;

        end loop;

        over_r <= ovf12;
      end if;
    end if;

  end process p_duty;

  cmp_a    <= cmp_r(0);
  cmp_b    <= cmp_r(1);
  cmp_c    <= cmp_r(2);
  over_mod <= over_r;

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
