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
-- cycles, and the two computations agree exactly.  What it costs is three
-- comparators instead of a sector decode, two dwell-time multiplies and a
-- per-sector duty table.
--
-- Why it is worth doing at all.  Without injection a balanced set of peak
-- amplitude A needs a duty excursion of A/sqrt(3) about the half-rail, so the
-- linear range ends at A = sqrt(3)/2 = 0.866.  With it the peak of v + v_off is
-- (sqrt(3)/2) A, so the excursion is A/2 and the linear range ends at A = 1.
-- That is a factor 2/sqrt(3) = 1.1547, or 15.47 per cent more fundamental
-- voltage from the same DC link, which on a motor drive is 15.47 per cent more
-- speed before field weakening starts and roughly 13 per cent lower conduction
-- loss at a given power.
--
-- Duty, with V_base = V_dc/sqrt(3):  d = 1/2 + (v + v_off)/sqrt(3), so the
-- linear limit sits at exactly 1.0 per unit and d covers exactly [0, 1].
--
-- Latency two clocks.
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
  constant C_W_MUL : natural := C_W_T + 16;          -- 36
  constant C_W_CMP : natural := C_W_T + G_PWM_BITS + 1;
  constant C_HALF  : integer := 2 ** (C_Q_DATA - 1); -- 0.5 in Q2.13
  constant C_UNITY : integer := 2 ** C_Q_DATA;       -- 1.0 in Q2.13

  type v3_t is array (0 to 2) of signed(C_W_V - 1 downto 0);

  signal v_r   : v3_t;
  signal off_r : signed(C_W_OFF - 1 downto 0);
  signal top_r : unsigned(G_PWM_BITS - 1 downto 0);
  signal vld1  : std_logic;
  signal vld2  : std_logic;

begin

  -- Stage one: inverse Clarke and the injection offset.
  p_stage1 : process (clk) is

    variable prod : signed(C_W_DATA + 16 - 1 downto 0);
    variable half : signed(C_W_DATA + 16 - 1 downto 0);
    variable vmax : signed(C_W_V - 1 downto 0);
    variable vmin : signed(C_W_V - 1 downto 0);
    variable v    : v3_t;

  begin

    if rising_edge(clk) then
      prod := vbeta * to_signed(C_SQRT3_2, 16);
      half := shift_left(resize(-valpha, C_W_DATA + 16), C_Q_TRIG - 1);

      v(0) := resize(valpha, C_W_V);
      v(1) := sat_to(round_shr(half + prod, C_Q_TRIG), C_W_V);
      v(2) := sat_to(round_shr(half - prod, C_Q_TRIG), C_W_V);

      vmax := v(0);
      vmin := v(0);

      for k in 1 to 2 loop

        if v(k) > vmax then
          vmax := v(k);
        end if;
        if v(k) < vmin then
          vmin := v(k);
        end if;

      end loop;

      v_r   <= v;
      off_r <= -round_shr(resize(vmax, C_W_OFF) + resize(vmin, C_W_OFF), 1);
      top_r <= pwm_top;
    end if;

  end process p_stage1;

  -- Stage two: duty and compare value.
  p_stage2 : process (clk) is

    variable t    : signed(C_W_T - 1 downto 0);
    variable d    : signed(C_W_MUL - 1 downto 0);
    variable dn   : signed(C_W_T - 1 downto 0);
    variable c    : signed(C_W_CMP - 1 downto 0);
    variable tops : signed(G_PWM_BITS downto 0);
    variable ovf  : std_logic;
    variable cmp  : unsigned(G_PWM_BITS - 1 downto 0);

  begin

    if rising_edge(clk) then
      tops := signed(resize(top_r, G_PWM_BITS + 1));
      ovf  := '0';

      for k in 0 to 2 loop

        t := resize(v_r(k), C_W_T) + resize(off_r, C_W_T);
        d := round_shr(t * to_signed(C_INV_SQRT3, 16), C_Q_TRIG)
             + to_signed(C_HALF, C_W_MUL);

        if d < 0 or d > to_signed(C_UNITY, C_W_MUL) then
          ovf := '1';
        end if;

        dn := sat_to(d, C_W_T);
        c  := round_shr(dn * tops, C_Q_DATA);

        if c < 0 then
          cmp := (others => '0');
        elsif c > resize(tops, C_W_CMP) then
          cmp := top_r;
        else
          cmp := unsigned(resize(c, G_PWM_BITS));
        end if;

        case k is

          when 0 =>

            cmp_a <= cmp;

          when 1 =>

            cmp_b <= cmp;

          when others =>

            cmp_c <= cmp;

        end case;

      end loop;

      over_mod <= ovf;
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
