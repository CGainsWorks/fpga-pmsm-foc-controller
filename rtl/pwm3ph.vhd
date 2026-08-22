-- -----------------------------------------------------------------------------
-- pwm3ph
--
-- Three-phase centre-aligned PWM with complementary outputs and dead time.
-- Derivation and the numbers behind every choice are in docs/06-svpwm-and-pwm.md.
--
-- Carrier.  An up-down counter running 0, 1, ... top, top-1, ... 1 and back to 0
-- gives a period of 2*top clocks.  At 100 MHz with top = 2500 that is 20 kHz.
--
-- Compare rule.  The leg is high while cnt < cmp.  Counting the states of one
-- period, cnt takes the values 1..top on the rising ramp and top-1..0 on the
-- falling one, so the high time is exactly 2*cmp - 1 clocks for cmp >= 1 and
-- zero for cmp = 0.  The half-clock shortfall is identical on all three legs, so
-- it is a common-mode term: it cancels in every line-to-line voltage and a
-- machine with an isolated star point cannot see it at all.
--
-- Sampling.  The strobe is issued at the trough, where every leg is on the same
-- side of its comparator and the inverter is applying a zero vector.  Between
-- switching instants the phase current is a straight line, and a centre-aligned
-- carrier makes that line symmetric about the trough, so the instantaneous
-- current at the trough is equal to the mean current over the period.  The
-- switching ripple is therefore not filtered out, it is not sampled in the first
-- place, which is why no anti-alias filter appears anywhere in this design.
--
-- Update.  New compare values are taken into the shadow registers at the same
-- trough that issues the strobe, so the answer computed from sample k is applied
-- over period k+1 in full.  Never part way through a period: changing a compare
-- value mid-period produces a pulse of an arbitrary width, which is a real
-- source of current spikes in drives that get this wrong.
--
-- Dead time.  Both transistors of a leg are held off for deadtime clocks after
-- every transition.  It has to outlast the worst-case turn-off of the outgoing
-- device plus the gate-driver propagation skew, minus the best-case turn-on of
-- the incoming one, or the two conduct together across the DC link.  One
-- microsecond, 100 clocks, is generous for a 48 V MOSFET stage; the cost of that
-- generosity is a volt-scale distortion that docs/10-limitations-and-roadmap.md
-- takes up as the first thing to compensate.
-- -----------------------------------------------------------------------------

library ieee;
  use ieee.std_logic_1164.all;
  use ieee.numeric_std.all;

entity pwm3ph is
  generic (
    G_PWM_BITS : natural := 13;
    G_DT_BITS  : natural := 10
  );
  port (
    clk      : in    std_logic;
    rst      : in    std_logic;
    enable   : in    std_logic;                         -- low forces safe off
    pwm_top  : in    unsigned(G_PWM_BITS - 1 downto 0); -- must be >= 2
    deadtime : in    unsigned(G_DT_BITS - 1 downto 0);  -- clocks
    cmp_a    : in    unsigned(G_PWM_BITS - 1 downto 0);
    cmp_b    : in    unsigned(G_PWM_BITS - 1 downto 0);
    cmp_c    : in    unsigned(G_PWM_BITS - 1 downto 0);
    sample   : out   std_logic;                         -- one clock, at trough
    hi       : out   std_logic_vector(2 downto 0);
    lo       : out   std_logic_vector(2 downto 0)
  );
end entity pwm3ph;

architecture rtl of pwm3ph is

  type cmp_array_t is array (0 to 2) of unsigned(G_PWM_BITS - 1 downto 0);

  type dt_array_t is array (0 to 2) of unsigned(G_DT_BITS - 1 downto 0);

  signal cnt    : unsigned(G_PWM_BITS - 1 downto 0);
  signal up     : std_logic;
  signal shadow : cmp_array_t;
  signal raw    : std_logic_vector(2 downto 0);
  signal raw_q  : std_logic_vector(2 downto 0);
  signal dt     : dt_array_t;

begin

  p_carrier : process (clk) is
  begin

    if rising_edge(clk) then
      if rst = '1' then
        cnt    <= (others => '0');
        up     <= '0';
        sample <= '0';

        for k in 0 to 2 loop

          shadow(k) <= (others => '0');

        end loop;

      else
        sample <= '0';
        if up = '1' then
          if cnt >= pwm_top then
            up  <= '0';
            cnt <= cnt - 1;
          else
            cnt <= cnt + 1;
          end if;
        else
          if cnt = 0 then
            up        <= '1';
            cnt       <= cnt + 1;
            sample    <= '1';
            shadow(0) <= cmp_a;
            shadow(1) <= cmp_b;
            shadow(2) <= cmp_c;
          else
            cnt <= cnt - 1;
          end if;
        end if;
      end if;
    end if;

  end process p_carrier;

  g_raw : for k in 0 to 2 generate
    raw(k) <= '1' when cnt < shadow(k) else
              '0';
  end generate g_raw;

  -- One dead-time inserter per leg.  Both outputs are driven low on the clock
  -- the raw signal changes and stay low for deadtime clocks in total, so the
  -- register means exactly what its name says.

  g_leg : for k in 0 to 2 generate

    p_leg : process (clk) is
    begin

      if rising_edge(clk) then
        if rst = '1' or enable = '0' then
          hi(k)    <= '0';
          lo(k)    <= '0';
          dt(k)    <= (others => '0');
          raw_q(k) <= '0';
        else
          raw_q(k) <= raw(k);
          if raw(k) /= raw_q(k) then
            if deadtime = 0 then
              hi(k) <= raw(k);
              lo(k) <= not raw(k);
              dt(k) <= (others => '0');
            else
              hi(k) <= '0';
              lo(k) <= '0';
              dt(k) <= deadtime - 1;
            end if;
          elsif dt(k) /= 0 then
            hi(k) <= '0';
            lo(k) <= '0';
            dt(k) <= dt(k) - 1;
          else
            hi(k) <= raw(k);
            lo(k) <= not raw(k);
          end if;
        end if;
      end if;

    end process p_leg;

  end generate g_leg;

  -- pragma translate_off
  p_check : process (clk) is
  begin

    if rising_edge(clk) then
      if rst = '0' then
        assert pwm_top >= 2
          report "pwm3ph: pwm_top must be at least 2"
          severity failure;

        for k in 0 to 2 loop

          assert not (hi(k) = '1' and lo(k) = '1')
            report "pwm3ph: both transistors of a leg driven on"
            severity failure;
          assert shadow(k) <= pwm_top
            report "pwm3ph: compare value above the carrier top"
            severity failure;

        end loop;

      end if;
    end if;

  end process p_check;

-- pragma translate_on

end architecture rtl;
