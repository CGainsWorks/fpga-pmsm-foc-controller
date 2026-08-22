-- -----------------------------------------------------------------------------
-- current_loop
--
-- The whole rotor-frame current controller, from two measured phase currents and
-- a rotor angle to three PWM compare values.
--
--   Clarke  ->  Park  ->  PI(d), PI(q)  ->  limit  ->  inverse Park  ->  SVM
--
-- Two structural points are worth reading the code for.
--
-- Voltage limiting is done in polar form.  The PI outputs are put through a
-- vectoring CORDIC to get (|v|, phi); the magnitude is clipped to the linear
-- modulation limit and the angle is left alone.  Clipping d and q separately
-- would rotate the applied voltage vector away from the direction the controller
-- asked for, which at high modulation shows up as torque ripple and as a
-- d-axis current the controller never commanded.  Clipping the magnitude alone
-- keeps the direction exactly.
--
-- The inverse Park rotation is then free.  The limiter has already put the
-- voltage in polar form, and
--
--     R(theta) R(phi) [m; 0] = R(theta + phi) [m; 0]
--
-- so rotating the clipped magnitude by the sum of the two angles does the work
-- of the limiter reconstruction and the inverse Park in one pass.  The angle
-- addition is a plain 16-bit add: the binary angular measure is a residue system
-- modulo one turn, so it wraps correctly with no comparison and no correction.
--
-- Latency 66 clocks, or 0.66 us at 100 MHz, against a 50 us control period.
-- Expressed as loop phase lag at the 318 Hz closed-loop bandwidth that is 0.076
-- degrees, which is why no attempt is made to shorten it.  The delay that
-- actually matters is the one and a half sample periods between taking the
-- current sample and the centre of the PWM period the answer is applied in, and
-- that is 8.6 degrees, a hundred times larger.  Moving the transform arithmetic
-- into hardware is what makes the first number negligible and leaves only the
-- second, which is set by the modulator and not by the arithmetic.
--
-- One control update is in flight at a time by construction.  A second sample
-- arriving before the first has finished sets the sticky overrun flag rather
-- than corrupting the result quietly.
-- -----------------------------------------------------------------------------

library ieee;
  use ieee.std_logic_1164.all;
  use ieee.numeric_std.all;

library work;
  use work.foc_pkg.all;

entity current_loop is
  generic (
    G_PWM_BITS : natural := 13
  );
  port (
    clk          : in    std_logic;
    rst          : in    std_logic;
    clr          : in    std_logic;
    sample_valid : in    std_logic;
    ia           : in    signed(C_W_DATA - 1 downto 0); -- Q2.13
    ib           : in    signed(C_W_DATA - 1 downto 0); -- Q2.13
    theta        : in    signed(C_W_ANG - 1 downto 0);  -- BAM16
    id_ref       : in    signed(C_W_DATA - 1 downto 0);
    iq_ref       : in    signed(C_W_DATA - 1 downto 0);
    kp           : in    signed(C_W_DATA - 1 downto 0); -- Q3.12
    ki           : in    signed(C_W_DATA - 1 downto 0); -- Q1.14
    vmax         : in    signed(C_W_DATA - 1 downto 0); -- Q2.13
    pwm_top      : in    unsigned(G_PWM_BITS - 1 downto 0);
    out_valid    : out   std_logic;
    cmp_a        : out   unsigned(G_PWM_BITS - 1 downto 0);
    cmp_b        : out   unsigned(G_PWM_BITS - 1 downto 0);
    cmp_c        : out   unsigned(G_PWM_BITS - 1 downto 0);
    id_meas      : out   signed(C_W_DATA - 1 downto 0);
    iq_meas      : out   signed(C_W_DATA - 1 downto 0);
    vd_out       : out   signed(C_W_DATA - 1 downto 0);
    vq_out       : out   signed(C_W_DATA - 1 downto 0);
    -- bit 3 over-modulated, 2 PI at its limit, 1 datapath saturated,
    -- 0 control-update overrun (sticky, cleared by clr)
    sat_flags : out   std_logic_vector(3 downto 0)
  );
end entity current_loop;

architecture rtl of current_loop is

  signal theta_h : signed(C_W_ANG - 1 downto 0);
  signal idref_h : signed(C_W_DATA - 1 downto 0);
  signal iqref_h : signed(C_W_DATA - 1 downto 0);

  signal cl_valid : std_logic;
  signal ialpha   : signed(C_W_DATA - 1 downto 0);
  signal ibeta    : signed(C_W_DATA - 1 downto 0);
  signal cl_sat   : std_logic;

  signal pk_valid : std_logic;
  signal id_i     : signed(C_W_DATA - 1 downto 0);
  signal iq_i     : signed(C_W_DATA - 1 downto 0);
  signal pk_res   : signed(C_W_ANG - 1 downto 0);
  signal pk_sat   : std_logic;

  signal pi_valid : std_logic;
  signal vd_i     : signed(C_W_DATA - 1 downto 0);
  signal vq_i     : signed(C_W_DATA - 1 downto 0);
  signal pid_sat  : std_logic;
  signal piq_sat  : std_logic;

  signal vc_valid : std_logic;
  signal mag      : signed(C_W_DATA - 1 downto 0);
  signal mag_y    : signed(C_W_DATA - 1 downto 0);
  signal phi      : signed(C_W_ANG - 1 downto 0);
  signal vc_sat   : std_logic;

  signal lm_valid : std_logic;
  signal mag_lim  : signed(C_W_DATA - 1 downto 0);
  signal psi      : signed(C_W_ANG - 1 downto 0);

  signal ip_valid : std_logic;
  signal valpha   : signed(C_W_DATA - 1 downto 0);
  signal vbeta    : signed(C_W_DATA - 1 downto 0);
  signal ip_res   : signed(C_W_ANG - 1 downto 0);
  signal ip_sat   : std_logic;

  signal sv_valid : std_logic;
  signal over_mod : std_logic;

  signal busy    : std_logic;
  signal overrun : std_logic;

  signal theta_neg : signed(C_W_ANG - 1 downto 0);

begin

  -- Hold the sample-time angle and references.  The angle is used twice, once to
  -- rotate the measurement into the rotor frame and once to rotate the answer
  -- back out, and both uses must see the same angle or the two transforms do not
  -- compose to the identity.
  p_hold : process (clk) is
  begin

    if rising_edge(clk) then
      if sample_valid = '1' then
        theta_h <= theta;
        idref_h <= id_ref;
        iqref_h <= iq_ref;
      end if;
    end if;

  end process p_hold;

  -- Negating a BAM angle is exact for every value including the end point:
  -- -(-pi) is +pi, and +pi and -pi are the same angle.
  theta_neg <= wrap_to(-resize(theta_h, C_W_ANG + 1), C_W_ANG);

  u_clarke : entity work.clarke
    port map (
      clk       => clk,
      rst       => rst,
      in_valid  => sample_valid,
      ia        => ia,
      ib        => ib,
      out_valid => cl_valid,
      ialpha    => ialpha,
      ibeta     => ibeta,
      sat_out   => cl_sat
    );

  u_park : entity work.cordic
    generic map (
      G_MODE       => rotate_mode,
      G_COMPENSATE => true
    )
    port map (
      clk       => clk,
      rst       => rst,
      in_valid  => cl_valid,
      x_in      => ialpha,
      y_in      => ibeta,
      ang_in    => theta_neg,
      out_valid => pk_valid,
      x_out     => id_i,
      y_out     => iq_i,
      ang_out   => pk_res,
      sat_out   => pk_sat
    );

  u_pi_d : entity work.pi_axis
    port map (
      clk       => clk,
      rst       => rst,
      clr       => clr,
      in_valid  => pk_valid,
      ref       => idref_h,
      meas      => id_i,
      kp        => kp,
      ki        => ki,
      umax      => vmax,
      out_valid => pi_valid,
      u         => vd_i,
      sat_out   => pid_sat
    );

  u_pi_q : entity work.pi_axis
    port map (
      clk       => clk,
      rst       => rst,
      clr       => clr,
      in_valid  => pk_valid,
      ref       => iqref_h,
      meas      => iq_i,
      kp        => kp,
      ki        => ki,
      umax      => vmax,
      out_valid => open,
      u         => vq_i,
      sat_out   => piq_sat
    );

  u_vector : entity work.cordic
    generic map (
      G_MODE       => vector_mode,
      G_COMPENSATE => true
    )
    port map (
      clk       => clk,
      rst       => rst,
      in_valid  => pi_valid,
      x_in      => vd_i,
      y_in      => vq_i,
      ang_in    => (others => '0'),
      out_valid => vc_valid,
      x_out     => mag,
      y_out     => mag_y,
      ang_out   => phi,
      sat_out   => vc_sat
    );

  -- Magnitude limit and the folded rotation angle.
  p_limit : process (clk) is
  begin

    if rising_edge(clk) then
      if mag > vmax then
        mag_lim <= vmax;
      else
        mag_lim <= mag;
      end if;
      psi <= wrap_to(resize(theta_h, C_W_ANG + 1) + resize(phi, C_W_ANG + 1),
                     C_W_ANG);
    end if;

  end process p_limit;

  u_ipark : entity work.cordic
    generic map (
      G_MODE       => rotate_mode,
      G_COMPENSATE => true
    )
    port map (
      clk       => clk,
      rst       => rst,
      in_valid  => lm_valid,
      x_in      => mag_lim,
      y_in      => (others => '0'),
      ang_in    => psi,
      out_valid => ip_valid,
      x_out     => valpha,
      y_out     => vbeta,
      ang_out   => ip_res,
      sat_out   => ip_sat
    );

  u_svm : entity work.svm
    generic map (
      G_PWM_BITS => G_PWM_BITS
    )
    port map (
      clk       => clk,
      rst       => rst,
      in_valid  => ip_valid,
      valpha    => valpha,
      vbeta     => vbeta,
      pwm_top   => pwm_top,
      out_valid => sv_valid,
      cmp_a     => cmp_a,
      cmp_b     => cmp_b,
      cmp_c     => cmp_c,
      over_mod  => over_mod
    );

  -- Pipeline bookkeeping, telemetry and the sticky status bits.
  p_track : process (clk) is
  begin

    if rising_edge(clk) then
      if rst = '1' then
        lm_valid <= '0';
        busy     <= '0';
        overrun  <= '0';
      else
        lm_valid <= vc_valid;

        if sample_valid = '1' then
          if busy = '1' then
            overrun <= '1';
          end if;
          busy <= '1';
        elsif sv_valid = '1' then
          busy <= '0';
        end if;

        if clr = '1' then
          overrun <= '0';
        end if;
      end if;

      if pk_valid = '1' then
        id_meas <= id_i;
        iq_meas <= iq_i;
      end if;
      if pi_valid = '1' then
        vd_out <= vd_i;
        vq_out <= vq_i;
      end if;
    end if;

  end process p_track;

  out_valid <= sv_valid;

  sat_flags <= over_mod & (pid_sat or piq_sat) & (cl_sat or pk_sat or vc_sat or ip_sat)
               & overrun;

end architecture rtl;
