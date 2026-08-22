-- -----------------------------------------------------------------------------
-- foc_top
--
-- The complete coprocessor: an AXI4-Lite register file, a quadrature encoder
-- interface, the rotor-frame current loop and the three-phase modulator.
--
-- The only sequencing in the whole design lives here and it is one signal.  The
-- modulator issues its sample strobe at the carrier trough; that strobe latches
-- the two phase currents and the rotor angle, starts the current loop and 66
-- clocks later the loop hands three compare values back to the modulator, which
-- takes them at the next trough.  There is no scheduler, no interrupt and no
-- software in the loop and the control period is therefore exactly the PWM
-- period with no jitter of any kind.  That is the part a microcontroller
-- implementation cannot match: not the arithmetic, which any Cortex-M4 with an
-- FPU will do in a few microseconds but the determinism.
--
-- Safety.  Reset and the pwm_enable bit both force all six gate signals low and
-- the dead-time inserter has no path that can drive the two transistors of a leg
-- on together.  That is the whole extent of the protection here: this is a
-- control block, not a safety function.  Desaturation detection, overcurrent
-- trip, gate-supply monitoring and the interlocks a real drive needs live
-- outside it and README.md says so plainly.
-- -----------------------------------------------------------------------------

library ieee;
  use ieee.std_logic_1164.all;
  use ieee.numeric_std.all;

library work;
  use work.foc_pkg.all;

entity foc_top is
  generic (
    G_PWM_BITS  : natural := 13;
    G_DT_BITS   : natural := 10;
    G_POS_BITS  : natural := 20;
    G_WIN_BITS  : natural := 24;
    G_ADDR_BITS : natural := 8
  );
  port (
    clk     : in    std_logic;
    aresetn : in    std_logic;

    s_axi_awaddr  : in    std_logic_vector(G_ADDR_BITS - 1 downto 0);
    s_axi_awvalid : in    std_logic;
    s_axi_awready : out   std_logic;
    s_axi_wdata   : in    std_logic_vector(31 downto 0);
    s_axi_wstrb   : in    std_logic_vector(3 downto 0);
    s_axi_wvalid  : in    std_logic;
    s_axi_wready  : out   std_logic;
    s_axi_bresp   : out   std_logic_vector(1 downto 0);
    s_axi_bvalid  : out   std_logic;
    s_axi_bready  : in    std_logic;
    s_axi_araddr  : in    std_logic_vector(G_ADDR_BITS - 1 downto 0);
    s_axi_arvalid : in    std_logic;
    s_axi_arready : out   std_logic;
    s_axi_rdata   : out   std_logic_vector(31 downto 0);
    s_axi_rresp   : out   std_logic_vector(1 downto 0);
    s_axi_rvalid  : out   std_logic;
    s_axi_rready  : in    std_logic;

    adc_ia : in    signed(C_W_DATA - 1 downto 0);   -- Q2.13, per unit
    adc_ib : in    signed(C_W_DATA - 1 downto 0);

    enc_a : in    std_logic;
    enc_b : in    std_logic;
    enc_z : in    std_logic;

    pwm_hi      : out   std_logic_vector(2 downto 0);
    pwm_lo      : out   std_logic_vector(2 downto 0);
    sample_tick : out   std_logic;

    -- Telemetry.  The compare values on their way into the modulator, valid
    -- from the strobe that loads them.  A drive that logs its own duty cycles
    -- can diagnose a modulation fault without an oscilloscope and the
    -- closed-loop testbench drives its machine model from these rather than
    -- from the gate waveform, which keeps the plant model exact and leaves the
    -- waveform itself to tb_pwm3ph.
    duty_a : out   unsigned(G_PWM_BITS - 1 downto 0);
    duty_b : out   unsigned(G_PWM_BITS - 1 downto 0);
    duty_c : out   unsigned(G_PWM_BITS - 1 downto 0)
  );
end entity foc_top;

architecture rtl of foc_top is

  signal rst : std_logic;

  signal ctrl_enable  : std_logic;
  signal ctrl_pwm_en  : std_logic;
  signal ctrl_clear   : std_logic;
  signal ctrl_manual  : std_logic;
  signal ctrl_use_idx : std_logic;

  signal id_ref    : signed(C_W_DATA - 1 downto 0);
  signal iq_ref    : signed(C_W_DATA - 1 downto 0);
  signal kp        : signed(C_W_DATA - 1 downto 0);
  signal ki        : signed(C_W_DATA - 1 downto 0);
  signal vmax      : signed(C_W_DATA - 1 downto 0);
  signal theta_man : signed(C_W_ANG - 1 downto 0);
  signal pwm_top_r : unsigned(15 downto 0);
  signal deadtm_r  : unsigned(15 downto 0);
  signal qei_cpr   : unsigned(31 downto 0);
  signal qei_scale : unsigned(31 downto 0);
  signal qei_win   : unsigned(31 downto 0);

  signal qei_pos   : unsigned(G_POS_BITS - 1 downto 0);
  signal qei_theta : signed(C_W_ANG - 1 downto 0);
  signal qei_speed : signed(15 downto 0);
  signal qei_err   : std_logic;
  signal qei_index : std_logic;

  signal theta_sel : signed(C_W_ANG - 1 downto 0);

  signal sample    : std_logic;
  signal loop_go   : std_logic;
  signal cmp_a     : unsigned(G_PWM_BITS - 1 downto 0);
  signal cmp_b     : unsigned(G_PWM_BITS - 1 downto 0);
  signal cmp_c     : unsigned(G_PWM_BITS - 1 downto 0);
  signal sat_flags : std_logic_vector(3 downto 0);
  signal id_meas   : signed(C_W_DATA - 1 downto 0);
  signal iq_meas   : signed(C_W_DATA - 1 downto 0);
  signal vd_i      : signed(C_W_DATA - 1 downto 0);
  signal vq_i      : signed(C_W_DATA - 1 downto 0);

  signal ia_s : signed(C_W_DATA - 1 downto 0) := (others => '0');
  signal ib_s : signed(C_W_DATA - 1 downto 0) := (others => '0');

begin

  rst <= not aresetn;

  u_regs : entity work.axi4lite_regs
    generic map (
      G_ADDR_BITS => G_ADDR_BITS
    )
    port map (
      clk           => clk,
      aresetn       => aresetn,
      s_axi_awaddr  => s_axi_awaddr,
      s_axi_awvalid => s_axi_awvalid,
      s_axi_awready => s_axi_awready,
      s_axi_wdata   => s_axi_wdata,
      s_axi_wstrb   => s_axi_wstrb,
      s_axi_wvalid  => s_axi_wvalid,
      s_axi_wready  => s_axi_wready,
      s_axi_bresp   => s_axi_bresp,
      s_axi_bvalid  => s_axi_bvalid,
      s_axi_bready  => s_axi_bready,
      s_axi_araddr  => s_axi_araddr,
      s_axi_arvalid => s_axi_arvalid,
      s_axi_arready => s_axi_arready,
      s_axi_rdata   => s_axi_rdata,
      s_axi_rresp   => s_axi_rresp,
      s_axi_rvalid  => s_axi_rvalid,
      s_axi_rready  => s_axi_rready,
      ctrl_enable   => ctrl_enable,
      ctrl_pwm_en   => ctrl_pwm_en,
      ctrl_clear    => ctrl_clear,
      ctrl_manual   => ctrl_manual,
      ctrl_use_idx  => ctrl_use_idx,
      id_ref        => id_ref,
      iq_ref        => iq_ref,
      kp            => kp,
      ki            => ki,
      vmax          => vmax,
      theta_man     => theta_man,
      pwm_top       => pwm_top_r,
      deadtime      => deadtm_r,
      qei_cpr       => qei_cpr,
      qei_scale     => qei_scale,
      qei_win       => qei_win,
      sat_flags     => sat_flags,
      qei_err       => qei_err,
      qei_index     => qei_index,
      pos_in        => resize(qei_pos, 32),
      speed_in      => qei_speed,
      theta_in      => theta_sel,
      id_meas       => id_meas,
      iq_meas       => iq_meas,
      vd_in         => vd_i,
      vq_in         => vq_i
    );

  u_qei : entity work.qei_decoder
    generic map (
      G_POS_BITS => G_POS_BITS,
      G_WIN_BITS => G_WIN_BITS
    )
    port map (
      clk         => clk,
      rst         => rst,
      clr         => ctrl_clear,
      a_in        => enc_a,
      b_in        => enc_b,
      z_in        => enc_z,
      cpr         => resize(qei_cpr, G_POS_BITS),
      angle_scale => qei_scale,
      win         => resize(qei_win, G_WIN_BITS),
      use_index   => ctrl_use_idx,
      pos         => qei_pos,
      theta       => qei_theta,
      speed       => qei_speed,
      index_seen  => qei_index,
      err         => qei_err
    );

  -- A manual angle is what an open-loop bring-up ramp and every encoder
  -- alignment procedure needs, so it is a mode rather than a debug hook.
  theta_sel <= theta_man when ctrl_manual = '1' else
               qei_theta;

  -- Latch the converter outputs on the strobe.  Sampling once, at the instant
  -- the modulator says the ripple is at its mean, is the whole reason the loop
  -- needs no input filtering.
  p_sample : process (clk) is
  begin

    if rising_edge(clk) then
      if sample = '1' then
        ia_s <= adc_ia;
        ib_s <= adc_ib;
      end if;
      loop_go <= sample and ctrl_enable;
    end if;

  end process p_sample;

  u_loop : entity work.current_loop
    generic map (
      G_PWM_BITS => G_PWM_BITS
    )
    port map (
      clk          => clk,
      rst          => rst,
      clr          => ctrl_clear or not ctrl_enable,
      sample_valid => loop_go,
      ia           => ia_s,
      ib           => ib_s,
      theta        => theta_sel,
      id_ref       => id_ref,
      iq_ref       => iq_ref,
      kp           => kp,
      ki           => ki,
      vmax         => vmax,
      pwm_top      => resize(pwm_top_r, G_PWM_BITS),
      out_valid    => open,
      cmp_a        => cmp_a,
      cmp_b        => cmp_b,
      cmp_c        => cmp_c,
      id_meas      => id_meas,
      iq_meas      => iq_meas,
      vd_out       => vd_i,
      vq_out       => vq_i,
      sat_flags    => sat_flags
    );

  u_pwm : entity work.pwm3ph
    generic map (
      G_PWM_BITS => G_PWM_BITS,
      G_DT_BITS  => G_DT_BITS
    )
    port map (
      clk      => clk,
      rst      => rst,
      enable   => ctrl_pwm_en,
      pwm_top  => resize(pwm_top_r, G_PWM_BITS),
      deadtime => resize(deadtm_r, G_DT_BITS),
      cmp_a    => cmp_a,
      cmp_b    => cmp_b,
      cmp_c    => cmp_c,
      sample   => sample,
      hi       => pwm_hi,
      lo       => pwm_lo
    );

  sample_tick <= sample;

  duty_a <= cmp_a;
  duty_b <= cmp_b;
  duty_c <= cmp_c;

end architecture rtl;
