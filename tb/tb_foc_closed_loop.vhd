-- -----------------------------------------------------------------------------
-- tb_foc_closed_loop
--
-- The whole coprocessor closing a current loop around a model of the reference
-- machine, configured over its own AXI4-Lite interface exactly as software
-- would.  This is the testbench that says the design controls a motor, rather
-- than that its arithmetic agrees with a model of its arithmetic.
--
-- The machine.  A surface-mount PMSM in the rotor frame, in per unit:
--
--     L d(i_d)/dt = v_d - R i_d + w L i_q
--     L d(i_q)/dt = v_q - R i_q - w L i_d - w psi
--
-- integrated by forward Euler in fifty sub-steps per PWM period.  The step is
-- 1 us against an electrical time constant of 4 ms, four thousand times inside
-- the stability bound, so the integration is not what any number below depends
-- on.  Constants come from tb/machine_pkg.vhd, which is generated from the same
-- file focmodel analyses, so the machine simulated here and the machine the
-- prediction is about cannot drift apart.
--
-- The inverter.  Averaged over one PWM period and driven from the compare values
-- the design actually applies.  Dead time is set to zero for this test: the
-- dead-time distortion belongs to the power stage rather than to the controller,
-- an averaged plant cannot represent it faithfully because its sign follows the
-- current, and the switching waveform including dead time is measured properly
-- in tb_pwm3ph.  Saying which effects a model does not carry is part of the
-- model.
--
-- Phase one, locked rotor.  The classic bench measurement of a current loop:
-- hold the rotor, step the torque-current reference and time the response.  The
-- continuous design predicts a 10 to 90 per cent rise of 1.10 ms; the exact
-- sampled analysis of the same gains, including the one-sample computation
-- delay, predicts 0.91 ms, and that is the figure this testbench checks against.
--
-- Phase two, turning at 500 rpm.  The back-EMF is 0.15 per unit and the
-- cross-coupling term another 0.045, neither of which the controller is told
-- about.  Both are rejected, and the test is that the q-axis current still
-- reaches its reference and the d-axis still sits at zero: a sign error anywhere
-- in the Park or inverse Park path passes every static test and fails this one.
--
-- Phase two is deliberately given 22 ms to settle, and the reason is worth
-- stating because it is a property of the tuning and not of this testbench.
-- Placing the PI zero on the plant pole cancels that pole from the reference
-- response, which is what buys the 0.91 ms rise.  It does not cancel it from the
-- disturbance response: the back-EMF enters after the controller, so rejecting
-- it still proceeds at the open-loop electrical time constant of L/R = 4 ms.
-- That is five times slower than the reference response and it is exactly why
-- production drives add back-EMF and cross-coupling feedforward, which
-- docs/10-limitations-and-roadmap.md carries as the first control-side item.
-- -----------------------------------------------------------------------------

library ieee;
  use ieee.std_logic_1164.all;
  use ieee.numeric_std.all;
  use ieee.math_real.all;

library work;
  use work.foc_pkg.all;
  use work.machine_pkg.all;
  use work.tb_pkg.all;

entity tb_foc_closed_loop is
end entity tb_foc_closed_loop;

architecture sim of tb_foc_closed_loop is

  constant C_PERIOD   : time    := 10 ns;
  constant C_PWM_BITS : natural := 13;
  constant C_ADDR_W   : natural := 8;
  constant C_SUB      : natural := 50;
  constant C_STEP_AT  : natural := 20; -- periods, phase one
  constant C_RUN1     : natural := 220;
  constant C_RUN2     : natural := 450;
  constant C_TAIL     : natural := 20; -- periods averaged
  constant C_RPM      : real    := 500.0;

  constant C_A_CTRL      : natural := 16#00#;
  constant C_A_ID_REF    : natural := 16#08#;
  constant C_A_IQ_REF    : natural := 16#0C#;
  constant C_A_KP        : natural := 16#10#;
  constant C_A_KI        : natural := 16#14#;
  constant C_A_VMAX      : natural := 16#18#;
  constant C_A_THETA_MAN : natural := 16#1C#;
  constant C_A_PWM_TOP   : natural := 16#20#;
  constant C_A_DEADTIME  : natural := 16#24#;

  signal clk     : std_logic := '0';
  signal aresetn : std_logic := '0';

  signal awaddr  : std_logic_vector(C_ADDR_W - 1 downto 0) := (others => '0');
  signal awvalid : std_logic                               := '0';
  signal awready : std_logic;
  signal wdata   : std_logic_vector(31 downto 0)           := (others => '0');
  signal wstrb   : std_logic_vector(3 downto 0)            := (others => '1');
  signal wvalid  : std_logic                               := '0';
  signal wready  : std_logic;
  signal bresp   : std_logic_vector(1 downto 0);
  signal bvalid  : std_logic;
  signal bready  : std_logic                               := '0';
  signal araddr  : std_logic_vector(C_ADDR_W - 1 downto 0) := (others => '0');
  signal arvalid : std_logic                               := '0';
  signal arready : std_logic;
  signal rdata   : std_logic_vector(31 downto 0);
  signal rresp   : std_logic_vector(1 downto 0);
  signal rvalid  : std_logic;
  signal rready  : std_logic                               := '0';

  signal adc_ia : signed(C_W_DATA - 1 downto 0) := (others => '0');
  signal adc_ib : signed(C_W_DATA - 1 downto 0) := (others => '0');

  signal pwm_hi      : std_logic_vector(2 downto 0);
  signal pwm_lo      : std_logic_vector(2 downto 0);
  signal sample_tick : std_logic;
  signal duty_a      : unsigned(C_PWM_BITS - 1 downto 0);
  signal duty_b      : unsigned(C_PWM_BITS - 1 downto 0);
  signal duty_c      : unsigned(C_PWM_BITS - 1 downto 0);

begin

  clk <= not clk after C_PERIOD / 2;

  p_reset : process is
  begin

    aresetn <= '0';
    wait for 10 * C_PERIOD;
    wait until rising_edge(clk);
    aresetn <= '1';
    wait;

  end process p_reset;

  u_dut : entity work.foc_top
    generic map (
      G_PWM_BITS  => C_PWM_BITS,
      G_ADDR_BITS => C_ADDR_W
    )
    port map (
      clk           => clk,
      aresetn       => aresetn,
      s_axi_awaddr  => awaddr,
      s_axi_awvalid => awvalid,
      s_axi_awready => awready,
      s_axi_wdata   => wdata,
      s_axi_wstrb   => wstrb,
      s_axi_wvalid  => wvalid,
      s_axi_wready  => wready,
      s_axi_bresp   => bresp,
      s_axi_bvalid  => bvalid,
      s_axi_bready  => bready,
      s_axi_araddr  => araddr,
      s_axi_arvalid => arvalid,
      s_axi_arready => arready,
      s_axi_rdata   => rdata,
      s_axi_rresp   => rresp,
      s_axi_rvalid  => rvalid,
      s_axi_rready  => rready,
      adc_ia        => adc_ia,
      adc_ib        => adc_ib,
      enc_a         => '0',
      enc_b         => '0',
      enc_z         => '0',
      pwm_hi        => pwm_hi,
      pwm_lo        => pwm_lo,
      sample_tick   => sample_tick,
      duty_a        => duty_a,
      duty_b        => duty_b,
      duty_c        => duty_c
    );

  p_run : process is

    variable errs   : natural := 0;
    variable checks : natural := 0;

    variable id_pu  : real;
    variable iq_pu  : real;
    variable th     : real;
    variable omega  : real;
    variable target : real;
    variable runlen : natural;
    variable tail0  : natural;

    variable da   : real;
    variable db   : real;
    variable dc   : real;
    variable valp : real;
    variable vbet : real;
    variable vd   : real;
    variable vq   : real;
    variable did  : real;
    variable diq  : real;
    variable dt   : real;
    variable ialp : real;
    variable ibet : real;

    variable t10   : integer;
    variable t90   : integer;
    variable peak  : real;
    variable tail  : real;
    variable rise  : real;
    variable maxid : real;

    procedure axi_write (
      addr : in natural;
      data : in integer
    ) is
    begin

      wait until rising_edge(clk);
      awaddr  <= std_logic_vector(to_unsigned(addr, C_ADDR_W));
      awvalid <= '1';
      wdata   <= std_logic_vector(to_signed(data, 32));
      wstrb   <= "1111";
      wvalid  <= '1';
      bready  <= '1';
      wait until rising_edge(clk) and awready = '1' and wready = '1';
      awvalid <= '0';
      wvalid  <= '0';
      wait until rising_edge(clk) and bvalid = '1';
      bready  <= '0';

    end procedure axi_write;

    -- The analogue front end: quantise to the converter, then present the result
    -- in the Q2.13 format the datapath speaks.

    function adc_q (x : real) return integer is

      variable v : real;

    begin

      v := real(integer(x / C_ADC_LSB)) * C_ADC_LSB;

      if v > C_ADC_FS - C_ADC_LSB then
        v := C_ADC_FS - C_ADC_LSB;
      end if;

      if v < -C_ADC_FS then
        v := -C_ADC_FS;
      end if;

      return integer(v * C_Q_SCALE);

    end function adc_q;

    function bam (x : real) return integer is

      variable n : integer;

    begin

      n := integer(x / MATH_PI * 32768.0) mod 65536;

      if n >= 32768 then
        n := n - 65536;
      end if;

      return n;

    end function bam;

  begin

    dt := C_TS / real(C_SUB);
    wait until aresetn = '1';
    wait until rising_edge(clk);

    for phase in 1 to 2 loop

      if phase = 1 then
        omega  := 0.0;
        target := 0.5;
        runlen := C_RUN1;
      else
        omega  := C_RPM / 60.0 * MATH_2_PI * real(C_POLE_PAIRS);
        target := 0.3;
        runlen := C_RUN2;
      end if;

      tail0 := runlen - C_TAIL;

      -- ------------------------------------------------------------------
      -- Configure with the loop and the gates held off, so that nothing here
      -- depends on the order the registers are written in.
      -- ------------------------------------------------------------------
      axi_write(C_A_CTRL, 0);
      axi_write(C_A_PWM_TOP, C_PWM_TOP);
      axi_write(C_A_DEADTIME, 0);
      axi_write(C_A_KP, C_KP_Q);
      axi_write(C_A_KI, C_KI_Q);
      axi_write(C_A_VMAX, C_VMAX_Q);
      axi_write(C_A_THETA_MAN, 0);
      axi_write(C_A_ID_REF, 0);

      if phase = 1 then
        axi_write(C_A_IQ_REF, 0);
      else
        axi_write(C_A_IQ_REF, integer(target * C_Q_SCALE));
      end if;

      axi_write(C_A_CTRL, 4);                                                        -- clear the state

      id_pu  := 0.0;
      iq_pu  := 0.0;
      th     := 0.0;
      t10    := -1;
      t90    := -1;
      peak   := 0.0;
      tail   := 0.0;
      maxid  := 0.0;
      adc_ia <= (others => '0');
      adc_ib <= (others => '0');

      axi_write(C_A_CTRL, 11);                                                       -- enable, gates, manual

      for k in 0 to runlen - 1 loop

        wait until rising_edge(clk) and sample_tick = '1';

        -- The compare values being loaded into the modulator on this very strobe
        -- are the ones applied over the period that starts here.
        da := real(to_integer(duty_a)) / real(C_PWM_TOP);
        db := real(to_integer(duty_b)) / real(C_PWM_TOP);
        dc := real(to_integer(duty_c)) / real(C_PWM_TOP);

        -- Duty to per-unit stationary-frame voltage.  The half-rail offsets
        -- cancel in the Clarke combination, which is another way of saying that
        -- the inverter's common mode never reaches the machine.
        valp := (2.0 * da - db - dc) / 3.0 * C_SQRT3;
        vbet := db - dc;

        for s in 1 to C_SUB loop

          vd    := cos(th) * valp + sin(th) * vbet;
          vq    := -sin(th) * valp + cos(th) * vbet;
          did   := (vd - C_R_PU * id_pu + omega * C_L_PU * iq_pu) / C_L_PU;
          diq   := (vq - C_R_PU * iq_pu - omega * C_L_PU * id_pu
                    - omega * C_PSI_PU) / C_L_PU;
          id_pu := id_pu + did * dt;
          iq_pu := iq_pu + diq * dt;
          th    := th + omega * dt;

        end loop;

        -- Present the new state to the converter and the new angle to the
        -- register; both are latched on the next strobe, which is where a real
        -- encoder and a real converter would also have been read.
        ialp   := cos(th) * id_pu - sin(th) * iq_pu;
        ibet   := sin(th) * id_pu + cos(th) * iq_pu;
        adc_ia <= to_signed(adc_q(ialp), C_W_DATA);
        adc_ib <= to_signed(adc_q((-ialp + C_SQRT3 * ibet) / 2.0), C_W_DATA);

        if phase = 1 and k = C_STEP_AT - 1 then
          axi_write(C_A_IQ_REF, integer(target * C_Q_SCALE));
        end if;

        if phase = 2 then
          axi_write(C_A_THETA_MAN, bam(th));
        end if;

        if phase = 1 and k >= C_STEP_AT then
          if t10 < 0 and iq_pu >= 0.1 * target then
            t10 := k;
          end if;
          if t90 < 0 and iq_pu >= 0.9 * target then
            t90 := k;
          end if;
          if iq_pu > peak then
            peak := iq_pu;
          end if;
        end if;

        if k >= tail0 - 80 and abs (id_pu) > maxid then
          maxid := abs (id_pu);
        end if;

        if k >= tail0 then
          tail := tail + iq_pu / real(C_TAIL);
        end if;

      end loop;

      -- ------------------------------------------------------------------
      -- Results.
      -- ------------------------------------------------------------------
      report "tb_foc_closed_loop phase " & integer'image(phase)
             & ": settled " & real'image(tail) & " pu against " & real'image(target)
             & ", worst |id| " & real'image(maxid) & " pu"
        severity note;

      check_true("steady-state error is inside half a per cent of full scale",
                 abs (tail - target) <= 0.005, errs);
      checks := checks + 1;

      if phase = 1 then
        check_true("the step was seen", t10 >= 0 and t90 >= 0, errs);
        checks := checks + 1;
        if t10 >= 0 and t90 >= 0 then
          rise   := real(t90 - t10) * C_TS;
          report "tb_foc_closed_loop: rise " & real'image(rise * 1.0e3)
                 & " ms against the sampled prediction "
                 & real'image(C_RISE_DISC * 1.0e3) & " ms and the continuous "
                 & real'image(C_RISE_CONT * 1.0e3) & " ms"
            severity note;
          check_true("rise time matches the sampled prediction",
                     abs (rise - C_RISE_DISC) <= 0.25 * C_RISE_DISC, errs);
          check_true("no significant overshoot", peak <= 1.10 * target, errs);
          checks := checks + 2;
        end if;
        check_true("d axis stays at zero at standstill", maxid <= 0.01, errs);
      else
        check_true("d axis is driven back to zero while turning",
                   maxid <= 0.01, errs);
      end if;

      checks := checks + 1;

    end loop;

    finish_report("tb_foc_closed_loop", errs, checks);
    wait;

  end process p_run;

  p_timeout : process is
  begin

    wait for 100 ms;
    report "tb_foc_closed_loop: timed out"
      severity failure;

  end process p_timeout;

end architecture sim;
