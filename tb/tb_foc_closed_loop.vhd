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
-- The shaft and the encoder.  tb/encoder_model.vhd turns the shaft at an
-- imposed speed, as a dynamometer would and drives the A and B pins of a 2500
-- line encoder from the true angle.  The design is told nothing else about the
-- rotor: CTRL.MANUAL is clear, so the angle the current loop uses is the one
-- the quadrature decoder works out from those edges.  The machine model reads
-- the same true angle back from the shaft model at every strobe, so there is
-- one rotor and the decoder is measured against it.
--
-- The inverter.  Averaged over one PWM period and driven from the compare values
-- the design actually applies.  Dead time is set to zero for this test: the
-- dead-time distortion belongs to the power stage rather than to the controller,
-- an averaged plant cannot represent it faithfully because its sign follows the
-- current and the switching waveform including dead time is measured properly
-- in tb_pwm3ph.  Saying which effects a model does not carry is part of the
-- model.
--
-- Four test cases, each run by VUnit as a simulation of its own.
--
-- locked_rotor_step.  The classic bench measurement of a current loop: hold
-- the rotor, step the torque-current reference and time the response.  The
-- continuous design predicts a 10 to 90 per cent rise of 1.10 ms; the exact
-- sampled analysis of the same gains, including the one-sample computation
-- delay, predicts 0.91 ms and that is the figure this testbench checks against.
--
-- forward_500_rpm and reverse_500_rpm.  The back-EMF is 0.15 per unit and the
-- cross-coupling term another 0.045, neither of which the controller is told
-- about.  Both are rejected and the test is that the q-axis current still
-- reaches its reference and the d-axis still sits at zero: a sign error anywhere
-- in the Park or inverse Park path, or in the decoder's direction, passes every
-- static test and fails one of these.  The reverse case starts at count zero
-- and so also takes the decoder through its modulo-revolution wrap downwards.
-- Afterwards the decoder's own registers are read over AXI and compared with
-- the shaft: position, speed, the angle in use and the error bit.
--
-- swapped_encoder_channels.  The forward case with A and B wired the wrong way
-- round.  The decoder angle then runs backwards and the loop must lose the d
-- axis.  It is the check that the angle really does come from the encoder pins:
-- if the loop were still being fed its angle some other way, this case would
-- regulate perfectly and fail.
--
-- The rotating cases are given 16 ms to settle and the reason is worth stating
-- because it is a property of the tuning and not of this testbench.  Placing
-- the PI zero on the plant pole cancels that pole from the reference response,
-- which is what buys the 0.91 ms rise.  It does not cancel it from the
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

library vunit_lib;
  context vunit_lib.vunit_context;

library work;
  use work.foc_pkg.all;
  use work.machine_pkg.all;
  use work.tb_pkg.all;

entity tb_foc_closed_loop is
  generic (
    RUNNER_CFG : string
  );
end entity tb_foc_closed_loop;

architecture sim of tb_foc_closed_loop is

  -- The carrier here is 250 steps on a 10 MHz clock rather than 2500 on 100 MHz.
  -- That is deliberate and it changes nothing this testbench is about: the
  -- control period is still 2 * top * T_clk = 50 us, the gains are per sample so
  -- they are unchanged, the machine is unchanged and the predicted response is
  -- unchanged.  What drops is the modulator resolution, from 2500 steps to 250,
  -- and focmodel puts the resulting change in settled current below 1e-4 per
  -- unit, twenty times under the tolerance asserted below.  What is bought is a
  -- tenfold cut in the number of clock edges to simulate, which is what makes 50
  -- ms of machine affordable in CI.  The modulator's own timing is checked at
  -- the real 100 MHz and 2500 ratio in tb_pwm3ph, where it belongs.
  constant C_PERIOD   : time    := 100 ns;
  constant C_TOP      : natural := 250;
  constant C_PWM_BITS : natural := 13;
  constant C_ADDR_W   : natural := 8;
  constant C_SUB      : natural := 50;
  constant C_STEP_AT  : natural := 20; -- periods, locked rotor
  constant C_RUN1     : natural := 220;
  constant C_RUN2     : natural := 320;
  constant C_TAIL     : natural := 20; -- periods averaged
  constant C_RPM      : real    := 500.0;

  -- The speed window is 1 ms, as on the real clock, so 10000 clocks here.
  constant C_QEI_WIN : natural := 10000;

  -- One encoder count is 4 * 65536 / 10000 = 26.2 LSB of the 16-bit angle.  The
  -- decoder's angle lags the shaft by a fraction of a count plus about ten clocks
  -- of synchroniser, filter and pipeline, so two counts is a generous bound that
  -- a wrong scale or a wrong direction still misses by orders of magnitude.
  constant C_THETA_TOL : natural := 53;

  constant C_A_CTRL      : natural := 16#00#;
  constant C_A_STATUS    : natural := 16#04#;
  constant C_A_ID_REF    : natural := 16#08#;
  constant C_A_IQ_REF    : natural := 16#0C#;
  constant C_A_KP        : natural := 16#10#;
  constant C_A_KI        : natural := 16#14#;
  constant C_A_VMAX      : natural := 16#18#;
  constant C_A_THETA_MAN : natural := 16#1C#;
  constant C_A_PWM_TOP   : natural := 16#20#;
  constant C_A_DEADTIME  : natural := 16#24#;
  constant C_A_QEI_CPR   : natural := 16#28#;
  constant C_A_QEI_SCALE : natural := 16#2C#;
  constant C_A_QEI_WIN   : natural := 16#30#;
  constant C_A_POS       : natural := 16#34#;
  constant C_A_SPEED     : natural := 16#38#;
  constant C_A_THETA     : natural := 16#3C#;

  type case_t is (locked, forward, reverse, swapped);

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

  signal enc_a    : std_logic;
  signal enc_b    : std_logic;
  signal rotor_w  : real    := 0.0;
  signal enc_swap : boolean := false;
  signal rotor_th : real;
  signal rotor_n  : integer;

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

  u_shaft : entity work.encoder_model
    generic map (
      G_LINES      => C_ENC_LINES,
      G_POLE_PAIRS => C_POLE_PAIRS,
      G_STEP       => C_PERIOD
    )
    port map (
      omega_e => rotor_w,
      swap_ab => enc_swap,
      theta_e => rotor_th,
      count   => rotor_n,
      enc_a   => enc_a,
      enc_b   => enc_b
    );

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
      enc_a         => enc_a,
      enc_b         => enc_b,
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
    variable tc     : case_t  := locked;

    variable id_pu  : real;
    variable iq_pu  : real;
    variable th0    : real;
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

    variable got      : std_logic_vector(31 downto 0);
    variable pos_got  : integer;
    variable pos_exp  : integer;
    variable spd_got  : integer;
    variable spd_exp  : real;
    variable ang_got  : integer;
    variable ang_exp  : integer;
    variable ang_diff : integer;

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

    procedure axi_read (
      addr : in  natural;
      data : out std_logic_vector(31 downto 0)
    ) is
    begin

      wait until rising_edge(clk);
      araddr  <= std_logic_vector(to_unsigned(addr, C_ADDR_W));
      arvalid <= '1';
      rready  <= '1';
      wait until rising_edge(clk) and arready = '1';
      arvalid <= '0';
      wait until rising_edge(clk) and rvalid = '1';
      data    := rdata;
      rready  <= '0';

    end procedure axi_read;

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

    -- An unwrapped angle in radians to a 16-bit binary angle.

    function bam (x : real) return integer is

      variable n : integer;

    begin

      n := integer(floor(x / MATH_PI * 32768.0 + 0.5)) mod 65536;

      if n >= 32768 then
        n := n - 65536;
      end if;

      return n;

    end function bam;

    -- The shortest signed distance between two values on a circle of m.

    function circ_diff (a : integer; b : integer; m : positive) return integer is

      variable d : integer;

    begin

      d := (a - b) mod m;

      if d > m / 2 then
        d := d - m;
      end if;

      return d;

    end function circ_diff;

  begin

    test_runner_setup(runner, RUNNER_CFG);

    -- The carrier and the clock have to give the control period the gains were
    -- derived for.  Otherwise every prediction below is about a different loop.
    assert (2 * C_TOP * C_PERIOD) / 1 ns = integer(C_TS * 1.0e9)
      report "tb_foc_closed_loop: carrier and clock do not give a 50 us period"
      severity failure;

    while test_suite loop

      if run("locked_rotor_step") then
        tc := locked;
      elsif run("forward_500_rpm") then
        tc := forward;
      elsif run("reverse_500_rpm") then
        tc := reverse;
      elsif run("swapped_encoder_channels") then
        tc := swapped;
      end if;

      case tc is

        when locked =>

          omega  := 0.0;
          target := 0.5;
          runlen := C_RUN1;

        when forward | swapped =>

          omega  := C_RPM / 60.0 * MATH_2_PI * real(C_POLE_PAIRS);
          target := 0.3;
          runlen := C_RUN2;

        when reverse =>

          omega  := -C_RPM / 60.0 * MATH_2_PI * real(C_POLE_PAIRS);
          target := -0.3;
          runlen := C_RUN2;

      end case;

      enc_swap <= tc = swapped;
      dt       := C_TS / real(C_SUB);
      tail0    := runlen - C_TAIL;

      wait until aresetn = '1';
      wait until rising_edge(clk);

      -- ------------------------------------------------------------------
      -- Configure with the loop and the gates held off, so that nothing here
      -- depends on the order the registers are written in.  This is steps 1
      -- and 3 of the bring-up in docs/08-register-map.md; step 2, finding the
      -- encoder zero, is the shaft model starting at count zero on the d axis.
      -- ------------------------------------------------------------------
      axi_write(C_A_CTRL, 0);
      axi_write(C_A_PWM_TOP, C_TOP);
      axi_write(C_A_DEADTIME, 0);
      axi_write(C_A_KP, C_KP_Q);
      axi_write(C_A_KI, C_KI_Q);
      axi_write(C_A_VMAX, C_VMAX_Q);
      axi_write(C_A_THETA_MAN, 0);
      axi_write(C_A_QEI_CPR, C_QEI_CPR);
      axi_write(C_A_QEI_SCALE, C_QEI_SCALE);
      axi_write(C_A_QEI_WIN, C_QEI_WIN);
      axi_write(C_A_ID_REF, 0);

      if tc = locked then
        axi_write(C_A_IQ_REF, 0);
      else
        axi_write(C_A_IQ_REF, integer(target * C_Q_SCALE));
      end if;

      -- Clear the state.
      axi_write(C_A_CTRL, 4);

      id_pu  := 0.0;
      iq_pu  := 0.0;
      t10    := -1;
      t90    := -1;
      peak   := 0.0;
      tail   := 0.0;
      maxid  := 0.0;
      adc_ia <= (others => '0');
      adc_ib <= (others => '0');

      -- Enable the loop and the gates and take the angle from the encoder.
      axi_write(C_A_CTRL, 3);

      for k in 0 to runlen - 1 loop

        wait until rising_edge(clk) and sample_tick = '1';

        -- The shaft starts turning on the first strobe.  From then on the
        -- machine takes its angle from the shaft model at every strobe, which
        -- is the same angle the encoder pins are being driven from.
        if k = 0 then
          rotor_w <= omega;
        end if;

        th0 := rotor_th;

        -- The compare values being loaded into the modulator on this very strobe
        -- are the ones applied over the period that starts here.
        da := real(to_integer(duty_a)) / real(C_TOP);
        db := real(to_integer(duty_b)) / real(C_TOP);
        dc := real(to_integer(duty_c)) / real(C_TOP);

        -- Duty to per-unit stationary-frame voltage.  The half-rail offsets
        -- cancel in the Clarke combination, which is another way of saying that
        -- the inverter's common mode never reaches the machine.
        valp := (2.0 * da - db - dc) / 3.0 * C_SQRT3;
        vbet := db - dc;

        for s in 0 to C_SUB - 1 loop

          th    := th0 + omega * dt * real(s);
          vd    := cos(th) * valp + sin(th) * vbet;
          vq    := -sin(th) * valp + cos(th) * vbet;
          did   := (vd - C_R_PU * id_pu + omega * C_L_PU * iq_pu) / C_L_PU;
          diq   := (vq - C_R_PU * iq_pu - omega * C_L_PU * id_pu
                    - omega * C_PSI_PU) / C_L_PU;
          id_pu := id_pu + did * dt;
          iq_pu := iq_pu + diq * dt;

        end loop;

        -- Present the new state to the converter.  It is latched on the next
        -- strobe, which is where a real converter would also have been read and
        -- where the decoder will be giving the angle of the shaft at that time.
        th     := th0 + omega * C_TS;
        ialp   := cos(th) * id_pu - sin(th) * iq_pu;
        ibet   := sin(th) * id_pu + cos(th) * iq_pu;
        adc_ia <= to_signed(adc_q(ialp), C_W_DATA);
        adc_ib <= to_signed(adc_q((-ialp + C_SQRT3 * ibet) / 2.0), C_W_DATA);

        if tc = locked and k = C_STEP_AT - 1 then
          axi_write(C_A_IQ_REF, integer(target * C_Q_SCALE));
        end if;

        if tc = locked and k >= C_STEP_AT then
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
      report "tb_foc_closed_loop " & case_t'image(tc)
             & ": settled " & real'image(tail) & " pu against " & real'image(target)
             & ", worst |id| " & real'image(maxid) & " pu"
        severity note;

      if tc = swapped then
        -- The decoder angle is running backwards, so the frame the controller
        -- regulates in turns against the rotor and the real d-axis current
        -- cannot be held anywhere near zero.
        check_true("with A and B swapped the loop loses the d axis",
                   maxid > 0.1, errs);
        checks := checks + 1;
      else
        check_true("steady-state error is inside half a per cent of full scale",
                   abs (tail - target) <= 0.005, errs);
        check_true("d axis is held at zero", maxid <= 0.01, errs);
        checks := checks + 2;
      end if;

      if tc = locked then
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
      end if;

      -- ------------------------------------------------------------------
      -- The decoder, read back over AXI and compared with the shaft.
      -- ------------------------------------------------------------------
      if tc /= swapped then
        axi_read(C_A_POS, got);
        pos_got  := to_integer(unsigned(got));
        pos_exp  := rotor_n mod C_QEI_CPR;
        axi_read(C_A_SPEED, got);
        spd_got  := to_integer(signed(got));
        spd_exp  := omega / real(C_POLE_PAIRS) / MATH_2_PI * real(C_QEI_CPR)
                    * real(C_QEI_WIN) * real(C_PERIOD / 1 ns) * 1.0e-9;
        axi_read(C_A_THETA, got);
        ang_got  := to_integer(signed(got));
        ang_exp  := bam(rotor_th);
        ang_diff := circ_diff(ang_got, ang_exp, 65536);

        report "tb_foc_closed_loop " & case_t'image(tc)
               & ": decoder position " & integer'image(pos_got)
               & " against shaft " & integer'image(pos_exp)
               & ", speed " & integer'image(spd_got) & " counts/window against "
               & real'image(spd_exp) & ", angle " & integer'image(ang_got)
               & " against " & integer'image(ang_exp) & " LSB"
          severity note;

        check_true("decoder position follows the shaft",
                   abs (circ_diff(pos_got, pos_exp, C_QEI_CPR)) <= 1, errs);
        check_true("decoder speed matches the shaft within one count",
                   abs (real(spd_got) - spd_exp) <= 1.0, errs);
        check_true("the angle in use is the shaft angle",
                   abs (ang_diff) <= C_THETA_TOL, errs);

        axi_read(C_A_STATUS, got);
        check_int("no invalid encoder transition", to_integer(unsigned(got(4 downto 4))),
                  0, errs);
        checks := checks + 4;
      end if;

    end loop;

    finish_report("tb_foc_closed_loop", errs, checks);
    test_runner_cleanup(runner);
    wait;

  end process p_run;

  p_timeout : process is
  begin

    wait for 40 ms;
    report "tb_foc_closed_loop: timed out"
      severity failure;

  end process p_timeout;

end architecture sim;
