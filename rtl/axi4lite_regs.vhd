-- -----------------------------------------------------------------------------
-- axi4lite_regs
--
-- AXI4-Lite slave holding the control and telemetry registers.  Map and bit
-- assignments in docs/08-register-map.md.
--
-- The handshaking is the minimal form that is still legal: a channel's ready may
-- depend combinationally on that channel's valid and only the reverse would
-- deadlock, so awready and wready are asserted together when a write can be
-- accepted and arready when a read can.  At most one write response and one read
-- response are outstanding, which is all AXI4-Lite ever needs and removes every
-- ordering question a deeper slave would have to answer.
--
-- Write strobes are honoured per byte.  A processor writing a 16-bit field with
-- a halfword store must not clear the other half of the word and software will
-- eventually do exactly that.
--
-- Everything the control loop reads from here crosses no clock domain: the
-- register file and the loop share the clock.  Driving the loop from a different
-- clock would need the parameter set to be transferred atomically, which is
-- noted in docs/10-limitations-and-roadmap.md and is not the case today.
-- -----------------------------------------------------------------------------

library ieee;
  use ieee.std_logic_1164.all;
  use ieee.numeric_std.all;

library work;
  use work.foc_pkg.all;

entity axi4lite_regs is
  generic (
    G_ADDR_BITS : natural := 8;
    G_VERSION   : natural := 16#00010000#
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

    -- Control outputs
    ctrl_enable  : out   std_logic;
    ctrl_pwm_en  : out   std_logic;
    ctrl_clear   : out   std_logic;                       -- one clock
    ctrl_manual  : out   std_logic;
    ctrl_use_idx : out   std_logic;
    id_ref       : out   signed(C_W_DATA - 1 downto 0);
    iq_ref       : out   signed(C_W_DATA - 1 downto 0);
    kp           : out   signed(C_W_DATA - 1 downto 0);
    ki           : out   signed(C_W_DATA - 1 downto 0);
    vmax         : out   signed(C_W_DATA - 1 downto 0);
    theta_man    : out   signed(C_W_ANG - 1 downto 0);
    pwm_top      : out   unsigned(15 downto 0);
    deadtime     : out   unsigned(15 downto 0);
    qei_cpr      : out   unsigned(31 downto 0);
    qei_scale    : out   unsigned(31 downto 0);
    qei_win      : out   unsigned(31 downto 0);

    -- Telemetry inputs
    sat_flags : in    std_logic_vector(3 downto 0);
    qei_err   : in    std_logic;
    qei_index : in    std_logic;
    pos_in    : in    unsigned(31 downto 0);
    speed_in  : in    signed(15 downto 0);
    theta_in  : in    signed(C_W_ANG - 1 downto 0);
    id_meas   : in    signed(C_W_DATA - 1 downto 0);
    iq_meas   : in    signed(C_W_DATA - 1 downto 0);
    vd_in     : in    signed(C_W_DATA - 1 downto 0);
    vq_in     : in    signed(C_W_DATA - 1 downto 0)
  );
end entity axi4lite_regs;

architecture rtl of axi4lite_regs is

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
  constant C_A_ID_MEAS   : natural := 16#40#;
  constant C_A_IQ_MEAS   : natural := 16#44#;
  constant C_A_VD        : natural := 16#48#;
  constant C_A_VQ        : natural := 16#4C#;
  constant C_A_VERSION   : natural := 16#50#;

  constant C_OKAY : std_logic_vector(1 downto 0) := "00";

  signal reg_ctrl      : std_logic_vector(31 downto 0);
  signal reg_id_ref    : std_logic_vector(31 downto 0);
  signal reg_iq_ref    : std_logic_vector(31 downto 0);
  signal reg_kp        : std_logic_vector(31 downto 0);
  signal reg_ki        : std_logic_vector(31 downto 0);
  signal reg_vmax      : std_logic_vector(31 downto 0);
  signal reg_theta_man : std_logic_vector(31 downto 0);
  signal reg_pwm_top   : std_logic_vector(31 downto 0);
  signal reg_deadtime  : std_logic_vector(31 downto 0);
  signal reg_qei_cpr   : std_logic_vector(31 downto 0);
  signal reg_qei_scale : std_logic_vector(31 downto 0);
  signal reg_qei_win   : std_logic_vector(31 downto 0);

  signal bvalid_r : std_logic;
  signal rvalid_r : std_logic;
  signal rdata_r  : std_logic_vector(31 downto 0);

  signal wr_go : std_logic;
  signal rd_go : std_logic;

  function merge (old_v : std_logic_vector(31 downto 0);
    new_v : std_logic_vector(31 downto 0);
    strb  : std_logic_vector(3 downto 0))
    return std_logic_vector is

    variable r : std_logic_vector(31 downto 0);

  begin

    r := old_v;

    for i in 0 to 3 loop

      if strb(i) = '1' then
        r(8 * i + 7 downto 8 * i) := new_v(8 * i + 7 downto 8 * i);
      end if;

    end loop;

    return r;

  end function merge;

begin

  wr_go <= '1' when s_axi_awvalid = '1' and s_axi_wvalid = '1' and bvalid_r = '0' else
           '0';
  rd_go <= '1' when s_axi_arvalid = '1' and rvalid_r = '0' else
           '0';

  s_axi_awready <= wr_go;
  s_axi_wready  <= wr_go;
  s_axi_arready <= rd_go;
  s_axi_bvalid  <= bvalid_r;
  s_axi_rvalid  <= rvalid_r;
  s_axi_rdata   <= rdata_r;
  s_axi_bresp   <= C_OKAY;
  s_axi_rresp   <= C_OKAY;

  p_write : process (clk) is

    variable a : natural;

  begin

    if rising_edge(clk) then
      if aresetn = '0' then
        reg_ctrl      <= (others => '0');
        reg_id_ref    <= (others => '0');
        reg_iq_ref    <= (others => '0');
        reg_kp        <= (others => '0');
        reg_ki        <= (others => '0');
        reg_vmax      <= (others => '0');
        reg_theta_man <= (others => '0');
        reg_pwm_top   <= (others => '0');
        reg_deadtime  <= (others => '0');
        reg_qei_cpr   <= (others => '0');
        reg_qei_scale <= (others => '0');
        reg_qei_win   <= (others => '0');
        bvalid_r      <= '0';
        ctrl_clear    <= '0';
      else
        ctrl_clear <= '0';

        if wr_go = '1' then
          a        := to_integer(unsigned(s_axi_awaddr));
          bvalid_r <= '1';

          case a is

            when C_A_CTRL =>

              reg_ctrl <= merge(reg_ctrl, s_axi_wdata, s_axi_wstrb);
              if s_axi_wstrb(0) = '1' and s_axi_wdata(2) = '1' then
                ctrl_clear <= '1';
              end if;

            when C_A_ID_REF =>

              reg_id_ref <= merge(reg_id_ref, s_axi_wdata, s_axi_wstrb);

            when C_A_IQ_REF =>

              reg_iq_ref <= merge(reg_iq_ref, s_axi_wdata, s_axi_wstrb);

            when C_A_KP =>

              reg_kp <= merge(reg_kp, s_axi_wdata, s_axi_wstrb);

            when C_A_KI =>

              reg_ki <= merge(reg_ki, s_axi_wdata, s_axi_wstrb);

            when C_A_VMAX =>

              reg_vmax <= merge(reg_vmax, s_axi_wdata, s_axi_wstrb);

            when C_A_THETA_MAN =>

              reg_theta_man <= merge(reg_theta_man, s_axi_wdata, s_axi_wstrb);

            when C_A_PWM_TOP =>

              reg_pwm_top <= merge(reg_pwm_top, s_axi_wdata, s_axi_wstrb);

            when C_A_DEADTIME =>

              reg_deadtime <= merge(reg_deadtime, s_axi_wdata, s_axi_wstrb);

            when C_A_QEI_CPR =>

              reg_qei_cpr <= merge(reg_qei_cpr, s_axi_wdata, s_axi_wstrb);

            when C_A_QEI_SCALE =>

              reg_qei_scale <= merge(reg_qei_scale, s_axi_wdata, s_axi_wstrb);

            when C_A_QEI_WIN =>

              reg_qei_win <= merge(reg_qei_win, s_axi_wdata, s_axi_wstrb);

            when others =>

              -- Writes to read-only or unmapped addresses complete with OKAY.
              -- A SLVERR would be more informative but AXI4-Lite masters in
              -- processor systems commonly treat it as a bus fault and a
              -- register file is not the right place to halt a CPU from.
              null;

          end case;

        elsif bvalid_r = '1' and s_axi_bready = '1' then
          bvalid_r <= '0';
        end if;

        -- The clear bit is a command, not state.
        reg_ctrl(2) <= '0';
      end if;
    end if;

  end process p_write;

  p_read : process (clk) is

    variable a : natural;

  begin

    if rising_edge(clk) then
      if aresetn = '0' then
        rvalid_r <= '0';
        rdata_r  <= (others => '0');
      else
        if rd_go = '1' then
          a        := to_integer(unsigned(s_axi_araddr));
          rvalid_r <= '1';
          rdata_r  <= (others => '0');

          case a is

            when C_A_CTRL =>

              rdata_r <= reg_ctrl;

            when C_A_STATUS =>

              rdata_r(3 downto 0) <= sat_flags;
              rdata_r(4)          <= qei_err;
              rdata_r(5)          <= qei_index;

            when C_A_ID_REF =>

              rdata_r <= reg_id_ref;

            when C_A_IQ_REF =>

              rdata_r <= reg_iq_ref;

            when C_A_KP =>

              rdata_r <= reg_kp;

            when C_A_KI =>

              rdata_r <= reg_ki;

            when C_A_VMAX =>

              rdata_r <= reg_vmax;

            when C_A_THETA_MAN =>

              rdata_r <= reg_theta_man;

            when C_A_PWM_TOP =>

              rdata_r <= reg_pwm_top;

            when C_A_DEADTIME =>

              rdata_r <= reg_deadtime;

            when C_A_QEI_CPR =>

              rdata_r <= reg_qei_cpr;

            when C_A_QEI_SCALE =>

              rdata_r <= reg_qei_scale;

            when C_A_QEI_WIN =>

              rdata_r <= reg_qei_win;

            when C_A_POS =>

              rdata_r <= std_logic_vector(pos_in);

            when C_A_SPEED =>

              rdata_r <= std_logic_vector(resize(speed_in, 32));

            when C_A_THETA =>

              rdata_r <= std_logic_vector(resize(theta_in, 32));

            when C_A_ID_MEAS =>

              rdata_r <= std_logic_vector(resize(id_meas, 32));

            when C_A_IQ_MEAS =>

              rdata_r <= std_logic_vector(resize(iq_meas, 32));

            when C_A_VD =>

              rdata_r <= std_logic_vector(resize(vd_in, 32));

            when C_A_VQ =>

              rdata_r <= std_logic_vector(resize(vq_in, 32));

            when C_A_VERSION =>

              rdata_r <= std_logic_vector(to_unsigned(G_VERSION, 32));

            when others =>

              rdata_r <= (others => '0');

          end case;

        elsif rvalid_r = '1' and s_axi_rready = '1' then
          rvalid_r <= '0';
        end if;
      end if;
    end if;

  end process p_read;

  ctrl_enable  <= reg_ctrl(0);
  ctrl_pwm_en  <= reg_ctrl(1);
  ctrl_manual  <= reg_ctrl(3);
  ctrl_use_idx <= reg_ctrl(4);

  id_ref    <= signed(reg_id_ref(C_W_DATA - 1 downto 0));
  iq_ref    <= signed(reg_iq_ref(C_W_DATA - 1 downto 0));
  kp        <= signed(reg_kp(C_W_DATA - 1 downto 0));
  ki        <= signed(reg_ki(C_W_DATA - 1 downto 0));
  vmax      <= signed(reg_vmax(C_W_DATA - 1 downto 0));
  theta_man <= signed(reg_theta_man(C_W_ANG - 1 downto 0));
  pwm_top   <= unsigned(reg_pwm_top(15 downto 0));
  deadtime  <= unsigned(reg_deadtime(15 downto 0));
  qei_cpr   <= unsigned(reg_qei_cpr);
  qei_scale <= unsigned(reg_qei_scale);
  qei_win   <= unsigned(reg_qei_win);

end architecture rtl;
