-- -----------------------------------------------------------------------------
-- tb_axi4lite_regs
--
-- Protocol and behaviour of the register file.
--
-- The protocol side matters more than it looks.  A slave that asserts a ready
-- before the corresponding valid, or that leaves a response outstanding, does
-- not fail in simulation against a well-behaved master; it fails months later
-- against a processor that happens to pipeline differently.  The driver here
-- follows the handshake strictly, waiting for each ready and each response, so
-- a slave that only worked because the driver was lenient would hang instead of
-- passing.
--
-- The behaviour side covers the three things software will get wrong first:
-- byte strobes on a halfword write, the self-clearing command bit, and writes to
-- addresses that do not exist.
-- -----------------------------------------------------------------------------

library ieee;
  use ieee.std_logic_1164.all;
  use ieee.numeric_std.all;

library work;
  use work.foc_pkg.all;
  use work.tb_pkg.all;

entity tb_axi4lite_regs is
end entity tb_axi4lite_regs;

architecture sim of tb_axi4lite_regs is

  constant C_PERIOD  : time    := 10 ns;
  constant C_ADDR_W  : natural := 8;
  constant C_VERSION : natural := 16#00010000#;

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

  signal ctrl_enable  : std_logic;
  signal ctrl_pwm_en  : std_logic;
  signal ctrl_clear   : std_logic;
  signal ctrl_manual  : std_logic;
  signal ctrl_use_idx : std_logic;
  signal id_ref       : signed(C_W_DATA - 1 downto 0);
  signal iq_ref       : signed(C_W_DATA - 1 downto 0);
  signal kp           : signed(C_W_DATA - 1 downto 0);
  signal ki           : signed(C_W_DATA - 1 downto 0);
  signal vmax         : signed(C_W_DATA - 1 downto 0);
  signal theta_man    : signed(C_W_ANG - 1 downto 0);
  signal pwm_top      : unsigned(15 downto 0);
  signal deadtime     : unsigned(15 downto 0);
  signal qei_cpr      : unsigned(31 downto 0);
  signal qei_scale    : unsigned(31 downto 0);
  signal qei_win      : unsigned(31 downto 0);

  signal sat_flags : std_logic_vector(3 downto 0)  := "1010";
  signal qei_err   : std_logic                     := '1';
  signal qei_index : std_logic                     := '0';
  signal pos_in    : unsigned(31 downto 0)         := to_unsigned(12345, 32);
  signal speed_in  : signed(15 downto 0)           := to_signed(-321, 16);
  signal theta_in  : signed(C_W_ANG - 1 downto 0)  := to_signed(-4096, 16);
  signal id_meas   : signed(C_W_DATA - 1 downto 0) := to_signed(111, 16);
  signal iq_meas   : signed(C_W_DATA - 1 downto 0) := to_signed(-222, 16);
  signal vd_in     : signed(C_W_DATA - 1 downto 0) := to_signed(333, 16);
  signal vq_in     : signed(C_W_DATA - 1 downto 0) := to_signed(-444, 16);

  signal clear_seen : std_logic := '0';

begin

  clk <= not clk after C_PERIOD / 2;

  p_reset : process is
  begin

    aresetn <= '0';
    wait for 6 * C_PERIOD;
    wait until rising_edge(clk);
    aresetn <= '1';
    wait;

  end process p_reset;

  u_dut : entity work.axi4lite_regs
    generic map (
      G_ADDR_BITS => C_ADDR_W,
      G_VERSION   => C_VERSION
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
      pwm_top       => pwm_top,
      deadtime      => deadtime,
      qei_cpr       => qei_cpr,
      qei_scale     => qei_scale,
      qei_win       => qei_win,
      sat_flags     => sat_flags,
      qei_err       => qei_err,
      qei_index     => qei_index,
      pos_in        => pos_in,
      speed_in      => speed_in,
      theta_in      => theta_in,
      id_meas       => id_meas,
      iq_meas       => iq_meas,
      vd_in         => vd_in,
      vq_in         => vq_in
    );

  -- The clear output is a single clock, so it is latched here rather than
  -- sampled: a test that happened to look one clock late would pass without it.
  p_catch_clear : process (clk) is
  begin

    if rising_edge(clk) then
      if ctrl_clear = '1' then
        clear_seen <= '1';
      end if;
    end if;

  end process p_catch_clear;

  p_run : process is

    variable errs   : natural := 0;
    variable checks : natural := 0;
    variable got    : std_logic_vector(31 downto 0);

    procedure axi_write (
      addr : in natural;
      data : in std_logic_vector(31 downto 0);
      strb : in std_logic_vector(3 downto 0)
    ) is
    begin

      wait until rising_edge(clk);
      awaddr  <= std_logic_vector(to_unsigned(addr, C_ADDR_W));
      awvalid <= '1';
      wdata   <= data;
      wstrb   <= strb;
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

    procedure write_read_check (
      addr : in    natural;
      data : in    std_logic_vector(31 downto 0);
      name : in    string
    ) is

      variable v : std_logic_vector(31 downto 0);

    begin

      axi_write(addr, data, "1111");
      axi_read(addr, v);
      check_int(name, to_integer(unsigned(v)), to_integer(unsigned(data)), errs);

    end procedure write_read_check;

  begin

    wait until aresetn = '1';
    wait until rising_edge(clk);

    -- Identification first: if this is wrong nothing else is worth reading.
    axi_read(16#50#, got);
    check_int("VERSION", to_integer(unsigned(got)), C_VERSION, errs);
    checks := checks + 1;

    -- Every read-write register, with a pattern per register so that a
    -- mis-decoded address shows up as a wrong value and not as a lucky match.
    write_read_check(16#08#, x"00001234", "ID_REF");
    write_read_check(16#0C#, x"0000ABCD", "IQ_REF");
    write_read_check(16#10#, x"00001718", "KP");
    write_read_check(16#14#, x"00000128", "KI");
    write_read_check(16#18#, x"00002000", "VMAX");
    write_read_check(16#1C#, x"00004000", "THETA_MAN");
    write_read_check(16#20#, x"000009C4", "PWM_TOP");
    write_read_check(16#24#, x"00000064", "DEADTIME");
    write_read_check(16#28#, x"00002710", "QEI_CPR");
    write_read_check(16#2C#, x"001A36E2", "QEI_SCALE");
    write_read_check(16#30#, x"000186A0", "QEI_WIN");
    checks := checks + 11;

    check_int("kp reaches its output", to_integer(kp), 16#1718#, errs);
    check_int("vmax reaches its output", to_integer(vmax), 16#2000#, errs);
    check_int("pwm_top reaches its output", to_integer(pwm_top), 2500, errs);
    check_int("qei_scale reaches its output",
              to_integer(qei_scale), 16#1A36E2#, errs);
    checks := checks + 4;

    -- Byte strobes.  A halfword or byte store must leave the rest alone.
    axi_write(16#08#, x"00000000", "1111");
    axi_write(16#08#, x"AABBCCDD", "0001");
    axi_read(16#08#, got);
    check_int("byte strobe writes one byte",
              to_integer(unsigned(got)), 16#000000DD#, errs);
    axi_write(16#08#, x"11223344", "0010");
    axi_read(16#08#, got);
    check_int("byte strobe leaves the other bytes",
              to_integer(unsigned(got)), 16#000033DD#, errs);
    checks := checks + 2;

    -- Control bits reach their outputs, and the clear bit is a command that
    -- pulses once and never reads back as set.
    axi_write(16#00#, x"0000001B", "1111");                                       -- bits 0, 1, 3, 4
    check_true("enable set", ctrl_enable = '1', errs);
    check_true("pwm enable set", ctrl_pwm_en = '1', errs);
    check_true("manual angle set", ctrl_manual = '1', errs);
    check_true("index enable set", ctrl_use_idx = '1', errs);
    checks := checks + 4;

    axi_write(16#00#, x"00000004", "1111");                                       -- bit 2, the clear
    axi_read(16#00#, got);
    check_int("clear bit does not stick", to_integer(unsigned(got)), 0, errs);
    check_true("clear pulsed", clear_seen = '1', errs);
    checks := checks + 2;

    -- Telemetry.  These are wired straight through, so this is an address
    -- decode test as much as anything.
    axi_read(16#04#, got);
    check_int("STATUS", to_integer(unsigned(got)), 16#1A#, errs);
    axi_read(16#34#, got);
    check_int("POS", to_integer(unsigned(got)), 12345, errs);
    axi_read(16#38#, got);
    check_int("SPEED", to_integer(signed(got)), -321, errs);
    axi_read(16#3C#, got);
    check_int("THETA", to_integer(signed(got)), -4096, errs);
    axi_read(16#40#, got);
    check_int("ID_MEAS", to_integer(signed(got)), 111, errs);
    axi_read(16#44#, got);
    check_int("IQ_MEAS", to_integer(signed(got)), -222, errs);
    axi_read(16#48#, got);
    check_int("VD", to_integer(signed(got)), 333, errs);
    axi_read(16#4C#, got);
    check_int("VQ", to_integer(signed(got)), -444, errs);
    checks := checks + 8;

    -- A write to a read-only register must be accepted and ignored, not
    -- answered with an error a processor would take as a bus fault.
    axi_write(16#34#, x"DEADBEEF", "1111");
    check_int("write response is OKAY", to_integer(unsigned(bresp)), 0, errs);
    axi_read(16#34#, got);
    check_int("read-only register unchanged",
              to_integer(unsigned(got)), 12345, errs);
    checks := checks + 2;

    -- And so must a write to an address that is not mapped at all.
    axi_write(16#7C#, x"00000001", "1111");
    axi_read(16#7C#, got);
    check_int("unmapped address reads zero", to_integer(unsigned(got)), 0, errs);
    check_int("read response is OKAY", to_integer(unsigned(rresp)), 0, errs);
    checks := checks + 2;

    finish_report("tb_axi4lite_regs", errs, checks);
    wait;

  end process p_run;

  p_timeout : process is
  begin

    wait for 1 ms;
    report "tb_axi4lite_regs: timed out"
      severity failure;

  end process p_timeout;

end architecture sim;
