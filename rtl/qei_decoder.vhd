-- -----------------------------------------------------------------------------
-- qei_decoder
--
-- Quadrature encoder interface: synchronise, filter, decode four counts per line
-- cycle, hold a modulo-revolution position, map it to an electrical angle and
-- estimate speed.  Numbers in docs/07-encoder.md.
--
-- Synchronising.  A and B are asynchronous to this clock, so each goes through
-- three flip-flops before anything looks at it.  The mean time between
-- synchroniser failures is
--
--     MTBF = exp(t_r / tau) / (T0 f_clk f_data)
--
-- with t_r the settling time allowed, one clock here.  For a 28 nm device with
-- the typical tau of about 20 ps and T0 of about 10 fs, one clock of 10 ns is
-- 500 time constants and the exponential puts the answer far beyond any
-- interesting horizon.  Two stages are enough and the third is free.
--
-- Filtering.  After the synchroniser each channel must hold the same value for
-- G_FILTER consecutive clocks before it is believed.  The filter must be short
-- compared with the closest two real edges can ever be.  At 6000 rpm on a 2500
-- line encoder, four counts per cycle gives 10^6 counts per second, so real
-- edges are 100 clocks apart at 100 MHz.  Four clocks is 25 times shorter than
-- that, so the filter cannot delete a real transition and it removes anything
-- under 40 ns, which covers the reflections and coupled noise a motor cable
-- carries.
--
-- Decoding.  A and B are a two-bit Gray code: exactly one of them changes at
-- each valid transition, so the direction is available from a single XOR of the
-- new A with the old B.  Both changing in one filtered sample means a count was
-- missed, which sets a sticky error bit rather than silently losing position.
--
-- Angle.  theta = pos * scale, taking bits 31 down to 16, with the register
-- scale set to round(pole_pairs * 2**32 / counts_per_rev).  The low 32 bits of
-- the product are the fractional part of the electrical turn, so the modulo is
-- free and the rounding error of scale is at most 0.5 in 2**-16 of a turn,
-- which is 0.076 of one LSB of the 16-bit angle at 10000 counts per revolution.
-- It does not accumulate, because pos itself is exact.
--
-- Speed.  Counts per fixed window, the M method.  Its quantisation is one count
-- per window regardless of speed, so the relative error grows without bound as
-- the machine slows: at 10000 counts per revolution and a 1 ms window one count
-- is 6 rpm, fine at 3000 rpm and useless at 30.  The fix is to time a whole
-- number of counts instead or to do both and pick; it is listed in
-- docs/10-limitations-and-roadmap.md and is not needed by the current loop,
-- which uses angle and never speed.
-- -----------------------------------------------------------------------------

library ieee;
  use ieee.std_logic_1164.all;
  use ieee.numeric_std.all;

entity qei_decoder is
  generic (
    G_POS_BITS : natural := 20;
    G_FILTER   : natural := 4;
    G_WIN_BITS : natural := 24
  );
  port (
    clk         : in    std_logic;
    rst         : in    std_logic;
    clr         : in    std_logic;                         -- clear sticky bits
    a_in        : in    std_logic;
    b_in        : in    std_logic;
    z_in        : in    std_logic;
    cpr         : in    unsigned(G_POS_BITS - 1 downto 0); -- counts per rev, x4
    angle_scale : in    unsigned(31 downto 0);
    win         : in    unsigned(G_WIN_BITS - 1 downto 0); -- speed window, clocks
    use_index   : in    std_logic;
    pos         : out   unsigned(G_POS_BITS - 1 downto 0);
    theta       : out   signed(15 downto 0);
    speed       : out   signed(15 downto 0);               -- counts per window
    index_seen  : out   std_logic;
    err         : out   std_logic
  );
end entity qei_decoder;

architecture rtl of qei_decoder is

  signal sync_a : std_logic_vector(2 downto 0);
  signal sync_b : std_logic_vector(2 downto 0);
  signal sync_z : std_logic_vector(2 downto 0);

  signal sh_a : std_logic_vector(G_FILTER - 1 downto 0);
  signal sh_b : std_logic_vector(G_FILTER - 1 downto 0);

  signal a_f : std_logic;
  signal b_f : std_logic;
  signal a_q : std_logic;
  signal b_q : std_logic;

  signal pos_i : unsigned(G_POS_BITS - 1 downto 0);
  signal total : signed(31 downto 0);
  signal prev  : signed(31 downto 0);

  signal win_cnt : unsigned(G_WIN_BITS - 1 downto 0);

  signal prod  : unsigned(G_POS_BITS + 32 - 1 downto 0);
  signal pos_d : unsigned(G_POS_BITS - 1 downto 0);

  constant C_ALL_ONES : std_logic_vector(G_FILTER - 1 downto 0) := (others => '1');
  constant C_ALL_ZERO : std_logic_vector(G_FILTER - 1 downto 0) := (others => '0');

begin

  p_sync : process (clk) is
  begin

    if rising_edge(clk) then
      sync_a <= sync_a(1 downto 0) & a_in;
      sync_b <= sync_b(1 downto 0) & b_in;
      sync_z <= sync_z(1 downto 0) & z_in;
    end if;

  end process p_sync;

  p_filter : process (clk) is
  begin

    if rising_edge(clk) then
      if rst = '1' then
        sh_a <= (others => '0');
        sh_b <= (others => '0');
        a_f  <= '0';
        b_f  <= '0';
      else
        sh_a <= sh_a(G_FILTER - 2 downto 0) & sync_a(2);
        sh_b <= sh_b(G_FILTER - 2 downto 0) & sync_b(2);
        if sh_a = C_ALL_ONES then
          a_f <= '1';
        elsif sh_a = C_ALL_ZERO then
          a_f <= '0';
        end if;
        if sh_b = C_ALL_ONES then
          b_f <= '1';
        elsif sh_b = C_ALL_ZERO then
          b_f <= '0';
        end if;
      end if;
    end if;

  end process p_filter;

  p_decode : process (clk) is

    variable fwd : boolean;

  begin

    if rising_edge(clk) then
      if rst = '1' then
        pos_i      <= (others => '0');
        total      <= (others => '0');
        a_q        <= '0';
        b_q        <= '0';
        err        <= '0';
        index_seen <= '0';
      else
        a_q <= a_f;
        b_q <= b_f;

        if clr = '1' then
          err        <= '0';
          index_seen <= '0';
        end if;

        -- sync_z(0) is the newest sample, so a rising edge on the index is the
        -- newer of the two settled samples high and the older one low.
        if use_index = '1' and sync_z(1) = '1' and sync_z(2) = '0' then
          pos_i      <= (others => '0');
          index_seen <= '1';
        elsif (a_f /= a_q) or (b_f /= b_q) then
          if (a_f /= a_q) and (b_f /= b_q) then
            -- Both channels moved between two filtered samples.  Either the
            -- encoder is turning faster than the filter can follow or a line is
            -- broken; in both cases the count is no longer trustworthy.
            err <= '1';
          else
            fwd := (a_f xor b_q) = '1';
            if fwd then
              if pos_i >= cpr - 1 then
                pos_i <= (others => '0');
              else
                pos_i <= pos_i + 1;
              end if;
              total <= total + 1;
            else
              if pos_i = 0 then
                pos_i <= cpr - 1;
              else
                pos_i <= pos_i - 1;
              end if;
              total <= total - 1;
            end if;
          end if;
        end if;
      end if;
    end if;

  end process p_decode;

  -- Two registers deep, so that the reported position and the angle derived
  -- from it belong to the same count.  A consumer that read a position one
  -- clock ahead of its own angle would see them disagree by a whole count at
  -- every edge, which is the kind of thing that only shows up as jitter.
  p_angle : process (clk) is
  begin

    if rising_edge(clk) then
      prod  <= pos_i * angle_scale;
      pos_d <= pos_i;
      theta <= signed(prod(31 downto 16));
      pos   <= pos_d;
    end if;

  end process p_angle;

  p_speed : process (clk) is
  begin

    if rising_edge(clk) then
      if rst = '1' then
        win_cnt <= (others => '0');
        prev    <= (others => '0');
        speed   <= (others => '0');
      elsif win_cnt >= win then
        win_cnt <= (others => '0');
        prev    <= total;
        speed   <= resize(total - prev, 16);
      else
        win_cnt <= win_cnt + 1;
      end if;
    end if;

  end process p_speed;

end architecture rtl;
