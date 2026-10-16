-- -----------------------------------------------------------------------------
-- encoder_model
--
-- The rotor shaft and the incremental encoder on it, for the closed-loop
-- testbench.  This is what puts the quadrature decoder inside the loop: the
-- design is given A and B and nothing else and has to work out the angle the
-- current loop needs from the edges, exactly as it would on a real drive.
--
-- Mechanics.  The speed is imposed, as a dynamometer holds the shaft of a drive
-- under test, rather than integrated from torque.  That is the standard bench
-- arrangement for testing a current loop and it keeps the predictions in
-- tb_foc_closed_loop about the current loop alone; a speed loop and a load
-- model belong to a speed controller, which this design does not contain.
--
-- The angle is integrated in fixed steps of G_STEP.  The step is placed between
-- clock edges by construction (the testbench clock rises on odd multiples of
-- half a period) so no edge of A or B ever coincides with a sampling edge in
-- the decoder's synchroniser, which in a simulator would be a race rather than
-- metastability.
--
-- The encoder.  A real optical encoder of G_LINES lines gives four quadrature
-- states per line.  The state for a given angle is the integer number of
-- counts floor(theta_mech * 4 G_LINES / 2 pi), taken modulo four through the
-- Gray sequence (A, B) = 00, 10, 11, 01 for increasing angle.  The count is
-- zero at electrical angle zero, which is the state the alignment procedure in
-- docs/08-register-map.md leaves a real drive in.
--
-- What it does not model: edge jitter, line-count eccentricity, the index
-- channel (tested on its own in tb_qei_decoder) and cable noise.  The filter
-- that would reject that noise is likewise tested in tb_qei_decoder.
--
-- swap_ab wires A and B the wrong way round.  The decoder then counts the
-- right number of edges in the wrong direction and the electrical angle runs
-- backwards, which is the classic commissioning fault and the testbench uses it
-- to show the loop really does take its angle from these pins.
-- -----------------------------------------------------------------------------

library ieee;
  use ieee.std_logic_1164.all;
  use ieee.math_real.all;

entity encoder_model is
  generic (
    G_LINES      : positive := 2500;
    G_POLE_PAIRS : positive := 4;
    G_STEP       : time     := 100 ns
  );
  port (
    omega_e : in    real;    -- imposed electrical speed, rad/s
    swap_ab : in    boolean; -- A and B wired the wrong way round
    theta_e : out   real;    -- true electrical angle, rad, unwrapped
    count   : out   integer; -- true quadrature count since time zero
    enc_a   : out   std_logic;
    enc_b   : out   std_logic
  );
end entity encoder_model;

architecture sim of encoder_model is

begin

  p_shaft : process is

    constant C_COUNTS_PER_RAD_E : real := 4.0 * real(G_LINES)
                                          / (MATH_2_PI * real(G_POLE_PAIRS));
    constant C_DT               : real := real(G_STEP / 1 ps) * 1.0e-12;

    variable th : real := 0.0;
    variable n  : integer;
    variable a  : std_logic;
    variable b  : std_logic;

  begin

    loop

      n := integer(floor(th * C_COUNTS_PER_RAD_E));

      case n mod 4 is

        when 0 =>

          a := '0';
          b := '0';

        when 1 =>

          a := '1';
          b := '0';

        when 2 =>

          a := '1';
          b := '1';

        when others =>

          a := '0';
          b := '1';

      end case;

      if swap_ab then
        enc_a <= b;
        enc_b <= a;
      else
        enc_a <= a;
        enc_b <= b;
      end if;

      theta_e <= th;
      count   <= n;

      wait for G_STEP;
      th := th + omega_e * C_DT;

    end loop;

  end process p_shaft;

end architecture sim;
