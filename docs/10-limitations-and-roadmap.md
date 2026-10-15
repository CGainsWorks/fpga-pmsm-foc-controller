# 10. Limitations and roadmap

## The one that matters most: no hardware

This design has never been on an FPGA and has never turned a motor.

Everything in this repository is verified in simulation against models that are
documented, derived and tested. That is a real result and it is a limited one.
Specifically, **none of the following is claimed anywhere and none of it should
be inferred**:

* that the design closes timing at 100 MHz on any device,
* any resource count from a vendor synthesis or place-and-route tool,
* any measured current waveform, step response, efficiency or thermal figure,
* that the dead time is adequate for any real gate driver and transistor pair,
* that the encoder filter is adequate against any real motor cable.

The synthesis step in CI elaborates all nine units through GHDL's synthesiser,
which catches a construct that simulates and cannot be built. It is not a
place-and-route and it produces no timing number.

### What would close the gap

In order and none of it needs an unusual board:

1. **Vendor synthesis.** Vivado or Quartus on a small part, for a resource count
   and a timing report. Expected to be the easy step: the design has no wide
   combinational paths except the PI's single-cycle comparison chain, which is
   the first place a timing report would point.
2. **Bench bring-up on an evaluation board with a three-phase inverter.** An
   Arty or a Cmod with a small BLDC driver board and a 24 V supply is enough.
   The bring-up procedure is in docs/08.
3. **Locked-rotor step response on a real machine**, measured with a current
   probe, against the 0.913 ms the analysis predicts. This is the measurement
   that would turn the closed-loop testbench result into a claim about hardware.
4. **A rotating test with the encoder in the loop**, checking that the d-axis
   current stays at zero while the machine turns under load.

Until at least step 1 and step 3 are done, the honest summary of this repository
is: a fully derived, fully simulated field-oriented control coprocessor with a
verification argument and no hardware evidence.

## Control-side work not done

**Feedforward decoupling.** The first item and the one with a measured
justification behind it. Cancelling the plant pole with the PI zero removes it
from the reference response but not from the disturbance response, so the
back-EMF is rejected at `L/R = 4 ms` rather than at the 0.9 ms the loop follows a
reference in. Adding

    v_d_ff = -w L_q i_q
    v_q_ff = +w (L_d i_d + psi_f)

cancels the disturbance at its source. Cost: three multipliers, registers for
`L_d`, `L_q` and `psi_f` and a speed input the current loop does not currently
use. The effect is quantified in
`tests/test_analysis.py::test_disturbance_rejection_runs_at_the_open_loop_time_constant`.

**Field weakening.** Above base speed, 3308 rpm for the reference machine, the
back-EMF reaches the rail and torque can only be extended by driving `i_d`
negative to oppose the rotor flux. The block accepts a `d`-axis reference and will
follow it, so field weakening can be implemented above it today; a proper
implementation regulates `i_d` from the voltage-limit error rather than from a
speed table and that regulator is not here.

**Salient machines.** With `L_d` not equal to `L_q` there is reluctance torque and
the maximum-torque-per-amp point is no longer at `i_d = 0`. That is a change to
the reference generator above this block, not to the block but the block would
also want per-axis gains rather than the single `KP` and `KI` it has.

**Sensorless operation.** No observer, so the design needs an encoder or a
resolver. A back-EMF observer would fit in the latency budget several times over,
which is one of the reasons the budget is worth having.

**An outer loop.** No speed or position control. The speed estimate exists and is
reported but nothing in the design consumes it.

## Power-stage work not done

**Dead-time compensation.** The largest single error in the voltage the machine
actually receives. At 1 us of dead time on a 50 us period the distortion is 0.96 V
on a 48 V link, **fifty times the duty quantisation** and it appears as a
current-dependent voltage offset whose sign follows the phase current. The
standard correction adds a compensating term to each duty based on the measured
current sign, which the design already has to hand.

**Minimum pulse width.** A leg asked for a duty whose off time is shorter than
the dead time cannot be given it and the pulse is swallowed. `tb_pwm3ph`
measures the effect and bounds it; the design does not currently either avoid the
region or report it.

**Fault handling.** There is no desaturation input, no overcurrent trip, no
gate-supply monitor and no brake chopper. This is a control block and not a
safety function and those belong in the power stage and in a separate protection
path.

## Implementation work not done

**A folded CORDIC.** Each of the three instances is used once per 5000 clocks and
each costs about 1260 flip-flops plus two DSP slices. An iterative version reusing
a single stage would be roughly a eighteenth of the logic and would still finish
in 18 clocks and one shared instance could serve all three uses, which would be
about a fiftieth of the current cost. The pipelined form is kept because it is
what makes the block reusable at full throughput, which is the multi-axis
argument but for a single-axis drive the folded version is plainly the right
one.

**Multi-axis time sharing.** The datapath has the throughput for many axes and
the arbitration to use it does not exist. It is the most valuable thing on this
list commercially and the most work.

**A second clock domain.** The register file and the control loop share a clock.
Driving the loop from a different clock to the AXI interface would need the
parameter set transferred atomically, so that a controller could not run for one
period with the new `KP` and the old `KI`.

**Per-axis gains.** `KP` and `KI` are shared between the two axes, which is
correct for a surface machine where `L_d = L_q` and wrong for a salient one.

**Encoder speed estimation.** The M method quantises to one count per window, so
its relative error grows without bound as the machine slows: 6 rpm at a 1 ms
window and 10000 counts per revolution. The T or M/T method fixes it. Not done
because the current loop uses the angle and never the speed.

**Resolver support.** Common on machines that see heat or vibration. It needs an
excitation output and a demodulator and the angle interface downstream of it is
the same 16-bit binary angular measure this design already speaks.
