# 9. How this is verified

Nothing in this repository is verified by looking at a waveform. Every testbench
decides for itself whether it passed, counts its comparisons, prints the count
and returns an exit code. A testbench that manages to run zero comparisons fails,
because a vector file that failed to open would otherwise look exactly like a
pass.

## The three-layer arrangement

**Layer one: the models test themselves.** `tests/` checks the properties the
algorithms are supposed to have, in Python, with no RTL involved. CORDIC
convergence, the amplitude invariance of the Clarke transform, the absence of a
dead zone in the integrator, the pole-zero cancellation, the phase margin, the
15.47 per cent. These are statements about the mathematics and they would be true
of any correct implementation.

**Layer two: the RTL matches the models exactly.** `focmodel` is not an
approximation of the hardware. It performs the same integer operations in the
same order with the same word lengths, so `scripts/gen_vectors.py` produces
expected values that the testbenches compare against **for equality, with no
tolerance**. A tolerance here would hide exactly the class of defect these vectors
exist to catch: an off-by-one in a shift, a rounding term applied in the wrong
place, a saturation that wraps.

**Layer three: the RTL matches the mathematics.** Layer two would pass if the
model and the RTL were wrong in the same way. So the CORDIC and Clarke
testbenches additionally compare against `ieee.math_real` floating point, within
a bound derived in docs/03 before it is measured and report the worst error they
actually saw. The closed-loop testbench compares against a control-theory
prediction rather than against any model of the arithmetic at all.

Each layer catches something the others cannot.

## The testbenches

| Testbench | What it proves | Reference |
| --- | --- | --- |
| `tb_cordic_rot` | 2000 rotations, exact against the model and within 2 LSB of real arithmetic, applied back to back so the pipeline runs at full rate | model and math_real |
| `tb_cordic_vec` | 2000 vectorings, magnitude and argument, including vectors a few LSB long where the argument is badly conditioned | model and math_real |
| `tb_clarke` | 600 pairs including a full electrical turn of a balanced set and inputs outside it that must saturate | model and math_real |
| `tb_pi_axis` | 851 samples: into the limit, held there, error reversed, then a random tail with one-LSB errors | model |
| `tb_svm` | 601 vectors, plus the centring property `max + min = top` on every one inside the linear range | model and property |
| `tb_current_loop` | 401 updates through the whole datapath, checking id, iq, vd, vq and all three compare values | model |
| `tb_pwm3ph` | carrier period, high time in clocks, dead-time interval count and length, narrow-pulse behaviour, never-both-on on every clock | derived properties |
| `tb_qei_decoder` | counting both ways, both wraps, index zeroing, the invalid-transition error, the speed estimate | exact integer expectations |
| `tb_axi4lite_regs` | every register, byte strobes, the self-clearing command bit, read-only and unmapped writes, strict handshaking | protocol and map |
| `tb_foc_closed_loop` | the whole design controlling a machine model, configured over its own AXI interface | control theory |

## What tb_current_loop is for

Every one of the block-level testbenches passes with the following bugs in place:

* the Park angle negated the wrong way,
* the held angle not being the one the inverse rotation uses,
* `R(theta) R(phi)` folded as `R(theta - phi)`,
* a stage started on the wrong valid.

All four produce perfectly reasonable-looking outputs from every individual
block and all four are caught by comparing the assembled datapath against the
model on 401 updates that include a rotating machine and random angles. It checks
seven internal nodes per update rather than only the outputs, so a mismatch says
which stage went wrong instead of only that the answer differs.

## What tb_foc_closed_loop is for

It is the only testbench that would notice if the design were internally
consistent and still could not control a motor.

It instantiates the top level, configures it over AXI4-Lite exactly as software
would and closes a loop around a per-unit PMSM model integrated by forward Euler
in fifty sub-steps per PWM period. The plant is driven from the compare values the
design actually applies and the currents it produces go back in through a model
of the 12-bit converter.

Two phases:

**Locked rotor.** The classic bench measurement of a current loop. Hold the
rotor, step the torque-current reference, time the response. The exact sampled
analysis in docs/05 predicts a 10 to 90 per cent rise of 0.913 ms and the
testbench asserts the measured figure is within 25 per cent of it, reports both
and additionally requires the steady-state error inside 0.005 per unit, no
significant overshoot and the d axis at zero.

**Turning at 500 rpm.** The back-EMF is 0.151 per unit and the cross-coupling
term another 0.045, neither of which the controller is told about. Both have to be
rejected by the integrators. A sign error anywhere in the Park or inverse Park
path passes every static test in the repository and fails this one.

The rotating phase is given 16 ms to settle and the reason is a result rather
than a fudge: the pole cancellation removes the plant pole from the reference
response but not from the disturbance response, so rejecting the back-EMF
proceeds at `L/R = 4 ms`. See docs/05 and
`tests/test_analysis.py::test_disturbance_rejection_runs_at_the_open_loop_time_constant`.

## What the models are not

The closed-loop testbench runs a 250-step carrier on a 10 MHz clock rather than
2500 on 100 MHz. The control period is still 2 * top * T_clk = 50 us, so the
gains, the machine and every prediction are unchanged; what drops is the
modulator resolution. focmodel puts the resulting change in settled current
below 1e-4 per unit, twenty times inside the tolerance the testbench asserts,
and what it buys is a tenfold cut in the number of clock edges to simulate. The
modulator's own timing is checked at the real ratio in `tb_pwm3ph`, which is
where it belongs. The testbench asserts the carrier and the clock really do give
the 50 us period the gains were derived for, so this cannot silently become a
test of a different loop.

The inverter in the closed-loop testbench is averaged over one PWM period.
Dead time is set to zero there and that is stated in the testbench header rather
than left to be discovered: an averaged plant cannot represent dead-time
distortion faithfully because the sign of the error follows the current. The
switching waveform, including dead time, is measured properly in `tb_pwm3ph`
instead. Splitting it this way covers what a single switching-resolution
closed-loop simulation would, at a fraction of the run time and it makes each
testbench say one thing.

Saying which effects a model does not carry is part of the model.

## Keeping the layers honest

Three mechanisms stop the three layers drifting apart and all three run in CI.

* `scripts/gen_tables.py --check` regenerates `rtl/cordic_atan_pkg.vhd` and
  `tb/machine_pkg.vhd` and fails if the committed files differ. The angle table
  and the machine constants therefore cannot disagree with the models.
* `tests/test_rtl_constants.py` parses the constants out of `rtl/foc_pkg.vhd` and
  compares them with `focmodel/params.py`. A word length changed in one place and
  not the other fails here rather than producing vectors that check the wrong
  arithmetic.
* `scripts/check_claims.py` recomputes every number quoted in the README from the
  models and fails if the text says something else.

## What CI does not do

It does not put the design on an FPGA. There is no timing closure figure, no
resource count from a vendor tool and no measurement on a motor anywhere in this
repository. The synthesis job elaborates the design through GHDL's synthesiser,
which catches a construct that simulates and cannot be built; it is not a
place-and-route and it makes no timing claim.

docs/10-limitations-and-roadmap.md says the same thing at greater length and
lists what would have to happen to change it.
