# fpga-pmsm-foc-controller

A field-oriented control coprocessor for permanent-magnet synchronous motors,
written in VHDL. Two phase currents and a rotor angle go in, six gate signals
come out and the whole current loop closes in fabric with no software in it: a
pipelined CORDIC does both frame rotations and the voltage limiting, two
fixed-point PI controllers regulate torque and flux and a min-max modulator
drives centre-aligned PWM with dead time. Configuration and telemetry over
AXI4-Lite.

Every word length, every gain and every constant in it is derived from something
the system already fixes and the derivation is written down next to the number.

> ### No hardware
> This has never been on an FPGA and has never turned a motor. Everything here is
> verified in simulation against documented models. There is no timing closure
> figure, no vendor resource count and no measurement on a machine anywhere in
> this repository and none should be inferred.
> **[docs/10-limitations-and-roadmap.md](docs/10-limitations-and-roadmap.md)**
> sets out exactly what that leaves unclaimed and what would close the gap.

> ### Clean-room implementation
> Built from scratch on personal time from public theory and open-source tooling.
> It is not derived from and contains no material belonging to any employer or
> client: no employer source code, schematics, calibration data, client data or
> internal documentation. The reference machine parameters are a plausible
> 48 V motor chosen so the documentation can quote real numbers; they are not
> anybody's product.

---

## Run it

```bash
pip install -r requirements.txt
make            # lint, model tests, vectors, ten testbenches, synthesis check
```

Needs GHDL 5 or later and Python 3.10 or later. Individual pieces:

```bash
make test       # the Python model test suite
make sim        # analyse and run every testbench
make sim-one TB=tb_foc_closed_loop
make synth      # elaborate every unit through the synthesiser
make lint       # VHDL and Python style
```

---

## What is in it

| File | What it does |
| --- | --- |
| [rtl/cordic.vhd](rtl/cordic.vhd) | 18-stage pipelined CORDIC, rotation and vectoring in one entity, no multiplier in the iteration |
| [rtl/clarke.vhd](rtl/clarke.vhd) | three-to-two phase transform from two current sensors |
| [rtl/pi_axis.vhd](rtl/pi_axis.vhd) | fixed-point PI with conditional-integration anti-windup |
| [rtl/svm.vhd](rtl/svm.vhd) | inverse Clarke and min-max injection to three compare values |
| [rtl/current_loop.vhd](rtl/current_loop.vhd) | the whole datapath, Clarke through to duty |
| [rtl/pwm3ph.vhd](rtl/pwm3ph.vhd) | centre-aligned PWM, complementary outputs, dead time, sample strobe |
| [rtl/qei_decoder.vhd](rtl/qei_decoder.vhd) | quadrature decode, position, electrical angle, speed |
| [rtl/axi4lite_regs.vhd](rtl/axi4lite_regs.vhd) | AXI4-Lite register file |
| [rtl/foc_top.vhd](rtl/foc_top.vhd) | all of the above, wired together by one strobe |
| [focmodel/](focmodel/) | bit-accurate Python models and the control analysis |
| [tb/](tb/) | ten self-checking testbenches |
| [docs/](docs/) | the derivations |

---

## The design, in the order the numbers fall out

Full derivations are in [docs/](docs/); this is the argument in one page.

**Per unit, chosen so the limits mean what they say.** Currents are normalised to
the rated peak phase current and voltages to `V_dc/sqrt(3)`. That second choice is
not cosmetic: `V_dc/sqrt(3)` is the linear modulation limit of space-vector PWM,
so the voltage limit is exactly **1.0 per unit** and no scaling constant appears
at any limit anywhere in the design.
[docs/01](docs/01-system-and-requirements.md)

**Word lengths, from the converter and from the gain.** The data format is Q2.13:
13 fraction bits because the 12-bit converter's LSB is 9.766e-4 per unit and the
arithmetic should sit about eight times finer than the thing it measures and two
integer bits because a drive has to keep measuring through a fault current of
twice rated. The CORDIC runs 24 bits wide internally, four integer bits because
the largest representable input pair has magnitude `sqrt(2) × 4` and the algorithm
multiplies it by `K = 1.6468`, giving 9.32 against a range of 16, so **no
representable input can overflow it**.
[docs/02](docs/02-fixed-point.md)

**CORDIC, 18 stages.** One step is a rotation by `±atan(2**-i)` and a scaling by
`sqrt(1+4**-i)`; the angle is data dependent and the scaling is not, so after `n`
steps the gain is the constant `K` whatever the data did and one constant multiply
removes it. The stage count comes from requiring the unresolved residual,
`atan(2**-17) = 7.63e-6 rad`, to sit below half an LSB of the 16-bit angle input,
so the interface and not the arithmetic is what limits accuracy. The iteration
reaches 1.7433 rad, more than a quadrant, so one exact quarter-turn fold covers
the circle with 0.17 rad of margin. Worst measured rotation error over 2000
vectors: **0.63 LSB**, against a derived bound of 1.64. Every stage is
clock-enabled by the valid bit travelling with it, so the pipeline toggles for
92 clocks in 5000 rather than continuously, which is most of the block's
dynamic power.
[docs/03](docs/03-cordic.md)

**Two current sensors, not three.** With an isolated star point Kirchhoff gives
`i_a + i_b + i_c = 0`, so the third sensor carries no information the first two do
not. Substituting collapses the Clarke transform to `i_alpha = i_a` and
`i_beta = (i_a + 2 i_b)/sqrt(3)`: one multiply and a sensor, a shunt, an
amplifier and an ADC channel removed from every drive built on it.
[docs/04](docs/04-clarke-park.md)

**Gains from one design rule.** Put the PI zero on the plant pole, `Ti = L/R`. The
open loop becomes a pure integrator, `Kp/(sL)`, so `wc = Kp/L` and there is exactly
one free parameter left. It belongs to the sample rate: one sample of computation
delay halves the gain the sampled loop can carry, from `a < 2` to `a < 1` and the
design sits at `a = 0.1`. That gives **81.3 degrees of phase margin and 26.0 dB of
gain margin**. [docs/05](docs/05-current-loop-tuning.md)

**Three predictions of the same step, quoted together.** The continuous design
says the 10 to 90 per cent rise is `ln(9)/wc = 1.099 ms`. The exact sampled
analysis of the same gains, including the delay, says **0.913 ms**. The
bit-accurate model measures 0.950 ms on the 50 us sample grid. `tb_foc_closed_loop`
measures the RTL against the second of those on every CI run.

**Voltage limiting in polar form and the inverse Park for free.** The controller
outputs go through a vectoring CORDIC to give magnitude and angle; the magnitude
is clipped and the angle is left alone, so the applied voltage keeps the direction
the controller asked for instead of being rotated by clipping the axes separately.
Then, since `R(theta) R(phi) [m; 0] = R(theta + phi) [m; 0]`, one rotation does the
limiter reconstruction and the inverse Park together and the angle sum is a plain
16-bit add because a binary angular measure is a residue system modulo one turn.

**Min-max injection is space-vector PWM.** Adding the same offset to all three
legs changes no line-to-line voltage, so in an isolated-neutral machine it is
free. Spending it on `-(max+min)/2` centres the references in the rail, which is
exactly what splitting the zero-vector time equally does to the duty cycles. The
peak of `v + v_off` is `(sqrt3/2) A` instead of `A`, so the linear range ends at
`A = 1.0` instead of `0.866`: **15.47 per cent more fundamental voltage** from the
same DC link, for three comparators instead of a sector decode and two dwell-time
multiplies. [docs/06](docs/06-svpwm-and-pwm.md)

**Sampling at the carrier trough removes the ripple rather than filtering it.**
Between switching instants the phase current is a straight line and a
centre-aligned carrier makes those segments symmetric about the trough, so the
instantaneous current there equals the mean over the period. That is why there is
no anti-alias filter anywhere: a filter would have solved the same problem by
adding phase lag to the feedback path, which is the one thing a current loop
cannot afford.

**Where the delay actually goes.** The datapath takes 92 clocks, 0.92 us, which is
0.105 degrees of phase at the 318 Hz closed-loop bandwidth. The sample-to-apply
delay is 1.5 sample periods, 75 us, which is 8.6 degrees. The arithmetic
contributes **about one per cent** of the loop delay, which is the argument for
putting it in fabric stated as a number. The 92 clocks are what pipelining every
stage for a 100 MHz clock costs; `tb_current_loop` measures them on every update.

---

## Verification

Three layers, each catching what the others cannot.
[docs/09](docs/09-verification.md)

1. **The models test themselves.** A Python suite over CORDIC convergence, the
   amplitude invariance of the Clarke transform, the absence of a dead zone in
   the integrator, the pole-zero cancellation, the margins, the 15.47 per cent.
   These are statements about the mathematics and would be true of any correct
   implementation.
2. **The RTL matches the models exactly.** `focmodel` performs the same integer
   operations in the same order with the same word lengths, so the testbenches
   compare **for equality with no tolerance**. A tolerance would hide the
   off-by-one in a shift that these vectors exist to catch.
3. **The RTL matches the mathematics.** The CORDIC and Clarke testbenches also
   compare against `ieee.math_real` within a bound derived before it is measured
   and report the worst error they saw. Layer two alone would pass if the model
   and the RTL were wrong in the same way.

Ten self-checking testbenches. Each counts its comparisons, prints the count and
fails if any comparison did not hold **or if it ran no comparisons at all**,
because a vector file that failed to open would otherwise look like a pass.

`tb_foc_closed_loop` is the one that would notice if the design were internally
consistent and still could not control a motor: it configures the top level over
its own AXI4-Lite interface and closes a loop around a per-unit PMSM model, first
at locked rotor against the predicted rise time, then turning at 500 rpm where
0.151 per unit of back-EMF and 0.045 of cross-coupling have to be rejected by
integrators that were never told they exist.

Three mechanisms stop the layers drifting apart, all in CI: the angle table and
the machine constants are generated from the models and regenerated to check;
`tests/test_rtl_constants.py` parses the word lengths out of the VHDL and compares
them with the Python; and `scripts/check_claims.py` recomputes every number quoted
in this README from the models and fails if the text says something else.

CI also elaborates all nine units through GHDL's synthesiser, which catches a
construct that simulates and cannot be built. Six of them used to crash the
tool outright; [docs/09](docs/09-verification.md) records how that was reduced
to one function out of four, narrowing a wide operand, plus the reformulation
that fixed it.

---

## Reference machine

A 48 V surface-mount PMSM of the size used in an AGV wheel drive or a small
collaborative-robot joint. The RTL carries no machine constants; these exist so
that every claim above is checkable against something concrete.

| | |
| --- | --- |
| Pole pairs, R, L, flux linkage | 4, 0.5 Ω, 2.0 mH, 0.02 Wb |
| DC link, rated peak current | 48 V, 10 A |
| Fabric clock, PWM, control period | 100 MHz, 20 kHz, 50 us |
| Closed-loop bandwidth | 2000 rad/s, 318 Hz |
| Electrical time constant | 4.0 ms |
| Base speed | 3308 rpm |

---

## Licence

MIT. See [LICENSE](LICENSE).
