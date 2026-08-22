# 5. The current loop: gains, margins and what the delay costs

The gains are not tuned by hand and they are not tuned by simulation. They come
from one design rule with one free parameter, and the free parameter is set by
the sample rate. `focmodel/analysis.py` computes every number below and
`tests/test_analysis.py` re-derives them.

## The plant

In the rotor frame, per axis:

    L d(i_d)/dt = v_d - R i_d + w L i_q
    L d(i_q)/dt = v_q - R i_q - w L i_d - w psi_f

The terms in `w` couple the axes and inject the back-EMF. Treated as
disturbances, which is what this design does, each axis is a first-order lag:

    G(s) = 1 / (R + s L)

with a pole at `-R/L`, that is at `-250 rad/s` for the reference machine, and a
time constant of 4 ms.

## The design rule

A PI controller is

    C(s) = Kp (1 + 1/(Ti s)) = Kp (Ti s + 1) / (Ti s)

Choosing `Ti = L/R` puts the controller zero exactly on the plant pole. They
cancel, and the open loop becomes a pure integrator:

    L(s) = C(s) G(s) = Kp / (s L)

whose crossover is at `wc = Kp / L`. So

    Kp = wc L_pu
    Ki = Kp R / L        (per second)
    Ki_per_sample = Kp (R/L) Ts

This is internal model control, or pole cancellation, and its virtue is that it
leaves exactly one free parameter. Everything about the machine has been absorbed
into `Kp` and `Ti`; the only thing left to decide is how fast the loop should be,
and that is a property of the controller and not of the motor.

## Choosing the bandwidth

The bandwidth is limited by the sample rate, not by the machine. The loop samples
once per PWM period and applies its answer over the following period, so there is
one sample of delay beyond the zero-order hold. The usual rule of thumb is
`wc Ts <= 0.1`, and it can be justified rather than quoted.

With the cancellation in place the sampled open loop is

    L(z) = a / (z**d (z - 1)),     a = (Kp + Ki)(1 - p) / R_pu,  p = exp(-Ts R/L)

For `d = 0` the closed loop is `z - 1 + a`, stable for `0 < a < 2`. For `d = 1` it
is `z² - z + a`, and the Jury conditions give `0 < a < 1`. **One sample of delay
halves the gain the loop can carry.** At the design point `a = 0.1006`, a tenth of
the delayed limit, and the margins that buys are:

| | Value |
| --- | --- |
| Gain crossover | 2013 rad/s, against a design target of 2000 |
| Phase margin | 81.3 degrees |
| Gain margin | 26.0 dB |

Both are far above the usual industrial thresholds of 60 degrees and 12 dB. The
loop is deliberately not fast; the headroom is what makes it insensitive to the
30 per cent that a copper winding resistance moves between cold and hot.

## The resulting gains

For the reference machine at `wc = 2000 rad/s`, `Ts = 50 us`:

    Kp = wc L_pu = 2000 × 7.2169e-4 = 1.443376   ->  Q3.12: 5912
    Ki = Kp (R/L) Ts = 1.443376 × 250 × 5e-5 = 0.018042  ->  Q1.14: 296

Quantisation moves both by 0.13 per cent, which moves the closed-loop pole by
0.13 per cent.

The discrete controller uses the current error in its integrator, so it is a
backward-Euler integrator and its zero sits at `z = Kp/(Kp+Ki) = 0.9876378`. The
zero-order-hold plant pole is `exp(-Ts R/L) = 0.9875778`. The cancellation
survives the discretisation to **six parts in a hundred thousand**, which is what
`tests/test_analysis.py::test_the_discrete_zero_lands_on_the_discrete_pole`
asserts.

## Predicted and measured response

Three numbers for the same step, and they should be quoted together because the
difference between them is the point.

| | 10 to 90 per cent rise |
| --- | --- |
| Continuous design, `ln(9)/wc` | 1.099 ms |
| Exact sampled analysis with one sample of delay | 0.913 ms |
| Bit-accurate model, locked rotor, 0.5 pu step | 0.950 ms |
| RTL in `tb_foc_closed_loop`, same step | reported on every CI run |

The continuous figure is the design intent. The sampled figure is what the same
gains actually give once the zero-order hold and the computation delay are
included: the delay moves the dominant pole from 0.8994 to 0.8865 and adds a
second, fast pole at 0.1135, and the net effect on the rise is to shorten it.
That is worth stating plainly, because the instinct is that delay slows a loop
down. It does not slow this one down; it takes away margin and it halves the gain
the loop could carry. The bit-accurate figure sits between the two and is
quantised to the 50 us sample grid, so 0.950 ms is the grid point next to 0.913.

## What the delay actually is

| Contribution | Time | Phase at 318 Hz |
| --- | --- | --- |
| Datapath, Clarke through to compare values, 66 clocks | 0.66 us | 0.076 degrees |
| Sample instant to the centre of the applied PWM period, 1.5 Ts | 75 us | 8.6 degrees |

The arithmetic contributes **one hundredth** of the delay. That is the argument
for putting the loop in fabric stated as a number: a software implementation
taking 10 us would add 1.1 degrees, which is still small, but it would add it
with jitter, and jitter is not something a phase margin can absorb. What is left
is set by the modulator and is the same for any implementation.

## Anti-windup

While the output is against its limit the loop is open, and an integrator that
keeps accumulating during that time has to be unwound again before the output can
come off the limit. On a drive that shows up as a long overshoot after every hard
acceleration.

The design uses conditional integration: if the unsaturated output exceeds the
limit and the error has the same sign, the integrator update is skipped. The
state cannot grow in the direction that is already saturated, so recovery is
immediate, and the proportional term alone takes the output off the limit on the
sample after the error reverses. It costs one comparison.

The state is additionally clamped to `±V_max` in its own format, so that a change
of limit at run time cannot leave a stale value behind that would take thousands
of samples to work off.

`tb_pi_axis` drives the output into the limit for two hundred samples and then
reverses the error, comparing every sample against the model, so a controller
that wound up would diverge from the vectors on the first sample of the recovery.

## The one thing the tuning does not fix

Cancelling the plant pole removes it from the **reference** transfer function. It
does not remove it from the **disturbance** transfer function, because the
back-EMF enters the plant after the controller. Rejecting a back-EMF step
therefore still proceeds at the open-loop electrical time constant, `L/R = 4 ms`,
which is five times slower than the loop follows a reference step.

This is visible in simulation and it is asserted, not hand-waved:
`tests/test_analysis.py::test_disturbance_rejection_runs_at_the_open_loop_time_constant`
shows the current still 16 per cent short of target after four reference rise
times, and settled after three electrical time constants. `tb_foc_closed_loop`
gives its rotating phase 22 ms to settle for the same reason.

It is also exactly why production drives add back-EMF and cross-coupling
feedforward:

    v_d_ff = -w L_q i_q
    v_q_ff = +w (L_d i_d + psi_f)

Those two terms cancel the disturbance at its source rather than waiting for the
integrator to find it, and they turn a 4 ms rejection into a rejection limited
only by how well `psi_f` and `L` are known. It is not implemented here, it needs
three multipliers and two more registers, and it is the first item in
docs/10-limitations-and-roadmap.md.
