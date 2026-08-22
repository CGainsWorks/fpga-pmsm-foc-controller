# 6. The modulator: why min-max is space-vector PWM, and what it buys

## The problem

The controller produces a voltage vector. The inverter can produce eight
switching states: six active vectors 60 degrees apart on a hexagon, and two zero
vectors with all three legs on the same rail. Modulation is the business of
producing the wanted vector on average over one switching period from those
eight.

## What the linear range is

The six active vectors have magnitude `2 V_dc / 3` in the amplitude-invariant
convention, and they are the corners of a hexagon. Any vector inside the hexagon
can be synthesised. But a **rotating** vector of constant magnitude must fit
inside the hexagon at every angle, so the usable magnitude is the radius of the
inscribed circle, which touches the mid-point of each edge:

    r = (2 V_dc / 3) cos(30°) = (2 V_dc / 3)(sqrt3/2) = V_dc / sqrt(3) = 0.5774 V_dc

That is the linear limit for the peak phase voltage. Choosing `V_base =
V_dc/sqrt(3)` makes it exactly **1.0 per unit**, which is why that base was
chosen in docs/01 and why no scaling constant appears at any limit in the design.

Plain sinusoidal PWM, with each leg's duty set to `1/2 + v/V_dc`, runs out of rail
when the peak phase voltage reaches `V_dc/2`, that is `0.866` per unit. The ratio

    (V_dc/sqrt3) / (V_dc/2) = 2/sqrt(3) = 1.1547

is the well-known **15.47 per cent** of extra fundamental voltage. On a motor
drive that is 15.47 per cent more speed before field weakening has to start, and
at a given power roughly 13 per cent less current and so about 25 per cent less
conduction loss.

## Getting it without a sector decode

The textbook implementation decodes which of the six sectors the vector is in,
computes two dwell times `T1` and `T2` from a pair of trigonometric expressions,
and splits the remainder between the two zero vectors. That is a sector
comparison, two multiplies and a per-sector duty table.

There is a much cheaper route to the same duty cycles.

Adding the same offset to all three phase references changes no line-to-line
voltage:

    (v_a + v_off) - (v_b + v_off) = v_a - v_b

and in a machine whose star point is isolated the line-to-line voltages are the
only thing that drives current. **The common mode is free**: it can be set to
anything without changing the machine current at all.

Spend it on centring. With

    v_off = -(max(v_a, v_b, v_c) + min(v_a, v_b, v_c)) / 2

the three shifted references become symmetric about zero, so the largest and the
smallest are equal and opposite, and the whole set is centred in the available
rail. Centring the references is exactly what splitting the zero-vector time
equally between V0 and V7 does to the duty cycles, so this produces the same
duties as symmetric space-vector PWM. It costs three comparators, an adder and a
shift.

### The headroom, in one line

For a balanced set of peak amplitude `A`, the peak of `v_x + v_off` is
`(sqrt3/2) A` rather than `A`, because the min-max offset removes the triple-n
harmonic content that was pushing the waveform against the rail. The duty is

    d_x = 1/2 + (v_x + v_off) / sqrt(3)          (per unit, V_base = V_dc/sqrt3)

so its excursion is `A/2` about the half rail and `d` stays inside `[0, 1]` for
`A <= 1`. Without the offset the excursion is `A/sqrt(3)` and the limit is
`A <= sqrt(3)/2 = 0.866`. The 15.47 per cent falls straight out.

`tests/test_foc.py::test_injection_buys_the_expected_headroom` measures both
limits by search rather than asserting the ratio, so the number is confirmed
rather than restated. `tb_svm` checks the centring property directly on the RTL:
whenever the reference is inside the linear range, `max(cmp) + min(cmp)` must
equal the carrier top to within the rounding of one halving.

## The carrier

An up-down counter running `0, 1, ... top, top-1, ... 1` and back to `0` has a
period of `2 top` clocks. At 100 MHz with `top = 2500` that is 20 kHz.

A leg is high while `cnt < cmp`. Counting the states of one period, the counter
takes the values `1..top` on the rising ramp and `top-1..0` on the falling one, so
the high time is

    2 cmp - 1  clocks for cmp >= 1, and 0 for cmp = 0

The half-clock shortfall is **identical on all three legs**, so it is a
common-mode term: it cancels in every line-to-line voltage and a machine with an
isolated star point cannot see it. Its size is one part in 5000, or 9.6 mV on a
48 V link, and it would be invisible even if it were not common mode. It is worth
writing down because a modulator that was out by half a clock **differentially**
would be a real fault, and knowing which of the two you have requires having done
the count.

`tb_pwm3ph` measures the high time in clocks and compares it against `2 cmp - 1`
exactly, rather than comparing a duty against a percentage, because a systematic
half clock is precisely the sort of thing a percentage hides.

## Where the current is sampled

The strobe is issued at the carrier trough. At that instant every leg is on the
same side of its comparator, so the inverter is applying a zero vector and the
line-to-line voltage is zero.

The reason that matters is not the switching state, it is the current. Between
switching instants the phase current is a straight line, because the machine is
inductive and the applied voltage is constant. A centre-aligned carrier makes
those straight-line segments symmetric about the trough, so the instantaneous
current at the trough is equal to the **mean** current over the period. The
switching ripple is not filtered out; it is not sampled in the first place.

This is why no anti-alias filter appears anywhere in the design. A filter would
have solved the same problem by adding phase lag to the measurement, and phase
lag in the feedback path is precisely what a current loop cannot afford.

## Update timing

New compare values are taken into shadow registers at the same trough that issues
the strobe, so the answer computed from the sample at trough `k` is applied over
the whole of period `k+1`. Never part way through a period: changing a compare
value mid-period produces a pulse of arbitrary width, which is a real source of
current spikes in drives that get it wrong.

The consequence is the one-and-a-half sample delay analysed in docs/05.

## Dead time

Both transistors of a leg are held off for `deadtime` clocks after every
transition. The requirement is

    t_dead > t_off(max) + t_skew(driver) - t_on(min)

using the worst-case turn-off of the outgoing device and the best-case turn-on of
the incoming one, plus the propagation mismatch between the two gate-driver
channels. For a 48 V MOSFET half bridge with a typical driver, that is on the
order of 200 to 400 ns; **1.0 us, 100 clocks, is generous** and is chosen to be so
because shoot-through destroys the leg while excess dead time only distorts.

The cost of that generosity is measurable:

    t_dead / T_pwm = 1.0 us / 50 us = 2 per cent
    voltage error  = 2 per cent of 48 V = 0.96 V

against a duty quantisation of `V_dc / top = 19.2 mV`. The dead-time distortion is
**fifty times larger than the PWM resolution**, which settles the question of
where the next improvement is: not more carrier bits, but dead-time compensation.
That is why it is the first power-stage item in
docs/10-limitations-and-roadmap.md and why the closed-loop testbench runs with
dead time set to zero and says so.

`tb_pwm3ph` measures the both-off interval in clocks, checks there are exactly two
per period, and checks on every clock of the run that the two transistors of a leg
are never on together.

### Narrow pulses

A leg asked for a duty so close to the rail that its off time is shorter than the
dead time cannot be given that duty by any inverter: the pulse is swallowed. The
testbench pins this down as an inequality rather than an equality, because the
exact loss depends on where the requested edge falls against the clock. Minimum
pulse-width handling sits next to dead-time compensation on the roadmap.
