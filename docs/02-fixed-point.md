# 2. Word lengths and why each one is the size it is

There are four numeric formats in the design. None of them was chosen by trying
a value in simulation and keeping it. Each one is derived from a quantity the
system already fixes and `tests/test_rtl_constants.py` re-derives the important
ones so a hand edit fails the suite.

## Notation

`Qm.n` is a signed two's-complement number in `1 + m + n` bits: one sign bit, `m`
integer bits, `n` fractional bits. Its value is the stored integer times `2**-n`,
so it covers `[-2**m, 2**m - 2**-n]` in steps of `2**-n`.

## The four formats

| Name | Format | Width | Range | Resolution | Where |
| --- | --- | --- | --- | --- | --- |
| Data | Q2.13 | 16 | [-4, +4) | 1.221e-4 pu | every current and voltage at a block boundary |
| Angle | BAM16 | 16 | one turn | 9.587e-5 rad | every angle |
| CORDIC datapath | Q4.19 | 24 | [-16, +16) | 1.907e-6 pu | inside the rotators only |
| CORDIC angle | BAM22 | 22 | one turn | 1.498e-6 rad | inside the rotators only |

## The data format, Q2.13

**Resolution.** The thing that actually limits current resolution is the
converter, not the datapath. A 12-bit bipolar ADC spanning ±2 per unit has an LSB
of

    2 × 2 / 2**12 = 9.766e-4 pu

The Q2.13 LSB is 1.221e-4 pu, **eight times finer**. That is the right
relationship: the arithmetic should not be what limits the measurement and it
should not be so much finer than the measurement that bits are being paid for
that carry no information. Three guard bits below the converter is the
conventional answer and it is what this is.

**Range.** Two integer bits give ±4 per unit. The requirement comes from three
places and the largest wins:

* transient overcurrent. A drive must still measure and act on, a current of
  twice rated during a fault or a hard acceleration. That needs ±2.
* the Clarke output. `i_beta = (i_a + 2 i_b)/sqrt(3)` reaches `sqrt(3)` times the
  peak for a balanced set and rather more for an unbalanced one.
* the controller output before limiting, which can ask for more voltage than the
  rail can give before the limiter trims it.

Four per unit covers all three. Three integer bits would waste a bit of
resolution; one would clip a real fault current.

## The angle format, BAM16

The binary angular measure puts `2**16` counts in one electrical turn, so the
16-bit signed value `a` means the angle `a π / 2**15` radians and the LSB is
9.587e-5 rad or 0.0055 degrees.

Two properties make this the right representation and not merely a convenient
one.

**Wrapping is the arithmetic, not an error.** Angles live on a circle, so they
form a residue system modulo one turn. A binary angular measure is exactly that
residue system, which means adding two angles is a plain 16-bit adder with the
carry-out discarded and negating one is a plain two's-complement negate. No
comparison against ±π, no correction step and no special case at the wrap point:
`-(-π)` gives `-π`, which is the same angle as `+π`. The design relies on this
twice, in `theta_neg` and in `psi = theta + phi`.

**The resolution is not what limits anything.** An angle error `e` in the Park
transform costs a torque error of `sin(e) ≈ e`, so one LSB is 9.6e-5 in relative
torque, about a thousandth of a per cent. For comparison, a 2500 line encoder
gives 10000 counts per mechanical revolution and for a four pole-pair machine
that is 2500 counts per electrical revolution, an angle LSB of 2.5e-3 rad. The
**encoder is 26 times coarser than the format that carries its output**, which is
the relationship to aim for.

## The CORDIC datapath, Q4.19 in 24 bits

Two things set this width. Both are checked in
`tests/test_rtl_constants.py::test_internal_format_cannot_overflow`.

**Integer bits, from the gain.** The CORDIC scales the vector it rotates by the
constant `K = 1.6467602581`. The largest vector the input format can present is
`(max, max)`, of magnitude `sqrt(2) × 4 = 5.657` per unit and after the gain that
is `9.316`. Four integer bits give ±16, so **no pair of representable inputs can
overflow the datapath** and that is a statement about every possible input and
not about the ones the tests happen to use. Three integer bits would give ±8 and
would overflow on inputs the format allows.

**Fraction bits, from the accumulated truncation.** Each iteration truncates two
shifted operands, so each contributes an error vector of magnitude at most
`sqrt(2)` internal LSB. Subsequent iterations rotate that error and scale it by at
most `K` and the output stage divides by `K`, so over `n` stages the accumulated
error is bounded by

    n × sqrt(2) × 2**-19 × K ≈ 18 × 1.414 × 1.907e-6 × 1.647 = 8.0e-5 pu

which is 0.65 of an output LSB in the worst case and, since the truncations are
close to independent, about 0.15 of one in practice. Six guard fraction bits over
Q2.13 is what buys that. Fewer and the truncation would dominate the output
rounding; more and it would already be invisible.

Together: `1 + 4 + 19 = 24`. That number has a second, unrelated virtue. A Xilinx
DSP48 slice multiplies 25 bits by 18, so the gain-compensation multiply, 24 bits
by the 18-bit constant `1/K`, fits **one slice with no cascade**. Choosing the
width from the arithmetic and finding it lands on the device's natural operand
size is luck but it is worth taking.

## The CORDIC angle accumulator, BAM22

The angle path needs more fraction than the interface because the table entries
are rounded and the roundings accumulate. With `2**21` counts to the half turn,
each of the 18 table entries carries at most half a count of error, so the total
angle error contributed by the table is at most

    18 × 0.5 × π / 2**21 = 1.35e-5 rad

which is 0.14 of one LSB of the 16-bit interface. At BAM20 it would be 0.56 of an
LSB, which starts to matter; at BAM24 it would be 0.03, which nothing would
notice. Two extra bits on one 22-bit adder per stage is cheap enough that taking
the comfortable answer is right.

## The gain formats

| Gain | Format | Range | Resolution | Reference value |
| --- | --- | --- | --- | --- |
| Kp | Q3.12 | [-8, +8) | 2.44e-4 | 1.4434 → 5912 |
| Ki | Q1.14 | [-2, +2) | 6.10e-5 | 0.018042 → 296 |

They differ on purpose. `Kp = wc L_pu` scales with bandwidth and inductance and
can reasonably reach several units, so it needs range. `Ki = wc R_pu Ts` is the
product of three small numbers; for stability `wc Ts` stays below about 0.5 and
`R_pu` below 1, so `Ki` cannot exceed about 0.5 and it needs resolution instead.
Using one format for both would mean either clipping a legitimate `Kp` or
quantising `Ki` four times more coarsely than necessary. At the reference tuning
both land within 0.13 per cent of the ideal, which moves the closed-loop pole by
0.13 per cent, two orders of magnitude below the temperature drift of a copper
winding.

## The integrator accumulator, 32 bits

The integrator is held in the natural format of the `Ki × e` product, Q?.27 and
requantised only on the way out. This is the single most important word length in
the design and the one most often got wrong.

At the reference tuning, an error of one data LSB produces

    Ki_q × 1 = 296 accumulator counts

so the smallest error the converter can resolve still moves the integrator by
nearly three hundred counts and the loop can integrate its way to zero error.
Holding the integrator in the Q2.13 output format instead would need `Ki × e`
above `2**-13`, that is an error of 68 LSB and everything smaller would leave the
state unchanged for ever. The loop would sit with a permanent offset of up to 68
LSB and no amount of time would remove it. That failure is silent: the controller
looks stable, the current looks about right and the steady-state error is simply
wrong. `tests/test_foc.py::test_integrator_has_no_dead_zone` is the guard.

Range: the accumulator is clamped to `±V_max` in the same format, so it holds at
most `4 × 2**27 = 5.4e8` and one update adds at most `2 × 4 × 2**27 = 1.07e9`.
The sum fits a 32-bit signed register with room to spare, which is why the
clamping can be done after the addition rather than before it.

## Rounding

Every requantisation in the design uses round-half-up: add half an LSB, then
arithmetic-shift right. One adder and an error in `(-0.5, +0.5]` LSB instead of
the `[-1, 0]` LSB of plain truncation.

The difference that matters is not the size but the bias. Truncation is a
systematic negative offset and a systematic offset in front of an integrator
becomes a growing output. Round-half-up is unbiased except on exact halves, which
are a measure-zero set in any real signal.

The one place truncation is used deliberately is inside the CORDIC iterations,
where rounding would cost 36 adders and the guard bits already hold the total
below a quarter of an output LSB. That is a deliberate trade with a number
attached, which is the difference between an engineering decision and an
oversight.

## Saturation

Every value that leaves a wide internal format for a narrow one saturates rather
than wrapping. Wrapping would turn a small overcurrent into a full-scale sign
reversal and the controller would then act on it as though it were a
measurement: it would drive the current the wrong way, hard, at exactly the
moment the machine was already in trouble. Saturation turns the same event into a
bounded error the integrator can still work against and it raises a status bit
so the fault is visible rather than merely survived.
