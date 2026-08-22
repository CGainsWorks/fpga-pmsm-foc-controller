# 3. The CORDIC, derived

Both rotations in the design and the voltage limiter are one entity,
`rtl/cordic.vhd`, in two modes. It contains no multiplier at all: the only
multiply in the whole block is the single constant one that removes the
processing gain at the output.

## The iteration

    x[i+1] = x[i] - s[i] 2**-i y[i]
    y[i+1] = y[i] + s[i] 2**-i x[i]
    z[i+1] = z[i] - s[i] a[i]              a[i] = atan(2**-i),  s[i] in {-1, +1}

Write the first two lines as a matrix:

    [x']   [ 1        -s 2**-i ] [x]
    [  ] = [                   ] [ ]
    [y']   [ s 2**-i   1       ] [y]

and factor out `sqrt(1 + 2**-2i)`:

    [x']                    [ cos(s a[i])  -sin(s a[i]) ] [x]
    [  ] = sqrt(1 + 4**-i)  [                           ] [ ]
    [y']                    [ sin(s a[i])   cos(s a[i]) ] [y]

because `cos(atan(t)) = 1/sqrt(1+t²)` and `sin(atan(t)) = t/sqrt(1+t²)`. So one
step is **a rotation by ±atan(2**-i) together with a scaling by
sqrt(1 + 4**-i)**. The rotation angle is chosen by the data through `s[i]`; the
scaling is not. After `n` steps the vector has turned by `Σ s[i] a[i]` and grown
by

    K_n = Π_{i=0}^{n-1} sqrt(1 + 4**-i)

whatever the `s[i]` were. That data independence is the whole point of the
algorithm: the gain can be removed afterwards by a single constant multiply,
which is why an arbitrary rotation costs shifts and adds instead of four
multiplies.

`K_n` converges quickly. At 18 stages it is `1.6467602581`, within 1.6e-11 of its
limit, which is `2**-36` of the gain and invisible in a 16-bit result.

## The two modes

**Rotation**, `s[i] = sign(z[i])`, drives `z` to zero. Since `z` decreases by the
angle actually applied at each step, driving it to zero means the total applied
rotation equals the angle presented on `z[0]`. The result is the input vector
rotated by that angle, scaled by `K`.

**Vectoring**, `s[i] = -sign(y[i])`, drives `y` to zero. A vector with `y = 0`
lies on the positive `x` axis, so `x` ends up holding the magnitude, scaled by
`K`, and `z` has accumulated the angle it took to get there, which is the
argument of the input.

The two differ in one comparison. Everything else, including the fold and the
output stage, is shared.

## Convergence, and why the fold is needed

A step can only ever subtract the angle it is given, so the algorithm resolves an
angle `θ` if and only if every partial remainder can still be reached by the
steps that remain. The sufficient condition is

    a[i] <= Σ_{j>i} a[j] + a[n-1]

which the binary angle set satisfies for every `i`; the test
`tests/test_cordic.py::test_every_step_is_reachable_from_the_ones_after_it`
checks it directly rather than taking it on trust. The total reach is

    Σ_{i=0}^{17} atan(2**-i) = 1.743278 rad = 99.88 degrees

which is more than a quadrant and less than a half turn. So the iteration alone
covers `±99.88°` and cannot cover the circle.

The fix is one pre-rotation by a quarter turn. A quarter turn maps `(x, y)` to
`(-y, x)` or `(y, -x)`: an exchange and a negate, no arithmetic error, no gain,
no hardware beyond a multiplexer. After folding, the residual angle is at most
`π/2 = 1.5708 rad`, against a reach of 1.7433, leaving **0.17 rad of margin**. The
margin is worth noting because it means convergence does not depend on the exact
stage count; dropping to 12 stages would still converge, it would simply be less
accurate.

In vectoring mode the same fold is applied on the sign of `x`, because vectoring
converges only in the right half plane, and the quarter turn it took out is
seeded into the accumulator so that the reported argument is still that of the
original vector.

## How many stages

The stage count is set by requiring that the arithmetic contribute less angle
error than the interface already carries. After `n` steps the unresolved residual
is at most `atan(2**-(n-1))`, and it should sit below half an LSB of the 16-bit
angle input:

    atan(2**-(n-1)) <= 0.5 × 2π / 2**16 = 4.79e-5 rad
    n >= 16.4  ->  n = 18 with a little margin

At `n = 18` the residual is `atan(2**-17) = 7.63e-6 rad`, which is **0.08 of one
angle LSB**. Adding stages beyond this improves nothing, because the input angle
is already quantised more coarsely than the residual; removing them would make
the algorithm, rather than the interface, the limiting factor.

## The error budget

For a rotation, with each input component within ±2 per unit so the vector
magnitude is at most 2.83:

| Source | Size | In output LSB |
| --- | --- | --- |
| Residual angle, atan(2**-17) = 7.63e-6 rad | × 2.83 | 0.18 |
| Angle table, 18 entries rounded to 2**-21 of a half turn | 1.35e-5 rad × 2.83 | 0.31 |
| Shift truncation, 18 stages at 2**-19, amplified by K | 8.0e-5 pu | 0.65 |
| Output rounding, half an LSB by construction | | 0.50 |
| **Worst case, all adding** | | **1.64** |

The bound is loose because the four terms cannot all reach their worst case with
the same input. The measured worst case over the committed 2000-vector set is
**0.63 LSB**, reported by `tb_cordic_rot` on every CI run, and the testbench fails
above 2 LSB. Both the derived bound and the measured figure are recorded because
a bound with no measurement is a guess and a measurement with no bound is an
anecdote.

The gain compensation constant deserves its own line. `1/K = 0.6072529350` is held
in Q0.17 as 79594, a relative error of 1.8e-6. At Q0.15 it would have been 19898,
a relative error of 1.8e-5, which showed up in the model as a systematic
**-0.27 LSB bias on every magnitude**: small, but a bias rather than noise, and
therefore visible to anything downstream that integrates. Two extra bits on one
constant removed it.

## Cost

Per instance, at 18 stages: 18 registers of 24 + 24 + 22 = 70 bits, so about 1260
flip-flops and a similar number of LUTs for the adders, plus two DSP slices for
the output gain compensation. The design instantiates three.

That is more than it needs to be. Each instance is used once per 5000 clocks, so
a folded implementation reusing a single stage 18 times would cost about a
eighteenth of the logic and still finish in 18 clocks, and one shared instance
could serve all three uses. The pipelined form is kept because it is what makes
the block reusable at full throughput, which is the multi-axis argument in
docs/01, and because it is the version worth publishing. The trade is listed in
docs/10-limitations-and-roadmap.md with the numbers attached.
