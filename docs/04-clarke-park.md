# 4. Clarke and Park, derived

## The Clarke transform

Three phase windings sit 120 degrees apart. Their currents produce a rotating
magnetomotive force which, for a machine with no zero-sequence path, is
completely described by two numbers. The Clarke transform is the projection onto
those two:

    [i_alpha]       [1   -1/2      -1/2   ] [i_a]
    [       ] = c   [                     ] [i_b]
    [i_beta ]       [0    sqrt3/2  -sqrt3/2] [i_c]

The rows are the projections of the three winding axes onto a stationary
orthogonal pair, with `alpha` aligned with phase A.

### Why the scaling is 2/3

The constant `c` is a free choice and two are in common use.

With `c = 2/3`, a balanced set of peak amplitude `A` gives a space vector of
magnitude exactly `A`. Substituting `i_a = A cos θ`, `i_b = A cos(θ - 2π/3)`,
`i_c = A cos(θ + 2π/3)`:

    i_alpha = (2/3)(A cos θ + (A/2) cos θ + (A/2) cos θ) = A cos θ
    i_beta  = (2/3)(sqrt3/2)(A cos(θ-2π/3) - A cos(θ+2π/3)) = A sin θ

so `|i| = A`. This is the amplitude-invariant convention.

With `c = sqrt(2/3)` the transform is orthonormal and power is invariant but a
balanced set of peak `A` gives a vector of magnitude `sqrt(3/2) A = 1.225 A`.

This design uses `2/3`. The reason is that **every limit in the system is stated
as a space-vector magnitude**: the current limit, the voltage limit, the
modulation limit. With the amplitude-invariant scaling, "one per unit" means the
same thing on the phase currents, on the space vector and at the modulator and
no conversion factor appears anywhere. With the power-invariant scaling every one
of those limits would need a `sqrt(3/2)` on it and one of them would eventually
be missed. `tests/test_foc.py::test_clarke_is_amplitude_invariant` holds the
implementation to it.

### Why only two current sensors

In a star-connected machine with an isolated neutral there is no return path, so
by Kirchhoff's current law

    i_a + i_b + i_c = 0

The third current is not additional information. Substituting `i_c = -i_a - i_b`
into the matrix collapses it to

    i_alpha = i_a
    i_beta  = (i_a + 2 i_b) / sqrt(3)

which is one multiply and one add. `i_alpha` needs no arithmetic at all.

The saving is not the multiplier. It is a current sensor, a shunt or Hall device,
an amplifier and an ADC channel, per drive, per unit shipped. On a product built
in thousands that is real money and it is the sort of thing that decides whether
a control block is used. The trade is that a broken phase or a leaking neutral is
no longer detectable from the currents alone, which is a protection question and
belongs in the power stage.

`1/sqrt(3) = 0.5773503` is held in Q0.15 as 18919, a relative error of 2.0e-5.
That is a gain error of 0.002 per cent on one axis, three orders of magnitude
below the matching of any real pair of current sensors.

### The inverse

The inverse Clarke, from a two-axis voltage back to three phase references, is
the transpose:

    v_a = v_alpha
    v_b = -v_alpha/2 + (sqrt3/2) v_beta
    v_c = -v_alpha/2 - (sqrt3/2) v_beta

`sqrt(3)/2 = 0.8660254` is held in Q0.15 as 28378, a relative error of 2.8e-6.
Both expressions are computed as a single rounded shift rather than as two
separate roundings, which is worth half an LSB and costs nothing.

## The Park transform

Clarke leaves a vector in a frame fixed to the stator, in which a machine running
at 3000 rpm with four pole pairs presents a 200 Hz sinusoid to the controller. A
PI controller cannot follow a 200 Hz sinusoid without steady-state error; that is
what its finite gain at 200 Hz means.

Park rotates into a frame fixed to the rotor:

    [i_d]   [ cos θ   sin θ] [i_alpha]
    [   ] = [              ] [       ]      = R(-θ) i_alphabeta
    [i_q]   [-sin θ   cos θ] [i_beta ]

In that frame a constant torque is a constant `i_q` and a constant flux demand is
a constant `i_d`, so the controllers see DC and their integrators can drive the
error to zero. This is the entire reason field-oriented control exists.

`i_d` lies along the rotor flux and produces no torque; `i_q` is perpendicular to
it and produces all of it. For a surface machine, where `L_d = L_q`, there is no
reluctance torque, so maximum torque per amp is at `i_d = 0` and that is the
reference used throughout.

### Implementation

Park is `rtl/cordic.vhd` in rotation mode with `-θ` on the angle input. Negating
a binary angular measure is a plain two's-complement negate with the overflow
discarded and it is exact at every value including the end point, because
`-(-π)` gives `-π` and that is the same angle as `+π`.

The inverse Park is the same entity with `+θ` and in this design it is folded
into the voltage limiter: see docs/05.

### The angle must be the same angle

Both transforms use the angle captured at the sampling instant, held in one
register for the 66 clocks the datapath takes. If the forward rotation used one
angle and the reverse another, the two would not compose to the identity and the
controller would be regulating in a frame that was not the frame it applied its
answer in. At 3000 rpm the rotor moves 0.63 degrees in 66 clocks, which sounds
negligible and is not: it would put a systematic cross-coupling between the axes
that no amount of integral gain would remove, because it is a rotation and not an
offset. `tb_current_loop` is the testbench that catches this, because every
block-level test passes with the bug in place.
