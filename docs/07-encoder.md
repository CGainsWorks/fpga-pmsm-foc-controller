# 7. The encoder interface

The current loop needs one thing from the rotor: the electrical angle at the
sampling instant. This block produces it from an incremental quadrature encoder.

## Synchronising

A and B arrive from a cable, asynchronous to the fabric clock, so each passes
through three flip-flops before anything looks at it. The mean time between
synchroniser failures is

    MTBF = exp(t_r / tau) / (T0 f_clk f_data)

where `t_r` is the settling time allowed, one clock period here. For a 28 nm
device the metastability time constant `tau` is of the order of 20 ps and the
aperture `T0` of the order of 10 fs. One clock of 10 ns is then about 500 time
constants, and `exp(500)` puts the answer so far beyond any interesting horizon
that the calculation stops being informative.

Two of those numbers are typical values from device literature rather than
measurements on a specific part, and they are quoted as such. The conclusion,
that two stages are ample at 100 MHz and a third is free, does not depend on
either of them being right to better than an order of magnitude.

## Filtering

After the synchroniser, each channel must hold the same value for `G_FILTER`
consecutive clocks before it is believed.

The filter length is bounded from above by the closest two real edges can ever
be. At 6000 rpm on a 2500 line encoder, four counts per line cycle gives

    10000 counts/rev × 6000/60 rev/s = 1e6 counts/s

so real edges are 100 clocks apart at 100 MHz. A four-clock filter is **25 times
shorter than the minimum real edge spacing**, so it cannot delete a real
transition even at the top of the speed range. It rejects anything under 40 ns,
which covers the reflections and the coupled switching noise a motor cable
carries alongside the encoder pair.

If the maximum speed or the line count rose by an order of magnitude the filter
would have to shrink, and the calculation above is where that would be noticed.

## Decoding

A and B are a two-bit Gray code: exactly one of them changes at each valid
transition. Forward rotation walks the pattern

    (A,B) = 00 -> 10 -> 11 -> 01 -> 00

Compare the new A against the old B: it differs on every forward transition and
matches on every reverse one, so the direction is a single exclusive-or and no
state machine is needed.

Both channels changing between two filtered samples is not a valid transition. It
means either that the encoder is turning faster than the filter can follow or
that a line is broken, and in both cases the position is no longer trustworthy.
The decoder raises a sticky error bit rather than guessing a direction, and the
bit stays set until software clears it.

## Position and angle

The position counter is modulo the counts per mechanical revolution, and the
index pulse zeroes it when the index is enabled.

The electrical angle is

    theta = bits 31 downto 16 of (pos × scale),   scale = round(p × 2**32 / cpr)

The mechanics of that expression are worth spelling out. `pos × scale / 2**32` is
`pos × p / cpr`, the number of electrical revolutions turned; taking the low 32
bits of the product discards the whole revolutions, which is the modulo, for
free; and bits 31 downto 16 are the fractional part scaled to a 16-bit binary
angular measure. No comparison, no conditional subtract, no divide.

The error is the rounding of `scale`, at most half a unit in `2**-32` of a turn,
which for a position of up to `cpr` counts is at most

    cpr × 0.5 / 2**16 = 10000 × 0.5 / 65536 = 0.076 of one angle LSB

and, crucially, **it does not accumulate**, because `pos` itself is exact. An
implementation that added a fixed angle increment per count instead would need no
multiplier but would drift by that amount every revolution, and would depend on
an index pulse to stay correct.

`tb_qei_decoder` configures 1024 counts per revolution and one pole pair, which
makes `scale = 2**22` exactly and the expected angle exactly `pos × 64`. The
mapping is then either right or wrong with nothing in between to argue about.

## Speed

Counts per fixed window, the M method. Its quantisation is one count per window
whatever the speed, so the **relative** error grows without bound as the machine
slows:

    error = 1 / (counts in the window) = 1 / (cpr × n × T_win)

At 10000 counts per revolution with a 1 ms window, one count is 6 rpm. That is
0.2 per cent at 3000 rpm and 20 per cent at 30 rpm, and below about 6 rpm the
estimate is zero most windows and one count occasionally.

The fix is the T method, timing a whole number of counts rather than counting in
a whole number of clocks, whose error is `1/(clocks between edges)` and therefore
improves as the machine slows. Production drives run both and cross over between
them, which is the M/T method. It is on the roadmap and it is not implemented,
for one honest reason: **the current loop uses the angle and never the speed**.
Speed only matters to an outer loop, which is not in this repository.
