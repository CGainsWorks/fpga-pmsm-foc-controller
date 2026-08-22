"""Bit-accurate model of ``rtl/cordic.vhd``.

The CORDIC iteration, in the circular coordinate system, is

    x_{i+1} = x_i - s_i * 2**-i * y_i
    y_{i+1} = y_i + s_i * 2**-i * x_i
    z_{i+1} = z_i - s_i * atan(2**-i)          s_i in {-1, +1}

One step is a rotation by ``s_i * atan(2**-i)`` followed by a scaling by
``sqrt(1 + 2**-2i)``, because

    [x'; y'] = [[1, -s 2^-i], [s 2^-i, 1]] [x; y]
             = sqrt(1 + 2^-2i) * R(s * atan(2^-i)) [x; y]

so after ``n`` steps the vector has been rotated by ``sum s_i atan(2**-i)`` and
scaled by the constant ``K_n = prod sqrt(1 + 2**-2i)``, independent of the
``s_i``.  That constant gain is the whole point of the algorithm: the rotation
angle is data dependent but the gain is not, so it can be undone by a single
constant multiply.

Two ways of choosing ``s_i``:

* rotation mode, ``s_i = sign(z_i)``, drives ``z`` to zero and therefore rotates
  the input vector by the angle presented on ``z_0``;
* vectoring mode, ``s_i = -sign(y_i)``, drives ``y`` to zero and therefore
  leaves the vector magnitude in ``x`` and its argument in ``z``.

Both modes converge only while the angle still to be resolved is within
``sum_{i>=0} atan(2**-i) = 1.7433 rad``, because from step ``i`` onwards the
remaining steps can contribute at most ``sum_{j>i} atan(2**-j)`` and, for the
binary angle set, that tail always exceeds ``atan(2**-i)`` itself.  1.7433 rad is
larger than ``pi/2`` but smaller than ``pi``, so a single exact pre-rotation by
``+/- pi/2`` (a swap and a negate, no arithmetic error at all) is enough to cover
the whole circle.
"""

from __future__ import annotations

from .fixed import round_shr, sat, wrap
from .params import (
    GUARD_BITS,
    KINV_Q,
    N_STAGES,
    Q_KINV,
    Q_Z_SHIFT,
    W_ANG,
    W_DATA,
    W_XY,
    W_Z,
    atan_table,
)

ROTATE = "rotate"
VECTOR = "vector"

_ATAN = atan_table()
_HALF_PI_Z = 1 << (W_Z - 2)      # pi/2 in the Z format


def fold(mode: str, x: int, y: int, z: int) -> tuple[int, int, int]:
    """Bring the residual angle inside the convergence region.

    Exact: a quarter-turn rotation only exchanges and negates coordinates, so it
    introduces no rounding error and no gain.
    """
    if mode == ROTATE:
        if z > _HALF_PI_Z:
            return -y, x, z - _HALF_PI_Z
        if z < -_HALF_PI_Z:
            return y, -x, z + _HALF_PI_Z
        return x, y, z
    if x < 0:
        if y >= 0:
            return y, -x, _HALF_PI_Z
        return -y, x, -_HALF_PI_Z
    return x, y, 0


def iterate(mode: str, x: int, y: int, z: int, n: int = N_STAGES):
    """Run ``n`` CORDIC steps in the internal format, truncating each shift."""
    for i in range(n):
        neg = z < 0 if mode == ROTATE else y >= 0
        xs = x >> i          # arithmetic floor shift, as in the RTL
        ys = y >> i
        if neg:              # s_i = -1
            x, y, z = x + ys, y - xs, z + _ATAN[i]
        else:                # s_i = +1
            x, y, z = x - ys, y + xs, z - _ATAN[i]
    return x, y, z


def cordic(mode: str, x_in: int, y_in: int, ang_in: int, compensate: bool = True):
    """Full fixed-point CORDIC, input and output in Q2.13 and BAM16.

    Returns ``(x_out, y_out, ang_out, saturated)``.  In rotation mode ``ang_out``
    is the unresolved residual, which is a free self-check: it must stay inside
    the bound derived in docs/03-cordic.md.
    """
    x = x_in << GUARD_BITS
    y = y_in << GUARD_BITS
    z = ang_in << Q_Z_SHIFT if mode == ROTATE else 0
    x, y, z = fold(mode, x, y, z)
    x, y, z = iterate(mode, x, y, z)

    if compensate:
        xr = round_shr(x * KINV_Q, Q_KINV + GUARD_BITS)
        yr = round_shr(y * KINV_Q, Q_KINV + GUARD_BITS)
    else:
        xr = round_shr(x, GUARD_BITS)
        yr = round_shr(y, GUARD_BITS)

    saturated = sat(xr, W_DATA) != xr or sat(yr, W_DATA) != yr
    ang_out = wrap(round_shr(z, Q_Z_SHIFT), W_ANG)
    return sat(xr, W_DATA), sat(yr, W_DATA), ang_out, saturated


# The pipeline depth the RTL presents: one input/fold register, one register per
# iteration, one output register.
LATENCY = 1 + N_STAGES + 1

assert W_XY == W_DATA + GUARD_BITS + 2, "internal width must carry 2 growth bits"
