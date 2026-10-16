# 8. Register map

AXI4-Lite slave, 32-bit data, byte-addressed, 8 address bits. All registers are
32 bits wide; narrower fields sit in the low bits and are sign-extended on read
where they are signed. Byte strobes are honoured, so a halfword store to one
field does not disturb the other half of the word.

Writes to read-only or unmapped addresses complete with an OKAY response and are
ignored. A SLVERR would be more informative but processor systems commonly
translate it into a bus fault and a register file is not the right place to halt
a CPU from.

## Map

| Offset | Name | Access | Format | Meaning |
| --- | --- | --- | --- | --- |
| 0x00 | CTRL | RW | bits | see below |
| 0x04 | STATUS | RO | bits | see below |
| 0x08 | ID_REF | RW | Q2.13 | d-axis current reference, per unit |
| 0x0C | IQ_REF | RW | Q2.13 | q-axis current reference, per unit |
| 0x10 | KP | RW | Q3.12 | proportional gain, both axes |
| 0x14 | KI | RW | Q1.14 | integral gain per sample, both axes |
| 0x18 | VMAX | RW | Q2.13 | voltage magnitude limit, per unit |
| 0x1C | THETA_MAN | RW | BAM16 | electrical angle when CTRL.MANUAL is set |
| 0x20 | PWM_TOP | RW | unsigned | carrier top, period is 2×TOP clocks |
| 0x24 | DEADTIME | RW | unsigned | dead time in clocks |
| 0x28 | QEI_CPR | RW | unsigned | encoder counts per mechanical revolution |
| 0x2C | QEI_SCALE | RW | unsigned | round(pole_pairs × 2³² / QEI_CPR) |
| 0x30 | QEI_WIN | RW | unsigned | speed measurement window, clocks |
| 0x34 | POS | RO | unsigned | encoder position, modulo QEI_CPR |
| 0x38 | SPEED | RO | signed | counts per QEI_WIN |
| 0x3C | THETA | RO | BAM16 | electrical angle in use |
| 0x40 | ID_MEAS | RO | Q2.13 | measured d-axis current |
| 0x44 | IQ_MEAS | RO | Q2.13 | measured q-axis current |
| 0x48 | VD | RO | Q2.13 | d-axis controller output |
| 0x4C | VQ | RO | Q2.13 | q-axis controller output |
| 0x50 | VERSION | RO | unsigned | 0x00010000 |

## CTRL, 0x00

| Bit | Name | Meaning |
| --- | --- | --- |
| 0 | ENABLE | run the current loop; clearing it also clears the integrators |
| 1 | PWM_EN | enable the gate outputs; clearing it forces all six low |
| 2 | CLEAR | command, self-clearing: clears the integrators and the sticky status bits |
| 3 | MANUAL | take the angle from THETA_MAN instead of from the encoder |
| 4 | USE_INDEX | let the encoder index pulse zero the position |

CLEAR is a command rather than state. Writing a one pulses the internal clear for
one clock and the bit always reads back as zero, so software cannot leave the
design held in reset by forgetting to write the bit back.

PWM_EN is an interlock and not a request. It forces both transistors of every leg
off on the next clock and there is no path in the dead-time inserter that can
drive the two transistors of a leg on together.

## STATUS, 0x04

| Bit | Name | Meaning |
| --- | --- | --- |
| 0 | OVERRUN | a sample arrived while the previous update was still in flight, sticky |
| 1 | SAT_DATA | a datapath stage saturated, sticky within the update |
| 2 | SAT_PI | a controller output is against its limit |
| 3 | OVER_MOD | the demanded voltage is outside the linear modulation range |
| 4 | QEI_ERR | both encoder channels moved between two filtered samples, sticky |
| 5 | QEI_INDEX | an index pulse has been seen since the last clear, sticky |

OVERRUN cannot happen with the design wired as intended, because the modulator
strobes once per 5000 clocks against a datapath latency of 92. It exists so that a
system that reconfigured PWM_TOP to something absurd or drove the sample input
from somewhere else, would find out rather than silently produce a corrupt
result.

## Bringing a drive up

1. Hold CTRL at zero. Write PWM_TOP, DEADTIME, KP, KI, VMAX, QEI_CPR, QEI_SCALE
   and QEI_WIN in any order; nothing depends on their sequence while the enable
   is low.
2. Align the encoder. Set MANUAL, write THETA_MAN to zero, set a small ID_REF
   with IQ_REF at zero and enable. The rotor pulls to the d axis. Clear
   USE_INDEX, then zero the position there.
3. Clear MANUAL to take the angle from the encoder, set ID_REF to zero and drive
   the machine from IQ_REF.

Step 2 is why the manual angle is a mode rather than a debug hook: every
incremental encoder needs its zero found against the rotor magnets once and
open-loop current injection is how it is done.

## Register values for the reference machine

| Register | Value | Derivation |
| --- | --- | --- |
| KP | 5912 | `round(wc × L_pu × 2¹²)`, wc = 2000 rad/s |
| KI | 296 | `round(wc × R_pu × Ts × 2¹⁴)` |
| VMAX | 8192 | 1.0 per unit, the space-vector linear limit |
| PWM_TOP | 2500 | `f_clk / (2 f_pwm)` = 100 MHz / 40 kHz |
| DEADTIME | 100 | 1.0 us at 100 MHz |
| QEI_CPR | 10000 | 2500 line encoder, ×4 decoding |
| QEI_SCALE | 1717987 | `round(4 × 2³² / 10000)` |
| QEI_WIN | 100000 | 1 ms at 100 MHz |
