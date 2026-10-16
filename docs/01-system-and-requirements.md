# 1. What this is and the machine it is sized for

## The problem

A permanent-magnet synchronous motor produces torque in proportion to the
component of stator current that is perpendicular to the rotor flux and produces
none at all from the component parallel to it. Field-oriented control is the
arrangement that measures the phase currents, rotates them into a frame that
turns with the rotor, regulates the two components separately and rotates the
answer back out again to drive the inverter. Doing that turns a three-phase
machine into something that behaves like a DC motor: torque follows one number,
flux follows another and both are regulated by ordinary linear controllers.

The arithmetic is not hard. Two rotations, one three-to-two phase map, two PI
controllers and a modulator. What is hard is doing it **on time, every time**.
The current loop has to close once per PWM period, at exactly the same point in
that period, for the lifetime of the drive. A microcontroller can do the
arithmetic in a few microseconds but the instant it samples and the instant it
updates are both at the mercy of interrupt latency, cache behaviour and whatever
else the firmware is doing. Jitter in those instants is indistinguishable, to the
control loop, from noise on the current measurement.

This repository puts the whole loop in fabric. The modulator issues its own
sample strobe at the carrier trough, the datapath answers 92 clocks later and
the modulator takes the answer at the next trough. There is no software in the
loop, no interrupt and no scheduler, so the control period is the PWM period
exactly and the jitter is zero by construction.

## Why the problem is worth solving in hardware

Three reasons, in the order a business would care about them.

**Axis count.** The datapath is fully pipelined and accepts one operation per
clock. One control update takes 92 clocks out of the 5000 in a PWM period, so a
single instance has the throughput for far more than one motor. A six-axis robot
arm or a four-wheel AGV needs one FPGA and one set of gate drivers per leg,
rather than one microcontroller per axis with the synchronisation problem that
brings.

**Determinism.** Every axis in a multi-axis machine samples on the same edge of
the same clock. Cross-axis current measurements are simultaneous, which matters
the moment anything downstream tries to estimate load, detect a collision or
share a DC link.

**Latency headroom for what comes next.** The transform arithmetic costs 0.92 us
of a 50 us period. That leaves the period essentially empty, which is what makes
sensorless observers, high-frequency injection or a fast current-derivative
protection trip possible later without re-architecting anything.

## The reference machine

Every number in this repository is checkable against one concrete machine. It is
a 48 V surface-mount PMSM of the size used in an AGV wheel drive or a small
collaborative-robot joint. The RTL itself carries no machine constants: they all
arrive through registers and the reference machine exists so that the
documentation can quote real figures rather than symbols.

| Quantity | Symbol | Value |
| --- | --- | --- |
| Pole pairs | p | 4 |
| Stator resistance, line to neutral | R | 0.5 Ω |
| Stator inductance | L_d = L_q | 2.0 mH |
| Rotor flux linkage | ψ_f | 0.02 Wb |
| DC link | V_dc | 48 V |
| Rated peak phase current | I_base | 10 A |
| Electrical time constant | L/R | 4.0 ms |
| Base speed, where the back-EMF reaches the rail | | 3308 rpm |

Surface magnets mean L_d = L_q, so there is no reluctance torque and the
maximum-torque-per-amp point is at i_d = 0. That is the operating point the
controller is set up for; salient machines need a d-axis reference that follows
the torque demand, which is a change to the reference generator above this block
and not to the block itself.

## Per unit

Currents are normalised to the rated peak phase current and voltages to

    V_base = V_dc / sqrt(3) = 27.71 V

That choice is not cosmetic. The linear modulation limit of space-vector PWM is a
peak phase voltage of V_dc/sqrt(3), so in these units the voltage limit is
**exactly 1.0 per unit**. The saturation limit in the controller, the clamp in
the voltage limiter and the point at which the modulator raises its
over-modulation flag are then all the same number and none of them needs a
scaling constant to be told what it is.

The machine equation keeps its shape under this normalisation:

    v_pu = R_pu i_pu + L_pu d(i_pu)/dt,   R_pu = R I_base / V_base = 0.1804
                                          L_pu = L I_base / V_base = 721.7 us

so the controller carries no machine constants at all and the same gain
expressions describe any machine once its own bases are used.

## The drive

| Quantity | Value | Why |
| --- | --- | --- |
| Fabric clock | 100 MHz | Comfortable for the datapath, see docs/02 |
| PWM frequency | 20 kHz | Above audibility, below the switching loss knee for 48 V MOSFETs |
| Carrier top | 2500 | 100 MHz / (2 × 20 kHz) |
| Control period | 50 us | One update per PWM period |
| Dead time | 1.0 us, 100 clocks | See docs/06 |
| Current ADC | 12 bit, ±2 pu | LSB 9.77e-4 pu, eight times the datapath LSB |
| Encoder | 2500 line incremental, ×4 | 10000 counts per mechanical revolution |

## Interfaces

* **AXI4-Lite slave**, 32-bit, for configuration and telemetry. The register map
  is docs/08.
* **Two current inputs**, signed Q2.13 per unit, latched on the sample strobe.
* **Quadrature encoder**, A, B and index, asynchronous.
* **Six gate signals**, three complementary pairs with dead time.

## Scope

This is a control block. It is not a safety function, it contains no
desaturation detection, no overcurrent trip, no gate-supply monitoring and no
interlock beyond forcing the gates off on reset and on the enable bit. Those
belong in the power stage and in a separate protection path.

It has also never been on an FPGA or on a motor. Everything in this repository is
verified in simulation against models that are documented here and
docs/10-limitations-and-roadmap.md sets out exactly which claims that leaves
unmade.
