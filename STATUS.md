# Status

Last updated 2026-10-16.

Nothing in this repository has been on an FPGA or turned a motor. Everything
below marked complete is complete **in simulation or in open-tool synthesis**
and says which.

## Complete

- **RTL.** A VHDL-2008 FOC current-loop coprocessor: CORDIC, Clarke, PI with
  anti-windup, min-max space-vector modulation, centre-aligned PWM with dead
  time, quadrature encoder decoder and an AXI4-Lite register file. The datapath
  is pipelined for a 100 MHz clock with a latency of 92 clocks.
- **Bit-accurate models.** `focmodel/` performs the same integer arithmetic as
  the RTL. 71 Python tests (`pytest`) check the models and the control analysis
  and that the RTL constants match the model's.
- **Testbenches.** Ten self-checking testbenches, run by VUnit 4.7.1 (pinned) as
  13 test cases under GHDL 5.0.1. The block-level ones compare against the
  models for equality; `tb_current_loop` also measures the 92-clock latency.
- **Closed loop through the encoder, simulated.** `tb_foc_closed_loop` closes the
  current loop around a per-unit PMSM model. A shaft and encoder model drives
  the A and B pins of a 2500 line encoder at an imposed speed and the design
  takes its angle from its own quadrature decoder. Four cases: locked-rotor step
  against the predicted rise time, 500 rpm forward, 500 rpm in reverse (both
  also checking the decoder's position, speed and angle registers against the
  shaft) and A and B swapped, which must lose control.
- **Synthesis, place and route, open tools.** GHDL, Yosys and nextpnr-ecp5 from
  pinned container images, for a Lattice ECP5 LFE5U-45F-8BG381C. Tool estimate:
  105.76 MHz against the 100 MHz constraint, 16843 logic cells, 7487 flip-flops,
  33 multipliers. CI fails if 100 MHz is not met or the committed figures in
  `fpga/ecp5/` go stale.
- **Consistency checks in CI.** Generated VHDL is regenerated and compared,
  every figure in the README is recomputed from the models, every unit is
  elaborated through GHDL's synthesiser, VHDL and Python style are checked.

## In progress

- Nothing.

## Not started

- Vendor synthesis and timing sign-off (Lattice Radiant or Diamond for this
  ECP5, or Vivado or Quartus on another part) with a board constraint file.
- Any hardware: an evaluation board, an inverter, a motor, a current probe.
- Fault inputs: overcurrent trip, desaturation, gate-supply monitoring.
- An SPI front end for a real ADC; the converter is a quantising model in the
  testbench.
- Back-EMF and cross-coupling feedforward, field weakening, dead-time
  compensation. See `docs/10-limitations-and-roadmap.md`.
- A mechanical model (inertia and load torque) in the closed-loop testbench.
- A cocotb or C-driver co-simulation of the register map from software.

## Limitations

- **No hardware result of any kind.** No bitstream has been programmed, no
  current has been measured.
- **Timing is an open-tool estimate for one device**, with under 6 per cent
  margin, I/O placed by the tool and unconstrained and no power estimate. It is
  not a vendor sign-off.
- **Synthesis GHDL is not simulation GHDL.** The ECP5 flow uses the GHDL in its
  pinned image (7.0.0-dev); the testbenches run on GHDL 5.0.1.
- **The closed-loop plant is simplified.** The inverter is averaged with dead time
  set to zero (dead time is tested separately in `tb_pwm3ph`), the speed is
  imposed rather than integrated from torque, the encoder has no jitter or noise
  and the index channel is not exercised in the loop (it is in
  `tb_qei_decoder`). The testbench runs a 10 MHz clock with a 250-step carrier,
  which keeps the 50 us control period.
