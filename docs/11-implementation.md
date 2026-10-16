# 11. Synthesis, place and route

The figures here come from open-source tools for one named device. They are
**tool estimates, not a vendor sign-off and not a measurement on hardware**. The
production path for this design is the FPGA vendor's own flow (Lattice Radiant
or Diamond for an ECP5, Vivado or Quartus for a Xilinx or Intel part) with a
board constraint file. None of those has been run.

## The flow

`scripts/ecp5_flow.sh`, also `make ecp5`, runs three tools from container images
pinned by digest:

| Step | Tool | What it does |
| --- | --- | --- |
| Elaboration | GHDL through ghdl-yosys-plugin | reads the VHDL-2008 RTL as written, no translation |
| Synthesis | Yosys `synth_ecp5` | maps to ECP5 LUT4, carry, flip-flop and MULT18X18D cells |
| Place and route | nextpnr-ecp5 with Project Trellis | places, routes and runs static timing analysis |

Target: **Lattice ECP5 LFE5U-45F-8BG381C** (speed grade 8), one clock
constrained at **100 MHz**, the fabric clock every number in this repository
assumes. The placer seed is fixed so the run is reproducible.

There is no pin constraint file. nextpnr places the 181 I/O itself and reports
the I/O paths without constraining them, so the figure that means something is
the register-to-register Fmax of the fabric clock. On a board the AXI interface
would come from a soft or hard processor in the same fabric and the converter
and gate signals through registered I/O, so neither is a combinational I/O path
in a real system.

## What CI does with it

The `ecp5` job runs the flow with `--check` on every push. It fails if:

* nextpnr's timing analysis puts the fabric clock below 100 MHz, or
* the utilisation, the synthesised cell counts or the tool versions differ from
  the committed [fpga/ecp5/results.json](../fpga/ecp5/results.json), or the Fmax
  moves by more than 2 per cent.

The second rule is what keeps the published numbers honest: changing the RTL
without regenerating `fpga/ecp5/` fails CI. Regenerate with `make ecp5` and
commit the two files.

## Results

[fpga/ecp5/results.md](../fpga/ecp5/results.md) is the generated table: Fmax,
utilisation after place and route, cells after synthesis, the critical path and
the exact tool versions. It is written by the flow and not edited by hand.

## What the first run found

The first run of the flow on the design as it was put the fabric clock at
26.8 MHz on a speed grade 6 part, against the 100 MHz the design is written
for. Run unit by unit on speed grade 8 it showed where:

| Unit | Fmax before | Longest stage |
| --- | --- | --- |
| `svm` | 38.9 MHz | two multiplies, four adds and three compares in one clock |
| `pi_axis` | 70.0 MHz | the integrator update, add, rounding add, add, compare, select, clamp |
| `cordic` | 84.5 MHz | the gain compensation multiply, rounding and saturation |
| `qei_decoder` | 121.8 MHz | the position times scale multiply |
| `pwm3ph` | 224.0 MHz | counter compare |

Simulation could never have shown this: every testbench passed, because the
arithmetic was right and only its arrangement between registers was too long.

The fix was registers, not different arithmetic.

* **svm** went from 2 stages to 13, each doing one multiply or one carry chain
  or one compare and select.
* **clarke** went from 1 stage to 3, the sum, the multiply and the rounding
  with saturation.
* **cordic** gained two output stages, so the gain multiply, the rounding add
  and the saturation each have a clock.
* **pi_axis** went from 2 stages to 9. The integrator is a recurrence and is
  kept to one clock so that back-to-back samples still update it in order; its
  decisions were taken out of the recurrence by comparing the integrator
  against thresholds computed from the current sample in earlier stages, which
  is exact integer arithmetic. The derivation is in the header of
  `rtl/pi_axis.vhd`.

Every one of these keeps the same operations in the same order, so the
testbenches still compare against focmodel for equality and still pass. The
datapath latency rose from 66 clocks to 92, 0.92 us of a 50 us period, which is
0.105 degrees of phase at the loop bandwidth. `C_LOOP_LATENCY` in
`rtl/foc_pkg.vhd` records it and `tb_current_loop` measures it on every update.

## What it does not establish

* That a vendor flow closes at 100 MHz on this or any other part. Open-tool
  timing for ECP5 uses Project Trellis' timing database, which is derived from
  the vendor's yet it is not the vendor's sign-off.
* Anything about I/O timing, which depends on a board.
* Power. No power estimate is produced.
* That the design works on silicon. Nothing has been programmed into a device.
