#!/usr/bin/env bash
# -----------------------------------------------------------------------------
# Open synthesis, place and route for one named target, Lattice ECP5
# LFE5U-45F-8BG381C, at the 100 MHz fabric clock the design is written for.
#
#   scripts/ecp5_flow.sh            run the flow, rewrite fpga/ecp5/results.*
#   scripts/ecp5_flow.sh --check    run the flow, fail if the committed
#                                   results no longer match what it produces
#
# Either way the script fails if nextpnr's timing analysis says the 100 MHz
# constraint is not met.  That is the gate CI applies on every push.
#
# GHDL (through the ghdl-yosys-plugin) elaborates the VHDL, Yosys synthesises it
# with synth_ecp5 and nextpnr-ecp5 places and routes it against Project Trellis'
# timing database.  All three come from pinned container images, by digest, so
# the numbers are reproducible from this file alone.
#
# What this is and is not.  The figures are estimates from the open tools for
# this one device.  The production path for a part like this is Lattice's own
# Radiant or Diamond and for a Xilinx or Intel part Vivado or Quartus, with a
# board-level constraint file.  None of those has been run.  There is no pin
# constraint file either: nextpnr places the I/O itself and the I/O paths are
# reported but not constrained, so the figure that means something is the
# register-to-register Fmax of the fabric clock.
# -----------------------------------------------------------------------------
set -euo pipefail

YOSYS_IMAGE="hdlc/ghdl:yosys@sha256:99f4c645c546723fec31c2723f86bb2f6430e0842c3d5b6c8484c65b5b83c4c8"
NEXTPNR_IMAGE="hdlc/nextpnr:ecp5@sha256:79a4cede4ae824d634ed008ce263ca01106ded6ac6e33a035e48e71b9efc69f2"

DEVICE="45k"
PACKAGE="CABGA381"
SPEED="8"
FREQ_MHZ="100"
SEED="1"
TOP="foc_top"

# Analysis order, as in the Makefile.
RTL="rtl/foc_pkg.vhd rtl/cordic_atan_pkg.vhd rtl/cordic.vhd rtl/clarke.vhd \
rtl/pi_axis.vhd rtl/svm.vhd rtl/current_loop.vhd rtl/pwm3ph.vhd \
rtl/qei_decoder.vhd rtl/axi4lite_regs.vhd rtl/foc_top.vhd"

MODE="--write"
if [ "${1:-}" = "--check" ]; then
  MODE="--check"
fi

cd "$(dirname "$0")/.."
OUT="build/ecp5"
rm -rf "$OUT"
mkdir -p "$OUT"

# Docker Desktop on Windows wants a Windows path for the mount and Git Bash must
# not rewrite the container-side paths.  On Linux both are no-ops.
HOST_DIR="$(pwd -W 2>/dev/null || pwd)"
export MSYS_NO_PATHCONV=1
RUN_AS=()
if [ "$(uname -s)" = "Linux" ]; then
  RUN_AS=(--user "$(id -u):$(id -g)")
fi

run() {
  docker run --rm "${RUN_AS[@]}" -e HOME=/tmp -v "$HOST_DIR:/src" -w /src "$@"
}

echo "== synthesis: GHDL plugin and Yosys synth_ecp5"
run "$YOSYS_IMAGE" sh -c "ghdl --version | head -n 1 > $OUT/ghdl.version && yosys -V > $OUT/yosys.version"
run "$YOSYS_IMAGE" yosys -q -m ghdl -l "$OUT/yosys.log" -p \
  "ghdl --std=08 -frelaxed $RTL -e $TOP; synth_ecp5 -top $TOP -json $OUT/$TOP.json; tee -q -o $OUT/yosys_stat.txt stat"

echo "== place and route: nextpnr-ecp5 --$DEVICE --package $PACKAGE --speed $SPEED --freq $FREQ_MHZ"
run "$NEXTPNR_IMAGE" sh -c "nextpnr-ecp5 --version > $OUT/nextpnr.version 2>&1"
run "$NEXTPNR_IMAGE" nextpnr-ecp5 \
  --"$DEVICE" --package "$PACKAGE" --speed "$SPEED" --freq "$FREQ_MHZ" --seed "$SEED" \
  --json "$OUT/$TOP.json" --report "$OUT/report.json" --log "$OUT/nextpnr.log" \
  --timing-allow-fail --quiet

echo "== results"
run "$YOSYS_IMAGE" python3 scripts/ecp5_report.py "$MODE" --build "$OUT" \
  --device "LFE5U-45F-${SPEED}BG381C" --freq "$FREQ_MHZ" --seed "$SEED" \
  --yosys-image "$YOSYS_IMAGE" --nextpnr-image "$NEXTPNR_IMAGE"
