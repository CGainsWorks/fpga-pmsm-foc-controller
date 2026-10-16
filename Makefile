# -----------------------------------------------------------------------------
# fpga-pmsm-foc-controller
#
#   make            everything CI runs, in the order CI runs it
#   make vectors    regenerate the testbench stimulus from the golden models
#   make sim        run every testbench under VUnit (run.py)
#   make sim-one TB=tb_cordic_rot     one testbench
#   make synth      elaborate every unit through the synthesiser
#   make ecp5       synthesise, place and route for the ECP5 target (Docker)
#   make lint       VHDL style and Python lint
#   make test       the Python model test suite
#   make clean
#
# There is no waveform target on purpose.  Every testbench decides for itself
# whether it passed and returns an exit code; none of them needs a person to
# look at a trace.  If you want one, add --wave=x.ghw to GHDL_RUN.
# -----------------------------------------------------------------------------

GHDL    ?= ghdl
PYTHON  ?= python3
STD     ?= 08
WORKDIR ?= sim/build
JOBS    ?= 4

GHDL_FLAGS = --std=$(STD) --workdir=$(WORKDIR) -frelaxed

# Analysis order is dependency order: packages, then leaves, then the top.
RTL = \
	rtl/foc_pkg.vhd \
	rtl/cordic_atan_pkg.vhd \
	rtl/cordic.vhd \
	rtl/clarke.vhd \
	rtl/pi_axis.vhd \
	rtl/svm.vhd \
	rtl/current_loop.vhd \
	rtl/pwm3ph.vhd \
	rtl/qei_decoder.vhd \
	rtl/axi4lite_regs.vhd \
	rtl/foc_top.vhd

TBSRC = $(wildcard tb/*.vhd)

# Every synthesisable unit, leaves before the things that instantiate them.
SYNTH_UNITS = \
	cordic \
	clarke \
	pi_axis \
	svm \
	pwm3ph \
	qei_decoder \
	axi4lite_regs \
	current_loop \
	foc_top

VECTORS = sim/vectors/cordic_rot.txt

.PHONY: all vectors tables analyse sim sim-one synth ecp5 lint lint-vhdl lint-py test clean

all: lint test tables vectors sim synth

$(WORKDIR):
	mkdir -p $(WORKDIR)

# The angle table and the machine constants are generated from focmodel and CI
# checks the committed copies still match rather than regenerating them silently.
tables:
	$(PYTHON) scripts/gen_tables.py --check

vectors: $(VECTORS)

$(VECTORS): scripts/gen_vectors.py focmodel/*.py
	$(PYTHON) scripts/gen_vectors.py

# The RTL only.  The testbenches need the VUnit libraries and run.py compiles
# them, in dependency order, into its own output folder.
analyse: | $(WORKDIR)
	@for f in $(RTL); do \
		echo "  analyse $$f"; \
		$(GHDL) -a $(GHDL_FLAGS) $$f || exit 1; \
	done

# Every testbench, one simulation per test case, through VUnit.  The file list
# lives in run.py and VUnit finds the test cases in it, so there is no second
# list here to forget.  Each testbench still counts its own comparisons and
# fails on any mismatch or on none at all.
sim: vectors
	$(PYTHON) run.py -p $(JOBS) --xunit-xml sim/vunit.xml

sim-one: vectors
	$(PYTHON) run.py "foc_lib.$(TB).*"

# Synthesis smoke test.  This produces no bitstream and makes no timing claim;
# it proves the RTL elaborates through a synthesiser, which is what catches a
# construct that simulates and cannot be built.
#
# Each unit is elaborated on its own rather than only the top. A failure then
# names an entity instead of a design. The leaves are also where the arithmetic
# actually is: the top level is wiring and one strobe.
#
# A crash is reported separately from a diagnostic, because the two mean
# different things. GHDL 5.0.1 used to crash on six of these nine units and
# docs/09-verification.md records how that was tracked down to one function.
synth: analyse
	@fail=""; \
	for u in $(SYNTH_UNITS); do \
		if $(GHDL) --synth $(GHDL_FLAGS) $$u \
			> $(WORKDIR)/$$u.synth.vhd 2> $(WORKDIR)/$$u.synth.log; then \
			echo "  ok    $$u"; \
		elif grep -q 'GHDL Bug occurred' $(WORKDIR)/$$u.synth.log; then \
			echo "  FAIL  $$u  crashed the synthesiser, see docs/09"; \
			fail="$$fail $$u"; \
		else \
			echo "  FAIL  $$u"; \
			sed 's/^/          /' $(WORKDIR)/$$u.synth.log | head -12; \
			fail="$$fail $$u"; \
		fi; \
	done; \
	if [ -n "$$fail" ]; then echo "synthesis failed for:$$fail"; exit 1; fi; \
	echo "every unit elaborated through the synthesiser"


# Yosys with the GHDL plugin, then nextpnr-ecp5, both from pinned container
# images.  Rewrites fpga/ecp5/results.* and fails if 100 MHz is not met.  See
# docs/11-implementation.md.  Needs Docker, not GHDL.
ecp5:
	scripts/ecp5_flow.sh

lint: lint-vhdl lint-py

lint-vhdl:
	vsg -c vsg.yaml -f $(RTL) $(TBSRC)

lint-py:
	ruff check .

test:
	$(PYTHON) -m pytest -q

clean:
	rm -rf $(WORKDIR) sim/vectors/*.txt sim/vunit.xml vunit_out .pytest_cache .ruff_cache
	find . -name '__pycache__' -type d -prune -exec rm -rf {} +
