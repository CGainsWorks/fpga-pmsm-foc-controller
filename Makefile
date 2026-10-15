# -----------------------------------------------------------------------------
# fpga-pmsm-foc-controller
#
#   make            everything CI runs, in the order CI runs it
#   make vectors    regenerate the testbench stimulus from the golden models
#   make sim        analyse and run every testbench
#   make sim-one TB=tb_cordic_rot     one testbench
#   make synth      synthesis smoke test of the top level
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

GHDL_FLAGS = --std=$(STD) --workdir=$(WORKDIR) -frelaxed

# Errors accumulate and the testbench decides at the end, so simulation must not
# stop on the first one.  Only a failure severity stops a run and the only place
# that is raised is tb_pkg.finish_report.
GHDL_RUN = --assert-level=failure

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

TBSRC = \
	tb/tb_pkg.vhd \
	tb/machine_pkg.vhd \
	tb/tb_cordic_rot.vhd \
	tb/tb_cordic_vec.vhd \
	tb/tb_clarke.vhd \
	tb/tb_pi_axis.vhd \
	tb/tb_svm.vhd \
	tb/tb_current_loop.vhd \
	tb/tb_pwm3ph.vhd \
	tb/tb_qei_decoder.vhd \
	tb/tb_axi4lite_regs.vhd \
	tb/tb_foc_closed_loop.vhd

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

TESTS = \
	tb_cordic_rot \
	tb_cordic_vec \
	tb_clarke \
	tb_pi_axis \
	tb_svm \
	tb_current_loop \
	tb_pwm3ph \
	tb_qei_decoder \
	tb_axi4lite_regs \
	tb_foc_closed_loop

VECTORS = sim/vectors/cordic_rot.txt

.PHONY: all vectors tables analyse sim sim-one synth lint lint-vhdl lint-py test clean

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

analyse: | $(WORKDIR)
	@for f in $(RTL) $(TBSRC); do \
		echo "  analyse $$f"; \
		$(GHDL) -a $(GHDL_FLAGS) $$f || exit 1; \
	done

sim: analyse vectors
	@fail=0; \
	for t in $(TESTS); do \
		echo "=== $$t"; \
		$(GHDL) --elab-run $(GHDL_FLAGS) $$t $(GHDL_RUN) || fail=1; \
	done; \
	if [ $$fail -ne 0 ]; then echo "SIMULATION FAILED"; exit 1; fi; \
	echo "all testbenches passed"

sim-one: analyse vectors
	$(GHDL) --elab-run $(GHDL_FLAGS) $(TB) $(GHDL_RUN)

# Synthesis smoke test.  This produces no bitstream and makes no timing claim;
# it proves the RTL elaborates through a synthesiser, which is what catches a
# construct that simulates and cannot be built.
#
# Each unit is elaborated on its own rather than only the top, for two reasons.
# A failure then names the entity instead of the design; the leaves are also
# where the arithmetic actually lives: the top level is wiring and one strobe.
synth: analyse
	@fail=""; \
	for u in $(SYNTH_UNITS); do \
		if $(GHDL) --synth $(GHDL_FLAGS) $$u \
			> $(WORKDIR)/$$u.synth.vhd 2> $(WORKDIR)/$$u.synth.log; then \
			echo "  ok    $$u"; \
		else \
			echo "  FAIL  $$u"; \
			sed 's/^/          /' $(WORKDIR)/$$u.synth.log | head -12; \
			fail="$$fail $$u"; \
		fi; \
	done; \
	if [ -n "$$fail" ]; then echo "synthesis failed for:$$fail"; exit 1; fi; \
	echo "every unit elaborated through the synthesiser"

lint: lint-vhdl lint-py

lint-vhdl:
	vsg -c vsg.yaml -f $(RTL) $(TBSRC)

lint-py:
	ruff check .

test:
	$(PYTHON) -m pytest -q

clean:
	rm -rf $(WORKDIR) sim/vectors/*.txt .pytest_cache .ruff_cache
	find . -name '__pycache__' -type d -prune -exec rm -rf {} +
