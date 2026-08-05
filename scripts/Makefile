# Makefile — Vivado xsim simulation flow
#
# Targets:
#   make sim              — compile + elaborate + run in batch mode
#   make gui               — compile + elaborate + launch xsim GUI
#   make clean              — remove all generated artifacts
#   make wave               — re-open saved waveform (after gui run)
#   make fifo-sweep          — FIFO_DEPTH={16,64,256}, always-ready sink
#                              (regression sweep; depth can't affect latency)
#   make fifo-sweep-stall    — same, with SINK_STALL_EN=1 (the sweep that
#                              actually exercises fifo_full — see stream_sink.sv)
#   make sim-all             — baseline + batching + both FIFO sweeps, logged
#                              to $(LOGDIR); prints all [RESULT] rows at the end
#
# Scenario selection (override on command line):
#   make sim TB_SCENARIO=0               # baseline
#   make sim TB_SCENARIO=1               # batching enabled
#   make sim TB_SCENARIO=2 FIFO_DEPTH=16 # FIFO depth sweep
#
# Example FIFO sweep (or just: make fifo-sweep):
#   for d in 16 64 256; do make sim TB_SCENARIO=2 FIFO_DEPTH=$$d; done
#
# NOTE: with an always-ready sink (default), FIFO_DEPTH cannot affect
# latency — the FIFO drains every cycle it holds data, so it never fills.
# To make the depth sweep actually mean something, enable the consumer
# back-pressure model (or just: make fifo-sweep-stall):
#   for d in 16 64 256; do make sim TB_SCENARIO=2 FIFO_DEPTH=$$d SINK_STALL_EN=1; done
# =============================================================================

SHELL := /bin/bash

# --- Tool -------------------------------------------------------------------
XVLOG   := xvlog
XELAB   := xelab
XSIM    := xsim

# --- Directories ------------------------------------------------------------
ROOT    := $(realpath $(dir $(abspath $(lastword $(MAKEFILE_LIST))))/..)
RTL_DIR := $(ROOT)/design/rtl
TB_DIR  := $(ROOT)/verif/tb
WORK    := $(ROOT)/build

# --- Top-level names --------------------------------------------------------
TOP_TB  := tb_pcie_axi_dma
SNAP    := $(TOP_TB)_snap

# --- Scenario parameters (override from command line) ----------------------
TB_SCENARIO  ?= 0
FIFO_DEPTH   ?= 64
BATCH_COUNT  ?= 4
BATCH_TIMEOUT?= 64

# Consumer back-pressure model (needed for FIFO_DEPTH to affect latency —
# see design/rtl/stream_sink.sv). Off by default; enable with SINK_STALL_EN=1.
#   make sim TB_SCENARIO=2 FIFO_DEPTH=16 SINK_STALL_EN=1
SINK_STALL_EN    ?= 0
SINK_STALL_PERIOD?= 4
SINK_STALL_READY ?= 1

# --- Generics passed to xelab -----------------------------------------------
GENERICS := -generic_top "TB_SCENARIO=$(TB_SCENARIO)" \
            -generic_top "FIFO_DEPTH=$(FIFO_DEPTH)"   \
            -generic_top "BATCH_COUNT=$(BATCH_COUNT)"  \
            -generic_top "BATCH_TIMEOUT=$(BATCH_TIMEOUT)" \
            -generic_top "SINK_STALL_EN=$(SINK_STALL_EN)" \
            -generic_top "SINK_STALL_PERIOD=$(SINK_STALL_PERIOD)" \
            -generic_top "SINK_STALL_READY=$(SINK_STALL_READY)"

# --- Source file list (compilation order matters for packages) ---------------
SV_SRCS := \
    $(RTL_DIR)/pcie_axi_pkg.sv     \
    $(RTL_DIR)/pcie_model.sv       \
    $(RTL_DIR)/dma_model.sv        \
    $(RTL_DIR)/axi_mm2s_adapter.sv \
    $(RTL_DIR)/axis_fifo.sv        \
    $(RTL_DIR)/batching_unit.sv    \
    $(RTL_DIR)/stream_sink.sv      \
    $(RTL_DIR)/latency_monitor.sv  \
    $(RTL_DIR)/pcie_axi_dma_top.sv \
    $(TB_DIR)/tb_pcie_axi_dma.sv

# --- WDB waveform database name ---------------------------------------------
WDB := $(WORK)/$(TOP_TB).wdb

# --- Where sweep/regression logs are written --------------------------------
LOGDIR := $(ROOT)/scripts/sim_logs

# ============================================================================
.PHONY: all sim gui wave clean dirs fifo-sweep fifo-sweep-stall sim-all

all: sim

# --- Create build directory -------------------------------------------------
dirs:
	mkdir -p $(WORK)

# --- Compile (xvlog) --------------------------------------------------------
$(WORK)/.compiled: dirs $(SV_SRCS)
	cd $(WORK) && \
	$(XVLOG) --sv --incr \
	    $(foreach f,$(SV_SRCS),$(f)) \
	    -log xvlog.log
	touch $(WORK)/.compiled

# --- Generics fingerprint -----------------------------------------------
# Re-touched only when the actual GENERICS string changes, so switching
# TB_SCENARIO/FIFO_DEPTH/SINK_STALL_*/etc. forces xelab to re-run even
# without a `make clean` in between. Without this, .elaborated below would
# only track $(WORK)/.compiled and silently reuse a stale snapshot built
# with the previous run's generics.
$(WORK)/.generics: dirs
	@if [ ! -f $@ ] || [ "$$(cat $@ 2>/dev/null)" != '$(GENERICS)' ]; then \
	    echo '$(GENERICS)' > $@; \
	fi

# --- Elaborate (xelab) ------------------------------------------------------
$(WORK)/.elaborated: $(WORK)/.compiled $(WORK)/.generics
	cd $(WORK) && \
	$(XELAB) $(TOP_TB) \
	    --snapshot $(SNAP) \
	    $(GENERICS) \
	    --timescale 1ns/1ps \
	    --debug typical \
	    -log xelab.log
	touch $(WORK)/.elaborated

# --- Batch simulation -------------------------------------------------------
sim: $(WORK)/.elaborated
	cd $(WORK) && \
	$(XSIM) $(SNAP) \
	    --runall \
	    --log xsim_batch.log
	@echo ""
	@echo "=== Batch simulation complete. Log: $(WORK)/xsim_batch.log ==="

# --- GUI simulation ---------------------------------------------------------
gui: $(WORK)/.elaborated
	cd $(WORK) && \
	$(XSIM) $(SNAP) \
	    --gui \
	    --wdb $(WDB) \
	    --tclbatch /dev/null \
	    --log xsim_gui.log &

# --- Re-open saved waveform -------------------------------------------------
wave:
	$(XSIM) --gui --wdb $(WDB)

# --- FIFO depth sweep, always-ready sink -------------------------------------
# Depth cannot affect latency here (see stream_sink.sv) — this is the
# baseline/regression sweep, not the meaningful one.
fifo-sweep:
	@mkdir -p $(LOGDIR)
	@set -o pipefail; \
	for d in 16 64 256; do \
	    echo "=== fifo-sweep: FIFO_DEPTH=$$d ==="; \
	    $(MAKE) --no-print-directory sim TB_SCENARIO=2 FIFO_DEPTH=$$d SINK_STALL_EN=0 \
	        2>&1 | tee $(LOGDIR)/fifo_sweep_depth$$d.log | tail -8; \
	done

# --- FIFO depth sweep under a stalled consumer (the meaningful sweep) -------
# Enables SINK_STALL_EN so back-pressure actually reaches the FIFO. Look at
# the FIFO_PEAK/FULL? columns, not LATENCY — LATENCY stays flat across
# depths once the consumer's fixed drain rate is the bottleneck (see
# sim_logs/summary_comparison.log for why that's the correct result).
fifo-sweep-stall:
	@mkdir -p $(LOGDIR)
	@set -o pipefail; \
	for d in 16 64 256; do \
	    echo "=== fifo-sweep-stall: FIFO_DEPTH=$$d ==="; \
	    $(MAKE) --no-print-directory sim TB_SCENARIO=2 FIFO_DEPTH=$$d SINK_STALL_EN=1 \
	        2>&1 | tee $(LOGDIR)/fifo_sweep_stall_depth$$d.log | tail -8; \
	done

# --- Full comparison matrix: baseline + batching + both FIFO sweeps --------
# Runs every scenario this project currently defines and leaves one log per
# run in $(LOGDIR). Does not regenerate the hand-written analysis in
# summary_comparison.log — re-derive that manually if the numbers move.
sim-all:
	@mkdir -p $(LOGDIR)
	@set -o pipefail; \
	echo "=== sim-all: baseline (TB_SCENARIO=0) ==="; \
	$(MAKE) --no-print-directory sim TB_SCENARIO=0 FIFO_DEPTH=64 \
	    2>&1 | tee $(LOGDIR)/scenario0_baseline.log | tail -8
	@set -o pipefail; \
	echo "=== sim-all: batching (TB_SCENARIO=1) ==="; \
	$(MAKE) --no-print-directory sim TB_SCENARIO=1 FIFO_DEPTH=64 \
	    2>&1 | tee $(LOGDIR)/scenario1_batching.log | tail -8
	@$(MAKE) --no-print-directory fifo-sweep
	@$(MAKE) --no-print-directory fifo-sweep-stall
	@echo ""
	@echo "=== sim-all complete. Logs in $(LOGDIR)/ ==="
	@grep -h "\[RESULT\]" $(LOGDIR)/*.log | grep -v '===\|---\|\.\.\.'

# --- Clean ------------------------------------------------------------------
clean:
	rm -rf $(WORK) xsim.dir *.pb *.wdb *.log .Xil

# --- Convenience: print build config ----------------------------------------
info:
	@echo "ROOT         = $(ROOT)"
	@echo "TB_SCENARIO  = $(TB_SCENARIO)"
	@echo "FIFO_DEPTH   = $(FIFO_DEPTH)"
	@echo "BATCH_COUNT  = $(BATCH_COUNT)"
	@echo "BATCH_TIMEOUT= $(BATCH_TIMEOUT)"
	@echo "SINK_STALL_EN    = $(SINK_STALL_EN)"
	@echo "SINK_STALL_PERIOD= $(SINK_STALL_PERIOD)"
	@echo "SINK_STALL_READY = $(SINK_STALL_READY)"
	@echo "Sources:"
	@for f in $(SV_SRCS); do echo "  $$f"; done
