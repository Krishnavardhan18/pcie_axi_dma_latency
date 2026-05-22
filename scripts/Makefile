# =============================================================================
# Makefile — Vivado xsim simulation flow
#
# Targets:
#   make sim          — compile + elaborate + run in batch mode
#   make gui          — compile + elaborate + launch xsim GUI
#   make clean        — remove all generated artifacts
#   make wave         — re-open saved waveform (after gui run)
#
# Scenario selection (override on command line):
#   make sim TB_SCENARIO=0               # baseline
#   make sim TB_SCENARIO=1               # batching enabled
#   make sim TB_SCENARIO=2 FIFO_DEPTH=16 # FIFO depth sweep
#
# Example FIFO sweep:
#   for d in 16 64 256; do make sim FIFO_DEPTH=$$d; done
# =============================================================================

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

# --- Generics passed to xelab -----------------------------------------------
GENERICS := -generic_top "TB_SCENARIO=$(TB_SCENARIO)" \
            -generic_top "FIFO_DEPTH=$(FIFO_DEPTH)"   \
            -generic_top "BATCH_COUNT=$(BATCH_COUNT)"  \
            -generic_top "BATCH_TIMEOUT=$(BATCH_TIMEOUT)"

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

# ============================================================================
.PHONY: all sim gui wave clean dirs

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

# --- Elaborate (xelab) ------------------------------------------------------
$(WORK)/.elaborated: $(WORK)/.compiled
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
	@echo "Sources:"
	@for f in $(SV_SRCS); do echo "  $$f"; done
