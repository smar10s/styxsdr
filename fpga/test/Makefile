# SPDX-License-Identifier: MIT
# fpga/test/Makefile — cocotb simulation for Styx RTL
#
# Usage:
#   make all_tests              — run ALL gate tests
#   make test_axi_build_id     — run one target
#   make waves TARGET=test_xxx — generate VCD for a target
#   make waves-check           — check if latest VCD is stale
#   make lint                  — Verilator lint (advisory)
#   make lint-strict           — Verilator lint (gate check)
#
# Simulator:
#   SIM=verilator (default, compiled, ~6x faster than Icarus)
#   SIM=icarus    (fallback for hierarchical signal access)

SIM ?= verilator
TOPLEVEL_LANG ?= verilog

RTL := $(shell pwd)/../rtl

# Verilator flags
ifeq ($(SIM),verilator)
EXTRA_ARGS += --no-timing +define+SIM \
              -Wno-WIDTHTRUNC -Wno-WIDTHEXPAND -Wno-ASCRANGE \
              -Wno-CASEINCOMPLETE -Wno-UNOPTFLAT -Wno-CMPCONST \
              -Wno-PINMISSING -Wno-MULTIDRIVEN
ifeq ($(VERILATOR_COVERAGE),1)
EXTRA_ARGS += --coverage
endif
ifeq ($(WAVES),1)
EXTRA_ARGS += --trace --trace-underscore
endif
endif

# Parameter overrides for small-buffer sim (faster convergence)
ifeq ($(SIM),verilator)
IQ_DMA_RX_OVERRIDE := EXTRA_ARGS="$(EXTRA_ARGS) -GDDR_BUF_SAMPLES=64"
else
IQ_DMA_RX_OVERRIDE := COMPILE_ARGS="-Piq_dma_rx.DDR_BUF_SAMPLES=64"
endif

.PHONY: all_tests
all_tests: test_axi_build_id test_async_fifo test_adc_sync test_debug_snap \
     test_snap_axi test_iq_dma_rx test_iq_dma_tx test_hil_ctrl

# ---- Per-module targets ----

.PHONY: test_axi_build_id
test_axi_build_id:
	@echo "--- Running test_axi_build_id ---"
	$(MAKE) sim SIM_BUILD=sim_build_axi_build_id \
		TOPLEVEL=axi_build_id \
		VERILOG_SOURCES="$(RTL)/axi_build_id.v" \
		COCOTB_TEST_MODULES=test_axi_build_id

.PHONY: test_async_fifo
test_async_fifo:
	@echo "--- Running test_async_fifo ---"
	$(MAKE) sim SIM_BUILD=sim_build_async_fifo \
		TOPLEVEL=async_fifo \
		VERILOG_SOURCES="$(RTL)/async_fifo.v" \
		COCOTB_TEST_MODULES=test_async_fifo

.PHONY: test_adc_sync
test_adc_sync:
	@echo "--- Running test_adc_sync ---"
	$(MAKE) sim SIM_BUILD=sim_build_adc_sync \
		TOPLEVEL=adc_sync \
		VERILOG_SOURCES="$(RTL)/adc_sync.v $(RTL)/async_fifo.v" \
		COCOTB_TEST_MODULES=test_adc_sync

.PHONY: test_debug_snap
test_debug_snap:
	@echo "--- Running test_debug_snap ---"
	$(MAKE) sim SIM_BUILD=sim_build_debug_snap \
		TOPLEVEL=debug_snap \
		VERILOG_SOURCES="$(RTL)/debug_snap.v" \
		COCOTB_TEST_MODULES=test_debug_snap

.PHONY: test_snap_axi
test_snap_axi:
	@echo "--- Running test_snap_axi ---"
	$(MAKE) sim SIM_BUILD=sim_build_snap_axi \
		TOPLEVEL=snap_axi \
		VERILOG_SOURCES="$(RTL)/snap_axi.v $(RTL)/debug_snap.v" \
		COCOTB_TEST_MODULES=test_snap_axi

.PHONY: test_iq_dma_rx
test_iq_dma_rx:
	@echo "--- Running test_iq_dma_rx ---"
	$(MAKE) sim SIM_BUILD=sim_build_iq_dma_rx \
		TOPLEVEL=iq_dma_rx \
		VERILOG_SOURCES="$(RTL)/iq_dma_rx.v" \
		COCOTB_TEST_MODULES=test_iq_dma_rx \
		$(IQ_DMA_RX_OVERRIDE)

.PHONY: test_iq_dma_tx
test_iq_dma_tx:
	@echo "--- Running test_iq_dma_tx ---"
	$(MAKE) sim SIM_BUILD=sim_build_iq_dma_tx \
		TOPLEVEL=iq_dma_tx \
		VERILOG_SOURCES="$(RTL)/iq_dma_tx.v" \
		COCOTB_TEST_MODULES=test_iq_dma_tx

.PHONY: test_hil_ctrl
test_hil_ctrl:
	@echo "--- Running test_hil_ctrl ---"
	$(MAKE) sim SIM_BUILD=sim_build_hil_ctrl \
		TOPLEVEL=hil_ctrl \
		VERILOG_SOURCES="$(RTL)/hil_ctrl.v" \
		COCOTB_TEST_MODULES=test_hil_ctrl

# ---- Lint ----

LINT_SRCS = $(wildcard $(RTL)/*.v)

LINT_SUPPRESS = -Wno-DECLFILENAME -Wno-UNUSED -Wno-MULTITOP \
                -Wno-PINCONNECTEMPTY -Wno-ASCRANGE -Wno-TIMESCALEMOD

.PHONY: lint
lint:
	@echo "=== Verilator lint (advisory) ==="
	verilator --lint-only --Wall $(LINT_SUPPRESS) $(LINT_SRCS)
	@echo "LINT PASSED (advisory)"

.PHONY: lint-strict
lint-strict:
	@echo "=== Verilator lint (strict gate) ==="
	verilator --lint-only --Wall $(LINT_SUPPRESS) \
		-Wno-WIDTHTRUNC -Wno-WIDTHEXPAND -Wno-CMPCONST \
		-Werror-CASEINCOMPLETE -Werror-UNOPTFLAT \
		$(LINT_SRCS)
	@echo "LINT PASSED (strict)"

# ---- Waveform generation ----

WAVES_DIR := $(CURDIR)/waves
STALE_SECONDS := 600

.PHONY: waves waves-check

waves: ## Generate VCD for TARGET (e.g. make waves TARGET=test_adc_sync)
ifndef TARGET
	@echo "ERROR: specify TARGET=test_xxx (e.g. make waves TARGET=test_adc_sync)"
	@echo "Available:"; grep -E '^\.(PHONY|PHONY):.*test_' Makefile | grep -hoE 'test_[a-z_]+' | sort -u | sed 's/^/  /'
	@exit 1
endif
	@# Trace requires different Verilator build flags — remove stale sim_build
	@rm -rf sim_build_$(patsubst test_%,%,$(TARGET)) sim_build_$(TARGET)
	@echo "=== Generating VCD for $(TARGET) ==="
	@$(MAKE) $(TARGET) WAVES=1
	@mkdir -p $(WAVES_DIR)
	@VCD=$$(find . -maxdepth 2 -name '*.vcd' -newer Makefile 2>/dev/null | head -1); \
	if [ -z "$$VCD" ]; then \
		echo "ERROR: No VCD produced. Check that WAVES=1 is supported."; \
		exit 1; \
	fi; \
	STAMP=$$(date +%Y%m%d_%H%M%S); \
	DEST="$(WAVES_DIR)/$(TARGET)_$${STAMP}.vcd"; \
	cp "$$VCD" "$$DEST"; \
	ln -sf "$$(basename $$DEST)" "$(WAVES_DIR)/$(TARGET)_latest.vcd"; \
	SIZE=$$(du -h "$$DEST" | cut -f1); \
	echo "=== VCD: $$DEST ($$SIZE) ==="

waves-check: ## Warn if latest VCD is stale (>10 min old)
ifndef TARGET
	@echo "ERROR: specify TARGET=test_xxx"
	@exit 1
endif
	@LATEST="$(WAVES_DIR)/$(TARGET)_latest.vcd"; \
	if [ ! -f "$$LATEST" ]; then \
		echo "ERROR: No VCD found for $(TARGET)"; \
		echo "  Generate with: make waves TARGET=$(TARGET)"; \
		exit 1; \
	fi; \
	if [ "$$(uname)" = "Darwin" ]; then \
		MTIME=$$(stat -f %m "$$LATEST"); \
	else \
		MTIME=$$(stat -c %Y "$$LATEST"); \
	fi; \
	NOW=$$(date +%s); \
	AGE=$$((NOW - MTIME)); \
	if [ $$AGE -gt $(STALE_SECONDS) ]; then \
		echo "WARNING: VCD is $$((AGE / 60)) minutes old (stale)"; \
		echo "  File: $$LATEST"; \
		echo "  Regenerate: make waves TARGET=$(TARGET)"; \
		exit 1; \
	else \
		echo "VCD is fresh ($${AGE}s old): $$LATEST"; \
	fi

# ---- cocotb include ----

include $(shell cocotb-config --makefiles)/Makefile.sim
