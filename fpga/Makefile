# SPDX-License-Identifier: MIT
# fpga/Makefile — FPGA build system for Styx
#
# Targets:
#   make project     — Create Vivado project (BD + sources)
#   make bitstream   — Full build: synth + impl + bitstream
#   make sim         — Run cocotb simulation suite
#   make waves       — Generate VCD waveform (TARGET=test_xxx)
#   make clean       — Remove build artifacts
#
# Configuration:
#   Copy config.mk.example to config.mk (project root) and edit, or pass
#   on command line:
#     make bitstream PLACE_STRATEGY=EarlyBlockPlacement
#     make bitstream REMOTE=paragon.example.com
#
# Remote builds:
#   When REMOTE is set, source is rsynced to the remote, built there,
#   and artifacts are downloaded. The Makefile and TCL are identical
#   in both modes — only execution location changes.

.PHONY: project bitstream sim waves clean help

# ---- Configuration ----
-include ../config.mk

# Vivado
VIVADO          ?= vivado
VIVADO_FLAGS    ?= -mode batch -nojournal -nolog

# Remote builds (empty = local)
REMOTE          ?=
REMOTE_USER     ?= $(USER)
REMOTE_PORT     ?= 22
REMOTE_DIR      ?= ~/styx

# Vivado environment (for remote builds where vivado isn't on PATH)
VIVADO_SETTINGS ?= /opt/Xilinx/2025.2/Vivado/settings64.sh
VIVADO_WRAPPER  ?= env LD_LIBRARY_PATH=/opt/xilinx-fixes:$$LD_LIBRARY_PATH XILINX_LOCAL_USER_DATA=no vivado

# Build parameters
PLACE_STRATEGY  ?= Explore
OOC_MODULES     ?=
INCREMENTAL     ?=

# Directories
ROOT_DIR        := $(abspath ..)
PROJECT_DIR     := $(CURDIR)/project
RTL_DIR         := $(CURDIR)/rtl
TCL_DIR         := $(CURDIR)/tcl
ADI_HDL_DIR     := $(CURDIR)/extern/adi-hdl
OUTPUT_DIR      := $(ROOT_DIR)/build/fpga
SIM_DIR         := $(CURDIR)/test

# ---- Derived ----
SSH             = ssh -p $(REMOTE_PORT) $(REMOTE_USER)@$(REMOTE)
RSYNC           = rsync -az --delete --exclude-from=$(ROOT_DIR)/.rsync-exclude -e "ssh -p $(REMOTE_PORT)"

BUILD_TCL_ARGS  = -project_dir $(PROJECT_DIR) \
                  -rtl_dir $(RTL_DIR) \
                  -adi_hdl_dir $(ADI_HDL_DIR) \
                  -output_dir $(OUTPUT_DIR) \
                  -place_strategy $(PLACE_STRATEGY)

ifneq ($(OOC_MODULES),)
  BUILD_TCL_ARGS += -ooc_modules "$(OOC_MODULES)"
endif

ifneq ($(INCREMENTAL),)
  BUILD_TCL_ARGS += -incremental $(INCREMENTAL)
endif

# ---- Targets ----

help: ## Show available targets
	@grep -E '^[a-zA-Z_-]+:.*?## .*$$' $(MAKEFILE_LIST) | sort | \
		awk 'BEGIN {FS = ":.*?## "}; {printf "\033[36m%-20s\033[0m %s\n", $$1, $$2}'

ifdef REMOTE
# ===== REMOTE BUILD MODE =====

project: _check_adi ## Create Vivado project (remote)
	@echo "=== Remote project creation on $(REMOTE) ==="
	$(RSYNC) $(ROOT_DIR)/ $(REMOTE_USER)@$(REMOTE):$(REMOTE_DIR)/
	$(SSH) "cd $(REMOTE_DIR)/fpga && \
		source $(VIVADO_SETTINGS) && \
		export LD_LIBRARY_PATH=/opt/xilinx-fixes:\$$LD_LIBRARY_PATH && \
		export XILINX_LOCAL_USER_DATA=no && \
		make project REMOTE= \
		VIVADO='vivado'"

bitstream: _check_adi ## Build bitstream (remote)
	@echo "=== Remote build on $(REMOTE) (strategy=$(PLACE_STRATEGY)) ==="
	$(RSYNC) $(ROOT_DIR)/ $(REMOTE_USER)@$(REMOTE):$(REMOTE_DIR)/
	$(SSH) "cd $(REMOTE_DIR)/fpga && \
		source $(VIVADO_SETTINGS) && \
		export LD_LIBRARY_PATH=/opt/xilinx-fixes:\$$LD_LIBRARY_PATH && \
		export XILINX_LOCAL_USER_DATA=no && \
		make bitstream REMOTE= \
		VIVADO='vivado' \
		PLACE_STRATEGY=$(PLACE_STRATEGY) \
		OOC_MODULES='$(OOC_MODULES)' \
		INCREMENTAL='$(INCREMENTAL)'"
	@mkdir -p $(OUTPUT_DIR)
	scp -P $(REMOTE_PORT) $(REMOTE_USER)@$(REMOTE):$(REMOTE_DIR)/build/fpga/system_top.xsa $(OUTPUT_DIR)/
	scp -P $(REMOTE_PORT) $(REMOTE_USER)@$(REMOTE):$(REMOTE_DIR)/build/fpga/system_top.bit $(OUTPUT_DIR)/
	scp -P $(REMOTE_PORT) $(REMOTE_USER)@$(REMOTE):$(REMOTE_DIR)/build/fpga/fingerprint $(OUTPUT_DIR)/
	scp -P $(REMOTE_PORT) $(REMOTE_USER)@$(REMOTE):$(REMOTE_DIR)/build/fpga/timing_status $(OUTPUT_DIR)/
	scp -P $(REMOTE_PORT) $(REMOTE_USER)@$(REMOTE):$(REMOTE_DIR)/build/fpga/styx_impl.dcp $(OUTPUT_DIR)/ 2>/dev/null || true
	scp -P $(REMOTE_PORT) $(REMOTE_USER)@$(REMOTE):$(REMOTE_DIR)/build/fpga/utilization.rpt $(OUTPUT_DIR)/ 2>/dev/null || true
	@echo "=== Build complete ==="
	@echo "Fingerprint: $$(cat $(OUTPUT_DIR)/fingerprint)"

else
# ===== LOCAL BUILD MODE =====

project: _check_adi ## Create Vivado project (local)
	@echo "=== Creating Vivado project ==="
	@mkdir -p $(OUTPUT_DIR)
	$(VIVADO) $(VIVADO_FLAGS) -source $(TCL_DIR)/build.tcl \
		-tclargs $(BUILD_TCL_ARGS) -skip_project
	@echo "=== Project created ==="

bitstream: _check_adi ## Build bitstream (local)
	@echo "=== Building bitstream (strategy=$(PLACE_STRATEGY)) ==="
	@mkdir -p $(OUTPUT_DIR)
	$(VIVADO) $(VIVADO_FLAGS) -source $(TCL_DIR)/build.tcl \
		-tclargs $(BUILD_TCL_ARGS); \
	RC=$$?; \
	if [ $$RC -eq 0 ]; then \
		echo "=== Build SUCCESS (timing MET) ==="; \
		echo "met" > $(OUTPUT_DIR)/timing_status; \
	elif [ $$RC -eq 2 ]; then \
		echo "=== Build FAILED (timing VIOLATED) ==="; \
		echo "violated" > $(OUTPUT_DIR)/timing_status; \
		exit 2; \
	else \
		echo "=== Build FAILED ==="; \
		echo "error" > $(OUTPUT_DIR)/timing_status; \
		exit 1; \
	fi
	@echo "Fingerprint: $$(cat $(OUTPUT_DIR)/fingerprint)"

endif

# ===== COMMON TARGETS =====

sim: ## Run cocotb simulation suite
	@$(MAKE) -C $(SIM_DIR) all_tests

waves: ## Generate VCD waveform (TARGET=test_xxx)
	@$(MAKE) -C $(SIM_DIR) waves$(if $(TARGET), TARGET=$(TARGET))

clean: ## Remove FPGA build artifacts
	rm -rf $(OUTPUT_DIR)/*
	rm -rf $(PROJECT_DIR)/pluto.xpr $(PROJECT_DIR)/pluto.gen \
		$(PROJECT_DIR)/pluto.runs $(PROJECT_DIR)/pluto.srcs \
		$(PROJECT_DIR)/pluto.cache $(PROJECT_DIR)/pluto.hw \
		$(PROJECT_DIR)/pluto.ip_user_files $(PROJECT_DIR)/pluto.sim
	@echo "Cleaned."

# ---- Dependency checks ----

_check_adi:
ifndef REMOTE
	@if [ ! -d "$(ADI_HDL_DIR)/projects/scripts" ]; then \
		echo "ERROR: ADI HDL submodule not initialized."; \
		echo "Run: git submodule update --init fpga/extern/adi-hdl"; \
		exit 1; \
	fi
endif
