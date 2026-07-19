# SPDX-License-Identifier: MIT
# Makefile — Styx top-level build orchestration
#
# Quick reference:
#   make setup        — One-time: init submodules, build plutosdr-fw (~2hrs)
#   make remote-setup — One-time: sync + build plutosdr-fw on remote host
#   make bitstream    — Build FPGA bitstream (local or remote via config.mk)
#   make firmware     — Cross-compile ARM firmware (Docker)
#   make package      — Create flashable pluto.frm (bitstream + kernel + rootfs)
#   make flash        — Flash pluto.frm to PlutoSDR
#   make validate     — Post-flash validation (AD9361, fingerprint)
#   make deploy       — Deploy firmware binaries to PlutoSDR via SCP
#   make sim          — Run FPGA simulation suite
#   make waves        — Generate VCD waveforms (use TARGET=test_xxx)
#   make test         — Run host-native firmware tests
#   make all          — bitstream + firmware + package
#   make clean        — Remove all build artifacts

.PHONY: all setup remote-setup bitstream firmware package flash validate deploy \
        sim waves test clean help

PLUTO_IP       ?= 192.168.2.1
PLUTO_USER     ?= root
PLUTO_PASS     ?= analog
PLUTO_SSH_OPTS  = -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null

PLUTOSDR_FW    := extern/plutosdr-fw
FW_DOCKER_IMG  := styx-firmware
FW_BUILD_DIR   := build/firmware
HOST_BUILD_DIR := build/host

# Configuration (shared with fpga/Makefile)
-include config.mk
REMOTE_PORT    ?= 22
REMOTE_USER    ?= $(USER)
REMOTE_DIR     ?= ~/styx
ifdef REMOTE
  SSH = ssh -p $(REMOTE_PORT) $(REMOTE_USER)@$(REMOTE)
  RSYNC = rsync -az --delete --exclude-from=.rsync-exclude -e "ssh -p $(REMOTE_PORT)"
endif

all: bitstream firmware package ## Build everything (bitstream + firmware + package)

# ============================================================================
# Setup (one-time)
# ============================================================================

setup: ## One-time: init submodules + build plutosdr-fw base (~2hrs first run)
	@echo "=== Initializing submodules ==="
	git submodule update --init fpga/extern/adi-hdl
	git submodule update --init extern/plutosdr-fw
	@echo "=== Building ADI HDL IP library ==="
	@echo "    (Requires Vivado on PATH. Takes ~5 min first time.)"
	$(MAKE) -C fpga/extern/adi-hdl/library -j$$(nproc 2>/dev/null || sysctl -n hw.ncpu || echo 4)
	@echo "=== Initializing plutosdr-fw dependencies ==="
	cd $(PLUTOSDR_FW) && git submodule update --init --recursive
	@echo "=== Building PlutoSDR firmware base (kernel + rootfs) ==="
	@echo "    First run takes 1-2 hours. Subsequent runs are cached."
	@$(MAKE) _build_plutosdr_fw
	@echo "=== Setup complete ==="

_build_plutosdr_fw:
	@if [ ! -f "$(PLUTOSDR_FW)/buildroot/output/images/rootfs.cpio.gz" ]; then \
		echo "Building plutosdr-fw (kernel + rootfs)..."; \
		if command -v docker >/dev/null 2>&1; then \
			echo "  Using Docker..."; \
			docker build -t plutosdr-fw-builder -f firmware/docker/Dockerfile.plutosdr-fw firmware/docker/; \
			docker run --rm -v "$(abspath $(PLUTOSDR_FW)):/src" plutosdr-fw-builder \
				make -C /src TARGET=pluto -j$$(nproc 2>/dev/null || sysctl -n hw.ncpu || echo 4); \
		else \
			echo "  Docker not found, building natively..."; \
			echo "  (Requires: build-essential, bc, bison, flex, libssl-dev, libelf-dev,"; \
			echo "   cpio, rsync, wget, unzip, u-boot-tools, device-tree-compiler)"; \
			$(MAKE) -C $(PLUTOSDR_FW) TARGET=pluto -j$$(nproc 2>/dev/null || echo 4); \
		fi; \
	else \
		echo "plutosdr-fw already built (cached)."; \
	fi

remote-setup: ## One-time: sync + build plutosdr-fw on remote build host
ifdef REMOTE
	@echo "=== Remote setup on $(REMOTE_USER)@$(REMOTE):$(REMOTE_PORT) ==="
	@echo "    Syncing source tree..."
	$(RSYNC) ./ $(REMOTE_USER)@$(REMOTE):$(REMOTE_DIR)/
	@echo "    Running make setup on remote (~2 hours first time)..."
	$(SSH) "cd $(REMOTE_DIR) && make setup"
else
	@echo "ERROR: REMOTE not set. Configure config.mk or pass REMOTE=host"
	@exit 1
endif

# ============================================================================
# FPGA
# ============================================================================

bitstream: ## Build FPGA bitstream (delegates to fpga/Makefile)
	@$(MAKE) -C fpga bitstream

sim: ## Run FPGA simulation suite
	@$(MAKE) -C fpga sim

waves: ## Generate VCD waveform (use TARGET=test_xxx)
	@$(MAKE) -C fpga waves$(if $(TARGET), TARGET=$(TARGET))

# ============================================================================
# Firmware (ARM cross-compile via Docker)
# ============================================================================

firmware: ## Cross-compile ARM firmware
	@echo "=== Building firmware (ARM cross-compile) ==="
	@if ! docker image inspect $(FW_DOCKER_IMG) >/dev/null 2>&1; then \
		docker build -t $(FW_DOCKER_IMG) -f firmware/docker/Dockerfile firmware/docker/; \
	fi
	@if [ ! -f "$(FW_BUILD_DIR)/CMakeCache.txt" ]; then \
		docker run --rm -v "$(CURDIR):/src" $(FW_DOCKER_IMG) \
			cmake -B /src/$(FW_BUILD_DIR) -S /src/firmware \
			-DCMAKE_TOOLCHAIN_FILE=/src/firmware/cmake/toolchain-pluto.cmake \
			-DCMAKE_BUILD_TYPE=Release; \
	fi
	docker run --rm -v "$(CURDIR):/src" $(FW_DOCKER_IMG) \
		cmake --build /src/$(FW_BUILD_DIR) --parallel
	@echo "=== Firmware build complete ==="
	@ls $(FW_BUILD_DIR)/tools/ 2>/dev/null | grep -v CMake || true

# ============================================================================
# Host-native builds (tests, validation tools)
# ============================================================================

test: ## Build and run host-native firmware tests
	@echo "=== Building host-native tests ==="
	@mkdir -p $(HOST_BUILD_DIR)
	cmake -B $(HOST_BUILD_DIR) -S firmware \
		-DCMAKE_BUILD_TYPE=Debug \
		-DSTYX_HOST_BUILD=ON
	cmake --build $(HOST_BUILD_DIR) --parallel
	@echo "=== Running tests ==="
	ctest --test-dir $(HOST_BUILD_DIR) --output-on-failure

# ============================================================================
# Packaging + Deployment
# ============================================================================

package: build/fpga/system_top.bit ## Create pluto.frm
	@if [ -f build/fpga/timing_status ] && [ "$$(cat build/fpga/timing_status)" != "met" ]; then \
		echo "ERROR: Cannot package — timing status is '$$(cat build/fpga/timing_status)'"; \
		echo "Fix timing violations before packaging. See build/fpga/utilization.rpt"; \
		exit 1; \
	fi
ifdef REMOTE
	@echo "=== Packaging on remote ($(REMOTE)) ==="
	$(SSH) "cd $(REMOTE_DIR) && packaging/package.sh build/fpga"
	@mkdir -p build/fpga
	scp -P $(REMOTE_PORT) $(REMOTE_USER)@$(REMOTE):$(REMOTE_DIR)/build/fpga/pluto.frm build/fpga/
	@echo "=== Downloaded: build/fpga/pluto.frm ==="
else
	@echo "=== Packaging pluto.frm ==="
	@$(MAKE) _package_frm
	@echo "=== Package complete: build/fpga/pluto.frm ==="
endif

_package_frm:
	@if [ ! -f "$(PLUTOSDR_FW)/buildroot/output/images/rootfs.cpio.gz" ]; then \
		echo "ERROR: plutosdr-fw not built. Run 'make setup' first."; exit 1; \
	fi
	@if [ ! -f "build/fpga/system_top.bit" ]; then \
		echo "ERROR: No bitstream. Run 'make bitstream' first."; exit 1; \
	fi
	packaging/package.sh build/fpga

flash: build/fpga/pluto.frm ## Flash pluto.frm to PlutoSDR
	@bin/flash.sh build/fpga/pluto.frm

validate: ## Validate PlutoSDR after flash (AD9361, fingerprint)
	@bin/validate.sh

deploy: ## Deploy firmware binaries to PlutoSDR via SCP
	@echo "=== Deploying firmware to $(PLUTO_IP) ==="
	@BINS=$$(find $(FW_BUILD_DIR) -maxdepth 1 -type f -perm +111 \
		! -name 'CMake*' 2>/dev/null); \
	if [ -z "$$BINS" ]; then \
		echo "ERROR: No firmware binaries found. Run 'make firmware' first."; exit 1; \
	fi; \
	echo "  Uploading:"; \
	for f in $$BINS; do echo "    $$(basename $$f)"; done; \
	sshpass -p $(PLUTO_PASS) scp -O $(PLUTO_SSH_OPTS) \
		$$BINS $(PLUTO_USER)@$(PLUTO_IP):/usr/bin/
	@echo "=== Deploy complete ==="

# ============================================================================
# Cleanup
# ============================================================================

clean: ## Remove all build artifacts
	@$(MAKE) -C fpga clean
	rm -rf $(FW_BUILD_DIR) $(HOST_BUILD_DIR)
	rm -f build/fpga/pluto.frm
	@echo "Cleaned."

# ============================================================================
# Help
# ============================================================================

help: ## Show available targets
	@grep -hE '^[a-zA-Z_-]+:.*?## .*$$' Makefile | sort | \
		awk 'BEGIN {FS = ":.*?## "}; {printf "\033[36m%-20s\033[0m %s\n", $$1, $$2}'
