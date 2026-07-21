# StyxSDR

**Pluto's smallest moon** — a minimal FPGA bitstream for the
[PlutoSDR](https://www.analog.com/en/resources/evaluation-hardware-and-software/evaluation-boards-kits/adalm-pluto.html)
(Xilinx Zynq 7010) that replaces the stock ADI IIO/DMA framework with
custom streaming DMA engines.

Continuous IQ streaming to DDR, waveform playback, hardware-in-the-loop
test injection, and a debug capture probe — all in ~4,300 LUTs and 2
DSPs (24.5% of Z-7010).  Designed as a reusable infrastructure platform
that downstream projects extend by connecting processing pipelines to
the muxed IQ outputs.

## Features

- **128 MB RX ring buffer** — continuous ADC capture at 20 MSPS, ~1.7 s depth
- **Two-phase TX** — load waveform (slow), trigger playback (single register write, deterministic)
- **HIL test injection** — mux live ADC vs. DDR playback, rate-matched, for golden-vector validation
- **Debug snap probe** — 1024-deep BRAM capture with configurable trigger position
- **Build fingerprint** — content-addressed SHA-256 of RTL+TCL, readable at runtime
- **Runtime sample rate** — AD9361 FIR decimation (20/10/5 MSPS), no bitstream rebuild

## Releases

Pre-built firmware images are available on
[GitHub Releases](https://github.com/smar10s/styxsdr/releases).

### Flashing (USB mass storage, primary)

1. Download `pluto.frm` from the latest release
2. Connect PlutoSDR to your host via USB
3. The Pluto appears as a USB mass storage drive after ~30 seconds
4. Copy `pluto.frm` to the drive root and wait for the write to finish
5. Eject/unmount the drive — Pluto reboots and flashes automatically

The Pluto LED blinks during flashing.  Wait until the LED stops and the
board reappears (as mass storage or network device).  It is safe to
power-cycle afterward.

**Rev.B Pluto users** — the AD9361 defaults to AD9363A tuning range on
Rev.B boards.  After the first flash, SSH in and run
`bin/fix_ad9361_guardrail.sh` to unlock the full AD9361 range.

### Flashing (SSH, advanced)

Alternative method with checksum verification and proper erase.  Requires
Pluto reachable at `192.168.2.1` and `sshpass` installed.

```bash
# macOS: brew install hudochenkov/sshpass/sshpass
# Linux: apt install sshpass

PLUTO_PASS=analog bin/flash.sh ~/Downloads/pluto.frm
```

### Post-flash Validation

```bash
PLUTO_PASS=analog bin/validate.sh
```

This checks AD9361 initialization and verifies the build fingerprint
matches the release artifact.

## Quick Start

```bash
cp config.mk.example config.mk   # configure build host (see comments inside)

# One-time setup (~2 hours first run — builds kernel + rootfs)
make remote-setup       # remote Linux host with Vivado
# or: make setup        # local Linux with Vivado on PATH

# Build → package → flash
make bitstream          # FPGA synthesis + implementation
make firmware           # ARM cross-compile (Docker)
make package            # creates pluto.frm (bitstream + kernel + rootfs)
make flash              # writes to PlutoSDR via SSH (see bin/flash.sh)
make validate           # checks AD9361 init + fingerprint register
```

### Prerequisites

- **Vivado 2025.2** (synthesis/implementation) — local or on a remote build host
- **Docker** (firmware cross-compile, plutosdr-fw base build)
- **Python 3.11+** with `cocotb` and `find_libpython` (simulation only)
- **Verilator** (RTL lint and simulation only)

### Testing

```bash
make sim                # run all 63 cocotb RTL tests (Verilator)
make test               # host-native firmware unit tests (no cross-compiler needed)
make waves TARGET=test_iq_dma_rx   # generate VCD for a specific module
```

## Architecture

```
AD9361 ADC ─→ [FIR, dec] ─→ adc_sync (CDC) ─→ iq_dma_rx ─→ DDR ring buffer (128 MB)
                                                     │
                                                     ▼
                                                hil_ctrl (mux) ─→ [downstream pipeline]
                                                     │
                                                     ▼
                                                snap_axi (debug probe)

DDR buffer ─→ iq_dma_tx ─→ [FIR, interp] ─→ AD9361 DAC
DDR buffer ─→ hil_ctrl (test playback mode)
```

| Module       | Lines | Purpose |
|--------------|-------|---------|
| **iq_dma_rx** | 595 | ADC-to-DDR via AXI3 HP0. Double-buffered BRAM, 128 MB circular buffer, 20 MSPS sustained. |
| **iq_dma_tx** | 897 | DDR-to-DAC via AXI3 HP2. 16-segment BRAM fill pipeline, zero-bubble drain FSM, speculative wrap AR. |
| **hil_ctrl**  | 427 | HIL test controller. IQ mux between live ADC and DDR playback, rate-matched (1 sample per 5 clocks). |
| **snap_axi**  | 253 | Debug capture probe. 1024-deep BRAM, pre/post trigger split, AXI4-Lite read-back. |
| **adc_sync**  | 79  | CDC from AD9361 l_clk (~20 MHz) to fabric sys_cpu_clk (100 MHz) via gray-code async FIFO. |
| **axi_build_id** | 121 | Read-only 32-bit register. SHA-256 fingerprint of RTL + TCL sources, injected at synthesis. |

### DDR Memory Map

| Region     | Size   | Purpose                |
|------------|--------|------------------------|
| 0x10000000 | 128 MB | RX ring buffer         |
| 0x18000000 | 4 MB   | TX waveform / HIL data |

144 MB reserved from 0x10000000 by device tree overlay (`styx-pluto.dtso`).
Stock ADI DMA engines are disabled.

**IQ sample packing**: each 32-bit DDR word is `{8'b0, imag[11:0], real[11:0]}`.
AXI bursts pack two samples per 64-bit beat (`{word_odd, word_even}`).
Burst length is 16 beats = 128 bytes = 32 IQ samples.

### Sample Rates

The AD9361 internal FIR + HB decimation chain is configured at runtime
via sysfs — no bitstream rebuild needed:

| Decimation | Effective Rate | DDR BW (RX) | Use Case               |
|------------|---------------|-------------|------------------------|
| 1x         | 20 MSPS       | 80 MB/s     | 802.11 (20 MHz BW)     |
| 2x         | 10 MSPS       | 40 MB/s     | General wideband       |
| 4x         | 5 MSPS        | 20 MB/s     | Narrowband (LoRa, BLE) |

## Firmware API

The C firmware runs on the Zynq PS (ARM Cortex-A9) under Linux.  It
provides register access via `/dev/mem` mmap and DMA convenience
functions.

### Headers

| Header       | Purpose |
|--------------|---------|
| `hal.h`      | Register map defines, IQ pack/unpack macros, AD9361 sysfs wrappers |
| `dma_rx.h`   | RX ring buffer: start, stop, read write-pointer, blocking capture |
| `dma_tx.h`   | TX waveform: one-shot, two-phase load/trigger, cyclic, stop |
| `convert.h`  | DDR ↔ float conversion (auto-scale TX, sign-extended RX) |

### Two-Phase TX/RX Loopback

The key DMA pattern — separate the slow work (DDR writes, conversion)
from the fast trigger (single register store):

```c
#include "hal.h"
#include "dma_rx.h"
#include "dma_tx.h"

// Start RX — ring buffer fills continuously
dma_rx_start();

// Phase 1: slow — write waveform to DDR, configure registers
dma_tx_load(tx_real, tx_imag, n_samples, false);

// Snapshot write pointer (time reference)
uint32_t t0 = dma_rx_wr_ptr();

// Phase 2: fast — single register write, returns in µs
dma_tx_trigger();

// Wait for propagation
usleep(tx_duration_us + 2000);

// Read captured IQ directly from ring buffer
volatile uint32_t *rx = hal_ddr_rx_buf();
convert_rx_to_float(&rx[t0], capture_len, out_real, out_imag);
```

The write-pointer snapshot falls between the slow load and the fast
trigger, so the transmitted frame is guaranteed to appear shortly after
`t0` in the ring buffer.

### Blocking RX Capture

```c
float real[4096], imag[4096];
int n = dma_rx_capture(4096, real, imag, 50);  // 50 ms timeout
if (n > 0) {
    // process fresh IQ data...
}
```

`dma_rx_capture()` polls the write pointer at 50 µs intervals and
handles ring-buffer wrap transparently.

### Register-Level Control

All engines expose registers over AXI4-Lite for direct access:

```c
// One-shot TX at register level
hal_reg_write(REG_IQ_DMA_TX_DDR_BASE, DDR_TX_BASE);
hal_reg_write(REG_IQ_DMA_TX_COUNT, n_samples);
hal_reg_write(REG_IQ_DMA_TX_CONTROL, TX_CTRL_ENABLE | TX_CTRL_TRIGGER);
while (!dma_tx_done()) usleep(100);

// Check RX overflow diagnostic
uint32_t status = hal_reg_read(REG_IQ_DMA_RX_STATUS);
uint16_t overflows = (status >> 16) & 0xFFFF;
```

### HIL Test Injection

Golden waveform injection for pipeline validation without RF.
`hil_ctrl` switches the IQ mux from live ADC to DDR playback,
rate-matched to the ADC clock:

```bash
hil_inject -v -w    # load hil_golden.json, verify watermark, capture snap
```

`firmware/vectors/hil_golden.json` contains a 1024-sample test vector:
- Samples 0–2: ASCII watermark (identity check)
- Samples 3–1023: linear chirp 0.5–9.5 MHz, full 12-bit dynamic range

## Project Layout

```
styx/
├── Makefile                Top-level orchestration (make help for targets)
├── config.mk.example      Build config template (remote host, Vivado path, etc.)
├── registers.md           Full register map documentation
├── fpga/
│   ├── Makefile            FPGA build targets (bitstream, sim, lint, waves)
│   ├── rtl/                Custom Verilog modules (7 files)
│   ├── project/            Vivado project (block design, top wrapper, constraints)
│   ├── tcl/                Build procs and batch driver
│   ├── test/               Cocotb + Verilator simulation (63 tests, 8 modules)
│   └── extern/adi-hdl/     ADI HDL IP library (submodule)
├── firmware/
│   ├── src/                HAL + DMA drivers (hal, dma_rx, dma_tx, convert)
│   ├── test/               Host-native unit tests (IQ conversion)
│   ├── tools/              On-target utilities (adc_capture, hil_inject)
│   ├── tools/platform/     Platform test programs (loopback, beacon, DMA test)
│   ├── vectors/            HIL test vectors + generator script
│   ├── docker/             Cross-compile containers (ARM + plutosdr-fw)
│   └── cmake/              Toolchain file
├── packaging/
│   ├── package.sh          Builds pluto.frm (FIT image + MD5 checksum)
│   ├── styx-pluto.dtso     Device tree overlay (reserves DDR, disables ADI DMA)
│   └── patch_adi_loadvals.sh
├── bin/                    Operator scripts (flash.sh, validate.sh)
├── extern/
│   ├── plutosdr-fw/        PlutoSDR firmware base (submodule)
│   └── lib80211/           802.11 PHY library (submodule)
└── build/                  Output artifacts (gitignored)
```

## Build Targets

| Target | Description |
|--------|-------------|
| `make all` | bitstream + firmware + package |
| `make bitstream` | FPGA synthesis + implementation (local or remote) |
| `make firmware` | ARM cross-compile via Docker |
| `make package` | Create flashable pluto.frm |
| `make flash` | Write pluto.frm to PlutoSDR USB mass storage |
| `make validate` | Post-flash check (AD9361 init, build ID register) |
| `make deploy` | SCP firmware binaries to PlutoSDR `/usr/bin/` |
| `make sim` | Run full RTL simulation suite |
| `make test` | Host-native firmware unit tests |
| `make waves` | Generate VCD waveforms (`TARGET=test_xxx`) |
| `make clean` | Remove all build artifacts |
| `make help` | List all targets with descriptions |

## Resource Utilization

Post-route, Vivado 2025.2, Zynq 7010:

| Resource        | Used  | Available | %     |
|-----------------|-------|-----------|-------|
| Slice LUTs      | 4,316 | 17,600    | 24.5% |
| Slice Registers | 6,134 | 35,200    | 17.4% |
| Block RAM Tiles | 3     | 60        | 5.0%  |
| DSP Blocks      | 2     | 80        | 2.5%  |

Leaves 75% of LUTs and 95% of BRAM free for downstream logic.

## Downstream Integration

To use styx as infrastructure for a larger project:

1. Add styx as a git submodule
2. Source `fpga/project/system_bd.tcl` in your block design
3. Connect to `hil_ctrl_0/iq_*` outputs (post-mux IQ stream, rate-matched)
4. Link against the firmware HAL (`hal.c`, `dma_rx.c`, `dma_tx.c`, `convert.c`)
5. Use `fpga/tcl/styx_procs.tcl` for build automation (OOC synthesis, incremental builds)

The `hil_ctrl` module reserves registers 0x14–0x38 for downstream
extension — your project can add custom control/status registers in that
address range without modifying styx RTL.

## CI

GitHub Actions runs on push/PR to `main`:

1. **RTL lint** — Verilator strict warnings on all modules
2. **RTL sim** — full cocotb test suite (63 tests across 8 modules)
3. **Firmware tests** — host-native build + ctest

## Further Reading

- [registers.md](registers.md) — complete register map with bit-field descriptions
- `fpga/rtl/*.v` — Verilog sources with inline architecture commentary
- `fpga/test/test_*.py` — cocotb testbenches (readable as spec-by-example)
- `firmware/tools/platform/README.md` — on-target test utility guide
- `config.mk.example` — all build options documented inline

## License

MIT.  `fpga/project/system_top.v` and
`fpga/project/constraints/system_constr.xdc` retain their original
ADI-BSD / GPLv2 dual license.
