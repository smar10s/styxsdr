# StyxSDR

**Pluto's smallest moon** — a minimal FPGA bitstream for the
[PlutoSDR](https://www.analog.com/en/resources/evaluation-hardware-and-software/evaluation-boards-kits/adalm-pluto.html)
(Xilinx Zynq 7010). It replaces the stock ADI IIO/DMA framework with
custom streaming DMA engines: continuous IQ capture, deterministic
playback, HIL injection, and a debug probe. ~4.4k LUTs and 2 DSPs leave
most of the Z-7010 for downstream logic. Projects source
`fpga/project/system_bd.tcl`, connect to the post-mux IQ stream, and link
the firmware HAL.

## Features

- **128 MB RX ring** — 33,554,432 samples. Depth follows the configured
  rate: 1.68 s at 20 MSPS, 1.09 s at 30.72 MSPS.
- **32 MB TX/HIL buffer** — 8,388,608 samples. One-shot, cyclic, or
  continuous ARM-fed streaming.
- **Two-phase TX** — load the waveform slowly, then trigger with a single
  deterministic register write.
- **HIL injection** — mux live ADC against DDR playback, paced by the ADC
  valid strobe. No rate register.
- **Debug snap** — 1024-deep BRAM capture with configurable pre/post
  trigger split.
- **Build fingerprint** — content-addressed SHA-256 of RTL, project
  sources, and the project ID, readable at runtime.
- **Rate-agnostic plumbing** — the ADC CDC and HIL pacing assume no
  particular sample rate. The AD9361 maximum is 61.44 MSPS.

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

Requires Vivado 2025.2, Docker, and — for simulation — Python 3.11+ with
`cocotb` and Verilator.

### Flashing

Pre-built `pluto.frm` images are on
[GitHub Releases](https://github.com/smar10s/styxsdr/releases).

Primary path: copy `pluto.frm` to the Pluto's USB mass-storage drive and
eject. The LED blinks while flashing; wait for the board to reappear.

SSH path, with checksum verification:

```bash
PLUTO_PASS=analog bin/flash.sh ~/Downloads/pluto.frm
```

**Rev.B Pluto users** — the AD9361 defaults to AD9363A tuning range on
Rev.B boards. After the first flash, SSH in and run
`bin/fix_ad9361_guardrail.sh` to unlock the full AD9361 range.

### Validation

```bash
PLUTO_PASS=analog bin/validate.sh
```

This checks AD9361 initialization and verifies the build fingerprint.

## Fabric Modules

| Module | Purpose |
|---|---|
| **iq_dma_rx** | ADC-to-DDR via AXI3 HP0. 128 MB circular buffer, double-buffered BRAM fill. |
| **iq_dma_tx** | DDR-to-DAC via AXI3 HP2. 16-segment fill pipeline, zero-bubble drain, stream mode with WR_PTR CDC. |
| **hil_ctrl** | Test mux and DDR playback. Output is paced by the post-CDC ADC valid strobe. |
| **snap_axi** | Debug capture probe. 1024-deep BRAM, pre/post trigger split, AXI4-Lite read-back. |
| **adc_sync** | Ratio-safe async FIFO from AD9361 `l_clk` to `sys_cpu_clk`. |
| **axi_build_id** | Read-only 32-bit fingerprint register. |

RX data path: ADC → `adc_sync` → `iq_dma_rx` (live capture) and
`adc_sync` → `hil_ctrl` mux → downstream and snap. `iq_dma_rx` taps
before the mux, so DDR captures the live ADC only. TX data path:
DDR → `iq_dma_tx` → DAC.

## Memory Map and Sample Packing

| Region | Size | Contents |
|---|---|---|
| 0x10000000 | 128 MB | RX ring buffer (33,554,432 samples) |
| 0x18000000 | 32 MB | TX / HIL playback / stream (8,388,608 samples) |

160 MB reserved from 0x10000000 by the device tree overlay
(`styx-pluto.dtso`). Stock ADI DMA engines are disabled.

**IQ packing**: each 32-bit DDR word is `{8'b0, imag[11:0], real[11:0]}`.
AXI bursts pack two samples per 64-bit beat (`{word_odd, word_even}`).
Burst length is 16 beats = 128 bytes = 32 IQ samples.

HIL playback shares the 32 MB TX region, so the same depth limit applies:
0.42 s at 20 MSPS, 0.27 s at 30.72 MSPS.

## Sample Rate and HIL Pacing

The AD9361 rate is set at runtime through IIO; no bitstream rebuild is
needed. The fabric assumes no particular rate:

- `adc_sync` is an async FIFO that is safe across the AD9361 range.
- `hil_ctrl` advances playback on the same post-CDC `adc_valid` strobe
  the live data path uses.

HIL therefore matches the configured rate exactly, including runtime rate
changes. There is no HIL rate register to keep in sync with the AD9361.

Consequences worth knowing:

- HIL requires the ADC to be streaming. If it is not, playback does not
  advance.
- HIL replaces the ADC at the mux; it is never summed with live samples.
- `iq_dma_rx` taps the ADC before the mux, so DDR never contains HIL data.
  HIL is visible downstream and at the snap probe.
- Downstream logic must derive timing from the sample strobe, not from the
  100 MHz fabric clock.

## Firmware API

The C firmware runs on the Zynq PS (ARM Cortex-A9) under Linux. It
provides register access via `/dev/mem` mmap and DMA convenience
functions.

| Header | Purpose |
|---|---|
| `hal.h` | Register map defines, IQ pack/unpack macros, AD9361 sysfs wrappers |
| `dma_rx.h` | RX ring buffer: start, stop, read write-pointer, blocking capture |
| `dma_tx.h` | TX waveform: one-shot, two-phase load/trigger, cyclic, continuous streaming, stop |
| `convert.h` | DDR ↔ float conversion (auto-scale TX, sign-extended RX) |

### One-Shot TX/RX Loopback

Separate the slow work (DDR writes, conversion) from the fast trigger
(single register store):

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

### Continuous Streaming TX

For waveforms that exceed the 32 MB buffer, feed the fabric in small
chunks:

```c
#include "hal.h"
#include "dma_tx.h"

float real[DMA_TX_STREAM_CHUNK];
float imag[DMA_TX_STREAM_CHUNK];
generate_tone(real, imag, DMA_TX_STREAM_CHUNK);

dma_tx_stream_start();

uint64_t t0 = time_ms();
while (time_ms() - t0 < 8000) {
    // wait for headroom before feeding the next chunk
    while (dma_tx_stream_available() < DMA_TX_STREAM_CHUNK)
        usleep(200);

    dma_tx_stream_feed(real, imag, DMA_TX_STREAM_CHUNK);
}
dma_tx_stream_stop();
```

The fill FSM wraps at the 32 MB boundary automatically.
`dma_tx_stream_feed()` issues a memory barrier before updating the WR_PTR
register so the FPGA sees all committed DDR stores.

### Blocking RX Capture

```c
float real[4096], imag[4096];
int n = dma_rx_capture(4096, real, imag, 50);  // 50 ms timeout
if (n > 0) {
    // process fresh IQ data...
}
```

`dma_rx_capture()` polls the write pointer at 50 µs intervals and handles
ring-buffer wrap transparently.

Register-level control is available through `hal.h`; see
[registers.md](registers.md).

### HIL Test Injection

Golden waveform injection without RF. `hil_ctrl` switches the IQ mux
from the live ADC to DDR playback, paced by the ADC sample strobe:

```bash
hil_inject -v -w    # load hil_golden.json, verify watermark, capture snap
```

`firmware/vectors/hil_golden.json` is a 1024-sample vector: a 3-sample
ASCII watermark, then a 0.5–9.5 MHz chirp.

## Testing and CI

```bash
make sim                # full cocotb RTL suite (Verilator)
make test               # host-native firmware unit tests
make waves TARGET=test_iq_dma_rx   # VCD for one module
```

GitHub Actions runs on push and pull request: Verilator lint, the full
cocotb suite, and host-native firmware tests.

## Downstream Integration

1. Add styx as a git submodule.
2. Source `fpga/project/system_bd.tcl` in your block design.
3. Connect to `hil_ctrl_0/iq_*` — the post-mux stream, paced by the ADC
   valid strobe.
4. Link against the firmware HAL (`hal.c`, `dma_rx.c`, `dma_tx.c`,
   `convert.c`).
5. Use `fpga/tcl/styx_procs.tcl` for build automation (OOC synthesis,
   incremental builds).

`iq_dma_rx` captures the ADC before the mux. If your project needs HIL
data in DDR, reconnect the capture tap; that is a project-level decision.

`hil_ctrl` reserves registers 0x14–0x38 for downstream control and status,
so a project can extend it without modifying styx RTL.

## Resources

Post-route, Vivado 2025.2, Zynq 7010:

| Resource | Used | Available | % |
|---|---|---|---|
| Slice LUTs | 4,427 | 17,600 | 25.2% |
| Slice Registers | 6,385 | 35,200 | 18.1% |
| Block RAM Tiles | 3 | 60 | 5.0% |
| DSP Blocks | 2 | 80 | 2.5% |

Leaves 75% of LUTs and 95% of BRAM free for downstream logic.

## Project Layout

```
styx/
├── Makefile                Top-level orchestration (make help)
├── config.mk.example      Build config template
├── registers.md           Register map
├── fpga/                   RTL, block design, build TCL, cocotb tests
│   └── extern/adi-hdl/     ADI HDL IP library (submodule)
├── firmware/               HAL, DMA drivers, tools, HIL vectors
├── packaging/              pluto.frm build, device tree overlay
├── bin/                    Operator scripts (flash, validate)
├── extern/                 plutosdr-fw, lib80211 (submodules)
└── build/                  Output artifacts (gitignored)
```

## Further Reading

- [registers.md](registers.md) — complete register map with bit fields
- `fpga/rtl/*.v` — Verilog sources with architecture commentary
- `fpga/test/test_*.py` — cocotb testbenches, readable as spec-by-example
- `firmware/tools/platform/README.md` — on-target test utility guide
- `config.mk.example` — all build options documented inline

## License

MIT. `fpga/project/system_top.v` and
`fpga/project/constraints/system_constr.xdc` retain their original
ADI-BSD / GPLv2 dual license.
