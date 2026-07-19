# Platform Tools

Validate the PlutoSDR platform: AD9361 configuration, DMA engines, DDR ring
buffer, and analog RF path. Use **lib80211 on ARM** for OFDM decode.

## Tools

| Tool | Purpose |
|------|---------|
| `pluto_loopback` | Cable loopback: TX → DAC → cable → ADC → DDR → ARM lib80211 decode |
| `pluto_sigladder` | Graduated signal complexity: tone → chirp → preamble → full frame |
| `pluto_burst_loopback` | Multi-frame burst: same path as loopback, multiple frames per TX |
| `pluto_tx_beacon` | Cyclic beacon TX (stimulus generation, no RX) |
| `pluto_dma_test` | DMA register-level start/stop/restart validation |

## Parent tools (firmware/tools/)

| Tool | Purpose |
|------|---------|
| `hil_inject` | HIL mux + snap probe test (no RF, no lib80211) |
| `adc_capture` | Capture raw ADC IQ during cable loopback TX |

## What they prove

- `hil_inject`: The FPGA fabric plumbing works (mux switches, snap captures)
- `pluto_sigladder` L1-L6: The full analog + digital path delivers OFDM without corruption
- Together: this RTL handles OFDM frames (decoded on ARM)
