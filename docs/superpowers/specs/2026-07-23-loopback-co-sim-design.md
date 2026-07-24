# IQ DMA Loopback Co-Simulation

**Date:** 2026-07-23
**Status:** design-approved

## Purpose

Add a cocotb co-simulation that instantiates `iq_dma_tx` and `iq_dma_rx` on a shared AXI3 mock DDR, wiring TX DAC output to RX ADC input. This models the physical loopback path and catches inter-module interaction bugs that per-module tests miss — specifically the hardware regression where streaming TX changes cause RX DMA stalls and FCS degradation.

## Architecture

```
tb_iq_dma_loopback.v
  ├─ iq_dma_tx  (TX DMA, AXI3 read master)
  ├─ iq_dma_rx  (RX DMA, AXI3 write master)
  │
  ├─ re_out[15:4] ────► re_in[11:0]   (TX→RX loopback)
  ├─ im_out[15:4] ────► im_in[11:0]
  ├─ valid_out ───────► valid_in
  │
  ├─ m_axi_tx_ar*/r* ──┐
  ├─ m_axi_rx_aw*/w*/b* ┤──► Axi3SharedSlave
  │                     │    (single dict, max_outstanding=N)
  └─ s_axi_* (both) ────┘    (TX reads, RX writes)
```

### Clock strategy

Parameter `TX_CLK_SAME_AS_RX` (default 1):
- When 1: both modules clocked by `clk`, TX output feeds RX input directly
- When 0: TX gets `tx_clk`, leaves an async FIFO insertion point for future CDC testing

### Axi3SharedSlave

Combined AXI3 read+write mock with:
- Shared byte-addressed dict memory — TX reads and RX writes to same space
- `max_outstanding` parameter (default 2) — limits total in-flight transactions modeling PS7 HP port credit limits
- Round-robin arbitration between concurrent read and write requests
- Pre-load API: `write_iq_samples()` for TX test data
- Verification API: `read_iq_samples()` for RX output verification

## Tests

| # | Test | What it verifies |
|---|------|-----------------|
| 1 | `test_loopback_basic` | TX 64-sample one-shot, RX captures all correctly |
| 2 | `test_loopback_sustained` | TX cyclic 512+ samples, RX continuous capture |
| 3 | `test_loopback_dma_stall` | TX one-shot while RX running; RX `sample_count` never freezes |
| 4 | `test_loopback_contention` | max_outstanding=2, back-to-back TX triggers; RX overflow=0 |
| 5 | `test_loopback_restart` | Stop TX mid-cyclic, restart; RX continues without gaps |
| 6 | `test_loopback_ddr_integrity` | Readback DDR after loopback; bit-perfect roundtrip |

## Files

| File | Action |
|------|--------|
| `fpga/test/tb_iq_dma_loopback.v` | New — wrapper |
| `fpga/test/test_iq_dma_loopback.py` | New — cocotb tests |
| `fpga/test/axi3_mocks.py` | Modify — add `Axi3SharedSlave` |
| `fpga/test/Makefile` | Modify — add target to `all_tests` |

## Dependencies

- Existing `fpga/rtl/iq_dma_tx.v`, `fpga/rtl/iq_dma_rx.v`
- Existing `AxiLiteMaster` from `fpga/test/axi_lite_driver.py`
- Existing pack/unpack helpers from `axi3_mocks.py`
- Verilator (already in CI)
- cocotb (already in CI)
