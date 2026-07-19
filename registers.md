# Register Map

## Top-Level Memory Map

| Address    | Module       | Purpose                  |
|------------|--------------|--------------------------|
| 0x43C00000 | axi_build_id | Build fingerprint (RO)   |
| 0x79020000 | axi_ad9361   | RF PHY control           |
| 0x7C430000 | axi_spi      | External SPI             |
| 0x7C4B0000 | iq_dma_rx    | RX DMA control           |
| 0x7C4D0000 | iq_dma_tx    | TX DMA control           |
| 0x7C4E0000 | snap_axi     | Debug snap probe         |
| 0x7C500000 | hil_ctrl     | HIL test controller      |

## DDR Layout

| Region      | Size   | Purpose                 |
|-------------|--------|-------------------------|
| 0x10000000  | 128 MB | RX ring buffer          |
| 0x18000000  | 4 MB   | TX / HIL playback       |

144 MB total reserved by device tree overlay at 0x10000000.

### IQ Sample Packing

Each 32-bit DDR word: `{8'b0, imag[11:0], real[11:0]}`.  
Each 64-bit AXI beat: `{word_odd, word_even}` (even-index sample at lower address).  
AXI3 burst length: 16 beats = 128 bytes = 32 IQ samples per burst.

## iq_dma_rx (0x7C4B0000)

Continuous ADC-to-DDR writer. 128 MB circular buffer, double-buffered BRAM
accumulator (32x64) → AXI3 HP0 burst writes.

| Offset | Name               | Access | Description                                          |
|--------|--------------------|--------|------------------------------------------------------|
| 0x00   | CONTROL            | R/W    | `[0]` enable (1 = streaming, 0 = idle)               |
| 0x04   | STATUS             | RO     | `[0]` active, `[31:16]` overflow_count              |
| 0x08   | DDR_BASE           | R/W    | Physical DDR base address (typically 0x10000000)      |
| 0x0C   | WR_PTR             | RO     | Current write pointer — sample index, 25-bit          |
| 0x10   | SAMPLE_COUNT       | RO     | Total samples written, 32-bit wrapping                |
| 0x14   | WRAP_COUNT         | RO     | Ring buffer wrap counter, 16-bit                      |

Enable initiates streaming; disable stops after the current burst
completes. `WR_PTR` is live — each read returns the instant value.
Overflow occurs if the AXI write channel stalls beyond one buffer
half-fill (nominally impossible at the 5:1 clock ratio but counted
for diagnostics).

## iq_dma_tx (0x7C4D0000)

DDR-to-DAC reader. 16-segment BRAM (256x64) with 15-segment fill-ahead
margin. Speculative wrap AR hides DDR row-activate latency at cyclic
boundaries. AXI3 HP2 read master, l_clk domain.

| Offset | Name     | Access | Description                                              |
|--------|----------|--------|----------------------------------------------------------|
| 0x00   | CONTROL  | R/W    | `[0]` enable, `[1]` trigger (write-1-to-set), `[2]` cyclic |
| 0x04   | STATUS   | RO     | `[0]` active, `[1]` tx_done                               |
| 0x08   | DDR_BASE | R/W    | Physical DDR base of TX waveform (typically 0x18000000)   |
| 0x0C   | TX_COUNT | R/W    | Total IQ samples to transmit                              |
| 0x10   | TX_PTR   | RO     | Current read pointer — sample index                       |

**One-shot mode** (`cyclic=0`): set DDR_BASE and TX_COUNT, write
`CONTROL` with enable=1 and trigger=1.  `tx_done` asserts when
`TX_PTR` reaches `TX_COUNT`.  Re-trigger by writing CONTROL again.

**Cyclic mode** (`cyclic=1`): loops playback continuously after
trigger.  `tx_done` never asserts.  To stop, write CONTROL with
enable=0 and trigger=1 — the FSM drains the current iteration then
halts.

**Two-phase TX**: write DDR_BASE + TX_COUNT with enable=0, then fire
a second write with enable=1 + trigger=1.  The trigger register write
returns in microseconds, enabling precise timing relative to
`iq_dma_rx` write pointer capture.

## snap_axi (0x7C4E0000)

AXI4-Lite wrapper around `debug_snap` — a parametric BRAM capture
buffer (depth 1024, post-trigger depth 512, 32-bit data width).

| Offset | Name        | Access | Description                                           |
|--------|-------------|--------|-------------------------------------------------------|
| 0x00   | CONTROL     | R/W    | `[0]` arm (write 1 rearms), `[1]` sw_trigger (W1S, self-clr), `[2]` circular_en |
| 0x04   | STATUS      | RO     | `[0]` captured, `[1]` armed, `[25:16]` trig_pos       |
| 0x08   | TRIG_CYCLE  | RO     | `[31:0]` cycle counter value at trigger               |
| 0x0C   | RD_ADDR     | R/W    | `[9:0]` read address                                  |
| 0x10   | RD_DATA     | RO     | `[31:0]` data at RD_ADDR (2-cycle BRAM latency hidden by AXI) |

**Arm + trigger**: write CONTROL arm=1, then either issue a software
trigger (write sw_trigger=1) or wait for an external trigger to fire.
Buffer freezes 512 samples after trigger.  Read back by writing
RD_ADDR and reading RD_DATA.

## hil_ctrl (0x7C500000)

Hardware-in-the-Loop test controller. IQ mux switches between live ADC
data and DDR playback. DDR read FSM uses AXI3 HP1 with 256x32 BRAM
FIFO.

| Offset | Name       | Access | Description                                    |
|--------|------------|--------|------------------------------------------------|
| 0x00   | CONTROL    | R/W    | `[0]` test_mode, `[1]` trigger (W1S)            |
| 0x04   | STATUS     | RO     | `[0]` playback_active, `[1]` playback_done       |
| 0x08   | DDR_BASE   | R/W    | Physical DDR base of test waveform               |
| 0x0C   | PLAY_COUNT | R/W    | Number of IQ samples to play back               |
| 0x10   | PLAY_PTR   | RO     | Current playback position (sample index)         |
| 0x3C   | ADC_CNT    | RO     | `[31:0]` adc_valid pulse counter (live mode only) |

**0x14–0x38**: reserved for downstream extension registers.

**Test mode**: set test_mode=1, write DDR_BASE and PLAY_COUNT, then
trigger.  The mux switches from live ADC to DDR playback for the
specified duration, then de-asserts test_mode and switches back.
Playback is rate-matched to live ADC (1 sample per 5 clocks at 100
MHz).

## axi_build_id (0x43C00000)

Single read-only 32-bit register.

| Offset | Name     | Access | Description                                      |
|--------|----------|--------|--------------------------------------------------|
| 0x00   | BUILD_ID | RO     | Content-addressed SHA-256 fingerprint (RTL + TCL sources) |

Read with `devmem 0x43C00000`. Writes are accepted on the AXI bus but
silently ignored.
