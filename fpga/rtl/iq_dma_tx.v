// SPDX-License-Identifier: MIT

`timescale 1ns/1ps
// fpga/rtl/iq_dma_tx.v
//
// TX DMA reader: reads pre-built IQ from DDR and outputs samples
// directly to the axi_ad9361 DAC in the l_clk domain.
//
// 16-segment BRAM (256 × 64-bit, sixteen segments of 16) to provide
// 15-segment fill-ahead margin that eliminates drain stalls under
// extreme DDR latency jitter.  A fill FSM reads DDR bursts into one
// segment while a drain FSM outputs from another at exactly 1 sample
// per l_clk.  Additionally, the fill FSM issues a SPECULATIVE WRAP AR
// during the last burst's data reception: it asserts a second AXI3 read
// address for the first burst of the next iteration while still receiving
// data for the current burst.  The DDR controller begins row-activate
// in parallel, hiding most of the penalty at cyclic iteration boundaries.
//
// Dual clock design:
//   clk        = l_clk (~20 MHz) — fill FSM, drain FSM, AXI3 DDR
//                read master, DAC output.  PS7 HP port handles DDR
//                clock crossing internally (ACLK = l_clk).
//   s_axi_aclk = sys_cpu_clk (100 MHz) — AXI4-Lite slave only.
//
// The drain FSM is the critical path: it MUST produce 1 new sample
// every l_clk cycle with no bubbles.  The DAC samples dac_data_i0/q0
// every l_clk cycle unconditionally.  A single stale cycle corrupts
// the waveform.
//
// Zero-bubble design: D_EVEN pre-fetches the BRAM address for
// beat 0 of the next segment when it detects the following D_ODD
// will trigger a segment-swap.  D_ODD then transitions directly to
// D_EVEN (skipping D_PRE).  D_WAIT speculatively pre-fetches
// so underrun recovery is also bubble-free.  D_PRE is only used
// at startup (D_IDLE → D_PRE → D_EVEN) where no valid output
// is active, so no DAC bubble occurs.
//
// IMPORTANT: The drain FSM outputs unconditionally every l_clk
// cycle, WITHOUT gating on dac_valid.  In 1R1T mode dac_valid
// fires every cycle, so gating should be a no-op — but on
// hardware, occasional dac_valid glitches cause the output to
// stall for 1 cycle, stretching the waveform by 1 sample.
// Over a full OFDM frame (>1000 samples), these accumulated
// insertions destroy decode fidelity.  Validated: removing
// dac_valid gating restored 8/8 rates (Apr 24).
//
// Output is 16-bit left-justified (12-bit sample << 4) to match
// axi_ad9361 dac_data_i0/q0 format.
//
// DDR packing (same as iq_dma_rx):
//   Each 32-bit word: {8'b0, im[11:0], re[11:0]}
//   Each 64-bit beat: {word_odd, word_even} (even at lower address)
//
// Stopping a cyclic TX:
//   1. Write CONTROL with enable=0 (clears reg_enable_axi)
//   2. Write CONTROL with trigger=1 (toggles trigger_toggle_axi)
//      — this latches lcl_enable=0 into the l_clk domain
//   3. Fill FSM stops at next F_SWAP or F_WAIT boundary
//   4. Drain FSM finishes its current iteration, sets tx_done=1
//   5. Poll STATUS until tx_done=1 (or active=0)
//   Both writes can be combined: write enable=0|trigger=1 in one
//   CONTROL write (0x02).  Transition is graceful — the current
//   waveform iteration completes, no mid-sample abort.
//
// CDC false_path note:
//   tx_ptr (clk/l_clk) is read directly from s_axi_aclk with no
//   synchronizer.  This is intentional — it is display-only (ARM
//   progress bar).  Synthesis should treat tx_ptr → s_axi_rdata as
//   a false path:
//     set_false_path -from [get_cells {tx_ptr_reg[*]}] \
//                    -to   [get_cells {s_axi_rdata_reg[*]}]
//   Torn reads are acceptable for this register.
//
// ARM control via AXI4-Lite (s_axi_aclk domain):
//   0x00  CONTROL   R/W  [0]=enable, [1]=trigger (W1S), [2]=cyclic, [3]=stream
//   0x04  STATUS    R    [0]=active, [1]=tx_done
//   0x08  DDR_BASE  R/W  Physical DDR base of TX buffer
//   0x0C  TX_COUNT  R/W  Total samples in buffer (one-shot/cyclic); buffer size (stream)
//   0x10  TX_PTR    R    Current read pointer (sample index, unsynchronized)
//   0x14  WR_PTR    R/W  ARM write cursor (stream mode: fill FSM reads up to here)
//   0x18  RD_PTR    R    FPGA read position (stream mode: ARM may overwrite past here)

module iq_dma_tx #(
    parameter BURST_LEN = 16,               // AXI3 max burst (16 beats of 64 bits)
    parameter BUF_SIZE_SAMPLES = 32'd8388608 // 32 MB / 4 bytes = 2^23
) (
    // l_clk domain — fill FSM + drain FSM + AXI3 master + DAC output
    (* X_INTERFACE_INFO = "xilinx.com:signal:clock:1.0 clk CLK" *)
    (* X_INTERFACE_PARAMETER = "ASSOCIATED_BUSIF m_axi, ASSOCIATED_RESET rst" *)
    input  wire        clk,

    (* X_INTERFACE_INFO = "xilinx.com:signal:reset:1.0 rst RST" *)
    (* X_INTERFACE_PARAMETER = "POLARITY ACTIVE_HIGH" *)
    input  wire        rst,

    // DAC output (16-bit, left-justified, directly to axi_ad9361)
    output reg  [15:0] re_out,
    output reg  [15:0] im_out,
    output reg         valid_out,

    // DAC handshake from axi_ad9361 (active every l_clk in 1R1T)
    input  wire        dac_valid,

    // --- AXI3 read-only master (from PS7 HP, 64-bit, l_clk domain) ---
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 m_axi ARADDR" *)
    (* X_INTERFACE_PARAMETER = "PROTOCOL AXI3, ADDR_WIDTH 32, DATA_WIDTH 64, HAS_BURST 1, HAS_LOCK 0, HAS_PROT 0, HAS_CACHE 0, HAS_QOS 0, HAS_REGION 0, HAS_WSTRB 0, MAX_BURST_LENGTH 16, NUM_WRITE_OUTSTANDING 0, NUM_READ_OUTSTANDING 2, SUPPORTS_NARROW_BURST 0, READ_WRITE_MODE READ_ONLY" *)
    output reg  [31:0] m_axi_araddr,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 m_axi ARLEN" *)
    output wire [3:0]  m_axi_arlen,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 m_axi ARSIZE" *)
    output wire [2:0]  m_axi_arsize,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 m_axi ARBURST" *)
    output wire [1:0]  m_axi_arburst,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 m_axi ARVALID" *)
    output reg         m_axi_arvalid,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 m_axi ARREADY" *)
    input  wire        m_axi_arready,

    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 m_axi RDATA" *)
    input  wire [63:0] m_axi_rdata,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 m_axi RRESP" *)
    input  wire [1:0]  m_axi_rresp,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 m_axi RLAST" *)
    input  wire        m_axi_rlast,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 m_axi RVALID" *)
    input  wire        m_axi_rvalid,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 m_axi RREADY" *)
    output reg         m_axi_rready,

    // --- AXI4-Lite slave (s_axi_aclk domain — sys_cpu_clk) ---
    (* X_INTERFACE_INFO = "xilinx.com:signal:clock:1.0 s_axi_aclk CLK" *)
    (* X_INTERFACE_PARAMETER = "ASSOCIATED_BUSIF s_axi, ASSOCIATED_RESET s_axi_aresetn, FREQ_HZ 100000000" *)
    input  wire        s_axi_aclk,

    (* X_INTERFACE_INFO = "xilinx.com:signal:reset:1.0 s_axi_aresetn RST" *)
    (* X_INTERFACE_PARAMETER = "POLARITY ACTIVE_LOW" *)
    input  wire        s_axi_aresetn,

    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 s_axi AWADDR" *)
    (* X_INTERFACE_PARAMETER = "PROTOCOL AXI4LITE, FREQ_HZ 100000000, ADDR_WIDTH 32, DATA_WIDTH 32" *)
    input  wire [31:0] s_axi_awaddr,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 s_axi AWVALID" *)
    input  wire        s_axi_awvalid,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 s_axi AWREADY" *)
    output reg         s_axi_awready,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 s_axi WDATA" *)
    input  wire [31:0] s_axi_wdata,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 s_axi WSTRB" *)
    input  wire [3:0]  s_axi_wstrb,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 s_axi WVALID" *)
    input  wire        s_axi_wvalid,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 s_axi WREADY" *)
    output reg         s_axi_wready,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 s_axi BRESP" *)
    output reg  [1:0]  s_axi_bresp,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 s_axi BVALID" *)
    output reg         s_axi_bvalid,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 s_axi BREADY" *)
    input  wire        s_axi_bready,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 s_axi ARADDR" *)
    input  wire [31:0] s_axi_araddr,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 s_axi ARVALID" *)
    input  wire        s_axi_arvalid,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 s_axi ARREADY" *)
    output reg         s_axi_arready,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 s_axi RDATA" *)
    output reg  [31:0] s_axi_rdata,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 s_axi RRESP" *)
    output reg  [1:0]  s_axi_rresp,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 s_axi RVALID" *)
    output reg         s_axi_rvalid,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 s_axi RREADY" *)
    input  wire        s_axi_rready
);

    // =========================================================
    // AXI3 read constants
    // =========================================================
    assign m_axi_arlen   = BURST_LEN - 1;
    assign m_axi_arsize  = 3'b011;       // 8 bytes
    assign m_axi_arburst = 2'b01;        // INCR

    localparam SAMPLES_PER_BURST = BURST_LEN * 2;  // 32

    // =========================================================
    // Register map
    // =========================================================
    localparam REG_CONTROL  = 32'h00;
    localparam REG_STATUS   = 32'h04;
    localparam REG_DDR_BASE = 32'h08;
    localparam REG_TX_COUNT = 32'h0C;
    localparam REG_TX_PTR   = 32'h10;
    localparam REG_WR_PTR   = 32'h14;
    localparam REG_RD_PTR   = 32'h18;

    // =========================================================
    // Control registers (s_axi_aclk domain — written by AXI-Lite)
    // =========================================================
    reg         reg_enable_axi;
    reg         reg_cyclic_axi;
    reg         reg_stream_axi;
    reg [31:0]  reg_ddr_base_axi;
    reg [31:0]  reg_tx_count_axi;
    reg [31:0]  reg_wr_ptr_axi;
    reg         trigger_toggle_axi;

    // =========================================================
    // CDC: s_axi_aclk -> clk (l_clk)
    // =========================================================
    reg         trig_sync1, trig_sync2, trig_sync3;
    wire        trigger_edge = trig_sync2 ^ trig_sync3;

    reg         lcl_enable;
    reg         lcl_cyclic;
    reg         lcl_stream;
    reg [31:0]  lcl_ddr_base;
    reg [31:0]  lcl_tx_count;
    reg [31:0]  lcl_wr_ptr;

    // WR_PTR 2-stage CDC: s_axi_aclk -> l_clk
    (* ASYNC_REG = "TRUE" *) reg [31:0] wr_ptr_sync1, wr_ptr_sync2;

    always @(posedge clk) begin
        if (rst) begin
            trig_sync1   <= 0;
            trig_sync2   <= 0;
            trig_sync3   <= 0;
            lcl_enable   <= 0;
            lcl_cyclic   <= 0;
            lcl_stream   <= 0;
            lcl_ddr_base <= 0;
            lcl_tx_count <= 0;
            lcl_wr_ptr   <= 0;
            wr_ptr_sync1 <= 0;
            wr_ptr_sync2 <= 0;
        end else begin
            trig_sync1 <= trigger_toggle_axi;
            trig_sync2 <= trig_sync1;
            trig_sync3 <= trig_sync2;

            if (trigger_edge) begin
                lcl_enable   <= reg_enable_axi;
                lcl_cyclic   <= reg_cyclic_axi;
                lcl_stream   <= reg_stream_axi;
                lcl_ddr_base <= reg_ddr_base_axi;
                lcl_tx_count <= reg_tx_count_axi;
                // Note: diagnostic counters are reset by trigger_edge
                // in their respective FSM always blocks (fill/drain)
            end

            // WR_PTR: continuously synchronized (not latched on trigger).
            // Updated by ARM in chunks during stream mode.  A stale value
            // is conservative (fill FSM stalls, never over-reads).
            wr_ptr_sync1 <= reg_wr_ptr_axi;
            wr_ptr_sync2 <= wr_ptr_sync1;
            lcl_wr_ptr   <= wr_ptr_sync2;
        end
    end

    // CDC: clk -> s_axi_aclk (status readback)
    // tx_active: driven by fill FSM only
    // tx_done: driven by drain FSM only
    reg         tx_active;
    reg         tx_done;
    reg         tx_active_sync1, tx_active_sync2;
    reg         tx_done_sync1,   tx_done_sync2;

    // RD_PTR Gray-code CDC: l_clk -> s_axi_aclk
    (* ASYNC_REG = "TRUE" *) reg [31:0] rd_ptr_gray_sync1, rd_ptr_gray_sync2;
    reg [31:0] rd_ptr_readable;

    // Gray-to-binary combinatorial decode
    function [31:0] gray_to_bin;
        input [31:0] g;
        integer j;
        begin
            gray_to_bin[31] = g[31];
            for (j = 30; j >= 0; j = j - 1)
                gray_to_bin[j] = gray_to_bin[j+1] ^ g[j];
        end
    endfunction

    always @(posedge s_axi_aclk) begin
        if (!s_axi_aresetn) begin
            tx_active_sync1 <= 0;
            tx_active_sync2 <= 0;
            tx_done_sync1   <= 0;
            tx_done_sync2   <= 0;
            rd_ptr_gray_sync1 <= 0;
            rd_ptr_gray_sync2 <= 0;
            rd_ptr_readable <= 0;
        end else begin
            tx_active_sync1 <= tx_active;
            tx_active_sync2 <= tx_active_sync1;
            tx_done_sync1   <= tx_done;
            tx_done_sync2   <= tx_done_sync1;

            rd_ptr_gray_sync1 <= rd_ptr_gray_reg;
            rd_ptr_gray_sync2 <= rd_ptr_gray_sync1;
            rd_ptr_readable   <= gray_to_bin(rd_ptr_gray_sync2);
        end
    end

    // =========================================================
    // 16-segment BRAM — 256 × 64-bit entries (16 segments of 16)
    //
    // fill_seg and drain_seg cycle 0→1→...→15→0...
    // The fill FSM can be up to 15 segments ahead of the drain FSM.
    // =========================================================
    localparam NUM_SEGS = 16;

    (* ram_style = "block" *)
    reg [63:0] burst_mem [0:NUM_SEGS*BURST_LEN-1];

    // Port A: synchronous write (fill side)
    reg        mem_wr_en;
    reg [7:0]  mem_wr_addr;
    reg [63:0] mem_wr_data;

    always @(posedge clk) begin
        if (mem_wr_en)
            burst_mem[mem_wr_addr] <= mem_wr_data;
    end

    // Port B: synchronous read (drain side, 1-cycle latency)
    reg [7:0]  mem_rd_addr;
    reg [63:0] mem_rd_data;

    always @(posedge clk) begin
        mem_rd_data <= burst_mem[mem_rd_addr];
    end

    // =========================================================
    // Per-segment status flags
    //
    // seg_ready[i]: segment i is full and ready to drain
    // Set by fill FSM when burst read completes into segment i.
    // Cleared by drain FSM when it finishes draining segment i.
    // =========================================================
    reg [NUM_SEGS-1:0]  seg_ready;

    // =========================================================
    // Fill FSM — reads DDR bursts into BRAM segments
    //
    // IDLE → (trigger) → LATCH → RD_ADDR → RD_DATA → ...
    // After each burst, checks if next segment is available and
    // continues.  If next segment is still being drained, waits.
    // =========================================================
    localparam F_IDLE     = 3'd0;
    localparam F_LATCH    = 3'd1;
    localparam F_RD_ADDR  = 3'd2;
    localparam F_RD_DATA  = 3'd3;
    localparam F_SWAP     = 3'd4;   // set seg_ready, advance to next segment
    localparam F_WAIT     = 3'd5;   // wait for next segment to be free
    localparam F_DONE     = 3'd6;
    localparam F_DRAIN_WRAP = 3'd7; // drain outstanding speculative wrap read on stop

    reg [2:0]  f_state;
    reg [3:0]  fill_seg;            // which segment we're filling (0-15)
    reg [3:0]  fill_beat;           // 0..15 during burst read
    reg [31:0] fill_ptr;            // DDR sample index (advances by 32 per burst)
    reg [31:0] fill_remaining;      // samples left to fill

    // Speculative wrap AR tracking (for zero-bubble cyclic iteration boundary)
    reg        wrap_ar_issued;      // AR for wrap burst has been asserted
    reg        wrap_ar_accepted;    // AR for wrap burst was accepted (arready handshake)

    // =========================================================
    // Drain FSM — outputs samples from BRAM to DAC
    //
    // Zero-bubble pipeline: EVEN and ODD alternate, producing
    // exactly 1 sample per l_clk cycle with no gaps — even
    // across segment-swap boundaries.
    //
    // Schedule for beat N within a segment:
    //   D_EVEN: output even sample from mem_rd_data
    //           latch full word in out_word
    //           if last beat: pre-fetch next segment's beat 0 addr
    //   D_ODD:  output odd sample from out_word
    //           if segment-swap: set beat 1 addr, go to D_EVEN
    //           else: set addr for beat N+2, go to D_EVEN
    //
    // At segment boundary (last odd of segment):
    //   clear seg_ready for current segment
    //   beat 0 addr was pre-fetched by D_EVEN → go directly
    //   to D_EVEN (D_PRE eliminated for zero-bubble drain)
    //
    // D_PRE is only used at startup (D_IDLE → D_PRE → D_EVEN)
    // where no valid output is active.
    // =========================================================
    localparam D_IDLE       = 3'd0;
    localparam D_PRE        = 3'd1;   // 1-cycle BRAM read latency
    localparam D_EVEN       = 3'd2;
    localparam D_ODD        = 3'd3;
    localparam D_WAIT       = 3'd4;   // underrun: next segment not ready
    localparam D_DONE       = 3'd5;

    reg [2:0]  d_state;
    reg [3:0]  drain_seg;           // which segment we're draining (0-15)
    reg [3:0]  drain_beat;          // 0..15 within current segment
    reg [63:0] out_word;            // latched for odd sample
    reg [31:0] drain_count;         // total samples output
    // WARNING: tx_ptr is in clk (l_clk) domain. Read from s_axi_aclk
    // is best-effort (potential torn reads on multi-bit crossing).
    // DO NOT use tx_ptr for synchronization decisions on ARM.
    // Use tx_done (properly synced) for completion gating.
    reg [31:0] tx_ptr;

    // RD_PTR Gray-code CDC: l_clk -> s_axi_aclk
    wire [31:0] rd_ptr_gray = tx_ptr ^ (tx_ptr >> 1);
    reg  [31:0] rd_ptr_gray_reg;

    // Next-segment helper: (drain_seg + 1) mod 16
    wire [3:0] next_drain_seg = drain_seg + 4'd1;

    // Ring-buffer wrap helper for fill_ptr (power-of-2 BUF_SIZE_SAMPLES).
    // BUF_SIZE_SAMPLES = 2^23; bitwise AND mask eliminates 32-bit comparator
    // and subtractor from the critical path (F_SWAP → m_axi_araddr).
    wire [31:0] fill_ptr_next = (fill_ptr + SAMPLES_PER_BURST) & (BUF_SIZE_SAMPLES - 1);

    // =========================================================
    // Fill FSM
    // =========================================================
    always @(posedge clk) begin
        if (rst) begin
            f_state       <= F_IDLE;
            fill_seg      <= 0;
            fill_beat     <= 0;
            fill_ptr      <= 0;
            fill_remaining <= 0;
            tx_active     <= 0;
            m_axi_araddr  <= 0;
            m_axi_arvalid <= 0;
            m_axi_rready  <= 0;
            mem_wr_en     <= 0;
            mem_wr_addr   <= 0;
            mem_wr_data   <= 0;
            seg_ready     <= {NUM_SEGS{1'b0}};
            wrap_ar_issued   <= 0;
            wrap_ar_accepted <= 0;
        end else begin
            mem_wr_en <= 0;

            // Default: drain FSM clears its segment via drain_clear.
            // F_LATCH has its own explicit clear (restart) — the
            // if/else prevents a Verilog double-NBA on seg_ready.
            if (f_state != F_LATCH)
                seg_ready <= seg_ready & ~drain_clear;

            case (f_state)
                // -------------------------------------------------
                F_IDLE: begin
                    if (trigger_edge) begin
                        f_state <= F_LATCH;
                    end
                end

                // -------------------------------------------------
                F_LATCH: begin
                    if (lcl_enable) begin
                        tx_active      <= 1;
                        fill_seg       <= 0;
                        fill_ptr       <= 0;
                        fill_remaining <= lcl_tx_count;
                        // Clear all segment ready flags so the drain
                        // FSM doesn't consume stale BRAM data from a
                        // previous run on restart.
                        seg_ready      <= {NUM_SEGS{1'b0}};
                        wrap_ar_issued   <= 0;
                        wrap_ar_accepted <= 0;
                        if (lcl_stream && lcl_wr_ptr == 0) begin
                            // Stream mode: no data written yet.
                            // Wait for the ARM to feed data before
                            // issuing the first DDR read.
                            f_state <= F_WAIT;
                        end else begin
                            // Start first burst
                            m_axi_araddr   <= lcl_ddr_base;
                            m_axi_arvalid  <= 1;
                            fill_beat      <= 0;
                            f_state        <= F_RD_ADDR;
                        end
                    end else begin
                        f_state <= F_IDLE;
                    end
                end

                // -------------------------------------------------
                F_RD_ADDR: begin
                    if (m_axi_arready && m_axi_arvalid) begin
                        m_axi_arvalid <= 0;
                        m_axi_rready  <= 1;
                        f_state       <= F_RD_DATA;
                    end
                end

                // -------------------------------------------------
                F_RD_DATA: begin
                    // Speculative wrap AR acceptance tracking.
                    // Must be in F_RD_DATA (not parallel) to avoid conflicts
                    // with F_RD_ADDR's arready handling.
                    if (wrap_ar_issued && !wrap_ar_accepted &&
                        m_axi_arvalid && m_axi_arready) begin
                        m_axi_arvalid    <= 0;
                        wrap_ar_accepted <= 1;
                    end

                    if (m_axi_rvalid && m_axi_rready) begin
                        mem_wr_en   <= 1;
                        mem_wr_addr <= {fill_seg, fill_beat};  // 4+4 = 8 bits
                        mem_wr_data <= m_axi_rdata;
                        fill_beat   <= fill_beat + 4'd1;

                        // SPECULATIVE WRAP AR: Issue wrap burst AR early
                        // during the last burst's data reception.  The DDR
                        // controller begins row-activate in parallel with
                        // serving the current burst.  By F_SWAP time, the
                        // wrap data is partially/fully ready.
                        //
                        // NOTE: This path uses the traditional fill_remaining-
                        // based wrap boundary (<= SAMPLES_PER_BURST). The
                        // mid-buffer path (F_SWAP via fill_ptr_next & mask)
                        // uses a different mechanism. Two wrap mechanisms
                        // coexist and BOTH must be updated if the wrap
                        // boundary definition changes.
                        //
                        // Conditions: first beat of last burst, cyclic,
                        // enabled, and next segment is free.
                        if (fill_beat == 4'd0 &&
                            fill_remaining <= SAMPLES_PER_BURST &&
                            lcl_cyclic && lcl_enable &&
                            !wrap_ar_issued &&
                            !seg_ready[fill_seg + 4'd1] &&
                            !drain_clear[fill_seg + 4'd1]) begin
                            m_axi_araddr     <= lcl_ddr_base;
                            m_axi_arvalid    <= 1;
                            wrap_ar_issued   <= 1;
                            wrap_ar_accepted <= 0;
                        end

                        if (m_axi_rlast) begin
                            m_axi_rready <= 0;
                            f_state      <= F_SWAP;
                        end
                    end
                end

                // -------------------------------------------------
                F_SWAP: begin
                    // Mark this segment as ready for drain
                    seg_ready[fill_seg] <= 1;

                    // Advance bookkeeping for next burst
                    if (fill_remaining <= SAMPLES_PER_BURST) begin
                        // Last burst of this iteration
                        fill_remaining <= 0;
                        if (lcl_cyclic && lcl_enable) begin
                            // Continue cycling — reset to start of waveform.
                            fill_seg       <= fill_seg + 4'd1;
                            fill_ptr       <= 0;
                            fill_remaining <= lcl_tx_count;

                            if (wrap_ar_accepted) begin
                                // SPECULATIVE AR PATH: AR already accepted,
                                // DDR is working on row-activate.  Go straight
                                // to receiving data (skip F_RD_ADDR entirely).
                                m_axi_rready     <= 1;
                                fill_beat        <= 0;
                                wrap_ar_issued   <= 0;
                                wrap_ar_accepted <= 0;
                                f_state          <= F_RD_DATA;
                            end else if (wrap_ar_issued) begin
                                // AR issued but not yet accepted — continue
                                // holding arvalid, go to F_RD_ADDR to await
                                // acceptance.
                                fill_beat        <= 0;
                                wrap_ar_issued   <= 0;
                                wrap_ar_accepted <= 0;
                                f_state          <= F_RD_ADDR;
                            end else if (!seg_ready[fill_seg + 4'd1] && !drain_clear[fill_seg + 4'd1]) begin
                                // No speculative AR — issue normally
                                m_axi_araddr   <= lcl_ddr_base;
                                m_axi_arvalid  <= 1;
                                fill_beat      <= 0;
                                f_state        <= F_RD_ADDR;
                            end else begin
                                // Segment still occupied — fall back to F_WAIT
                                f_state        <= F_WAIT;
                            end
                        end else begin
                            // One-shot complete, or cyclic disabled mid-flight.
                            // Clean up any outstanding speculative wrap AR.
                            if (wrap_ar_accepted) begin
                                // AR accepted — must drain the read data
                                m_axi_rready     <= 1;
                                wrap_ar_issued   <= 0;
                                wrap_ar_accepted <= 0;
                                f_state          <= F_DRAIN_WRAP;
                            end else begin
                                if (wrap_ar_issued) begin
                                    // AR pending but not yet accepted — cancel
                                    m_axi_arvalid <= 0;
                                end
                                wrap_ar_issued   <= 0;
                                wrap_ar_accepted <= 0;
                                f_state          <= F_DONE;
                            end
                        end
                    end else if (!lcl_enable) begin
                        // Disabled mid-stream — stop after this burst.
                        // If a speculative wrap AR is outstanding, must drain it.
                        if (wrap_ar_issued && !wrap_ar_accepted) begin
                            // AR pending but not yet accepted — clear arvalid,
                            // then go to F_DONE (no data to drain since AR
                            // wasn't accepted).
                            m_axi_arvalid    <= 0;
                            wrap_ar_issued   <= 0;
                            wrap_ar_accepted <= 0;
                            f_state          <= F_DONE;
                        end else if (wrap_ar_accepted) begin
                            // AR accepted — must drain the read data (rlast)
                            // before going idle, otherwise AXI bus hangs.
                            m_axi_rready     <= 1;
                            wrap_ar_issued   <= 0;
                            wrap_ar_accepted <= 0;
                            f_state          <= F_DRAIN_WRAP;
                        end else begin
                            wrap_ar_issued   <= 0;
                            wrap_ar_accepted <= 0;
                            f_state          <= F_DONE;
                        end
                    end else begin
                        fill_remaining <= fill_remaining - SAMPLES_PER_BURST;
                        fill_ptr       <= fill_ptr_next;
                        fill_seg       <= fill_seg + 4'd1;
                        // Normal mid-waveform: check if next segment is free
                        // for immediate issue (same optimization as cyclic wrap).
                        // Uses fill_ptr_next for DDR address so stream-mode
                        // ring-buffer wrap is transparent.
                        // In stream mode, also check we haven't caught up to
                        // the ARM's write pointer before issuing the next read.
                        if ((lcl_stream && fill_ptr_next >= lcl_wr_ptr) ||
                            (seg_ready[fill_seg + 4'd1] || drain_clear[fill_seg + 4'd1])) begin
                            f_state <= F_WAIT;
                        end else begin
                            m_axi_araddr   <= lcl_ddr_base + {fill_ptr_next, 2'b00};
                            m_axi_arvalid  <= 1;
                            fill_beat      <= 0;
                            f_state        <= F_RD_ADDR;
                        end
                    end
                end

                // -------------------------------------------------
                F_WAIT: begin
                    // Check for disable before issuing next burst
                    if (!lcl_enable) begin
                        // Handle outstanding speculative wrap AR on stop
                        if (wrap_ar_issued && !wrap_ar_accepted) begin
                            m_axi_arvalid    <= 0;
                            wrap_ar_issued   <= 0;
                            wrap_ar_accepted <= 0;
                            f_state          <= F_DONE;
                        end else if (wrap_ar_accepted) begin
                            m_axi_rready     <= 1;
                            wrap_ar_issued   <= 0;
                            wrap_ar_accepted <= 0;
                            f_state          <= F_DRAIN_WRAP;
                        end else begin
                            f_state <= F_DONE;
                        end
                    end else if (!seg_ready[fill_seg]) begin
                        // Target segment is free — but in stream mode,
                        // also check we haven't caught up to the ARM's
                        // write pointer (lcl_wr_ptr).  If fill_ptr has
                        // reached wr_ptr, stay in F_WAIT — the ARM hasn't
                        // written the next chunk yet.
                        if (lcl_stream && fill_ptr >= lcl_wr_ptr) begin
                            // Stall: caught up to write pointer
                        end else begin
                            m_axi_araddr   <= lcl_ddr_base + {fill_ptr, 2'b00};
                            m_axi_arvalid  <= 1;
                            fill_beat      <= 0;
                            f_state        <= F_RD_ADDR;
                        end
                    end
                end

                // -------------------------------------------------
                F_DRAIN_WRAP: begin
                    // Drain outstanding speculative wrap read on stop.
                    // Keep rready high until rlast arrives, then go to F_DONE.
                    // We discard the data (don't write to BRAM).
                    if (m_axi_rvalid && m_axi_rready && m_axi_rlast) begin
                        m_axi_rready <= 0;
                        f_state      <= F_DONE;
                    end
                end

                // -------------------------------------------------
                F_DONE: begin
                    // One-shot complete.  Wait for drain to finish.
                    tx_active <= 0;
                    if (trigger_edge) begin
                        f_state <= F_LATCH;
                    end
                end

                default: f_state <= F_IDLE;
            endcase
        end
    end

    // =========================================================
    // Drain FSM
    // =========================================================
    reg [NUM_SEGS-1:0] drain_clear;

    always @(posedge clk) begin
        if (rst) begin
            d_state     <= D_IDLE;
            drain_seg   <= 0;
            drain_beat  <= 0;
            drain_count <= 0;
            tx_ptr      <= 0;
            tx_done     <= 0;
            out_word    <= 0;
            re_out      <= 0;
            im_out      <= 0;
            valid_out   <= 0;
            mem_rd_addr <= 0;
            drain_clear <= {NUM_SEGS{1'b0}};
        end else begin
            valid_out   <= 0;
            drain_clear <= {NUM_SEGS{1'b0}};

            case (d_state)
                // -------------------------------------------------
                D_IDLE: begin
                    // Wait for first segment to become ready
                    tx_done <= 0;
                    if (seg_ready[0]) begin
                        drain_seg   <= 0;
                        drain_beat  <= 0;
                        drain_count <= 0;
                        tx_ptr      <= 0;
                        mem_rd_addr <= {4'b0000, 4'd0};
                        d_state     <= D_PRE;
                    end
                end

                // -------------------------------------------------
                D_PRE: begin
                    // 1-cycle BRAM read latency wait.  Address was set
                    // by the previous state (D_IDLE).
                    // Pre-fetch next beat for D_ODD's look-ahead.
                    mem_rd_addr <= {drain_seg, drain_beat[3:0] + 4'd1};  // 4+4 = 8 bits
                    d_state     <= D_EVEN;
                end

                // -------------------------------------------------
                D_EVEN: begin
                    // mem_rd_data has current beat (set >=1 cycle ago).
                    // Output unconditionally — do NOT gate on dac_valid
                    // (see header comment for rationale).
                    out_word  <= mem_rd_data;
                    re_out    <= {mem_rd_data[11:0], 4'b0000};
                    im_out    <= {mem_rd_data[23:12], 4'b0000};
                    valid_out <= 1;
                    drain_count <= drain_count + 32'd1;
                    tx_ptr      <= tx_ptr + 32'd1;

                    // Pre-fetch: at segment boundary or cyclic wrap,
                    // pre-fetch beat 0 of the next segment.  Otherwise
                    // the pre-fetch addr was already set by D_PRE,
                    // D_ODD, or D_WAIT — don't overwrite it.
                    if (drain_beat == BURST_LEN[3:0] - 4'd1 ||
                        (drain_count + 32'd2 >= lcl_tx_count && lcl_cyclic))
                        mem_rd_addr <= {next_drain_seg, 4'd0};

                    d_state     <= D_ODD;
                end

                // -------------------------------------------------
                D_ODD: begin
                    // Output unconditionally — do NOT gate on dac_valid.
                    re_out    <= {out_word[43:32], 4'b0000};
                    im_out    <= {out_word[55:44], 4'b0000};
                    valid_out <= 1;
                    drain_count <= drain_count + 32'd1;
                    tx_ptr      <= tx_ptr + 32'd1;

                    if (drain_count + 32'd1 >= lcl_tx_count) begin
                        // All samples output for this iteration
                        drain_clear[drain_seg] <= 1;
                        if (lcl_cyclic && lcl_enable) begin
                            // Continue cycling
                            drain_count <= 0;
                            tx_ptr      <= 0;
                            drain_seg   <= next_drain_seg;
                            if (seg_ready[next_drain_seg]) begin
                                drain_beat  <= 0;
                                // Beat 0 addr was pre-fetched in D_EVEN.
                                // Set beat 1 addr (replaces D_PRE's job).
                                mem_rd_addr <= {next_drain_seg, 4'd1};
                                d_state     <= D_EVEN;
                            end else begin
                                d_state <= D_WAIT;
                            end
                        end else begin
                            // One-shot done, or cyclic disabled mid-flight
                            tx_done   <= 1;
                            d_state   <= D_DONE;
                        end
                    end else if (drain_beat == BURST_LEN[3:0] - 4'd1) begin
                        // Last beat of this segment — swap to next segment
                        drain_clear[drain_seg] <= 1;
                        drain_seg <= next_drain_seg;
                        if (seg_ready[next_drain_seg]) begin
                            drain_beat  <= 0;
                            // Beat 0 addr was pre-fetched in D_EVEN.
                            // Set beat 1 addr (replaces D_PRE's job).
                            mem_rd_addr <= {next_drain_seg, 4'd1};
                            d_state     <= D_EVEN;
                        end else begin
                            // Underrun: next segment not ready yet
                            d_state <= D_WAIT;
                        end
                    end else begin
                        // Continue within current segment.
                        // mem_rd_data already has next beat (addr set
                        // in previous D_EVEN or D_PRE).
                        // Set addr for beat after next.
                        drain_beat  <= drain_beat + 4'd1;
                        mem_rd_addr <= {drain_seg, drain_beat[3:0] + 4'd2};
                        d_state     <= D_EVEN;
                    end
                end

                // -------------------------------------------------
                D_WAIT: begin
                    // Underrun stall — next segment not ready.
                    //
                    // If lcl_enable has been cleared (stop requested),
                    // exit cleanly rather than waiting for fill that
                    // will never come.
                    //
                    // SAFETY: D_WAIT is always entered from D_ODD's
                    // segment-swap path.  D_EVEN (2 cycles earlier)
                    // pre-fetches beat 0 of next_drain_seg.  D_ODD does
                    // NOT overwrite mem_rd_addr on the segment-swap-to-
                    // D_WAIT path, so the BRAM read port still holds
                    // that address.  By the time D_WAIT's first cycle
                    // executes, mem_rd_data already contains beat 0.
                    //
                    // Speculatively read beat 0 so data is ready
                    // the cycle seg_ready goes high.
                    if (!lcl_enable) begin
                        tx_done <= 1;
                        d_state <= D_DONE;
                    end else begin
                        mem_rd_addr <= {drain_seg, 4'd0};  // 4+4 = 8 bits
                        if (seg_ready[drain_seg]) begin
                            drain_beat  <= 0;
                            // Beat 0 data is already in mem_rd_data
                            // (speculative read captured it last cycle).
                            // Set beat 1 addr for the next D_EVEN.
                            mem_rd_addr <= {drain_seg, 4'd1};
                            d_state     <= D_EVEN;
                        end
                    end
                end

                // -------------------------------------------------
                D_DONE: begin
                    // One-shot complete.  Restart on trigger.
                    if (trigger_edge) begin
                        d_state <= D_IDLE;
                    end
                end

                default: d_state <= D_IDLE;
            endcase

            rd_ptr_gray_reg <= rd_ptr_gray;
        end
    end

    // =========================================================
    // AXI4-Lite slave (s_axi_aclk domain)
    // =========================================================
    wire s_axi_rst = ~s_axi_aresetn;

    reg        aw_done_l, w_done_l;
    reg [31:0] aw_addr_l;
    reg [31:0] w_data_l;

    always @(posedge s_axi_aclk) begin
        if (s_axi_rst) begin
            s_axi_awready      <= 0;
            s_axi_wready       <= 0;
            s_axi_bvalid       <= 0;
            s_axi_bresp        <= 0;
            aw_done_l          <= 0;
            w_done_l           <= 0;
            aw_addr_l          <= 0;
            w_data_l           <= 0;
            reg_enable_axi     <= 0;
            trigger_toggle_axi <= 0;
            reg_cyclic_axi     <= 0;
            reg_stream_axi     <= 0;
            reg_ddr_base_axi   <= 0;
            reg_tx_count_axi   <= 0;
            reg_wr_ptr_axi     <= 0;
        end else begin
            if (s_axi_awvalid && !aw_done_l) begin
                s_axi_awready <= 1;
                aw_addr_l     <= s_axi_awaddr;
                aw_done_l     <= 1;
            end else
                s_axi_awready <= 0;

            if (s_axi_wvalid && !w_done_l) begin
                s_axi_wready <= 1;
                w_data_l     <= s_axi_wdata;
                w_done_l     <= 1;
            end else
                s_axi_wready <= 0;

            if (aw_done_l && w_done_l && !s_axi_bvalid) begin
                case (aw_addr_l[7:0])
                    REG_CONTROL[7:0]: begin
                        reg_enable_axi <= w_data_l[0];
                        if (w_data_l[1])
                            trigger_toggle_axi <= ~trigger_toggle_axi;
                        reg_cyclic_axi <= w_data_l[2];
                        reg_stream_axi <= w_data_l[3];
                    end
                    REG_DDR_BASE[7:0]: reg_ddr_base_axi <= w_data_l;
                    REG_TX_COUNT[7:0]: reg_tx_count_axi <= w_data_l;
                    REG_WR_PTR[7:0]:   reg_wr_ptr_axi   <= w_data_l;
                    default: ;
                endcase
                s_axi_bvalid <= 1;
                s_axi_bresp  <= 2'b00;
                aw_done_l    <= 0;
                w_done_l     <= 0;
            end

            if (s_axi_bvalid && s_axi_bready)
                s_axi_bvalid <= 0;
        end
    end

    // Read path (s_axi_aclk domain)
    reg        ar_done_l;
    reg [31:0] ar_addr_l;

    always @(posedge s_axi_aclk) begin
        if (s_axi_rst) begin
            s_axi_arready <= 0;
            s_axi_rvalid  <= 0;
            s_axi_rresp   <= 0;
            s_axi_rdata   <= 0;
            ar_done_l     <= 0;
            ar_addr_l     <= 0;
        end else begin
            if (s_axi_arvalid && !ar_done_l) begin
                s_axi_arready <= 1;
                ar_addr_l     <= s_axi_araddr;
                ar_done_l     <= 1;
            end else
                s_axi_arready <= 0;

            if (ar_done_l && !s_axi_rvalid) begin
                case (ar_addr_l[7:0])
                    REG_CONTROL[7:0]:  s_axi_rdata <= {28'b0, reg_stream_axi, reg_cyclic_axi, 1'b0, reg_enable_axi};
                    REG_STATUS[7:0]:   s_axi_rdata <= {30'b0, tx_done_sync2, tx_active_sync2};
                    REG_DDR_BASE[7:0]: s_axi_rdata <= reg_ddr_base_axi;
                    REG_TX_COUNT[7:0]: s_axi_rdata <= reg_tx_count_axi;
                    REG_TX_PTR[7:0]:   s_axi_rdata <= tx_ptr;  // UNSYNCHRONIZED: display only, not for sync decisions
                    REG_WR_PTR[7:0]:   s_axi_rdata <= reg_wr_ptr_axi;
                    REG_RD_PTR[7:0]:   s_axi_rdata <= rd_ptr_readable;  // Gray-decoded, 2-cycle stale
                    default:           s_axi_rdata <= 32'hDEADBEEF;
                endcase
                s_axi_rvalid <= 1;
                s_axi_rresp  <= 2'b00;
                ar_done_l    <= 0;
            end

            if (s_axi_rvalid && s_axi_rready)
                s_axi_rvalid <= 0;
        end
    end

endmodule
