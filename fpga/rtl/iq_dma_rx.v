// SPDX-License-Identifier: MIT

`timescale 1ns/1ps
// fpga/rtl/iq_dma_rx.v
//
// Continuous IQ DMA writer: streams ADC samples into DDR via AXI3 HP0
// in a circular buffer.  No triggering, no gating, no slots — just a
// continuous write pointer that wraps.
//
// IQ samples arrive from adc_sync at the configured ADC rate into a
// 100 MHz fabric clock. Accumulate 32 samples (16 pairs) in a
// BRAM-backed ring of bursts, then burst-write to DDR as 16 × 64-bit
// beats.
//
// The ring holds RING_BURSTS bursts in one BRAM36.  The fill side
// reserves the next free slot before accepting a burst, so the reader
// can be many bursts behind.  AW and W are decoupled from B: up to
// MAX_OUTSTANDING address writes may await responses while new bursts
// keep streaming at one beat per clock.  wr_ptr advances only when a
// response lands, so it still means "all data up to here is in DDR".
//
// Previous register-file implementation (buf0[0:31], buf1[0:31]) used
// per-element write-enables that synthesised to ~41 unique control sets,
// pushing the Zynq 7010 past its 4400-slice control set budget.  BRAM
// contributes zero control sets.
//
// DDR packing (matches ring_dma for compatibility):
//   Each 32-bit word: {8'b0, im[11:0], re[11:0]}
//   Each 64-bit beat: {word_odd, word_even} (even at lower address)
//
// ARM control via AXI4-Lite:
//   0x00  CONTROL       R/W  [0]=enable
//   0x04  STATUS        R    [0]=active, [31:16]=overflow_count
//   0x08  DDR_BASE      R/W  Physical DDR base address
//   0x0C  WR_PTR        R    Current write pointer (sample index, 25-bit)
//   0x10  SAMPLE_COUNT  R    Total samples written (32-bit, wraps)

module iq_dma_rx #(
    parameter DDR_BUF_SAMPLES = 33554432, // 128 MB / 4 bytes per sample
    parameter BURST_LEN       = 16,       // AXI3 max burst (16 beats of 64 bits)
    parameter RING_BURSTS     = 16        // burst slots in the BRAM ring
) (
    (* X_INTERFACE_INFO = "xilinx.com:signal:clock:1.0 clk CLK" *)
    (* X_INTERFACE_PARAMETER = "ASSOCIATED_BUSIF m_axi:s_axi, ASSOCIATED_RESET rst" *)
    input  wire        clk,

    (* X_INTERFACE_INFO = "xilinx.com:signal:reset:1.0 rst RST" *)
    (* X_INTERFACE_PARAMETER = "POLARITY ACTIVE_HIGH" *)
    input  wire        rst,

    // IQ input (from adc_sync, sys_cpu_clk domain, configured ADC rate)
    input  wire [11:0] re_in,
    input  wire [11:0] im_in,
    input  wire        valid_in,

    // Write pointer output (sys_cpu_clk domain — consumers must be same clock).
    // If needed in l_clk domain, add gray-code synchronizer.
    // NOTE: Without explicit interface markup, Vivado's automatic interface
    // inference (in module-reference mode) can absorb this port into an
    // inferred FIFO interface based on the name pattern "wr_ptr", leaving
    // it disconnected in the block design (reads as 0 on hardware).
    (* X_INTERFACE_INFO = "xilinx.com:signal:data:1.0 ddr_wr_ptr DATA" *)
    (* DONT_TOUCH = "TRUE" *)
    output wire [24:0] ddr_wr_ptr_out,

    // Sample-accurate DDR ring position: bits [24:0] of the total sample
    // counter. Increments on every valid_in (when enabled), wraps naturally
    // at 2^25 = DDR_BUF_SAMPLES. Unlike ddr_wr_ptr_out (which jumps by 32
    // on burst completion), this advances one sample at a time.
    (* X_INTERFACE_INFO = "xilinx.com:signal:data:1.0 sample_cnt DATA" *)
    (* DONT_TOUCH = "TRUE" *)
    output wire [24:0] sample_cnt_out,

    // --- AXI3 write-only master (to PS7 HP0, 64-bit) ---
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 m_axi AWADDR" *)
    (* X_INTERFACE_PARAMETER = "PROTOCOL AXI3, FREQ_HZ 100000000, ADDR_WIDTH 32, DATA_WIDTH 64, HAS_BURST 1, HAS_LOCK 0, HAS_PROT 0, HAS_CACHE 0, HAS_QOS 0, HAS_REGION 0, HAS_WSTRB 1, MAX_BURST_LENGTH 16, NUM_WRITE_OUTSTANDING 8, NUM_READ_OUTSTANDING 0, SUPPORTS_NARROW_BURST 0, READ_WRITE_MODE WRITE_ONLY" *)
    output reg  [31:0] m_axi_awaddr,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 m_axi AWLEN" *)
    output wire [3:0]  m_axi_awlen,     // BURST_LEN - 1 = 15
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 m_axi AWSIZE" *)
    output wire [2:0]  m_axi_awsize,    // 3'b011 = 8 bytes
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 m_axi AWBURST" *)
    output wire [1:0]  m_axi_awburst,   // 2'b01 = INCR
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 m_axi AWVALID" *)
    output reg         m_axi_awvalid,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 m_axi AWREADY" *)
    input  wire        m_axi_awready,

    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 m_axi AWID" *)
    output wire [5:0]  m_axi_awid,

    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 m_axi WDATA" *)
    output wire [63:0] m_axi_wdata,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 m_axi WSTRB" *)
    output wire [7:0]  m_axi_wstrb,     // 8'hFF
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 m_axi WID" *)
    output wire [5:0]  m_axi_wid,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 m_axi WLAST" *)
    output wire        m_axi_wlast,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 m_axi WVALID" *)
    output wire        m_axi_wvalid,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 m_axi WREADY" *)
    input  wire        m_axi_wready,

    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 m_axi BRESP" *)
    input  wire [1:0]  m_axi_bresp,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 m_axi BVALID" *)
    input  wire        m_axi_bvalid,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 m_axi BREADY" *)
    output wire        m_axi_bready,

    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 m_axi BID" *)
    input  wire [5:0]  m_axi_bid,

    // --- AXI4-Lite slave (ARM control/status) ---
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
    // AXI3 constants — 64-bit, 2 samples packed per beat
    // =========================================================
    localparam [3:0] BURST_LAST = BURST_LEN[3:0] - 4'd1;

    assign m_axi_awlen   = BURST_LAST;              // 15 = 16 beats
    assign m_axi_awsize  = 3'b011;                  // 8 bytes per beat
    assign m_axi_awburst = 2'b01;                   // INCR
    assign m_axi_wstrb   = 8'hFF;                   // all bytes valid
    assign m_axi_awid    = 6'b0;
    assign m_axi_wid     = 6'b0;

    // =========================================================
    // Derived parameters
    // =========================================================
    localparam SAMPLES_PER_BURST = BURST_LEN * 2;   // 32 samples per burst
    localparam SLOT_W            = $clog2(RING_BURSTS);
    localparam BEAT_W            = (BURST_LEN <= 1) ? 1 : $clog2(BURST_LEN);
    localparam MEM_AW            = $clog2(RING_BURSTS * BURST_LEN);
    localparam BURST_CNT_W       = SLOT_W + 1;
    localparam MAX_OUTSTANDING   = 8;
    localparam AW_LEAD           = 2;

    localparam [BURST_CNT_W-1:0] RING_FULL = RING_BURSTS[BURST_CNT_W-1:0];
    localparam [BURST_CNT_W-1:0] MAX_OUT_V = MAX_OUTSTANDING[BURST_CNT_W-1:0];
    localparam [BURST_CNT_W-1:0] AW_LEAD_V = AW_LEAD[BURST_CNT_W-1:0];

    function [31:0] extend32(input [BEAT_W-1:0] value);
        extend32 = {{(32-BEAT_W){1'b0}}, value};
    endfunction

    initial begin
        if (DDR_BUF_SAMPLES % SAMPLES_PER_BURST != 0) begin
            $error("DDR_BUF_SAMPLES (%0d) must be a multiple of SAMPLES_PER_BURST (%0d)",
                   DDR_BUF_SAMPLES, SAMPLES_PER_BURST);
        end
        // BURST_LAST is a 4-bit field, so a longer burst would wrap silently.
        // AXI3 caps a burst at 16 beats anyway.
        if (BURST_LEN < 1 || BURST_LEN > 16) begin
            $error("BURST_LEN (%0d) must be 1..16 (AXI3 limit, 4-bit awlen)",
                   BURST_LEN);
        end
        // The next burst's first read uses the beat counter's low bits, and
        // the slot address multiplies by BURST_LEN under a fixed address
        // width.  Both assume a power of two.
        if ((BURST_LEN & (BURST_LEN - 1)) != 0) begin
            $error("BURST_LEN (%0d) must be a power of two", BURST_LEN);
        end
        // The write pointer is 25 bits wide and is zero-extended when it
        // forms the DDR byte address.
        if (DDR_BUF_SAMPLES > (1 << 25)) begin
            $error("DDR_BUF_SAMPLES (%0d) must not exceed 2**25 (wr_ptr_r width)",
                   DDR_BUF_SAMPLES);
        end
        // Slot selection uses a bit slice, so the ring size must be a power
        // of two.  Eight is the smallest size that can hold the AXI3
        // outstanding-write depth.
        if (RING_BURSTS < MAX_OUTSTANDING || RING_BURSTS > 32) begin
            $error("RING_BURSTS (%0d) must be %0d..32",
                   RING_BURSTS, MAX_OUTSTANDING);
        end
        if ((RING_BURSTS & (RING_BURSTS - 1)) != 0) begin
            $error("RING_BURSTS (%0d) must be a power of two", RING_BURSTS);
        end
        if (RING_BURSTS * BURST_LEN > 512) begin
            $error("RING_BURSTS * BURST_LEN (%0d) must fit one 512x64 BRAM36",
                   RING_BURSTS * BURST_LEN);
        end
        // If the DDR buffer were smaller than the ring, an address pass
        // could lap an unacknowledged burst.
        if (DDR_BUF_SAMPLES < RING_BURSTS * SAMPLES_PER_BURST) begin
            $error("DDR_BUF_SAMPLES (%0d) must hold at least one full ring (%0d)",
                   DDR_BUF_SAMPLES, RING_BURSTS * SAMPLES_PER_BURST);
        end
    end

    // =========================================================
    // AXI-Lite register map
    // =========================================================
    localparam REG_CONTROL      = 32'h00;
    localparam REG_STATUS       = 32'h04;
    localparam REG_DDR_BASE     = 32'h08;
    localparam REG_WR_PTR       = 32'h0C;
    localparam REG_SAMPLE_COUNT = 32'h10;
    localparam REG_WRAP_COUNT   = 32'h14;

    // =========================================================
    // Control/status registers
    // =========================================================
    reg         reg_enable;
    reg [31:0]  reg_ddr_base;
    (* DONT_TOUCH = "TRUE" *)
    reg [24:0]  wr_ptr_r;
    reg [31:0]  reg_sample_count;
    reg [15:0]  reg_wrap_count;
    reg [15:0]  overflow_count;
    reg         base_configured;

    assign ddr_wr_ptr_out = wr_ptr_r;
    assign sample_cnt_out = reg_sample_count[24:0];

    // =========================================================
    // Burst ring — one BRAM36, RING_BURSTS × BURST_LEN × 64 bits
    //
    // The fill side writes the slot selected by produced_bursts; the
    // read side drains the slot selected by sent_bursts.  A slot is
    // reused only after its last W beat has been accepted, so response
    // latency does not block new bursts.
    // =========================================================
    (* ram_style = "block" *)
    reg [63:0] burst_mem [0:RING_BURSTS*BURST_LEN-1];

    // Fill-side BRAM write port
    reg              mem_wr_en;
    reg [MEM_AW-1:0] mem_wr_addr;
    reg [63:0]       mem_wr_data;

    // Read-side BRAM read port (1-cycle latency)
    reg [MEM_AW-1:0] mem_rd_addr;
    reg [63:0]       mem_rd_data;

    always @(posedge clk) begin
        if (mem_wr_en)
            burst_mem[mem_wr_addr] <= mem_wr_data;
    end

    always @(posedge clk) begin
        mem_rd_data <= burst_mem[mem_rd_addr];
    end

    // =========================================================
    // Burst counters — produced → AW → W → B, all in order
    // =========================================================
    reg [BURST_CNT_W-1:0] produced_bursts;  // fill side, advances on burst completion
    reg [BURST_CNT_W-1:0] aw_bursts;        // AW side, advances on AW handshake
    reg [BURST_CNT_W-1:0] sent_bursts;      // W side, advances on last W beat
    reg [BURST_CNT_W-1:0] acked_bursts;     // B side, advances on B handshake

    wire [BURST_CNT_W-1:0] ring_occupancy = produced_bursts - sent_bursts;
    wire [BURST_CNT_W-1:0] aw_pending     = produced_bursts - aw_bursts;
    wire [BURST_CNT_W-1:0] aw_outstanding = aw_bursts - acked_bursts;

    // =========================================================
    // Fill logic — reserves a ring slot, then accepts 32 samples
    // =========================================================
    reg [5:0]        fill_count;    // 0..SAMPLES_PER_BURST
    reg [BEAT_W-1:0] pair_count;    // 0..BURST_LEN-1
    reg              has_even;
    reg [23:0]       pending_even;
    reg [24:0]       aw_sample_ptr; // DDR sample index for the next AW

    wire [31:0] fill_word_addr32 = produced_bursts * BURST_LEN
                                   + extend32(pair_count);

    always @(posedge clk) begin
        if (rst) begin
            fill_count       <= 0;
            pair_count       <= 0;
            produced_bursts  <= 0;
            has_even         <= 0;
            pending_even     <= 0;
            reg_sample_count <= 0;
            reg_wrap_count   <= 0;
            overflow_count   <= 0;
            mem_wr_en        <= 0;
            mem_wr_addr      <= 0;
            mem_wr_data      <= 0;
        end else begin
            mem_wr_en <= 0;

            if (reg_enable && valid_in && base_configured) begin
                if (fill_count == 0 && ring_occupancy >= RING_FULL) begin
                    // Every slot is filled and no W burst has completed.
                    // A burst reserves its slot before its first sample,
                    // so this is the only place samples can be dropped.
                    overflow_count <= overflow_count + 1;
                end else if (fill_count < SAMPLES_PER_BURST[5:0]) begin
                    if (!has_even) begin
                        // First sample of a pair — latch it.
                        pending_even <= {im_in, re_in};
                        has_even     <= 1;
                    end else begin
                        // Second sample — write the packed 64-bit word.
                        mem_wr_en   <= 1;
                        mem_wr_addr <= fill_word_addr32[MEM_AW-1:0];
                        mem_wr_data <= {
                            {8'b0, im_in, re_in},       // odd: high 32 bits
                            {8'b0, pending_even}        // even: low 32 bits
                        };
                        has_even    <= 0;
                        pair_count  <= pair_count + 1;
                    end

                    fill_count       <= fill_count + 1;
                    reg_sample_count <= reg_sample_count + 1;
                    if (reg_sample_count == 32'hFFFFFFFF)
                        reg_wrap_count <= reg_wrap_count + 1;

                    if (fill_count == SAMPLES_PER_BURST[5:0] - 6'd1) begin
                        // Last sample of the burst.  The final pair write
                        // was issued above; release the slot to the reader.
                        produced_bursts <= produced_bursts + 1;
                        fill_count      <= 0;
                        pair_count      <= 0;
                        has_even        <= 0;
                    end
                end else begin
                    // Defensive: fill_count is never above SAMPLES_PER_BURST.
                    overflow_count <= overflow_count + 1;
                end
            end
        end
    end

    // =========================================================
    // AW engine — issue addresses ahead of responses, in order
    // =========================================================
    // aw_sample_ptr tracks the DDR position of the next AW.  It advances
    // on AW, not on B, so several bursts can be addressed at once.  wr_ptr
    // remains the B-driven "landed" pointer.
    always @(posedge clk) begin
        if (rst) begin
            aw_bursts     <= 0;
            aw_sample_ptr <= 0;
            m_axi_awaddr  <= 0;
            m_axi_awvalid <= 0;
        end else begin
            if (!m_axi_awvalid) begin
                // Do not run addresses more than a burst or two ahead of
                // the W stream.  An interconnect can hold a write credit
                // from AW until the data arrives, so racing far ahead
                // steals grants from other masters on a shared port.
                if (aw_pending != 0 && aw_outstanding < MAX_OUT_V
                        && ((aw_bursts - sent_bursts) < AW_LEAD_V)) begin
                    m_axi_awaddr  <= reg_ddr_base + {5'b0, aw_sample_ptr, 2'b00};
                    m_axi_awvalid <= 1;
                end
            end else if (m_axi_awready) begin
                m_axi_awvalid <= 0;
                aw_bursts     <= aw_bursts + 1;
                if ({7'b0, aw_sample_ptr} + SAMPLES_PER_BURST >= DDR_BUF_SAMPLES)
                    aw_sample_ptr <= 0;
                else
                    aw_sample_ptr <= aw_sample_ptr + SAMPLES_PER_BURST[24:0];
            end
        end
    end

    // =========================================================
    // W engine — 1 beat/clock with a prefetch skid
    //
    // The BRAM output register needs one cycle after the address, so a
    // beat passes through two pipeline stages before it reaches the
    // skid.  The skid is four entries deep and credits (in_system)
    // throttle reads, which keeps 1 beat/clock without the old gap
    // state and absorbs any WREADY stall.
    // =========================================================
    reg              w_busy;
    reg [BEAT_W:0]   rd_beat;     // next beat to issue (0..BURST_LEN)
    reg [BEAT_W-1:0] out_beat;    // next beat on the bus
    reg              rd_issued;   // address in flight; data arrives next cycle
    reg              rd_push;     // mem_rd_data is valid and ready to push

    reg [2:0]  fifo_count;
    reg [1:0]  fifo_rd_ptr;
    reg [1:0]  fifo_wr_ptr;
    reg [63:0] fifo_mem [0:3];

    wire [31:0] read_word_addr32 = sent_bursts * BURST_LEN
                                   + extend32(rd_beat[BEAT_W-1:0]);
    wire        fifo_empty = (fifo_count == 0);
    wire        w_pop = w_busy && !fifo_empty && m_axi_wready;
    wire        w_push = rd_push;
    wire [2:0]  fifo_count_next = fifo_count
                                + (w_push ? 3'd1 : 3'd0)
                                - (w_pop ? 3'd1 : 3'd0);
    wire [2:0]  in_system = fifo_count
                          + (rd_push ? 3'd1 : 3'd0)
                          + (rd_issued ? 3'd1 : 3'd0);
    wire        can_issue = w_busy
                          && (rd_beat < BURST_LEN)
                          && ((in_system - (w_pop ? 3'd1 : 3'd0)) < 3'd4);

    assign m_axi_wdata  = fifo_mem[fifo_rd_ptr];
    assign m_axi_wvalid = w_busy && !fifo_empty;
    assign m_axi_wlast  = w_busy && !fifo_empty && (out_beat == BURST_LAST);

    always @(posedge clk) begin
        if (rst) begin
            w_busy      <= 0;
            rd_beat     <= 0;
            out_beat    <= 0;
            rd_issued   <= 0;
            rd_push     <= 0;
            fifo_count  <= 0;
            fifo_rd_ptr <= 0;
            fifo_wr_ptr <= 0;
            sent_bursts <= 0;
            mem_rd_addr <= 0;
        end else begin
            // Push the beat whose read completed last cycle.
            if (rd_push) begin
                fifo_mem[fifo_wr_ptr] <= mem_rd_data;
                fifo_wr_ptr <= fifo_wr_ptr + 2'd1;
            end

            // Pop the head when the bus accepts it.
            if (w_pop) begin
                fifo_rd_ptr <= fifo_rd_ptr + 2'd1;
                out_beat    <= out_beat + 1;
            end

            fifo_count <= fifo_count_next;
            rd_push    <= rd_issued;

            if (!w_busy) begin
                if (sent_bursts != aw_bursts) begin
                    // A filled burst has an accepted address; stream it.
                    w_busy      <= 1;
                    rd_beat     <= 1;
                    out_beat    <= 0;
                    mem_rd_addr <= read_word_addr32[MEM_AW-1:0];
                    rd_issued   <= 1;
                end else begin
                    rd_issued <= 0;
                end
            end else if (can_issue) begin
                mem_rd_addr <= read_word_addr32[MEM_AW-1:0];
                rd_beat     <= rd_beat + 1;
                rd_issued   <= 1;
            end else begin
                rd_issued <= 0;
            end

            // Last beat of this burst was accepted.
            if (w_pop && (out_beat == BURST_LAST)) begin
                w_busy      <= 0;
                sent_bursts <= sent_bursts + 1;
            end
        end
    end

    // =========================================================
    // B engine — advance wr_ptr one burst per response, in order
    // =========================================================
    assign m_axi_bready = (aw_bursts != acked_bursts);

    wire dma_active = w_busy || (produced_bursts != acked_bursts);

    always @(posedge clk) begin
        if (rst) begin
            acked_bursts <= 0;
            wr_ptr_r     <= 0;
        end else begin
            if (m_axi_bvalid && m_axi_bready) begin
                acked_bursts <= acked_bursts + 1;
                if ({7'b0, wr_ptr_r} + SAMPLES_PER_BURST >= DDR_BUF_SAMPLES)
                    wr_ptr_r <= 0;
                else
                    wr_ptr_r <= wr_ptr_r + SAMPLES_PER_BURST[24:0];
            end
        end
    end

    // =========================================================
    // AXI4-Lite slave — control registers
    // (Same pattern as ring_dma.v: independent AW/W capture with
    //  aw_done/w_done flags, combined write execution, AR+mux read)
    // =========================================================
    reg        aw_done, w_done;
    reg [31:0] aw_addr;
    reg [31:0] w_data;

    // Write path
    always @(posedge clk) begin
        if (rst) begin
            s_axi_awready  <= 0;
            s_axi_wready   <= 0;
            s_axi_bvalid   <= 0;
            s_axi_bresp    <= 0;
            aw_done        <= 0;
            w_done         <= 0;
            aw_addr        <= 0;
            w_data         <= 0;
            reg_enable     <= 0;
            reg_ddr_base   <= 0;
            base_configured <= 0;
        end else begin
            // AW handshake
            if (s_axi_awvalid && !aw_done) begin
                s_axi_awready <= 1;
                aw_addr       <= s_axi_awaddr;
                aw_done       <= 1;
            end else begin
                s_axi_awready <= 0;
            end

            // W handshake
            if (s_axi_wvalid && !w_done) begin
                s_axi_wready <= 1;
                w_data       <= s_axi_wdata;
                w_done       <= 1;
            end else begin
                s_axi_wready <= 0;
            end

            // Execute write
            if (aw_done && w_done && !s_axi_bvalid) begin
                case (aw_addr[7:0])
                    REG_CONTROL[7:0]: begin
                        reg_enable <= w_data[0];
                    end
                    REG_DDR_BASE[7:0]: begin
                        reg_ddr_base    <= w_data;
                        base_configured <= 1;
                    end
                    default: ;
                endcase
                s_axi_bvalid <= 1;
                s_axi_bresp  <= 2'b00;
                aw_done      <= 0;
                w_done       <= 0;
            end

            // B handshake
            if (s_axi_bvalid && s_axi_bready) begin
                s_axi_bvalid <= 0;
            end
        end
    end

    // Read path
    reg        ar_done;
    reg [31:0] ar_addr;

    always @(posedge clk) begin
        if (rst) begin
            s_axi_arready <= 0;
            s_axi_rvalid  <= 0;
            s_axi_rresp   <= 0;
            s_axi_rdata   <= 0;
            ar_done       <= 0;
            ar_addr       <= 0;
        end else begin
            // AR handshake
            if (s_axi_arvalid && !ar_done) begin
                s_axi_arready <= 1;
                ar_addr       <= s_axi_araddr;
                ar_done       <= 1;
            end else begin
                s_axi_arready <= 0;
            end

            // R response
            if (ar_done && !s_axi_rvalid) begin
                case (ar_addr[7:0])
                    REG_CONTROL[7:0]:      s_axi_rdata <= {31'b0, reg_enable};
                    REG_STATUS[7:0]:       s_axi_rdata <= {overflow_count, 15'b0, dma_active};
                    REG_DDR_BASE[7:0]:     s_axi_rdata <= reg_ddr_base;
                    REG_WR_PTR[7:0]:       s_axi_rdata <= {7'b0, wr_ptr_r};
                    REG_SAMPLE_COUNT[7:0]: s_axi_rdata <= reg_sample_count;
                    REG_WRAP_COUNT[7:0]:   s_axi_rdata <= {16'b0, reg_wrap_count};
                    default:               s_axi_rdata <= 32'hDEADBEEF;
                endcase
                s_axi_rvalid <= 1;
                s_axi_rresp  <= 2'b00;
                ar_done      <= 0;
            end

            // R handshake complete
            if (s_axi_rvalid && s_axi_rready) begin
                s_axi_rvalid <= 0;
            end
        end
    end

endmodule
