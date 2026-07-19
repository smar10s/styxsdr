// SPDX-License-Identifier: MIT

`timescale 1ns/1ps
// fpga/rtl/iq_dma_rx.v
//
// Continuous IQ DMA writer: streams ADC samples into DDR via AXI3 HP0
// in a circular buffer.  No triggering, no gating, no slots — just a
// continuous write pointer that wraps.
//
// IQ samples arrive from adc_sync at ~20 MSPS into a 100 MHz fabric
// clock.  Accumulate 32 samples (16 pairs) in a BRAM-backed buffer,
// then burst-write to DDR as 16 × 64-bit beats.
//
// Double-buffered via a single 32×64-bit BRAM addressed as two halves
// (buf_sel=0 → entries 0..15, buf_sel=1 → entries 16..31).  While
// one half is being burst-written to DDR, the other accepts incoming
// samples.  At the real 5:1 clock ratio this never overflows; overflow
// is counted for diagnostics if AXI stalls badly.
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
    parameter BURST_LEN       = 16        // AXI3 max burst (16 beats of 64 bits)
) (
    (* X_INTERFACE_INFO = "xilinx.com:signal:clock:1.0 clk CLK" *)
    (* X_INTERFACE_PARAMETER = "ASSOCIATED_BUSIF m_axi:s_axi, ASSOCIATED_RESET rst" *)
    input  wire        clk,

    (* X_INTERFACE_INFO = "xilinx.com:signal:reset:1.0 rst RST" *)
    (* X_INTERFACE_PARAMETER = "POLARITY ACTIVE_HIGH" *)
    input  wire        rst,

    // IQ input (from adc_sync, sys_cpu_clk domain, ~20 MSPS into 100 MHz fabric)
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
    (* X_INTERFACE_PARAMETER = "PROTOCOL AXI3, FREQ_HZ 100000000, ADDR_WIDTH 32, DATA_WIDTH 64, HAS_BURST 1, HAS_LOCK 0, HAS_PROT 0, HAS_CACHE 0, HAS_QOS 0, HAS_REGION 0, HAS_WSTRB 1, MAX_BURST_LENGTH 16, NUM_WRITE_OUTSTANDING 2, NUM_READ_OUTSTANDING 0, SUPPORTS_NARROW_BURST 0, READ_WRITE_MODE WRITE_ONLY" *)
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
    output reg  [63:0] m_axi_wdata,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 m_axi WSTRB" *)
    output wire [7:0]  m_axi_wstrb,     // 8'hFF
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 m_axi WID" *)
    output wire [5:0]  m_axi_wid,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 m_axi WLAST" *)
    output reg         m_axi_wlast,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 m_axi WVALID" *)
    output reg         m_axi_wvalid,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 m_axi WREADY" *)
    input  wire        m_axi_wready,

    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 m_axi BRESP" *)
    input  wire [1:0]  m_axi_bresp,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 m_axi BVALID" *)
    input  wire        m_axi_bvalid,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 m_axi BREADY" *)
    output reg         m_axi_bready,

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
    assign m_axi_awlen   = BURST_LEN - 1;          // 15 = 16 beats
    assign m_axi_awsize  = 3'b011;                  // 8 bytes per beat
    assign m_axi_awburst = 2'b01;                   // INCR
    assign m_axi_wstrb   = 8'hFF;                   // all bytes valid
    assign m_axi_awid    = 6'b0;
    assign m_axi_wid     = 6'b0;

    // =========================================================
    // Derived parameters
    // =========================================================
    localparam SAMPLES_PER_BURST = BURST_LEN * 2;   // 32 samples per burst

    // Verify DDR buffer is a whole number of bursts (wrap alignment)
    initial begin
        if (DDR_BUF_SAMPLES % SAMPLES_PER_BURST != 0) begin
            $error("DDR_BUF_SAMPLES (%0d) must be a multiple of SAMPLES_PER_BURST (%0d)",
                   DDR_BUF_SAMPLES, SAMPLES_PER_BURST);
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
    // Double-buffered burst buffer — inferred dual-port BRAM
    //
    // 32 × 64-bit entries: buf_sel=0 fills entries [0..15],
    // buf_sel=1 fills entries [16..31].  Burst reads from the
    // opposite half.  Each 64-bit entry holds a packed pair:
    //   {8'b0, im_odd, re_odd, 8'b0, im_even, re_even}
    //
    // This replaces 2 × 32 × 24-bit register files that each
    // needed a unique write-enable, consuming ~41 control sets.
    // BRAM uses zero additional control sets.
    // =========================================================
    (* ram_style = "block" *)
    reg [63:0] burst_mem [0:2*BURST_LEN-1];

    reg        buf_sel;       // 0 = filling low half, 1 = filling high half
    reg [5:0]  fill_count;    // 0..32 samples (6 bits for value 32)
    reg [23:0] pending_even;  // latched even sample waiting for its odd pair
    reg        has_even;      // 1 = pending_even is valid
    reg [3:0]  pair_count;    // 0..15, index within current half

    // BRAM write port (fill side)
    reg        mem_wr_en;
    reg [4:0]  mem_wr_addr;   // {buf_sel, pair_count[3:0]}
    reg [63:0] mem_wr_data;

    // BRAM read port (burst side) — 1-cycle latency
    reg [4:0]  mem_rd_addr;
    reg [63:0] mem_rd_data;

    // Port A: synchronous write
    always @(posedge clk) begin
        if (mem_wr_en)
            burst_mem[mem_wr_addr] <= mem_wr_data;
    end

    // Port B: synchronous read (1-cycle latency)
    always @(posedge clk) begin
        mem_rd_data <= burst_mem[mem_rd_addr];
    end

    // =========================================================
    // DMA FSM states
    // =========================================================
    localparam S_IDLE  = 3'd0;  // waiting for a full buffer
    localparam S_ADDR  = 3'd1;  // AW phase
    localparam S_PIPE  = 3'd2;  // 1-cycle pipeline wait for first beat
    localparam S_WRITE = 3'd3;  // W beat: wdata valid, waiting for wready
    localparam S_NEXT  = 3'd4;  // 1-cycle gap: load next beat from BRAM
    localparam S_RESP  = 3'd5;  // B response

    reg [2:0]  state;
    reg [3:0]  beat_cnt;      // 0..15 within burst
    reg        burst_sel;     // which half the burst is reading from
    reg [24:0] burst_wr_ptr;  // wr_ptr snapshot for this burst
    reg        burst_pending; // a filled buffer is waiting for burst

    reg [3:0]  rd_beat;       // next beat index to read from BRAM

    // =========================================================
    // Fill logic — runs independently, always accepts samples
    //
    // Accumulates even/odd pairs and writes packed 64-bit words
    // to BRAM.  fill_count tracks individual samples (0..32),
    // pair_count tracks 64-bit words written (0..15).
    // =========================================================
    always @(posedge clk) begin
        if (rst) begin
            fill_count       <= 0;
            pair_count       <= 0;
            buf_sel          <= 0;
            burst_pending    <= 0;
            has_even         <= 0;
            pending_even     <= 0;
            reg_sample_count <= 0;
            reg_wrap_count   <= 0;
            overflow_count   <= 0;
            mem_wr_en        <= 0;
            mem_wr_addr      <= 0;
            mem_wr_data      <= 0;
        end else begin
            mem_wr_en <= 0;  // default: no write

            // Clear burst_pending when burst FSM picks it up
            if (burst_pending_clear)
                burst_pending <= 0;

            if (reg_enable && valid_in && base_configured) begin
                if (fill_count < SAMPLES_PER_BURST) begin
                    // Normal fill into current buffer half
                    if (!has_even) begin
                        // First sample of pair (even) — latch it
                        pending_even <= {im_in, re_in};
                        has_even     <= 1;
                    end else begin
                        // Second sample of pair (odd) — write packed word
                        mem_wr_en   <= 1;
                        mem_wr_addr <= {buf_sel, pair_count};
                        mem_wr_data <= {
                            {8'b0, im_in, re_in},            // odd: high 32
                            {8'b0, pending_even}              // even: low 32
                        };
                        has_even    <= 0;
                        pair_count  <= pair_count + 1;
                    end

                    fill_count       <= fill_count + 1;
                    reg_sample_count <= reg_sample_count + 1;
                    if (reg_sample_count == 32'hFFFFFFFF)
                        reg_wrap_count <= reg_wrap_count + 1;

                    // Buffer just became full? (fill_count is pre-increment value)
                    if (fill_count == SAMPLES_PER_BURST - 1) begin
                        if (!burst_pending) begin
                            // Other half is free — swap and continue
                            burst_pending <= 1;
                            buf_sel       <= ~buf_sel;
                            fill_count    <= 0;
                            pair_count    <= 0;
                            has_even      <= 0;
                        end else begin
                            // Other half still being burst-written.
                            // Stuck — subsequent samples overflow.
                            // fill_count stays at 32.
                        end
                    end
                end else begin
                    // fill_count == SAMPLES_PER_BURST and couldn't swap.
                    // Check if burst_pending was cleared (burst started).
                    if (!burst_pending) begin
                        // Safe to swap now.  Accept this sample.
                        burst_pending <= 1;
                        buf_sel       <= ~buf_sel;
                        fill_count    <= 1;
                        pair_count    <= 0;
                        has_even      <= 1;
                        pending_even  <= {im_in, re_in};
                        reg_sample_count <= reg_sample_count + 1;
                        if (reg_sample_count == 32'hFFFFFFFF)
                            reg_wrap_count <= reg_wrap_count + 1;
                    end else begin
                        // True overflow — both halves occupied
                        overflow_count <= overflow_count + 1;
                    end
                end
            end
        end
    end

    // =========================================================
    // Burst FSM — reads packed words from the non-fill BRAM half
    //
    // BRAM has 1-cycle read latency.  To guarantee correct data
    // regardless of wready timing, the FSM inserts a 1-cycle gap
    // (S_NEXT) between each AXI W beat to allow the BRAM read to
    // settle.  This trades throughput (2 clocks/beat instead of 1)
    // for correctness.  At 20 MSPS into 100 MHz, a 32-sample burst
    // takes 32 clocks (16 beats × 2 clocks/beat).  Samples arrive
    // every 5 clocks, giving 160 clocks per burst — 5× margin.
    //
    // Schedule:
    //   S_IDLE  → addr = beat 0
    //   S_ADDR  → AW handshake; addr = beat 1
    //   S_PIPE  → wdata = mem_rd_data (beat 0); wvalid = 1
    //             addr = beat 2  → S_WRITE
    //   S_WRITE → wait for wready.  On handshake, if last beat
    //             → S_RESP.  Else → S_NEXT.
    //   S_NEXT  → wdata = mem_rd_data (next beat); wvalid = 1
    //             addr = beat N+2  → S_WRITE
    //   S_RESP  → wait for bresp, advance wr_ptr → S_IDLE
    // =========================================================
    reg burst_pending_clear;

    always @(posedge clk) begin
        if (rst) begin
            state            <= S_IDLE;
            beat_cnt         <= 0;
            rd_beat          <= 0;
            wr_ptr_r         <= 0;
            burst_wr_ptr     <= 0;
            burst_sel        <= 0;
            m_axi_awaddr     <= 0;
            m_axi_awvalid    <= 0;
            m_axi_wdata      <= 0;
            m_axi_wvalid     <= 0;
            m_axi_wlast      <= 0;
            m_axi_bready     <= 0;
            burst_pending_clear <= 0;
            mem_rd_addr      <= 0;
        end else begin
            burst_pending_clear <= 0;

            case (state)
                // -------------------------------------------------
                S_IDLE: begin
                    if (burst_pending && base_configured) begin
                        burst_sel     <= ~buf_sel;
                        burst_wr_ptr  <= wr_ptr_r;
                        m_axi_awaddr  <= reg_ddr_base + {wr_ptr_r, 2'b00};
                        m_axi_awvalid <= 1;
                        beat_cnt      <= 0;
                        // Start BRAM read for beat 0
                        mem_rd_addr   <= {~buf_sel, 4'd0};
                        rd_beat       <= 1;
                        burst_pending_clear <= 1;
                        state         <= S_ADDR;
                    end
                end

                // -------------------------------------------------
                S_ADDR: begin
                    if (m_axi_awready && m_axi_awvalid) begin
                        m_axi_awvalid <= 0;
                        // Beat 0 addr was set in S_IDLE (≥1 cycle ago).
                        // Start read for beat 1; by S_PIPE, mem_rd_data
                        // will have beat 0 (1 cycle after S_IDLE's addr).
                        mem_rd_addr <= {burst_sel, rd_beat[3:0]};
                        rd_beat     <= rd_beat + 1;
                        state       <= S_PIPE;
                    end
                end

                // -------------------------------------------------
                S_PIPE: begin
                    // mem_rd_data has beat 0 (addr set in S_IDLE,
                    // ≥2 cycles ago).  Load it into wdata directly.
                    m_axi_wdata  <= mem_rd_data;
                    m_axi_wvalid <= 1;
                    m_axi_wlast  <= (BURST_LEN == 1) ? 1'b1 : 1'b0;
                    // Don't advance addr — beat 1's addr (from S_ADDR)
                    // needs 1 more cycle to produce data.
                    state        <= S_WRITE;
                end

                // -------------------------------------------------
                S_WRITE: begin
                    if (m_axi_wready && m_axi_wvalid) begin
                        beat_cnt <= beat_cnt + 1;
                        m_axi_wvalid <= 0;  // deassert until S_NEXT loads next beat

                        if (beat_cnt == BURST_LEN - 1) begin
                            // Last beat accepted
                            m_axi_wlast  <= 0;
                            m_axi_bready <= 1;
                            state        <= S_RESP;
                        end else begin
                            // Next beat's data is in mem_rd_data
                            // (addr was set 2 cycles ago: in S_PIPE or
                            // previous S_NEXT).  Go to S_NEXT to load it.
                            state <= S_NEXT;
                        end
                    end
                end

                // -------------------------------------------------
                S_NEXT: begin
                    // mem_rd_data has the next beat: its addr was
                    // set ≥2 cycles ago (in S_ADDR for beat 1, or
                    // in the previous S_NEXT for subsequent beats).
                    m_axi_wdata  <= mem_rd_data;
                    m_axi_wvalid <= 1;
                    m_axi_wlast  <= (beat_cnt == BURST_LEN - 1) ? 1'b1 : 1'b0;
                    // Start read for the beat after next
                    mem_rd_addr  <= {burst_sel, rd_beat[3:0]};
                    rd_beat      <= rd_beat + 1;
                    state        <= S_WRITE;
                end

                // -------------------------------------------------
                S_RESP: begin
                    if (m_axi_bvalid && m_axi_bready) begin
                        m_axi_bready <= 0;
                        if (wr_ptr_r + SAMPLES_PER_BURST >= DDR_BUF_SAMPLES)
                            wr_ptr_r <= 0;
                        else
                            wr_ptr_r <= wr_ptr_r + SAMPLES_PER_BURST;
                        state <= S_IDLE;
                    end
                end

                default: state <= S_IDLE;
            endcase
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
                    REG_STATUS[7:0]:       s_axi_rdata <= {overflow_count, 15'b0, (state != S_IDLE)};
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
