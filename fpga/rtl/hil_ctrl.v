// SPDX-License-Identifier: MIT

`timescale 1ns/1ps
// fpga/rtl/hil_ctrl.v
//
// HIL (Hardware-in-the-Loop) test controller.
//
// Provides test infrastructure for golden vector verification:
//   1. AXI-Lite control registers (ARM configures test mode)
//   2. DDR read FSM via AXI3 master (plays IQ from DDR)
//   3. IQ mux (selects between live ADC and test playback)
//
// Register map (base 0x7C500000):
//   0x00  CONTROL    RW  [0]=test_mode, [1]=trigger (W1S)
//   0x04  STATUS     RO  [0]=playback_active, [1]=playback_done
//   0x08  DDR_BASE   RW  Physical DDR base of test waveform
//   0x0C  PLAY_COUNT RW  Number of IQ samples to play back
//   0x10  PLAY_PTR   RO  Current playback position (sample index)
//   0x14–0x38        --  Reserved for downstream extension
//   0x3C  ADC_CNT    RO  [31:0]=adc_valid pulse counter (live mode only)
//
// DDR packing (matches iq_dma_rx/tx):
//   32-bit word: {8'b0, im[11:0], re[11:0]}
//   64-bit beat: {word_odd, word_even}  (even at lower addr)
//
// Playback outputs one sample per clock with valid=1 while active.

module hil_ctrl #(
    parameter BURST_LEN = 16
) (
    (* X_INTERFACE_INFO = "xilinx.com:signal:clock:1.0 clk CLK" *)
    (* X_INTERFACE_PARAMETER = "ASSOCIATED_BUSIF s_axi:m_axi, ASSOCIATED_RESET rst" *)
    input  wire        clk,

    (* X_INTERFACE_INFO = "xilinx.com:signal:reset:1.0 rst RST" *)
    (* X_INTERFACE_PARAMETER = "POLARITY ACTIVE_HIGH" *)
    input  wire        rst,

    // Live ADC input (from adc_sync)
    input  wire        adc_valid,
    input  wire signed [11:0] adc_re,
    input  wire signed [11:0] adc_im,

    // Muxed output (to pipeline / snap probe)
    output wire        iq_valid,
    output wire signed [11:0] iq_re,
    output wire signed [11:0] iq_im,

    // Playback start pulse (for downstream module reset)
    output wire        playback_start,

    // Test mode status (for gating live trigger)
    output wire        test_mode_out,

    // AXI4-Lite slave
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
    output wire [1:0]  s_axi_bresp,
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
    output wire [1:0]  s_axi_rresp,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 s_axi RVALID" *)
    output reg         s_axi_rvalid,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 s_axi RREADY" *)
    input  wire        s_axi_rready,

    // AXI3 read master (DDR via HP1)
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 m_axi ARID" *)
    (* X_INTERFACE_PARAMETER = "PROTOCOL AXI3, FREQ_HZ 100000000, ADDR_WIDTH 32, DATA_WIDTH 64, READ_WRITE_MODE READ_ONLY, MAX_BURST_LENGTH 16" *)
    output wire [3:0]  m_axi_arid,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 m_axi ARADDR" *)
    output reg  [31:0] m_axi_araddr,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 m_axi ARLEN" *)
    output wire [3:0]  m_axi_arlen,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 m_axi ARSIZE" *)
    output wire [2:0]  m_axi_arsize,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 m_axi ARBURST" *)
    output wire [1:0]  m_axi_arburst,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 m_axi ARLOCK" *)
    output wire [1:0]  m_axi_arlock,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 m_axi ARCACHE" *)
    output wire [3:0]  m_axi_arcache,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 m_axi ARPROT" *)
    output wire [2:0]  m_axi_arprot,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 m_axi ARVALID" *)
    output reg         m_axi_arvalid,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 m_axi ARREADY" *)
    input  wire        m_axi_arready,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 m_axi RID" *)
    input  wire [3:0]  m_axi_rid,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 m_axi RDATA" *)
    input  wire [63:0] m_axi_rdata,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 m_axi RRESP" *)
    input  wire [1:0]  m_axi_rresp,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 m_axi RLAST" *)
    input  wire        m_axi_rlast,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 m_axi RVALID" *)
    input  wire        m_axi_rvalid,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 m_axi RREADY" *)
    output wire        m_axi_rready
);

    // AXI3 constants
    assign m_axi_arid    = 4'd0;
    assign m_axi_arlen   = BURST_LEN - 1;
    assign m_axi_arsize  = 3'b011;       // 8 bytes/beat
    assign m_axi_arburst = 2'b01;        // INCR
    assign m_axi_arlock  = 2'b00;
    assign m_axi_arcache = 4'b0011;
    assign m_axi_arprot  = 3'b000;
    assign m_axi_rready  = (state == S_DATA);  // Only accept in S_DATA

    assign s_axi_bresp = 2'b00;
    assign s_axi_rresp = 2'b00;

    // =========================================================
    // Control Registers
    // =========================================================
    reg        reg_test_mode;
    reg [31:0] reg_ddr_base;
    reg [31:0] reg_play_count;
    reg [31:0] play_ptr;
    reg        playback_active;
    reg        playback_done;
    reg        trigger_latch;    // set by write, cleared by FSM
    reg        playback_active_d; // delayed for edge detect

    assign playback_start = playback_active && !playback_active_d;

    always @(posedge clk) begin
        if (rst)
            playback_active_d <= 0;
        else
            playback_active_d <= playback_active;
    end

    // =========================================================
    // AXI-Lite Slave (simplified)
    // =========================================================
    reg aw_en;

    always @(posedge clk) begin
        if (rst) begin
            s_axi_awready  <= 0;
            s_axi_wready   <= 0;
            s_axi_bvalid   <= 0;
            s_axi_arready  <= 0;
            s_axi_rvalid   <= 0;
            s_axi_rdata    <= 0;
            aw_en           <= 1;
            reg_test_mode   <= 0;
            reg_ddr_base    <= 32'h10000000;
            reg_play_count  <= 0;
            trigger_latch   <= 0;
        end else begin
            // Write address + data (combined handshake)
            if (~s_axi_awready && s_axi_awvalid && s_axi_wvalid && aw_en) begin
                s_axi_awready <= 1;
                s_axi_wready  <= 1;
                aw_en <= 0;
            end else begin
                s_axi_awready <= 0;
                s_axi_wready  <= 0;
            end

            // Register writes
            if (s_axi_awready && s_axi_wready) begin
                case (s_axi_awaddr[5:2])
                    4'd0: begin
                        reg_test_mode <= s_axi_wdata[0];
                        if (s_axi_wdata[1]) trigger_latch <= 1;
                    end
                    4'd2: reg_ddr_base   <= s_axi_wdata;
                    4'd3: reg_play_count <= s_axi_wdata;
                    default: ;
                endcase
            end

            // Write response
            if (s_axi_awready && s_axi_wready && ~s_axi_bvalid)
                s_axi_bvalid <= 1;
            else if (s_axi_bready && s_axi_bvalid) begin
                s_axi_bvalid <= 0;
                aw_en <= 1;
            end

            // Read handshake
            if (~s_axi_arready && s_axi_arvalid)
                s_axi_arready <= 1;
            else
                s_axi_arready <= 0;

            // Read data
            if (s_axi_arready && s_axi_arvalid && ~s_axi_rvalid) begin
                s_axi_rvalid <= 1;
                case (s_axi_araddr[5:2])
                    4'd0:  s_axi_rdata <= {30'd0, trigger_latch, reg_test_mode};
                    4'd1:  s_axi_rdata <= {30'd0, playback_done, playback_active};
                    4'd2:  s_axi_rdata <= reg_ddr_base;
                    4'd3:  s_axi_rdata <= reg_play_count;
                    4'd4:  s_axi_rdata <= play_ptr;
                    4'd15: s_axi_rdata <= adc_valid_cnt;
                    default: s_axi_rdata <= 32'd0;
                endcase
            end else if (s_axi_rvalid && s_axi_rready)
                s_axi_rvalid <= 0;

            // Clear trigger once FSM starts
            if (playback_active)
                trigger_latch <= 0;
        end
    end

    // =========================================================
    // ADC Valid Counter (diagnostic — proves adc_valid reaches hil_ctrl)
    // =========================================================
    reg [31:0] adc_valid_cnt;

    always @(posedge clk) begin
        if (rst)
            adc_valid_cnt <= 0;
        else if (adc_valid && !reg_test_mode)
            adc_valid_cnt <= adc_valid_cnt + 1;
    end

    // =========================================================
    // DDR Read FSM
    // =========================================================
    // Each 64-bit beat = 2 IQ samples. Burst of 16 = 32 samples.
    // Unpack into a 64-deep sample FIFO (distributed RAM).

    localparam S_IDLE = 3'd0;
    localparam S_AR   = 3'd1;   // Assert read address
    localparam S_DATA = 3'd2;   // Receive beat, write even sample
    localparam S_ODD  = 3'd3;   // Write odd sample from latched beat
    localparam S_DONE = 3'd4;   // Wait for FIFO drain

    reg [2:0]  state;
    reg [31:0] ddr_addr;
    reg [31:0] samples_left;    // samples still to fetch from DDR
    reg [3:0]  beat_cnt;
    reg [31:0] beat_odd_word;   // latched upper 32 bits of 64-bit beat
    reg        beat_was_last;   // was last beat of burst?
    reg        fifo_reset;      // pulse: reset read pointer on new playback

    // FIFO (BRAM, 256×32)
    (* ram_style = "block" *) reg [31:0] fifo [0:255];
    reg [8:0]  wr_ptr, rd_ptr;
    wire [7:0] wr_addr = wr_ptr[7:0];
    wire [7:0] rd_addr = rd_ptr[7:0];
    wire empty = (wr_ptr == rd_ptr);
    wire full  = (wr_ptr[8] != rd_ptr[8]) && (wr_ptr[7:0] == rd_ptr[7:0]);

    wire has_burst_space = ((wr_ptr - rd_ptr) <= 9'd224);  // room for 32 more

    always @(posedge clk) begin
        if (rst) begin
            state           <= S_IDLE;
            m_axi_arvalid   <= 0;
            m_axi_araddr    <= 0;
            wr_ptr          <= 0;
            playback_active <= 0;
            playback_done   <= 0;
            samples_left    <= 0;
            ddr_addr        <= 0;
            beat_cnt        <= 0;
            beat_odd_word   <= 0;
            beat_was_last   <= 0;
            fifo_reset      <= 0;
        end else begin
            fifo_reset <= 0;  // default: clear pulse
            case (state)
                S_IDLE: begin
                    if (trigger_latch && reg_play_count > 0) begin
                        ddr_addr        <= reg_ddr_base;
                        samples_left    <= reg_play_count;
                        wr_ptr          <= 0;
                        fifo_reset      <= 1;
                        playback_active <= 1;
                        playback_done   <= 0;
                        state           <= S_AR;
                    end
                end

                S_AR: begin
                    if (samples_left == 0) begin
                        state <= S_DONE;
                    end else if (has_burst_space) begin
                        m_axi_araddr  <= ddr_addr;
                        m_axi_arvalid <= 1;
                        if (m_axi_arvalid && m_axi_arready) begin
                            m_axi_arvalid <= 0;
                            beat_cnt      <= 0;
                            state         <= S_DATA;
                        end
                    end
                end

                S_DATA: begin
                    m_axi_arvalid <= 0;
                    if (m_axi_rvalid) begin
                        // Write even sample (lower 32 bits)
                        if (samples_left > 0) begin
                            fifo[wr_addr] <= m_axi_rdata[31:0];
                            wr_ptr <= wr_ptr + 1;
                            samples_left <= samples_left - 1;
                        end
                        // Latch odd sample (upper 32 bits)
                        beat_odd_word <= m_axi_rdata[63:32];
                        beat_was_last <= m_axi_rlast;
                        beat_cnt      <= beat_cnt + 1;
                        state         <= S_ODD;
                    end
                end

                S_ODD: begin
                    // Write odd sample from latched upper word
                    if (samples_left > 0) begin
                        fifo[wr_addr] <= beat_odd_word;
                        wr_ptr <= wr_ptr + 1;
                        samples_left <= samples_left - 1;
                    end
                    if (beat_was_last) begin
                        ddr_addr <= ddr_addr + (BURST_LEN * 8);
                        state    <= S_AR;
                    end else begin
                        state <= S_DATA;
                    end
                end

                S_DONE: begin
                    if (empty) begin
                        playback_active <= 0;
                        playback_done   <= 1;
                        state           <= S_IDLE;
                    end
                end

                default: state <= S_IDLE;
            endcase
        end
    end

    // =========================================================
    // FIFO Read — one sample per 5 clocks (matches live ADC rate)
    // =========================================================
    // In live mode, ADC delivers 20 MSPS into 100 MHz fabric = 1 valid per 5
    // clocks. Test mode must match this rate so that ALL downstream modules
    // (stf_detect BRAM timing, ltf_correlator 4-stage pipeline, rx_top
    // windowed-max) operate identically to live mode. Without this gating,
    // the correlator never completes its 4-stage accumulation and the
    // windowed-max path is broken for burst frame 2+.
    reg        test_valid;
    reg [11:0] test_re;
    reg [11:0] test_im;
    reg [2:0]  test_div;   // modulo-5 divider (0-4)

    always @(posedge clk) begin
        if (rst) begin
            test_valid <= 0;
            test_re    <= 0;
            test_im    <= 0;
            rd_ptr     <= 0;
            play_ptr   <= 0;
            test_div   <= 0;
        end else if (fifo_reset) begin
            test_valid <= 0;
            rd_ptr     <= 0;
            play_ptr   <= 0;
            test_div   <= 0;
        end else begin
            if (!empty && playback_active && test_div == 0) begin
                test_valid <= 1;
                test_re    <= fifo[rd_addr][11:0];
                test_im    <= fifo[rd_addr][23:12];
                rd_ptr     <= rd_ptr + 1;
                play_ptr   <= play_ptr + 1;
            end else begin
                test_valid <= 0;
            end
            // Advance divider (free-running while playback active)
            if (playback_active) begin
                test_div <= (test_div == 3'd4) ? 3'd0 : test_div + 1;
            end else begin
                test_div <= 0;
            end
        end
    end



    // =========================================================
    // IQ Mux
    // =========================================================
    assign iq_valid = reg_test_mode ? test_valid : adc_valid;
    assign iq_re    = reg_test_mode ? $signed(test_re) : adc_re;
    assign iq_im    = reg_test_mode ? $signed(test_im) : adc_im;

    // Expose test_mode for trigger gating
    assign test_mode_out = reg_test_mode;

endmodule
