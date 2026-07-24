// SPDX-License-Identifier: MIT
// Co-simulation wrapper: instantiates iq_dma_tx and iq_dma_rx with
// TX DAC output wired to RX ADC input, sharing an AXI3 mock DDR.
//
// Parameters:
//   TX_CLK_SAME_AS_RX = 1 (default): single-clock mode, TX uses `clk`
//   TX_CLK_SAME_AS_RX = 0: TX uses `tx_clk` (async FIFO insertion point)
//   VALID_RATIO = 1: when TX_CLK_SAME_AS_RX=1, forward 1-in-N TX
//     valid_out cycles to RX valid_in.  Set to 5 to model 20 MSPS TX
//     into 100 MHz RX (hardware l_clk / sys_cpu_clk ratio).

module tb_iq_dma_loopback #(
    parameter TX_CLK_SAME_AS_RX = 1,
    parameter integer VALID_RATIO = 1
) (
    input  wire        clk,                // system clock (RX domain, 100 MHz)
    input  wire        tx_clk,             // TX clock (20 MHz, unused when same-as-RX)
    input  wire        rst,

    // --- TX AXI-Lite (s_axi_tx_ prefix) ---
    input  wire        s_axi_tx_aclk,
    input  wire        s_axi_tx_aresetn,
    input  wire [31:0] s_axi_tx_awaddr,
    input  wire        s_axi_tx_awvalid,
    output wire        s_axi_tx_awready,
    input  wire [31:0] s_axi_tx_wdata,
    input  wire [3:0]  s_axi_tx_wstrb,
    input  wire        s_axi_tx_wvalid,
    output wire        s_axi_tx_wready,
    output wire [1:0]  s_axi_tx_bresp,
    output wire        s_axi_tx_bvalid,
    input  wire        s_axi_tx_bready,
    input  wire [31:0] s_axi_tx_araddr,
    input  wire        s_axi_tx_arvalid,
    output wire        s_axi_tx_arready,
    output wire [31:0] s_axi_tx_rdata,
    output wire [1:0]  s_axi_tx_rresp,
    output wire        s_axi_tx_rvalid,
    input  wire        s_axi_tx_rready,

    // --- RX AXI-Lite (s_axi_rx_ prefix) ---
    input  wire [31:0] s_axi_rx_awaddr,
    input  wire        s_axi_rx_awvalid,
    output wire        s_axi_rx_awready,
    input  wire [31:0] s_axi_rx_wdata,
    input  wire [3:0]  s_axi_rx_wstrb,
    input  wire        s_axi_rx_wvalid,
    output wire        s_axi_rx_wready,
    output wire [1:0]  s_axi_rx_bresp,
    output wire        s_axi_rx_bvalid,
    input  wire        s_axi_rx_bready,
    input  wire [31:0] s_axi_rx_araddr,
    input  wire        s_axi_rx_arvalid,
    output wire        s_axi_rx_arready,
    output wire [31:0] s_axi_rx_rdata,
    output wire [1:0]  s_axi_rx_rresp,
    output wire        s_axi_rx_rvalid,
    input  wire        s_axi_rx_rready,

    // --- AXI3: TX read master (m_axi_tx_ prefix) ---
    output wire [31:0] m_axi_tx_araddr,
    output wire [3:0]  m_axi_tx_arlen,
    output wire [2:0]  m_axi_tx_arsize,
    output wire [1:0]  m_axi_tx_arburst,
    output wire        m_axi_tx_arvalid,
    input  wire        m_axi_tx_arready,
    input  wire [63:0] m_axi_tx_rdata,
    input  wire [1:0]  m_axi_tx_rresp,
    input  wire        m_axi_tx_rlast,
    input  wire        m_axi_tx_rvalid,
    output wire        m_axi_tx_rready,

    // --- AXI3: RX write master (m_axi_rx_ prefix) ---
    output wire [31:0] m_axi_rx_awaddr,
    output wire [3:0]  m_axi_rx_awlen,
    output wire [2:0]  m_axi_rx_awsize,
    output wire [1:0]  m_axi_rx_awburst,
    output wire        m_axi_rx_awvalid,
    input  wire        m_axi_rx_awready,
    output wire [5:0]  m_axi_rx_awid,
    output wire [63:0] m_axi_rx_wdata,
    output wire [7:0]  m_axi_rx_wstrb,
    output wire [5:0]  m_axi_rx_wid,
    output wire        m_axi_rx_wlast,
    output wire        m_axi_rx_wvalid,
    input  wire        m_axi_rx_wready,
    input  wire [1:0]  m_axi_rx_bresp,
    input  wire        m_axi_rx_bvalid,
    output wire        m_axi_rx_bready,
    input  wire [5:0]  m_axi_rx_bid,

    // --- Monitoring ---
    output wire [24:0] rx_sample_cnt,
    output wire [24:0] rx_wr_ptr,
    output wire [15:0] tx_re_out,
    output wire [15:0] tx_im_out,
    output wire        tx_valid_out
);

    wire tx_dac_clk;
    assign tx_dac_clk = TX_CLK_SAME_AS_RX ? clk : tx_clk;

    // TX DAC output → RX ADC input loopback (truncate 16→12 bit)
    wire [15:0] loop_re, loop_im;
    wire        loop_valid;

    // Sample-rate decimator: forward 1-in-VALID_RATIO valid cycles to
    // model the 5:1 clock ratio between l_clk (20 MHz) and sys_cpu_clk
    // (100 MHz) on hardware.
    wire        rx_valid;
    wire [11:0] rx_re;
    wire [11:0] rx_im;

    generate
        if (VALID_RATIO == 1) begin : g_direct
            assign rx_valid = loop_valid;
            assign rx_re = loop_re[15:4];
            assign rx_im = loop_im[15:4];
        end else begin : g_decimate
            reg [$clog2(VALID_RATIO)-1:0] ratio_cnt;
            reg         rx_valid_r;
            reg [11:0]  rx_re_r, rx_im_r;

            always @(posedge clk) begin
                if (rst) begin
                    ratio_cnt <= 0;
                    rx_valid_r <= 0;
                end else begin
                    rx_valid_r <= 0;
                    if (loop_valid) begin
                        if (ratio_cnt == 0) begin
                            rx_valid_r <= 1;
                            rx_re_r <= loop_re[15:4];
                            rx_im_r <= loop_im[15:4];
                        end
                        ratio_cnt <= (ratio_cnt == VALID_RATIO - 1)
                                     ? 0 : ratio_cnt + 1;
                    end else begin
                        ratio_cnt <= 0;
                    end
                end
            end

            assign rx_valid = rx_valid_r;
            assign rx_re = rx_re_r;
            assign rx_im = rx_im_r;
        end
    endgenerate

    wire [24:0] rx_sc;
    wire [24:0] rx_wp;

    assign rx_sample_cnt = rx_sc;
    assign rx_wr_ptr    = rx_wp;
    assign tx_re_out    = loop_re;
    assign tx_im_out    = loop_im;
    assign tx_valid_out = loop_valid;

    iq_dma_tx #(
        .BUF_SIZE_SAMPLES(1024)
    ) tx_inst (
        .clk              (tx_dac_clk),
        .rst              (rst),
        .re_out           (loop_re),
        .im_out           (loop_im),
        .valid_out        (loop_valid),
        .dac_valid        (1'b1),
        .m_axi_araddr     (m_axi_tx_araddr),
        .m_axi_arlen      (m_axi_tx_arlen),
        .m_axi_arsize     (m_axi_tx_arsize),
        .m_axi_arburst    (m_axi_tx_arburst),
        .m_axi_arvalid    (m_axi_tx_arvalid),
        .m_axi_arready    (m_axi_tx_arready),
        .m_axi_rdata      (m_axi_tx_rdata),
        .m_axi_rresp      (m_axi_tx_rresp),
        .m_axi_rlast      (m_axi_tx_rlast),
        .m_axi_rvalid     (m_axi_tx_rvalid),
        .m_axi_rready     (m_axi_tx_rready),
        .s_axi_aclk       (s_axi_tx_aclk),
        .s_axi_aresetn    (s_axi_tx_aresetn),
        .s_axi_awaddr     (s_axi_tx_awaddr),
        .s_axi_awvalid    (s_axi_tx_awvalid),
        .s_axi_awready    (s_axi_tx_awready),
        .s_axi_wdata      (s_axi_tx_wdata),
        .s_axi_wstrb      (s_axi_tx_wstrb),
        .s_axi_wvalid     (s_axi_tx_wvalid),
        .s_axi_wready     (s_axi_tx_wready),
        .s_axi_bresp      (s_axi_tx_bresp),
        .s_axi_bvalid     (s_axi_tx_bvalid),
        .s_axi_bready     (s_axi_tx_bready),
        .s_axi_araddr     (s_axi_tx_araddr),
        .s_axi_arvalid    (s_axi_tx_arvalid),
        .s_axi_arready    (s_axi_tx_arready),
        .s_axi_rdata      (s_axi_tx_rdata),
        .s_axi_rresp      (s_axi_tx_rresp),
        .s_axi_rvalid     (s_axi_tx_rvalid),
        .s_axi_rready     (s_axi_tx_rready)
    );

    iq_dma_rx #(
        .DDR_BUF_SAMPLES(1024)
    ) rx_inst (
        .clk              (clk),
        .rst              (rst),
        .re_in            (rx_re),
        .im_in            (rx_im),
        .valid_in         (rx_valid),
        .ddr_wr_ptr_out   (rx_wp),
        .sample_cnt_out   (rx_sc),
        .m_axi_awaddr     (m_axi_rx_awaddr),
        .m_axi_awlen      (m_axi_rx_awlen),
        .m_axi_awsize     (m_axi_rx_awsize),
        .m_axi_awburst    (m_axi_rx_awburst),
        .m_axi_awvalid    (m_axi_rx_awvalid),
        .m_axi_awready    (m_axi_rx_awready),
        .m_axi_awid       (m_axi_rx_awid),
        .m_axi_wdata      (m_axi_rx_wdata),
        .m_axi_wstrb      (m_axi_rx_wstrb),
        .m_axi_wid        (m_axi_rx_wid),
        .m_axi_wlast      (m_axi_rx_wlast),
        .m_axi_wvalid     (m_axi_rx_wvalid),
        .m_axi_wready     (m_axi_rx_wready),
        .m_axi_bresp      (m_axi_rx_bresp),
        .m_axi_bvalid     (m_axi_rx_bvalid),
        .m_axi_bready     (m_axi_rx_bready),
        .m_axi_bid        (m_axi_rx_bid),
        .s_axi_awaddr     (s_axi_rx_awaddr),
        .s_axi_awvalid    (s_axi_rx_awvalid),
        .s_axi_awready    (s_axi_rx_awready),
        .s_axi_wdata      (s_axi_rx_wdata),
        .s_axi_wstrb      (s_axi_rx_wstrb),
        .s_axi_wvalid     (s_axi_rx_wvalid),
        .s_axi_wready     (s_axi_rx_wready),
        .s_axi_bresp      (s_axi_rx_bresp),
        .s_axi_bvalid     (s_axi_rx_bvalid),
        .s_axi_bready     (s_axi_rx_bready),
        .s_axi_araddr     (s_axi_rx_araddr),
        .s_axi_arvalid    (s_axi_rx_arvalid),
        .s_axi_arready    (s_axi_rx_arready),
        .s_axi_rdata      (s_axi_rx_rdata),
        .s_axi_rresp      (s_axi_rx_rresp),
        .s_axi_rvalid     (s_axi_rx_rvalid),
        .s_axi_rready     (s_axi_rx_rready)
    );

endmodule
