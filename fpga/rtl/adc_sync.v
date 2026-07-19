// SPDX-License-Identifier: MIT
// fpga/rtl/adc_sync.v
//
// ADC data clock domain crossing: l_clk -> sys_clk.
//
// Crosses IQ samples + valid from the ADC clock domain (l_clk)
// to the system clock domain (sys_clk) using an async FIFO.
//
// The async FIFO is safe at any clock ratio. Previous versions used a
// toggle-based CDC that required sys_clk >= 3x adc_clk.

module adc_sync #(
    parameter SAMPLE_WIDTH = 12
) (
    // Source domain (l_clk)
    input  wire                            adc_clk,     // l_clk
    input  wire                            adc_rst,     // active high, adc_clk domain
    input  wire                            valid_in,
    input  wire signed [SAMPLE_WIDTH-1:0]  re_in,
    input  wire signed [SAMPLE_WIDTH-1:0]  im_in,

    // Destination domain (sys_clk)
    input  wire                            sys_clk,
    input  wire                            sys_rst,     // active high
    output reg                             valid_out,
    output reg  signed [SAMPLE_WIDTH-1:0]  re_out,
    output reg  signed [SAMPLE_WIDTH-1:0]  im_out
);

    // =========================================================
    // IQ data CDC via async FIFO (l_clk -> sys_clk)
    // =========================================================
    // FIFO data: {re[11:0], im[11:0]} = 24 bits
    // Depth 4 is sufficient: adc_clk <= sys_clk, so reads are
    // always faster than writes. The FIFO absorbs CDC latency.

    localparam FIFO_WIDTH = 2 * SAMPLE_WIDTH;  // 24

    wire                   fifo_full;
    wire                   fifo_empty;
    wire [FIFO_WIDTH-1:0]  fifo_rd_data;

    // Write: push on every valid_in (adc_clk domain)
    async_fifo #(
        .WIDTH(FIFO_WIDTH),
        .DEPTH(4)
    ) u_iq_fifo (
        .wr_clk(adc_clk),
        .wr_rst(adc_rst),
        .wr_en(valid_in & ~fifo_full),
        .wr_data({re_in, im_in}),
        .full(fifo_full),

        .rd_clk(sys_clk),
        .rd_rst(sys_rst),
        .rd_en(~fifo_empty),
        .rd_data(fifo_rd_data),
        .empty(fifo_empty)
    );

    // Read: pop whenever FIFO has data (sys_clk domain)
    // Pipeline register for output stability.
    always @(posedge sys_clk) begin
        if (sys_rst) begin
            valid_out <= 0;
            re_out    <= 0;
            im_out    <= 0;
        end else begin
            if (!fifo_empty) begin
                valid_out <= 1;
                re_out    <= $signed(fifo_rd_data[FIFO_WIDTH-1 -: SAMPLE_WIDTH]);
                im_out    <= $signed(fifo_rd_data[SAMPLE_WIDTH-1 : 0]);
            end else begin
                valid_out <= 0;
            end
        end
    end

endmodule
