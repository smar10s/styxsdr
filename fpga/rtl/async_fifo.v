// SPDX-License-Identifier: MIT
// fpga/rtl/async_fifo.v
//
// Gray-code pointer asynchronous FIFO for clock domain crossing.
// Uses distributed RAM (LUT-based) — no BRAM.
//
// Write port: wr_clk domain. Assert wr_en when data is valid.
// Read port:  rd_clk domain. Assert rd_en to pop. Check !empty first.
//
// Resets are synchronous to each clock domain. Registers use initial
// values (honored by Xilinx FPGA and Icarus simulation). The write
// side can safely have wr_rst tied low if no reset is available in
// that clock domain.
//
// Parameters:
//   WIDTH: data width in bits
//   DEPTH: number of entries (must be power of 2)

module async_fifo #(
    parameter WIDTH = 25,
    parameter DEPTH = 4
) (
    // Write port
    input  wire              wr_clk,
    input  wire              wr_rst,    // synchronous reset, wr_clk domain
    input  wire              wr_en,
    input  wire [WIDTH-1:0]  wr_data,
    output wire              full,

    // Read port
    input  wire              rd_clk,
    input  wire              rd_rst,    // synchronous reset, rd_clk domain
    input  wire              rd_en,
    output wire [WIDTH-1:0]  rd_data,
    output wire              empty
);

    localparam PTR_W = (DEPTH <= 2) ? 2 :
                       (DEPTH <= 4) ? 3 :
                       (DEPTH <= 8) ? 4 :
                       (DEPTH <= 16) ? 5 :
                       (DEPTH <= 32) ? 6 :
                       (DEPTH <= 64) ? 7 : 8;  // avoids $clog2 Icarus VPI issues

    // --- Memory ---
    reg [WIDTH-1:0] mem [0:DEPTH-1];

    // --- Write pointer (wr_clk domain, binary + gray) ---
    reg [PTR_W-1:0] wr_bin;
    wire [PTR_W-1:0] wr_bin_next = wr_bin + (wr_en & ~full);
    wire [PTR_W-1:0] wr_gray_next = wr_bin_next ^ (wr_bin_next >> 1);
    reg [PTR_W-1:0] wr_gray;

    initial begin
        wr_bin = 0;
        wr_gray = 0;
    end

    always @(posedge wr_clk) begin
        if (wr_rst) begin
            wr_bin  <= 0;
            wr_gray <= 0;
        end else begin
            wr_bin  <= wr_bin_next;
            wr_gray <= wr_gray_next;
        end
    end

    // --- Write to memory ---
    always @(posedge wr_clk) begin
        if (wr_en && !full)
            mem[wr_bin[PTR_W-2:0]] <= wr_data;
    end

    // --- Read pointer (rd_clk domain, binary + gray) ---
    reg [PTR_W-1:0] rd_bin;
    wire [PTR_W-1:0] rd_bin_next = rd_bin + (rd_en & ~empty);
    wire [PTR_W-1:0] rd_gray_next = rd_bin_next ^ (rd_bin_next >> 1);
    reg [PTR_W-1:0] rd_gray;

    initial begin
        rd_bin = 0;
        rd_gray = 0;
    end

    always @(posedge rd_clk) begin
        if (rd_rst) begin
            rd_bin  <= 0;
            rd_gray <= 0;
        end else begin
            rd_bin  <= rd_bin_next;
            rd_gray <= rd_gray_next;
        end
    end

    // --- Read from memory (combinational) ---
    assign rd_data = mem[rd_bin[PTR_W-2:0]];

    // --- Synchronize wr_gray into rd_clk domain (for empty) ---
    (* ASYNC_REG = "TRUE" *) reg [PTR_W-1:0] wr_gray_sync1, wr_gray_sync2;

    initial begin
        wr_gray_sync1 = 0;
        wr_gray_sync2 = 0;
    end

    always @(posedge rd_clk) begin
        if (rd_rst) begin
            wr_gray_sync1 <= 0;
            wr_gray_sync2 <= 0;
        end else begin
            wr_gray_sync1 <= wr_gray;
            wr_gray_sync2 <= wr_gray_sync1;
        end
    end

    // --- Synchronize rd_gray into wr_clk domain (for full) ---
    (* ASYNC_REG = "TRUE" *) reg [PTR_W-1:0] rd_gray_sync1, rd_gray_sync2;

    initial begin
        rd_gray_sync1 = 0;
        rd_gray_sync2 = 0;
    end

    always @(posedge wr_clk) begin
        if (wr_rst) begin
            rd_gray_sync1 <= 0;
            rd_gray_sync2 <= 0;
        end else begin
            rd_gray_sync1 <= rd_gray;
            rd_gray_sync2 <= rd_gray_sync1;
        end
    end

    // --- Full and empty flags ---
    // Empty: rd gray == synced wr gray (in rd_clk domain)
    assign empty = (rd_gray == wr_gray_sync2);

    // Full: wr gray matches synced rd gray with top two bits inverted.
    // In gray code, a full-wrap difference shows as the top two bits
    // being different and the rest being the same.
    // Uses registered wr_gray (not wr_gray_next) to avoid a
    // combinatorial loop through wr_bin_next -> full -> wr_bin_next.
    // This makes full one cycle late (pessimistic), which is safe.
    assign full = (wr_gray == {~rd_gray_sync2[PTR_W-1:PTR_W-2],
                                rd_gray_sync2[PTR_W-3:0]});

endmodule
