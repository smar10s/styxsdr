// SPDX-License-Identifier: MIT
// fpga/rtl/debug_snap.v — parametric debug snapshot BRAM
//
// Single-BRAM-pair capture buffer: writes continuously on sample_valid,
// freezes POST_DEPTH samples after `trig`, reads back via rd_addr.
//
// Pre-trigger region: [trig_wr - PRE_DEPTH + 1, trig_wr]
// Post-trigger region: [trig_wr + 1, trig_wr + POST_DEPTH]
// where PRE_DEPTH = DEPTH - POST_DEPTH.
//
// Circular mode (circular_en=1): buffer never freezes.  Each trig
// updates trig_wr/trig_cycle/post_cnt but writes continue forever.
// Rearm resets all state.
//
// Resource: 2 BRAM36K (48-bit × 1024), ~60 LUTs.

module debug_snap #(
    parameter DATA_WIDTH  = 32,
    parameter DEPTH       = 1024,
    parameter POST_DEPTH  = 512
) (
    input  wire        clk,
    input  wire        rst,

    input  wire        sample_valid,
    input  wire [DATA_WIDTH-1:0] sample_data,

    input  wire        trig,
    input  wire        rearm,
    input  wire        circular_en,

    input  wire [9:0]  rd_addr,
    output wire [DATA_WIDTH-1:0] rd_data,

    output wire        captured,
    output wire [9:0]  trig_wr,
    output wire [31:0] trig_cycle,
    output wire [10:0] wr_count
);

    localparam ADDR_W = 10;

    // =========================================================
    // BRAM (true dual-port, split across 2 BRAM36K for 48-bit width)
    // =========================================================
    (* ram_style = "block" *)
    reg [DATA_WIDTH-1:0] snap_mem [0:DEPTH-1];

    // Read pipeline
    reg [ADDR_W-1:0]     rd_addr_r;
    reg [DATA_WIDTH-1:0] rd_data_r;

    always @(posedge clk) begin
        rd_addr_r <= rd_addr;
        rd_data_r <= snap_mem[rd_addr_r];
    end

    assign rd_data = rd_data_r;

`ifndef SYNTHESIS
    integer _i;
    initial begin
        for (_i = 0; _i < DEPTH; _i = _i + 1)
            snap_mem[_i] = 0;
    end
`endif

    // =========================================================
    // All state in ONE always block — eliminates inter-block
    // ordering issues in Icarus simulation.
    // =========================================================
    reg [ADDR_W-1:0] wr_addr;
    reg              trig_seen;
    reg              captured_reg;
    reg [ADDR_W:0]   post_cnt;
    reg [ADDR_W-1:0] trig_wr_reg;
    reg [31:0]       trig_cycle_reg;
    reg [ADDR_W:0]   wr_cnt;
    reg [31:0]       cycle_counter;

    assign captured   = captured_reg;
    assign trig_wr    = trig_wr_reg;
    assign trig_cycle = trig_cycle_reg;
    assign wr_count   = wr_cnt;

    always @(posedge clk) begin
        if (rst || rearm) begin
            wr_addr        <= 0;
            trig_seen      <= 0;
            captured_reg   <= 0;
            post_cnt       <= 0;
            trig_wr_reg    <= 0;
            trig_cycle_reg <= 0;
            wr_cnt         <= 0;
            cycle_counter  <= 0;
        end else begin
            cycle_counter <= cycle_counter + 1'b1;

            // Write gate: active until capture completes, or always in
            // circular mode.
            if (sample_valid && !captured_reg) begin

                // BRAM write — uses pre-update wr_addr.
                snap_mem[wr_addr] <= sample_data;

                // Advance write pointer
                wr_addr <= wr_addr + 1'b1;

                // Write counter
                if (wr_cnt < DEPTH)
                    wr_cnt <= wr_cnt + 1'b1;

                // Post-trigger countdown
                if (trig_seen && (post_cnt > 0))
                    post_cnt <= post_cnt - 1;
            end

            // Trigger detection
            if (trig && (!trig_seen || circular_en) && !captured_reg) begin
                trig_seen      <= 1;
                trig_wr_reg    <= wr_addr;
                trig_cycle_reg <= cycle_counter;
                post_cnt       <= POST_DEPTH;
            end

            // Capture complete — suppressed in circular mode
            if (trig_seen && (post_cnt == 0) && !captured_reg && !circular_en)
                captured_reg <= 1;
        end
    end

endmodule
