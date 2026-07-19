// SPDX-License-Identifier: MIT

`timescale 1ns/1ps
// fpga/rtl/snap_axi.v
//
// AXI4-Lite wrapper for debug_snap.  Exposes control/status registers
// to the ARM PS so software can arm, trigger, and read back the IQ
// capture buffer.
//
// Register map:
//   0x00  CONTROL    RW  [0]=arm (W1 rearms), [1]=sw_trigger (W1S, self-clr), [2]=circular_en
//   0x04  STATUS     RO  [0]=captured, [1]=armed, [25:16]=trig_pos
//   0x08  TRIG_CYCLE RO  [31:0] cycle counter at trigger
//   0x0C  RD_ADDR    RW  [9:0] read address
//   0x10  RD_DATA    RO  [31:0] data at RD_ADDR (2-cycle BRAM latency hidden by AXI)

module snap_axi (
    (* X_INTERFACE_INFO = "xilinx.com:signal:clock:1.0 clk CLK" *)
    (* X_INTERFACE_PARAMETER = "ASSOCIATED_BUSIF s_axi, ASSOCIATED_RESET rst" *)
    input  wire        clk,

    (* X_INTERFACE_INFO = "xilinx.com:signal:reset:1.0 rst RST" *)
    (* X_INTERFACE_PARAMETER = "POLARITY ACTIVE_HIGH" *)
    input  wire        rst,

    // Sample input (from adc_sync or other fabric source)
    input  wire [31:0] sample_data,
    input  wire        sample_valid,

    // External trigger (from other fabric logic)
    input  wire        ext_trig,

    // --- AXI4-Lite slave ---
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
    // Register offsets
    // =========================================================
    localparam REG_CONTROL    = 8'h00;
    localparam REG_STATUS     = 8'h04;
    localparam REG_TRIG_CYCLE = 8'h08;
    localparam REG_RD_ADDR    = 8'h0C;
    localparam REG_RD_DATA    = 8'h10;

    // =========================================================
    // Internal state
    // =========================================================
    reg        armed;
    reg        circular_reg;
    reg [9:0]  rd_addr_reg;

    // Pulses
    reg        rearm_pulse;
    reg        sw_trig_pulse;

    // debug_snap interface wires
    wire        snap_captured;
    wire [9:0]  snap_trig_wr;
    wire [31:0] snap_trig_cycle;
    wire [10:0] snap_wr_count;
    wire [31:0] snap_rd_data;

    // Trigger gating
    wire trig_to_snap = (ext_trig | sw_trig_pulse) & armed;

    // =========================================================
    // debug_snap instance
    // =========================================================
    debug_snap #(
        .DATA_WIDTH (32),
        .DEPTH      (1024),
        .POST_DEPTH (512)
    ) u_snap (
        .clk          (clk),
        .rst          (rst),
        .sample_valid (sample_valid),
        .sample_data  (sample_data),
        .trig         (trig_to_snap),
        .rearm        (rearm_pulse),
        .circular_en  (circular_reg),
        .rd_addr      (rd_addr_reg),
        .rd_data      (snap_rd_data),
        .captured     (snap_captured),
        .trig_wr      (snap_trig_wr),
        .trig_cycle   (snap_trig_cycle),
        .wr_count     (snap_wr_count)
    );

    // =========================================================
    // AXI4-Lite slave — write path
    // =========================================================
    reg        aw_done, w_done;
    reg [31:0] aw_addr;
    reg [31:0] w_data;

    always @(posedge clk) begin
        if (rst) begin
            s_axi_awready <= 0;
            s_axi_wready  <= 0;
            s_axi_bvalid  <= 0;
            s_axi_bresp   <= 0;
            aw_done       <= 0;
            w_done        <= 0;
            aw_addr       <= 0;
            w_data        <= 0;
            armed         <= 0;
            circular_reg  <= 0;
            rd_addr_reg   <= 0;
            rearm_pulse   <= 0;
            sw_trig_pulse <= 0;
        end else begin
            // Default: clear pulses
            rearm_pulse   <= 0;
            sw_trig_pulse <= 0;

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

            // Execute write when both channels captured
            if (aw_done && w_done && !s_axi_bvalid) begin
                case (aw_addr[7:0])
                    REG_CONTROL: begin
                        // [0] arm: writing 1 arms/rearms, writing 0 disarms
                        // Rearm is suppressed when sw_trigger is also set,
                        // because rearm + trig on the same cycle causes
                        // debug_snap's reset branch to win (losing trigger).
                        // Normal flow: write ARM alone (rearms), feed data,
                        // then write ARM|TRIG (triggers without re-rearming).
                        if (w_data[0]) begin
                            armed       <= 1;
                            rearm_pulse <= ~w_data[1];
                        end else begin
                            armed <= 0;
                        end
                        // [1] sw_trigger: W1S, self-clearing
                        if (w_data[1])
                            sw_trig_pulse <= 1;
                        // [2] circular_en: level
                        circular_reg <= w_data[2];
                    end
                    REG_RD_ADDR: begin
                        rd_addr_reg <= w_data[9:0];
                    end
                    default: ;
                endcase
                s_axi_bvalid <= 1;
                s_axi_bresp  <= 2'b00;
                aw_done      <= 0;
                w_done       <= 0;
            end

            // B handshake
            if (s_axi_bvalid && s_axi_bready)
                s_axi_bvalid <= 0;
        end
    end

    // =========================================================
    // AXI4-Lite slave — read path
    // =========================================================
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
                    REG_CONTROL:    s_axi_rdata <= {29'b0, circular_reg, 1'b0, armed};
                    REG_STATUS:     s_axi_rdata <= {6'b0, snap_trig_wr, 14'b0, armed, snap_captured};
                    REG_TRIG_CYCLE: s_axi_rdata <= snap_trig_cycle;
                    REG_RD_ADDR:    s_axi_rdata <= {22'b0, rd_addr_reg};
                    REG_RD_DATA:    s_axi_rdata <= snap_rd_data;
                    default:        s_axi_rdata <= 32'hDEADBEEF;
                endcase
                s_axi_rvalid <= 1;
                s_axi_rresp  <= 2'b00;
                ar_done      <= 0;
            end

            // R handshake complete
            if (s_axi_rvalid && s_axi_rready)
                s_axi_rvalid <= 0;
        end
    end

endmodule
