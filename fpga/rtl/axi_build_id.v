// SPDX-License-Identifier: MIT
// axi_build_id.v — AXI4-Lite read-only build fingerprint register
//
// Single 32-bit register at offset 0x00, returns BUILD_ID parameter.
// Injected at synthesis time by the build script.
//
// AXI address: set in system_bd.tcl (default 0x43C0_0000)
// Read:  devmem 0x43C00000 → returns BUILD_ID value
// Write: ignored (read-only)
//
// Port naming follows Xilinx AXI4-Lite convention so Vivado
// automatically infers the interface for block design integration.

`timescale 1ns/1ps

module axi_build_id #(
    parameter BUILD_ID = 32'hDEAD_BEEF
) (
    // AXI4-Lite slave interface
    input  wire        s_axi_aclk,
    input  wire        s_axi_aresetn,

    // Read address channel
    input  wire [3:0]  s_axi_araddr,
    input  wire [2:0]  s_axi_arprot,
    input  wire        s_axi_arvalid,
    output reg         s_axi_arready,

    // Read data channel
    output reg  [31:0] s_axi_rdata,
    output wire [1:0]  s_axi_rresp,
    output reg         s_axi_rvalid,
    input  wire        s_axi_rready,

    // Write address channel (accept but ignore)
    input  wire [3:0]  s_axi_awaddr,
    input  wire [2:0]  s_axi_awprot,
    input  wire        s_axi_awvalid,
    output reg         s_axi_awready,

    // Write data channel (accept but ignore)
    input  wire [31:0] s_axi_wdata,
    input  wire [3:0]  s_axi_wstrb,
    input  wire        s_axi_wvalid,
    output reg         s_axi_wready,

    // Write response channel
    output wire [1:0]  s_axi_bresp,
    output reg         s_axi_bvalid,
    input  wire        s_axi_bready
);

    // Always OKAY response
    assign s_axi_rresp = 2'b00;
    assign s_axi_bresp = 2'b00;

    // Read logic: AR handshake, then R response one cycle later
    reg ar_latched;

    always @(posedge s_axi_aclk) begin
        if (!s_axi_aresetn) begin
            s_axi_arready <= 1'b0;
            s_axi_rvalid  <= 1'b0;
            s_axi_rdata   <= 32'h0;
            ar_latched    <= 1'b0;
        end else begin
            // Default: deassert ready after one cycle
            s_axi_arready <= 1'b0;

            if (s_axi_arvalid && !s_axi_arready && !s_axi_rvalid && !ar_latched) begin
                // Accept read address
                s_axi_arready <= 1'b1;
                ar_latched    <= 1'b1;
            end

            if (ar_latched && !s_axi_rvalid) begin
                // Present read data one cycle after AR handshake
                s_axi_rdata  <= BUILD_ID;
                s_axi_rvalid <= 1'b1;
                ar_latched   <= 1'b0;
            end

            if (s_axi_rvalid && s_axi_rready) begin
                // Read data consumed
                s_axi_rvalid <= 1'b0;
            end
        end
    end

    // Write logic: accept and discard (keeps AXI bus happy)
    // Pipeline: AW+W handshake in one cycle, B response in the next
    reg aw_w_latched;

    always @(posedge s_axi_aclk) begin
        if (!s_axi_aresetn) begin
            s_axi_awready <= 1'b0;
            s_axi_wready  <= 1'b0;
            s_axi_bvalid  <= 1'b0;
            aw_w_latched  <= 1'b0;
        end else begin
            s_axi_awready <= 1'b0;
            s_axi_wready  <= 1'b0;

            if (s_axi_awvalid && s_axi_wvalid && !s_axi_awready && !aw_w_latched) begin
                s_axi_awready <= 1'b1;
                s_axi_wready  <= 1'b1;
                aw_w_latched  <= 1'b1;
            end

            if (aw_w_latched && !s_axi_bvalid) begin
                s_axi_bvalid <= 1'b1;
                aw_w_latched <= 1'b0;
            end

            if (s_axi_bvalid && s_axi_bready) begin
                s_axi_bvalid <= 1'b0;
            end
        end
    end

endmodule
