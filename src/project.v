
/*
 * TinyTapeout wrapper for Baud-Rate PAM4 CDR
 * SPDX-License-Identifier: Apache-2.0
 */
`default_nettype none

module tt_um_baud_rate_cdr (
`ifdef GL_TEST
    input  wire VPWR,
    input  wire VGND,
`endif
    input  wire [7:0] ui_in,    // DATA[7:0]
    output wire [7:0] uo_out,   // debug/status
    input  wire [7:0] uio_in,   // unused
    output wire [7:0] uio_out,  // unused
    output wire [7:0] uio_oe,   // unused
    input  wire       ena,      // enable
    input  wire       clk,      // ~50 MHz
    input  wire       rst_n     // async, active-low
);

  // Tie off bidir
  assign uio_out = 8'h00;
  assign uio_oe  = 8'h00;

  // Gate inputs when disabled
  wire signed [7:0] DATA = ena ? $signed(ui_in) : 8'sd0;

  // Core nets
  wire               Sample_en;
  wire signed [7:0]  X, X1;
  wire signed [3:0]  S, S1;
  wire signed [15:0] PHI;
  wire signed [31:0] PI;

  // Core
  cdr u_core (
    .clk(clk),
    .rst_n(rst_n & ena),
    .DATA(DATA),
    .Sample_en(Sample_en),
    .X(X), .S(S), .X1(X1), .S1(S1),
    .PHI(PHI), .PI(PI)
  );

  // Register uo_out so top is unquestionably sequential for ABC
  reg [7:0] uo_q;
  always @(posedge clk or negedge rst_n) begin
    if (!rst_n || !ena) uo_q <= 8'h00;
    else                uo_q <= {Sample_en, S[2:0], X[7:4]};
  end
  assign uo_out = uo_q;

endmodule

`default_nettype wire
