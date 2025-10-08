/*
 * TinyTapeout wrapper for Baud-Rate PAM4 CDR
 * SPDX-License-Identifier: Apache-2.0
 */

`default_nettype none

module tt_um_baud_rate_cdr (
    input  wire [7:0] ui_in,    // Dedicated inputs (DATA[7:0])
    output wire [7:0] uo_out,   // Dedicated outputs (debug/status)
    input  wire [7:0] uio_in,   // Bidirectional IOs: input path (unused)
    output wire [7:0] uio_out,  // Bidirectional IOs: output path (unused)
    output wire [7:0] uio_oe,   // Bidirectional IOs: output enable (unused)
    input  wire       ena,      // 1 when active; 0 = disabled/scan mode
    input  wire       clk,      // System clock (~50 MHz)
    input  wire       rst_n     // Active-low reset
);

  // ---------------------------------------------------------------------------
  // Tie off unused bidirectional IOs
  // ---------------------------------------------------------------------------
  assign uio_out = 8'h00;
  assign uio_oe  = 8'h00;

  // ---------------------------------------------------------------------------
  // Input gating: quiet core when disabled
  // ---------------------------------------------------------------------------
  wire signed [7:0] DATA = ena ? $signed(ui_in) : 8'sd0;

  // ---------------------------------------------------------------------------
  // Internal nets from CDR core
  // ---------------------------------------------------------------------------
  wire               Sample_en;
  wire signed [7:0]  X, X1;
  wire signed [3:0]  S, S1;
  wire signed [15:0] PHI;
  wire signed [31:0] PI;

  // ---------------------------------------------------------------------------
  // Instantiate the core CDR
  // ---------------------------------------------------------------------------
  cdr u_core (
    .clk(clk),
    .rst_n(rst_n & ena),  // hold in reset when disabled
    .DATA(DATA),
    .Sample_en(Sample_en),
    .X(X), .S(S), .X1(X1), .S1(S1),
    .PHI(PHI), .PI(PI)
  );

  // ---------------------------------------------------------------------------
  // Drive outputs (for debug / visibility on TinyTapeout board)
  // Bits:
  //   [7] = Sample_en  (symbol strobe)
  //   [6:4] = S[2:0]   (quantizer level)
  //   [3:0] = X[7:4]   (MSBs of sampled data)
  // ---------------------------------------------------------------------------
  assign uo_out = {Sample_en, S[2:0], X[7:4]};

endmodule

`default_nettype wire
