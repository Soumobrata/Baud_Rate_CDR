// -----------------------------------------------------------------------------
// cdr.v — Baud-rate CDR (PAM4) per your block diagram (TT-friendly)
// DATA(1) → counter(256-UI span) → X(8) → Quantizer → S(4) → DELAY_S → S1(4)
// X(8)    → DELAY_X → X1(8) → MMPD(shift/add) → PHI(16) → Filter(PI 24b int) → DCO → Sample_en
// Notes:  - no multipliers (MMPD uses shifts/adds)
//         - trimmed widths for tiny tool runs (PHASE_BITS=24)
//         - Sample_en ≈ 25 MHz (UI ≈ 40 ns @ 50 MHz clk)
// -----------------------------------------------------------------------------
`timescale 1ps/1ps
`default_nettype none

module cdr (
  input  wire               clk,        // 50 MHz system clock
  input  wire               rst_n,
  input  wire               DATA,       // 1-bit VCO clock (<= 25 MHz)
  output wire               Sample_en,  // symbol strobe (1-cycle pulse)
  output wire signed [7:0]  X,          // counted/centered sample
  output wire signed [3:0]  S,          // quantized symbol (-3,-1,+1,+3)
  output wire signed [7:0]  X1,         // delayed X
  output wire signed [3:0]  S1,         // delayed S
  output wire signed [15:0] PHI,        // PD output
  output wire signed [31:0] PI          // filter output (port stays 32b)
);

  // ================= configuration =================
  localparam integer PHASE_BITS         = 24;           // trimmed from 32
  localparam [PHASE_BITS-1:0] FCW_NOM   = 24'h80_0000;  // ≈ UI = 2 clks → ~25 MHz
  localparam integer KP_SHIFT           = 12;
  localparam integer KI_SHIFT           = 18;
  localparam integer DFCW_SHIFT         = 27;           // phase-only control
  localparam [PHASE_BITS-1:0] DFCW_STEP = (FCW_NOM >> 10);
  localparam signed  [PHASE_BITS:0] DFCW_CLAMP =
      $signed({1'b0, DFCW_STEP});                       // ±step in PHASE_BITS domain

  localparam integer CNTR_BITS          = 14;           // free-run VCO counter width

  wire rst = ~rst_n;

  // ================= Counter front-end (spans 256 UIs) =================
  // For DATA ∈ {10,15,20,25} MHz and UI=25 MHz:
  // span counts ≈ {102,154,206,256}; center near mid (~180).
  reg [CNTR_BITS-1:0] N0_reg = 14'd180;

  counter #(
    .W(8),
    .CNTR_BITS(CNTR_BITS),
    .GAIN_SHIFT(0),      // no extra scaling; wide X swing (≈ ±80)
    .SPAN_UIS(256)
  ) u_counter (
    .clk      (clk),
    .rst      (rst),
    .Sample_en(Sample_en),
    .DATA     (DATA),
    .N0       (N0_reg),
    .Q        (X)
  );

  // ================= Quantizer (PAM4) =================
  Quantizer u_quant (.X(X), .S(S));

  // ================= 1-UI Delays =================
  DELAY_X #(.W(8)) u_dly_x (
    .clk(clk), .rst(rst), .en(Sample_en),
    .din(X), .dout(X1)
  );

  DELAY_S #(.W(4)) u_dly_s (
    .clk(clk), .rst(rst), .en(Sample_en),
    .din(S), .dout(S1)
  );

  // ================= Mueller–Müller PD (no multipliers) =================
  MMPD u_mmpd (.X(X), .X1(X1), .S(S), .S1(S1), .PHI(PHI));

  // ================= Loop Filter (PI) — 24b internal, 32b port =================
  wire signed [31:0] PI_32;
  Filter #(.KP_SHIFT(KP_SHIFT), .KI_SHIFT(KI_SHIFT)) u_pi (
    .clk(clk), .rst(rst), .en(Sample_en),
    .PHI(PHI),
    .PI (PI_32)
  );
  assign PI = PI_32;

  // ================= Scale + Clamp in PHASE_BITS domain =================
  // Convert PI_32 to small dfcw; clamp to ±DFCW_STEP (PHASE_BITS wide)
  wire signed [31:0] df_unclamped = $signed(PI_32) >>> DFCW_SHIFT;
  wire signed [PHASE_BITS-1:0] dfcw_limited =
      (df_unclamped >  $signed(DFCW_CLAMP)) ?  DFCW_CLAMP[PHASE_BITS-1:0] :
      (df_unclamped < -$signed(DFCW_CLAMP)) ? -DFCW_CLAMP[PHASE_BITS-1:0] :
        df_unclamped[PHASE_BITS-1:0];

  // ================= DCO =================
  DCO #(.PHASE_BITS(PHASE_BITS)) u_dco (
    .clk(clk), .rst(rst),
    .FCW_NOM(FCW_NOM),
    .dfcw(dfcw_limited),
    .Sample_en(Sample_en)
  );

endmodule


// -----------------------------------------------------------------------------
// counter — VCO-edge counter with multi-UI span (CDC + Gray)
// Counts edges of 1-bit DATA (async VCO clock). Each UI, outputs the
// number of edges accumulated over the last SPAN_UIS UIs, centered by N0,
// then scaled to W-bit signed Q.
// -----------------------------------------------------------------------------
module counter #(
  parameter integer W          = 8,
  parameter integer CNTR_BITS  = 14,  // free-running edge counter width
  parameter integer GAIN_SHIFT = 0,   // coarse right shift
  parameter integer SPAN_UIS   = 256  // number of UIs to span
)(
  input  wire              clk,         // CDR clock domain
  input  wire              rst,         // sync to clk
  input  wire              Sample_en,   // 1-cycle strobe (~25 MHz)
  input  wire              DATA,        // async VCO clock (≤ 25 MHz)
  input  wire [CNTR_BITS-1:0] N0,       // nominal span count (centering)
  output reg  signed [W-1:0] Q
);
  // VCO domain: free-running binary counter
  reg [CNTR_BITS-1:0] vco_bin = {CNTR_BITS{1'b0}};
  always @(posedge DATA) vco_bin <= vco_bin + 1'b1;

  // Binary → Gray
  wire [CNTR_BITS-1:0] vco_gray = vco_bin ^ (vco_bin >> 1);

  // 2-FF sync of Gray into clk domain
  reg [CNTR_BITS-1:0] g1, g2;
  always @(posedge clk) begin
    if (rst) begin g1 <= {CNTR_BITS{1'b0}}; g2 <= {CNTR_BITS{1'b0}}; end
    else begin g1 <= vco_gray; g2 <= g1; end
  end

  // Gray → Binary
  function [CNTR_BITS-1:0] gray2bin(input [CNTR_BITS-1:0] g);
    integer i; begin
      gray2bin[CNTR_BITS-1] = g[CNTR_BITS-1];
      for (i = CNTR_BITS-2; i >= 0; i = i - 1)
        gray2bin[i] = gray2bin[i+1] ^ g[i];
    end
  endfunction
  wire [CNTR_BITS-1:0] bin_now = gray2bin(g2);

  // Snapshot from SPAN_UIS UIs ago (simple shift-register history)
  reg [CNTR_BITS-1:0] hist [0:SPAN_UIS-1];
  integer k;

  // diff over K UIs = bin_now - bin_{now-K}
  wire [CNTR_BITS:0] diff_span =
      {1'b0,bin_now} - {1'b0,hist[SPAN_UIS-1]}; // assume no wrap over span

  // Center & scale to W-bit signed
  wire signed [CNTR_BITS:0] centered = $signed({1'b0,diff_span[CNTR_BITS-1:0]}) - $signed({1'b0,N0});
  wire signed [CNTR_BITS:0] scaled   = centered >>> GAIN_SHIFT;

  always @(posedge clk) begin
    if (rst) begin
      for (k = 0; k < SPAN_UIS; k = k + 1) hist[k] <= {CNTR_BITS{1'b0}};
      Q <= {W{1'b0}};
    end else if (Sample_en) begin
      // shift history & capture
      for (k = SPAN_UIS-1; k > 0; k = k - 1) hist[k] <= hist[k-1];
      hist[0] <= bin_now;

      // saturate to W bits
      if      (scaled >  $signed({1'b0,{(W-1){1'b1}}})) Q <=  {1'b0,{(W-1){1'b1}}};
      else if (scaled <  $signed({1'b1,{(W-1){1'b0}}})) Q <=  {1'b1,{(W-1){1'b0}}};
      else                                              Q <=  scaled[W-1:0];
    end
  end
endmodule


// -----------------------------------------------------------------------------
// Quantizer — PAM4 hard decision slicer
// -----------------------------------------------------------------------------
module Quantizer (
  input  wire signed [7:0] X,
  output reg  signed [3:0] S
);
  always @* begin
    if      (X < -64) S = -4'sd3;
    else if (X <   0) S = -4'sd1;
    else if (X <  64) S =  4'sd1;
    else              S =  4'sd3;
  end
endmodule


// -----------------------------------------------------------------------------
// Separate Delay Cells for X (8-bit) and S (4-bit)
// -----------------------------------------------------------------------------
module DELAY_X #(
  parameter integer W = 8
)(
  input  wire         clk,
  input  wire         rst,
  input  wire         en,
  input  wire [W-1:0] din,
  output reg  [W-1:0] dout
);
  always @(posedge clk) begin
    if (rst)     dout <= {W{1'b0}};
    else if (en) dout <= din;
  end
endmodule

module DELAY_S #(
  parameter integer W = 4
)(
  input  wire         clk,
  input  wire         rst,
  input  wire         en,
  input  wire [W-1:0] din,
  output reg  [W-1:0] dout
);
  always @(posedge clk) begin
    if (rst)     dout <= {W{1'b0}};
    else if (en) dout <= din;
  end
endmodule


// -----------------------------------------------------------------------------
// MMPD — Multilevel Mueller–Müller Phase Detector (shift-add only)
// PHI = S[n]*X[n-1] - S[n-1]*X[n], with S ∈ {−3, −1, +1, +3}
// -----------------------------------------------------------------------------
module MMPD (
  input  wire signed [7:0]  X,
  input  wire signed [7:0]  X1,
  input  wire signed [3:0]  S,
  input  wire signed [3:0]  S1,
  output wire signed [15:0] PHI
);
  // sign-extend once
  wire signed [15:0] X16   = {{8{X[7]}},  X};
  wire signed [15:0] X1_16 = {{8{X1[7]}}, X1};

  function automatic signed [15:0] mul_sx;
    input signed [15:0] x;
    input signed [3:0]  s; // -3,-1,+1,+3
    begin
      case (s)
        -4'sd3: mul_sx = - (x + (x <<< 1)); // -(3*x)
        -4'sd1: mul_sx = - x;
         4'sd1: mul_sx =   x;
         4'sd3: mul_sx =   x + (x <<< 1);  // 3*x
        default: mul_sx = 16'sd0;
      endcase
    end
  endfunction

  wire signed [15:0] a = mul_sx(X1_16, S);   // S[n]   * X[n-1]
  wire signed [15:0] b = mul_sx(X16,   S1);  // S[n-1] * X[n]
  assign PHI = a - b;
endmodule


// -----------------------------------------------------------------------------
// Filter — PI controller (digital loop filter)
// Internals use 24 bits; output port remains 32 bits (sign-extended).
// -----------------------------------------------------------------------------
module Filter #(
  parameter integer KP_SHIFT = 12,
  parameter integer KI_SHIFT = 18
)(
  input  wire               clk,
  input  wire               rst,
  input  wire               en,
  input  wire signed [15:0] PHI,
  output reg  signed [31:0] PI
);
  localparam integer ACC_BITS = 24;

  reg  signed [ACC_BITS-1:0] acc;                // 24-bit integrator

  wire signed [31:0] phi32         = {{16{PHI[15]}}, PHI};
  wire signed [31:0] phi32_shifted = phi32 >>> KP_SHIFT;
  wire signed [ACC_BITS-1:0] p24   = phi32_shifted[ACC_BITS-1:0];

  wire signed [ACC_BITS-1:0] i24   = ($signed(acc) >>> KI_SHIFT);
  wire signed [ACC_BITS-1:0] pi24_prev = PI[ACC_BITS-1:0];
  wire signed [ACC_BITS-1:0] pi24_next = pi24_prev + p24 + i24;

  always @(posedge clk) begin
    if (rst) begin
      acc <= '0;
      PI  <= 32'sd0;
    end else if (en) begin
      acc <= acc + {{(ACC_BITS-16){PHI[15]}}, PHI};  // integrate PHI
      PI  <= {{(32-ACC_BITS){pi24_next[ACC_BITS-1]}}, pi24_next}; // sign-extend
    end
  end
endmodule


// -----------------------------------------------------------------------------
// DCO — digital controlled oscillator (tick-on-wrap type), param PHASE_BITS
// -----------------------------------------------------------------------------
module DCO #(
  parameter integer PHASE_BITS = 24
)(
  input  wire                         clk,
  input  wire                         rst,
  input  wire [PHASE_BITS-1:0]        FCW_NOM,
  input  wire signed [PHASE_BITS-1:0] dfcw,
  output wire                         Sample_en
);
  reg [PHASE_BITS-1:0] phase;

  wire signed [PHASE_BITS:0] sum =
      $signed({1'b0, FCW_NOM}) + $signed({dfcw[PHASE_BITS-1], dfcw});
  wire [PHASE_BITS-1:0] eff =
      (sum <= 0) ? {PHASE_BITS{1'b0}} :
      (sum >  $signed({1'b0,{PHASE_BITS{1'b1}}})) ? {PHASE_BITS{1'b1}} :
       sum[PHASE_BITS-1:0];

  wire [PHASE_BITS-1:0] nxt = phase + eff;
  assign Sample_en = (nxt < phase);   // wrap → 1-cycle strobe

  always @(posedge clk) begin
    if (rst) phase <= {PHASE_BITS{1'b0}};
    else     phase <= nxt;
  end
endmodule

`default_nettype wire
