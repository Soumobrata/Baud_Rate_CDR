// -----------------------------------------------------------------------------
// cdr.v — Baud-rate CDR (PAM4) per your block diagram (TT-friendly)
// DATA(8) → DFF → X(8) → Quantizer → S(4) → DELAY_S → S1(4)
// X(8) → DELAY_X → X1(8) → MMPD → PHI(16) → Filter(PI 32) → DCO → Sample_en
// Notes: no multipliers; trimmed internal widths to fit 1×1 better.
// -----------------------------------------------------------------------------
`timescale 1ps/1ps
`default_nettype none

module cdr (
  input  wire               clk,        // 50 MHz system clock
  input  wire               rst_n,
  input  wire signed [7:0]  DATA,       // input data stream
  output wire               Sample_en,  // symbol strobe (1-cycle pulse)
  output wire signed [7:0]  X,          // sampled data
  output wire signed [3:0]  S,          // quantized symbol (-3,-1,+1,+3)
  output wire signed [7:0]  X1,         // delayed X
  output wire signed [3:0]  S1,         // delayed S
  output wire signed [15:0] PHI,        // PD output
  output wire signed [31:0] PI          // filter output (port stays 32b)
);

  // ================= configuration =================
  localparam integer PHASE_BITS         = 24;               // trimmed from 32
  localparam [PHASE_BITS-1:0] FCW_NOM   = 24'h80_0000;      // ≈ UI = 2 clks
  localparam integer KP_SHIFT           = 12;
  localparam integer KI_SHIFT           = 18;
  localparam integer DFCW_SHIFT         = 27;               // was 29; small tweak
  localparam [PHASE_BITS-1:0] DFCW_STEP = (FCW_NOM >> 10);
  localparam signed  [PHASE_BITS:0] DFCW_CLAMP  =
      $signed({1'b0, DFCW_STEP}); // ±step in PHASE_BITS domain

  wire rst = ~rst_n;

  // ================= D Flip-Flop (Sampler) =================
  DFF u_dff (
    .clk(clk),
    .en (Sample_en),   // sample at symbol strobe
    .rst(rst),
    .D  (DATA),
    .Q  (X)
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
// DFF — D Flip-Flop (Sampler)
// -----------------------------------------------------------------------------
module DFF (
  input  wire              clk,
  input  wire              en,
  input  wire              rst,
  input  wire signed [7:0] D,
  output reg  signed [7:0] Q
);
  always @(posedge clk) begin
    if (rst)      Q <= 8'sd0;
    else if (en)  Q <= D;
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
  // sign-extend to 16b once
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

  reg  signed [ACC_BITS-1:0] acc;        // 24-bit integrator
  wire signed [31:0] phi32 = {{16{PHI[15]}}, PHI};
  wire signed [ACC_BITS-1:0] p24 = (phi32 >>> KP_SHIFT)[ACC_BITS-1:0];
  wire signed [ACC_BITS-1:0] i24 = acc >>> KI_SHIFT;
  wire signed [ACC_BITS-1:0] pi24_next = (PI[ACC_BITS-1:0]) + p24 + i24;

  always @(posedge clk) begin
    if (rst) begin
      acc <= '0;
      PI  <= 32'sd0;
    end else if (en) begin
      acc <= acc + {{(ACC_BITS-16){PHI[15]}}, PHI}; // integrate PHI
      // Sign-extend 24b to 32b for the external PI port
      PI  <= {{(32-ACC_BITS){pi24_next[ACC_BITS-1]}}, pi24_next};
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
