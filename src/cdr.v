// -----------------------------------------------------------------------------
// cdr.v — Baud-rate CDR (PAM4) per your block diagram (TinyTapeout friendly)
// DATA(8) → DFF → X(8) → Quantizer → S(4) → DELAY_S → S1(4)
// X(8) → DELAY_X → X1(8) → MMPD → PHI(16) → Filter(PI 32) → DCO → Sample_en
// Notes for TT: no inferred multipliers; all shifts/adds; no initial blocks.
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
  output wire signed [31:0] PI          // filter output
);

  // ================= configuration =================
  localparam integer PHASE_BITS         = 32;
  // With 50 MHz clk, FCW_NOM = 0x8000_0000 gives Sample_en ≈ 25 MHz (UI = 2 clks)
  localparam [PHASE_BITS-1:0] FCW_NOM   = 32'h8000_0000;
  localparam integer KP_SHIFT           = 12;
  localparam integer KI_SHIFT           = 18;
  localparam integer DFCW_SHIFT         = 29;              // phase-only control
  localparam [PHASE_BITS-1:0] DFCW_STEP = (FCW_NOM >> 10);
  localparam signed  [31:0] DFCW_CLAMP  = $signed({1'b0, DFCW_STEP});

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

  // ================= Loop Filter (PI) =================
  wire signed [31:0] v_raw;
  Filter #(.KP_SHIFT(KP_SHIFT), .KI_SHIFT(KI_SHIFT)) u_pi (
    .clk(clk), .rst(rst), .en(Sample_en),
    .PHI(PHI),
    .PI (v_raw)
  );

  // ================= Scale + Clamp =================
  wire signed [31:0] df_unclamped = $signed(v_raw) >>> DFCW_SHIFT;
  wire signed [31:0] df_limited =
      (df_unclamped >  DFCW_CLAMP) ?  DFCW_CLAMP :
      (df_unclamped < -DFCW_CLAMP) ? -DFCW_CLAMP : df_unclamped;

  assign PI = v_raw;

  // ================= DCO =================
  DCO #(.PHASE_BITS(PHASE_BITS)) u_dco (
    .clk(clk), .rst(rst),
    .FCW_NOM(FCW_NOM),
    .dfcw(df_limited[PHASE_BITS-1:0]),
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
  wire signed [15:0] X16  = {{8{X[7]}},  X};
  wire signed [15:0] X1_16= {{8{X1[7]}}, X1};

  // multiply-by-const (±1, ±3) using shifts/adds
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

  wire signed [15:0] a = mul_sx(X1_16, S);   // S[n] * X[n-1]
  wire signed [15:0] b = mul_sx(X16,  S1);   // S[n-1] * X[n]
  assign PHI = a - b;
endmodule

// -----------------------------------------------------------------------------
// Filter — PI controller (digital loop filter)
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
  reg signed [31:0] acc;
  wire signed [31:0] p = $signed(PHI) >>> KP_SHIFT;
  wire signed [31:0] i = acc         >>> KI_SHIFT;

  always @(posedge clk) begin
    if (rst) begin
      acc <= 32'sd0;
      PI  <= 32'sd0;
    end else if (en) begin
      acc <= acc + $signed({{16{PHI[15]}}, PHI});
      PI  <= PI + p + i;
    end
  end
endmodule

// -----------------------------------------------------------------------------
// DCO — digital controlled oscillator (tick-on-wrap type)
// -----------------------------------------------------------------------------
module DCO #(
  parameter integer PHASE_BITS = 32
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
