`timescale 1ns / 1ps

module pe_pair
  import sa_pkg::*;
#(
  parameter int K         = K_PACK,
  // Saturate incoming weights into [WGT_MIN, WGT_MAX] as they are shifted in.
  // On by default: w = -128 is the design's one silent-corruption hazard, and
  // one comparator per cell removes it at the only place it can enter.
  parameter bit CLAMP_WGT = 1'b1
) (
  input  logic                      clk,
  input  logic                      rst_n,
  input  logic                      en,            // datapath pipeline advance

  // ---- weight load chain (vertical) ----------------------------------------
  input  logic                      wgt_shift_en,  // advance the weight chain
  input  logic signed [WGT_W-1:0]   wgt_in,
  output logic signed [WGT_W-1:0]   wgt_out,

  // ---- activation (horizontal) ---------------------------------------------
  input  logic signed [ACT_W-1:0]   act_in,
  output logic signed [ACT_W-1:0]   act_out,

  // ---- partial-sum cascade (vertical) --------------------------------------
  input  logic                      acc_first,
  input  logic signed [DSP_P_W-1:0] pcin,
  output logic signed [DSP_P_W-1:0] pcout
);

  // Latency from act_in to this cell's contribution appearing at pcout.
  // Identical to dsp_pack_mac: pe_pair adds no datapath stages. A chain of N
  // cells costs LATENCY + (N-1), one cycle per further cascade stage.
  localparam int LATENCY = 3;

  // ---------------------------------------------------------------------------
  // Stationary weights: a 2-deep shift register that doubles as the weight
  // storage. No separate load register -- the shift register IS the bank.
  // ---------------------------------------------------------------------------
  logic signed [WGT_W-1:0] w0_r, w1_r;
  logic signed [WGT_W-1:0] wgt_in_c;

  // Clamp on the input stage only. w1_r is fed from w0_r and wgt_out from
  // w1_r, both already clamped, so one comparator per cell is enough to make
  // every stored weight legal -- including weights that arrived through an
  // upstream neighbour.
  assign wgt_in_c = CLAMP_WGT ? sa_pkg::clamp_wgt(wgt_in) : wgt_in;

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      w0_r <= '0;
      w1_r <= '0;
    end else if (wgt_shift_en) begin
      w0_r <= wgt_in_c;
      w1_r <= w0_r;
    end
  end

  assign wgt_out = w1_r;

  // ---------------------------------------------------------------------------
  // The MAC. act_in goes in unregistered; act_out comes back registered once
  // (BREG/BCOUT), which is the horizontal systolic hand-off.
  // ---------------------------------------------------------------------------
  dsp_pack_mac #(.K(K)) u_mac (
    .clk       (clk),
    .rst_n     (rst_n),
    .en        (en),
    .w0        (w0_r),
    .w1        (w1_r),
    .act       (act_in),
    .acc_first (acc_first),
    .pcin      (pcin),
    .pcout     (pcout),
    .act_out   (act_out)
  );

`ifdef SA_ASSERT
  // The clamp makes -128 harmless in hardware, but a host that emits it is
  // still quantising wrongly and should hear about it in simulation.
  // `always @(posedge clk)` rather than `always_ff`: simulation-only, and
  // $warning is not synthesisable.
  always @(posedge clk) begin
    if (rst_n && wgt_shift_en && CLAMP_WGT) begin
      if (wgt_in < WGT_W'(WGT_MIN) || wgt_in > WGT_W'(WGT_MAX))
        $warning("pe_pair: weight %0d clamped into [%0d,%0d]; host quantisation should be symmetric",
                 wgt_in, WGT_MIN, WGT_MAX);
    end
  end
`endif

endmodule
