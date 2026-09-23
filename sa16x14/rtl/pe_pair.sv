// -----------------------------------------------------------------------------
// pe_pair.sv — one processing element: two stationary weights, one DSP
//
// A pe_pair is the array's unit cell. It owns one dsp_pack_mac (two INT8 MACs
// packed into a single DSP48E1) and adds the two things a systolic cell needs
// beyond the arithmetic:
//
//   1. the horizontal activation hand-off to the PE on its right
//   2. a serial weight-load shift path, so a whole column of PEs is loaded
//      through one 8-bit port instead of 2*ROWS parallel ones
//
// It is deliberately thin. Everything that could go in the DSP stays in the
// DSP; pe_pair only adds the two registers holding the stationary weights.
//
// -----------------------------------------------------------------------------
// Geometry: which wire goes which way
// -----------------------------------------------------------------------------
// This cell sits at (row i, column-pair p) and serves output columns
// j0 = 2p (via w0, the packed LOW field) and j1 = 2p+1 (via w1, the HIGH
// field). That mapping matches model/gemm_int8.py:gemm_tile_hw(), which feeds
// packed_group() with w0s from the even column and w1s from the odd one.
//
//   act_in  ->  act_out     horizontal, left to right, 1 cycle per PE
//   pcin    ->  pcout       vertical, top to bottom, 1 cycle per PE
//   wgt_in  ->  wgt_out     vertical, top to bottom, 2 cycles per PE
//
// The activation path and the cascade path both advance exactly one cycle per
// cell, which is what keeps the wavefront coherent.
//
// -----------------------------------------------------------------------------
// The horizontal register is inside the DSP, not around it
// -----------------------------------------------------------------------------
// act_in feeds the MAC's B input directly and act_out is the MAC's registered
// copy of it. That is not a shortcut, it is the DSP48E1's BREG/BCOUT cascade:
// one B register, used both by this cell's multiplier and as the value handed
// to the neighbour. Adding a register here *around* the MAC would insert a
// second stage into the horizontal path only, so the activation wavefront
// would advance at half the rate of the cascade and every column past the
// first would combine the wrong vectors. tb_pe_pair.sv's horizontal-delay
// phase pins act_out to exactly one cycle for that reason.
//
// -----------------------------------------------------------------------------
// The weight shift path, and the order weights must be presented in
// -----------------------------------------------------------------------------
// Each PE holds a 2-deep shift register: wgt_in -> w1 (odd column) -> wgt_out,
// with w0 (even column) as the first stage.
//
//     wgt_in --> [w0] --> [w1] --> wgt_out --> next PE's wgt_in
//
// Chaining NPE cells makes one 2*NPE-deep shift register, loaded through a
// single 8-bit port in 2*NPE cycles with wgt_shift_en held high. Because the
// first value shifted in travels furthest, the load order is:
//
//     for i = NPE-1 down to 0:   present w1[i], then w0[i]
//
// i.e. the FARTHEST cell's ODD-column weight goes in first. Getting this
// backwards transposes the tile and is silent -- every number is plausible.
// tb_pe_pair.sv's one-hot placement phase pins each of the 2*NPE positions
// individually so a reversal cannot pass.
//
// wgt_shift_en and en are independent controls: the weight chain is not gated
// by the pipeline enable. There is a single weight bank, so a load *does*
// disturb results in flight -- the array must be drained across a tile change
// until the double buffer lands (roadmap day 6). Until then, hold en and drain
// with zero activations while loading, exactly as the testbenches do.
// -----------------------------------------------------------------------------
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
