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
  input  logic                      wgt_shift_en,  // advance the shadow chain
  input  logic signed [WGT_W-1:0]   wgt_in,
  output logic signed [WGT_W-1:0]   wgt_out,

  // ---- weight bank swap (travels rightwards with the activation) ------------
  input  logic                      swap_in,       // 1-cycle: active <= shadow
  output logic                      swap_out,      // swap_in delayed one cycle

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
  // Shadow bank: the 2-deep shift register. This is the load path and nothing
  // else reads it except the swap below and the neighbour beneath.
  // ---------------------------------------------------------------------------
  logic signed [WGT_W-1:0] w0_s, w1_s;
  logic signed [WGT_W-1:0] wgt_in_c;

  // Clamp on the input stage only. w1_s is fed from w0_s and wgt_out from
  // w1_s, both already clamped, so one comparator per cell is enough to make
  // every stored weight legal -- including weights that arrived through an
  // upstream neighbour. The active bank is a copy of the shadow bank, so it
  // inherits the clamp rather than needing its own.
  assign wgt_in_c = CLAMP_WGT ? sa_pkg::clamp_wgt(wgt_in) : wgt_in;

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      w0_s <= '0;
      w1_s <= '0;
    end else if (wgt_shift_en) begin
      w0_s <= wgt_in_c;
      w1_s <= w0_s;
    end
  end

  assign wgt_out = w1_s;

  // ---------------------------------------------------------------------------
  // Active bank: what the multiplier sees. Updated only by a swap, and only on
  // an enabled edge -- the swap is part of the datapath timing, so it must
  // freeze with the wavefront. A swap presented while en is low simply waits:
  // en = 0 stretches the cycle in which the current activation is still being
  // consumed, and a bank change inside that stretch would corrupt it.
  //
  // swap_out sits in the same register stage under the same enable, which is
  // what keeps the swap pulse and the activation advancing at the same rate
  // across the columns.
  // ---------------------------------------------------------------------------
  logic signed [WGT_W-1:0] w0_a, w1_a;

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      w0_a     <= '0;
      w1_a     <= '0;
      swap_out <= 1'b0;
    end else if (en) begin
      swap_out <= swap_in;
      if (swap_in) begin
        w0_a <= w0_s;
        w1_a <= w1_s;
      end
    end
  end

  // ---------------------------------------------------------------------------
  // The MAC. act_in goes in unregistered; act_out comes back registered once
  // (BREG/BCOUT), which is the horizontal systolic hand-off.
  // ---------------------------------------------------------------------------
  dsp_pack_mac #(.K(K)) u_mac (
    .clk       (clk),
    .rst_n     (rst_n),
    .en        (en),
    .w0        (w0_a),
    .w1        (w1_a),
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

  // A swap held high for a second consecutive cycle *while the shadow chain is
  // shifting* copies a half-loaded tile into the active bank. One cycle
  // overlapping the start of the next load is fine and is how back-to-back
  // tiles work (see the header); two is not. This is silent otherwise: the
  // active bank ends up holding a mixture of two tiles, entirely in range.
  // Synchronous reset: sim-only state does not need an async one, and this keeps
  // the number of async-reset usages in the file at one. (It does not silence
  // the SYNCASYNCNET that `-Wall -DSA_ASSERT` reports on rst_n -- that comes
  // from day 2's clamp-warning block reading rst_n synchronously while the
  // weight shift register resets asynchronously, and it predates this file's
  // double buffer. Lint is run without SA_ASSERT, as days 1-5 did.)
  logic swap_q;
  always @(posedge clk) begin
    if (!rst_n)  swap_q <= 1'b0;
    else if (en) swap_q <= swap_in;
  end

  always @(posedge clk) begin
    if (rst_n && en && swap_in && swap_q && wgt_shift_en)
      $error("pe_pair: swap_in held over two shifting cycles; the active bank is taking a partially loaded tile");
  end
`endif

endmodule
