`timescale 1ns / 1ps

module axis_act_slave
  import sa_pkg::*;
#(
  parameter int NROWS   = ROWS,              // = 16, the GEMM tile's K
  parameter int USER_W  = 1,                 // sideband bits carried per beat
  parameter int TDATA_W = ROWS * ACT_W       // = 128; must equal NROWS*ACT_W
) (
  input  logic                               clk,
  input  logic                               rst_n,

  // ---- AXI4-Stream slave: activations, one whole vector per beat ------------
  input  logic                               s_axis_tvalid,
  output logic                               s_axis_tready,
  input  logic [TDATA_W-1:0]                 s_axis_tdata,
  input  logic                               s_axis_tlast,
  input  logic [USER_W-1:0]                  s_axis_tuser,

  // ---- post-reset / mid-stream flush; day 13 schedules it -------------------
  input  logic                               flush,

  // ---- the consumer's acceptance; day 12 drives it --------------------------
  // High means "whatever leaves sa_datapath.sum_out this cycle will be taken".
  // Because the datapath has no internal storage, this is also what permits an
  // input vector to be admitted -- the two are one event.
  input  logic                               dn_ready,

  // ---- sa_datapath face ----------------------------------------------------
  output logic                               act_en,    // -> sa_datapath.en
  output logic                               act_vld,   // a REAL vector, not a
                                                        // flush zero
  output logic signed [NROWS-1:0][ACT_W-1:0] act_vec,   // -> sa_datapath.act_in
  output logic [USER_W-1:0]                  act_user,  // qualified by act_vld
  output logic                               act_last   // qualified by act_vld
);

  localparam int VEC_W = NROWS * ACT_W;              // 128
  localparam int PAY_W = TDATA_W + 1 + USER_W;       // 137 at the defaults

  // ---------------------------------------------------------------------------
  // Elaboration checks. The TDATA_W one is the load-bearing one and it is
  // verified directly with a throwaway top rather than by a mutant, in the style
  // of days 4, 7, 8 and 9: no legal instantiation can trip it, so no sweep can
  // kill the guard itself.
  // ---------------------------------------------------------------------------
  initial begin
    if (NROWS < 1)
      $fatal(1, "axis_act_slave: NROWS=%0d must be at least 1", NROWS);
    if (USER_W < 1)
      $fatal(1, "axis_act_slave: USER_W=%0d must be at least 1; a stream with no sideband should still carry one bit, because day 6's swap lead needs a per-vector marker",
             USER_W);
    if (TDATA_W != VEC_W)
      $fatal(1, "axis_act_slave: TDATA_W=%0d but NROWS*ACT_W=%0d. The array consumes one whole activation vector per enabled cycle, so a narrower stream halves its throughput and idles half the DSPs -- widen the stream outside this module with an AXI4-Stream width converter rather than adding a gearbox inside it",
             TDATA_W, VEC_W);
  end

