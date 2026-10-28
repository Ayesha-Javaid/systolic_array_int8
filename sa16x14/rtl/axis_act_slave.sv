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

  input  logic                               flush,

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


  logic             up_v, up_r;
  logic [PAY_W-1:0] up_pay;
  logic             q_v, q_r;
  logic [PAY_W-1:0] q_pay;

  assign up_pay        = {s_axis_tuser, s_axis_tlast, s_axis_tdata};
  assign up_v          = s_axis_tvalid & rst_n;
  assign s_axis_tready = up_r & rst_n;

  skid_buf #(
    .W (PAY_W)
  ) u_skid (
    .clk      (clk),
    .rst_n    (rst_n),
    .up_valid (up_v),
    .up_ready (up_r),
    .up_data  (up_pay),
    .dn_valid (q_v),
    .dn_ready (q_r),
    .dn_data  (q_pay)
  );


  assign act_en  = dn_ready & (flush | q_v);
  assign act_vld = dn_ready & ~flush & q_v;
  assign q_r     = act_vld;

  // Row i comes from byte lane i of TDATA: the order a host memcpy of an int8
  // K-vector produces, and the order the AXI byte lanes already imply. It is not
  // a free choice -- the weights arrive by a separate port, so a reversed or
  // rotated mapping pairs row i's activation with row NROWS-1-i's weights and
  // every column sum comes out plausible and wrong. tb_axis_act_slave scores
  // that against the golden model, and a one-hot lane phase names the offender.
  //
  // genvar continuous assigns, not a procedural element-wise write: day 4's
  // combinational-array trap, and the propagation trap in NOTES-tooling.md.
  genvar gi;
  generate
    for (gi = 0; gi < NROWS; gi = gi + 1) begin : g_lane
      assign act_vec[gi] = flush ? '0
                                 : $signed(q_pay[ACT_W*gi +: ACT_W]);
    end
  endgenerate

  assign act_last = act_vld & q_pay[TDATA_W];
  assign act_user = act_vld ? q_pay[TDATA_W+1 +: USER_W] : '0;

`ifdef SA_ASSERT

  logic             p_v, p_r;
  logic [PAY_W-1:0] p_pay;

  always @(posedge clk) begin
    if (!rst_n) begin
      p_v   <= 1'b0;
      p_r   <= 1'b0;
      p_pay <= '0;
    end else begin
      if (p_v && !p_r) begin
        assert (s_axis_tvalid)
          else $error("axis_act_slave: TVALID dropped before TREADY -- a beat was withdrawn, so an activation vector is silently missing");
        assert ({s_axis_tuser, s_axis_tlast, s_axis_tdata} === p_pay)
          else $error("axis_act_slave: TDATA/TLAST/TUSER changed while TVALID was held");
      end
      p_v   <= s_axis_tvalid;
      p_r   <= s_axis_tready;
      p_pay <= up_pay;
    end
  end

  // act_vec must never carry a don't-care into the multipliers on an enabled
  // cycle. This is the payoff of skid_buf resetting its data registers; it would
  // fire if that reset were dropped.
  always @(posedge clk) begin
    if (rst_n && act_en) begin
      assert (!$isunknown(act_vec))
        else $error("axis_act_slave: act_vec has an x/z bit on an enabled cycle");
    end
  end
`endif

endmodule
