// -----------------------------------------------------------------------------
// pack_extract.sv — separate the two products out of a packed DSP result
//
// Given P = p1 * 2^K + p0, recover p0 and p1.
//
// This is NOT a plain bit-slice. A negative p0 is stored in two's complement in
// the low field, which borrows from the field above it, so the naive slice
// returns p1-1 whenever p0 < 0. The fix is to add the low field's sign bit back
// into the high field:
//
//     p0 = signed(P[K-1:0])
//     p1 = signed(P[47:K]) + P[K-1]
//
// Proof: signed(P[47:K]) = floor(P / 2^K) = floor(p1 + p0/2^K), which is p1 for
// p0 >= 0 and p1-1 for p0 < 0. P[K-1] is exactly the sign of the low field. QED.
//
// Getting this wrong is the classic failure mode of INT8 DSP packing: it is
// correct for ~half of all inputs, so it passes casual testing and then
// produces subtly wrong inference results. tb_pack_extract.sv tests it
// exhaustively for that reason.
// -----------------------------------------------------------------------------
`timescale 1ns / 1ps

module pack_extract
  import sa_pkg::*;
#(
  parameter int K   = K_PACK,
  parameter int OUT_W = GROUP_W
) (
  input  logic signed [DSP_P_W-1:0] p_packed,
  output logic signed [OUT_W-1:0]   p0,
  output logic signed [OUT_W-1:0]   p1
);

  // Low field: K bits, interpreted signed. Exact as long as the accumulated
  // magnitude stays under 2^(K-1) -- which is what bounds CASCADE_DEPTH to 4.
  logic signed [K-1:0] lo;
  assign lo = p_packed[K-1:0];

  // High field at full width, plus the borrow correction.
  //
  // Only the low OUT_W bits of hi_full survive into p1. The bits above are
  // sign extension by construction: the high field accumulates at most
  // CASCADE_DEPTH products of magnitude <= PROD_MAX, so |p1| <= 65024, which
  // fits OUT_W = 18 bits signed with room to spare. The assertion below checks
  // that reasoning at runtime rather than trusting it, and the lint waiver is
  // scoped to this one declaration so genuinely-unused signals elsewhere are
  // still reported.
  /* verilator lint_off UNUSEDSIGNAL */
  logic signed [DSP_P_W-K-1:0] hi_full;
  /* verilator lint_on UNUSEDSIGNAL */
  // The borrow is widened explicitly and kept signed. Writing it inline as
  // `+ p_packed[K-1]` makes the whole expression unsigned, which silently
  // destroys the sign of the high field.
  logic signed [DSP_P_W-K-1:0] borrow;
  assign borrow  = p_packed[K-1] ? {{(DSP_P_W-K-1){1'b0}}, 1'b1} : '0;
  assign hi_full = $signed(p_packed[DSP_P_W-1:K]) + borrow;

  assign p0 = OUT_W'(lo);
  assign p1 = OUT_W'(hi_full);

`ifdef SA_ASSERT
  // If this fires, either CASCADE_DEPTH was raised past the low-field bound or
  // OUT_W is too narrow for the accumulation depth feeding this extractor.
  // Both corrupt results silently without it.
  // `always @(*)` rather than `always_comb`: this block is simulation-only and
  // $error is not synthesisable, which always_comb would flag.
  always @(*) begin
    if (hi_full !== 'x)
      assert (hi_full >= $signed({{(DSP_P_W-K-OUT_W){1'b1}}, {(OUT_W-1){1'b0}}, 1'b0}) &&
              hi_full <= $signed({{(DSP_P_W-K-OUT_W+1){1'b0}}, {(OUT_W-1){1'b1}}}))
        else $error("pack_extract: high field %0d does not fit OUT_W=%0d", hi_full, OUT_W);
  end
`endif

endmodule
