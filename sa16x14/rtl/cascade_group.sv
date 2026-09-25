// -----------------------------------------------------------------------------
// cascade_group.sv — the deepest legal DSP cascade, plus its extractor
//
// A cascade_group is DEPTH pe_pairs chained through the DSP PCOUT->PCIN path,
// terminated by one pack_extract. It is the unit in which packed-domain
// accumulation is allowed to happen: inside the group the two INT8 products
// share one 48-bit accumulator and add for free; at the group boundary they are
// separated into two ordinary signed numbers and everything above this level
// (day 4's fabric adder tree) adds them the expensive way.
//
// The whole reason this module exists as a named thing, rather than as a loop
// inside column_pair, is that DEPTH is not a free parameter. It is pinned by
// arithmetic:
//
//     low field width             = K_PACK           = 17 bits signed
//     largest single product      = 127 * 128        = 16256   (PROD_MAX)
//     products before it bleeds   = 65535 / 16256    = 4       (CASCADE_DEPTH)
//
// A fifth term in the same cascade pushes the low field past 2^16-1 and it
// carries into the high field, which is a silent corruption of the *other*
// output column. So DEPTH is bounded here, in one place, with an elaboration
// check, and the cascade head is tied off rather than exposed as a port. A
// module with a pcin port is a module someone can chain six deep.
//
// -----------------------------------------------------------------------------
// Geometry
// -----------------------------------------------------------------------------
// The group covers DEPTH consecutive rows (accumulation indices k) of one
// column-pair p, and produces that column-pair's two partial sums over those
// rows:
//
//     sum_even  =  sum over g of  w0[g] * act_in[g]     -> output column 2p
//     sum_odd   =  sum over g of  w1[g] * act_in[g]     -> output column 2p+1
//
// w0 is the packed LOW field and w1 the HIGH field, matching
// sa_pkg::pack_weights() and model/gemm_int8.py:gemm_tile_hw().
//
//     wgt_in ---> [pe 0] ---> [pe 1] ---> ... ---> [pe DEPTH-1] ---> wgt_out
//     casc 0 ===> [pe 0] ===> [pe 1] ===> ... ===> [pe DEPTH-1] ===> extract
//     act_in[g] -> [pe g] -> act_out[g]        (one per row, sideways)
//
// The cascade and the horizontal activation path both advance exactly one cycle
// per cell. The weight chain advances two cycles per cell and is independent of
// `en`.
//
// -----------------------------------------------------------------------------
// The group does NOT contain the stagger
// -----------------------------------------------------------------------------
// act_in[g] must already carry row g's skew: row g's activation arrives g
// cycles after row 0's, because cell g's product has to meet a partial sum that
// is g cycles old. Feeding a flat broadcast into act_in combines g different
// activation vectors into one dot product and every result still looks
// plausible. The delay line that produces the skew is day 8; until then the
// testbenches apply it by hand, exactly as day 1's and day 2's do.
//
// -----------------------------------------------------------------------------
// Weight load order — unchanged from pe_pair, just longer
// -----------------------------------------------------------------------------
// The DEPTH cells form one 2*DEPTH-deep 8-bit shift register. The first value
// presented travels furthest, so:
//
//     for g = DEPTH-1 down to 0:   present w1[g], then w0[g]
//
// farthest row first, odd column before even. wgt_out carries the chain on to
// the next group below, so day 4 loads four groups through a single port in
// 8*DEPTH cycles by simply continuing to shift.
//
// -----------------------------------------------------------------------------
// Extraction is combinational, on purpose
// -----------------------------------------------------------------------------
// sum_even/sum_odd come straight out of pack_extract, off the last cell's PREG.
// No output register: the group's consumer (column_pair) has an adder tree to
// build and should own the pipeline stage that follows it, rather than
// inheriting a fixed one from here. The path is a 48-bit slice plus one 31-bit
// increment, which is not what will limit 100 MHz.
// -----------------------------------------------------------------------------
`timescale 1ns / 1ps

module cascade_group
  import sa_pkg::*;
#(
  // Cells in this cascade. Must not exceed CASCADE_DEPTH -- see the elaboration
  // check below, and §4 of the architecture notes for the derivation.
  parameter int DEPTH     = CASCADE_DEPTH,
  parameter int K         = K_PACK,
  parameter int OUT_W     = GROUP_W,
  parameter bit CLAMP_WGT = 1'b1
) (
  input  logic                               clk,
  input  logic                               rst_n,
  input  logic                               en,            // datapath advance

  // ---- weight load chain (vertical, top to bottom) --------------------------
  input  logic                               wgt_shift_en,
  input  logic signed [WGT_W-1:0]            wgt_in,
  output logic signed [WGT_W-1:0]            wgt_out,

  // ---- activations (horizontal, one per row) --------------------------------
  // Packed, not unpacked: an unpacked array port written element-by-element in
  // a testbench can fail to propagate under the lint/sim tool named in
  // docs/NOTES-tooling.md, silently and only in that tool.
  input  logic signed [DEPTH-1:0][ACT_W-1:0] act_in,
  output logic signed [DEPTH-1:0][ACT_W-1:0] act_out,

  // ---- extracted partial sums for this group's DEPTH rows -------------------
  output logic signed [OUT_W-1:0]            sum_even,      // w0 field, col 2p
  output logic signed [OUT_W-1:0]            sum_odd        // w1 field, col 2p+1
);

  // Cycles from an act_in[0] being applied to its contribution appearing on
  // sum_even/sum_odd. pe_pair costs 3 and each further cascade stage costs
  // exactly one more. The testbenches count the same number as DEPTH+1 edges
  // after the capture edge, which is the same instant described differently --
  // an easy off-by-one that makes every result look right while being
  // attributed to the neighbouring vector.
  localparam int LATENCY = 3 + (DEPTH - 1);   // = 6 at DEPTH = 4

  // ---------------------------------------------------------------------------
  // The bound that names this module. Checked at elaboration rather than left
  // as a comment, because exceeding it corrupts the odd column only, only for
  // large activations, and never raises anything.
  // ---------------------------------------------------------------------------
  initial begin
    if (DEPTH < 1 || DEPTH > CASCADE_DEPTH)
      $fatal(1, "cascade_group: DEPTH=%0d outside 1..%0d; the %0d-bit low field holds only %0d products of %0d",
             DEPTH, CASCADE_DEPTH, K, CASCADE_DEPTH, PROD_MAX);
  end

  // ---------------------------------------------------------------------------
  // The chain
  // ---------------------------------------------------------------------------
  logic signed [DEPTH:0][WGT_W-1:0]   wch;    // wch[0] = wgt_in
  logic signed [DEPTH:0][DSP_P_W-1:0] casc;   // casc[0] = tied-off head

  assign wch[0] = wgt_in;

  // The head of the cascade is zero and cell 0 always starts a fresh
  // accumulation. Multi-tile accumulation (K > ROWS, day 18) is done on the
  // extracted 32-bit psums downstream, never by re-entering the packed domain:
  // a second pass through the cascade would double the low field's occupancy
  // and break the DEPTH bound above.
  assign casc[0] = '0;

  genvar g;
  generate
    for (g = 0; g < DEPTH; g++) begin : g_cell
      pe_pair #(
        .K         (K),
        .CLAMP_WGT (CLAMP_WGT)
      ) u_pe (
        .clk          (clk),
        .rst_n        (rst_n),
        .en           (en),
        .wgt_shift_en (wgt_shift_en),
        .wgt_in       (wch[g]),
        .wgt_out      (wch[g+1]),
        .act_in       (act_in[g]),
        .act_out      (act_out[g]),
        .acc_first    (g == 0),
        .pcin         (casc[g]),
        .pcout        (casc[g+1])
      );
    end
  endgenerate

  assign wgt_out = wch[DEPTH];

  // ---------------------------------------------------------------------------
  // Separate the two columns. The borrow correction inside pack_extract is what
  // makes this exact for negative low fields; see pack_extract.sv.
  // ---------------------------------------------------------------------------
  pack_extract #(
    .K     (K),
    .OUT_W (OUT_W)
  ) u_ext (
    .p_packed (casc[DEPTH]),
    .p0       (sum_even),
    .p1       (sum_odd)
  );

`ifdef SA_ASSERT
  // Runtime restatement of the elaboration bound. If a future DEPTH change or a
  // weight that dodged the clamp pushes the low field past what K bits hold,
  // the symptom is a wrong sum_odd, so check the magnitude of the field that
  // would have to carry rather than waiting for the wrong answer.
  // `always @(posedge clk)` rather than `always_ff`: simulation-only, and
  // $error is not synthesisable.
  always @(posedge clk) begin
    if (rst_n && en) begin
      assert (sum_even >= -(DEPTH * PROD_MAX) && sum_even <= (DEPTH * PROD_MAX))
        else $error("cascade_group: sum_even=%0d exceeds %0d*%0d; low field has bled into the high field",
                    sum_even, DEPTH, PROD_MAX);
      assert (sum_odd >= -(DEPTH * PROD_MAX) && sum_odd <= (DEPTH * PROD_MAX))
        else $error("cascade_group: sum_odd=%0d exceeds %0d*%0d", sum_odd, DEPTH, PROD_MAX);
    end
  end
`endif

endmodule
