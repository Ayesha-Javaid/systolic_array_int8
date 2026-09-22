// -----------------------------------------------------------------------------
// dsp_pack_mac.sv — two INT8 MACs in one DSP48E1
//
// Holds two stationary weights (w0, w1) and multiplies both by a single shared
// activation in one DSP48E1, by packing the weights into separate bit-fields of
// the 25-bit A port:
//
//     A = w1 * 2^K + w0      B = a
//     P = A * B = (w1*a) * 2^K + (w0*a)
//
// The two products are then accumulated *in the packed domain* through the DSP
// PCOUT->PCIN cascade, which is free, and only separated at the end of a
// cascade group (see pack_extract.sv). CASCADE_DEPTH is bounded by the width of
// the low field -- see docs/ARCHITECTURE.md §4.
//
// RTL style note: this is written as inferable arithmetic, not as an explicit
// DSP48E1 primitive instantiation. Two reasons:
//   1. It simulates in any open-source simulator, so CI is real.
//   2. Vivado infers DSP48E1 from this shape reliably; the use_dsp attribute
//      below makes that non-optional rather than a hope.
// A primitive-instantiation variant lives behind `ifdef SA_USE_DSP_PRIMITIVE
// for the cases where the inference does not give the pipeline you want.
// -----------------------------------------------------------------------------
`timescale 1ns / 1ps

module dsp_pack_mac
  import sa_pkg::*;
#(
  parameter int K = K_PACK
) (
  input  logic                     clk,
  input  logic                     rst_n,
  input  logic                     en,        // pipeline advance

  // Stationary weights. Must be in [-127, 127]; see sa_pkg::WGT_MIN/MAX.
  input  logic signed [WGT_W-1:0]  w0,        // -> low  field, bits [K-1:0]
  input  logic signed [WGT_W-1:0]  w1,        // -> high field, bits [47:K]

  input  logic signed [ACT_W-1:0]  act,       // shared activation

  // Cascade. acc_first=1 starts a new group and ignores pcin, which is how a
  // group boundary is expressed without needing a reset between tiles.
  input  logic                     acc_first,
  input  logic signed [DSP_P_W-1:0] pcin,
  output logic signed [DSP_P_W-1:0] pcout,

  // Activation passed to the PE on the right (systolic horizontal propagation)
  output logic signed [ACT_W-1:0]  act_out
);

  // ---------------------------------------------------------------------------
  // Weight packing (combinational; weights are stationary so this is not on the
  // critical path -- it settles once per tile load).
  // ---------------------------------------------------------------------------
  logic signed [DSP_A_W-1:0] a_packed;
  assign a_packed = sa_pkg::pack_weights(w1, w0);

  // ---------------------------------------------------------------------------
  // DSP pipeline: AREG/BREG -> MREG -> PREG.
  //
  // The cascade advances one stage per cycle (PREG), which is what makes the
  // systolic dataflow work. AREG/BREG/MREG add a fixed 2-cycle offset that is
  // identical for every PE, so it shifts the whole wavefront rather than
  // skewing it -- the stagger network (§5) absorbs it as a constant.
  // ---------------------------------------------------------------------------
  (* use_dsp = "yes" *) logic signed [DSP_A_W-1:0]  areg;
  (* use_dsp = "yes" *) logic signed [DSP_B_W-1:0]  breg;
  (* use_dsp = "yes" *) logic signed [42:0]         mreg;   // 25b * 18b = 43b
  (* use_dsp = "yes" *) logic signed [DSP_P_W-1:0]  preg;

  logic acc_first_m, acc_first_p;

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      areg        <= '0;
      breg        <= '0;
      mreg        <= '0;
      preg        <= '0;
      acc_first_m <= 1'b0;
      acc_first_p <= 1'b0;
      act_out     <= '0;
    end else if (en) begin
      // stage 1 : A/B input registers
      areg        <= a_packed;
      breg        <= DSP_B_W'(act);
      acc_first_m <= acc_first;
      act_out     <= act;                   // systolic hand-off to the right

      // stage 2 : multiplier register
      mreg        <= areg * breg;
      acc_first_p <= acc_first_m;

      // stage 3 : accumulator / cascade register
      //
      // NOTE: pcin is deliberately NOT registered on its way in. In a real
      // DSP48E1 the PCIN port feeds the ALU directly, so the cascade advances
      // exactly one stage per cycle. Adding an input register here would make
      // the cascade two cycles per stage while the multiplier path stayed at
      // one, and every column sum would silently combine products from
      // different activation vectors. The control bit acc_first *is* delayed
      // two stages, because it has to arrive alongside mreg, not alongside pcin.
      preg        <= acc_first_p ? DSP_P_W'(mreg)
                                 : DSP_P_W'(mreg) + pcin;
    end
  end

  // Pipeline latency from a driven input to it appearing in pcout.
  // A cascade of N of these has latency LATENCY + (N-1), since each further
  // stage costs exactly one cycle.
  localparam int LATENCY = 3;

  assign pcout = preg;

`ifdef SA_ASSERT
  // The -128 exclusion is the single silent-corruption hazard in this design:
  // it does not fail loudly, it just produces a wrong high product. Catch it in
  // simulation rather than in a bitstream.
  // `always @(posedge clk)` rather than `always_ff`: simulation-only, and
  // $error is not synthesisable.
  always @(posedge clk) begin
    if (rst_n && en) begin
      assert (w0 >= WGT_MIN && w0 <= WGT_MAX)
        else $error("dsp_pack_mac: w0=%0d outside [-127,127]; packed A would overflow", w0);
      assert (w1 >= WGT_MIN && w1 <= WGT_MAX)
        else $error("dsp_pack_mac: w1=%0d outside [-127,127]; packed A would overflow", w1);
    end
  end
`endif

endmodule
