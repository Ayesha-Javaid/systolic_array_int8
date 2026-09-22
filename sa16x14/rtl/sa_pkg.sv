// -----------------------------------------------------------------------------
// sa_pkg.sv — shared parameters for the 16x14 INT8 weight-stationary array
//
// Every magic number in this design traces back to one of two constraints:
//   * the DSP48E1 A-port is 25 bits  -> sets K_PACK (docs/ARCHITECTURE.md §3.3)
//   * the packed low field is K_PACK -> sets CASCADE_DEPTH (§4)
// If you change one, re-derive the other. The testbenches will catch you if
// you do not.
// -----------------------------------------------------------------------------
`ifndef SA_PKG_SV
`define SA_PKG_SV
`timescale 1ns / 1ps

package sa_pkg;

  // ---- array geometry -------------------------------------------------------
  parameter int ROWS        = 16;  // accumulation depth (K of the GEMM tile)
  parameter int COLS        = 14;  // output channels per tile; even, so every
                                   // DSP carries two fully-used MACs
  parameter int COL_PAIRS   = COLS / 2;  // = 7 DSPs per row

  // ---- datatypes ------------------------------------------------------------
  parameter int ACT_W       = 8;
  parameter int WGT_W       = 8;
  parameter int PSUM_W      = 32;  // exported accumulator

  // Weights are clamped to [-127, 127]: w = -128 would overflow the packed
  // 25-bit A port (§3.5). This is also what symmetric INT8 quantisation does,
  // so it costs nothing in accuracy.
  parameter int WGT_MIN     = -127;
  parameter int WGT_MAX     =  127;

  // ---- DSP packing ----------------------------------------------------------
  parameter int K_PACK      = 17;  // bit offset of the high product field.
                                   // max is 25-8 = 17; we take the max because
                                   // every bit of headroom buys accumulator
                                   // depth in the low field.
  parameter int DSP_A_W     = 25;
  parameter int DSP_B_W     = 18;
  parameter int DSP_P_W     = 48;

  // Largest magnitude of a single INT8 product, given the weight clamp:
  //   127 * 128 = 16256
  parameter int PROD_MAX    = WGT_MAX * 128;  // 16256

  // Deepest DSP cascade that keeps the low field from bleeding into the high
  // field:  N * PROD_MAX <= 2^(K_PACK-1) - 1 = 65535  ->  N = 4
  parameter int CASCADE_DEPTH = 4;
  parameter int CASCADE_GROUPS = ROWS / CASCADE_DEPTH;  // = 4

  // Width needed to hold an extracted field after a full cascade group.
  //   |4 * 16256| = 65024 -> 18 bits signed
  parameter int GROUP_W     = 18;

  // Width needed for a full 16-deep column sum.
  //   |16 * 16256| = 260096 -> 20 bits signed
  parameter int COLSUM_W    = 20;

  // ---- packing function -----------------------------------------------------
  // Single source of truth for the A-port packing. Both the RTL and the
  // testbenches call this, so there is no way for the design and its test to
  // drift apart on the one formula that matters.
  //
  //   A = w1 * 2^K_PACK + w0,  as a signed 25-bit value
  //
  // Requires w0, w1 in [-127, 127]. At w1 = -128 the result is -16777344,
  // which is 128 below the 25-bit signed floor, and wraps.
  function automatic logic signed [DSP_A_W-1:0] pack_weights(
      input logic signed [WGT_W-1:0] w1,
      input logic signed [WGT_W-1:0] w0);
    logic signed [DSP_A_W-1:0] w1_ext, w0_ext;
    w1_ext = DSP_A_W'(w1);
    w0_ext = DSP_A_W'(w0);
    return (w1_ext <<< K_PACK) + w0_ext;
  endfunction

endpackage

`endif
