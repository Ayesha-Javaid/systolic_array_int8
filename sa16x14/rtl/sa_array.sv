// -----------------------------------------------------------------------------
// sa_array.sv — the complete ROWS x COLS weight-stationary INT8 array
//
// COL_PAIRS = COLS/2 column_pairs placed side by side. Each column_pair is one
// physical column of DSPs serving two output columns through the packed-weight
// trick, so the array is ROWS x COL_PAIRS = 16 x 7 = 112 DSPs producing
// 224 MACs/cycle.
//
//   act_in[0..15]      col-pair 0      col-pair 1            col-pair 6
//   (staggered)   -->  [16 cells] -->  [16 cells] --> ... --> [16 cells] --> act_out
//                          |               |                      |
//                       sum 0,1         sum 2,3               sum 12,13
//
// This module is deliberately thin: column_pair already owns the cascade bound,
// the group alignment and the adder tree. What day 5 adds is the horizontal
// dimension, and with it exactly two new facts that everything downstream is
// built on. Both are stated here because both are silent when wrong.
//
// -----------------------------------------------------------------------------
// Fact 1 — the horizontal delay is one cycle per COLUMN PAIR, and it is uniform
// -----------------------------------------------------------------------------
// pe_pair hands its activation to the right through the DSP's own BREG/BCOUT:
// exactly one cycle per cell, and a cell serves BOTH of its output columns. So
// column-pair p sees row i's activation p cycles after column-pair 0 does --
// one cycle per pair, not per output column.
//
// Two consequences:
//
//   * The per-row stagger survives the traverse. Column-pair p sees row i at
//     t0 + i + p, so the skew *within* every column-pair is still exactly i
//     cycles per row, which is what the cascade requires. This holds only
//     because the horizontal delay is the same for every row: a single row
//     delayed by two cycles in one column-pair would leave that column-pair
//     summing products from two different activation vectors, with every
//     number still in range. Under SA_ASSERT that uniformity is checked
//     continuously against a shadow chain at the bottom of this file, and
//     tb_sa_array phase 1 measures it per (row, column) for all ROWS*COLS
//     positions by timing a one-hot weight.
//
//   * The output columns come out skewed, by pair:
//
//         LAT(j) = ROWS + 2 + (j/2)        = 18 .. 24 at 16x14
//
//     Removing that skew is day 9's de-skew network, and NOTE that the depth
//     is (COL_PAIRS-1 - j/2) = 6,6,5,5,4,4,3,3,2,2,1,1,0,0 -- seven distinct
//     depths shared pairwise, not the fourteen distinct depths (COLS-1 - j)
//     that a one-DSP-per-output-column array would need. The packing halves
//     the horizontal dimension and the de-skew network with it. This module
//     does not de-skew: it exports the raw, pair-skewed sums and LATENCY_MAX,
//     because inventing a de-skew here would mean two of them later.
//
// -----------------------------------------------------------------------------
// Fact 2 — weight load: one 8-bit port per column-pair, loaded in parallel
// -----------------------------------------------------------------------------
// column_pair left this open. The array takes COL_PAIRS independent 8-bit
// ports driven by one shared wgt_shift_en, so a whole ROWS x COLS tile loads in
// 2*ROWS = 32 cycles through a COL_PAIRS*8 = 56-bit bus.
//
// The alternative -- chaining all seven columns onto one 8-bit port -- would
// take 2*ROWS*COL_PAIRS = 224 cycles. That is longer than the array's own
// pipeline and longer than most tiles spend computing, so it would make the
// load, not the arithmetic, the throughput limit, and day 6's double buffer
// would be hiding a 224-cycle hole instead of a 32-cycle one. 56 bits of
// weight bus is cheap by comparison; it is fed from BRAM, not from the AXI
// stream that carries activations.
//
// The per-column load order is day 2's rule, unchanged: within each port, the
// first byte presented travels furthest, so
//
//     for i = ROWS-1 down to 0:  present w[i][2p+1], then w[i][2p]
//
// farthest row first, odd output column before even, on all COL_PAIRS ports
// simultaneously. Backwards, this transposes the tile and every number still
// looks plausible -- tb_sa_array phase 1 pins all ROWS*COLS positions
// individually so that it cannot be got wrong silently.
//
// wgt_out exports the tail of each chain. Nothing in this design consumes it;
// it exists so the chain depth is directly measurable (phase 8) and so a
// second array could be daisy-chained below this one without touching the
// internals.
//
// -----------------------------------------------------------------------------
// What this module deliberately does not have
// -----------------------------------------------------------------------------
// No stagger network (day 8 -- act_in must arrive already skewed), no output
// de-skew (day 9), no psum input or accumulate control (multi-tile K > ROWS is
// day 18, on the extracted sums, never back through the packed domain), no
// double buffering (day 6 -- there is one weight bank, so a load disturbs
// results in flight and a tile change must be drained with zero activations),
// and no valid signal (latency is a constant, flow control is days 11-13).
// -----------------------------------------------------------------------------
`timescale 1ns / 1ps

module sa_array
  import sa_pkg::*;
#(
  parameter int NROWS     = ROWS,             // = 16, the GEMM tile's K
  parameter int NCOLS      = COLS,            // = 14, must be even
  parameter int GROUPS    = CASCADE_GROUPS,   // = 4 cascade groups per column
  parameter int DEPTH     = CASCADE_DEPTH,    // = 4 cells per group
  parameter int K         = K_PACK,
  parameter int GRP_W     = GROUP_W,
  parameter int OUT_W     = COLSUM_W,
  parameter bit CLAMP_WGT = 1'b1
) (
  input  logic                                clk,
  input  logic                                rst_n,
  input  logic                                en,            // datapath advance

  // ---- weight load: one 8-bit port per column-pair, one shared enable -------
  input  logic                                wgt_shift_en,
  input  logic signed [NCOLS/2-1:0][WGT_W-1:0] wgt_in,
  output logic signed [NCOLS/2-1:0][WGT_W-1:0] wgt_out,

  // ---- activations: one per row, already staggered (row i is i cycles late) -
  input  logic signed [NROWS-1:0][ACT_W-1:0]  act_in,
  output logic signed [NROWS-1:0][ACT_W-1:0]  act_out,

  // ---- column sums, still skewed by column-pair: LAT(j) = NROWS+2 + j/2 -----
  output logic signed [NCOLS-1:0][OUT_W-1:0]  col_sum
);

  localparam int NPAIRS     = NCOLS / 2;              // = 7 DSP columns
  localparam int CHAIN      = 2 * NROWS;              // = 32, per-port depth
  localparam int COL_LAT    = NROWS + 2;              // one column_pair = 18
  localparam int LATENCY_MIN = COL_LAT;               // col 0,1  -> 18
  localparam int LATENCY_MAX = COL_LAT + NPAIRS - 1;  // col 12,13 -> 24
  localparam int N_MACS     = NROWS * NCOLS;          // 224 MACs/cycle
  localparam int N_DSPS     = NROWS * NPAIRS;         // 112 DSP48E1

  // ---------------------------------------------------------------------------
  // Elaboration checks. An odd NCOLS is the interesting one: COLS/2 truncates,
  // so the array would quietly build one output column short rather than fail.
  // ---------------------------------------------------------------------------
  initial begin
    if (NCOLS < 2)
      $fatal(1, "sa_array: NCOLS=%0d must be at least 2", NCOLS);
    if ((NCOLS % 2) != 0)
      $fatal(1, "sa_array: NCOLS=%0d must be even; every DSP carries two packed output columns, so an odd NCOLS would silently drop one",
             NCOLS);
    if (NROWS != GROUPS * DEPTH)
      $fatal(1, "sa_array: NROWS=%0d but GROUPS*DEPTH=%0d; the rows must divide exactly into cascade groups",
             NROWS, GROUPS * DEPTH);
    if (DEPTH > CASCADE_DEPTH)
      $fatal(1, "sa_array: DEPTH=%0d exceeds CASCADE_DEPTH=%0d; the packed low field cannot hold that many products",
             DEPTH, CASCADE_DEPTH);
  end

  // ---------------------------------------------------------------------------
  // The horizontal activation bus. ach[p] is what column-pair p is presented;
  // ach[0] is the array input and ach[NPAIRS] is the array output.
  //
  // Every crossing is a continuous assignment on a single element with genvar
  // indices. That is not a style choice: a whole-dimension select out of a
  // packed multi-dimensional array is not portable across the simulators this
  // design is built under, and a variable-indexed read inside always_comb gets
  // no sensitivity list at all under Icarus 12 (see column_pair.sv).
  // ---------------------------------------------------------------------------
  logic signed [NPAIRS:0][NROWS-1:0][ACT_W-1:0] ach;
  logic signed [NPAIRS-1:0][OUT_W-1:0]          se, so;

  genvar p, r;
  generate
    for (r = 0; r < NROWS; r++) begin : g_ain
      assign ach[0][r]     = act_in[r];
      assign act_out[r]    = ach[NPAIRS][r];
    end

    for (p = 0; p < NPAIRS; p++) begin : g_pair
      // Per-instance buses, so every port connection is a whole variable.
      logic signed [NROWS-1:0][ACT_W-1:0] cp_ai, cp_ao;

      for (r = 0; r < NROWS; r++) begin : g_row
        assign cp_ai[r]      = ach[p][r];
        assign ach[p+1][r]   = cp_ao[r];
      end

      column_pair #(
        .GROUPS    (GROUPS),
        .DEPTH     (DEPTH),
        .K         (K),
        .GRP_W     (GRP_W),
        .OUT_W     (OUT_W),
        .CLAMP_WGT (CLAMP_WGT)
      ) u_cp (
        .clk          (clk),
        .rst_n        (rst_n),
        .en           (en),
        .wgt_shift_en (wgt_shift_en),
        .wgt_in       (wgt_in[p]),
        .wgt_out      (wgt_out[p]),
        .act_in       (cp_ai),
        .act_out      (cp_ao),
        .sum_even     (se[p]),
        .sum_odd      (so[p])
      );

      // w0 -> low packed field -> even output column 2p
      // w1 -> high packed field -> odd  output column 2p+1
      // This is the mapping gemm_tile_hw() in the golden model uses; swapping
      // it transposes nothing and breaks everything.
      assign col_sum[2*p]     = se[p];
      assign col_sum[2*p + 1] = so[p];
    end
  endgenerate

`ifdef SA_ASSERT
  // ---------------------------------------------------------------------------
  // The uniform-horizontal-delay invariant, checked continuously.
  //
  // act_out[i] must be act_in[i] delayed by exactly NPAIRS enabled cycles, the
  // same count for every row. If it is not, some column-pair is combining
  // products from two different activation vectors and every column sum from
  // that pair rightwards is wrong while staying entirely in range.
  //
  // The shadow chain resets to zero and advances only on en, exactly as the
  // pe_pair activation registers do, so the two track from cycle zero and no
  // warm-up window is needed.
  //
  // `always @(posedge clk)` rather than `always_ff`: simulation-only, and
  // $error is not synthesisable.
  // ---------------------------------------------------------------------------
  logic signed [NPAIRS-1:0][NROWS-1:0][ACT_W-1:0] shadow;

  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      shadow <= '0;
    end else if (en) begin
      shadow[0] <= act_in;
      for (int q = 1; q < NPAIRS; q++)
        shadow[q] <= shadow[q-1];
    end
  end

  always @(posedge clk) begin
    if (rst_n && en) begin
      for (int i = 0; i < NROWS; i++)
        assert (act_out[i] === shadow[NPAIRS-1][i])
          else $error("sa_array: act_out[%0d]=%0d, want %0d -- horizontal delay is not a uniform %0d cycles",
                      i, act_out[i], shadow[NPAIRS-1][i], NPAIRS);
    end
  end
`endif

endmodule
