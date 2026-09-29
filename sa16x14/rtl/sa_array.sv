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

  // ---- weight bank swap: one bit per row, staggered exactly like act_in -----
  // swap_in[i] is a one-cycle pulse that must be presented one cycle before the
  // first activation of the new tile appears on act_in[i]. It then rides
  // rightwards through the array at one cycle per column-pair, in the same
  // register stage as the activation, under the same enable. swap_out is the
  // tail of that path: nothing consumes it, it exists so the depth is directly
  // measurable (phase 5 of tb_wgt_dbuf) and so a second array could be chained.
  input  logic        [NROWS-1:0]             swap_in,
  output logic        [NROWS-1:0]             swap_out,

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
  logic        [NPAIRS:0][NROWS-1:0]            sch;   // the swap bus, alongside
  logic signed [NPAIRS-1:0][OUT_W-1:0]          se, so;

  genvar p, r;
  generate
    for (r = 0; r < NROWS; r++) begin : g_ain
      assign ach[0][r]     = act_in[r];
      assign act_out[r]    = ach[NPAIRS][r];
      assign sch[0][r]     = swap_in[r];
      assign swap_out[r]   = sch[NPAIRS][r];
    end

    for (p = 0; p < NPAIRS; p++) begin : g_pair
      // Per-instance buses, so every port connection is a whole variable.
      logic signed [NROWS-1:0][ACT_W-1:0] cp_ai, cp_ao;
      logic        [NROWS-1:0]            cp_si, cp_so;

      for (r = 0; r < NROWS; r++) begin : g_row
        assign cp_ai[r]      = ach[p][r];
        assign ach[p+1][r]   = cp_ao[r];
        assign cp_si[r]      = sch[p][r];
        assign sch[p+1][r]   = cp_so[r];
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
        .swap_in      (cp_si),
        .swap_out     (cp_so),
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
  logic        [NPAIRS-1:0][NROWS-1:0]            sshadow;

  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      shadow  <= '0;
      sshadow <= '0;
    end else if (en) begin
      shadow[0]  <= act_in;
      sshadow[0] <= swap_in;
      for (int q = 1; q < NPAIRS; q++) begin
        shadow[q]  <= shadow[q-1];
        sshadow[q] <= sshadow[q-1];
      end
    end
  end

  always @(posedge clk) begin
    if (rst_n && en) begin
      for (int i = 0; i < NROWS; i++)
        assert (act_out[i] === shadow[NPAIRS-1][i])
          else $error("sa_array: act_out[%0d]=%0d, want %0d -- horizontal delay is not a uniform %0d cycles",
                      i, act_out[i], shadow[NPAIRS-1][i], NPAIRS);

      // The swap pulse must traverse at exactly the same rate as the activation
      // it belongs to. Both are checked against a chain of the same depth, so a
      // swap path that is one stage short or one stage long -- which would put
      // some column-pair on the wrong tile for one vector, in range and wrong --
      // fails here rather than in the arithmetic.
      for (int i = 0; i < NROWS; i++)
        assert (swap_out[i] === sshadow[NPAIRS-1][i])
          else $error("sa_array: swap_out[%0d]=%0b, want %0b -- the swap pulse is not advancing at one cycle per column-pair like the activation",
                      i, swap_out[i], sshadow[NPAIRS-1][i]);
    end
  end

  // ---------------------------------------------------------------------------
  // The shift chain must be idle for the whole swap window.
  //
  // A swap copies the shadow bank as it stood *before* the edge, and one shift
  // displaces the whole 2*NROWS-deep chain by one byte. The staggered swap takes
  // NROWS enabled cycles to reach every row, so a shift anywhere inside that
  // window hands some row a weight from a neighbouring row's position. Every
  // value stays a legal INT8 and every column sum stays in range, so there is no
  // other symptom.
  //
  // The window is SWAP_WIN = NROWS + NPAIRS - 1 cycles measured at this
  // boundary: NROWS of input skew plus NPAIRS-1 of horizontal traverse, because
  // the swap is skewed in both directions while wgt_shift_en is broadcast.
  //
  // swwin counts down from the FIRST pulse of a burst and must NOT be reloaded
  // by the later pulses of the same burst, or a correct back-to-back schedule
  // (next load starting at v0 + NROWS + NPAIRS - 1) would be flagged.
  // ---------------------------------------------------------------------------
  localparam int SWAP_WIN = NROWS + NPAIRS - 1;          // = 22

  logic [$clog2(SWAP_WIN+1)-1:0] swwin;

  always @(posedge clk or negedge rst_n) begin
    if (!rst_n)                        swwin <= '0;
    else if (en) begin
      if ((|swap_in) && swwin == '0)   swwin <= SWAP_WIN - 1;
      else if (swwin != '0)            swwin <= swwin - 1'b1;
    end
  end

  always @(posedge clk) begin
    if (rst_n && en && wgt_shift_en && ((|swap_in) || swwin != '0))
      $error("sa_array: wgt_shift_en asserted inside the %0d-cycle swap window; the chain has moved under a cell that has not swapped yet",
             SWAP_WIN);
  end
`endif

endmodule
