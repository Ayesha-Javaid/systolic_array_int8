`timescale 1ns / 1ps

module column_pair
  import sa_pkg::*;
#(
  parameter int GROUPS    = CASCADE_GROUPS,   // = 4 cascade groups
  parameter int DEPTH     = CASCADE_DEPTH,    // = 4 cells per group
  parameter int K         = K_PACK,
  parameter int GRP_W     = GROUP_W,          // width out of one group
  parameter int OUT_W     = COLSUM_W,         // width of a full column sum
  parameter bit CLAMP_WGT = 1'b1
) (
  input  logic                              clk,
  input  logic                              rst_n,
  input  logic                              en,            // datapath advance

  // ---- weight load chain (vertical, top to bottom, through all groups) ------
  input  logic                              wgt_shift_en,
  input  logic signed [WGT_W-1:0]           wgt_in,
  output logic signed [WGT_W-1:0]           wgt_out,

  // ---- activations (horizontal, one per row, already staggered) -------------
  input  logic signed [GROUPS*DEPTH-1:0][ACT_W-1:0] act_in,
  output logic signed [GROUPS*DEPTH-1:0][ACT_W-1:0] act_out,

  // ---- weight bank swap, one bit per row, already staggered like act_in -----
  // No alignment delay is applied to these, deliberately: the swap acts on the
  // weights at the cell *input* side, where the only skew that exists is the
  // activation skew already carried on act_in. The ALIGN() delays below are on
  // the group *outputs* and have nothing to say about it. Putting the swap
  // through them would delay the bank change by 12/8/4/0 cycles per group and
  // leave the four groups of one column on two different tiles.
  input  logic        [GROUPS*DEPTH-1:0]            swap_in,
  output logic        [GROUPS*DEPTH-1:0]            swap_out,

  // ---- the two complete column sums ----------------------------------------
  output logic signed [OUT_W-1:0]           sum_even,      // w0 field, col 2p
  output logic signed [OUT_W-1:0]           sum_odd        // w1 field, col 2p+1
);

  localparam int NROWS  = GROUPS * DEPTH;         // = 16
  localparam int LEVELS = $clog2(GROUPS);         // adder-tree ranks = 2

  // Cycles from a row-0 activation being applied to the corresponding column
  // sum appearing on sum_even/sum_odd:
  //   group 3's own result is DEPTH+1 cycles after its row 12 is applied,
  //   row 12 is applied (GROUPS-1)*DEPTH cycles after row 0, and the output
  //   register adds one. Groups 0..2 are delayed to match. Total = NROWS + 2.
  localparam int LATENCY = NROWS + 2;             // = 18 at 16 rows

  // Largest magnitude a complete column sum can reach, and the narrowest
  // signed width that holds it.
  localparam int COL_MAX   = NROWS * PROD_MAX;    // 16 * 16256 = 260096
  localparam int OUT_W_MIN = $clog2(COL_MAX + 1) + 1;   // = 19

  // ---------------------------------------------------------------------------
  // Elaboration checks. Every one of these is a silent wrap or a silent
  // mis-sum if it is violated at run time, so none of them is left as a
  // comment.
  // ---------------------------------------------------------------------------
  initial begin
    if (GROUPS < 1)
      $fatal(1, "column_pair: GROUPS=%0d must be at least 1", GROUPS);
    if ((GROUPS & (GROUPS - 1)) != 0)
      $fatal(1, "column_pair: GROUPS=%0d must be a power of two; the adder tree pairs its inputs",
             GROUPS);
    if (DEPTH < 1 || DEPTH > CASCADE_DEPTH)
      $fatal(1, "column_pair: DEPTH=%0d outside 1..%0d", DEPTH, CASCADE_DEPTH);
    if (OUT_W < OUT_W_MIN)
      $fatal(1, "column_pair: OUT_W=%0d cannot hold a column sum of +/-%0d; needs %0d bits signed",
             OUT_W, COL_MAX, OUT_W_MIN);
    if (OUT_W < GRP_W + LEVELS)
      $fatal(1, "column_pair: OUT_W=%0d is narrower than GRP_W+%0d=%0d, so the adder tree would wrap",
             OUT_W, LEVELS, GRP_W + LEVELS);
  end

  // ---------------------------------------------------------------------------
  // The groups
  // ---------------------------------------------------------------------------
  logic signed [GROUPS:0][WGT_W-1:0]                wch;   // wch[0] = wgt_in
  logic signed [GROUPS-1:0][DEPTH-1:0][ACT_W-1:0]   ga_in, ga_out;
  logic        [GROUPS-1:0][DEPTH-1:0]              gs_in, gs_out;
  logic signed [GROUPS-1:0][GRP_W-1:0]              ge, go;   // raw group sums

  assign wch[0] = wgt_in;

  genvar g, r;
  generate
    for (g = 0; g < GROUPS; g++) begin : g_grp
      // Slice this group's DEPTH rows out of the column's activation bus with
      // continuous assignments, one row at a time. A part-select across the
      // outer dimension of a packed array is not portable across the three
      // simulators this design is built under; per-element drivers are.
      for (r = 0; r < DEPTH; r++) begin : g_row
        assign ga_in[g][r]            = act_in[g*DEPTH + r];
        assign act_out[g*DEPTH + r]   = ga_out[g][r];
        assign gs_in[g][r]            = swap_in[g*DEPTH + r];
        assign swap_out[g*DEPTH + r]  = gs_out[g][r];
      end

      cascade_group #(
        .DEPTH     (DEPTH),
        .K         (K),
        .OUT_W     (GRP_W),
        .CLAMP_WGT (CLAMP_WGT)
      ) u_grp (
        .clk          (clk),
        .rst_n        (rst_n),
        .en           (en),
        .wgt_shift_en (wgt_shift_en),
        .wgt_in       (wch[g]),
        .wgt_out      (wch[g+1]),
        .act_in       (ga_in[g]),
        .act_out      (ga_out[g]),
        .swap_in      (gs_in[g]),
        .swap_out     (gs_out[g]),
        .sum_even     (ge[g]),
        .sum_odd      (go[g])
      );
    end
  endgenerate

  assign wgt_out = wch[GROUPS];

  // ---------------------------------------------------------------------------
  // Alignment: put all GROUPS results onto the same activation vector.
  //
  // Group g is ALIGN(g) = (GROUPS-1-g)*DEPTH cycles early relative to the
  // bottom group, so it is delayed by exactly that much. Group GROUPS-1 needs
  // no delay and must get a WIRE, not a register -- a register there would be an
  // extra pipeline stage on one input of the tree only, which is the same bug
  // in a different place. That case is not special-cased here any more:
  // delay_line's DEPTH = 0 is combinational by contract, and tb_delay_line
  // phase 1 measures it as arriving at index 0 rather than leaving it to a
  // reader of this file to notice.
  // ---------------------------------------------------------------------------
  logic signed [GROUPS-1:0][GRP_W-1:0] ae, ao;     // aligned group sums

  generate
    for (g = 0; g < GROUPS; g++) begin : g_align
      localparam int AD = (GROUPS - 1 - g) * DEPTH;

      // delay_line's ports are unsigned; these are bit-for-bit copies at the
      // same width, and the sign is reapplied at level 0 of the tree below.
      logic [GRP_W-1:0] dle_o, dlo_o;

      delay_line #(.W(GRP_W), .DEPTH(AD)) u_dl_e (
        .clk  (clk),
        .en   (en),
        .din  (ge[g]),
        .dout (dle_o)
      );

      delay_line #(.W(GRP_W), .DEPTH(AD)) u_dl_o (
        .clk  (clk),
        .en   (en),
        .din  (go[g]),
        .dout (dlo_o)
      );

      assign ae[g] = $signed(dle_o);
      assign ao[g] = $signed(dlo_o);
    end
  endgenerate

  // ---------------------------------------------------------------------------
  // Fabric adder tree, carried at OUT_W from level 0 upwards.
  //
  // Level 0 is the only place a value is widened, and therefore the only place
  // the sign can be lost. 1800-2017 §11.8.1 makes the result of a select
  // unsigned even when the array selected from is declared signed, which would
  // zero-extend every group sum and turn every negative column sum into a
  // large positive one. Icarus 12 and the lint/sim tool of
  // docs/NOTES-tooling.md both sign-extend it anyway, so removing the
  // $signed() here changes nothing that either of them can see -- it is
  // written explicitly because the standard, and possibly the vendor
  // simulator, say otherwise, not because a test caught it. See the
  // documented-equivalent-mutant note in the status doc.
  //
  // From level 1 upwards every operand is already OUT_W wide, and
  // two's-complement addition at a fixed width is the same operation signed or
  // unsigned, so no further care is needed.
  //
  // Every node is a continuous assignment with genvar indices, and the node
  // array is UNPACKED. Neither is a style choice; each fixes a different
  // simulator:
  //
  //   * written as an `always_comb` containing a for loop, the reads are
  //     `tre[g-1][2*m]` with a variable m, and Icarus 12 then infers no
  //     sensitivity list at all. It warns "always_comb process has no
  //     sensitivities", evaluates the block once at time zero when its inputs
  //     are still X, and never runs it again -- the column sums stay X for the
  //     entire simulation.
  //   * the lint pass does its combinational-loop detection at whole-signal
  //     granularity, so level l reading level l-1 looks like `tre` depending
  //     on itself and it refuses to build with UNOPTFLAT "circular
  //     combinational logic" -- for a tree that is acyclic by construction.
  //     Making the level dimension unpacked is not enough on its own; the
  //     split_var metacomment is, and it is exactly what that metacomment is
  //     for. Icarus and Vivado both read it as an ordinary comment, so it
  //     costs nothing outside that one tool.
  // ---------------------------------------------------------------------------
  logic signed [OUT_W-1:0] tre [0:LEVELS][0:GROUPS-1] /* verilator split_var */;
  logic signed [OUT_W-1:0] tro [0:LEVELS][0:GROUPS-1] /* verilator split_var */;

  generate
    for (r = 0; r < GROUPS; r++) begin : g_lvl0
      assign tre[0][r] = OUT_W'($signed(ae[r]));
      assign tro[0][r] = OUT_W'($signed(ao[r]));
    end

    for (g = 1; g <= LEVELS; g++) begin : g_tree
      for (r = 0; r < (GROUPS >> g); r++) begin : g_add
        assign tre[g][r] = tre[g-1][2*r] + tre[g-1][2*r + 1];
        assign tro[g][r] = tro[g-1][2*r] + tro[g-1][2*r + 1];
      end
      // Slots this level no longer uses are tied off rather than left
      // undriven, so an X can never reach the output register by accident.
      for (r = (GROUPS >> g); r < GROUPS; r++) begin : g_tie
        assign tre[g][r] = '0;
        assign tro[g][r] = '0;
      end
    end
  endgenerate

  // ---------------------------------------------------------------------------
  // The one pipeline stage this module owns. cascade_group deliberately leaves
  // its extraction combinational so that the register lands here, after the
  // tree, where it breaks the longest path: extract -> align -> LEVELS adds.
  // ---------------------------------------------------------------------------
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      sum_even <= '0;
      sum_odd  <= '0;
    end else if (en) begin
      sum_even <= tre[LEVELS][0];
      sum_odd  <= tro[LEVELS][0];
    end
  end

`ifdef SA_ASSERT
  // Runtime restatement of the COLSUM_W bound. A column sum outside
  // +/-ROWS*PROD_MAX means either a weight dodged the clamp or the tree is
  // summing misaligned vectors; both are silent otherwise.
  // `always @(posedge clk)` rather than `always_ff`: simulation-only, and
  // $error is not synthesisable.
  always @(posedge clk) begin
    if (rst_n && en) begin
      assert (sum_even >= -COL_MAX && sum_even <= COL_MAX)
        else $error("column_pair: sum_even=%0d exceeds %0d*%0d", sum_even, NROWS, PROD_MAX);
      assert (sum_odd >= -COL_MAX && sum_odd <= COL_MAX)
        else $error("column_pair: sum_odd=%0d exceeds %0d*%0d", sum_odd, NROWS, PROD_MAX);
    end
  end
`endif

endmodule
