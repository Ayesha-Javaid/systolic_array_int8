// -----------------------------------------------------------------------------
// tb_column_pair.sv — self-checking testbench for column_pair
//
// column_pair adds three things to what day 3 already verified, and the first
// of them is the one that can destroy every result in the array without
// producing a single implausible number:
//
//   1. the alignment of the GROUPS cascade groups against each other. Group g
//      is g*DEPTH cycles ahead of the bottom group because the input stagger
//      puts it there; adding the four raw group outputs adds four different
//      activation vectors. Phases 3a and 3b are directed at exactly this: a
//      single-vector pulse, where a misaligned group smears one answer across
//      several cycles, and a per-vector ramp, where a misaligned group shows up
//      as a constant arithmetic offset rather than as noise.
//   2. the adder tree and the width it is carried at. A dropped group, a
//      doubled group, or a group zero-extended instead of sign-extended are all
//      caught by the streaming phase; the COLSUM_W corner phase additionally
//      drives the sum to exactly +/-ROWS*PROD_MAX and asserts that the corner
//      was actually reached, so it cannot quietly stop being tested.
//   3. the weight chain now running 2*ROWS = 32 cells through one 8-bit port,
//      across four group boundaries. Phase 8 measures that depth directly,
//      because day 5 stacks this module and inherits the number.
//
// Everything else here is the day-2/day-3 battery re-run at 16 rows: one-hot
// placement over all 32 chain positions, randomised streaming with reloads, two
// different en=0 stalls, wgt_shift_en gating, and a two-sample check of the
// horizontal hand-off per row.
//
// House conventions that are not style preferences (docs/NOTES-tooling.md):
//   * packed arrays, and every port-connected variable assigned whole once
//     before any element-wise write
//   * `initial forever` for the clock, not `always`
//   * module-scope declarations and fixed-size arrays, no dynamic-array task
//     arguments
//   * each phase prints the failures *it* caused, not the running total. A
//     failure still bleeds up to LAT checks into the next phase's window,
//     because the scoreboard is that deep.
// -----------------------------------------------------------------------------
`timescale 1ns / 1ps

module tb_column_pair;
  import sa_pkg::*;

  localparam int NG       = CASCADE_GROUPS;        // groups per column = 4
  localparam int ND       = CASCADE_DEPTH;         // cells per group   = 4
  localparam int NR       = NG * ND;               // rows              = 16
  localparam int NW       = 2 * NR;                // weight chain      = 32
  localparam int LAT      = NR + 2;                // issue -> result   = 18
  localparam int TILES    = 20;
  localparam int PER_TILE = 60;

  // A full column sum must reach exactly this, at least once, in phase 4.
  localparam int BOUND    = NR * PROD_MAX;         // 260096

  // ---- clock / reset --------------------------------------------------------
  logic clk   = 1'b0;
  logic rst_n = 1'b0;
  logic en    = 1'b1;
  initial forever #5 clk = ~clk;             // 100 MHz. `initial forever` and
                                             // not `always`, or the lint pass
                                             // calls this sequential logic and
                                             // fires BLKSEQ.

  // ---- DUT ------------------------------------------------------------------
  logic                            wsh;      // wgt_shift_en
  logic signed [WGT_W-1:0]         wser;     // serial weight input
  logic signed [NR-1:0][ACT_W-1:0] act;      // per-row activation, staggered
  logic signed [NR-1:0][ACT_W-1:0] act_o;
  logic signed [WGT_W-1:0]         wgt_o;
  logic signed [COLSUM_W-1:0]      s_even, s_odd;

  column_pair #(
    .GROUPS    (NG),
    .DEPTH     (ND),
    .K         (K_PACK),
    .GRP_W     (GROUP_W),
    .OUT_W     (COLSUM_W),
    .CLAMP_WGT (1'b1)
  ) dut (
    .clk          (clk),
    .rst_n        (rst_n),
    .en           (en),
    .wgt_shift_en (wsh),
    .wgt_in       (wser),
    .wgt_out      (wgt_o),
    .act_in       (act),
    .act_out      (act_o),
    .sum_even     (s_even),
    .sum_odd      (s_odd)
  );

  // ---- skew history: ahist[d][i] is row i's activation for the vector -------
  // ---- issued d cycles ago. Row i is driven from ahist[i][i], which is the --
  // ---- global stagger the array will get from day 8's network. -------------
  integer ahist [0:NR-1][0:NR-1];

  // ---- scoreboard -----------------------------------------------------------
  integer expE_q [0:LAT];
  integer expO_q [0:LAT];
  bit     vld_q  [0:LAT];

  // ---- weights --------------------------------------------------------------
  // W0v/W1v are what the host presents; E0v/E1v are what the hardware holds
  // after the clamp, and the expectation is built from those.
  integer W0v [0:NR-1];
  integer W1v [0:NR-1];
  integer E0v [0:NR-1];
  integer E1v [0:NR-1];

  integer a_vec    [0:NR-1];
  integer prev_act [0:NR-1];
  integer wq       [0:NW-1];                 // model of the weight chain

  integer n_checked, n_failed, i, d, s, v, tile, q;
  integer c_before, f_before;
  integer eE, eO;
  integer max_abs_e, max_abs_o;
  integer sv0, sv1, av, hold_e, hold_o;
  integer ramp;

  // ---------------------------------------------------------------------------
  // Helpers
  // ---------------------------------------------------------------------------
  function automatic integer clampw(input integer w);
    begin
      if (w < WGT_MIN)      clampw = WGT_MIN;
      else if (w > WGT_MAX) clampw = WGT_MAX;
      else                  clampw = w;
    end
  endfunction

  // Expectation for whatever is currently in a_vec, from the weights the cells
  // are supposed to be holding. This is the plain 16-term dot product -- the
  // golden model has already proved that the hardware's group decomposition
  // equals it, so the testbench checks against the mathematics rather than
  // against a re-implementation of the decomposition it is testing.
  task automatic set_expect;
    begin
      eE = 0;
      eO = 0;
      for (i = 0; i < NR; i = i + 1) begin
        eE = eE + E0v[i] * a_vec[i];
        eO = eO + E1v[i] * a_vec[i];
      end
    end
  endtask

  // Issue one vector: push its per-row activations into the skew history, push
  // its expectation into the scoreboard, advance one clock, then check the
  // result arriving LAT edges after this vector's capture edge.
  task automatic issue(input bit valid);
    begin
      for (s = NR-1; s > 0; s = s - 1)
        for (i = 0; i < NR; i = i + 1)
          ahist[s][i] = ahist[s-1][i];
      for (i = 0; i < NR; i = i + 1)
        ahist[0][i] = a_vec[i];

      for (i = 0; i < NR; i = i + 1)
        act[i] = ACT_W'(ahist[i][i]);

      for (s = LAT; s > 0; s = s - 1) begin
        expE_q[s] = expE_q[s-1];
        expO_q[s] = expO_q[s-1];
        vld_q[s]  = vld_q[s-1];
      end
      expE_q[0] = eE;
      expO_q[0] = eO;
      vld_q[0]  = valid;

      @(posedge clk);
      #1;

      if (vld_q[LAT]) begin
        n_checked = n_checked + 1;
        if (expE_q[LAT] >  max_abs_e) max_abs_e = expE_q[LAT];
        if (-expE_q[LAT] > max_abs_e) max_abs_e = -expE_q[LAT];
        if (expO_q[LAT] >  max_abs_o) max_abs_o = expO_q[LAT];
        if (-expO_q[LAT] > max_abs_o) max_abs_o = -expO_q[LAT];
        if (s_even !== COLSUM_W'(expE_q[LAT]) || s_odd !== COLSUM_W'(expO_q[LAT])) begin
          n_failed = n_failed + 1;
          if (n_failed - f_before <= 8)
            $display("  FAIL: got odd=%0d even=%0d, want odd=%0d even=%0d",
                     s_odd, s_even, expO_q[LAT], expE_q[LAT]);
        end
      end
    end
  endtask

  // Zero activations for long enough that nothing in flight can straddle a
  // weight change. The column is LAT deep, so this is longer than day 3's.
  task automatic drain;
    begin
      for (i = 0; i < NR; i = i + 1) a_vec[i] = 0;
      eE = 0;
      eO = 0;
      for (d = 0; d < LAT + NR + 2; d = d + 1) issue(1'b1);
    end
  endtask

  // Flush the pipeline with zeros without reloading anything.
  task automatic flush;
    begin
      for (d = 0; d < LAT + NR; d = d + 1) begin
        for (i = 0; i < NR; i = i + 1) a_vec[i] = 0;
        eE = 0; eO = 0;
        issue(1'b1);
      end
    end
  endtask

  // Shift 2*NR weights in. The first value presented travels furthest -- across
  // four group boundaries now -- so the chain fills from the bottom row
  // backwards, odd column before even:
  //
  //     W1v[NR-1], W0v[NR-1], ... W1v[0], W0v[0]
  //
  // E0v/E1v record what the hardware clamp will actually store.
  task automatic load_weights;
    integer li;
    begin
      for (li = 0; li < NR; li = li + 1) begin
        E0v[li] = clampw(W0v[li]);
        E1v[li] = clampw(W1v[li]);
      end
      wsh = 1'b1;
      for (li = NR-1; li >= 0; li = li - 1) begin
        wser = WGT_W'(W1v[li]);
        @(posedge clk);
        #1;
        wser = WGT_W'(W0v[li]);
        @(posedge clk);
        #1;
      end
      wsh  = 1'b0;
      wser = '0;
    end
  endtask

  // ---------------------------------------------------------------------------
  // Stimulus
  // ---------------------------------------------------------------------------
  initial begin
    n_checked = 0;
    n_failed  = 0;
    c_before  = 0;
    f_before  = 0;
    eE = 0; eO = 0;
    max_abs_e = 0; max_abs_o = 0;
    ramp = 0;
    for (i = 0; i <= LAT; i = i + 1) begin
      expE_q[i] = 0; expO_q[i] = 0; vld_q[i] = 1'b0;
    end

    // Whole-variable initialisation, and it has to stay whole-variable: an
    // element-wise write to a port-connected variable that has never been
    // assigned as a whole does not reach the instance under the sim named in
    // docs/NOTES-tooling.md, while Icarus simulates it correctly.
    act  = '0;
    wser = '0;
    wsh  = 1'b0;
    for (i = 0; i < NR; i = i + 1) begin
      a_vec[i]    = 0;
      prev_act[i] = 0;
      W0v[i] = 0; W1v[i] = 0; E0v[i] = 0; E1v[i] = 0;
      for (d = 0; d < NR; d = d + 1) ahist[d][i] = 0;
    end
    for (i = 0; i < NW; i = i + 1) wq[i] = 0;

    $display("=========================================================");
    $display(" tb_column_pair : %0d groups x %0d cells = %0d rows", NG, ND, NR);
    $display("   chain %0d deep, issue->result %0d cycles, |colsum| <= %0d",
             NW, LAT, BOUND);
    $display("=========================================================");

    repeat (4) @(posedge clk);
    rst_n = 1'b1;
    repeat (2) @(posedge clk);
    #1;

    // =========================================================================
    // Phase 1: one-hot placement over all 2*NR chain positions.
    //
    // A single weight of 1 at one position, everything else zero. With distinct
    // per-row activations sum_even reads back exactly that row's activation if
    // the 1 landed in a w0, and sum_odd does if it landed in a w1. Every
    // position is pinned individually, so a reversed load order (which
    // transposes the tile and is otherwise silent), a w0/w1 swap, and a chain
    // that loses or duplicates a cell at a group boundary all die here.
    // =========================================================================
    c_before = n_checked;
    f_before = n_failed;
    for (q = 0; q < NW; q = q + 1) begin
      drain();
      for (i = 0; i < NR; i = i + 1) begin
        W0v[i] = 0;
        W1v[i] = 0;
      end
      if (q < NR) W0v[q]      = 1;
      else        W1v[q - NR] = 1;
      load_weights();

      for (v = 0; v < 8; v = v + 1) begin
        for (i = 0; i < NR; i = i + 1)
          a_vec[i] = $urandom_range(0, 255) - 128;
        set_expect();
        issue(1'b1);
      end
      flush();
    end
    $display("[1] one-hot placement    : %0d checks, %0d fail",
             n_checked - c_before, n_failed - f_before);

    // =========================================================================
    // Phase 2: randomised streaming with chain reloads between tiles. This is
    // the steady state: a new 16-term dot product issued every cycle.
    // =========================================================================
    c_before = n_checked;
    f_before = n_failed;
    for (tile = 0; tile < TILES; tile = tile + 1) begin
      drain();
      for (i = 0; i < NR; i = i + 1) begin
        W0v[i] = $urandom_range(0, 254) - 127;
        W1v[i] = $urandom_range(0, 254) - 127;
      end
      load_weights();

      for (v = 0; v < PER_TILE; v = v + 1) begin
        for (i = 0; i < NR; i = i + 1)
          a_vec[i] = (v % 3 == 0) ? -128 : $urandom_range(0, 255) - 128;
        set_expect();
        issue(1'b1);
      end
      flush();
    end
    $display("[2] streaming + reload   : %0d checks, %0d fail",
             n_checked - c_before, n_failed - f_before);

    // =========================================================================
    // Phase 3a: group alignment, by pulse.
    //
    // Weights are w0[i] = i+1, so the four groups have distinct row sums
    // (10, 26, 42, 58) and the whole column sums to 136. Exactly one activation
    // vector of all-ones is issued into an otherwise empty pipeline.
    //
    // Correctly aligned, that produces 136 on exactly one cycle and zero on
    // every other. A group delayed by the wrong number of cycles smears the
    // answer out into several partial sums drawn from that group's own row
    // subset -- so the failure prints as 10, 26, 42 or 58 appearing where a 0 or
    // a 136 belongs, which names the offending group directly.
    // =========================================================================
    c_before = n_checked;
    f_before = n_failed;
    drain();
    for (i = 0; i < NR; i = i + 1) begin
      W0v[i] =  (i + 1);
      W1v[i] = -(i + 1);
    end
    load_weights();

    for (v = 0; v < 6; v = v + 1) begin
      for (i = 0; i < NR; i = i + 1) a_vec[i] = (v == 2) ? 1 : 0;
      set_expect();
      issue(1'b1);
      // a long gap, so the pulse cannot be confused with its neighbours
      for (i = 0; i < NR; i = i + 1) a_vec[i] = 0;
      eE = 0; eO = 0;
      for (d = 0; d < LAT + NR; d = d + 1) issue(1'b1);
    end
    $display("[3a] alignment pulse     : %0d checks, %0d fail",
             n_checked - c_before, n_failed - f_before);

    // =========================================================================
    // Phase 3b: group alignment, by ramp.
    //
    // Same weights; every row of vector v now carries the same value A(v), and
    // A steps by 1 from vector to vector. The correct column sum is therefore
    // A(v)*136, a clean ramp. A group misaligned by delta contributes
    // A(v-delta)*S_g instead of A(v)*S_g, i.e. a constant offset of delta*S_g --
    // which is visible, reproducible and directly attributable, unlike the
    // same error under random data.
    // =========================================================================
    c_before = n_checked;
    f_before = n_failed;
    ramp = -60;
    for (v = 0; v < 120; v = v + 1) begin
      for (i = 0; i < NR; i = i + 1) a_vec[i] = ramp;
      set_expect();
      issue(1'b1);
      ramp = ramp + 1;
      if (ramp > 60) ramp = -60;
    end
    flush();
    $display("[3b] alignment ramp      : %0d checks, %0d fail",
             n_checked - c_before, n_failed - f_before);

    // =========================================================================
    // Phase 4: the COLSUM_W corner, from both sides, with the clamp in the way.
    //
    // All NR rows share one weight, so the column sum is 16 times a single
    // product: 16 * 127 * 128 = 260096, which needs 19 bits signed and is what
    // COLSUM_W = 20 is sized for. -128 weights are included so the clamp is
    // exercised here as well; the expectation is built from the clamped value,
    // so a removed clamp fails.
    //
    // The extremes actually reached are checked afterwards: if a future change
    // makes the corner unreachable, this phase would otherwise pass while
    // proving nothing.
    // =========================================================================
    c_before  = n_checked;
    f_before  = n_failed;
    max_abs_e = 0;
    max_abs_o = 0;
    for (sv0 = 0; sv0 < 3; sv0 = sv0 + 1) begin
      for (sv1 = 0; sv1 < 3; sv1 = sv1 + 1) begin
        drain();
        for (i = 0; i < NR; i = i + 1) begin
          W0v[i] = (sv0 == 0) ? WGT_MAX : ((sv0 == 1) ? WGT_MIN : -128);
          W1v[i] = (sv1 == 0) ? WGT_MIN : ((sv1 == 1) ? WGT_MAX : -128);
        end
        load_weights();

        for (av = 0; av < 6; av = av + 1) begin
          for (v = 0; v < 4; v = v + 1) begin
            for (i = 0; i < NR; i = i + 1)
              a_vec[i] = (av == 0) ? -128 :
                         (av == 1) ?  127 :
                         (av == 2) ? -127 :
                         (av == 3) ?    0 :
                         (av == 4) ?    1 : -1;
            set_expect();
            issue(1'b1);
          end
        end
        flush();
      end
    end
    n_checked = n_checked + 2;
    if (max_abs_e !== BOUND) begin
      n_failed = n_failed + 1;
      $display("  FAIL: even column reached only %0d, expected the bound %0d",
               max_abs_e, BOUND);
    end
    if (max_abs_o !== BOUND) begin
      n_failed = n_failed + 1;
      $display("  FAIL: odd column reached only %0d, expected the bound %0d",
               max_abs_o, BOUND);
    end
    $display("[4] COLSUM_W corners     : %0d checks, %0d fail  (max |colsum| = %0d)",
             n_checked - c_before, n_failed - f_before, max_abs_e);

    // =========================================================================
    // Phase 5: en = 0 must freeze the whole column -- the groups, the alignment
    // shift registers and the output register alike. An alignment register that
    // keeps shifting through a stall is the most likely way to get this wrong,
    // and it de-aligns the tree permanently rather than for the duration of the
    // stall.
    //
    // Two stalls, as day 3:
    //   A  noise driven onto act_in, in-flight expectations discarded (the
    //      vectors really are incomplete -- the skew means their later rows
    //      were never applied)
    //   B  the activation source frozen with the datapath and nothing
    //      discarded, which is what day 11's AXI backpressure will do
    // =========================================================================
    c_before = n_checked;
    f_before = n_failed;
    drain();
    for (i = 0; i < NR; i = i + 1) begin
      W0v[i] = $urandom_range(0, 254) - 127;
      W1v[i] = $urandom_range(0, 254) - 127;
    end
    load_weights();
    for (v = 0; v < 24; v = v + 1) begin
      for (i = 0; i < NR; i = i + 1)
        a_vec[i] = $urandom_range(0, 255) - 128;
      set_expect();
      issue(1'b1);
    end

    hold_e = s_even;
    hold_o = s_odd;
    for (i = 0; i < NR; i = i + 1) prev_act[i] = act_o[i];
    en = 1'b0;
    for (d = 0; d < 25; d = d + 1) begin
      for (i = 0; i < NR; i = i + 1)
        act[i] = ACT_W'($urandom_range(0, 255) - 128);
      @(posedge clk);
      #1;
      n_checked = n_checked + 1;
      if (s_even !== COLSUM_W'(hold_e) || s_odd !== COLSUM_W'(hold_o)) begin
        n_failed = n_failed + 1;
        if (n_failed - f_before <= 5)
          $display("  FAIL: en=0 but sums moved: odd=%0d even=%0d", s_odd, s_even);
      end
      for (i = 0; i < NR; i = i + 1) begin
        n_checked = n_checked + 1;
        if (act_o[i] !== ACT_W'(prev_act[i])) begin
          n_failed = n_failed + 1;
          if (n_failed - f_before <= 5)
            $display("  FAIL: en=0 but act_out[%0d] moved: %0d", i, act_o[i]);
        end
      end
    end
    en = 1'b1;
    // Discard the expectations that were in flight, and only these.
    for (i = 0; i <= LAT; i = i + 1) vld_q[i] = 1'b0;
    act = '0;
    for (i = 0; i < NR; i = i + 1) begin
      a_vec[i] = 0;
      for (d = 0; d < NR; d = d + 1) ahist[d][i] = 0;
    end
    eE = 0; eO = 0;
    for (d = 0; d < 2 * (LAT + NR); d = d + 1) issue(1'b0);
    // ... and prove the column came back correct rather than merely unstuck.
    for (v = 0; v < 40; v = v + 1) begin
      for (i = 0; i < NR; i = i + 1)
        a_vec[i] = $urandom_range(0, 255) - 128;
      set_expect();
      issue(1'b1);
    end
    flush();

    // ---- stall B: freeze the activation source with the datapath ------------
    for (v = 0; v < 24; v = v + 1) begin
      for (i = 0; i < NR; i = i + 1)
        a_vec[i] = $urandom_range(0, 255) - 128;
      set_expect();
      issue(1'b1);
    end
    hold_e = s_even;
    hold_o = s_odd;
    for (i = 0; i < NR; i = i + 1) prev_act[i] = act_o[i];
    en = 1'b0;
    for (d = 0; d < 20; d = d + 1) begin
      @(posedge clk);                       // act, ahist and the queue all held
      #1;
      n_checked = n_checked + 1;
      if (s_even !== COLSUM_W'(hold_e) || s_odd !== COLSUM_W'(hold_o)) begin
        n_failed = n_failed + 1;
        if (n_failed - f_before <= 5)
          $display("  FAIL: stall B, sums moved: odd=%0d even=%0d", s_odd, s_even);
      end
      for (i = 0; i < NR; i = i + 1) begin
        n_checked = n_checked + 1;
        if (act_o[i] !== ACT_W'(prev_act[i])) begin
          n_failed = n_failed + 1;
          if (n_failed - f_before <= 5)
            $display("  FAIL: stall B, act_out[%0d] moved: %0d", i, act_o[i]);
        end
      end
    end
    en = 1'b1;
    // Everything that was mid-flight must come out exactly as it would have
    // without the stall: the alignment registers held their contents, so the
    // groups are still aligned with each other on the far side of it.
    for (v = 0; v < 40; v = v + 1) begin
      for (i = 0; i < NR; i = i + 1)
        a_vec[i] = $urandom_range(0, 255) - 128;
      set_expect();
      issue(1'b1);
    end
    flush();
    $display("[5] en=0 stall           : %0d checks, %0d fail",
             n_checked - c_before, n_failed - f_before);

    // =========================================================================
    // Phase 6: wgt_shift_en = 0 with noise on the serial port. The 32 stored
    // weights must not move, so the running results must not change.
    // =========================================================================
    c_before = n_checked;
    f_before = n_failed;
    drain();
    for (i = 0; i < NR; i = i + 1) begin
      W0v[i] = $urandom_range(0, 254) - 127;
      W1v[i] = $urandom_range(0, 254) - 127;
    end
    load_weights();
    for (v = 0; v < 80; v = v + 1) begin
      wser = WGT_W'($urandom_range(0, 255) - 128);   // noise, wsh stays low
      for (i = 0; i < NR; i = i + 1)
        a_vec[i] = $urandom_range(0, 255) - 128;
      set_expect();
      issue(1'b1);
    end
    wser = '0;
    flush();
    $display("[6] wgt_shift_en gating  : %0d checks, %0d fail",
             n_checked - c_before, n_failed - f_before);

    // =========================================================================
    // Phase 7: the horizontal hand-off, measured directly, for all 16 rows.
    //
    // act_out[i] must be act_in[i] delayed by exactly one cycle, per row and
    // independently of the others. Two cycles, or a row crossed with its
    // neighbour, breaks every column-pair to the right once day 5 places seven
    // of these side by side, and does so without this one looking wrong.
    //
    // Two samples per cycle: before the edge act_out must still show the
    // PREVIOUS value (which kills a combinational hand-off, since the new input
    // is already being driven), and after it the new one (which kills a
    // two-deep one).
    // =========================================================================
    c_before = n_checked;
    f_before = n_failed;
    act = '0;
    @(posedge clk);
    #1;
    for (i = 0; i < NR; i = i + 1) prev_act[i] = 0;
    for (d = 0; d < 150; d = d + 1) begin
      for (i = 0; i < NR; i = i + 1)
        act[i] = ACT_W'($urandom_range(0, 255) - 128);
      #1;
      for (i = 0; i < NR; i = i + 1) begin
        n_checked = n_checked + 1;
        if (act_o[i] !== ACT_W'(prev_act[i])) begin
          n_failed = n_failed + 1;
          if (n_failed - f_before <= 5)
            $display("  FAIL: act_out[%0d] = %0d before the edge, want %0d (it is not registered)",
                     i, act_o[i], prev_act[i]);
        end
      end
      for (i = 0; i < NR; i = i + 1) prev_act[i] = $signed(act[i]);
      @(posedge clk);
      #1;
      for (i = 0; i < NR; i = i + 1) begin
        n_checked = n_checked + 1;
        if (act_o[i] !== ACT_W'(prev_act[i])) begin
          n_failed = n_failed + 1;
          if (n_failed - f_before <= 5)
            $display("  FAIL: act_out[%0d] = %0d, want %0d (one cycle of act_in)",
                     i, act_o[i], prev_act[i]);
        end
      end
    end
    $display("[7] horizontal hand-off  : %0d checks, %0d fail",
             n_checked - c_before, n_failed - f_before);

    // =========================================================================
    // Phase 8: the weight chain runs through all four groups, exactly 2*NR deep.
    //
    // wgt_out must be wgt_in delayed by 32 shift cycles. Day 5 needs this to be
    // exact if it chains column-pairs; a chain that is 31 or 33 deep misplaces
    // every weight in everything below the first.
    // =========================================================================
    c_before = n_checked;
    f_before = n_failed;
    act = '0;
    // Empty the chain first. It still holds phase 6's weights, and a model that
    // starts at zero against hardware that does not reports NW-1 failures that
    // say nothing about the depth being measured.
    wsh  = 1'b1;
    wser = '0;
    for (d = 0; d < NW; d = d + 1) begin
      @(posedge clk);
      #1;
    end
    for (i = 0; i < NW; i = i + 1) wq[i] = 0;
    for (d = 0; d < 3 * NW; d = d + 1) begin
      // Values stay inside [-127,127] so the clamp does not alter them and this
      // phase measures depth alone. The clamp itself is phase 4's job.
      v = $urandom_range(0, 254) - 127;
      wser = WGT_W'(v);
      // Shift the model first, then clock: after the edge the value presented
      // to this edge occupies stage 0, so stage NW-1 holds the value presented
      // NW-1 edges ago. Modelling it the other way round is an off-by-one that
      // makes a chain of NW-1 or NW+1 look correct.
      for (s = NW-1; s > 0; s = s - 1) wq[s] = wq[s-1];
      wq[0] = v;
      @(posedge clk);
      #1;
      n_checked = n_checked + 1;
      if (wgt_o !== WGT_W'(wq[NW-1])) begin
        n_failed = n_failed + 1;
        if (n_failed - f_before <= 5)
          $display("  FAIL: wgt_out = %0d, want %0d (wgt_in delayed %0d shifts)",
                   wgt_o, wq[NW-1], NW);
      end
    end
    // The chain must also stand still when told to.
    wsh = 1'b0;
    for (d = 0; d < 12; d = d + 1) begin
      wser = WGT_W'($urandom_range(0, 255) - 128);
      @(posedge clk);
      #1;
      n_checked = n_checked + 1;
      if (wgt_o !== WGT_W'(wq[NW-1])) begin
        n_failed = n_failed + 1;
        if (n_failed - f_before <= 5)
          $display("  FAIL: wgt_out moved with wgt_shift_en low: %0d", wgt_o);
      end
    end
    wser = '0;
    $display("[8] weight chain depth   : %0d checks, %0d fail",
             n_checked - c_before, n_failed - f_before);

    // =========================================================================
    $display("---------------------------------------------------------");
    if (n_failed == 0)
      $display(" PASS   %0d checks, 0 failures", n_checked);
    else
      $display(" FAIL   %0d checks, %0d failures", n_checked, n_failed);
    $display("=========================================================");

    if (n_failed != 0) $fatal(1, "tb_column_pair: %0d failures", n_failed);
    $finish;
  end

  // Watchdog: a deadlocked run must not hang CI forever.
  initial begin
    #200_000_000;
    $display(" FAIL   tb_column_pair: timeout");
    $fatal(1, "timeout");
  end

endmodule
