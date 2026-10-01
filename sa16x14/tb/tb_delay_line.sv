`timescale 1ns / 1ps

module tb_delay_line;
  import sa_pkg::*;

  localparam int MAXD = 17;              // depths 0..17
  localparam int ND   = MAXD + 1;        // 18 lines per width
  localparam int W1   = 1;
  localparam int W9   = ACT_W + 1;       // 9: day 8's {swap, act}
  localparam int W40  = 2 * COLSUM_W;    // 40: day 9's column-sum pair
  localparam int MAXW = W40;

  // Actual is 105,192 under both simulators; a count far below this means a
  // phase ran a fraction of its iterations rather than passing.
  localparam int MINCHK = 90000;

  // ---- clock. There is no reset in this testbench, deliberately: delay_line
  // ---- has no reset port and phase 0 is about what it does without one. -----
  logic clk = 1'b0;
  logic en  = 1'b1;
  initial forever #5 clk = ~clk;         // 100 MHz

  // ---- the lines ------------------------------------------------------------
  logic [ND-1:0]             d1_in,  d1_out;
  logic [ND-1:0][W9-1:0]     d9_in,  d9_out;
  logic [ND-1:0][W40-1:0]    d40_in, d40_out;

  genvar gd;
  generate
    for (gd = 0; gd <= MAXD; gd++) begin : g_line
      delay_line #(.W(W1), .DEPTH(gd)) u_w1 (
        .clk (clk), .en (en), .din (d1_in[gd]),  .dout (d1_out[gd]));
      delay_line #(.W(W9), .DEPTH(gd)) u_w9 (
        .clk (clk), .en (en), .din (d9_in[gd]),  .dout (d9_out[gd]));
      delay_line #(.W(W40), .DEPTH(gd)) u_w40 (
        .clk (clk), .en (en), .din (d40_in[gd]), .dout (d40_out[gd]));
    end
  endgenerate

  // ---- the model ------------------------------------------------------------
  // dr[d] is the word currently being driven to all three widths of line d,
  // truncated per width. mdl[d][k] mirrors stage k of line d. The mirror exists
  // for the streaming phases; the depth itself is measured independently in
  // phases 1 and 4, so a mirror that agreed with a wrong depth could not hide
  // it.
  logic [MAXW-1:0] dr  [0:MAXD];
  logic [MAXW-1:0] mdl [0:MAXD][0:MAXD-1];
  logic [MAXW-1:0] exp_w;

  // Temporaries for whole-variable port writes.
  logic [ND-1:0]          t1;
  logic [ND-1:0][W9-1:0]  t9;
  logic [ND-1:0][W40-1:0] t40;

  // ---- marker-measurement bookkeeping (phases 1 and 4) ---------------------
  bit              obs;                  // record instead of compare
  integer          seen_n   [0:MAXD];
  integer          seen_at  [0:MAXD];
  logic [MAXW-1:0] seen_val [0:MAXD];
  integer          ecyc;                  // enabled cycles since the marker

  // Counters used INSIDE tasks. Nothing enclosing a task call may use them.
  integer i, s;
  // Counters for loops that DO enclose task calls.
  integer oi, oj, ok;

  integer n_checked, n_failed;
  integer c_before, f_before;
  logic [MAXW-1:0] marker;

  // ---------------------------------------------------------------------------
  // Helpers
  // ---------------------------------------------------------------------------
  // Full-width random word. Built from whole $urandom draws rather than from a
  // `%` of the low bits: the low bits of the RNG have a short period under the
  // lint/sim tool of docs/NOTES-tooling.md, which is what made a whole row of
  // day 6's weight tiles come out identical.
  function automatic logic [MAXW-1:0] rand_word;
    logic [63:0] r;
    begin
      r = {$urandom, $urandom};
      rand_word = r[MAXW-1:0];
    end
  endfunction

  task automatic set_rand;
    begin
      for (i = 0; i <= MAXD; i = i + 1) dr[i] = rand_word();
    end
  endtask

  task automatic set_const(input logic [MAXW-1:0] val);
    begin
      for (i = 0; i <= MAXD; i = i + 1) dr[i] = val;
    end
  endtask

  // Compare every line at every width against the mirror. The expectation is
  // the same expression before and after the edge -- what differs is whether
  // the mirror has been shifted -- which is what makes one task cover both
  // samples, and makes "frozen while en is low" fall out of the model rather
  // than needing a separate case.
  task automatic check_all(input integer tag);
    begin
      for (i = 0; i <= MAXD; i = i + 1) begin
        exp_w = (i == 0) ? dr[i] : mdl[i][i-1];

        n_checked = n_checked + 1;
        if (d1_out[i] !== exp_w[W1-1:0]) begin
          n_failed = n_failed + 1;
          if (n_failed - f_before <= 10)
            $display("  FAIL: W=%0d depth=%0d %0s-edge: got %0h, want %0h",
                     W1, i, (tag == 0) ? "pre" : "post",
                     d1_out[i], exp_w[W1-1:0]);
        end

        n_checked = n_checked + 1;
        if (d9_out[i] !== exp_w[W9-1:0]) begin
          n_failed = n_failed + 1;
          if (n_failed - f_before <= 10)
            $display("  FAIL: W=%0d depth=%0d %0s-edge: got %0h, want %0h",
                     W9, i, (tag == 0) ? "pre" : "post",
                     d9_out[i], exp_w[W9-1:0]);
        end

        n_checked = n_checked + 1;
        if (d40_out[i] !== exp_w[W40-1:0]) begin
          n_failed = n_failed + 1;
          if (n_failed - f_before <= 10)
            $display("  FAIL: W=%0d depth=%0d %0s-edge: got %0h, want %0h",
                     W40, i, (tag == 0) ? "pre" : "post",
                     d40_out[i], exp_w[W40-1:0]);
        end
      end
    end
  endtask

  // One cycle. Drives dr onto all three widths, samples BEFORE the edge, clocks,
  // then samples after it.
  //
  // The pre-edge sample is what kills a combinational line, because the new
  // input is already being driven at that point: a registered line must still
  // be showing the previous word. The post-edge sample is what kills a line one
  // stage too deep. Only the pair kills both, which is day 3's lesson applied to
  // a module whose entire job is a delay.
  task automatic step(input bit e, input bit chk);
    begin
      en = e;

      t1  = '0;
      t9  = '0;
      t40 = '0;
      for (i = 0; i <= MAXD; i = i + 1) begin
        t1[i]  = dr[i][0];
        t9[i]  = dr[i][W9-1:0];
        t40[i] = dr[i][W40-1:0];
      end
      d1_in  = t1;
      d9_in  = t9;
      d40_in = t40;

      #1;
      if (chk) check_all(0);

      // Marker measurement, on enabled cycles only. A stalled cycle holds the
      // marker at dout without advancing it, so counting stalled observations
      // would report a smear where the line is behaving correctly; counting
      // enabled cycles is also the thing phase 4 is trying to prove.
      if (obs && e) begin
        for (i = 0; i <= MAXD; i = i + 1) begin
          if (d40_out[i] !== {W40{1'b0}}) begin
            seen_n[i] = seen_n[i] + 1;
            if (seen_n[i] == 1) begin
              seen_at[i]  = ecyc;
              seen_val[i] = d40_out[i];
            end
          end
        end
      end

      @(posedge clk);

      if (e) begin
        for (i = 0; i <= MAXD; i = i + 1) begin
          for (s = MAXD-1; s > 0; s = s - 1) mdl[i][s] = mdl[i][s-1];
          mdl[i][0] = dr[i];
        end
      end

      #1;
      if (chk) check_all(1);

      if (e) ecyc = ecyc + 1;
    end
  endtask

  task automatic banner(input string txt);
    begin
      c_before = n_checked;
      f_before = n_failed;
      $display("---------------------------------------------------------");
      $display(" %s", txt);
    end
  endtask

  task automatic report(input string txt);
    begin
      $display(" %s : %0d checks, %0d fail", txt,
               n_checked - c_before, n_failed - f_before);
    end
  endtask

  // Run one marker measurement and score it. stall_pat selects whether cycles
  // are stalled: 0 = never, otherwise a stall is inserted whenever
  // (cycle % stall_pat) == 1, plus one long stall, so the arrival index must be
  // counted in enabled cycles to come out right.
  task automatic measure_depth(input integer stall_pat);
    integer m, held;
    begin
      // Empty every line first. A measurement that starts with stale data in
      // the lines reports a smear that says nothing about the depth -- the same
      // trap as day 3's shift-register depth check starting from an unflushed
      // chain.
      set_const('0);
      obs = 1'b0;
      for (m = 0; m < MAXD + 4; m = m + 1) step(1'b1, 1'b0);

      for (m = 0; m <= MAXD; m = m + 1) begin
        seen_n[m]   = 0;
        seen_at[m]  = -1;
        seen_val[m] = '0;
      end
      ecyc = 0;
      obs  = 1'b1;

      // One cycle of the marker, then zeros. The marker is non-zero in every
      // bit of all three widths, so a lost or crossed bit shows up as a wrong
      // value rather than as a missed arrival.
      set_const(marker);
      step(1'b1, 1'b0);
      set_const('0);
      for (m = 0; m < 2 * MAXD + 12; m = m + 1) begin
        held = 0;
        if (stall_pat != 0) begin
          if ((m % stall_pat) == 1)                held = 1;
          if (m >= MAXD + 2 && m <= MAXD + 5)      held = 1;  // one long stall
        end
        if (held) step(1'b0, 1'b0);
        else      step(1'b1, 1'b0);
      end
      obs = 1'b0;

      for (m = 0; m <= MAXD; m = m + 1) begin
        n_checked = n_checked + 3;
        if (seen_n[m] !== 1) begin
          n_failed = n_failed + 1;
          $display("  FAIL: depth=%0d went non-zero on %0d enabled cycles, want exactly 1",
                   m, seen_n[m]);
        end
        if (seen_at[m] !== m) begin
          n_failed = n_failed + 1;
          $display("  FAIL: depth=%0d marker arrived at enabled cycle %0d, want %0d",
                   m, seen_at[m], m);
        end
        if (seen_val[m] !== marker[W40-1:0]) begin
          n_failed = n_failed + 1;
          $display("  FAIL: depth=%0d marker came out %0h, want %0h",
                   m, seen_val[m], marker[W40-1:0]);
        end
      end
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
    ecyc      = 0;
    obs       = 1'b0;
    exp_w     = '0;

    // Whole-variable initialisation of everything that feeds a port, before any
    // element-wise write anywhere.
    d1_in  = '0;
    d9_in  = '0;
    d40_in = '0;
    t1     = '0;
    t9     = '0;
    t40    = '0;

    for (i = 0; i <= MAXD; i = i + 1) begin
      dr[i]       = '0;
      seen_n[i]   = 0;
      seen_at[i]  = -1;
      seen_val[i] = '0;
      for (s = 0; s < MAXD; s = s + 1) mdl[i][s] = '0;
    end

    // Non-zero in every bit of all three widths, and not a power of two, so a
    // crossed or dropped bit changes the value.
    marker = {8'hA5, 32'h3C5A_96E7};

    $display("=========================================================");
    $display(" tb_delay_line : depths 0..%0d at W = %0d / %0d / %0d", MAXD, W1, W9, W40);
    $display("   %0d lines, no reset port -- the power-up state IS the reset",
             3 * ND);
    $display("=========================================================");

    // =======================================================================
    // Phase 0: the power-up state, which is what replaces the reset.
    //
    // Nothing has been clocked and there is no reset to assert. Every output
    // must already read exactly zero, checked with `===` so an X fails here
    // rather than surviving into an arithmetic comparison downstream. Then the
    // same with en held LOW while non-zero data is driven: a line that shifts
    // regardless of en would leak that data out within DEPTH cycles, and a
    // depth-0 line -- being a wire -- must follow it, which the mirror expects.
    // =======================================================================
    banner("Phase 0: power-up state is zero, and en=0 lets nothing in");
    #1;
    check_all(0);
    for (ok = 0; ok < 4; ok = ok + 1) begin
      set_const('0);
      step(1'b0, 1'b1);
    end
    for (ok = 0; ok < MAXD + 4; ok = ok + 1) begin
      set_rand();
      step(1'b0, 1'b1);           // en low: only the depth-0 wire may respond
    end
    set_const('0);
    for (ok = 0; ok < 2; ok = ok + 1) step(1'b0, 1'b1);
    report("[0] power-up state       ");

    // =======================================================================
    // Phase 1: the depth, measured. See the header -- this is the check that a
    // mirror cannot make on its own, and it covers depth 0 with the same
    // arithmetic as every other depth.
    // =======================================================================
    banner("Phase 1: depth measured by marker pulse, no stalls");
    measure_depth(0);
    report("[1] depth measured       ");

    // =======================================================================
    // Phase 2: random streaming at full rate, every line on its own stream.
    //
    // Independent streams per (depth, width) line are the point: a line that
    // read a neighbour's din -- a mis-indexed generate loop, which is exactly
    // what days 8 and 9 are about to write -- produces plausible shifted data
    // and dies only against a per-line reference.
    // =======================================================================
    banner("Phase 2: random streaming at full rate");
    for (ok = 0; ok < 400; ok = ok + 1) begin
      set_rand();
      step(1'b1, 1'b1);
    end
    report("[2] full-rate streaming  ");

    // =======================================================================
    // Phase 3: random streaming with stalls, including single-cycle stalls and
    // a long one. A stage that advances through a stall, or an en that is only
    // applied to the first stage, changes the mirror comparison on the very
    // next cycle. The depth-0 wire must keep following din throughout.
    // =======================================================================
    banner("Phase 3: streaming with random and long stalls");
    for (ok = 0; ok < 400; ok = ok + 1) begin
      set_rand();
      // A mix of isolated stalls and one long burst, so both a lost cycle and a
      // lost multi-cycle hold are covered.
      if ((ok % 7) == 3)                 step(1'b0, 1'b1);
      else if (ok >= 200 && ok <= 215)   step(1'b0, 1'b1);
      else                               step(1'b1, 1'b1);
    end
    report("[3] stalled streaming    ");

    // =======================================================================
    // Phase 4: the depth again, re-measured with stalls interleaved. The
    // arrival index is counted in ENABLED cycles, so a line that is the right
    // depth at full rate but loses or gains a stage across a stall fails here
    // and nowhere else.
    // =======================================================================
    banner("Phase 4: depth re-measured across stalls");
    measure_depth(3);
    measure_depth(5);
    report("[4] depth across stalls  ");

    // =======================================================================
    // Phase 5: bit integrity across the full 40-bit word.
    //
    // A walking one at every bit position, then all-ones and all-zeros. This is
    // what kills a bit reversal, a dropped top bit and a crossed pair inside the
    // word -- none of which the random streams are guaranteed to expose at a
    // specific position, and all of which would corrupt day 9's packed pair of
    // column sums in a way that looks like an arithmetic bug.
    // =======================================================================
    banner("Phase 5: walking ones over all 40 bits, plus the flat patterns");
    for (ok = 0; ok < MAXW; ok = ok + 1) begin
      set_const({{(MAXW-1){1'b0}}, 1'b1} << ok);
      step(1'b1, 1'b1);
      step(1'b1, 1'b1);
    end
    for (ok = 0; ok < MAXD + 4; ok = ok + 1) begin
      set_const('1);
      step(1'b1, 1'b1);
    end
    for (ok = 0; ok < MAXD + 4; ok = ok + 1) begin
      set_const('0);
      step(1'b1, 1'b1);
    end
    // ...and a per-line distinct constant, held long enough to fill every line,
    // so two lines that shared storage could not both read back their own.
    for (oj = 0; oj < MAXD + 6; oj = oj + 1) begin
      for (i = 0; i <= MAXD; i = i + 1)
        dr[i] = {{(MAXW-8){1'b0}}, 8'h11} * (i + 1);
      step(1'b1, 1'b1);
    end
    report("[5] bit integrity        ");

    // =======================================================================
    $display("---------------------------------------------------------");
    if (n_checked < MINCHK) begin
      $display(" FAIL   only %0d checks, expected at least %0d", n_checked, MINCHK);
      $fatal(1, "tb_delay_line: check count collapsed -- a phase ran a fraction of its iterations");
    end
    if (n_failed != 0) begin
      $display(" FAIL   %0d checks, %0d failures", n_checked, n_failed);
      $fatal(1, "tb_delay_line: %0d failures", n_failed);
    end
    $display(" PASS   %0d checks, 0 failures", n_checked);
    $display("=========================================================");
    $finish;
  end

  // Wall-clock guard, so a hang is a failure rather than a timeout in CI.
  initial begin
    #10_000_000;
    $display(" FAIL   tb_delay_line: timeout");
    $fatal(1, "tb_delay_line: timeout");
  end

endmodule
