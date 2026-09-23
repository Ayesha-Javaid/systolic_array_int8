// -----------------------------------------------------------------------------
// tb_pe_pair.sv — self-checking test of the array's unit cell
//
// pe_pair adds exactly two things to dsp_pack_mac, and both of them fail
// silently when wrong, so both are pinned here rather than sampled:
//
//   * the serial weight-load chain. A reversed chain, a swapped w0/w1, or a
//     wgt_out taken from the wrong stage all still produce plausible numbers
//     from a plausible-looking array -- they just compute a transposed or
//     permuted tile. Phase 1 drives a one-hot activation vector per row, which
//     makes every one of the 2*N weight positions individually observable.
//   * the horizontal activation hand-off. It must be exactly one cycle. Two
//     cycles (a register added around the MAC instead of using its BREG) or
//     zero (a combinational pass-through) both simulate fine in isolation and
//     desynchronise the array at column 1. Phase 6 measures the delay directly.
//
// Structures under test:
//   vertical   : N = CASCADE_DEPTH cells, weight chain + PCOUT->PCIN cascade,
//                activations applied with the per-row skew the cascade requires
//   horizontal : M cells chained act_out -> act_in, weights irrelevant
//
// Phases:
//   1  one-hot weight placement   -- every weight position, individually
//   2  randomised streaming       -- N tiles, weights reloaded through the chain
//   3  weight clamp               -- -128 presented at the port must store -127
//   4  en = 0 stall               -- datapath freezes, nothing drifts
//   5  wgt_shift_en = 0           -- the chain must not move when not told to
//   6  horizontal delay           -- exactly one cycle per cell, hen-gated
//
// Exits non-zero on any failure ($fatal), so it is usable as a CI gate under
// both Icarus and Verilator.
//
// Style: module-scope declarations, fixed-size arrays, whole-variable
// initialisation of everything that feeds a port. See docs/NOTES-tooling.md --
// this is portability, not taste.
// -----------------------------------------------------------------------------
`timescale 1ns / 1ps

module tb_pe_pair;

  import sa_pkg::*;

  localparam int N        = CASCADE_DEPTH;   // vertical chain depth = 4
  localparam int LAT      = N + 1;           // issue -> result, as day 1
  localparam int M        = 4;               // horizontal chain length
  localparam int TILES    = 30;
  localparam int PER_TILE = 80;

  // ---- clock / reset --------------------------------------------------------
  logic clk = 1'b0;
  logic rst_n = 1'b0;
  logic en  = 1'b1;
  logic hen = 1'b1;
  initial forever #5 clk = ~clk;             // 100 MHz. `initial forever`, not
                                             // `always`: the lint pass would
                                             // otherwise call this sequential
                                             // logic and fire BLKSEQ.

  // ===========================================================================
  // Vertical structure: N pe_pairs, weight chain and cascade both top-to-bottom
  // ===========================================================================
  logic                             wsh;     // wgt_shift_en
  logic signed [WGT_W-1:0]          wser;    // serial weight input

  logic signed [N-1:0][ACT_W-1:0]   act;     // per-row activation (skewed)
  logic signed [N-1:0][ACT_W-1:0]   act_o;
  logic signed [N:0]  [WGT_W-1:0]   wch;     // wch[0] = wser, wch[g+1] = cell g
  logic signed [N:0]  [DSP_P_W-1:0] casc;

  assign wch[0]  = wser;
  assign casc[0] = '0;                       // acc_first coverage lives in
                                             // tb_dsp_pack_mac; here the head
                                             // of the cascade is simply zero.

  genvar g;
  generate
    for (g = 0; g < N; g++) begin : g_cell
      pe_pair #(.K(K_PACK), .CLAMP_WGT(1'b1)) u_pe (
        .clk          (clk),
        .rst_n        (rst_n),
        .en           (en),
        .wgt_shift_en (wsh),
        .wgt_in       (wch[g]),
        .wgt_out      (wch[g+1]),
        .act_in       (act[g]),
        .act_out      (act_o[g]),
        .acc_first    (g == 0),              // only the top cell starts fresh
        .pcin         (casc[g]),
        .pcout        (casc[g+1])
      );
    end
  endgenerate

  logic signed [GROUP_W-1:0] p0, p1;
  pack_extract #(.K(K_PACK), .OUT_W(GROUP_W)) u_ext (
    .p_packed (casc[N]),
    .p0       (p0),
    .p1       (p1)
  );

  // ===========================================================================
  // Horizontal structure: M pe_pairs chained act_out -> act_in
  // ===========================================================================
  logic signed [ACT_W-1:0]          h_in;
  logic signed [M:0]  [ACT_W-1:0]   hact;    // hact[0] = h_in, hact[m+1] = cell m
  logic signed [M-1:0][DSP_P_W-1:0] hpc;

  assign hact[0] = h_in;

  genvar h;
  generate
    for (h = 0; h < M; h++) begin : g_hcell
      pe_pair #(.K(K_PACK), .CLAMP_WGT(1'b1)) u_hpe (
        .clk          (clk),
        .rst_n        (rst_n),
        .en           (hen),
        .wgt_shift_en (1'b0),
        .wgt_in       ('0),
        .wgt_out      (),
        .act_in       (hact[h]),
        .act_out      (hact[h+1]),
        .acc_first    (1'b1),
        .pcin         ('0),
        .pcout        (hpc[h])
      );
    end
  endgenerate

  // ===========================================================================
  // Bookkeeping
  // ===========================================================================
  integer n_checked, n_failed, n_mark, f_mark;
  integer i, d, s, k, tile, v, t;

  // W*v : what the testbench SHIFTS IN.  E*v : what the cell should STORE.
  // They differ only in phase 3, which is the entire point of that phase.
  integer W0v [0:N-1];
  integer W1v [0:N-1];
  integer E0v [0:N-1];
  integer E1v [0:N-1];

  integer a_vec [0:N-1];
  integer ahist [0:N-1][0:N-1];              // ahist[d][i]: row i, d cycles ago
  integer exp0_q [0:LAT];
  integer exp1_q [0:LAT];
  bit     vld_q  [0:LAT];
  integer e0, e1;

  integer href [0:M-1];                      // expected act_out of each h cell
  integer hv;

  logic signed [GROUP_W-1:0]        hold0, hold1;
  logic signed [N-1:0][ACT_W-1:0]   hold_acto;

  task automatic fail_msg(input integer got0, input integer got1,
                          input integer want0, input integer want1);
    begin
      n_failed = n_failed + 1;
      if (n_failed <= 12)
        $display("  FAIL: got p1=%0d p0=%0d, want p1=%0d p0=%0d",
                 got1, got0, want1, want0);
    end
  endtask

  // Issue one activation vector: push it into the skew history, push its
  // expectation into the scoreboard, advance one clock, check what arrives.
  // Identical in shape to tb_dsp_pack_mac's, deliberately -- a divergence
  // between the two would mean one of them is wrong about the pipeline.
  task automatic issue(input bit valid);
    begin
      for (s = N-1; s > 0; s = s - 1)
        for (i = 0; i < N; i = i + 1)
          ahist[s][i] = ahist[s-1][i];
      for (i = 0; i < N; i = i + 1)
        ahist[0][i] = a_vec[i];

      for (i = 0; i < N; i = i + 1)
        act[i] = ACT_W'(ahist[i][i]);

      for (s = LAT; s > 0; s = s - 1) begin
        exp0_q[s] = exp0_q[s-1];
        exp1_q[s] = exp1_q[s-1];
        vld_q[s]  = vld_q[s-1];
      end
      exp0_q[0] = e0;
      exp1_q[0] = e1;
      vld_q[0]  = valid;

      @(posedge clk);
      #1;

      if (vld_q[LAT]) begin
        n_checked = n_checked + 1;
        if (p0 !== GROUP_W'(exp0_q[LAT]) || p1 !== GROUP_W'(exp1_q[LAT]))
          fail_msg(p0, p1, exp0_q[LAT], exp1_q[LAT]);
      end
    end
  endtask

  // Compute the expectation for whatever is currently in a_vec, from the
  // weights the cells are supposed to be holding.
  task automatic set_expect;
    begin
      e0 = 0;
      e1 = 0;
      for (i = 0; i < N; i = i + 1) begin
        e0 = e0 + E0v[i] * a_vec[i];
        e1 = e1 + E1v[i] * a_vec[i];
      end
    end
  endtask

  // Zero activations for long enough that nothing in flight can straddle a
  // weight change, and the scoreboard holds only zero expectations.
  task automatic drain;
    begin
      for (i = 0; i < N; i = i + 1) a_vec[i] = 0;
      e0 = 0;
      e1 = 0;
      for (d = 0; d < LAT + N + 2; d = d + 1) issue(1'b1);
    end
  endtask

  // Shift 2*N weights in. The first value presented travels furthest, so the
  // chain is filled from the far end backwards, odd column before even:
  //
  //     W1v[N-1], W0v[N-1], W1v[N-2], W0v[N-2], ... W1v[0], W0v[0]
  //
  // Activations are held at zero throughout (drain() ran first), so the
  // datapath produces zeros while the weights move and nothing observable
  // straddles the change.
  task automatic load_weights;
    integer li;
    begin
      wsh = 1'b1;
      for (li = N-1; li >= 0; li = li - 1) begin
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

  // ===========================================================================
  // Stimulus
  // ===========================================================================
  initial begin
    n_checked = 0;
    n_failed  = 0;
    e0 = 0; e1 = 0;
    hv = 0;

    // Whole-variable initialisation of everything that feeds a port, before
    // any element-wise write. Under Verilator 5.020 an element-only write to a
    // port-connected variable can fail to propagate, silently, while Icarus
    // simulates it correctly. See docs/NOTES-tooling.md.
    act       = '0;
    hold_acto = '0;
    wser      = '0;
    wsh       = 1'b0;
    h_in      = '0;

    for (i = 0; i <= LAT; i = i + 1) begin
      exp0_q[i] = 0; exp1_q[i] = 0; vld_q[i] = 1'b0;
    end
    for (i = 0; i < N; i = i + 1) begin
      a_vec[i] = 0;
      W0v[i] = 0; W1v[i] = 0; E0v[i] = 0; E1v[i] = 0;
      for (d = 0; d < N; d = d + 1) ahist[d][i] = 0;
    end
    for (i = 0; i < M; i = i + 1) href[i] = 0;

    $display("=========================================================");
    $display(" tb_pe_pair : %0d-cell vertical chain, %0d-cell horizontal chain",
             N, M);
    $display("   weight chain is %0d deep, issue->result %0d cycles", 2*N, LAT);
    $display("=========================================================");

    repeat (4) @(posedge clk);
    rst_n = 1'b1;
    repeat (2) @(posedge clk);

    // -----------------------------------------------------------------------
    // Phase 1: one-hot weight placement.
    //
    // Every weight is distinct, so a single non-zero activation in row k makes
    // the group result exactly (E1v[k]*a, E0v[k]*a) -- that is, it reads back
    // one cell's two weights in isolation. Running k over every row reads the
    // whole chain position by position. A reversed load order, a w0/w1 swap or
    // a wgt_out taken from the wrong stage each move at least one weight and
    // are caught here with a diagnosis rather than a pile of mismatches.
    // -----------------------------------------------------------------------
    for (i = 0; i < N; i = i + 1) begin
      W1v[i] =   7 + 11*i;                   //  7,  18,  29,  40
      W0v[i] = -13 - 17*i;                   // -13, -30, -47, -64
      E1v[i] = W1v[i];
      E0v[i] = W0v[i];
    end
    drain();
    load_weights();
    drain();

    n_mark = n_checked; f_mark = n_failed;
    for (k = 0; k < N; k = k + 1) begin
      for (t = 0; t < 3; t = t + 1) begin
        for (i = 0; i < N; i = i + 1) a_vec[i] = 0;
        case (t)
          0: a_vec[k] =  127;
          1: a_vec[k] = -128;
          default: a_vec[k] = 3;
        endcase
        set_expect();
        issue(1'b1);
        // let this vector clear before the next one-hot, so a result can only
        // be attributed to the row that produced it
        for (i = 0; i < N; i = i + 1) a_vec[i] = 0;
        e0 = 0; e1 = 0;
        for (d = 0; d < LAT + N; d = d + 1) issue(1'b1);
      end
    end
    $display("[1] one-hot placement   : %0d checks, %0d fail",
             n_checked - n_mark, n_failed - f_mark);

    // -----------------------------------------------------------------------
    // Phase 2: randomised streaming, weights reloaded through the chain between
    // tiles. Back-to-back vectors, which is the steady state the array runs in.
    // Every fourth tile is a corner tile that drives the 17-bit low field to
    // its limit.
    // -----------------------------------------------------------------------
    n_mark = n_checked; f_mark = n_failed;
    for (tile = 0; tile < TILES; tile = tile + 1) begin
      for (i = 0; i < N; i = i + 1) begin
        case (tile % 4)
          0: begin W0v[i] =  127; W1v[i] =  127; end
          1: begin W0v[i] =  127; W1v[i] = -127; end
          2: begin W0v[i] = -127; W1v[i] =  127; end
          default: begin
            W0v[i] = $urandom_range(0, 254) - 127;
            W1v[i] = $urandom_range(0, 254) - 127;
          end
        endcase
        E0v[i] = W0v[i];
        E1v[i] = W1v[i];
      end

      drain();
      load_weights();
      drain();

      for (v = 0; v < PER_TILE; v = v + 1) begin
        for (i = 0; i < N; i = i + 1)
          a_vec[i] = (tile % 4 < 3) && (v % 3 == 0)
                     ? -128                  // |w*a| is maximal at a = -128
                     : $urandom_range(0, 255) - 128;
        set_expect();
        issue(1'b1);
      end
    end
    $display("[2] streaming + reload  : %0d checks, %0d fail",
             n_checked - n_mark, n_failed - f_mark);

    // -----------------------------------------------------------------------
    // Phase 3: the weight clamp.
    //
    // -128 is presented on the serial port; -127 must be what the cell stores.
    // Without the clamp both failure modes fire at once: the A port overflows
    // (w1 = -128 with w0 < 0) and a full group of 4*128*128 = 65536 bleeds out
    // of the 17-bit low field by exactly 1. Activations are pinned at -128 for
    // part of the phase so the low field is driven right to that boundary.
    // -----------------------------------------------------------------------
    n_mark = n_checked; f_mark = n_failed;
    for (i = 0; i < N; i = i + 1) begin
      W0v[i] = -128;  E0v[i] = -127;
      W1v[i] = -128;  E1v[i] = -127;
    end
    drain();
    load_weights();
    drain();

    for (v = 0; v < 60; v = v + 1) begin
      for (i = 0; i < N; i = i + 1)
        a_vec[i] = (v % 2 == 0) ? -128 : $urandom_range(0, 255) - 128;
      set_expect();
      issue(1'b1);
    end

    // mixed: an illegal high weight beside a legal low one, and the reverse
    for (i = 0; i < N; i = i + 1) begin
      W1v[i] = -128;                E1v[i] = -127;
      W0v[i] = (i % 2) ? 127 : -128; E0v[i] = (i % 2) ? 127 : -127;
    end
    drain();
    load_weights();
    drain();
    for (v = 0; v < 60; v = v + 1) begin
      for (i = 0; i < N; i = i + 1)
        a_vec[i] = (v % 2 == 0) ? -128 : $urandom_range(0, 255) - 128;
      set_expect();
      issue(1'b1);
    end
    $display("[3] weight clamp        : %0d checks, %0d fail",
             n_checked - n_mark, n_failed - f_mark);

    // -----------------------------------------------------------------------
    // Phase 4: en = 0 freezes the datapath.
    //
    // A constant activation vector is streamed until the whole pipeline holds
    // the same vector, so the output is a known non-zero constant. en then goes
    // low while the activation inputs are driven with noise: nothing -- neither
    // the cascade result nor any act_out -- may move. This is the property the
    // AXI backpressure path (days 11-12) will depend on, and an en that is
    // ignored looks perfectly healthy without it.
    // -----------------------------------------------------------------------
    n_mark = n_checked; f_mark = n_failed;
    for (i = 0; i < N; i = i + 1) begin
      W1v[i] =  100;  E1v[i] =  100;
      W0v[i] =  -99;  E0v[i] =  -99;
    end
    drain();
    load_weights();
    drain();

    for (i = 0; i < N; i = i + 1) a_vec[i] = 7;
    set_expect();
    for (d = 0; d < LAT + N + 2; d = d + 1) issue(1'b1);   // fill with one vector

    hold0     = p0;
    hold1     = p1;
    hold_acto = act_o;
    n_checked = n_checked + 1;
    if (p0 === GROUP_W'(0) || p1 === GROUP_W'(0)) begin
      n_failed = n_failed + 1;
      $display("  FAIL: stall phase needs a non-zero steady state, got p1=%0d p0=%0d",
               p1, p0);
    end

    en = 1'b0;
    for (d = 0; d < 6; d = d + 1) begin
      for (i = 0; i < N; i = i + 1)
        act[i] = ACT_W'($urandom_range(0, 255) - 128);   // noise on the inputs
      @(posedge clk);
      #1;
      n_checked = n_checked + 1;
      if (p0 !== hold0 || p1 !== hold1)
        fail_msg(p0, p1, hold0, hold1);
      if (act_o !== hold_acto) begin
        n_failed = n_failed + 1;
        if (n_failed <= 12)
          $display("  FAIL: act_out moved while en=0 (%h -> %h)", hold_acto, act_o);
      end
    end
    en = 1'b1;

    // resume: the pipeline still holds only the constant vector, so the result
    // must be unchanged the cycle after the stall lifts.
    for (i = 0; i < N; i = i + 1) act[i] = ACT_W'(a_vec[i]);
    @(posedge clk);
    #1;
    n_checked = n_checked + 1;
    if (p0 !== hold0 || p1 !== hold1)
      fail_msg(p0, p1, hold0, hold1);
    $display("[4] en=0 stall          : %0d checks, %0d fail",
             n_checked - n_mark, n_failed - f_mark);

    // -----------------------------------------------------------------------
    // Phase 5: wgt_shift_en = 0 must freeze the weight chain.
    //
    // Noise is driven on the serial weight port while the array streams. If the
    // chain advances anyway the stationary weights walk away from what the
    // scoreboard expects, within 2*N cycles.
    // -----------------------------------------------------------------------
    n_mark = n_checked; f_mark = n_failed;
    wsh = 1'b0;
    for (v = 0; v < 200; v = v + 1) begin
      wser = WGT_W'($urandom_range(0, 255) - 128);
      for (i = 0; i < N; i = i + 1)
        a_vec[i] = $urandom_range(0, 255) - 128;
      set_expect();
      issue(1'b1);
    end
    wser = '0;
    $display("[5] wgt_shift_en gating : %0d checks, %0d fail",
             n_checked - n_mark, n_failed - f_mark);

    // -----------------------------------------------------------------------
    // Phase 6: the horizontal hand-off is exactly one cycle per cell.
    //
    // A reference shift register of the same depth is maintained in the
    // testbench and compared every cycle, so a delay of 0 or 2 is caught on the
    // first vector rather than showing up as a column offset three days later.
    // hen is dropped for a stretch to confirm the hand-off is enable-gated too.
    // -----------------------------------------------------------------------
    n_mark = n_checked; f_mark = n_failed;
    for (v = 0; v < 400; v = v + 1) begin
      hv   = $urandom_range(0, 255) - 128;
      h_in = ACT_W'(hv);
      hen  = !((v > 100) && (v < 110));       // a 9-cycle stall mid-stream

      @(posedge clk);
      #1;

      if (hen) begin
        for (s = M-1; s > 0; s = s - 1) href[s] = href[s-1];
        href[0] = hv;
      end

      for (s = 0; s < M; s = s + 1) begin
        n_checked = n_checked + 1;
        if (hact[s+1] !== ACT_W'(href[s])) begin
          n_failed = n_failed + 1;
          if (n_failed <= 12)
            $display("  FAIL: horizontal cell %0d at v=%0d: got %0d, want %0d",
                     s, v, $signed(hact[s+1]), href[s]);
        end
      end
    end
    hen = 1'b1;
    $display("[6] horizontal delay    : %0d checks, %0d fail",
             n_checked - n_mark, n_failed - f_mark);

    // ---- verdict ------------------------------------------------------------
    $display("---------------------------------------------------------");
    if (n_failed == 0 && n_checked > TILES * PER_TILE) begin
      $display(" PASS   %0d checks, 0 failures", n_checked);
      $display("=========================================================");
      $finish;
    end else begin
      $display(" FAIL   %0d checks, %0d failures", n_checked, n_failed);
      $display("=========================================================");
      $fatal(1, "tb_pe_pair failed");
    end
  end

  // safety net so a hang is a failure rather than a CI timeout
  initial begin
    #20000000;
    $fatal(1, "tb_pe_pair: timeout");
  end

endmodule
