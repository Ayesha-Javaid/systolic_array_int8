// -----------------------------------------------------------------------------
// tb_dsp_pack_mac.sv — clocked test of one CASCADE_DEPTH-deep DSP cascade group
//
// Builds the real structure from ARCHITECTURE.md §4: four dsp_pack_mac cells
// chained PCOUT -> PCIN, each holding two stationary weights and receiving its
// own row's activation. Extracts the group result and compares against an
// integer golden sum.
//
// ---------------------------------------------------------------------------
// The skew requirement, which is the whole point of this testbench
// ---------------------------------------------------------------------------
// The first version of this testbench drove all four cells on the same clock
// edge and failed on 1999 of 2000 vectors. That was the testbench being wrong,
// not the RTL, and the reason is worth writing down because it is the entire
// justification for the stagger network in §5:
//
//   cell g's product reaches its PREG three edges after its inputs are applied.
//   cell g+1 adds that PREG through the *unregistered* PCIN path, so cell g+1's
//   own product must reach ITS multiplier register on the same edge. That
//   happens only if cell g+1's inputs are applied one cycle LATER than cell g's.
//
// So row i must receive its activation i cycles after row 0. The cascade does
// not tolerate a flat broadcast. This testbench therefore issues each vector
// with a per-row skew of i cycles -- a 4-deep miniature of the input stagger
// network -- and a regression here would catch anyone "simplifying" that away.
//
// Also covered:
//   * back-to-back streaming: a new dot product issued every cycle, which is the
//     steady state the array runs in.
//   * stationary weights held across a whole tile, reloaded between tiles, with
//     a drain in between so no vector ever sees two different weight sets.
// -----------------------------------------------------------------------------
`timescale 1ns / 1ps

module tb_dsp_pack_mac;

  import sa_pkg::*;

  localparam int N        = CASCADE_DEPTH;   // 4 cells

  // Issue cycle -> result at pcout, in clock edges.
  //   cell 0 is driven on edge 0 and its PREG is valid after edge 2
  //     (edge 0 captures A/B, edge 1 forms M, edge 2 forms P)
  //   cell i is driven i cycles later, so its PREG is valid after edge i+2
  //   the last cell is i = N-1, giving N+1
  // Counting the capture edge twice here is an easy off-by-one, and it makes
  // every result look correct while being attributed to the wrong vector.
  localparam int LAT      = N + 1;           // = 5 for N = 4
  localparam int TILES    = 40;
  localparam int PER_TILE = 60;

  // ---- clock / reset --------------------------------------------------------
  logic clk = 1'b0;
  logic rst_n = 1'b0;
  logic en = 1'b1;
  initial forever #5 clk = ~clk;             // 100 MHz
                                             // `initial forever` rather than
                                             // `always`, so Verilator does not
                                             // classify it as sequential logic
                                             // and fire BLKSEQ.

  // ---- DUT: a cascade of N cells -------------------------------------------
  //
  // PACKED arrays, not unpacked -- a portability requirement, not a
  // style preference. See the whole-variable initialisation note below and
  // docs/NOTES-tooling.md: element-wise writes to a port-connected variable
  // can silently fail to propagate under Verilator while Icarus simulates it
  // correctly.
  //
  // Repo convention: assign the whole vector once at init, and never
  // rely on a port-connected variable that has only ever been written
  // element-by-element.
  logic signed [N-1:0][WGT_W-1:0]   w0;
  logic signed [N-1:0][WGT_W-1:0]   w1;
  logic signed [N-1:0][ACT_W-1:0]   act;
  logic signed [N:0]  [DSP_P_W-1:0] casc;
  logic signed [N-1:0][ACT_W-1:0]   act_o;

  // Cascade seed and the top cell's acc_first are TB-driven so that the
  // acc_first control path can actually be exercised. With casc[0] tied to
  // zero, acc_first makes no observable difference on the top cell -- a
  // mutation that mis-aligns it by a pipeline stage survives unnoticed. This
  // is the multi-tile accumulation path (K > ROWS), so it matters.
  logic signed [DSP_P_W-1:0] seed;
  logic                      af0;
  assign casc[0] = seed;

  genvar g;
  generate
    for (g = 0; g < N; g++) begin : g_cell
      dsp_pack_mac #(.K(K_PACK)) u_cell (
        .clk       (clk),
        .rst_n     (rst_n),
        .en        (en),
        .w0        (w0[g]),
        .w1        (w1[g]),
        .act       (act[g]),
        .acc_first (g == 0 ? af0 : 1'b0),    // only the top cell may start fresh
        .pcin      (casc[g]),
        .pcout     (casc[g+1]),
        .act_out   (act_o[g])
      );
    end
  endgenerate

  logic signed [GROUP_W-1:0] p0, p1;
  pack_extract #(.K(K_PACK), .OUT_W(GROUP_W)) u_ext (
    .p_packed (casc[N]),
    .p0       (p0),
    .p1       (p1)
  );

  // ---- the stagger: ahist[d][i] = row i's activation for the vector issued ---
  // ---- d cycles ago. Cell i is driven from ahist[i][i].                   ---
  integer ahist [0:N-1][0:N-1];

  // ---- scoreboard: expectation for the vector issued LAT cycles ago ---------
  integer exp0_q [0:LAT];
  integer exp1_q [0:LAT];
  bit     vld_q  [0:LAT];

  integer n_checked, n_failed, i, d, tile, v, s;
  integer a_vec [0:N-1];
  integer n_before, seed_lo, seed_hi;
  integer e0, e1;

  // Issue one vector: push its per-row activations into the skew history, push
  // its expectation into the scoreboard, advance one clock, then check whatever
  // result is arriving.
  task automatic issue(input bit valid);
    begin
      // shift the activation skew history
      for (s = N-1; s > 0; s = s - 1)
        for (i = 0; i < N; i = i + 1)
          ahist[s][i] = ahist[s-1][i];
      for (i = 0; i < N; i = i + 1)
        ahist[0][i] = a_vec[i];

      // drive each cell from its own depth in the history
      for (i = 0; i < N; i = i + 1)
        act[i] = ACT_W'(ahist[i][i]);

      // shift the scoreboard
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
        if (p0 !== GROUP_W'(exp0_q[LAT]) || p1 !== GROUP_W'(exp1_q[LAT])) begin
          n_failed = n_failed + 1;
          if (n_failed <= 10)
            $display("  FAIL: got p1=%0d p0=%0d, want p1=%0d p0=%0d",
                     p1, p0, exp1_q[LAT], exp0_q[LAT]);
        end
      end
    end
  endtask

  initial begin
    n_checked = 0;
    n_failed  = 0;
    e0 = 0; e1 = 0;
    for (i = 0; i <= LAT; i = i + 1) begin
      exp0_q[i] = 0; exp1_q[i] = 0; vld_q[i] = 1'b0;
    end
    // Whole-variable initialisation, and it has to stay whole-variable.
    //
    // In Verilator 5.020, element-wise writes to a variable that has never been
    // assigned as a whole do not propagate into a submodule port feeding
    // combinational logic: the instance reads the reset value forever while
    // Icarus simulates it correctly. Assigning the complete vector once here
    // establishes the dependency, after which per-element writes propagate
    // normally in both simulators.
    //
    // (A comment line must not begin with the word V-e-r-i-l-a-t-o-r either,
    //  or the tool parses it as an unknown lint pragma and refuses to compile.)
    //
    // Writing `for (i...) w0[i] = '0;` instead of `w0 = '0;` is enough to
    // reintroduce the bug, and it fails silently. See docs/NOTES-tooling.md.
    w0  = '0;
    w1  = '0;
    act = '0;
    for (i = 0; i < N; i = i + 1) begin
      a_vec[i] = 0;
      for (d = 0; d < N; d = d + 1) ahist[d][i] = 0;
    end
    seed = '0;
    af0  = 1'b1;

    $display("=========================================================");
    $display(" tb_dsp_pack_mac : %0d-deep cascade, issue->result %0d cycles", N, LAT);
    $display("   %0d tiles x %0d vectors, per-row skew applied", TILES, PER_TILE);
    $display("=========================================================");

    repeat (4) @(posedge clk);
    rst_n = 1'b1;
    repeat (2) @(posedge clk);

    for (tile = 0; tile < TILES; tile = tile + 1) begin
      // ---- load stationary weights for this tile ---------------------------
      for (i = 0; i < N; i = i + 1) begin
        case (tile % 4)
          // worst-case tiles: drive the 17-bit low field to its limits
          0: begin w0[i] =  127; w1[i] =  127; end
          1: begin w0[i] =  127; w1[i] = -127; end
          2: begin w0[i] = -127; w1[i] =  127; end
          default: begin
            w0[i] = WGT_W'($urandom_range(0, 254) - 127);
            w1[i] = WGT_W'($urandom_range(0, 254) - 127);
          end
        endcase
      end

      // ---- stream activation vectors ---------------------------------------
      for (v = 0; v < PER_TILE; v = v + 1) begin
        e0 = 0;
        e1 = 0;
        for (i = 0; i < N; i = i + 1) begin
          // bias towards the extremes: |w*a| is maximal at a = -128
          a_vec[i] = (tile % 4 < 3) && (v % 3 == 0)
                     ? -128
                     : $urandom_range(0, 255) - 128;
          e0 = e0 + $signed(w0[i]) * a_vec[i];
          e1 = e1 + $signed(w1[i]) * a_vec[i];
        end
        issue(1'b1);
      end

      // ---- drain before the next weight load -------------------------------
      // Zero activations, so results are 0 whichever weights are loaded. This
      // guarantees no vector ever straddles a weight change.
      for (i = 0; i < N; i = i + 1) a_vec[i] = 0;
      e0 = 0;
      e1 = 0;
      for (d = 0; d < LAT + N; d = d + 1) issue(1'b1);
    end

    // -------------------------------------------------------------------------
    // Directed phase: acc_first with a non-zero cascade seed.
    //
    // af0 = 1 -> the top cell starts a fresh accumulation and must IGNORE pcin
    // af0 = 0 -> it must ADD pcin, which is how accumulation chains across
    //            cascade groups and across tiles when K > ROWS
    //
    // seed is held constant for the whole phase so its arrival timing cannot
    // confound the result; only af0 varies.
    // -------------------------------------------------------------------------
    seed_lo = 1234;
    seed_hi = -5678;
    seed    = DSP_P_W'((seed_hi <<< K_PACK) + seed_lo);

    for (i = 0; i < N; i = i + 1) begin
      w0[i] = WGT_W'(WGT_MAX);
      w1[i] = WGT_W'(WGT_MIN);
    end
    for (i = 0; i < N; i = i + 1) a_vec[i] = 0;
    e0 = 0; e1 = 0;
    af0 = 1'b1;
    for (d = 0; d < LAT + N; d = d + 1) issue(1'b0);   // settle, unscored

    n_before = n_checked;
    for (v = 0; v < 400; v = v + 1) begin
      af0 = (v % 2 == 0);
      e0 = 0;
      e1 = 0;
      for (i = 0; i < N; i = i + 1) begin
        a_vec[i] = $urandom_range(0, 255) - 128;
        e0 = e0 + $signed(w0[i]) * a_vec[i];
        e1 = e1 + $signed(w1[i]) * a_vec[i];
      end
      // af0 is sampled with THIS vector's row-0 activation, so the expectation
      // for this vector follows the af0 driven now.
      if (!af0) begin
        e0 = e0 + seed_lo;
        e1 = e1 + seed_hi;
      end
      issue(1'b1);
    end
    for (i = 0; i < N; i = i + 1) a_vec[i] = 0;
    af0 = 1'b1; e0 = 0; e1 = 0;
    for (d = 0; d < LAT + N; d = d + 1) issue(1'b0);
    $display("[acc_first x seed]    : %0d checks, %0d fail", n_checked - n_before, n_failed);

    $display("---------------------------------------------------------");
    if (n_failed == 0 && n_checked > TILES * PER_TILE) begin
      $display(" PASS   %0d dot products checked, 0 failures", n_checked);
      $display("=========================================================");
      $finish;
    end else begin
      $display(" FAIL   %0d checked, %0d failures", n_checked, n_failed);
      $display("=========================================================");
      $fatal(1, "tb_dsp_pack_mac failed");
    end
  end

  // safety net so a hang is a failure rather than a CI timeout
  initial begin
    #5000000;
    $fatal(1, "tb_dsp_pack_mac: timeout");
  end

endmodule
