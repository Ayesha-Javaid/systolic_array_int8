// -----------------------------------------------------------------------------
// tb_pack_extract.sv — exhaustive proof of the INT8 DSP packing round-trip
//
// This is the most important testbench in the repo. Every result the array
// produces flows through this arithmetic, and its two hazards (the signed
// borrow, and the -128 A-port overflow) are both *silent*: they yield plausible
// numbers, not X's or errors. So this is tested exhaustively rather than
// sampled.
//
//   quick mode (default) : structured corners + 200k random  -> Icarus, seconds
//   full  mode (+FULL)   : all 255 x 255 x 256 = 16,646,400  -> Verilator
//
// Run:  make test           (quick, both simulators)
//       make test-full      (exhaustive, Verilator)
//
// Style note: everything is declared at module scope with fixed-size arrays.
// That is deliberately plainer than necessary so the same source compiles under
// Icarus 12, Verilator 5 and Vivado's simulator without per-tool ifdefs.
// -----------------------------------------------------------------------------
`timescale 1ns / 1ps

module tb_pack_extract;

  import sa_pkg::*;

  // ---- DUT ------------------------------------------------------------------
  logic signed [DSP_P_W-1:0] p_packed;
  logic signed [GROUP_W-1:0] p0, p1;

  pack_extract #(.K(K_PACK), .OUT_W(GROUP_W)) dut (
    .p_packed (p_packed),
    .p0       (p0),
    .p1       (p1)
  );

  // ---- bookkeeping ----------------------------------------------------------
  integer n_checked;
  integer n_failed;
  integer n_before;
  integer i, j, k, t, w1v, w0v, av;
  bit     full_mode;

  // group stimulus (fixed size: sa_pkg::CASCADE_DEPTH)
  integer gw1 [0:CASCADE_DEPTH-1];
  integer gw0 [0:CASCADE_DEPTH-1];
  integer ga  [0:CASCADE_DEPTH-1];

  integer corners     [0:10];
  integer act_corners [0:11];

  // ---- model ----------------------------------------------------------------
  // One DSP48E1 multiply with the packed A operand.
  function automatic logic signed [DSP_P_W-1:0] dsp_mul(
      input integer w1i, input integer w0i, input integer ai);
    logic signed [DSP_A_W-1:0] a_port;
    logic signed [DSP_B_W-1:0] b_port;
    begin
      a_port = sa_pkg::pack_weights(WGT_W'(w1i), WGT_W'(w0i));
      b_port = DSP_B_W'(ACT_W'(ai));
      dsp_mul = DSP_P_W'(a_port * b_port);
    end
  endfunction

  task automatic check_one(input integer w1i, input integer w0i, input integer ai);
    integer exp0, exp1;
    begin
      p_packed = dsp_mul(w1i, w0i, ai);
      #1;
      exp0 = w0i * ai;
      exp1 = w1i * ai;
      n_checked = n_checked + 1;
      if (p0 !== GROUP_W'(exp0) || p1 !== GROUP_W'(exp1)) begin
        n_failed = n_failed + 1;
        if (n_failed <= 20)
          $display("  FAIL w1=%0d w0=%0d a=%0d : got p1=%0d p0=%0d, want p1=%0d p0=%0d (P=%h)",
                   w1i, w0i, ai, p1, p0, exp1, exp0, p_packed);
      end
    end
  endtask

  // Accumulate CASCADE_DEPTH packed products, extract once. This is the
  // property that makes ARCHITECTURE.md §4 true: the 17-bit low field must
  // survive 4 accumulations without bleeding into the high field.
  task automatic check_group;
    integer gi, exp0, exp1;
    logic signed [DSP_P_W-1:0] acc;
    begin
      acc  = '0;
      exp0 = 0;
      exp1 = 0;
      for (gi = 0; gi < CASCADE_DEPTH; gi = gi + 1) begin
        acc  = acc + dsp_mul(gw1[gi], gw0[gi], ga[gi]);
        exp0 = exp0 + gw0[gi] * ga[gi];
        exp1 = exp1 + gw1[gi] * ga[gi];
      end
      p_packed = acc;
      #1;
      n_checked = n_checked + 1;
      if (p0 !== GROUP_W'(exp0) || p1 !== GROUP_W'(exp1)) begin
        n_failed = n_failed + 1;
        if (n_failed <= 20)
          $display("  FAIL group: got p1=%0d p0=%0d, want p1=%0d p0=%0d (P=%h)",
                   p1, p0, exp1, exp0, acc);
      end
    end
  endtask

  // ---- stimulus -------------------------------------------------------------
  initial begin
    n_checked = 0;
    n_failed  = 0;
    full_mode = $test$plusargs("FULL");

    corners[0]=-127; corners[1]=-126; corners[2]=-64; corners[3]=-2;
    corners[4]=-1;   corners[5]=0;    corners[6]=1;   corners[7]=2;
    corners[8]=64;   corners[9]=126;  corners[10]=127;

    act_corners[0]=-128; act_corners[1]=-127; act_corners[2]=-126;
    act_corners[3]=-64;  act_corners[4]=-2;   act_corners[5]=-1;
    act_corners[6]=0;    act_corners[7]=1;    act_corners[8]=2;
    act_corners[9]=64;   act_corners[10]=126; act_corners[11]=127;

    $display("=========================================================");
    $display(" tb_pack_extract : %0s mode", full_mode ? "EXHAUSTIVE" : "quick");
    $display("   K_PACK=%0d  CASCADE_DEPTH=%0d  PROD_MAX=%0d",
             K_PACK, CASCADE_DEPTH, PROD_MAX);
    $display("=========================================================");

    // ---- 1. corner grid ----------------------------------------------------
    n_before = n_checked;
    for (i = 0; i <= 10; i = i + 1)
      for (j = 0; j <= 10; j = j + 1)
        for (k = 0; k <= 11; k = k + 1)
          check_one(corners[i], corners[j], act_corners[k]);
    $display("[1] corner grid         : %0d checks, %0d fail", n_checked-n_before, n_failed);

    // ---- 2. the signed-borrow hazard ---------------------------------------
    // Low product negative, high product positive: exactly the cases a naive
    // bit-slice gets wrong. If the +P[K-1] correction is deleted, this section
    // fails on every single check.
    n_before = n_checked;
    for (w1v = 1; w1v <= 127; w1v = w1v + 3)
      for (w0v = -127; w0v <= -1; w0v = w0v + 3)
        for (av = 1; av <= 127; av = av + 7)
          check_one(w1v, w0v, av);
    $display("[2] signed-borrow cases : %0d checks, %0d fail", n_checked-n_before, n_failed);

    // ---- 3. cascade-group accumulation at the overflow boundary ------------
    n_before = n_checked;

    // 3a. all-maximum-positive low field: 4 * 127 * 127 = 64516
    for (i = 0; i < CASCADE_DEPTH; i = i + 1) begin
      gw1[i] = 127; gw0[i] = 127; ga[i] = 127;
    end
    check_group();

    // 3b. the true worst case: 4 * 127 * -128 = -65024, vs the 17-bit signed
    //     floor of -65536. 512 of margin. With w=-128 permitted this would be
    //     -65536 exactly and the field would wrap.
    for (i = 0; i < CASCADE_DEPTH; i = i + 1) begin
      gw1[i] = -127; gw0[i] = 127; ga[i] = -128;
    end
    check_group();

    // 3c. opposite polarity
    for (i = 0; i < CASCADE_DEPTH; i = i + 1) begin
      gw1[i] = 127; gw0[i] = -127; ga[i] = 127;
    end
    check_group();

    // 3d. randomised groups
    for (t = 0; t < 20000; t = t + 1) begin
      for (i = 0; i < CASCADE_DEPTH; i = i + 1) begin
        gw1[i] = $urandom_range(0, 254) - 127;
        gw0[i] = $urandom_range(0, 254) - 127;
        ga[i]  = $urandom_range(0, 255) - 128;
      end
      check_group();
    end
    $display("[3] cascade-group accum : %0d checks, %0d fail", n_checked-n_before, n_failed);

    // ---- 4. exhaustive or randomised ---------------------------------------
    n_before = n_checked;
    if (full_mode) begin
      for (w1v = -127; w1v <= 127; w1v = w1v + 1) begin
        for (w0v = -127; w0v <= 127; w0v = w0v + 1)
          for (av = -128; av <= 127; av = av + 1)
            check_one(w1v, w0v, av);
        if (((w1v + 127) % 32) == 0)
          $display("    ... w1 = %0d / 127   (%0d checks so far)", w1v, n_checked);
      end
      $display("[4] EXHAUSTIVE          : %0d checks, %0d fail", n_checked-n_before, n_failed);
    end else begin
      for (t = 0; t < 200000; t = t + 1)
        check_one($urandom_range(0,254) - 127,
                  $urandom_range(0,254) - 127,
                  $urandom_range(0,255) - 128);
      $display("[4] randomised          : %0d checks, %0d fail", n_checked-n_before, n_failed);
    end

    // ---- verdict ------------------------------------------------------------
    $display("---------------------------------------------------------");
    if (n_failed == 0) begin
      $display(" PASS   %0d checks, 0 failures", n_checked);
      $display("=========================================================");
      $finish;
    end else begin
      $display(" FAIL   %0d checks, %0d failures", n_checked, n_failed);
      $display("=========================================================");
      $fatal(1, "tb_pack_extract failed");
    end
  end

endmodule
