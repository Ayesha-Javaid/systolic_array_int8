`timescale 1ns / 1ps

module tb_stagger_in;
  import sa_pkg::*;

  localparam int NR    = ROWS;        // 16
  localparam int WA    = ACT_W;       // 8
  localparam int MAXD  = NR - 1;      // deepest row


  localparam int HIST  = 64;
  localparam int HMASK = HIST - 1;


  localparam int MINCHK = 95000;

  // ---- clock. No reset in this testbench, deliberately: stagger_in has no
  // ---- reset port and phase 0 is about what it does without one. -----------
  logic clk = 1'b0;
  logic en  = 1'b1;
  initial forever #5 clk = ~clk;      // 100 MHz

  // ---- DUT ------------------------------------------------------------------
  logic signed [NR-1:0][WA-1:0] act_in,  act_out;
  logic                         swap_in;
  logic        [NR-1:0]         swap_out;

  stagger_in #(.NROWS(NR), .W_ACT(WA)) dut (
    .clk      (clk),
    .en       (en),
    .act_in   (act_in),
    .swap_in  (swap_in),
    .act_out  (act_out),
    .swap_out (swap_out)
  );

  // ---- drive state and the reference ---------------------------------------
  logic [WA-1:0] dr_act [0:NR-1];     // what is being driven this cycle
  logic          dr_swp;

  logic [WA-1:0] hact [0:HIST-1][0:NR-1];
  logic          hswp [0:HIST-1];

  logic signed [NR-1:0][WA-1:0] t_act;

  integer        ecyc;                // enabled cycles completed, monotonic;
                                      // the history index
  integer        mcyc;                // enabled cycles since the last arm();
                                      // the measurement index. Kept separate
                                      // from ecyc because the history must not
                                      // be rewound when a measurement starts.

  // ---- marker-measurement bookkeeping (phases 1, 4 and 5) ------------------
  bit            obs;                 // record instead of compare
  integer        a_n   [0:NR-1];      // times act_out went non-zero
  integer        a_at  [0:NR-1];      // enabled cycle of the first
  logic [WA-1:0] a_val [0:NR-1];
  integer        s_n   [0:NR-1];      // times swap_out went high
  integer        s_at  [0:NR-1];

  // Per-row activation marker: distinct, non-zero, and not a power of two, so a
  // row reading a neighbour's input fails on the VALUE and not only on the
  // arrival index.
  logic [WA-1:0] mk [0:NR-1];

  // Counters used INSIDE tasks. Nothing enclosing a task call may use them.
  integer i, s;
  // Counters for loops that DO enclose task calls.
  integer oi, oj, ok, om;

  integer n_checked, n_failed;
  integer c_before, f_before;

  integer exp_i;
  logic [WA-1:0] exp_a;
  logic          exp_s;


  function automatic logic [WA-1:0] rand_act;
    logic [31:0] r;
    begin
      r = $urandom;
      rand_act = r[24:17];
    end
  endfunction

  function automatic logic rand_bit;
    logic [31:0] r;
    begin
      r = $urandom;
      rand_bit = r[29];
    end
  endfunction

  task automatic set_rand;
    begin
      for (i = 0; i < NR; i = i + 1) dr_act[i] = rand_act();
      dr_swp = rand_bit();
    end
  endtask

  task automatic set_zero;
    begin
      for (i = 0; i < NR; i = i + 1) dr_act[i] = '0;
      dr_swp = 1'b0;
    end
  endtask

  task automatic set_marker;
    begin
      for (i = 0; i < NR; i = i + 1) dr_act[i] = mk[i];
    end
  endtask

  task automatic set_const(input logic [WA-1:0] v);
    begin
      for (i = 0; i < NR; i = i + 1) dr_act[i] = v;
    end
  endtask

  task automatic check_all(input integer tag, input integer base);
    begin
      for (i = 0; i < NR; i = i + 1) begin
        exp_i = base - i;
        if (i == 0) begin
          exp_a = dr_act[0];
          exp_s = dr_swp;
        end else if (exp_i < 0) begin
          exp_a = '0;             // power-up state; no reset exists
          exp_s = 1'b0;
        end else begin
          exp_a = hact[exp_i & HMASK][i];
          exp_s = hswp[exp_i & HMASK];
        end

        n_checked = n_checked + 1;
        if (act_out[i] !== exp_a) begin
          n_failed = n_failed + 1;
          if (n_failed - f_before <= 10)
            $display("  FAIL: act row=%0d %0s-edge ecyc=%0d: got %0h, want %0h",
                     i, (tag == 0) ? "pre" : "post", ecyc, act_out[i], exp_a);
        end

        n_checked = n_checked + 1;
        if (swap_out[i] !== exp_s) begin
          n_failed = n_failed + 1;
          if (n_failed - f_before <= 10)
            $display("  FAIL: swap row=%0d %0s-edge ecyc=%0d: got %0b, want %0b",
                     i, (tag == 0) ? "pre" : "post", ecyc, swap_out[i], exp_s);
        end
      end
    end
  endtask


  task automatic step(input bit e, input bit chk);
    begin
      en = e;

      t_act = '0;                      // whole-variable init before any element
      for (i = 0; i < NR; i = i + 1) t_act[i] = dr_act[i];
      act_in  = t_act;
      swap_in = dr_swp;

      #1;
      if (chk) check_all(0, ecyc);

      
      if (obs && e) begin
        for (i = 0; i < NR; i = i + 1) begin
          if (act_out[i] !== {WA{1'b0}}) begin
            a_n[i] = a_n[i] + 1;
            if (a_n[i] == 1) begin
              a_at[i]  = mcyc;
              a_val[i] = act_out[i];
            end
          end
          if (swap_out[i] === 1'b1) begin
            s_n[i] = s_n[i] + 1;
            if (s_n[i] == 1) s_at[i] = mcyc;
          end
        end
      end

      @(posedge clk);

      if (e) begin
        for (i = 0; i < NR; i = i + 1) hact[ecyc & HMASK][i] = dr_act[i];
        hswp[ecyc & HMASK] = dr_swp;
      end

      #1;
      if (chk) check_all(1, e ? ecyc + 1 : ecyc);

      if (e) begin
        ecyc = ecyc + 1;
        mcyc = mcyc + 1;
      end
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


  // ---------------------------------------------------------------------------
  // Stimulus
  // ---------------------------------------------------------------------------
  initial begin
    n_checked = 0;
    n_failed  = 0;
    c_before  = 0;
    f_before  = 0;
    ecyc      = 0;
    mcyc      = 0;
    obs       = 1'b0;
    exp_i     = 0;
    exp_a     = '0;
    exp_s     = 1'b0;

    // Whole-variable initialisation of everything that feeds a port, before any
    // element-wise write anywhere.
    act_in  = '0;
    t_act   = '0;
    swap_in = 1'b0;

    for (i = 0; i < NR; i = i + 1) begin
      dr_act[i] = '0;
      a_n[i]    = 0;
      a_at[i]   = -1;
      a_val[i]  = '0;
      s_n[i]    = 0;
      s_at[i]   = -1;
      // 0x31, 0x3C, 0x47 ... 0xD6: distinct, non-zero, none a power of two.
      mk[i]     = 8'h31 + WA'(i) * 8'h0B;
    end
    dr_swp = 1'b0;
    for (i = 0; i < HIST; i = i + 1) begin
      for (s = 0; s < NR; s = s + 1) hact[i][s] = '0;
      hswp[i] = 1'b0;
    end

    $display("=========================================================");
    $display(" tb_stagger_in : %0d rows, depths 0..%0d, W = %0d ({swap, act})",
             NR, MAXD, WA + 1);
    $display("   no reset port -- the power-up state IS the reset");
    $display("=========================================================");

    banner("Phase 0: power-up state is zero, row 0 is a wire, en=0 lets nothing in");
    set_rand();
    act_in  = '0;
    t_act   = '0;
    for (i = 0; i < NR; i = i + 1) t_act[i] = dr_act[i];
    act_in  = t_act;
    swap_in = dr_swp;
    #1;
    check_all(0, ecyc);
    for (ok = 0; ok < 4; ok = ok + 1) begin
      set_zero();
      step(1'b0, 1'b1);
    end
    for (ok = 0; ok < NR + 6; ok = ok + 1) begin
      set_rand();
      dr_swp = 1'b1;                 // a held pulse must not enter either
      step(1'b0, 1'b1);
    end
    set_zero();
    for (ok = 0; ok < 2; ok = ok + 1) step(1'b0, 1'b1);
    report("[0] power-up / row 0 wire", 1700);

    
    $finish;
  end

  // Wall-clock guard, so a hang is a failure rather than a timeout in CI.
  initial begin
    #10_000_000;
    $display(" FAIL   tb_stagger_in: timeout");
    $fatal(1, "tb_stagger_in: timeout");
  end

endmodule
