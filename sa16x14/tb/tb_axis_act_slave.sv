`timescale 1ns / 1ps

module tb_axis_act_slave;
  import sa_pkg::*;

  localparam int NR   = ROWS;                 // 16 rows = TDATA byte lanes
  localparam int NC   = COLS;                 // 14 output columns
  localparam int NP   = NC / 2;               // 7 column-pairs
  localparam int NW   = 2 * NR;               // 32-byte weight chain per port
  localparam int NG   = CASCADE_GROUPS;       // 4
  localparam int ND   = CASCADE_DEPTH;        // 4
  localparam int TDW  = NR * ACT_W;           // 128: one whole vector per beat
  localparam int CAP  = 2;                    // skid_buf's capacity

  // sa_datapath's numbers, all in ENABLED cycles. Day 10 measured them; this
  // testbench's job is to show they are unchanged when `en` stops being a knob.
  localparam int LAT     = NR + 2 + NP - 1;   // 24
  localparam int STAG    = NR - 1;            // 15
  localparam int SETTLE  = STAG + LAT + 4;    // 43
  localparam int SWAPWIN = NR + NP - 1;       // 22
  localparam int PERIOD  = 3 * NR + NP;       // 55 vectors
  localparam int RST_IDLE = (NR - 1) + SWAPWIN;  // 37, day 10's number

  // Vector-file limits, duplicated in model/gen_vectors.py.
  localparam int MAXT  = 40;
  localparam int MAXV  = 2048;
  localparam int MAGIC = 32'h00C0FFEE;
  localparam int P2T   = 6;        // tiles taken from the set; the whole 24 would
                                   // triple this testbench's runtime and phase 2
                                   // is about backpressure, not about coverage of
                                   // the packing (tb_sa_array phase 4 owns that)

  localparam int MAXB  = 512;      // beats held by the software queue at a time

  // Global backstop. MEASURED, then set about 5% below -- day 9's lesson.
  localparam int MINCHK = 58000;   // measured 62,618

  // ---- clock / reset --------------------------------------------------------
  logic clk   = 1'b0;
  logic rst_n = 1'b0;
  initial forever #5 clk = ~clk;

  // ---- AXI4-Stream master side (driven here) --------------------------------
  logic                 tv;
  logic [TDW-1:0]       td;
  logic                 tl;
  logic [0:0]           tu;
  logic                 trdy;

  // ---- the two controls day 12 and day 13 will own --------------------------
  logic                 flush;
  logic                 dnr;

  // ---- the slave's datapath face -------------------------------------------
  logic                            a_en;
  logic                            a_vld;
  logic signed [NR-1:0][ACT_W-1:0] a_vec;
  logic [0:0]                      a_user;
  logic                            a_last;

  // ---- the datapath ---------------------------------------------------------
  logic                               wsh;
  logic signed [NP-1:0][WGT_W-1:0]    wser;
  logic signed [NP-1:0][WGT_W-1:0]    wgt_o;
  logic                               sw_force;
  logic                               swp;
  logic signed [NC-1:0][COLSUM_W-1:0] sout;
  logic signed [NR-1:0][ACT_W-1:0]    act_o;
  logic        [NR-1:0]               swp_o;

  // The sequencer's two sources for the swap pulse, as day 13 will have them:
  // the in-band TUSER marker while a stream is running, and its own control
  // while the stream is flushed and a tile is being installed. act_user is
  // already qualified by act_vld, so the two can never collide.
  assign swp = sw_force | a_user[0];

  axis_act_slave #(
    .NROWS   (NR),
    .USER_W  (1),
    .TDATA_W (TDW)
  ) u_s (
    .clk           (clk),
    .rst_n         (rst_n),
    .s_axis_tvalid (tv),
    .s_axis_tready (trdy),
    .s_axis_tdata  (td),
    .s_axis_tlast  (tl),
    .s_axis_tuser  (tu),
    .flush         (flush),
    .dn_ready      (dnr),
    .act_en        (a_en),
    .act_vld       (a_vld),
    .act_vec       (a_vec),
    .act_user      (a_user),
    .act_last      (a_last)
  );

  sa_datapath #(
    .NROWS     (NR),
    .NCOLS     (NC),
    .GROUPS    (NG),
    .DEPTH     (ND),
    .K         (K_PACK),
    .GRP_W     (GROUP_W),
    .OUT_W     (COLSUM_W),
    .CLAMP_WGT (1'b1)
  ) u_d (
    .clk          (clk),
    .rst_n        (rst_n),
    .en           (a_en),
    .wgt_shift_en (wsh),
    .wgt_in       (wser),
    .wgt_out      (wgt_o),
    .act_in       (a_vec),
    .swap_in      (swp),
    .sum_out      (sout),
    .act_out      (act_o),
    .swap_out     (swp_o)
  );


  integer qv   [0:MAXB-1][0:NR-1];   // the activation vector of each beat
  bit     qu   [0:MAXB-1];           // TUSER[0]: "swap during this vector"
  bit     ql   [0:MAXB-1];           // TLAST
  bit     qf   [0:MAXB-1];           // score against the file, not the model
  integer qe   [0:MAXB-1][0:NC-1];   // the file's expectation, when qf
  integer qlane[0:MAXB-1];           // >= 0: a one-hot lane probe, for the message
  integer nb, b_tx, b_rx;
  bit     tv_hold;                   // the offered beat was not taken last edge

  // ---- the weight model ----------------------------------------------------
  integer WA  [0:NR-1][0:NC-1];      // host tile A
  integer WB  [0:NR-1][0:NC-1];      // host tile B (being loaded)
  integer RW  [0:NR-1][0:NC-1];      // ACTIVE weights, as modelled
  integer RWI [0:NR-1][0:NC-1];      // the same, for the "ideally timed" model
  integer PW  [0:NR-1][0:NC-1];      // pending (clamped) weights
  integer pend_e, pend_ei;           // enabled cycle at which RW/RWI take PW
  integer lseq [0:NW-1][0:NP-1];     // one tile's byte stream, per port
  integer lptr;                      // -1 = chain idle

  // ---- the scoreboard, indexed in ENABLED cycles ---------------------------
  integer ecyc;                      // enabled cycles observed so far
  integer pres  [0:NR-1];            // the vector presented this enabled cycle
  integer e_now [0:NC-1];
  integer e_idl [0:NC-1];
  integer e_ref [0:NC-1];
  integer expq  [0:LAT][0:NC-1];
  integer expqi [0:LAT][0:NC-1];
  bit     vldq  [0:LAT];
  integer bidq  [0:LAT];             // which beat each queued expectation is
  integer lanq  [0:LAT];             // its lane probe, for the message
  bit     scoring;
  integer n_diff;

  integer hold_s [0:NC-1];
  bit     en_s;

  integer n_checked, n_failed;
  integer c_before, f_before;
  integer mism, n_same;

  // Counters used INSIDE tasks. Nothing enclosing a task call may use them.
  integer i, j, s;
  // Counters for loops that DO enclose task calls.
  integer oi, oj, ok, om;
  // Counters used inside install(), stream() and enq_* only.
  integer mi, mj;
  // Phase-local counters that call nothing from their loop body.
  integer p0n, p3n, p3c, p6k, rr, vmode, rmode, bnext, win_e, v0, nt, pv, vi;
  integer sg;   // stream()'s unconditional progress guard
  logic   hold_tr;
  bit     offer_b;

  // Temporaries for whole-variable port writes.
  logic [TDW-1:0]                  td_t;
  logic signed [NP-1:0][WGT_W-1:0] wser_t;

  // ---- golden-model vector set --------------------------------------------
  logic [31:0]                mmem [0:15];
  logic [7:0]                 wmem [0:MAXT*ROWS*COLS-1];
  logic [7:0]                 amem [0:MAXV*ROWS-1];
  logic signed [COLSUM_W-1:0] emem [0:MAXV*COLS-1];

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

endmodule
