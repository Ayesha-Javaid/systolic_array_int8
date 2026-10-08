`timescale 1ns / 1ps

module stagger_in
  import sa_pkg::*;
#(
  parameter int NROWS = ROWS,    // = 16; row i is delayed by i
  parameter int W_ACT = ACT_W    // = 8
) (
  input  logic                               clk,
  input  logic                               en,      // advance; the array's own

  // ---- flat input: one whole activation vector per enabled cycle -----------
  input  logic signed [NROWS-1:0][W_ACT-1:0] act_in,
  // One-cycle pulse, presented one enabled cycle before the first activation
  // of the new tile. See the header: the lead is the caller's to create.
  input  logic                               swap_in,

  // ---- staggered output: row i is i enabled cycles behind row 0 ------------
  output logic signed [NROWS-1:0][W_ACT-1:0] act_out,
  output logic        [NROWS-1:0]            swap_out
);

  // The line word. The swap is the top bit; the activation occupies the low
  // W_ACT. Written once, here, so the packing lives in one place exactly as
  // sa_pkg::pack_weights does for the A port.
  localparam int W_LINE = W_ACT + 1;

  // Exported so day 10 can state the array's total latency in terms of this
  // rather than re-deriving it: the last row leaves this network NROWS-1
  // enabled cycles after the vector was presented.
  localparam int LATENCY = NROWS - 1;

  initial begin
    if (NROWS < 1)
      $fatal(1, "stagger_in: NROWS=%0d must be at least 1", NROWS);
    if (W_ACT < 1)
      $fatal(1, "stagger_in: W_ACT=%0d must be at least 1", W_ACT);
  end

  genvar gr;
  generate
    for (gr = 0; gr < NROWS; gr++) begin : g_row
      // Row gr's delay is gr, which makes row 0 a delay_line of DEPTH 0 --
      // a wire by that module's contract, measured by tb_delay_line phase 1
      // and again here by phase 1.
      localparam int RD = gr;

      // delay_line's ports are unsigned; act_in is signed. These are
      // bit-for-bit copies at the same width and the sign is reapplied on the
      // output, in the same style as column_pair's alignment delays.
      logic [W_LINE-1:0] din_w, dout_w;

      // Whole-variable assignment, not element-wise: a variable written
      // piecewise does not propagate through a port into combinational logic
      // under the lint/sim tool of docs/NOTES-tooling.md, and fails silently.
      assign din_w = {swap_in, act_in[gr]};

      delay_line #(.W(W_LINE), .DEPTH(RD)) u_dl (
        .clk  (clk),
        .en   (en),
        .din  (din_w),
        .dout (dout_w)
      );

      assign act_out [gr] = dout_w[W_ACT-1:0];
      assign swap_out[gr] = dout_w[W_ACT];
    end
  endgenerate

endmodule
