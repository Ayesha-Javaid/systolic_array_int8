`timescale 1ns / 1ps

module delay_line #(
  parameter int W     = 8,    // word width
  parameter int DEPTH = 1     // enabled cycles of delay; 0 is a wire
) (
  // At DEPTH = 0 this module is a wire, so clk and en really are unused in that
  // elaboration and the lint pass says so. The waiver is scoped to the port
  // list, in the style of pack_extract.sv, so a genuinely unused signal
  // elsewhere in this file is still reported.
  /* verilator lint_off UNUSEDSIGNAL */
  input  logic         clk,
  input  logic         en,    // advance; maps to the SRL16E CE
  /* verilator lint_on UNUSEDSIGNAL */
  input  logic [W-1:0] din,
  output logic [W-1:0] dout
);

  // Exported so a consumer can state its own latency in terms of this one
  // rather than re-deriving it.
  localparam int LATENCY = DEPTH;

  initial begin
    if (W < 1)
      $fatal(1, "delay_line: W=%0d must be at least 1", W);
    if (DEPTH < 0)
      $fatal(1, "delay_line: DEPTH=%0d must not be negative", DEPTH);
  end


endmodule
