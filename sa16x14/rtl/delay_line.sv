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

  generate
    if (DEPTH == 0) begin : g_wire
      // A wire, not a register. See the header: this is a contract, not an
      // optimisation, and it is measured rather than assumed.
      assign dout = din;
    end else begin : g_sr
      // Unpacked array of vectors, which is the shape the SRL inference looks
      // for; a packed 2-D vector is recognised less reliably. The attribute
      // makes the mapping an instruction rather than a hope, exactly as
      // dsp_pack_mac's use_dsp does. At DEPTH < 3 there is nothing worth
      // putting in an SRL and the tool will place registers anyway, which is
      // the right answer -- the attribute is left on at every depth so that
      // there is one code path here instead of two that can drift apart.
      (* srl_style = "srl_reg" *) logic [W-1:0] sr [0:DEPTH-1];

      // The configured power-up state. This is the INIT attribute of the
      // SRL16E/FDRE on 7-series, so it is as real in the bitstream as it is in
      // simulation -- it is what stands in for the reset this module does not
      // have. Written as a plain loop over a module-scope index so that the
      // same source elaborates under Icarus, the lint/sim tool of
      // docs/NOTES-tooling.md, and Vivado without a per-tool variant.
      integer init_k;
      initial begin
        for (init_k = 0; init_k < DEPTH; init_k = init_k + 1) sr[init_k] = '0;
      end

      // No reset in the sensitivity list, deliberately: an asynchronous reset
      // here is precisely what forces the flip-flop mapping.
      always_ff @(posedge clk) begin
        if (en) begin
          sr[0] <= din;
          for (int k = 1; k < DEPTH; k++)
            sr[k] <= sr[k-1];
        end
      end

      assign dout = sr[DEPTH-1];
    end
  endgenerate

endmodule
