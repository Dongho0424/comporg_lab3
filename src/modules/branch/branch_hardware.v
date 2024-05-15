// branch_hardware.v

/* This module comprises a branch predictor and a branch target buffer.
 * Our CPU will use the branch target address only when BTB is hit.
 */

module branch_hardware #(
  parameter DATA_WIDTH = 32,
  parameter COUNTER_WIDTH = 2,
  parameter NUM_ENTRIES = 256 // 2^8
) (
  input clk,
  input rstn,

  // update interface
  input update_predictor,
  input update_btb,
  input actually_taken,
  input [DATA_WIDTH-1:0] resolved_pc,
  input [DATA_WIDTH-1:0] resolved_pc_target,  // actual target address when the branch is resolved.

  // access interface
  input [DATA_WIDTH-1:0] pc,

  output reg hit,          // btb hit or not
  output reg pred,         // predicted taken or not
  output reg [DATA_WIDTH-1:0] branch_target  // branch target address for a hit
);

`ifdef GSHARE
  // TODO: Instantiate the Gshare branch predictor
`endif

`ifdef PERCEPTRON
  // TODO: Instantiate the Perceptron branch predictor
`endif

endmodule
