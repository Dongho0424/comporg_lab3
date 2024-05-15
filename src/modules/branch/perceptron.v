// perceptron.v

/* The perceptron predictor uses the simplest form of neural networks
 * (perceptron), instead of using two-bit counters.  Note that PC[1:0] is not
 * used when indexing into the table of perceptrons.
 *
 * D. Jimenez and C. Lin. "Dynamic Branch Prediction with Perceptrons" HPCA 2001.
 */

module perceptron #(
  parameter DATA_WIDTH = 32,
  parameter HIST_LEN = 25, // Since x0 is always 1, 26 weights will reside in the perceptron table 
  parameter NUM_ENTRIES = 32
) (
  input clk,
  input rstn,

  // update interface
  input update,
  input actually_taken,
  input [DATA_WIDTH-1:0] resolved_pc,

  // access interface
  input [DATA_WIDTH-1:0] pc,

  output reg pred
);

  // TODO: Implement the perceptron branch predictor
  // NOTE: DO NOT CHANGE the local parameters
  localparam INDEX_WIDTH     = $clog2(NUM_ENTRIES);
  localparam THRESHOLD       = $rtoi($floor(1.93 * HIST_LEN + 14));
  localparam WEIGHT_BITWIDTH = 1 + $clog2(THRESHOLD + 1);
  localparam WEIGHT_MAX      = $signed({1'b0, {WEIGHT_BITWIDTH-1{1'b1}}});
  localparam WEIGHT_MIN      = $signed({1'b1, {WEIGHT_BITWIDTH-1{1'b0}}});
  localparam OUTPUT_BITWIDTH = 1 + $clog2((HIST_LEN + 1) * WEIGHT_MAX + 1);

endmodule
