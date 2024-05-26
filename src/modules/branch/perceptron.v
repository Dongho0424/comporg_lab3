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

  // \TODO: Implement the perceptron branch predictor
  // NOTE: DO NOT CHANGE the local parameters
  localparam INDEX_WIDTH     = $clog2(NUM_ENTRIES); // 5
  localparam THRESHOLD       = $rtoi($floor(1.93 * HIST_LEN + 14)); // 62
  localparam WEIGHT_BITWIDTH = 1 + $clog2(THRESHOLD + 1); // 7
  localparam WEIGHT_MAX      = $signed({1'b0, {WEIGHT_BITWIDTH-1{1'b1}}}); // 63
  localparam WEIGHT_MIN      = $signed({1'b1, {WEIGHT_BITWIDTH-1{1'b0}}}); // -64
  localparam OUTPUT_BITWIDTH = 1 + $clog2((HIST_LEN + 1) * WEIGHT_MAX + 1); // 12

  ///////////////////
  /// wires, regs ///
  ///////////////////

  // BHR: 25-bit global branch history register
  // x25, x24, ..., x1, in order (x0 is always 1)
  reg [HIST_LEN:1] BHR;
  reg [HIST_LEN:1] BHR_snapshot;

  // Perceptron Table
  // total entry: 32
  // each cell: 26-bit weight vector
  // each weight: 7-bit signed integer
  reg signed [WEIGHT_BITWIDTH-1:0] PT [0:NUM_ENTRIES-1][HIST_LEN:0];

  wire [INDEX_WIDTH-1:0] access_idx;
  reg signed [OUTPUT_BITWIDTH-1:0] access_out;

  wire [INDEX_WIDTH-1:0] update_idx;
  reg signed [OUTPUT_BITWIDTH-1:0] update_out;
  reg signed [OUTPUT_BITWIDTH-1:0] update_abs_out;

  integer i;  
  integer j;  

  ///////////////////
  /// initialize  ///
  ///////////////////

  always @(negedge rstn) begin
    BHR <= 25'd0;
    for (i = 0; i < NUM_ENTRIES; i=i+1) begin
      for (j = 0; j <= HIST_LEN; j=j+1) begin
        PT[i][j] <= 7'd0;
      end
    end
  end

  ///////////////////
  /// assignment  ///
  ///////////////////

  // access
  assign access_idx = pc[INDEX_WIDTH+1:2];

  always @(*) begin
    access_out = PT[access_idx][0];
    for (i = 1; i <= HIST_LEN; i=i+1) begin
      // BHR[i]:1 => add PT[access_idx][i]
      // BHR[i]:0 => add -PT[access_idx][i] (2's complement)
      access_out = access_out + (BHR_snapshot[i] ? PT[access_idx][i] : -PT[access_idx][i]);
    end

    pred = (access_out >= 0);
  end

  // update snapshot on posedge
  always @(posedge clk) begin
    BHR_snapshot <= BHR;
  end

  // update on negedge
  assign update_idx = resolved_pc[INDEX_WIDTH+1:2];

  always @(negedge clk) begin
    if (update) begin
      // update PT
      update_out = PT[update_idx][0];
      for (i = 1; i <= HIST_LEN; i=i+1) begin
        // BHR[i]:1 => add PT[update_idx][i]
        // BHR[i]:0 => add -PT[update_idx][i] (2's complement)
        update_out = update_out + (BHR_snapshot[i] ? PT[update_idx][i] : -PT[update_idx][i]);
      end

      if (update_out < 0) begin
        update_abs_out = -update_out;
      end else begin
        update_abs_out = update_out;
      end

      if (((update_out >= 0) != actually_taken) || (update_abs_out <= THRESHOLD)) begin
        // w_0 = w_0 + t
        if (actually_taken) begin
          PT[update_idx][0] = (PT[update_idx][0] == WEIGHT_MAX) ? WEIGHT_MAX : (PT[update_idx][0] + 7'd1);
        end
        else begin
          PT[update_idx][0] = (PT[update_idx][0] == WEIGHT_MIN) ? WEIGHT_MIN : (PT[update_idx][0] - 7'd1);
        end
        for (i = 1; i <= HIST_LEN; i=i+1) begin
          // w_i = w_i + (t * x_i)
          // saturate w_i to [WEIGHT_MIN, WEIGHT_MAX]
          if (actually_taken == BHR_snapshot[i]) begin
            PT[update_idx][i] = (PT[update_idx][i] == WEIGHT_MAX) ? WEIGHT_MAX : (PT[update_idx][i] + 7'd1);
          end
          else begin
            PT[update_idx][i] = (PT[update_idx][i] == WEIGHT_MIN) ? WEIGHT_MIN : (PT[update_idx][i] - 7'd1);
          end
        end
      end

      // update BHR
      BHR = {BHR_snapshot[HIST_LEN-1:1], actually_taken};
    end
  end

endmodule
