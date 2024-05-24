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
  // \TODO: Instantiate the Gshare branch predictor
  wire temp_pred;
  wire temp_hit;
  wire [DATA_WIDTH-1:0] temp_branch_target;

  gshare m_gshare (
      .clk(clk),
      .rstn(rstn),
      
      .update(update_predictor),
      .actually_taken(actually_taken),
      .resolved_pc(resolved_pc),
      .pc(pc),

      .pred(temp_pred)
    );

  branch_target_buffer m_btb (
      .clk(clk),
      .rstn(rstn),
      
      .update(update_btb),
      .resolved_pc(resolved_pc),
      .resolved_pc_target(resolved_pc_target),
      
      .pc(pc),
      
      .hit(temp_hit),
      .target_address(temp_branch_target)
    );

  always @(*) begin
    pred = temp_pred;
    hit = temp_hit;
    branch_target = temp_branch_target;
  end
  
`endif

`ifdef PERCEPTRON
  // TODO: Instantiate the Perceptron branch predictor
`endif

endmodule
