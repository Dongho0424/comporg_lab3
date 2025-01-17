// ifid_reg.v
// This module is the IF/ID pipeline register.


module ifid_reg #(
  parameter DATA_WIDTH = 32
)(
  // \TODO: Add flush or stall signal if it is needed

  input flush,
  input ifid_write,

  //////////////////////////////////////
  // Inputs
  //////////////////////////////////////
  input clk,

  input [DATA_WIDTH-1:0] if_PC,
  input [DATA_WIDTH-1:0] if_pc_plus_4,
  input [DATA_WIDTH-1:0] if_instruction,

  // branch prediction
  input if_pred,
  input if_hit,
  input [DATA_WIDTH-1:0] if_pred_PC_target,

  //////////////////////////////////////
  // Outputs
  //////////////////////////////////////
  output reg [DATA_WIDTH-1:0] id_PC,
  output reg [DATA_WIDTH-1:0] id_pc_plus_4,
  output reg [DATA_WIDTH-1:0] id_instruction,

  // branch prediction
  output reg id_pred,
  output reg id_hit,
  output reg [DATA_WIDTH-1:0] id_pred_PC_target
);

// \TODO: Implement IF/ID pipeline register module
always @(posedge clk) begin
  if (flush) begin
    id_PC <= 32'b0;
    id_pc_plus_4 <= 32'b0;
    id_instruction <= 32'b0;
    id_pred <= 1'b0;
    id_hit <= 1'b0;
    id_pred_PC_target <= 32'b0;
  end
  else if (ifid_write) begin
    id_PC <= if_PC;
    id_pc_plus_4 <= if_pc_plus_4;
    id_instruction <= if_instruction;
    id_pred <= if_pred;
    id_hit <= if_hit;
    id_pred_PC_target <= if_pred_PC_target;
  end
  // else: stall
end  

endmodule
