//exmem_reg.v


module exmem_reg #(
  parameter DATA_WIDTH = 32
)(
  // \TODO: Add flush or stall signal if it is needed
  input flush,

  //////////////////////////////////////
  // Inputs
  //////////////////////////////////////
  input clk,

  // branch prediction
  input [DATA_WIDTH-1:0] ex_PC,
  input [DATA_WIDTH-1:0] ex_pc_plus_4,
  input [DATA_WIDTH-1:0] ex_pc_target,
  input ex_taken,
  input ex_branch,
  input ex_pred,
  input ex_hit,
  input [DATA_WIDTH-1:0] ex_pred_PC_target,

  // mem control
  input ex_memread,
  input ex_memwrite,

  // wb control
  input [1:0] ex_jump,
  input ex_memtoreg,
  input ex_regwrite,
  
  input [DATA_WIDTH-1:0] ex_alu_result,
  input [DATA_WIDTH-1:0] ex_writedata,
  input [2:0] ex_funct3,
  input [4:0] ex_rd,
  
  //////////////////////////////////////
  // Outputs
  //////////////////////////////////////

  // branch prediction
  output reg [DATA_WIDTH-1:0] mem_PC,
  output reg [DATA_WIDTH-1:0] mem_pc_plus_4,
  output reg [DATA_WIDTH-1:0] mem_pc_target,
  output reg mem_taken,
  output reg mem_branch,
  output reg mem_pred,
  output reg mem_hit,
  output reg [DATA_WIDTH-1:0] mem_pred_PC_target,

  // mem control
  output reg mem_memread,
  output reg mem_memwrite,

  // wb control
  output reg [1:0] mem_jump,
  output reg mem_memtoreg,
  output reg mem_regwrite,
  
  output reg [DATA_WIDTH-1:0] mem_alu_result,
  output reg [DATA_WIDTH-1:0] mem_writedata,
  output reg [2:0] mem_funct3,
  output reg [4:0] mem_rd
);

// \TODO: Implement EX / MEM pipeline register module
always @(posedge clk) begin
  if (flush) begin
    mem_PC <= 32'b0;
    mem_pc_plus_4 <= 32'b0;
    mem_pc_target <= 32'b0;
    mem_taken <= 1'b0;
    mem_branch <= 1'b0;
    mem_pred <= 1'b0;
    mem_hit <= 1'b0;
    mem_pred_PC_target <= 32'b0;

    mem_memread <= 1'b0;
    mem_memwrite <= 1'b0;
    mem_jump <= 2'b0;
    mem_memtoreg <= 1'b0;
    mem_regwrite <= 1'b0;
    mem_alu_result <= 32'b0;
    mem_writedata <= 32'b0;
    mem_funct3 <= 3'b0;
    mem_rd <= 5'b0;
  end
  else begin
    mem_PC <= ex_PC;
    mem_pc_plus_4 <= ex_pc_plus_4;
    mem_pc_target <= ex_pc_target;
    mem_taken <= ex_taken;
    mem_branch <= ex_branch;
    mem_pred <= ex_pred;
    mem_hit <= ex_hit;
    mem_pred_PC_target <= ex_pred_PC_target;

    mem_memread <= ex_memread;
    mem_memwrite <= ex_memwrite;
    mem_jump <= ex_jump;
    mem_memtoreg <= ex_memtoreg;
    mem_regwrite <= ex_regwrite;
    mem_alu_result <= ex_alu_result;
    mem_writedata <= ex_writedata;
    mem_funct3 <= ex_funct3;
    mem_rd <= ex_rd;
  end
end

endmodule
