// idex_reg.v
// This module is the ID/EX pipeline register.


module idex_reg #(
  parameter DATA_WIDTH = 32
)(
  // \TODO: Add flush or stall signal if it is needed
  input flush,
  input idex_write,

  //////////////////////////////////////
  // Inputs
  //////////////////////////////////////
  input clk,

  input [DATA_WIDTH-1:0] id_PC,
  input [DATA_WIDTH-1:0] id_pc_plus_4,

  // ex control
  input [1:0] id_jump,
  input id_branch,
  input [1:0] id_aluop,
  input id_alusrc,
  input [1:0] id_u_type,

  // mem control
  input id_memread,
  input id_memwrite,

  // wb control
  input id_memtoreg,
  input id_regwrite,

  input [DATA_WIDTH-1:0] id_sextimm,
  input [6:0] id_funct7,
  input [2:0] id_funct3,
  input [DATA_WIDTH-1:0] id_readdata1,
  input [DATA_WIDTH-1:0] id_readdata2,
  input [4:0] id_rs1,
  input [4:0] id_rs2,
  input [4:0] id_rd,

  // branch prediction
  input id_pred,
  input id_hit,
  input [DATA_WIDTH-1:0] id_pred_PC_target,

  //////////////////////////////////////
  // Outputs
  //////////////////////////////////////
  output reg [DATA_WIDTH-1:0] ex_PC,
  output reg [DATA_WIDTH-1:0] ex_pc_plus_4,

  // ex control
  output reg ex_branch,
  output reg [1:0] ex_aluop,
  output reg ex_alusrc,
  output reg [1:0] ex_jump,
  output reg [1:0] ex_u_type,

  // mem control
  output reg ex_memread,
  output reg ex_memwrite,

  // wb control
  output reg ex_memtoreg,
  output reg ex_regwrite,

  output reg [DATA_WIDTH-1:0] ex_sextimm,
  output reg [6:0] ex_funct7,
  output reg [2:0] ex_funct3,
  output reg [DATA_WIDTH-1:0] ex_readdata1,
  output reg [DATA_WIDTH-1:0] ex_readdata2,
  output reg [4:0] ex_rs1,
  output reg [4:0] ex_rs2,
  output reg [4:0] ex_rd,

  // branch prediction
  output reg ex_pred,
  output reg ex_hit,
  output reg [DATA_WIDTH-1:0] ex_pred_PC_target
);

// \TODO: Implement ID/EX pipeline register module
always @(posedge clk) begin
  if (flush) begin
    ex_PC <= 32'b0;
    ex_pc_plus_4 <= 32'b0;
    ex_branch <= 32'b0;
    ex_aluop <= 32'b0;
    ex_alusrc <= 32'b0;
    ex_jump <= 32'b0;
    ex_memread <= 32'b0;
    ex_memwrite <= 32'b0;
    ex_memtoreg <= 32'b0;
    ex_regwrite <= 32'b0;
    ex_u_type <= 32'b0;
    ex_sextimm <= 32'b0;
    ex_funct7 <= 32'b0;
    ex_funct3 <= 32'b0;
    ex_readdata1 <= 32'b0;
    ex_readdata2 <= 32'b0;
    ex_rs1 <= 32'b0;
    ex_rs2 <= 32'b0;
    ex_rd <= 32'b0;
    ex_pred <= 1'b0;
    ex_hit <= 1'b0;
    ex_pred_PC_target <= 32'b0;
  end
  else if (idex_write) begin 
    ex_PC <= id_PC;
    ex_pc_plus_4 <= id_pc_plus_4;
    ex_branch <= id_branch;
    ex_aluop <= id_aluop;
    ex_alusrc <= id_alusrc;
    ex_jump <= id_jump;
    ex_memread <= id_memread;
    ex_memwrite <= id_memwrite;
    ex_memtoreg <= id_memtoreg;
    ex_regwrite <= id_regwrite;
    ex_u_type <= id_u_type;
    ex_sextimm <= id_sextimm;
    ex_funct7 <= id_funct7;
    ex_funct3 <= id_funct3;
    ex_readdata1 <= id_readdata1;
    ex_readdata2 <= id_readdata2;
    ex_rs1 <= id_rs1;
    ex_rs2 <= id_rs2;
    ex_rd <= id_rd;
    ex_pred <= id_pred;
    ex_hit <= id_hit;
    ex_pred_PC_target <= id_pred_PC_target;
  end  
  else begin // stall. All control signals become zero.
    ex_branch <= 32'b0;
    ex_aluop <= 32'b0;
    ex_alusrc <= 32'b0;
    ex_jump <= 32'b0;
    ex_memread <= 32'b0;
    ex_memwrite <= 32'b0;
    ex_memtoreg <= 32'b0;
    ex_regwrite <= 32'b0;
    ex_u_type <= 32'b0;
    ex_pred <= 1'b0;
    ex_hit <= 1'b0;
  end
end

endmodule
