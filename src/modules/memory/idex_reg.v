// idex_reg.v
// This module is the ID/EX pipeline register.


module idex_reg #(
  parameter DATA_WIDTH = 32
)(
  // TODO: Add flush or stall signal if it is needed

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

  //////////////////////////////////////
  // Outputs
  //////////////////////////////////////
  output [DATA_WIDTH-1:0] ex_PC,
  output [DATA_WIDTH-1:0] ex_pc_plus_4,

  // ex control
  output ex_branch,
  output [1:0] ex_aluop,
  output ex_alusrc,
  output [1:0] ex_jump,

  // mem control
  output ex_memread,
  output ex_memwrite,

  // wb control
  output ex_memtoreg,
  output ex_regwrite,

  output [DATA_WIDTH-1:0] ex_sextimm,
  output [6:0] ex_funct7,
  output [2:0] ex_funct3,
  output [DATA_WIDTH-1:0] ex_readdata1,
  output [DATA_WIDTH-1:0] ex_readdata2,
  output [4:0] ex_rs1,
  output [4:0] ex_rs2,
  output [4:0] ex_rd
);

// TODO: Implement ID/EX pipeline register module

endmodule
