// immediate_generator.v

module immediate_generator #(
  parameter DATA_WIDTH = 32
)(
  input [31:0] instruction,

  output reg [DATA_WIDTH-1:0] sextimm
);

wire [6:0] opcode;
wire [4:0] rs1, rs2, rd;

wire [6:0] funct7;
wire [2:0] funct3;

assign opcode = instruction[6:0];

assign funct7 = instruction[31:25];
assign funct3 = instruction[14:12];
assign rs1 = instruction[19:15];
assign rs2 = instruction[24:20];
assign rd  = instruction[11:7];

always @(*) begin
  case (opcode)
    //////////////////////////////////////////////////////////////////////////
    // \TODO : Generate sextimm using instruction
    //////////////////////////////////////////////////////////////////////////
    // R-type: do not use imm_gen -> make fall to default 
    7'b0010011: sextimm = {{20{instruction[31]}}, instruction[31:20]}; // I-type, 12
    7'b0000011: sextimm = {{20{funct7[6]}}, funct7, rs2}; // Load, 12
    7'b0100011: sextimm = {{20{funct7[6]}}, funct7, rd}; // Store, 12
    7'b1100011: sextimm = {{19{funct7[6]}}, funct7[6], rd[0], funct7[5:0], rd[4:1], 1'b0}; // Branch, 13
    7'b1101111: sextimm = {{11{funct7[6]}}, funct7[6], rs1, funct3, rs2[0], funct7[5:0], rs2[4:1], 1'b0}; // jal, 21
    7'b1100111: sextimm = {{20{funct7[6]}}, funct7, rs2}; // jalr, 12
    7'b0110111: sextimm = {instruction[31:12], {12{1'b0}}}; // lui, left shift 20
    7'b0010111: sextimm = {instruction[31:12], {12{1'b0}}}; // auipc, left shift 20
    default:    sextimm = 32'h0000_0000;
  endcase
end


endmodule
