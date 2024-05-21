// control.v

// The main control module takes as input the opcode field of an instruction
// (i.e., instruction[6:0]) and generates a set of control signals.

module control(
  input [6:0] opcode,

  output [1:0] jump,
  output branch,
  output mem_read,
  output mem_to_reg,
  output [1:0] alu_op,
  output mem_write,
  output alu_src,
  output reg_write,
  output [1:0] u_type
);
// total 12bit controls including 2b u_type
reg [11:0] controls;

// combinational logic
always @(*) begin
  case (opcode)
    7'b0110011: controls = 12'b00_000_10_001_00; // R-type
    
    //////////////////////////////////////////////////////////////////////////
    // \TODO : Implement signals for other instruction types
    //////////////////////////////////////////////////////////////////////////
    7'b0010011: controls = 12'b00_000_11_011_00; // I-type
    7'b0000011: controls = 12'b00_011_00_011_00; // Load
    7'b0100011: controls = 12'b00_00x_00_110_00; // Store
    7'b1100011: controls = 12'b01_10x_01_000_00; // Branch
    7'b1101111: controls = 12'b10_000_11_011_00; // jal. assign jump = 2'b10
    7'b1100111: controls = 12'b11_000_11_011_00; // jalr. assign jump = 2'b11
    7'b0110111: controls = 12'b00_000_00_011_01; // lui, utype: 01
    7'b0010111: controls = 12'b00_000_00_011_10; // auipc, utype: 10
    default:    controls = 12'b00_000_00_000_00;
  endcase
end

assign {jump, branch, mem_read, mem_to_reg, alu_op, mem_write, alu_src, reg_write, u_type} = controls;

endmodule
