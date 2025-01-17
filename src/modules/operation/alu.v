// ALU.v

// This module performs ALU operations according to the "alu_func" value,
// which is generated by the ALU control unit.
// Note that there exist 10 R-type instructions in RV32I:
// add, sub, xor, or, and, sll, srl, sra, slt, sltu

`include "src/modules/utils/defines.v"

module alu
#(parameter DATA_WIDTH = 32)(
  input [DATA_WIDTH-1:0] in_a, 
  input [DATA_WIDTH-1:0] in_b,
  input [3:0] alu_func,

  output reg [DATA_WIDTH-1:0] result,
  output reg check 
);

// combinational logic 
always @(*) begin
  case (alu_func)
    `OP_ADD: result = in_a + in_b; 
    `OP_SUB: result = in_a - in_b;
    `OP_XOR: result = in_a ^ in_b;
    `OP_OR:  result = in_a | in_b;
    `OP_AND: result = in_a & in_b;

    //////////////////////////////////////////////////////////////////////////
    // \TODO : Add other operations
    // - The example below is given as a hint
    // - `OP_SRA: result = $signed(in_a) >>> in_b[4:0];
    //////////////////////////////////////////////////////////////////////////
    `OP_SLL:  result = in_a << in_b[4:0];
    `OP_SRL:  result = in_a >> in_b[4:0];
    `OP_SRA:  result = $signed(in_a) >>> in_b[4:0];
    `OP_SLT:  result = ($signed(in_a) < $signed(in_b)) ? 32'h0000_0001 : 32'h0000_0000;
    `OP_SLTU: result = ($unsigned(in_a) < $unsigned(in_b)) ? 32'h0000_0001 : 32'h0000_0000;
    `OP_BGE: result = ($signed(in_a) >= $signed(in_b)) ? 32'h0000_0001 : 32'h0000_0000;
    `OP_BGEU: result = ($unsigned(in_a) >= $unsigned(in_b)) ? 32'h0000_0001 : 32'h0000_0000;
    default:  result = 32'h0000_0000;
  endcase
end

// combinational logic
always @(*) begin
  case (alu_func)
    //////////////////////////////////////////////////////////////////////////
    // \TODO : Generate check signal
    //////////////////////////////////////////////////////////////////////////
    
    // As ALU cannot check whether instruction is beq or bne,
    // external muxes are used with funct3 as control signal.
    `OP_SUB: check = result == 32'h0000_0000; // beq
    `OP_XOR: check = result != 32'h0000_0000; // bne
    `OP_SLT: check = result; // blt
    `OP_BGE: check = result; // bge
    `OP_SLTU: check = result; // bltu
    `OP_BGEU: check = result; // bgeu
    default:  check = 1'b0;
  endcase
end
endmodule
