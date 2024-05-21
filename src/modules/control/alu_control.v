// ALU_control.v

/* This unit generates a 4-bit ALU control input (alu_func)
 * based on the 2-bit ALUOp control, funct7, and funct3 field.
 *
 * ALUOp | ALU action | notes  
 * ------|------------|---------------------
 *   00  | add        | for loads and stores
 *   01  | it varies  | for branches
 *   10  | it varies  | for R-types
 *   11  | it varies  | immediate
 *
 * R-type instructions (opcode: 0110011)
 * Name | funct3 | funct7
 * -----------------------
 * add  |  0x0   | 0x00
 * sub  |  0x0   | 0x20
 * xor  |  0x4   | 0x00
 * or   |  0x6   | 0x00
 * and  |  0x7   | 0x00
 * sll  |  0x1   | 0x00
 * srl  |  0x5   | 0x00
 * sra  |  0x5   | 0x20
 * slt  |  0x2   | 0x00
 * sltu |  0x3   | 0x00
 */

`include "src/modules/utils/defines.v"

module alu_control(
  input wire [1:0] alu_op,
  input wire [6:0] funct7,
  input wire [2:0] funct3,

  output reg [3:0] alu_func
);

wire [3:0] funct;
assign funct = {funct7[5], funct3};

// combinational logic
always @(*) begin
  case (alu_op)
    2'b00: begin
      ///////////////////////////////////////////////////////////////////////
      // \TODO : select operation for loads/stores
      ///////////////////////////////////////////////////////////////////////
      // load-store: always add
      alu_func = `OP_ADD;
    end
    2'b01: begin
      ///////////////////////////////////////////////////////////////////////
      // \TODO : select operation for branches
      ///////////////////////////////////////////////////////////////////////
      casex (funct) 
        4'bx_000: alu_func = `OP_SUB; // beq
        4'bx_001: alu_func = `OP_XOR; // bne 
        4'bx_100: alu_func = `OP_SLT; // blt
        4'bx_101: alu_func = `OP_BGE; // bge
        4'bx_110: alu_func = `OP_SLTU; // bltu
        4'bx_111: alu_func = `OP_BGEU; // bgeu
        default:  alu_func = `OP_EEE;  // shoud not fall here 
      endcase
    end
    2'b10: begin                // R-types
      case (funct)
        4'b0_000: alu_func = `OP_ADD; // add
        4'b1_000: alu_func = `OP_SUB; // sub
        4'b0_100: alu_func = `OP_XOR; // xor
        4'b0_110: alu_func = `OP_OR;  // or
        4'b0_111: alu_func = `OP_AND; // and
        4'b0_001: alu_func = `OP_SLL; // sll
        4'b0_101: alu_func = `OP_SRL; // srl
        4'b1_101: alu_func = `OP_SRA; // sra
        4'b0_010: alu_func = `OP_SLT; // slt
        4'b0_011: alu_func = `OP_SLTU; // sltu
        default:  alu_func = `OP_EEE; // shoud not fall here 
      endcase
    end
    2'b11: begin
      ///////////////////////////////////////////////////////////////////////
      // \TODO : select operation for I-types with immediate
      ///////////////////////////////////////////////////////////////////////
      casex (funct)
        4'bx_000: alu_func = `OP_ADD;  // addi, jalr
        4'bx_100: alu_func = `OP_XOR;  // xori
        4'bx_110: alu_func = `OP_OR;   // ori
        4'bx_111: alu_func = `OP_AND;  // andi
        4'b0_001: alu_func = `OP_SLL;  // slli
        4'b0_101: alu_func = `OP_SRL;  // srli
        4'b1_101: alu_func = `OP_SRA;  // srai
        4'bx_010: alu_func = `OP_SLT;  // slti
        4'bx_011: alu_func = `OP_SLTU; // sltui
        default:  alu_func = `OP_EEE;  // shoud not fall here 
      endcase
    end
    default: alu_func = `OP_EEE;       // should not fall here
  endcase
end

endmodule
