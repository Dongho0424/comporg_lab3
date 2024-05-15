// simple_cpu.v
// a pipelined RISC-V microarchitecture (RV32I)

///////////////////////////////////////////////////////////////////////////////////////////
//// [*] In simple_cpu.v you should connect the correct wires to the correct ports
////     - All modules are given so there is no need to make new modules
////       (it does not mean you do not need to instantiate new modules)
////     - However, you may have to fix or add in / out ports for some modules
////     - In addition, you are still free to instantiate simple modules like multiplexers,
////       adders, etc.
///////////////////////////////////////////////////////////////////////////////////////////

module simple_cpu
#(parameter DATA_WIDTH = 32)(
  input clk,
  input rstn
);

///////////////////////////////////////////////////////////////////////////////
// TODO:  Declare all wires / registers that are needed
///////////////////////////////////////////////////////////////////////////////
// e.g., wire [DATA_WIDTH-1:0] if_pc_plus_4;
// 1) Pipeline registers (wires to / from pipeline register modules)
// 2) In / Out ports for other modules
// 3) Additional wires for multiplexers or other mdoules you instantiate

///////////////////////////////////////////////////////////////////////////////
// Instruction Fetch (IF)
///////////////////////////////////////////////////////////////////////////////

reg [DATA_WIDTH-1:0] PC;    // program counter (32 bits)

wire [DATA_WIDTH-1:0] NEXT_PC;

/* m_next_pc_adder */
adder m_pc_plus_4_adder(
  .in_a   (),
  .in_b   (),

  .result ()
);

always @(posedge clk) begin
  if (rstn == 1'b0) begin
    PC <= 32'h00000000;
  end
  else PC <= NEXT_PC;
end

/* instruction: read current instruction from inst mem */
instruction_memory m_instruction_memory(
  .address    (),

  .instruction()
);

/* forward to IF/ID stage registers */
ifid_reg m_ifid_reg(
  // TODO: Add flush or stall signal if it is needed
  .clk            (),
  .if_PC          (),
  .if_pc_plus_4   (),
  .if_instruction (),

  .id_PC          (),
  .id_pc_plus_4   (),
  .id_instruction ()
);


//////////////////////////////////////////////////////////////////////////////////
// Instruction Decode (ID)
//////////////////////////////////////////////////////////////////////////////////

/* m_hazard: hazard detection unit */
hazard m_hazard(
  // TODO: implement hazard detection unit & do wiring
);

/* m_control: control unit */
control m_control(
  .opcode     (),

  .jump       (),
  .branch     (),
  .alu_op     (),
  .alu_src    (),
  .mem_read   (),
  .mem_to_reg (),
  .mem_write  (),
  .reg_write  ()
);

/* m_imm_generator: immediate generator */
immediate_generator m_immediate_generator(
  .instruction(),

  .sextimm    ()
);

/* m_register_file: register file */
register_file m_register_file(
  .clk        (),
  .readreg1   (),
  .readreg2   (),
  .writereg   (),
  .wen        (),
  .writedata  (),

  .readdata1  (),
  .readdata2  ()
);

/* forward to ID/EX stage registers */
idex_reg m_idex_reg(
  // TODO: Add flush or stall signal if it is needed
  .clk          (),
  .id_PC        (),
  .id_pc_plus_4 (),
  .id_jump      (),
  .id_branch    (),
  .id_aluop     (),
  .id_alusrc    (),
  .id_memread   (),
  .id_memwrite  (),
  .id_memtoreg  (),
  .id_regwrite  (),
  .id_sextimm   (),
  .id_funct7    (),
  .id_funct3    (),
  .id_readdata1 (),
  .id_readdata2 (),
  .id_rs1       (),
  .id_rs2       (),
  .id_rd        (),

  .ex_PC        (),
  .ex_pc_plus_4 (),
  .ex_jump      (),
  .ex_branch    (),
  .ex_aluop     (),
  .ex_alusrc    (),
  .ex_memread   (),
  .ex_memwrite  (),
  .ex_memtoreg  (),
  .ex_regwrite  (),
  .ex_sextimm   (),
  .ex_funct7    (),
  .ex_funct3    (),
  .ex_readdata1 (),
  .ex_readdata2 (),
  .ex_rs1       (),
  .ex_rs2       (),
  .ex_rd        ()
);

//////////////////////////////////////////////////////////////////////////////////
// Execute (EX) 
//////////////////////////////////////////////////////////////////////////////////

/* m_branch_target_adder: PC + imm for branch address */
adder m_branch_target_adder(
  .in_a   (),
  .in_b   (),

  .result ()
);

/* m_branch_control : checks T/NT */
branch_control m_branch_control(
  .branch (),
  .check  (),
  
  .taken  ()
);

/* alu control : generates alu_func signal */
alu_control m_alu_control(
  .alu_op   (),
  .funct7   (),
  .funct3   (),

  .alu_func ()
);

/* m_alu */
alu m_alu(
  .alu_func (),
  .in_a     (), 
  .in_b     (), 

  .result   (),
  .check    ()
);

forwarding m_forwarding(
  // TODO: implement forwarding unit & do wiring
);

/* forward to EX/MEM stage registers */
exmem_reg m_exmem_reg(
  // TODO: Add flush or stall signal if it is needed
  .clk            (),
  .ex_pc_plus_4   (),
  .ex_pc_target   (),
  .ex_taken       (), 
  .ex_jump        (),
  .ex_memread     (),
  .ex_memwrite    (),
  .ex_memtoreg    (),
  .ex_regwrite    (),
  .ex_alu_result  (),
  .ex_writedata   (),
  .ex_funct3      (),
  .ex_rd          (),
  
  .mem_pc_plus_4  (),
  .mem_pc_target  (),
  .mem_taken      (), 
  .mem_jump       (),
  .mem_memread    (),
  .mem_memwrite   (),
  .mem_memtoreg   (),
  .mem_regwrite   (),
  .mem_alu_result (),
  .mem_writedata  (),
  .mem_funct3     (),
  .mem_rd         ()
);


//////////////////////////////////////////////////////////////////////////////////
// Memory (MEM) 
//////////////////////////////////////////////////////////////////////////////////

/* m_data_memory : main memory module */
data_memory m_data_memory(
  .clk         (),
  .address     (),
  .write_data  (),
  .mem_read    (),
  .mem_write   (),
  .maskmode    (),
  .sext        (),

  .read_data   ()
);

/* forward to MEM/WB stage registers */
memwb_reg m_memwb_reg(
  // TODO: Add flush or stall signal if it is needed
  .clk            (),
  .mem_pc_plus_4  (),
  .mem_jump       (),
  .mem_memtoreg   (),
  .mem_regwrite   (),
  .mem_readdata   (),
  .mem_alu_result (),
  .mem_rd         (),

  .wb_pc_plus_4   (),
  .wb_jump        (),
  .wb_memtoreg    (),
  .wb_regwrite    (),
  .wb_readdata    (),
  .wb_alu_result  (),
  .wb_rd          ()
);

//////////////////////////////////////////////////////////////////////////////////
// Write Back (WB) 
//////////////////////////////////////////////////////////////////////////////////


endmodule
