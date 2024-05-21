// forwarding.v

// This module determines if the values need to be forwarded to the EX stage.

// \TODO: declare propoer input and output ports and implement the
// forwarding unit

module forwarding #(
  parameter DATA_WIDTH = 32
)(
    // younger inst
    input [4:0] ex_rs1,
    input [4:0] ex_rs2,

    // older inst
    // dist 1
    input mem_regwrite,
    input [4:0] mem_rd,

    // dist2
    input wb_regwrite,
    input [4:0] wb_rd,

    // output
    // 00: RF
    // 01: MEM
    // 10: WB
    // 11: dummy
    output reg [1:0] forward_a,
    output reg [1:0] forward_b
    );


always @(*) begin
    // Forward A
    if (mem_regwrite && (ex_rs1 != 0) && (mem_rd == ex_rs1)) // dist 1
        forward_a = 2'b01;
    else if (wb_regwrite && (ex_rs1 != 0) && (wb_rd == ex_rs1)) // dist 2
        forward_a = 2'b10; 
    else forward_a = 2'b00; // no forwarding 
    

    // Forward B
    if (mem_regwrite && (ex_rs2 != 0) && (mem_rd == ex_rs2)) // dist 1
        forward_b = 2'b01;
    else if (wb_regwrite && (ex_rs2 != 0) && (wb_rd == ex_rs2)) // dist 2
        forward_b = 2'b10;
    else forward_b = 2'b00; // no forwarding
end

endmodule
