// hazard.v

// This module determines if pipeline stalls or flushing are required

// \TODO: declare propoer input and output ports and implement the
// hazard detection unit

module hazard #(
  parameter DATA_WIDTH = 32
)(

    input rstn,
    input taken,            // flush
    input [4:0] id_rs1,     // stall
    input [4:0] id_rs2,     // stall
    input [4:0] ex_rd,      // stall
    input ex_memread,       // stall

    output reg ifid_flush,  // flush
    output reg idex_flush,  // flush
    output reg exmem_flush, // flush
    output reg pc_write,    // stall
    output reg ifid_write,  // stall
    output reg idex_write   // stall
);

always @(negedge rstn) begin // initialize
    ifid_flush = 1'b0;
    idex_flush = 1'b0;
    exmem_flush = 1'b0;
    pc_write = 1'b1;
    ifid_write = 1'b1;
    idex_write = 1'b1;
end

wire stall; // stall logic
assign stall = (ex_memread && (id_rs1 == ex_rd) && (id_rs1 != 0)) || 
               (ex_memread && (id_rs2 == ex_rd) && (id_rs2 != 0));

always @(*) begin
    if (taken) begin // flush logic
        ifid_flush = 1'b1;
        idex_flush = 1'b1;
        exmem_flush = 1'b1;
        pc_write = 1'b1;
        ifid_write = 1'b1;
        idex_write = 1'b1;
    end else if (stall) 
    begin
        ifid_flush = 1'b0;
        idex_flush = 1'b0;
        exmem_flush = 1'b0;
        pc_write = 1'b0;
        ifid_write = 1'b0;
        idex_write = 1'b0;
    end else begin
        ifid_flush = 1'b0;
        idex_flush = 1'b0;
        exmem_flush = 1'b0;
        pc_write = 1'b1;
        ifid_write = 1'b1;
        idex_write = 1'b1;
    end
end


endmodule
