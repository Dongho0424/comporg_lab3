// branch_target_buffer.v

/* The branch target buffer (BTB) stores the branch target address for
 * a branch PC. Our BTB is essentially a direct-mapped cache.
 */

module branch_target_buffer #(
  parameter DATA_WIDTH = 32,
  parameter VALID_WIDTH = 1,
  parameter TAG_WIDTH = 22,
  parameter INDEX_WIDTH = 8,
  parameter NUM_ENTRIES = 256
) (
  input clk,
  input rstn,

  // update interface
  input update,                              // when 'update' is true, we update the BTB entry
  input [DATA_WIDTH-1:0] resolved_pc,
  input [DATA_WIDTH-1:0] resolved_pc_target,

  // access interface
  input [DATA_WIDTH-1:0] pc,

  output reg hit,
  output reg [DATA_WIDTH-1:0] target_address
);

  ///////////////////
  /// wires, regs ///
  ///////////////////
  wire [TAG_WIDTH-1:0] access_tag;
  wire [INDEX_WIDTH-1:0] access_idx;
  reg valid;
  reg same_tag;

  wire [TAG_WIDTH-1:0] update_tag;
  wire [INDEX_WIDTH-1:0] update_idx;

  integer i;

  // BTB: Branch Target Buffer
  // each cell: valid bit, tag, and target address: 1+22+32 = 55 bits
  // total entry: 256
  reg [VALID_WIDTH+TAG_WIDTH+DATA_WIDTH-1:0] BTB [0:NUM_ENTRIES-1]; 

  ///////////////////
  /// initialize  ///
  ///////////////////

  always @(negedge rstn) begin
    for (i = 0; i < NUM_ENTRIES; i=i+1) begin
      BTB[i] <= 55'd0;
    end
  end

  ///////////////////
  /// assignment  ///
  ///////////////////
  assign access_tag = pc[DATA_WIDTH-1:DATA_WIDTH-TAG_WIDTH];
  assign access_idx = pc[INDEX_WIDTH+1:2];

  assign update_tag = resolved_pc[DATA_WIDTH-1:DATA_WIDTH-TAG_WIDTH];
  assign update_idx = resolved_pc[INDEX_WIDTH+1:2];

  // access on posedge 
  always @(*) begin
    // not branch or jump instruction
    if (pc == 32'h0000_0000) begin
      hit = 1'b0;
      target_address = 32'h0000_0000;
    end
    // branch or jump instruction
    else begin
      valid = BTB[access_idx][54];
      same_tag = (BTB[access_idx][53:32] == access_tag);
      target_address = BTB[access_idx][31:0];
      hit = same_tag & valid;
    end
  end

  // update on negedge
  always @(negedge clk) begin
    if (update == 1'b1) begin 
      BTB[update_idx] = {1'b1, update_tag, resolved_pc_target};
    end
  end

endmodule
