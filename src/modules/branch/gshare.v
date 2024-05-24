// gshare.v

/* The Gshare predictor consists of the global branch history register (BHR)
 * and a pattern history table (PHT). Note that PC[1:0] is not used for
 * indexing.
 */

module gshare #(
  parameter DATA_WIDTH = 32,
  parameter COUNTER_WIDTH = 2,
  parameter TAG_WIDTH = 22,
  parameter INDEX_WIDTH = 8,
  parameter NUM_ENTRIES = 256
) (
  input clk,
  input rstn,

  // update interface
  input update,
  input actually_taken,
  input [DATA_WIDTH-1:0] resolved_pc,

  // access interface
  input [DATA_WIDTH-1:0] pc,

  output reg pred
);
  ///////////////////
  /// wires, regs ///
  ///////////////////
  wire [TAG_WIDTH-1:0] access_tag;
  wire [INDEX_WIDTH-1:0] access_idx;
  reg [INDEX_WIDTH-1:0] access_xored;

  wire [TAG_WIDTH-1:0] update_tag;
  wire [INDEX_WIDTH-1:0] update_idx;
  reg [INDEX_WIDTH-1:0] update_xored;

  // BHR: 8-bit global branch history register
  reg [INDEX_WIDTH-1:0] BHR;
  reg [INDEX_WIDTH-1:0] BHR_snapshot;
  
  // PHT: Pattern History Table
  // each cell: 2-bit saturating counter
  // total entry: 256
  reg [COUNTER_WIDTH-1:0] PHT [0:NUM_ENTRIES-1]; 

  integer i;  

  ///////////////////
  /// initialize  ///
  ///////////////////

   always @(negedge rstn) begin
    BHR = 8'b0000_0000;
    for (i = 0; i < NUM_ENTRIES; i=i+1) begin
      PHT[i] = 2'b01;
    end
  end

  ///////////////////
  /// assignment  ///
  ///////////////////
  assign access_tag = pc[DATA_WIDTH-1:DATA_WIDTH-TAG_WIDTH];
  assign access_idx = pc[INDEX_WIDTH+1:2];

  assign update_tag = resolved_pc[DATA_WIDTH-1:DATA_WIDTH-TAG_WIDTH];
  assign update_idx = resolved_pc[INDEX_WIDTH+1:2];
  
  // access 
  always @(*) begin
    // not branch or jump instruction
    if (pc == 32'h0000_0000) begin
      pred = 1'b0;
    end
    // branch or jump instruction
    else begin
      access_xored = BHR_snapshot ^ access_idx;
      // 2bit saturating counter
      case (PHT[access_xored])
        2'b00: pred = 1'b0; // strongly not taken
        2'b01: pred = 1'b0; // weakly not taken
        2'b10: pred = 1'b1; // weakly taken
        2'b11: pred = 1'b1; // strongly taken
      endcase
    end
  end

  // update snapshot on posedge
  always @(posedge clk) begin
    BHR_snapshot <= BHR;
  end

  // update on negedge
  always @(negedge clk) begin
    if (update == 1'b1) begin 
      update_xored = BHR ^ update_idx;
      // update PHT
      // - update cnt given xored index
      // 2bit saturating counter
      case (PHT[update_xored])
        2'b00: PHT[update_xored] <= actually_taken ? 2'b01 : 2'b00; // strongly not taken
        2'b01: PHT[update_xored] <= actually_taken ? 2'b10 : 2'b00; // weakly not taken
        2'b10: PHT[update_xored] <= actually_taken ? 2'b11 : 2'b01; // weakly taken
        2'b11: PHT[update_xored] <= actually_taken ? 2'b11 : 2'b10; // strongly taken
      endcase

      // update BHR
      // - shift left
      BHR <= {BHR[6:0], actually_taken};
    end
  end

endmodule
  // reg [INDEX_WIDTH-1:0] next_BHR;
  // reg [COUNTER_WIDTH-1:0] next_cnt;
  
  // udpate on negedge
  // always @(negedge clk) begin
  //   if (~rstn) begin // intialize
  //     BHR <= 8'b0000_0000;
  //     PHT #TODO
  //   end
  //   else if (update) begin 
  //     // update PHT
  //     PHT[update_xored] <= next_cnt;
  //     // update BHR
  //     BHR <= next_BHR;
  //   end
  // end

  // always @(*) begin
  //   // update PHT
  //   // - update cnt given xored index
  //   // 2bit saturating counter
  //   case (PHT[update_xored])
  //     2'b00: pred <= 1'b0; // strongly not taken
  //     2'b01: pred <= 1'b0; // weakly not taken
  //     2'b10: pred <= 1'b1; // weakly taken
  //     2'b11: pred <= 1'b1; // strongly taken
  //   endcase

  //   // update BHR
  //   // - shift left
  //   BHR <= {BHR[6:0], actually_taken};

  // end
