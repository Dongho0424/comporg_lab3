// data_memory.v

module data_memory #(
  parameter DATA_WIDTH = 32, MEM_ADDR_SIZE = 14
)(
  input  clk,
  input  mem_write,
  input  mem_read,
  input  [1:0] maskmode,
  input  sext,
  input  [DATA_WIDTH-1:0] address,
  input  [DATA_WIDTH-1:0] write_data,

  output reg [DATA_WIDTH-1:0] read_data
);

  // memory
  // memory cell의 크기가 32b인 data_mem.
  // cell의 개수는 2^14
  reg [DATA_WIDTH-1:0] mem_array [0:2**MEM_ADDR_SIZE-1]; // change memory size
  initial $readmemh("data/data_memory.mem", mem_array);
  // wire reg for writedata
  wire [MEM_ADDR_SIZE-1:0] address_internal; // 2^14 = 14-bit addresses

  assign address_internal = address[MEM_ADDR_SIZE+1:2]; // 2^14 = 14-bit addresses

  // update at negative edge
  always @(negedge clk) begin 
    if (mem_write == 1'b1) begin
      ////////////////////////////////////////////////////////////////////////
      // \TODO : Perform writes (select certain bits from write_data
      // according to maskmode
      ////////////////////////////////////////////////////////////////////////
      case (maskmode) 
        2'b00: mem_array[address_internal] = {mem_array[address_internal][DATA_WIDTH-1:8], write_data[7:0]}; // sb
        2'b01: mem_array[address_internal] = {mem_array[address_internal][DATA_WIDTH-1:16], write_data[15:0]}; // sh
        2'b10: mem_array[address_internal] = write_data; // sw
        default: mem_array[address_internal] = write_data; // default case
      endcase
    end
  end

  // combinational logic
  always @(*) begin
    if (mem_read == 1'b1) begin
      ////////////////////////////////////////////////////////////////////////
      // \TODO : Perform reads (select bits according to sext & maskmode)
      ////////////////////////////////////////////////////////////////////////
      case ({maskmode, sext}) 
        3'b000: read_data = {{24{mem_array[address_internal][7]}}, mem_array[address_internal][7:0]}; // lb 
        3'b010: read_data = {{16{mem_array[address_internal][15]}}, mem_array[address_internal][15:0]}; // lh
        3'b100: read_data = mem_array[address_internal]; // lw
        3'b001: read_data = {{24{1'b0}}, mem_array[address_internal][7:0]}; // lbu, zero-extended
        3'b011: read_data = {{16{1'b0}}, mem_array[address_internal][15:0]}; // lhu, zero-extended
        default: read_data = mem_array[address_internal]; // default case
      endcase
    end else begin
      read_data = 32'h0000_0000;
    end
  end

endmodule
