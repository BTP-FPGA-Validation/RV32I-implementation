// Data Memory
module Data_Memory(clk, reset, MemWrite, MemRead, read_address, Write_data, MemData_out);
 input clk, reset, MemWrite, MemRead;
 input [31:0] read_address, Write_data;
 output [31:0] MemData_out;
 integer k;
 reg [31:0] D_memory[0:63];
 wire [5:0] word_index = read_address[7:2];
 always @(posedge clk or posedge reset) begin
  if(reset) begin
   for(k=0; k<64; k=k+1) begin
    D_memory[k] <= 32'd0;
   end
  end
  else if(MemWrite) begin
   D_memory[word_index] <= Write_data;
  end
 end
 assign MemData_out = (MemRead) ? D_memory[word_index] : 32'd0;
endmodule