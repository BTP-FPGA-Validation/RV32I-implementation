
//Register File 
module Reg_File(clk, reset, RegWrite, Rs1, Rs2, Rd, Write_data, read_data1, read_data2);
 input clk, reset, RegWrite; 
 input [4:0] Rs1, Rs2, Rd ;
 input [31:0] Write_data;
 output [31:0] read_data1, read_data2;
 integer k;
 reg [31:0] Registers[31:0];
 reg initialized;
 
 initial begin
  initialized = 0;
 end
 
 always @(posedge clk or posedge reset) begin
  if(reset) begin 
  	for(k=0; k<32; k=k+1) begin
  	 Registers[k] <= 32'd0;
  	end
  	initialized <= 0;
  end
  else begin
    // Initialize registers with test values after reset
    if (!initialized) begin
      Registers[0] <= 0;
      initialized <= 1;
    end
    else if(RegWrite && Rd != 0) begin // x0 should always be 0 in RISC-V
      Registers[Rd] <= Write_data;
    end
  end
 end
 
 assign read_data1 = Registers[Rs1];
 assign read_data2 = Registers[Rs2];
endmodule


