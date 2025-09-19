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
      Registers[1] <= 72;
      Registers[2] <= 19;
      Registers[3] <= 3;
      Registers[4] <= 5;
      Registers[5] <= 7;
      Registers[6] <= 94;
      Registers[7] <= 4;
      Registers[8] <= 2;
      Registers[9] <= 31;
      Registers[10] <= 13;
      Registers[11] <= 2;
      Registers[12] <= 60;
      Registers[13] <= 12;
      Registers[14] <= 21;
      Registers[15] <= 64;
      Registers[16] <= 41;
      Registers[17] <= 34;
      Registers[18] <= 42;
      Registers[19] <= 51;
      Registers[20] <= 5;
      Registers[21] <= 6;
      Registers[22] <= 65;
      Registers[23] <= 78;
      Registers[24] <= 10;
      Registers[25] <= 57;
      Registers[26] <= 24;
      Registers[27] <= 87;
      Registers[28] <= 7;
      Registers[29] <= 15;
      Registers[30] <= 71;
      Registers[31] <= 89;
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