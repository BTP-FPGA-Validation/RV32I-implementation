e
// Instruction Memory
module Instruction_Mem(clk, reset, read_address, inst_out);
input clk, reset; 
input [31:0] read_address;
output reg [31:0] inst_out;
integer k; 
reg [31:0] I_mem[0:63];

initial begin
	// Initialize to 0 (optional, as regs default to 'x', but good practice)
	for(k=0; k<64; k=k+1) begin
		I_mem[k] = 32'd0;
	end
	// Load instructions (done once at sim start)
	I_mem[0] = 32'd0; // No operation
	I_mem[1] = 32'b0000000_11001_10000_000_01101_0110011;	// add x13, x16, x25
	I_mem[2] = 32'b0100000_00011_01000_000_00101_0110011;	// sub x5, x8, x3
	I_mem[3] = 32'b0000000_00011_00010_111_00001_0110011;	// and x1, x2, x3
	I_mem[4] = 32'b0000000_00101_00011_110_00100_0110011;	// or x4, x3, x5
	I_mem[5] = 32'b00000000011_10101_000_10110_0010011;	    // addi x22, x21, 3
	I_mem[6] = 32'b00000000001_01000_110_01001_0010011;	    // ori x9, x8, 1
	I_mem[7] = 32'b00000001111_00101_010_01000_0000011;	    // lw x8, 15(x5)
	I_mem[8] = 32'b00000000011_00011_010_01001_0000011;	    // lw x9, 3(x3)
	I_mem[9] = 32'b0000000_01111_00101_010_01100_0100011;    // sw x15, 12(x5)
	I_mem[10] = 32'b0000000_01110_00110_010_01010_0100011;    // sw x14, 10(x6)
	I_mem[11] = 32'h00948663; // beq x9, x9, 12

end

always @(*) begin
	if(reset)
		inst_out = 32'd0;
	else 
		inst_out = I_mem[read_address[31:2] ];
end

endmodule


