// Control Unit
module Control_Unit(instruction, Branch, MemRead, MemtoReg, ALUOp, MemWrite, ALUSrc, RegWrite);
 input [6:0] instruction;
 output reg Branch, MemRead, MemtoReg, MemWrite, ALUSrc, RegWrite;
 output reg [1:0] ALUOp;
always @(*) begin
 case(instruction)
  7'b0110011 : {ALUSrc, MemtoReg, RegWrite, MemRead, MemWrite, Branch, ALUOp} <= 8'b001000_10;  // R-type (note: I changed ALUOp to 10 for R-type, common in RISC-V designs)
  7'b0010011 : {ALUSrc, MemtoReg, RegWrite, MemRead, MemWrite, Branch, ALUOp} <= 8'b101000_10;  // I-type (ALUSrc=1, RegWrite=1, ALUOp=10)
  7'b0000011 : {ALUSrc, MemtoReg, RegWrite, MemRead, MemWrite, Branch, ALUOp} <= 8'b111100_00;  // Load
  7'b0100011 : {ALUSrc, MemtoReg, RegWrite, MemRead, MemWrite, Branch, ALUOp} <= 8'b100010_00;  // Store
  7'b1100011 : {ALUSrc, MemtoReg, RegWrite, MemRead, MemWrite, Branch, ALUOp} <= 8'b000001_01;  // Branch
  default    : {ALUSrc, MemtoReg, RegWrite, MemRead, MemWrite, Branch, ALUOp} <= 8'b000000_00;  // Default: no-op to avoid 'x'
 endcase
end

endmodule