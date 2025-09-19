// ALU Control
module ALU_Control(ALUOp, fun7, fun3, Control_out);
 input fun7;
 input [2:0] fun3;
 input [1:0] ALUOp;
 output reg [3:0] Control_out;
 always @(*) begin
  case({ALUOp, fun7, fun3})
   6'b00_0_000: Control_out <= 4'b0010; // add (loads)
   6'b01_0_000: Control_out <= 4'b0110;  // beq
   6'b10_0_000: Control_out <= 4'b0010;// R-type add
   6'b10_1_000: Control_out <= 4'b0110;   // R-type sub
   6'b10_0_111: Control_out <= 4'b0000;	// and 
   6'b10_0_110: Control_out <= 4'b0001; // or
   default:  Control_out <= 4'b0010; // default to add
  endcase
 end
endmodule// ALU 
module ALU_unit(A, B, Control_in, ALU_Result, zero);
 input [31:0] A, B;
 input [3:0] Control_in;
 output reg zero; 
 output reg [31:0] ALU_Result;
 always @(*) begin
  // default
  ALU_Result = 32'd0;
  zero = 1'b0;

  case(Control_in)
   4'b0000: begin ALU_Result = A & B; end 
   4'b0001: begin ALU_Result = A | B; end 
   4'b0010: begin ALU_Result = A + B; end 
   4'b0110: begin 
     ALU_Result = A - B;
     zero = (A == B) ? 1'b1 : 1'b0;
   end
   default: begin ALU_Result = 32'd0; zero = 1'b0; end
  endcase
  end
endmodule// All modules instantiated

module top(clk, reset);
 input clk, reset;
 wire [31:0] pc_top, instruction_top, Rd1_top, Rd2_top, ImmExt_top, mux1_top, Sum_out_top, NextoPC_top, PCin_top, address_top, Memdata_top, WriteBack_top;
 wire RegWrite_top, ALUSrc_top, zero_top, branch_top, sel2_top, MemtoReg_top, MemWrite_top, MemRead_top;
 wire [1:0] ALUOp_top;
 wire [3:0] control_top;

// Program Counter
 pc Program_Counter(.clk(clk), .reset(reset), .pc_in(PCin_top), .pc_out(pc_top));

// PC Adder 
 PCplus4 PC_Adder(.fromPC(pc_top), .NextoPC(NextoPC_top));

// Instruction Memory
 Instruction_Mem Inst_Mem(.clk(clk), .reset(reset), .read_address(pc_top), .inst_out(instruction_top));

// Register File
 Reg_File Reg_File (.clk(clk), .reset(reset), .RegWrite(RegWrite_top), .Rs1(instruction_top[19:15]), .Rs2(instruction_top[24:20]), .Rd(instruction_top[11:7]), .Write_data(WriteBack_top), .read_data1(Rd1_top), .read_data2(Rd2_top));

// Immediate Generator
 ImmGen ImmGen(.Opcode(instruction_top[6:0]), .instruction(instruction_top), .ImmExt(ImmExt_top));

// Control Unit
 Control_Unit Control_Unit(.instruction(instruction_top[6:0]), .Branch(branch_top), .MemRead(MemRead_top), .MemtoReg(MemtoReg_top), .ALUOp(ALUOp_top), .MemWrite(MemWrite_top), .ALUSrc(ALUSrc_top), .RegWrite(RegWrite_top));

// ALU Control
 ALU_Control ALU_Control(.ALUOp(ALUOp_top), .fun7(instruction_top[30]), .fun3(instruction_top[14:12]), .Control_out(control_top));

// ALU
  ALU_unit ALU(.A(Rd1_top), .B(mux1_top), .Control_in(control_top), .ALU_Result(address_top), .zero(zero_top));

// ALU Mux
 Mux1 ALU_mux(.sel1(ALUSrc_top), .A1(Rd2_top), .B1(ImmExt_top), .Mux1_out(mux1_top));

// Adder
 Adder Adder(.in1(pc_top), .in2(ImmExt_top), .Sum_out(Sum_out_top));

// AND Gate
 AND_logic AND(.branch(branch_top), .zero(zero_top), .and_out(sel2_top));

// Mux
 Mux2 Adder_mux(.sel2(sel2_top), .A2(NextoPC_top), .B2(Sum_out_top), .Mux2_out(PCin_top));

// Data Memory
 Data_Memory Data_mem(.clk(clk), .reset(reset), .MemWrite(MemWrite_top), .MemRead(MemRead_top), .read_address(address_top), .Write_data(Rd2_top), .MemData_out(Memdata_top));

// Mux
 Mux3 Memory_mux(.sel3(MemtoReg_top), .A3(address_top), .B3(Memdata_top), .Mux3_out(WriteBack_top));
endmodule
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

endmodule// Data Memory
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
endmodule// Immediate Generator
 module ImmGen(Opcode, instruction, ImmExt);
 input [6:0] Opcode;
 input [31:0] instruction;
 output reg [31:0] ImmExt;
always @(*) begin
 case(Opcode)
  7'b0010011 : ImmExt <= {{20{instruction[31]}}, instruction[31:20]};  // I-type (same as load)
  7'b0000011 : ImmExt <= {{20{instruction[31]}}, instruction[31:20]};  // Load
  7'b0100011 : ImmExt <= {{20{instruction[31]}}, instruction[31:25], instruction[11:7]};  // Store (fixed typo in your original)
  7'b1100011 : ImmExt <= {{19{instruction[31]}}, instruction[31], instruction[7], instruction[30:25], instruction[11:8], 1'b0};  // Branch (fixed to standard SB format)
  default    : ImmExt <= 32'd0;  // Avoid 'x'
 endcase 
end

 endmodule// Instruction Memory
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
I_mem[0] = 32'b0000000_11001_10000_000_01101_0110011;	// add x13, x16, x25
I_mem[1] = 32'b0100000_00011_01000_000_00101_0110011;	// sub x5, x8, x3
I_mem[2] = 32'b0000000_00011_00010_111_00001_0110011;	// and x1, x2, x3
I_mem[3] = 32'b0000000_00101_00011_110_00100_0110011;	// or x4, x3, x5
I_mem[4] = 32'b00000000011_10101_000_10110_0010011;	    // addi x22, x21, 3
I_mem[5] = 32'b00000000001_01000_110_01001_0010011;	    // ori x9, x8, 1
I_mem[6] = 32'b00000001111_00101_010_01000_0000011;	    // lw x8, 15(x5)
I_mem[7] = 32'b00000000011_00011_010_01001_0000011;	    // lw x9, 3(x3)
I_mem[8] = 32'b0000000_01111_00101_010_01100_0100011;    // sw x15, 12(x5)
I_mem[9] = 32'b0000000_01110_00110_010_01010_0100011;    // sw x14, 10(x6)
I_mem[10] = 32'h00948663; // beq x9, x9, 12

 end

always @(*) begin
  if(reset)
  	inst_out = 32'd0;
  else 
  	inst_out = I_mem[read_address[31:2] ];
end

endmodule// Multiplexers 

// Mux 1
module Mux1(sel1, A1, B1, Mux1_out);
 input sel1;
 input [31:0] A1, B1;
 output [31:0] Mux1_out;
 assign Mux1_out = (sel1==1'b0) ? A1:B1;
endmodule

// Mux 2
module Mux2(sel2, A2, B2, Mux2_out);
 input sel2;
 input [31:0] A2, B2;
 output [31:0] Mux2_out;
 assign Mux2_out = (sel2==1'b0) ? A2:B2;
endmodule

// Mux 3
module Mux3(sel3, A3, B3, Mux3_out);
 input sel3;
 input [31:0] A3, B3;
 output [31:0] Mux3_out;
 assign Mux3_out = (sel3==1'b0) ? A3:B3;
endmodule

// AND logic
module AND_logic(branch, zero, and_out);
 input branch, zero;
 output and_out;
 assign and_out = branch & zero;
endmodule

// Adder
module Adder(in1, in2, Sum_out);
 input [31:0] in1, in2;
 output [31:0] Sum_out;
 assign Sum_out = in1 + in2;
endmodule// PC + 4 
module PCplus4(fromPC, NextoPC);
input [31:0] fromPC;
output [31:0] NextoPC;
assign NextoPC = fromPC + 32'd4;
endmodule// Program Counter
module pc(clk, reset, pc_in, pc_out);
 input clk, reset;
 input [31:0] pc_in;
 output reg [31:0] pc_out;
 always @(posedge clk or posedge reset) begin
  if(reset) pc_out <= 32'd0;
  else pc_out <= pc_in;
  end
endmodule//Register File 
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