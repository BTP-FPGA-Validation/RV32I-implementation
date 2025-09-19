// ALU 
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
endmodule