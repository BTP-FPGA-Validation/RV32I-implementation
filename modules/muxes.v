// Multiplexers 
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
endmodule

