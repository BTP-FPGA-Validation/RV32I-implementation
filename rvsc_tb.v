`timescale 1ns / 1ps

module testbench;
    reg clk;
    reg reset;

    top dut (
        .clk(clk),
        .reset(reset)
    );

    // Clock generation
    always #5 clk = ~clk;

    initial begin
    clk = 0;
    reset = 1;

    $dumpfile("waveform.vcd");
    $dumpvars(0, testbench);

    // Short reset pulse
    #15 reset = 0;

    // Initialize registers after reset
    dut.Reg_File.Registers[1]  = 7;
    dut.Reg_File.Registers[2]  = 19;
    dut.Reg_File.Registers[3]  = 3;
    dut.Reg_File.Registers[4]  = 5;
    dut.Reg_File.Registers[5]  = 7;
    dut.Reg_File.Registers[6]  = 9;
    dut.Reg_File.Registers[7]  = 4;
    dut.Reg_File.Registers[8]  = 2;
    dut.Reg_File.Registers[9]  = 31;
    dut.Reg_File.Registers[10] = 13;
    dut.Reg_File.Registers[11] = 2;
    dut.Reg_File.Registers[12] = 6;
    dut.Reg_File.Registers[13] = 12;
    dut.Reg_File.Registers[14] = 21;
    dut.Reg_File.Registers[15] = 4;
    dut.Reg_File.Registers[16] = 5;
    dut.Reg_File.Registers[17] = 34;
    dut.Reg_File.Registers[18] = 4;
    dut.Reg_File.Registers[19] = 1;
    dut.Reg_File.Registers[20] = 5;
    dut.Reg_File.Registers[21] = 6;
    dut.Reg_File.Registers[22] = 5;
    dut.Reg_File.Registers[23] = 7;
    dut.Reg_File.Registers[24] = 10;
    dut.Reg_File.Registers[25] = 7;
    dut.Reg_File.Registers[26] = 24;
    dut.Reg_File.Registers[27] = 8;
    dut.Reg_File.Registers[28] = 7;
    dut.Reg_File.Registers[29] = 15;
    dut.Reg_File.Registers[30] = 11;
    dut.Reg_File.Registers[31] = 9;

    // Run simulation
    #200;
    $finish;
end


    // Simple monitoring
    always @(posedge clk) begin
        if (!reset) begin
            $display("Time=%0t | PC=%08h | Inst=%08h | rs1=%0d(%08h) | rs2=%0d(%08h) | rd=%0d | ALUout=%08h | WB=%08h | RegWrite=%b", 
                     $time, 
                     dut.pc_top, dut.instruction_top,
                     dut.instruction_top[19:15], dut.Rd1_top,
                     dut.instruction_top[24:20], dut.Rd2_top,
                     dut.instruction_top[11:7], dut.address_top, 
                     dut.WriteBack_top, dut.RegWrite_top);
        end
    end
endmodule

