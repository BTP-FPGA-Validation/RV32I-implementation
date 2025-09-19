`timescale 1ns / 1ps

module testbench;
    reg clk;
    reg reset;

    top dut (
        .clk(clk),
        .reset(reset)
    );

    always #5 clk = ~clk;

    initial begin
        clk = 0;
        reset = 1;

        $dumpfile("waveform.vcd");
        $dumpvars(0, testbench);

        // Short reset pulse
        #15 reset = 0;
        
        // Run simulation
        #200;
        $finish;
    end

    // Simple monitoring
    always @(posedge clk) begin
        if (!reset) begin
            $display("Time: %0t | PC: %08h | Inst: %08h | RegWrite: %b", 
                     $time, dut.pc_top, dut.instruction_top, dut.RegWrite_top);
        end
    end
endmodule
