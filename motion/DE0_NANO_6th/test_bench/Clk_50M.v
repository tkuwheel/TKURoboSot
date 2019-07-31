`timescale 10 ns/ 1 ns
module Clk_50M(
    output reg clk
);

// reg clk;
parameter clkper = 20; //20ns = 50 MHz

initial begin
    clk = 0;
end
always begin
    #1 clk <= ~clk;
end
endmodule