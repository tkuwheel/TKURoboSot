`timescale 10ns / 1ns
module  test1(
        iA,
        iB,
        
        oSum,
        oCarry
);
input   [3:0]   iA;
input   [3:0]   iB;

output  [3:0]   oSum;
output          oCarry;

wire    [4:0]   wTotal;

assign  wTotal = iA + iB;

assign  {oCarry, oSum} = {wTotal[4], wTotal[3:0]};


endmodule
