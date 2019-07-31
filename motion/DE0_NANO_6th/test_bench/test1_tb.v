`timescale 10ns / 1ns
module test1_tb;

// Inputs
reg    [3:0]    iA, iB;
wire   [3:0]    wSum;
wire            wCarry;

// assert simulation signals
 initial begin
  // Initialize Inputs
    iA=0;   iB=0;
  // Wait 100 ns 
  #100  // time 100ns
        iA=0; iB=5;
  #100  // time 200ns 
        iA=1; iB=5;
  #100  // time 300ns
        iA=3; iB=5;
  #100  // time 400ns
        iA=5; iB=5;
  #100  // time 500ns
        iA=7; iB=5;
  #100  // time 600ns
        iA=9; iB=5;
  #100  // time 700ns
        iA=11; iB=5;
  #100  // time 800ns
        iA=13; iB=5;
  #100  // time 900ns
        iA=15; iB=5;
  #100  // time 1000ns 
        iA=15; iB=10;
  #100  // time 1100ns
        iA=15; iB=15;
end


 
test1   u1(
        .iA(iA),
        .iB(iB),
        
        .oSum(wSum),
        .oCarry(wCarry)
);
 


endmodule
