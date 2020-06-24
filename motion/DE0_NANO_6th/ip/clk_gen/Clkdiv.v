// --------------------------------------------------------------------
// Copyright (c) 2011 by TKU ICLAB. 
// --------------------------------------------------------------------
/*  use follow instruction to call the function

Clkdiv #( .CLKFREQ(50000000), .EXCEPTCLK(10), .multipleX(4) ) UX1
        (   
            .iClk(iCLOCK_50), // 50Mhz clock 
            .oError(oLEDG[0]),  // if CLKFREQ/2 great than ExpectClk, you will get a error
            .oSampClk(oSampClk),  // multipleX expect clock, for SignalTap use
            .oClk(wExpectClk)     // ExpectClk clock
        );
*/
// Revision History :
// --------------------------------------------------------------------
//   Ver  :| Author            :| Mod. Date :| Changes Made:
//   V1.0 :| Shih-An Li        :| 15/03/2011:| 1. Frequency Divider OK
//                                             2. add error led. if CLKFREQ great than iClk /2
//                                                you will get a error.
//   V1.1 :| Shih-An Li        :| 18/05/2011:| 1. Add multipleX parameter
//                                             2. fix oError signal
//   V1.2 :| Shih-An Li        :| 18/05/2011:| 1. Add odd multipleX clock generate
// --------------------------------------------------------------------
`default_nettype none
module Clkdiv #(
	parameter CLKFREQ = 50000000, // Input Clock
	parameter EXCEPTCLK = 1       // Output Clock
)(
	iClk, 		// 50Mhz clock 
    iRst_n,		// Reset
    oError,  	// if CLKFREQCNTVALUE great than iClk /2, you will get a error
    oSampClk,  // multipleX expect clock, for SignalTap use
    oClk     	// ExpectClk clock
);
                 
//===========================================================================
// PARAMETER declarations
//===========================================================================
parameter multipleX = 100; // generate a multiple number of expect clk
parameter N = CLKFREQ /EXCEPTCLK;   // multiple number of divider

//===========================================================================
// PORT declarations
//===========================================================================
input      iClk;  // input 50Mhz clock
input      iRst_n;
output     oError;   // Error of CLKFREQCNTVALUE input
output reg oSampClk; // sample clock
output     oClk;     // output except clock 

//=============================================================================
// REG/WIRE declarations
//=============================================================================
reg [$clog2((CLKFREQ /(EXCEPTCLK*2*multipleX)))-1:0] rSampClkCnt;


reg [$clog2(N)-1:0] rCnt_p;
reg [$clog2(N)-1:0] rCnt_n;
reg                rClk_p;
reg                rClk_n;
//=============================================================================
// Structural coding
//=============================================================================

// Generate expect frequency
assign oClk  = (N == 1) ? iClk :
               (N[0])   ? (rClk_p | rClk_n) : (rClk_p);
        
always@(posedge iClk or negedge iRst_n) begin
  if (!iRst_n)
    rCnt_p <= 0;
  else begin 
    if (rCnt_p == (N-1))		// rCnt_p range is from 0 to (CLKFREQ /EXCEPTCLK)-1
      rCnt_p <= 0;
    else
      rCnt_p <= rCnt_p + 1;	//up count
  end
end

always@(posedge iClk or negedge iRst_n) begin
  if (!iRst_n) 
    rClk_p <= 1;
  else begin
      if (rCnt_p < (N>>1))
        rClk_p = 1;
      else
        rClk_p = 0;    
  end
end

always@(negedge iClk or negedge iRst_n) begin
  if (!iRst_n)
    rCnt_n <= 0;
  else begin
      if (rCnt_n == (N-1))		// rCnt_n range is from 0 to (CLKFREQ /EXCEPTCLK)-1
        rCnt_n <= 0;
      else
        rCnt_n <= rCnt_n + 1;	//up count
  end
end

always@(negedge iClk or negedge iRst_n) begin
  if (!iRst_n)
    rClk_n <= 1;
  else begin
      if (rCnt_n < (N>>1))	
        rClk_n = 1;
      else
        rClk_n = 0;
  end
end



// generate sample clock ( = 2*expect clock) for SignalTap use
always@(posedge iClk or negedge iRst_n) begin
    if (!iRst_n) begin
        oSampClk <= 0;
        rSampClkCnt <= 0;
    end
    else begin
        if(oError) begin 
            oSampClk <= 0;
            rSampClkCnt <= 0;
        end
        else if( rSampClkCnt >= (CLKFREQ /(EXCEPTCLK*2*multipleX)-1 ) )begin
            oSampClk <= ~oSampClk;				// reverse oSampClk
            rSampClkCnt <= 0;
        end 
        else begin
            oSampClk <= oSampClk;  				// stay original form
            rSampClkCnt <= rSampClkCnt +1;	//up count
        end
    end
end

// check except clock small than iClk/2 frequency
assign oError = ((CLKFREQ/2) <= EXCEPTCLK) ? 1'b1: 1'b0;

endmodule

