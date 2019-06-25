// --------------------------------------------------------------------
// Copyright (c) 2012 by Intelligent Control Lab. of Tamkang University. 
// --------------------------------------------------------------------
//
//
// Major Functions: Decelerator
//
// --------------------------------------------------------------------
//
// Revision History :
// --------------------------------------------------------------------
//   Ver  :| Author            :| Mod. Date  :|  Changes Made:
//   0.1  :| Chun-Jui Huang    :| 2019/05/31 :|  Initial Version
// --------------------------------------------------------------------
`default_nettype none
module DecelController(
input			iCLK,				// 50MHz, System Clock
input			iRst_n,				// Reset
input			iFREQ,				// Frequence
//input	[15:0]	iSPD,	// Speed of motor
//input	[15:0]	iFB,	// Feedback of motor
input	[8:0]		iDuty_Curr,	// Currnt Pulse Width Modulation
output	reg [8:0]		oDuty	// Pulse Width Modulation
);
reg				rFREQ;
`include "param.h"
always @(posedge iCLK)begin
    if(!iRst_n)
        oDuty <= 0;
    else begin
		rFREQ <= iFREQ;
		if(~rFREQ & iFREQ)
			
			oDuty <= iDuty_Curr - 4'hf;
		else
			oDuty <= oDuty;
    end
end
endmodule
