// --------------------------------------------------------------------
// Copyright (c) 2012 by Intelligent Control Lab. of Tamkang University. 
// --------------------------------------------------------------------
//
//
// Major Functions: Keep Speed
//
// --------------------------------------------------------------------
//
// Revision History :
// --------------------------------------------------------------------
//   Ver  :| Author            :| Mod. Date  :|  Changes Made:
//   0.1  :| Chun-Jui Huang    :| 2019/05/31 :|  Initial Version
// --------------------------------------------------------------------
module ConstSpdController(
input			iCLK,			    // 50MHz, System Clock
input			iRst_n,		        // Reset
input			iFREQ,		        // Frequence
//input	[15:0]	iSPD,	// Speed of motor
//input	[15:0]	iFB,	// Feedback of motor
input			[8:0]	iDuty_C,		// Currnt Pulse Width Modulation

output	reg	[8:0]	oDuty		// Pulse Width Modulation
);
reg rFREQ;
always@(posedge iCLK)
begin
	if(!iRst_n) begin
		oDuty	<=	0;
    end
    else begin
			rFREQ <= iFREQ;
			if(~rFREQ & iFREQ)
				oDuty <= iDuty_C;
			else
				oDuty <= oDuty;
	end
end

endmodule