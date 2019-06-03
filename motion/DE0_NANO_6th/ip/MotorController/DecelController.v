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
module DecelController #(
parameter STREAM_SIZE = 8,
parameter DUTY_SIZE = 8
)(
input			iCLK,				// 50MHz, System Clock
input			iRst_n,				// Reset
input			iFREQ,				// Frequence
input	[STREAM_SIZE-1:0]	iSPD,	// Speed of motor
input	[STREAM_SIZE-1:0]	iFB,	// Feedback of motor
input	[DUTY_SIZE-1:0]		iDuty_C,	// Currnt Pulse Width Modulation
output	reg [DUTY_SIZE-1:0]		oDuty	// Pulse Width Modulation
);
`include "param.h"
reg		[5:0]	rCNT;	// Count
reg				state;
wire			wClk;
reg				rFREQ;

// // Clock Divisor
// Clkdiv #(
// 	.EXCEPTCLK	(10000)
// ) Clk10K (
// 	.iClk		(iCLK),			// System Clock
// 	.iRst_n		(iRst_n),	// Reset
// 	.oClk		(wClk)			// Clock	Output
// );

// always @(posedge wClk or negedge iRst_n)
// begin
// 	if (!iRst_n) begin	// Reset
// 		oDuty	<=	0;
// 	end
// 	else begin
// 		case (state)
// /////////////////	brake	////////////////
// 			0:	begin
// 				if	(rCNT == 6'd5) begin	// Is rCNT equal to 5
// 					state 	<=	1;			// let state is equal to one
// 					rCNT	<=	0;				// let rCNT is equal to zero
// 					oDIR	<=	2'b00;
// 				end
// 				else begin
// 					rCNT	<=	rCNT+1;
// 					oDIR	<=	2'b11;
// 				end
// 			end
// /////////////////	release	/////////////
// 			1:	begin
// 				if	(rCNT == 6'd15) begin	// Is rCNT equal to 15
// 					state 	<=	0;				// let state is equal to zero
// 					rCNT	<=	0;					// let rCNT is equal to zero
// 					oDIR	<=	2'b11;
// 				end
// 				else begin
// 					rCNT	<=	rCNT+1;	// rCNT 
// 					oDIR	<=	2'b00;
// 					state	<=	state;	// state keep same
// 				end
// 			end
// 		default: begin
// 			rCNT	<=	rCNT;			// rCNT keep same
// 			oDIR	<=	iDIR;
// 			state	<=	state;		// state keep same
// 			end	
// 		endcase
// 	end
// end
always @(posedge iCLK)begin
    if(!iRst_n)
        oDuty <= 0;
    else begin
		rFREQ <= iFREQ;
		if(~rFREQ & iFREQ)
			if(iDuty_C < MIN_DUTY)
					oDuty <= MIN_DUTY;
			else if(iDuty_C > MAX_DUTY)
				oDuty <= MAX_DUTY;
			else 
        		oDuty <= iDuty_C - 4'hF;
		else
			oDuty <= oDuty;
    end


end
endmodule
