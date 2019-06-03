// --------------------------------------------------------------------
// Copyright (c) 2012 by Intelligent Control Lab. of Tamkang University. 
// --------------------------------------------------------------------
//
//
// Major Functions: Accelerator
//
// --------------------------------------------------------------------
//
// Revision History :
// --------------------------------------------------------------------
//   Ver  :| Author            :| Mod. Date  :|  Changes Made:
//   0.1  :| Chun-Jui Huang    :| 2019/05/31 :|  Initial Version
// --------------------------------------------------------------------
// `include "param.h"
`default_nettype  none
module AccelController #(
parameter STREAM_SIZE = 8,
parameter DUTY_SIZE = 8
)(
//===========================================================================
// PORT declarations
//===========================================================================
input			iCLK,				// 50MHz, System Clock
input			iRst_n,				// Reset
input			iFREQ,				// Frequence
input		[STREAM_SIZE-1:0]	iSPD_T,
input							iDIR_T,
input		[STREAM_SIZE-1:0]	iSPD_C,
input							iDIR_C,
input		[STREAM_SIZE-1:0]	iSpeedErrVal,
input						iSmaller,
input		[DUTY_SIZE-1:0]		iDuty_C,		// Currnt Pulse Width Modulation
output	reg [DUTY_SIZE-1:0]		oDuty		// Pulse Width Modulation
);
//===========================================================================
// PARAMETER declarations
//===========================================================================
`include "param.h"
parameter CCW = 1'b1;
parameter CW = 1'b0;
//=======================================================
//  REG/WIRE declarations
//=======================================================
reg				rFREQ;

wire signed	[7:0]	wErr_0;
reg signed	[7:0]	rErr_1;

wire signed	[7:0]	wP_Term;
wire signed	[7:0]	wI_Term;

reg signed	[8:0]	rPWM_0;
reg signed	[8:0]	rPWM_1;

//=======================================================
//  Structural coding
//=======================================================

// assign	wErr_0	=	iSPD - iFB;
// assign	oPWM 	= 	rPWM_0;

assign	wP_Term	=	wErr_0 / P_GAIN;
assign	wI_Term	=	rErr_1 / I_GAIN;

// always@(posedge iCLK)
// begin
// 	if(!iRst_n) begin
// 		rErr_1	<=	0;
// 		rPWM_0	<=	0;
// 		rPWM_1	<=	0;
//     end
//     else begin
// 		rFREQ	<=	iFREQ;
// 		 if ( ~rFREQ & iFREQ ) begin	
// 			rPWM_1	<=	rPWM_0;
// 			rErr_1	<=	wErr_0;
// 			if(wErr_0 <= 9'sd0) begin
// 				rPWM_0	<=	0;
// 			end
// 			// PWM Output : max=127, min=0
// 			else if ((rPWM_1 + wP_Term - wI_Term) >= 9'sd127) begin
// 				rPWM_0	<=	9'd127;
// 			end
// 			else if ((rPWM_1 + wP_Term - wI_Term) <= 9'sd0) begin
// 				rPWM_0	<=	0;
// 			end
// 			else
// 				rPWM_0	<=	rPWM_1 + wP_Term - wI_Term;
// 		end
// 		else begin
// 			rErr_1	<=	rErr_1;
// 			rPWM_0	<=	rPWM_0;
// 			rPWM_1	<=	rPWM_1;
// 		end
// 	end
// end
always @(posedge iCLK)begin
    if(!iRst_n)
        oDuty <= 0;
    else begin
		rFREQ <= iFREQ;
		if(~rFREQ & iFREQ) begin
			case(iDIR_T)
			CW: begin
				if(iDuty_C < MIN_DUTY)
					oDuty <= MIN_DUTY;
				else if(iDuty_C > MAX_DUTY)
					oDuty <= MAX_DUTY;
				else begin
					if(iSmaller)
						oDuty <= iDuty_C - 1;
					else 
						oDuty <= iDuty_C + 1;
				end
			end
			CCW: begin
				if(iDuty_C < MIN_DUTY)
					oDuty <= MIN_DUTY;
				else if(iDuty_C > MAX_DUTY)
					oDuty <= MAX_DUTY;
				else begin
					if(iSmaller)
						oDuty <= iDuty_C + 1;
					else 
						oDuty <= iDuty_C - 1;
				end
			end
			default:
				oDuty <= 0;
			
			endcase
        end

		else
			oDuty <= oDuty;
    end

end
endmodule
