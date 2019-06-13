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
module AccelController(
//===========================================================================
// PORT declarations
//===========================================================================
input			iCLK,				// 50MHz, System Clock
input			iRst_n,				// Reset
input			iFREQ,				// Frequence
input		[15:0]	iSPD_T,
//input							iDIR_T,
//input		[15:0]	iSPD_C,
//input							iDIR_C,
//input		[15:0]	iSpeedErrVal,
//input							iSmaller,
//input		[8:0]		iDuty_C,		// Currnt Pulse Width Modulation
output	reg [8:0]		oDuty		// Pulse Width Modulation
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

reg [15:0]	rErr_0;
reg [15:0]	rErr_1;

wire [15:0]	wP_Term;
wire [15:0]	wI_Term;

//reg signed	[8:0]	rPWM_0;
//reg signed	[8:0]	rPWM_1;

//=======================================================
//  Structural coding
//=======================================================

// assign	wErr_0	=	iSPD - iFB;
// assign	oPWM 	= 	rPWM_0;

assign	wP_Term	=	rErr_0 / P_GAIN;
assign	wI_Term	=	rErr_1 / I_GAIN;

//always @(posedge iCLK)begin
//    if(!iRst_n)
//        oDuty <= 0;
//    else begin
//		rFREQ <= iFREQ;
//		rErr_0 <= iSpeedErrVal;
//		rErr_1 <= rErr_0 + iSpeedErrVal;
//		if(~rFREQ & iFREQ) begin
//			case(iDIR_T)
//			CW: begin
//				if(iDuty_C < MIN_DUTY)
//					oDuty <= MIN_DUTY;
//				else if(iDuty_C > MAX_DUTY)
//					oDuty <= MAX_DUTY;
//				else begin
//					if(iSmaller)
//						oDuty <= iDuty_C - wP_Term - wI_Term;
//					else 
//						oDuty <= iDuty_C + wP_Term + wI_Term;
//				end
//			end
//			CCW: begin
//				if(iDuty_C < MIN_DUTY)
//					oDuty <= MIN_DUTY;
//				else if(iDuty_C > MAX_DUTY)
//					oDuty <= MAX_DUTY;
//				else begin
//					if(iSmaller)
//						oDuty <= iDuty_C + wP_Term + wI_Term;
//					else 
//						oDuty <= iDuty_C - wP_Term - wI_Term;
//				end
//			end
//			default:
//				oDuty <= 0;
//			
//			endcase
//        end
//
//		else
//			oDuty <= oDuty;
//    end
//
//end
always @(posedge iCLK)begin
    if(!iRst_n)
        oDuty <= 0;
    else begin
		rFREQ <= iFREQ;
		if(~rFREQ & iFREQ) begin
			if(iSPD_T>MAX_DUTY)
				oDuty <= MAX_DUTY;
			else if(iSPD_T<MIN_DUTY)
				oDuty <= MIN_DUTY;
			else
				oDuty <= iSPD_T;
		end
		else
			oDuty <= oDuty;
		
	end
end
endmodule
