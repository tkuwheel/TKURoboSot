// --------------------------------------------------------------------
// Copyright (c) 2012 by Intelligent Control Lab. of Tamkang University. 
// --------------------------------------------------------------------
//
//
// Major Functions: MUX
//
// --------------------------------------------------------------------
//
// Revision History :
// --------------------------------------------------------------------
//   Ver  :| Author            :| Mod. Date  :|  Changes Made:
//   0.1  :| Chih-Yang Yang    :| 2012/08/10 :|  Initial Version
// --------------------------------------------------------------------
`default_nettype  none
module AccelController #(
//===========================================================================
// PARAMETER declarations
//===========================================================================
parameter P_GAIN = 3,
parameter I_GAIN = 10
)(
//===========================================================================
// PORT declarations
//===========================================================================
input			iCLK,			// 50MHz, System Clock
input			iRst_n,		// Reset
input			iFREQ,		// Frequence
input	[7:0]	iSPD,			// Speed of motor
input	[7:0]	iFB,			// Feedback of motor

output	[7:0]	oPWM		// Pulse Width Modulation
);

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

assign	wErr_0	=	iSPD - iFB;
assign	oPWM 	= 	rPWM_0;

assign	wP_Term	=	wErr_0 / P_GAIN;
assign	wI_Term	=	rErr_1 / I_GAIN;

always@(posedge iCLK)
begin
	if(!iRst_n) begin
		rErr_1	<=	0;
		rPWM_0	<=	0;
		rPWM_1	<=	0;
    end
    else begin
		rFREQ	<=	iFREQ;
		 if ( ~rFREQ & iFREQ ) begin	
			rPWM_1	<=	rPWM_0;
			rErr_1	<=	wErr_0;
			if(wErr_0 <= 9'sd0) begin
				rPWM_0	<=	0;
			end
			// PWM Output : max=127, min=0
			else if ((rPWM_1 + wP_Term - wI_Term) >= 9'sd127) begin
				rPWM_0	<=	9'd127;
			end
			else if ((rPWM_1 + wP_Term - wI_Term) <= 9'sd0) begin
				rPWM_0	<=	0;
			end
			else
				rPWM_0	<=	rPWM_1 + wP_Term - wI_Term;
		end
		else begin
			rErr_1	<=	rErr_1;
			rPWM_0	<=	rPWM_0;
			rPWM_1	<=	rPWM_1;
		end
	end
end

endmodule
