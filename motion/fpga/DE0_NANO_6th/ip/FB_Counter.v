// --------------------------------------------------------------------
// Copyright (c) 2012 by Intelligent Control Lab. of Tamkang University. 
// --------------------------------------------------------------------
//
//
// Major Functions: Feedback Delay Count
//
// --------------------------------------------------------------------
//
// Revision History :
// --------------------------------------------------------------------
//   Ver  :| Author            :| Mod. Date  :|  Changes Made:
//   0.1  :| Chih-En Wu        :| 2012/08/09 :|  Initial Version, create with Quartus II 9.1 SP2
// --------------------------------------------------------------------

`default_nettype  none
module FB_Counter (
//===========================================================================
// PORT declarations
//===========================================================================
input				iCLK,				// 50MHz, System Clock
input				iRst_n,			// Reset
input		[31:0]	iFB,				// Feedback of motor Input
input				iSend,
input				iFB_FREQ,		
output	reg	[31:0]	oFB,	// Feedback of motor Output
output	reg			oFB_FREQ
);

//===========================================================================
// PARAMETER declarations
//===========================================================================

//=======================================================
//  REG/WIRE declarations
//=======================================================

reg				rFB_FREQ;

reg				rSend;

reg		[31:0]	rFB_CNT;

//=======================================================
//  Structural coding
//=======================================================
always @(posedge iCLK) begin
	if (!iRst_n) begin
		rFB_CNT	<=	0;
	end
	else begin
		if (~rFB_FREQ & iFB_FREQ) begin	//Delay Signal
			rFB_CNT	<=	rFB_CNT + iFB;
			if (rFB_CNT > 15'h 7FFF) begin	// rFB_CNT is range is from 0 to 15'h 7FFF
				rFB_CNT	<=	0;
			end
		end
		if (~rSend & iSend) begin		//Delay Signal
			oFB			<=	rFB_CNT;
			rFB_CNT		<=	0;
			oFB_FREQ	<=	1;
		end
		else begin
			oFB_FREQ	<=	0;
		end
		rFB_FREQ	<=	iFB_FREQ;
		rSend		<=	iSend;
	end
end

endmodule
