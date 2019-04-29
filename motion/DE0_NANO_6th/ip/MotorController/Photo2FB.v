// --------------------------------------------------------------------
// Copyright (c) 2012 by Intelligent Control Lab. of Tamkang University. 
// --------------------------------------------------------------------
//
//
// Major Functions: Motor Encoder Feedback
//
// --------------------------------------------------------------------
//
// Revision History :
// --------------------------------------------------------------------
//   Ver  :| Author            :| Mod. Date  :|  Changes Made:
//   0.1  :| Chih-En Wu        :| 2012/08/04 :|  Convert from VHDL version
// --------------------------------------------------------------------

//`default_nettype  none
module Photo2FB (
input				iCLK,			// System Clock
input				iPA,			// Motor Channel A
input				iPB,			// Motor Channel B      
input				iRst_n,		// Reset
output	reg			oST,	// Feedback renew trigger
output	reg			oDIR,	// Dircetion of motor
output	reg	[31:0]	oDB,	// Feedback, 1ms pls
output	[31:0]	oFB_AllTime	// Feedback sum pluse,until reset signal

);

//===========================================================================
// PARAMETER declarations
//===========================================================================

//=======================================================
//  REG/WIRE declarations
//=======================================================

reg		[9:0]	rCNT;
reg		[31:0]	rFB_CNT;
reg		[9:0]	rDB;
wire			wFREQ;
reg				rDFREQ;
reg				rPHA, rPHB;
wire			wDIR;
wire			wPLS;
reg				rD0_A, rD1_A, rD2_A;
reg				rD0_B, rD1_B, rD2_B;
reg				rDLA, rDLB;

//=======================================================
//  Structural coding
//=======================================================

//Clock Divisor
Clkdiv #(
	.EXCEPTCLK	(1000)
) Clk1K (
	.iClk		(iCLK),	// 50Mhz clock 
	.iRst_n	(iRst_n),// Reset
	.oClk		(wFREQ)	// ExpectClk clock
);

// Filter Photo A noise
always @(posedge iCLK)
begin : Filter_A
	rD2_A	<=	rD1_A;
	rD1_A	<=	rD0_A;
	rD0_A	<=	iPA;
	rPHA	<=	(rD0_A & rD1_A & rD2_A) |
				((rD0_A | rD1_A | rD2_A) & rPHA);
end

// Filter Photo B noise
always @(posedge iCLK)
begin : Filter_B
	rD2_B	<=	rD1_B;
	rD1_B	<=	rD0_B;
	rD0_B	<=	iPB;
	rPHB	<=	(rD0_B & rD1_B & rD2_B) |
				((rD0_B | rD1_B | rD2_B) & rPHB);
end

always @(posedge iCLK)
begin : Decoder
	rDLA	<=	rPHA;
	rDLB	<=	rPHB;
end

// Get DIR
assign wDIR	=	(~rPHA & rDLA & ~rPHB) |
				(rPHA & ~rDLA & rPHB) |
				(~rPHB & rDLB & rPHA) |
				(rPHB & ~rDLB & ~rPHA);

// Get Pulse
assign wPLS	=	(~rPHA & rDLA) | (rPHA & ~rDLA) |
				(~rPHB & rDLB) | (rPHB & ~rDLB);

always @(posedge iCLK) begin
	rDFREQ	<=	wFREQ;
end

//// Count feedback for 1ms
//always @(posedge iCLK)
//begin : Counter
//	if (~rDFREQ & wFREQ) begin
//		rCNT	<=	0;
//		//oDIR	<=	0;
//	end
//	else if (wPLS) begin
//		oDIR	<=	wDIR;
//		if (wDIR) begin
//			rCNT	<=	rCNT - 1;
//		end
//		else if (~wDIR) begin
//			rCNT	<=	rCNT + 1;
//		end
//	end
//	else begin
//		rCNT	<=	rCNT;
//	end
//end
//

// Count feedback for all time 
always @(posedge iCLK  or negedge iRst_n)
begin : Counter_AllTime
	if (!iRst_n)
		rFB_CNT <= 0;
  else begin 
		if (wPLS & wDIR) begin
			rFB_CNT	<=	rFB_CNT - 1;
		end
		else if (wPLS & ~wDIR) begin
			rFB_CNT	<=	rFB_CNT + 1;
		end
		else begin
			rFB_CNT	<=	rFB_CNT;
		end
  end
end

assign oFB_AllTime = rFB_CNT;

// Send feedback every 1ms
always @(posedge iCLK)
begin : DataTran
	if (~rDFREQ & wFREQ) begin
		oDB	<=	oFB_AllTime;
		oST	<=	1;
	end
	else begin
		rDB	<=	rDB;
		oST	<=	0;
	end
end

endmodule
