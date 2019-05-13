// --------------------------------------------------------------------
// Copyright (c) 2019 by Intelligent Control Lab. of Tamkang University. 
// --------------------------------------------------------------------
//
//
// Major Functions: CRC 16
//
// --------------------------------------------------------------------
//
// Revision History :
// --------------------------------------------------------------------
//   Ver  :| Author            :| Mod. Date  :|  Changes Made:
//   1.0  :| Chun-Jui Huang    :| 2019/04/30 :|  Initial version
// --------------------------------------------------------------------
//`default_nettype  none
module Crc16 #(
	parameter PACKAGE_SIZE 	= 	1,
	parameter STREAM_SIZE	=	8
)(
//===========================================================================
// PORT declarations
//===========================================================================
input					iClk,
input					iRst_n,
input					iDataValid,
input			[STREAM_SIZE:0]	iData,
output  reg 	[15:0]	oCrc,
output  reg 			oSuccess,
output  reg	 			oFinish
);


// parameter STREAM_SIZE	=	127;

//=============================================================================
// REG/WIRE declarations
//=============================================================================
reg 					rDataValid;
reg						rReady;

reg	[STREAM_SIZE-1: 0]	rData;
integer i;
// integer end_bit = 127 - 17;
//=============================================================================
// Structural coding
//=============================================================================
always@(posedge iClk) begin
	if(!iRst_n)begin
		oCrc = 16'hffff;
		oSuccess = 0;
		oFinish = 1;
		rDataValid = 0;
	end
	else if(~rDataValid & iDataValid)begin
		rData = iData;
		for(i=0; i<=STREAM_SIZE-17; i=i+1)begin
			if(rData[STREAM_SIZE-1] == 1 )begin
				rData[STREAM_SIZE-1:STREAM_SIZE-17] = rData[STREAM_SIZE-1:STREAM_SIZE-17] ^ 17'h11021;
				rData = (rData << 1);
				// oCrc = rData[71:56];
			end
			else begin
				rData = (rData << 1);
				// oCrc = rData[71:56];
			end
		end
		oCrc = rData[STREAM_SIZE-1:STREAM_SIZE-16];
		oFinish = 1;
		if(oCrc == 16'h0) begin
			oSuccess = 1;
			
		end
		else begin
			oSuccess = 0;
		end
	end
	else begin
		oCrc = oCrc;
		oSuccess = 0;
		oFinish = 0;

		
	end

	rDataValid = iDataValid;
end
endmodule
