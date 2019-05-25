// --------------------------------------------------------------------
// Copyright (c) 2012 by Intelligent Control Lab. of Tamkang University. 
// --------------------------------------------------------------------
//
//
// Major Functions: Convert feedback to serial
//
// --------------------------------------------------------------------
//
// Revision History :
// --------------------------------------------------------------------
//   Ver  :| Author            :| Mod. Date  :|  Changes Made:
//   0.1  :| Chih-en Wu        :| 2012/08/05 :|  Initial Version
// --------------------------------------------------------------------
// --------------------------------------------------------------------
//   Ver  :| Author            :| Mod. Date  :|  Changes Made:
//   1.9  :| Chun-Jui Huang    :| 2017/07/07 :|  Add Checksum
// --------------------------------------------------------------------

`default_nettype  none
module CMD2Serial (
//===========================================================================
// PORT declarations
//===========================================================================
input				iCLK, 	// 50MHz
input				iRst_n,	// Reset
input				iSend_Ready,
input				iTx_busy,
input		[31:0]	iFB_Motor1,		// Feedback of motor1
input		[31:0]	iFB_Motor2,		// Feedback of motor2
input		[31:0]	iFB_Motor3,		// Feedback of motor3
//input		[14:0]	iFB_Motor4,		// Feedback of motor4
input				iDIR_Motor1,		// Direction of motor1
input				iDIR_Motor2,		// Direction of motor2
input				iDIR_Motor3,		// Direction of motor3
//input				iDIR_Motor4,		// Direction of motor4
output	reg			oTx_send,
output	reg	[7:0]	oTx_data
);
`include "param.h"
//===========================================================================
// PARAMETER declarations
//===========================================================================
parameter SIZE	=	3;
parameter SEND	=	3'b001;
parameter WAIT	=	3'b010;
parameter END	=	3'b100;

parameter PACKAGE_SIZE	=	TX_PACKAGE_SIZE;
parameter STREAM_SIZE	=	PACKAGE_SIZE * 8;
parameter DELAY			=	1000;

//=============================================================================
// REG/WIRE declarations
//=============================================================================

reg	[STREAM_SIZE-1:0]	rData;

reg	[SIZE-1:0]	state;

reg				rSend_Ready;

reg				rSend;

reg		[7:0]	rCNT_Package;

reg		[15:0]	rCNT;

reg				rTx_busy;
reg				rDataReady;
reg				rCheckReady;

wire	[15:0]	wCrc;
wire 			wCrcFinish;
wire	[STREAM_SIZE-1 : 0] wData;
/*
reg		[31:0]	rTest1;
reg		[31:0]	rTest2;
*/
//=============================================================================
// Structural coding
//=============================================================================


always @(posedge iCLK) begin
	if ( (~rSend_Ready & iSend_Ready) & ~rSend) begin
		rSend	<=	1;
	end
	rSend_Ready	<=	iSend_Ready;

	if (!iRst_n) begin	// Reset
		state	<=	SEND;
		rSend	<=	0;
		rCNT	<=	0;
		rCNT_Package	<=	0;
		rDataReady		<=	0;
		rCheckReady		<=	1;
		rData			<=	0;
/*		rTest1			<= 0;
		rTest2			<= 0;
*/
		end
	// Combine Data
	else if (rSend) begin
		rDataReady <= 0;
		if (rCheckReady) begin
			case(state)
				SEND:
					begin
						oTx_data	<=	rData[STREAM_SIZE - 1 :STREAM_SIZE -8];
						oTx_send	<=	1'b1;
						if (rTx_busy & ~iTx_busy) begin	// Delay Signal
							oTx_send	<=	1'b0;
							rData		<=	{rData[STREAM_SIZE-9:0], rData[STREAM_SIZE-1 :STREAM_SIZE-8]};
							state		<=	WAIT;
						end
					end
				WAIT:
					begin
						if (rCNT < DELAY) begin
							rCNT	<=	rCNT + 1;	// up count
						end
						else begin
							rCNT	<=	0;
							state	<=	END;
						end
					end
				END:
					begin
						if (rCNT_Package < PACKAGE_SIZE-1) begin
							rCNT_Package	<=	rCNT_Package + 1;	// up count
							state			<=	SEND;
						end
						else begin
							rCNT_Package	<=	0;
							state			<=	SEND;
							rSend			<=	0;
						end
					end
			endcase
		end
		else begin
			rSend <= 0;
		end
	end
	else begin 
		if(~wCrcFinish)begin

			rCheckReady		<= 	0;
			rDataReady		<=	1;

		end
		else begin
			rCheckReady		<=	1;
			rDataReady		<=	0;
			rData			<=	wData;
		end
	end
	rTx_busy	<=	iTx_busy;
end

Crc16 #(
	.STREAM_SIZE(STREAM_SIZE)		//PACKAGE SIZE *　8
	) Crc_TX (
	.iClk(iCLK),
	.iRst_n(iRst_n),
	.iDataValid(rDataReady),
	.iData({8'hFF, 8'hFA, iFB_Motor1 ,iFB_Motor2, iFB_Motor3, 16'h0}),
	// .iData({8'hFF, 8'hFA, 16'h0 , 16'h0, 16'h0, 16'h0}),
	.oCrc(wCrc),
	.oSuccess(),
	.oFinish(wCrcFinish),
	.oData(wData)
);

endmodule
