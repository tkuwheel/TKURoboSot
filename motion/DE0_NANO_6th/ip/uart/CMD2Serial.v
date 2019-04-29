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
/*//TEST
output	reg	[7:0]	oChecksum,
output	reg	[31:0]	oMotor1,
output	reg	[31:0]	oMotor2,
output	reg	[31:0]	oMotor3
*/
);

//===========================================================================
// PARAMETER declarations
//===========================================================================
parameter SIZE	=	3;
parameter SEND	=	3'b001;
parameter WAIT	=	3'b010;
parameter END	=	3'b100;

parameter PACKAGE_SIZE	=	15;
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
		rData[7:0]		<=	8'hFF;
		rData[15:8]		<=	8'hFA;
		rData[23:16]	<=	0;
		rData[31:24]	<=	0;
		rData[39:32]	<=	0;
		rData[47:40]	<=	0;
		rData[55:48]	<=	0;
		rData[63:56]	<=	0;
		rData[71:64]	<=	0;
		rData[79:72]	<=	0;
		rData[87:80]	<= 0;
		rData[95:88]	<= 0;
		rData[103:96]	<= 0;
		rData[111:104]	<= 0;
		rData[119:112]	<= 0;		//checksum
/*		rTest1			<= 0;
		rTest2			<= 0;
*/
		end
	// Combine Data
	else if (rSend) begin
		case(state)
			SEND:
				begin
					oTx_data	<=	rData[7:0];
					oTx_send	<=	1'b1;
					if (rTx_busy & ~iTx_busy) begin	// Delay Signal
						oTx_send	<=	1'b0;
						rData		<=	{rData[7:0], rData[STREAM_SIZE-1:8]};
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
		rData[7:0]		<=	8'hFF;
		rData[15:8]		<=	8'hFA;
		rData[23:16]	<=	iFB_Motor1[31:24];
		rData[31:24]	<=	iFB_Motor1[23:16];
		rData[39:32]	<=	iFB_Motor1[15:8];
		rData[47:40]	<=	iFB_Motor1[7:0];
		rData[55:48]	<=	iFB_Motor2[31:24];
		rData[63:56]	<=	iFB_Motor2[23:16];
		rData[71:64]	<=	iFB_Motor2[15:8];
		rData[79:72]	<=	iFB_Motor2[7:0];
		rData[87:80]	<=	iFB_Motor3[31:24];
		rData[95:88]	<=	iFB_Motor3[23:16];
		rData[103:96]	<=	iFB_Motor3[15:8];
		rData[111:104]	<=	iFB_Motor3[7:0];
		rData[119:112]	<= (iFB_Motor1[31:24]+iFB_Motor1[23:16]+iFB_Motor1[15:8]+iFB_Motor1[7:0]+
							iFB_Motor2[31:24]+iFB_Motor2[23:16]+iFB_Motor2[15:8]+iFB_Motor2[7:0]+
							iFB_Motor3[31:24]+iFB_Motor3[23:16]+iFB_Motor3[15:8]+iFB_Motor3[7:0]);	//checksum

/*//	TEST
		rTest1 <= rTest1 + 1;
		rTest2 <= rTest2 - 1;
		rData[7:0]		<=	8'hFF;
		rData[15:8]		<=	8'hFA;
		rData[23:16]	<=	rTest1[31:24];
		rData[31:24]	<=	rTest1[23:16];
		rData[39:32]	<=	rTest1[15:8];
		rData[47:40]	<=	rTest1[7:0];
		rData[55:48]	<=	rTest2[31:24];
		rData[63:56]	<=	rTest2[23:16];
		rData[71:64]	<=	rTest2[15:8];
		rData[79:72]	<=	rTest2[7:0];
		rData[87:80]	<=	0;
		rData[95:88]	<=	0;
		rData[103:96]	<=	0;
		rData[111:104]	<=	rTest2[7:0];
		rData[119:112]	<= ((rTest1[31:24]+rTest1[23:16])+(rTest1[15:8]+rTest1[7:0]))+
							((rTest2[31:24]+rTest2[23:16])+(rTest2[15:8]+rTest2[7:0])+rTest2[7:0]);	//checksum
		oChecksum		<= ((rTest1[31:24]+rTest1[23:16])+(rTest1[15:8]+rTest1[7:0]))+
							((rTest2[31:24]+rTest2[23:16])+(rTest2[15:8]+rTest2[7:0])+rTest2[7:0]);	//checksum
		oMotor1		<=	rTest1;
		oMotor2		<=	rTest2;
		oMotor3		<=	rTest2[7:0];
*/
	end
	rTx_busy	<=	iTx_busy;
end

endmodule
