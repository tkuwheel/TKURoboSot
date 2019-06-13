// --------------------------------------------------------------------
// Copyright (c) 2012 by Intelligent Control Lab. of Tamkang University. 
// --------------------------------------------------------------------
//
//
// Major Functions: Convert serial to command
//
// --------------------------------------------------------------------
//
// Revision History :
// --------------------------------------------------------------------
//   Ver  :| Author            :| Mod. Date  :|  Changes Made:
//   0.1  :| Chih-en Wu        :| 2012/07/19 :|  Initial Version
// --------------------------------------------------------------------
// --------------------------------------------------------------------
//   Ver  :| Author            :| Mod. Date  :|  Changes Made:
//   1.8  :| Chun-Jui Huang    :| 2017/07/07 :|  Add Checksum and Shoot Control
// --------------------------------------------------------------------
// --------------------------------------------------------------------
//   Ver  :| Author            :| Mod. Date  :|  Changes Made:
//   2.0  :| Chun-Jui Huang    :| 2017/07/07 :|  use Crc16
// --------------------------------------------------------------------
`default_nettype  none
module Serial2CMD #(
	MOTOR_STREAM_SIZE = 16
)(
//===========================================================================
// PORT declarations
//===========================================================================
input				iCLK, 		// 50MHz
input				iRst_n,		// Reset
input				iRx_ready,
input		[7:0]	iData,		// Data
output	reg	[MOTOR_STREAM_SIZE-1:0]	oCMD_Motor1,	// Command of motor1
output	reg	[MOTOR_STREAM_SIZE-1:0]	oCMD_Motor2,	// Command of motor2
output	reg	[MOTOR_STREAM_SIZE-1:0]	oCMD_Motor3,	// Command of motor3
output	reg	[7:0]	oSignal,		// Command of EN&STOP
output	reg	[7:0]	oPower,			// shoot a ball 
output	reg			oRx_done,
output 	reg	[15:0]	oCrc,			// CRC debug
output	reg			oCrcSuccess,
output 	reg	[7:0]	debug			//  debug

);
`include "param.h"
//===========================================================================
// PARAMETER declarations
//===========================================================================
parameter PACKAGE_SIZE	=	RX_PACKAGE_SIZE;
parameter STREAM_SIZE	=	PACKAGE_SIZE * 8;
// differentiate state in order to change state
parameter DATA0	=	0;
parameter DATA1	=	1;
parameter DATA2	=	2;
parameter DATA3	=	3;
parameter DATA4	=	4;
parameter DATA5	=	5;
parameter DATA6	=	6;
parameter DATA7	=	7;
parameter DATA8	=	8;
parameter DATA9	=	9;
parameter DATA10	=	10;
parameter DATA11	=	11;
parameter END	=	8'hFF;

//=============================================================================
// REG/WIRE declarations
//=============================================================================
reg		[STREAM_SIZE-1-32:0] 	rPacket;
reg		[7:0]	rData_0, rData_1, rData_2, rData_3, rData_4, rData_5; 	//
reg		[7:0] rData_6, rData_7, rData_8, rData_9, rData_10, rData_11;	//	divide information to 11 part and 8 bits per part
reg		[7:0]	state;
reg				rRx_ready;
reg				rCheck;
reg		[7:0]	rChecksum;
// reg		[7:0]	null = 8'h0;

wire				wCrcFinish;
wire				wCrcSuccess;
wire		[15:0]	wCrc;
wire		[MOTOR_STREAM_SIZE-1:0]	wCMD_Motor1, wCMD_Motor2, wCMD_Motor3;
wire		[7:0]	wSignal, wPower;

//=============================================================================
// Structural coding
//=============================================================================

always @(posedge iCLK) begin
	if(!iRst_n) begin		// Reset
		oRx_done	<=	0;
		rData_0		<=	0;
		rData_1		<=	0;
		rData_2		<=	0;
		rData_3		<=	0;
		rData_4		<=	0;
		rData_5		<=	0;
		rData_6		<=	0;
		rData_7		<=	0;
		state		<=	DATA0;
		oCMD_Motor1	<=	0;
		oCMD_Motor2	<=	0;
		oCMD_Motor3	<=	0;
		oSignal		<=	0;
		oPower		<=	0;

		oCrcSuccess	<=	0;
		rPacket		<= 	0;
		// rError		<=	1;
		// rCheck		<= 0;
		debug <= 0;

	end
	// Take apart Data
	else begin
		
//		rChecksum <= rData_2 + rData_3 + rData_4 + rData_5 + rData_6;

		if(~rRx_ready & iRx_ready) begin
			oCrc <= oCrc;
			oCrcSuccess	<=	oCrcSuccess;
			debug	<= debug;
			// rError <= rError;
			case(state)
				DATA0:
					begin
						rCheck <= 0;
						if( iData == 8'hFF ) begin	// when getting initiation packet, state jump next
							rData_0	<=	iData;
							state	<=	DATA1;
						end
					end
				DATA1:
					begin
						rCheck <= 0;
						if( iData == 8'hFA ) begin	//when getting second initiation packet, start to receive and transmit Data
							rData_1	<=	iData;
							state	<=	DATA2;
						end
						else begin
							state	<=	DATA0;		
						end
					end
				DATA2:				
					begin
						rData_2	<=	iData;		//motor1
						rCheck <= 0;
						state	<=	DATA3;
					end
				DATA3:				
					begin
						rData_3	<=	iData;		
						rCheck <= 0;
						state	<=	DATA4;
					end
				DATA4:
					begin
						rData_4	<=	iData;		//motor2
						rCheck <= 0;
						state	<=	DATA5;
					end
				DATA5:
					begin
						rData_5	<=	iData;		
						rCheck <= 0;
						state	<=	DATA6;
					end
				DATA6:
					begin
						rData_6	<=	iData;		//motor3
						rCheck <= 0;
						state	<=	DATA7;
					end
				DATA7:
					begin
						rData_7	<=	iData;		
						rCheck <= 0;
						state	<=	DATA8;
					end
				DATA8:
					begin
						rData_8	<=	iData;		//enable+stop
						rCheck <= 0;
						state	<=	DATA9;
					end
				DATA9:
					begin
						rData_9	<=	iData;		//shoot
						rCheck <= 0;
						state	<=	DATA10;
					end
				DATA10:
					begin
						rData_10	<=	iData;		//crc_1
						rCheck <= 0;
						state	<=	DATA11;
					end
				DATA11:
					begin
						rData_11	<=	iData;		//crc_2
						rPacket <= {rData_2,rData_3,rData_4,rData_5,rData_6,rData_7,rData_8,rData_9};
						rCheck <= 1;
						state	<=	END;
					end
				END:
					begin
						
						
						
						rCheck <= 0;
						oRx_done	<=	1;
						state	<=	DATA0;
					end
						

			endcase
		end
		else begin
			rPacket	<= rPacket;
			oCMD_Motor1 	<= 	wCMD_Motor1;
			oCMD_Motor2 	<= 	wCMD_Motor2;
			oCMD_Motor3 	<= 	wCMD_Motor3;
			oSignal 		<= 	wSignal;
			oPower			<= 	wPower;
			oCrcSuccess		<=	wCrcSuccess;
			debug			<=	wCrcFinish;
			rCheck 			<=	rCheck;
			oRx_done		<=	0;
			oCrc <= wCrc;
		end
		
		rRx_ready	<=	iRx_ready;
	end
end
Crc16 #(
	.STREAM_SIZE(STREAM_SIZE)
	) Crc_RX (
	.iClk(iCLK),
	.iRst_n(iRst_n),
	.iDataValid(rCheck),
	.iData({rData_0,rData_1,rData_2,rData_3,rData_4,rData_5,rData_6,rData_7,rData_8,rData_9,rData_10,rData_11}),
	.oCrc(wCrc),
	.oSuccess(wCrcSuccess),
	.oFinish(wCrcFinish)
);

Packet2CMD #(
	.STREAM_SIZE(STREAM_SIZE - 32), // remove ff fa crc_1 crc_2
	.MOTOR_STREAM_SIZE(MOTOR_STREAM_SIZE)
	) Packet (
	.iClk(iCLK),
	.iRst_n(iRst_n),
	.iDataValid(wCrcSuccess),
	.iPacket(rPacket),
	.oMotor1(wCMD_Motor1),
	.oMotor2(wCMD_Motor2),
	.oMotor3(wCMD_Motor3),
	.oEN(wSignal),
	.oShoot(wPower)
);

endmodule

