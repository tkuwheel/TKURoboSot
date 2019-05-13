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
//`default_nettype  none
module Serial2CMD (
//===========================================================================
// PORT declarations
//===========================================================================
input				iCLK, 		// 50MHz
input				iRst_n,		// Reset
input				iRx_ready,
input		[7:0]	iData,		// Data
output	reg	[7:0]	oCMD_Motor1,	// Command of motor1
output	reg	[7:0]	oCMD_Motor2,	// Command of motor2
output	reg	[7:0]	oCMD_Motor3,	// Command of motor3
output	reg	[7:0]	oSignal,		// Command of EN&STOP
output	reg	[7:0]	oKick,			// shoot a ball 
output	reg			oRx_done,
output 	reg	[15:0]	oCrc,			// CRC debug
output	reg			oCrcSuccess,
output 	reg	[7:0]	debug			//  debug

);

//===========================================================================
// PARAMETER declarations
//===========================================================================
//parameter SIZE	=	8;
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
parameter END	=	8'hFF;

//=============================================================================
// REG/WIRE declarations
//=============================================================================
reg		[71:0] 	rPacket;
reg		[7:0]	rData_0, rData_1, rData_2, rData_3, rData_4, rData_5,rData_6, rData_7, rData_8;	//	divide information to 6 part and 8 bits per part
reg		[7:0]	state;
reg				rRx_ready;
reg				rCheck;
reg		[7:0]	rChecksum;
reg		[7:0]	null = 8'h0;

wire				wCrcFinish;
wire				wCrcSuccess;
wire		[15:0]	wCrc;
wire		[7:0]	wCMD_Motor1, wCMD_Motor2, wCMD_Motor3, wSignal, wKick;

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
		oKick		<=	0;

		oCrcSuccess	<=	0;
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
						// rCheck <= 0;
						if( iData == 8'hFF ) begin	// when getting initiation packet, state jump next
							rData_0	<=	iData;
							state	<=	DATA1;
						end
					end
				DATA1:
					begin
						// rCheck <= 0;
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
						// rCheck <= 0;
						state	<=	DATA3;
					end
				DATA3:				
					begin
						rData_3	<=	iData;		//motor2
						// rCheck <= 0;
						state	<=	DATA4;
					end
				DATA4:
					begin
						rData_4	<=	iData;		//motor3
						// rCheck <= 0;
						state	<=	DATA5;
					end
				DATA5:
					begin
						rData_5	<=	iData;		//enable+stop
						// rCheck <= 0;
						state	<=	DATA6;
					end
				DATA6:
					begin
						rData_6	<=	iData;		//shoot
						// rCheck <= 0;
						state	<=	DATA7;
					end
				DATA7:
					begin
						rData_7	<=	iData;		//crc_1
						// rCheck <= 0;
						state	<=	DATA8;
					end
				DATA8:
					begin
						rData_8	<=	iData;		//crc_2
						// rCheck <= 0;
						state	<=	END;
					end
				END:
					begin
						
						rPacket <= {rData_0,rData_1,rData_2,rData_3,rData_4,rData_5,rData_6,rData_7,rData_8};
						
						// rCheck <= 1;
						oRx_done	<=	1;
						state	<=	DATA0;
					end
						

			endcase
		end
		else begin

			oCMD_Motor1 	<= 	wCMD_Motor1;
			oCMD_Motor2 	<= 	wCMD_Motor2;
			oCMD_Motor3 	<= 	wCMD_Motor3;
			oSignal 		<= 	wSignal;
			oKick			<= 	wKick;
			oCrcSuccess		<=	wCrcSuccess;
			debug	<= wCrcFinish;
			// rCheck 			<=	0;
			oRx_done		<=	0;
			oCrc <= wCrc;
		end
		
		rRx_ready	<=	iRx_ready;
	end
end
Crc16 #(
	.PACKAGE_SIZE(9),
	.STREAM_SIZE(72)
	) Crc_RX (
	.iClk(iCLK),
	.iRst_n(iRst_n),
	.iDataValid(oRx_done),
	.iData(rPacket),
	.oCrc(wCrc),
	.oSuccess(wCrcSuccess),
	.oFinish(wCrcFinish)
);
Packet2CMD packet(
	.iClk(iCLK),
	.iRst_n(iRst_n),
	.iDataValid(wCrcSuccess),
	.iPacket(rPacket),
	.oMotor1(wCMD_Motor1),
	.oMotor2(wCMD_Motor2),
	.oMotor3(wCMD_Motor3),
	.oEN(wSignal),
	.oShoot(wKick)
);

endmodule

