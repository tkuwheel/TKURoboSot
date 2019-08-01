// --------------------------------------------------------------------
// Copyright (c) 2019 by Intelligent Control Lab. of Tamkang University. 
// --------------------------------------------------------------------
//
//
// Major Functions: Convert packet into robot command
//
// --------------------------------------------------------------------
//
// Revision History :
// --------------------------------------------------------------------
//   Ver  :| Author            :| Mod. Date  :|  Changes Made:
//   1.0  :| Chun-Jui Huang    :| 2019/05/8 :|  Initial version
// --------------------------------------------------------------------
`default_nettype  none

module Packet2CMD #(
	parameter STREAM_SIZE	=	8,
	parameter MOTOR_STREAM_SIZE = 16
)(

//===========================================================================
// PORT declarations
//===========================================================================
input 					iClk,
input 					iRst_n,
input 					iDataValid,
input 		[63:0]		iPacket,
output reg 	[MOTOR_STREAM_SIZE-1:0]		oMotor1,
output reg 	[MOTOR_STREAM_SIZE-1:0]		oMotor2,
output reg 	[MOTOR_STREAM_SIZE-1:0]		oMotor3,
output reg 	[7:0]		oEN,
output reg 	[7:0]		oShoot
);
`include "param.h"
always@(posedge iClk)begin
	if(!iRst_n)begin
		oMotor1 <= 0;
		oMotor2 <= 0;
		oMotor3 <= 0;
		oEN <= 0;
		oShoot <= 0;
	end
	else begin
		if(iDataValid)begin
			oMotor1 <= iPacket[63:48];
			oMotor2 <= iPacket[47:32];
			oMotor3 <= iPacket[31:16];
			oEN <= iPacket[15:8];
			oShoot <= iPacket[7:0];
		end
		else begin
			oMotor1 <= oMotor1;
			oMotor2 <= oMotor2;
			oMotor3 <= oMotor3;
			oEN <= oEN;
			oShoot <= oShoot;
		end
	end
end
endmodule