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
//`default_nettype  none
module Packet2CMD(
input 					iClk,
input 					iRst_n,
input 					iDataValid,
input 		[71:0]	iPacket,
output reg 	[7:0]		oMotor1,
output reg 	[7:0]		oMotor2,
output reg 	[7:0]		oMotor3,
output reg 	[7:0]		oEN,
output reg 	[7:0]		oShoot
);

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
			oMotor1 <= iPacket[55:48];
			oMotor2 <= iPacket[47:40];
			oMotor3 <= iPacket[39:32];
			oEN <= iPacket[31:24];
			oShoot <= iPacket[23:16];
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