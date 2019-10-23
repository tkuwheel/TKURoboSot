// --------------------------------------------------------------------
// Copyright (c) 2012 by Intelligent Control Lab. of Tamkang University. 
// --------------------------------------------------------------------
//
// Major Functions: Motor Controller
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
//   1.0  :| Chun-Jui Huang    :| 2019/05/27 :|  use rpm as taget velocity (-Max rpm ~ max rpm)
// --------------------------------------------------------------------

`default_nettype  none
module MotorController (
//===========================================================================
// PORT declarations
//===========================================================================
input			iCLK,			// 50MHz, System Clock
input			iRst_n,		// Reset
input			iPA,			// Motor Channel A
input			iPB,			// Motor Channel B
input [15:0] iCMD,		// Command

output oPWM_Pulse,
output oDIR,				// Direction of motor
output oFB_FREQ,			// Feedback renew trigger
output [31:0]	oFB,		// Feedback of motor
output [5:0]	oStates,
output oSampClk
);
`include "param.h"
//===========================================================================
// PARAMETER declarations
//===========================================================================
parameter SPD_DIV = 4;
parameter DURY_SIZE = 9;
// parameter FB_STREAM_SIZE = TX_MOTOR_SIZE * 8;
//=======================================================
//  REG/WIRE declarations
//=======================================================
wire			wPWM;	//output pwm pulse
wire			wDIR;	//output direction CW:1 CCW:0
wire			wST;	//feedback frequency

wire	[31:0]	wDFB;		//current speed	(signed) for transmit to PC
wire	[15:0]	wFB;		//current speed	(signed)
wire	[8:0]			wDuty;		//target duty cycle

wire	[15:0]	wSPD_T;		//currnet speed	(unsigned)
wire	wDIR_C;								//current direction
wire	wDIR_T;

wire	[5:0]	wSel;


//=======================================================
//  Structural coding
//=======================================================
assign oFB		=	wDFB;
assign oFB_FREQ	=	wST;
assign oPWM_Pulse =  wPWM;
assign oDIR			=  wDIR;
assign oStates = wSel;

 CMDTRAN #(
 	.SIZE(RX_MOTOR_SIZE * 8) //bytes -> bits
 )(
 	.iCLK (iCLK),
 	.iCMD (iCMD),
 	.oSPD (wSPD_T),
 	.oDIR (wDIR)
 );

pwmgen#(
	.SPD_DIV(SPD_DIV)
)(
	.iClk(iCLK),		// 25Mhz clock
	.iRst_n(iRst_n),	// reset, low active
	.iDuty(wSPD_T),	// Range is 0~128*SPD_DIV
	.oPWM(wPWM),
	.oSampClk(oSampClk),
	.oErrorValue()
);

Photo2FB(		// Motor Encoder Feedback
	.iCLK	(iCLK),
	.iRst_n	(iRst_n),
	.oST	(wST),
	.iPA	(iPA),
	.iPB	(iPB),
	.oDB	(wDFB)  //32 bits
//	.oDIR	(oDIR_Now)
);



endmodule
