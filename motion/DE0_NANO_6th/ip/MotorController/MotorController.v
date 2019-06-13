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
input	[15:0]	iCMD,			// Command
input			iPA,			// Motor Channel A
input			iPB,			// Motor Channel B
input			iRst_n,		// Reset
output			oPWM_Pulse,
output			oDIR,		// Direction of motor
output	[31:0]	oFB,		// Feedback of motor
output			oFB_FREQ,	// Feedback renew trigger
output			oEN,
output			oStop,
output	[5:0]	oStates,
output			oSampClk
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
// wire	[RX_MOTOR_SIZE*8-1:0]	wSPD;		
// wire	[1:0]	wSel;
// wire  	[2:0]	wEN;
// wire			wSTOP;
// wire	[7:0]	wSPD_SM;
// wire	[1:0]	wDIR_DC;
// wire	[1:0]	wDIR_SM;
// wire	[7:0]	wPWM_MUX;
// wire	[6:0] wCMD;


//=======================================================
//  Structural coding
//=======================================================
assign oFB		=	wDFB;
assign oFB_FREQ	=	wST;
assign oPWM_Pulse =  wPWM;
assign oDIR			=  wDIR;
assign oStates = wSel;

// CMDTRAN #(
// 	.STEAM_SIZE(RX_MOTOR_SIZE * 8) //bytes -> bits
// )(
// 	.iCLK (iCLK),
// 	.iCMD (iCMD),
// 	.oSPD (wSPD),
// 	.oDIR (wDIR)
// );


Photo2FB(		// Motor Encoder Feedback
	.iCLK	(iCLK),
	.iRst_n	(iRst_n),
	.oST	(wST),
	.iPA	(iPA),
	.iPB	(iPB),
	.oDB	(wDFB)  //32 bits
//	.oDIR	(oDIR_Now)
);


Normalize #(	// Normoalize speed and feedback
	// .SPD_DIV	(SPD_DIV),
	.CMD_SIZE	(RX_MOTOR_SIZE * 8), //bytes -> bits
	.FB_SIZE	(TX_MOTOR_SIZE * 8)	//bytes -> bits
)(
	.iFB	(wDFB),  //32 bits
	.oFB	(wFB)   //16 bits
);



StateMachine (
	.iCLK		(iCLK),
	.iRst_n	(iRst_n),
	.iFREQ  	(wST),    
	.iCMD		(iCMD),
	.iFB		(wFB),
	
	.oSel		(wSel),
	.oDIR		(wDIR)
);

 SpeedController (
	.iCLK(iCLK),       // 50MHz system clock
	.iRst_n(iRst_n),
	.iFREQ(wST),
//	.iFB(),
	.iSPD_T(wSPD_T),
	.iSel(wSel),

//	.oDIR(wDIR),
	.oDuty(wDuty)
);

pwmgen #(		// PWM generator
	.SPD_DIV	(SPD_DIV),
	.DURY_SIZE	(DURY_SIZE)
)pwm(
	.iClk		(iCLK),
	.iRst_n		(iRst_n),
	.iDuty		(wDuty),
	.oPWM		(wPWM),
	.oSampClk	(oSampClk)
);

MotorStates (
	.iCLK	(iCLK),
	.iRst_n	(iRst_n),
	.iDuty	(wDuty),
	.iSel(wSel),
	.oMotorEnable(oEN),
	.oMotorStop(oStop)
);

Absolute #(
	.SIZE(RX_MOTOR_SIZE*8)
)TaregetVelocity(
	.iValue(iCMD),
	.oValue(wSPD_T),
	.oSign(wDIR_T)
);
/*
Absolute #(
	.SIZE(RX_MOTOR_SIZE * 8)
	
)(
	.iValue(wFB),
	.oValue(wSPD_C),
	.oSign(wDIR_C)
);
*/
endmodule
