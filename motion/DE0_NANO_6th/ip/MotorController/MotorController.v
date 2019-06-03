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
output			oDIR_Now,// Direction of motor now
output			oFB_FREQ,// Feedback renew trigger
output			oEN,
output			oStop,
output	[5:0]	oStates
// output			oSampClk
);
`include "param.h"
//===========================================================================
// PARAMETER declarations
//===========================================================================
parameter SPD_DIV = 5;
parameter DURY_SIZE = 8 + SPD_DIV/2;
// parameter FB_STREAM_SIZE = TX_MOTOR_SIZE * 8;
//=======================================================
//  REG/WIRE declarations
//=======================================================
wire			wDIR;
wire	[1:0]	wDIR_DC;
wire	[1:0]	wDIR_SM;
wire	[7:0]	wPWM;
wire	[7:0]	wPWM_MUX;
wire	[RX_MOTOR_SIZE*8 -1:0]	wSPD;
wire	[7:0]	wSPD_SM;
wire			wST;
wire	[TX_MOTOR_SIZE*8 -1:0]	wDFB;
wire	[RX_MOTOR_SIZE*8 -1:0]	wFB;
wire	[1:0]	wSel;
wire  	[2:0]	wEN;
wire			wSTOP;


wire	[6:0] wCMD;
wire	[DURY_SIZE-1:0]	wDuty;
wire	[RX_MOTOR_SIZE*8-1:0]wSPD_C;
wire	wDIR_C;
//=======================================================
//  Structural coding
//=======================================================
assign oFB		=	wDFB;
assign oFB_FREQ	=	wST;
assign oPWM_Pulse =  wPWM;
assign oDIR			=  wDIR;



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
	.oDB	(wDFB),  //32 bits
	.oDIR	(oDIR_Now)
);

Normalize #(	// Normoalize speed and feedback
	// .SPD_DIV	(SPD_DIV),
	.CMD_SIZE	(RX_MOTOR_SIZE * 8), //bytes -> bits
	.FB_SIZE	(TX_MOTOR_SIZE * 8)	//bytes -> bits
)(
	.iFB	(wDFB),  //32 bits
	.oFB	(wFB)   //16 bits
);



SpeedController #(
	.STREAM_SIZE		(RX_MOTOR_SIZE * 8),
	.DUTY_SIZE			(DURY_SIZE)
)(
	.iCLK	(iCLK),
	.iRst_n	(iRst_n),
	.iFREQ	(wST),
	.iFB	(wFB),
	.iCMD	(iCMD),
	.oDuty	(wDuty),
	.oDIR	(wDIR),
	.oStates(oStates)
);

pwmgen #(		// PWM generator
	.SPD_DIV	(SPD_DIV)
)pwm(
	.iClk		(iCLK),
	.iRst_n		(iRst_n),
	.iDuty		(wDuty),
	.oPWM		(wPWM)
	// .oSampClk	(oSampClk)
);

MotorStates #(
	.SIZE(DURY_SIZE),
	.DUTY_SIZE(DURY_SIZE)
)(
	.iCLK	(iCLK),
	.iRst_n	(iRst_n),
	.iSPD_C	(wSPD_C),
	.iDuty(wDuty),
	.oMotorEnable(oEN),
	.oMotorStop(oStop)
);

Absolute #(
	.SIZE(RX_MOTOR_SIZE * 8)
	
)(
	.iValue(wFB),
	.oValue(wSPD_C),
	.oSign(wDIR_C)
);
/*
StateMachine (	// Motor Controller State Machine
	.iCLK	(iCLK),
	.iSPD	(wSPD),
	.iDIR	(wDIR),
	.iFB	(wFB),
	.iRst_n	(iRst_n),
	.oSel	(wSel),
	.oDIR	(wDIR_SM),
	.oSPD	(wSPD_SM)
);

AccelController (
	.iCLK	(iCLK),
	.iRst_n	(iRst_n),
	.iFREQ	(wST),
	.iSPD	(wSPD_SM),
	.iFB	(wFB),
	.oPWM	(wPWM_MUX)
);

DecelController (
	.iCLK	(iCLK),
	.iRst_n	(iRst_n),
	.iDIR	(wDIR_SM),
	.oDIR	(wDIR_DC)
);

MUX (
	.iSel	(wSel),
	.iPWM0	(wPWM),
	.iDIR0	(wDIR_SM),
	.iDIR1	(wDIR_DC),
	.oDIR	(oDIR),
	.oPWM	(oPWM_Pulse)
);
*/
// wire wSampClk;
endmodule
