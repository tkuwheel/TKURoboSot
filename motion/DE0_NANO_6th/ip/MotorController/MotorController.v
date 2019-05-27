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
output			oFB_FREQ// Feedback renew trigger
);
`include "param.h"
//===========================================================================
// PARAMETER declarations
//===========================================================================
parameter SPD_DIV = 1;
// parameter FB_STREAM_SIZE = TX_MOTOR_SIZE * 8;
//=======================================================
//  REG/WIRE declarations
//=======================================================
wire	[1:0]		wDIR;
wire	[1:0]	wDIR_DC;
wire	[1:0]	wDIR_SM;
wire	[7:0]	wPWM;
wire	[7:0]	wPWM_MUX;
wire	[RX_MOTOR_SIZE*8 -2:0]	wSPD;
wire	[7:0]	wSPD_SM;
wire			wST;
wire	[TX_MOTOR_SIZE*8 -1:0]	wDB;
wire	[RX_MOTOR_SIZE-8 -1:0]	wFB;
wire	[1:0]	wSel;
wire  	[2:0]	wEN;
wire			wSTOP;


wire  [6:0] wCMD;

//=======================================================
//  Structural coding
//=======================================================
assign oFB		=	wDB;
assign oFB_FREQ	=	wST;
assign oPWM_Pulse =  wPWM;
assign oDIR			=  wDIR;



CMDTRAN #(
	.STEAM_SIZE(RX_MOTOR_SIZE * 8) //bytes -> bits
)(
	.iCLK (iCLK),
	.iCMD (iCMD),
	.oSPD (wSPD),
	.oDIR (wDIR)
);


Photo2FB (		// Motor Encoder Feedback
	.iCLK	(iCLK),
	.iRst_n	(iRst_n),
	.oST	(wST),
	.iPA	(iPA),
	.iPB	(iPB),
	.oDB	(wDB),  //32 bits
	.oDIR	(oDIR_Now)
);



Normalize #(	// Normoalize speed and feedback
	.SPD_DIV	(SPD_DIV),
	.CMD_SIZE	(RX_MOTOR_SIZE * 8), //bytes -> bits
	.FB_SIZE	(TX_MOTOR_SIZE * 8)	//bytes -> bits
)(
	.iFB	(wDB),  //32 bits
	.oFB	(wFB)   //16 bits
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
pwmgen #(		// PWM generator
	.SPD_DIV	(SPD_DIV)
)pwm(
	.iClk		(iCLK),
	.iRst_n	(iRst_n),
	.iDuty	(wSPD),
	.oPWM		(wPWM)
	// .oSampClk	(wSampClk)
);

SpeedMonitor #(
	.CMD_SIZE	(RX_MOTOR_SIZE * 8), //bytes -> bits
	.FB_SIZE	(TX_MOTOR_SIZE * 8)	//bytes -> bits
)SM(
	.iCLK	(iCLK), 
	.iRst_n	(iRst_n),
	.iFB	(wDB),
	.iNew_FB(wFB),
	.iPWM	(wPWM),
	.iCMD	(wSPD)
	// .oFB	(),
	// .oNew_FB(),
	// .oPWM	(),
	// .oCMD	()
	);
endmodule
