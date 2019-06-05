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

`default_nettype  none
module MotorController (
//===========================================================================
// PORT declarations
//===========================================================================
input			iCLK,			// 50MHz, System Clock
input	[7:0]	iCMD,			// Command
input			iPA,			// Motor Channel A
input			iPB,			// Motor Channel B
input			iRst_n,		// Reset
output			oPWM_Pulse,
output			oDIR,		// Direction of motor
output	[31:0]	oFB,		// Feedback of motor
output			oDIR_Now,// Direction of motor now
output			oFB_FREQ// Feedback renew trigger
);

//===========================================================================
// PARAMETER declarations
//===========================================================================
parameter SPD_DIV = 1;

//=======================================================
//  REG/WIRE declarations
//=======================================================
wire	[1:0]		wDIR;
wire	[1:0]	wDIR_DC;
wire	[1:0]	wDIR_SM;
wire	[7:0]	wPWM;
wire	[7:0]	wPWM_MUX;
wire	[7:0]	wSPD;
wire	[7:0]	wSPD_SM;
wire			wST;
wire	[31:0]	wDB;
wire	[7:0]	wFB;
wire	[1:0]	wSel;
wire  [2:0]	wEN;
wire			wSTOP;


wire  [6:0] wCMD;

//=======================================================
//  Structural coding
//=======================================================
assign oFB		=	wDB;
assign oFB_FREQ	=	wST;
assign oPWM_Pulse =  wPWM;
assign oDIR			=  wDIR;



CMDTRAN (
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
	.oDB	(wDB),  //10 bits
	.oDIR	(oDIR_Now)
);


/*
Normalize #(	// Normoalize speed and feedback
	.SPD_DIV	(SPD_DIV),
	.FB_MAX	(190)
)(
	.iFB	(wDB),  //10 bits
	.oFB	(wFB)   // 8 bits
);
*/

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
pwmgen #(		// PWM generator
	.SPD_DIV	(SPD_DIV)
)(
	.iClk		(iCLK),
	.iRst_n	(iRst_n),
	.iDuty	(wSPD),
	.oPWM		(wPWM)
);

endmodule
