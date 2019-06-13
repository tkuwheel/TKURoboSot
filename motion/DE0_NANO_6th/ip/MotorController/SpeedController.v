// --------------------------------------------------------------------
// Copyright (c) 2012 by Intelligent Control Lab. of Tamkang University. 
// --------------------------------------------------------------------
//
//
// Major Functions: Speed controller
//
// --------------------------------------------------------------------
//
// Revision History : Speed Control

// --------------------------------------------------------------------
//   Ver  :| Author            :| Mod. Date  :|  Changes Made:
//   2.0  :| Chun-Jui Huang    :| 2019/05/28 :|  Initial version
// --------------------------------------------------------------------
`default_nettype none
module SpeedController (
//===========================================================================
// PORT declarations
//===========================================================================

input       	      iCLK,       // 50MHz system clock
input       	      iRst_n,
input    	         iFREQ,
input   	[15:0]		iFB,
input   	[15:0]		iSPD_T,
input		[5:0]			iSel,
input		[8:0]     	iDuty_C,
// input   [STREAM_SIZE-1:0]   iSPDErr

//output				oDIR,
output  [8:0]     oDuty
);
// `include "param.h"
//===========================================================================
// PARAMETER declarations
//===========================================================================

parameter   SEL_SIZE = 6;
// parameter	SIZE		=	SEL_SIZE;
parameter	ACCEL		=	6'b000001;	// Accelerate
parameter	DECEL		=	6'b000010;	// Decelerate
parameter	CONST_SPD	=	6'b000100;	// Keep Speed
parameter	START		=	6'b001000;	// Start
// parameter	DIFF_DIR	=	6'b010000;	// Change direction
parameter	STOP		=	6'b100000;	// Stop
//=======================================================
//  REG/WIRE declarations
//=======================================================
wire              wClk;
wire              wDIR;

wire    	[5:0]    wSel;
wire    	[15:0]	wSPD_T;
wire    	[15:0]	wSPD_C;

wire					wDIR_T;
wire					wDIR_C;
wire           	wErr_S;

wire    	[8:0]    wAccelDuty;
wire    	[8:0]    wDeccelDuty;
wire    	[8:0]    wConstDuty;
wire    	[8:0]    wDuty;
reg           	   rST;
//=============================================================================
// Structural coding
//=============================================================================
assign oDuty = wDuty;


AccelController (
    .iCLK(iCLK),			// 50MHz, System Clock
    .iRst_n(iRst_n),		// Reset
    .iFREQ(iFREQ),		// Frequence
    .iSPD_T(iSPD_T),
//    .iDIR_T(wDIR_T),

    .oDuty(wAccelDuty)		// Pulse Width Modulation
);


DecelController (
	.iCLK(iCLK),		// 50MHz, System Clock
	.iRst_n(iRst_n),	// Reset
   .iFREQ(iFREQ),		// Frequence
//    .iDuty_C(wDuty),
	.oDuty(wDeccelDuty),
);

ConstSpdController(
    .iCLK(iCLK),			// 50MHz, System Clock
    .iRst_n(iRst_n),		// Reset
    .iFREQ(iFREQ),		// Frequence
    // .iSPD,			// Speed of motor
    // .iFB,			// Feedback of motor
    .iDuty_C(iDuty_C),
    .oDuty(wConstDuty)		// Pulse Width Modulation
);

MUX (
   .iClk  	(iCLK),
   .iRst_n	(iRst_n),
	.iSel		(iSel),
	.iDuty_0 (wAccelDuty),
   .iDuty_1 (wDeccelDuty),
   .iDuty_2 (wConstDuty),

	.oDuty	(wDuty)
);


endmodule