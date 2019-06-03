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
module SpeedController #(
parameter STREAM_SIZE = 16,
parameter DUTY_SIZE = 7
//===========================================================================
// PORT declarations
//===========================================================================
)(
input                       iCLK,       // 50MHz system clock
input                       iRst_n,
input                       iFREQ,
input   [STREAM_SIZE-1:0]   iFB,
input   [STREAM_SIZE-1:0]   iCMD,
// input   [STREAM_SIZE-1:0]   iSPDErr,
output                      oDIR,

output  [5:0]               oStates,
output  [DUTY_SIZE-1:0]     oDuty
);
// `include "param.h"
//===========================================================================
// PARAMETER declarations
//===========================================================================
`include "param.h"
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
wire                        wClk;
wire                        wDIR;

wire    [SEL_SIZE-1:0]      wSel;
wire    [STREAM_SIZE-1:0]	wSPD_T;
wire    [STREAM_SIZE-1:0]	wSPD_C;

wire	wDIR_T;
wire	wDIR_C;
wire    [STREAM_SIZE-1:0]   wSpdErrVal;
wire                        wErr_S;

wire    [DUTY_SIZE-1:0]     wAccelDuty;
wire    [DUTY_SIZE-1:0]     wDeccelDuty;
wire    [DUTY_SIZE-1:0]     wConstDuty;
wire    [DUTY_SIZE-1:0]     wDuty;
reg                         rST;
reg    [DUTY_SIZE-1:0]      rDuty;
//=============================================================================
// Structural coding
//=============================================================================
assign oDuty = wDuty;
assign oStates = wSel;
Clkdiv #(
	.EXPECTCLK	(FEEDBACK_FREQUENCY)
)(
    .iClk		(iCLK),  // 50Mhz clock
    .iRst_n		(iRst_n),// reset
    // .oSampClk	(oSampClk),
    .oClk		(wClk)
);

// always @(posedge wClk)begin
//     if(!iRst_n)
//         rDuty <= 0;
//     else begin
//         if(wSpdErrVal != 0) begin
//             if(wErr_S)begin
//                 rDuty <= rDuty - 1;
//             end
//             else begin
//                 if(rDuty >= MAX_DUTY)
//                     rDuty <= rDuty;
//                 else 
//                     rDuty <= rDuty + 1;
//             end
//         end
//         else begin
//             if(rDuty < MIN_DUTY)
//                 rDuty <= 0 ;
//             else
//                 rDuty <= rDuty;
//         end
//     end


// end

StateMachine #(	// Motor Controller State Machine
    .STREAM_SIZE    (STREAM_SIZE),
    .DUTY_SIZE      (DUTY_SIZE),
    .SEL_SIZE		(SEL_SIZE)
)(
	.iCLK	(iCLK),
    .iRst_n	(iRst_n),
    .iFREQ  (iFREQ),    
    .iErrVal(wSpdErrVal),
    .iSPD_T(wSPD_T),
    .iDIR_T(wDIR_T),
    .iSPD_C(wSPD_C),
    .iDIR_C(wDIR_C),
	.oSel	(wSel),
	.oDIR	(oDIR)
	// .oSPD	(wSPD_SM)
);

AccelController #(
    .STREAM_SIZE(STREAM_SIZE),
    .DUTY_SIZE(DUTY_SIZE)
)(
    .iCLK(iCLK),			// 50MHz, System Clock
    .iRst_n(iRst_n),		// Reset
    .iFREQ(iFREQ),		// Frequence
    .iSPD_T(wSPD_T),
    .iDIR_T(wDIR_T),
    .iSPD_C(wSPD_C),
    .iDIR_C(wDIR_C),
    .iSpeedErrVal(wSpdErrVal),
	.iSmaller(wErr_S),
    .iDuty_C(wDuty),
    .oDuty(wAccelDuty)		// Pulse Width Modulation
);

DecelController #(
    .STREAM_SIZE(STREAM_SIZE),
    .DUTY_SIZE(DUTY_SIZE)
)(
	.iCLK(iCLK),		// 50MHz, System Clock
	.iRst_n(iRst_n),	// Reset
    .iFREQ(iFREQ),		// Frequence
    .iDuty_C(wDuty),
	.oDuty(wDeccelDuty),
);

ConstSpdController #(
    .STREAM_SIZE(STREAM_SIZE),
    .DUTY_SIZE(DUTY_SIZE)
)(
    .iCLK(iCLK),			// 50MHz, System Clock
    .iRst_n(iRst_n),		// Reset
    .iFREQ(iFREQ),		// Frequence
    // .iSPD,			// Speed of motor
    // .iFB,			// Feedback of motor
    .iDuty_C(wDuty),
    .oDuty(wConstDuty)		// Pulse Width Modulation
);

MUX #(
    .DUTY_SIZE(DUTY_SIZE),
    .SEL_SIZE(SEL_SIZE)
)(
    .iClk   (iCLK),
    .iRst_n (iRst_n),
	.iSel	(wSel),
	.iDuty_0    (wAccelDuty),
    .iDuty_1    (wDeccelDuty),
    .iDuty_2    (wConstDuty),
	// .iDIR0	(wDIR_SM),
	// .iDIR1	(wDIR_DC),
	// .oDIR	(oDIR),
	.oDuty	(wDuty)
);

SpeedErrorVal #(
	.SIZE(STREAM_SIZE)
)(
    .iClk(iCLK),
    .iRst_n(iRst_n),
	.iFB(iFB),
	.iCMD(iCMD),
    .iFREQ(iFREQ),
	.oSpeedErrVal(wSpdErrVal),
	.oSmaller(wErr_S)
);

Absolute #(
	.SIZE(STREAM_SIZE)
)TaregetVelocity(
	.iValue(iCMD),
	.oValue(wSPD_T),
	.oSign(wDIR_T)
);

Absolute #(
	.SIZE(STREAM_SIZE)
)CurrentVelocity(
	.iValue(iFB),
	.oValue(wSPD_C),
	.oSign(wDIR_C)
);

endmodule