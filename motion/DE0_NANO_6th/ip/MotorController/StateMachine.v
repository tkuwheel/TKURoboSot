// --------------------------------------------------------------------
// Copyright (c) 2012 by Intelligent Control Lab. of Tamkang University. 
// --------------------------------------------------------------------
//
//
// Major Functions: Motor Controller State Machine
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
//   1.0  :| Chun-Jui Huang    :| 2019/05/29 :|  New Version
// --------------------------------------------------------------------
`default_nettype none
module StateMachine #(
parameter MOTOR_SIZE 	= 16,
parameter DUTY_SIZE 	= 7
// parameter SEL_SIZE		= 6

)(
//===========================================================================
// PORT declarations
//===========================================================================
input				iCLK,				// 50MHz, System Clock
input				iRst_n,				// Reset
input				iFREQ,
input	[15:0]	iCMD,
input	[15:0]	iFB,
output[5:0]		oSel,
output	reg	oDIR				// Direction of motor Output
);

//===========================================================================
// PARAMETER declarations
//===========================================================================
`include "param.h"

reg [5:0]	states;

//=======================================================
//  REG/WIRE declarations
//=======================================================
wire [15:0]	wSPD_Tar;
wire 			wDIR_Tar;
wire [15:0]	wSPD_Curr;
wire 			wDIR_Curr;
wire [15:0]	wSpdErrVal;
wire        wErr_S;
reg			rFREQ;
reg			rDiff_DIR;
reg			rSPD_Valid;
reg			rSPD_Curr_Valid;
//=======================================================
//  Structural coding
//=======================================================
assign oSel = states;

always@(posedge iCLK)begin
	if(!iRst_n)begin
		rDiff_DIR <= 0;
		rSPD_Valid = 0;
	end
	else begin
		rDiff_DIR <= wDIR_Tar ^ wDIR_Curr;
		if(wSPD_Tar > MIN_DUTY)
			rSPD_Valid <= 1;
		else 
			rSPD_Valid <= 0;
		if(wSPD_Curr > MIN_VELOCITY )
			rSPD_Curr_Valid <= 1;
		else
			rSPD_Curr_Valid <= 0;
	end

end

always @(posedge iCLK) begin
	if (!iRst_n) begin
		states	<=	IDLE;
		oDIR	<= 0;
	end
	else begin
		rFREQ <= iFREQ;
		if(~rFREQ & iFREQ)begin
			case(states)
			IDLE:begin
				oDIR	<=	oDIR;
				if (rSPD_Valid) 		// Is now speed greater than zero
					states	<=	START;			// let state is equal to START
				else
					states	<=	IDLE;
				end
			STOP:	begin	// Stop Speed of motor
				oDIR	<=	oDIR;
				if(rSPD_Valid) 		// Is now speed greater than zero
					states	<=	IDLE;			// let state is equal to START
				else
					states	<=	STOP;
				end
			START:	begin
				oDIR		<=	~wDIR_Tar;			// let Direction of motor Output is equal to old Direction
				if(!rSPD_Valid)
					states	<=	STOP;
				else 
					states	<=	ACCEL;
				end
			ACCEL:	begin	// Accelerate Speed of motor
				oDIR	<=	oDIR;
				if(!rSPD_Curr_Valid)
					states	<=	IDLE;
				else if(!rSPD_Valid)
					states <= DECEL;
				else if(rDiff_DIR && rSPD_Curr_Valid)
					states <= DECEL;
				/*else if(wSpdErrVal < Threshold_Diff_Vel || wSPD_Curr >= MAX_VELOCITY)
					oSel <= CONST_SPD;*/
				else 
					states <= ACCEL;
				end
			CONST:	begin// Keep Speed of motor
				end
			DECEL:	begin	// Decelerate Speed of motor
				oDIR	<=	oDIR;
				if(!rSPD_Curr_Valid)
					states <= STOP;
				else 
					states <= DECEL;
				end
			default: begin
			
				states <= IDLE;
				
				end
			endcase
		end
		else 
			states <= states;
	end

end

Absolute #(
	.SIZE(MOTOR_SIZE)
)TaregetVelocity(
	.iValue(iCMD),
	.oValue(wSPD_Tar),
	.oSign(wDIR_Tar)
);

Absolute #(
	.SIZE(MOTOR_SIZE)
)CurrentVelocity(
	.iValue(iFB),
	.oValue(wSPD_Curr),
	.oSign(wDIR_Curr)
);

/*
SpeedErrorVal #(
	.SIZE(MOTOR_SIZE)
)(
	.iClk(iCLK),
	.iRst_n(iRst_n),
	.iFB(iFB),
	.iCMD(iCMD),
	.iFREQ(iFREQ),
	.oSpeedErrVal(wSpdErrVal),
	.oSmaller(wErr_S)
);
*/
endmodule
