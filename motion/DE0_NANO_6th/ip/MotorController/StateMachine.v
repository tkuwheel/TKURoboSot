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
input	[MOTOR_SIZE-1:0]	iCMD,
input	[MOTOR_SIZE-1:0]	iFB,
output		[5:0]		oSel,
output	reg			oDIR				// Direction of motor Output

);

//===========================================================================
// PARAMETER declarations
//===========================================================================
`include "param.h"
parameter	SIZE		=	6;
parameter	ACCEL		=	6'b000001;	// Accelerate
parameter	DECEL		=	6'b000010;	// Decelerate
parameter	CONST_SPD	=	6'b000100;	// Keep Speed
parameter	START		=	6'b001000;	// Start
// parameter	DIFF_DIR	=	6'b010000;	// Change direction
parameter	STOP		=	6'b100000;	// Stop

// parameter	Threshold_Start	=	0;	// let Threshold of start = 50
// parameter	Threshold_Stop	=	16'hF;	// let Threshold of start = 50
// parameter	Threshold_Diff	=	16'hFF;

//=======================================================
//  REG/WIRE declarations
//=======================================================
wire [MOTOR_SIZE-1:0]wSPD_T;
wire [MOTOR_SIZE-1:0]wSPD_C;
wire 						wDIR_T;
wire 						wDIR_C;
wire [MOTOR_SIZE-1:0]wSpdErrVal;
wire                 wErr_S;
reg [SIZE-1:0]			state;
reg						rFREQ;
// reg		[7:0]	rSPD_O;	// old speed
// reg		[1:0]	rDIR_O;	// old direction

// reg		[7:0]	rSPD_N;	// now speed
// reg		[1:0]	rDIR_N;	// now direction

//=======================================================
//  Structural coding
//=======================================================
assign 	oSel	=	state;


always @(posedge iCLK) begin
	if (!iRst_n) begin
		state	<=	STOP;
	end
	else begin
		rFREQ <= iFREQ;
		if(~rFREQ & iFREQ) begin
			case(state)
			STOP:	begin	// Stop Speed of motor
				oDIR	<=	0;
				if (wSPD_T > MIN_VELOCITY) 		// Is now speed greater than zero
					state	<=	START;			// let state is equal to START
				else
					state	<=	STOP;
			end
			START:	begin
				oDIR		<=	~wDIR_T;			// let Direction of motor Output is equal to old Direction
				state	<=	ACCEL;			// let state is equal to ACCEL
			end
			ACCEL:	begin	// Accelerate Speed of motor
				if(wSPD_T == 0)
					state <= DECEL;
				else if(wDIR_C != wDIR_T && wSPD_C != 0)
					state <= DECEL;
				else if(wSPD_T < MIN_VELOCITY && wSPD_C < MIN_VELOCITY) 
					state <= STOP;
				else if(wSpdErrVal < Threshold_Diff_Vel || wSPD_C >= MAX_VELOCITY)
					state <= CONST_SPD;
				else 
					state <= ACCEL;
			end
			CONST_SPD:	begin// Keep Speed of motor
				if(wDIR_C != wDIR_T)
					state <= DECEL;
				else if(wSPD_T < MIN_VELOCITY && wSPD_C < MIN_VELOCITY) 
					state <= STOP;
				else if(wSpdErrVal > Threshold_Diff_Vel)
					state <= ACCEL;
				else 
					state <= CONST_SPD;

			end
			DECEL:	begin	// Decelerate Speed of motor
				if(wDIR_C == wDIR_T && wSPD_T != 0)
					state <= ACCEL;
				else if(wSPD_C < MIN_VELOCITY)
					state <= STOP;
				else 
					state <= DECEL;
				
				
			end
			default: begin
				state <= STOP;
			end
			endcase
		end
		else 
			state <= state;
	end
end

Absolute #(
	.SIZE(MOTOR_SIZE)
)TaregetVelocity(
	.iValue(iCMD),
	.oValue(wSPD_T),
	.oSign(wDIR_T)
);

Absolute #(
	.SIZE(MOTOR_SIZE)
)CurrentVelocity(
	.iValue(iFB),
	.oValue(wSPD_C),
	.oSign(wDIR_C)
);
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
endmodule
