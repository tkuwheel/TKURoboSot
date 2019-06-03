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
parameter STREAM_SIZE 	= 16,
parameter DUTY_SIZE 	= 7,
parameter SEL_SIZE		= 6

)(
//===========================================================================
// PORT declarations
//===========================================================================
input				iCLK,				// 50MHz, System Clock
input				iRst_n,				// Reset
input				iFREQ,
// input		[STREAM_SIZE-1:0]	iCMD,	// Speed of motor
// input		[STREAM_SIZE-1:0]	iFB,	// Feedback of motor Input
input		[STREAM_SIZE-1:0]	iErrVal,
input		[STREAM_SIZE-1:0]	iSPD_T,
input							iDIR_T,
input		[STREAM_SIZE-1:0]	iSPD_C,
input							iDIR_C,
output		[SEL_SIZE-1:0]		oSel,
output	reg			oDIR				// Direction of motor Output

);

//===========================================================================
// PARAMETER declarations
//===========================================================================
`include "param.h"
parameter	SIZE		=	SEL_SIZE;
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
// wire [STREAM_SIZE-1:0]	wSPD_T;
// wire [STREAM_SIZE-1:0]	wSPD_C;

// wire	wDIR_T;
// wire	wDIR_C;

reg	[SEL_SIZE-1:0]	state;
reg		rFREQ;
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
				if (iSPD_T > MIN_VELOCITY) 		// Is now speed greater than zero
					state	<=	START;			// let state is equal to START
				else
					state	<=	STOP;
			end
			START:	begin
				oDIR		<=	~iDIR_T;			// let Direction of motor Output is equal to old Direction
				state	<=	ACCEL;			// let state is equal to ACCEL
			end
			ACCEL:	begin	// Accelerate Speed of motor
				if(iSPD_T == 0)
					state <= DECEL;
				else if(iDIR_C != iDIR_T && iSPD_C != 0)
					state <= DECEL;
				else if(iSPD_T < MIN_VELOCITY && iSPD_C < MIN_VELOCITY) 
					state <= STOP;
				else if(iErrVal < Threshold_Diff_Vel)
					state <= CONST_SPD;
				else 
					state <= ACCEL;
				// state	<=	KEEP_SPD;			// let state is equal to KEEP_SPD
			end
			CONST_SPD:	begin// Keep Speed of motor
				if(iDIR_C != iDIR_T)
					state <= DECEL;
				else if(iSPD_T < MIN_VELOCITY && iSPD_C < MIN_VELOCITY) 
					state <= STOP;
				else if(iErrVal > Threshold_Diff_Vel)
					state <= ACCEL;
				else 
					state <= CONST_SPD;
				// oAccelMode	<=	2'b01;
				// oSel		<=	1'b1;
				// oSPD		<=	rSPD_O;			// let Speed of motor Output is equal to old speed
				// oDIR		<=	rDIR_O;			// let Direction of motor Output is equal to old direction
				// if (rSPD_N > rSPD_O) begin	// Is now speed greater than old speed
				// 	rSPD_O	<=	rSPD_N;		// let old speed is equal to now speed
				// 	state	<=	ACCEL;			// let state is equal to ACCEL
				// end
				// else if (rSPD_N < rSPD_O) begin	// Is now speed smaller than old speed
				// 	rSPD_O	<=	rSPD_N;				// let old speed is equal to now speed
				// 	state	<=	DECEL;					// let state is equal to DECEL
				// end
				// else if (rDIR_N != rDIR_O) begin // Is now direction not equal to old direction
				// 	state	<=	DIFF_DIR;				// let state is equal to DIFF_DIR
				// end
			end
			DECEL:	begin	// Decelerate Speed of motor
				if(iDIR_C == iDIR_T && iSPD_T != 0)
					state <= ACCEL;
				else if(iSPD_C < MIN_VELOCITY)
					state <= STOP;
				else 
					state <= DECEL;
				
				// oSel	<=	1'b0;
				// if (iFB <= rSPD_O) begin	// Is feedbak of motor equal to old speed
				// 	if (rSPD_O == 0) begin	// Is old direction equal to zero
				// 		state	<=	STOP;			// let state is equal to STOP
				// 	end
				// 	else begin
				// 		state	<=	KEEP_SPD;	// let state is equal to KEEP_SPD
				// 	end
				// end
			end
			// DIFF_DIR:	begin	// Chang Direction of motor
				
			// 	oSel	<=	1'b0;
			// 	if (iFB < Threshold_Diff) begin	// Is feedbak of motor smaller than Threshold_Diff
			// 		rDIR_O	<=	rDIR_N;				// let old direction is equal to now direction
			// 		rSPD_O	<=	rSPD_N;				// let old speed is equal to now speed
			// 		state	<=	ACCEL;					// let state is equal to ACCEL
			// 	end
			// end
			endcase
		end
		else 
			state <= state;
	end
end

endmodule
