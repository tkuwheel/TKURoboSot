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
`default_nettype none
module StateMachine (
//===========================================================================
// PORT declarations
//===========================================================================
input				iCLK,				// 50MHz, System Clock
input		[7:0]	iSPD,				// Speed of motor
input		[1:0]	iDIR,				// Direction of motor Input
input		[7:0]	iFB,				// Feedback of motor Input
input				iRst_n,			// Reset
output	reg	[1:0]	oDIR,		// Direction of motor Output
output	reg			oSel,
output	reg	[1:0]	oAccelMode,	//	Mode of Acceleration
output	reg	[7:0]	oSPD		// Speed of motor Output
);

//===========================================================================
// PARAMETER declarations
//===========================================================================
parameter	SIZE		=	6;
parameter	ACCEL		=	6'b000001;	// Accelerate
parameter	DECEL		=	6'b000010;	// Decelerate
parameter	KEEP_SPD	=	6'b000100;	// Keep Speed
parameter	START		=	6'b001000;	// START
parameter	DIFF_DIR	=	6'b010000;
parameter	STOP		=	6'b100000;	// Stop

parameter	Threshold_Start	=	0;	// let Threshold of start = 50
parameter	Threshold_Diff	=	50;

//=======================================================
//  REG/WIRE declarations
//=======================================================
reg	[SIZE-1:0]	state = STOP;

reg		[7:0]	rSPD_O;	// old speed
reg		[1:0]	rDIR_O;	// old direction

reg		[7:0]	rSPD_N;	// now speed
reg		[1:0]	rDIR_N;	// now direction

//=======================================================
//  Structural coding
//=======================================================
always @(posedge iCLK) begin
	rSPD_N	<=	iSPD;
	rDIR_N	<=	iDIR;
	if (!iRst_n) begin
		rSPD_O	<=	0;
		rDIR_O	<=	0;
		rSPD_N	<=	0;
		rDIR_N	<=	0;
		state	<=	STOP;
	end
	else begin
		case(state)
		STOP:		// Stop Speed of motor
			begin
				oSel	<=	1'b1;
				oSPD	<=	0;
				oDIR	<=	2'b11;
				if (rSPD_N > 0) begin		// Is now speed greater than zero
					rDIR_O	<=	rDIR_N;
					state	<=	START;			// let state is equal to START
				end
			end
		START:
			begin
				oDIR		<=	rDIR_O;			// let Direction of motor Output is equal to old Direction
				oAccelMode	<=	2'b00;
				oSel		<=	1'b1;
				//if (iFB > Threshold_Start) begin
					rSPD_O	<=	rSPD_N;
					state	<=	ACCEL;			// let state is equal to ACCEL
				//end
			end
		ACCEL:		// Accelerate Speed of motor
			begin
				oAccelMode	<=	2'b01;
				oSel		<=	1'b1;
				state	<=	KEEP_SPD;			// let state is equal to KEEP_SPD
			end
		KEEP_SPD:	// Keep Speed of motor
			begin
				oAccelMode	<=	2'b01;
				oSel		<=	1'b1;
				oSPD		<=	rSPD_O;			// let Speed of motor Output is equal to old speed
				oDIR		<=	rDIR_O;			// let Direction of motor Output is equal to old direction
				if (rSPD_N > rSPD_O) begin	// Is now speed greater than old speed
					rSPD_O	<=	rSPD_N;		// let old speed is equal to now speed
					state	<=	ACCEL;			// let state is equal to ACCEL
				end
				else if (rSPD_N < rSPD_O) begin	// Is now speed smaller than old speed
					rSPD_O	<=	rSPD_N;				// let old speed is equal to now speed
					state	<=	DECEL;					// let state is equal to DECEL
				end
				else if (rDIR_N != rDIR_O) begin // Is now direction not equal to old direction
					state	<=	DIFF_DIR;				// let state is equal to DIFF_DIR
				end
			end
		DECEL:		// Decelerate Speed of motor
			begin
				oSel	<=	1'b0;
				if (iFB <= rSPD_O) begin	// Is feedbak of motor equal to old speed
					if (rSPD_O == 0) begin	// Is old direction equal to zero
						state	<=	STOP;			// let state is equal to STOP
					end
					else begin
						state	<=	KEEP_SPD;	// let state is equal to KEEP_SPD
					end
				end
			end
		DIFF_DIR:	// Chang Direction of motor
			begin
				oSel	<=	1'b0;
				if (iFB < Threshold_Diff) begin	// Is feedbak of motor smaller than Threshold_Diff
					rDIR_O	<=	rDIR_N;				// let old direction is equal to now direction
					rSPD_O	<=	rSPD_N;				// let old speed is equal to now speed
					state	<=	ACCEL;					// let state is equal to ACCEL
				end
			end
		endcase
	end
end

endmodule
