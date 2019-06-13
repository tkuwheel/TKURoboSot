// --------------------------------------------------------------------
// Copyright (c) 2012 by Intelligent Control Lab. of Tamkang University. 
// --------------------------------------------------------------------
//
//
// Major Functions: MUX
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
//   1.0  :| Chun-Jui Huang    :| 2019/05/31 :|  New Version
// --------------------------------------------------------------------
`default_nettype none
module MUX (
//===========================================================================
// PORT declarations
//===========================================================================
input   iClk,
input   iRst_n,
input	[5:0]  iSel,
input	[8:0] iDuty_0,
input	[8:0] iDuty_1,
input	[8:0] iDuty_2,
input	[8:0] iDuty_3,
output reg [8:0] oDuty
);
`include "param.h"
parameter	ACCEL		=	6'b000001;	// Accelerate
parameter	DECEL		=	6'b000010;	// Decelerate
parameter	CONST_SPD	=	6'b000100;	// Keep Speed
parameter	START		=	6'b001000;	// Start
parameter	STOP		=	6'b100000;	// Stop

always @(posedge iClk) begin
    if(!iRst_n)
        oDuty <= 0;
    else begin
        case(iSel)
        ACCEL:
            oDuty <= iDuty_0;
        DECEL:
            oDuty <= iDuty_1;
        CONST_SPD:
            oDuty <= iDuty_2;
        START:
            oDuty <= MIN_DUTY;
        STOP:
            oDuty <= MIN_DUTY;
        default:
            oDuty <= 0;
        endcase
    end
end

endmodule
