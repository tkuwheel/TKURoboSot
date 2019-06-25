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
input   iFREQ,
input	[5:0]  iSel,
input	[8:0] iDuty_0,
input	[8:0] iDuty_1,
input	[8:0] iDuty_2,
input	[8:0] iDuty_3,
output reg [8:0] oDuty
);

`include "param.h"
reg rFREQ;
always @(posedge iClk) begin
   if(!iRst_n)
        oDuty <= 0;
   else begin
		rFREQ <= iFREQ;
		if(~rFREQ & iFREQ)begin
			case(iSel)
			IDLE: begin
				oDuty <= 0;
				end
		   ACCEL: begin
				if(iDuty_0 > MAX_DUTY)
					oDuty <= MAX_DUTY;
				else if(iDuty_0 < MIN_DUTY)
					oDuty <= MIN_DUTY;
				else
					oDuty <= iDuty_0;
				end
		   DECEL: begin
				if(iDuty_1 > MAX_DUTY)
					oDuty <= MAX_DUTY;
				else if(iDuty_1 < MIN_DUTY)
					oDuty <= MIN_DUTY;
				else
					oDuty <= iDuty_1;
				end
		   CONST: begin
				if(iDuty_2 > MAX_DUTY)
					oDuty <= MAX_DUTY;
				else if(iDuty_2 < MIN_DUTY)
					oDuty <= MIN_DUTY;
				else
					oDuty <= iDuty_2;
				end
		   START: begin
				oDuty <= MIN_DUTY;
				end
		   STOP: begin
				oDuty <= MIN_DUTY;
				end
		   default:
            oDuty <= 0;
         endcase
		end
		else begin
			oDuty <= oDuty;
		end
   end
end

endmodule
