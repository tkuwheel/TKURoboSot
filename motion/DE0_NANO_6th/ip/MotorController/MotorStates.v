module MotorStates (
input iCLK,
input iRst_n,
// input [MOTOR_SIZE-1:0]iSPD_C,
input 		[8:0]		iDuty,
input 		[5:0] 	iSel,
input						iFREQ,
output reg 				oMotorEnable,
output reg 				oMotorStop
);

`include "param.h"
reg [5:0] states;
reg rFREQ;
always @(posedge iCLK) begin
	if(!iRst_n)begin
		oMotorEnable <= 0;
		oMotorStop  <= 0;
		states <= IDLE;
	end
	else begin
		rFREQ <= iFREQ;
		if(~rFREQ & iFREQ)begin
			states <= iSel;
			case(states)
				IDLE: begin
					oMotorEnable    <= 0;
					oMotorStop      <= 0;
					end
				STOP: begin
					oMotorEnable    <= 1;
					oMotorStop      <= 1;
					end
				START: begin
					oMotorEnable    <= 1;
					oMotorStop      <= 0;
					end
				ACCEL: begin
					oMotorEnable    <= 1;
					oMotorStop      <= 0;
					end
				DECEL: begin
					oMotorEnable    <= 1;
					oMotorStop      <= 0;
					end
				CONST: begin
					oMotorEnable    <= 1;
					oMotorStop      <= 0;
					end
				default: begin
					states <= IDLE;
					end
			endcase
		end
		else begin
			oMotorEnable    <= oMotorEnable;
			oMotorStop      <= oMotorStop;
		end

    end
end 

endmodule