module MotorStates (
input iCLK,
input iRst_n,
// input [MOTOR_SIZE-1:0]iSPD_C,
input 		[8:0]		iDuty,
input 		[5:0] 	iSel,
output reg 				oMotorEnable,
output reg 				oMotorStop
);

// parameter	SIZE		=	SEL_SIZE;
parameter	ACCEL		=	6'b000001;	// Accelerate
parameter	DECEL		=	6'b000010;	// Decelerate
parameter	CONST_SPD	=	6'b000100;	// Keep Speed
parameter	START		=	6'b001000;	// Start
// parameter	DIFF_DIR	=	6'b010000;	// Change direction
parameter	STOP		=	6'b100000;	// Stop

reg [5:0]  state;

always @(posedge iCLK) begin
	if(!iRst_n)begin
		oMotorEnable <= 0;
		oMotorStop  <= 0;
		state <= STOP;
	end
	else begin

        state <= iSel;
        case(state)
        STOP: begin
            oMotorEnable    <= 1;
            oMotorStop      <= 1;
        end
        START: begin
            oMotorEnable    <= 0;
            oMotorStop      <= 0;
        end
        ACCEL: begin
            oMotorEnable    <= 1;
            oMotorStop      <= 0;
        end
        DECEL: begin
            oMotorEnable    <= 1;
            oMotorStop      <= 1;
        end
        CONST_SPD: begin
            oMotorEnable    <= 1;
            oMotorStop      <= 0;
        end
        default:
            state <= STOP;
        endcase

    end
end 

endmodule