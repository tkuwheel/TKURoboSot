module MotorStates #(
parameter SIZE = 8,
parameter DUTY_SIZE = 8
)(
input iCLK,
input iRst_n,
input [SIZE-1:0]iSPD_C,
input [DUTY_SIZE-1:0]iDuty,
output reg oMotorEnable,
output reg oMotorStop
);

wire [SIZE-1:0]  wDuty;
wire             sign;

`include "param.h"
always @(posedge iCLK) begin
    if(!iRst_n)begin
        oMotorEnable <= 0;
        oMotorStop  <= 0;
    end
    else begin
        if((iSPD_C < MIN_VELOCITY) & (iDuty < MIN_DUTY))begin
            oMotorEnable    <= 0;
            oMotorStop      <= 1;
        end
        else begin
            oMotorEnable    <= 1;
            oMotorStop      <= 0;
        end
    end
end 

// Absolute #(
//     .SIZE(SIZE)
// )(
//     .iValue (iDuty),
//     .oValue (wDuty),
//     .oSign  (sign)
// );

endmodule