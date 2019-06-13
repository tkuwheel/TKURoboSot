`default_nettype none

module SpeedErrorVal #(
parameter SIZE = 1
)(
input iClk,
input iRst_n,
input iFREQ,
input [15 : 0] iFB,
input [15 : 0] iCMD,

output [15 : 0] oSpeedErrVal,
output oSmaller
);

reg [SIZE-1 : 0] rSpeedErrVal;
reg rFREQ;
always@(posedge iClk)begin
    if(! iRst_n) begin
        rSpeedErrVal <= 0;
    end
    else begin
        rFREQ <= iFREQ;
        if(~rFREQ & iFREQ )begin
            rSpeedErrVal <= iCMD - iFB;
        end
        else begin
            rSpeedErrVal <= rSpeedErrVal;
        end
    end
end

Absolute #(
	.SIZE(SIZE)
)SpdErr(
	.iValue(rSpeedErrVal),
	.oValue(oSpeedErrVal),
	.oSign(oSmaller)
);
endmodule