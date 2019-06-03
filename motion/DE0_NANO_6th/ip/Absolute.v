module Absolute #(
parameter SIZE = 8
)(
input 	[SIZE-1:0]	iValue,
output	[SIZE-1:0]	oValue,
output				oSign
);


assign oValue = (iValue[SIZE-1] == 1)? (~iValue+1):iValue;
assign oSign = iValue[SIZE-1];

endmodule