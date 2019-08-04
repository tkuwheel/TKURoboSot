module holdball7(holdPower,holdPower1,reset,oPower,oPower1,clk);

input [6:0]holdPower;
input [6:0]holdPower1;
input reset;
input clk;
output oPower;
output oPower1;

wire pwmclk;


Clkdiv #(
	.EXPECTCLK (1000*100)
)u2(

.iClk(clk), 
.iRst_n(reset),
.oClk(pwmclk)
	
);

ishootpwm (
.clk(pwmclk),
.reset(reset),
.iSw(holdPower),
.oPwm(oPower)
);

ishootpwm u1 (
.clk(pwmclk),
.reset(reset),
.iSw(holdPower1),
.oPwm(oPower1)
);


endmodule
