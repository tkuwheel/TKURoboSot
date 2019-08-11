 //
//
// Major Functions: holdball control
//
// --------------------------------------------------------------------
//
// Revision History :
// --------------------------------------------------------------------
//   Ver  :| Author            :| Mod. Date  :|  Changes Made:
//   1.0  :| Yin-Chen,Li       :| 2019/08/06 :|  Shoot7
// --------------------------------------------------------------------
module holdball7(holdPower,holdPower1,reset,oPower,oPower1,clk);

input [6:0]holdPower;
input [6:0]holdPower1;
input reset;
input clk;
output oPower;
output oPower1;

wire pwmclk;


Clkdiv #(
	.EXPECTCLK (500*100)
)u2(

.iClk(clk), 
.iRst_n(reset),
.oClk(pwmclk)
	
);

holdpwm (
.clk(pwmclk),
.reset(reset),
.iSw(holdPower),
.oPwm(oPower)
);

holdpwm u1 (
.clk(pwmclk),
.reset(reset),
.iSw(holdPower1),
.oPwm(oPower1)
);


endmodule
