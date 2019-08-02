// --------------------------------------------------------------------
// Copyright (c) 2011 by TKU ICLAB. 
// --------------------------------------------------------------------
//
// Revision History :
// --------------------------------------------------------------------
//   Ver  :| Author            :| Mod. Date :| Changes Made:
//   V1.0 :| Shih-An Li        :| 15/03/2011:| 1. Generate PWM signal
//     _   _   _   _   _   _        _   _   _   _
//   _| |_| |_| |_| |_| |_| | ... _| |_| |_| |_| |_|
//    |<----   PWM cycle          ->| 
//  ->|   |<-pwm_clock
// 
//  PWM cycle = 127*pwm_clock*2 ( 2 -> positive and negtive cycle)
// --------------------------------------------------------------------

module pwmgen #(
parameter SPD_DIV = 1
)(
			iClk,		// 25Mhz clock
			iRst_n,	// reset, low active
			iDuty,	// Range is ??~??
			oPWM,
			oSampClk,
			oErrorValue
);

//===========================================================================
// PARAMETER declarations
//===========================================================================
//parameter PWMCYCLE=10000;  // PWM cycle = 10Khz
parameter PWMClk=1000*(128*SPD_DIV-1);
parameter DUTY_CYCLE = 128*SPD_DIV-1;
//===========================================================================
// PORT declarations
//===========================================================================
input       iClk;
input       iRst_n;
input [8:0] iDuty;
output      oPWM;
output      oSampClk;
output      oErrorValue;

//=============================================================================
// REG/WIRE declarations
//=============================================================================
reg	[8:0]	rPWMCnt;
wire				wClk;
wire				wTempPWM;

//=============================================================================
// Structural coding
//=============================================================================

//Clock Divsor
Clkdiv #(
	.EXPECTCLK	(PWMClk)
)(
    .iClk		(iClk),  // 50Mhz clock
    .iRst_n		(iRst_n),// reset
    .oSampClk	(oSampClk),
    .oClk		(wClk)
);

always @(posedge wClk or negedge iRst_n)
begin
    if(!iRst_n) begin
        rPWMCnt <= 0;
    end
    else begin
        if(rPWMCnt >= DUTY_CYCLE )	// rPWMCnt range is 0~127
            rPWMCnt <= 0;
        else
            rPWMCnt <= rPWMCnt + 8'd1;	// up count
    end
end

// if iDuty value is eqaul to 0, and the wTempPWM will output 0, and turn off PWM
// if iDuty value is not eqaul to 0, and if iDuty value great than rPWMCnt, the wTempPWM will output 1
assign wTempPWM = (iDuty == 8'b0) ? 1'b0:
                  (iDuty >= rPWMCnt ) ? 1'b1 : 1'b0;

// if iDuty value great than 100, the oErrorValue will output 1, and turn off PWM
assign oErrorValue = (iDuty > DUTY_CYCLE) ? 1'b1: 1'b0;
assign oPWM = wTempPWM & (~oErrorValue);

endmodule
