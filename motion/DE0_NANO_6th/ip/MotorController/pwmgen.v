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
//  PWM cycle = 100*pwm_clock*2 ( 2 -> positive and negtive cycle)
// --------------------------------------------------------------------

module pwmgen #(
parameter SPD_DIV = 1
)(
			iClk,		// 25Mhz clock
			iRst_n,	// reset, low active
			iDuty,	// Range is 0~100
			oPWM,
			oSampClk,
			oErrorValue
);

//===========================================================================
// PARAMETER declarations
//===========================================================================
//parameter PWMCYCLE=10000;  // PWM cycle = 10Khz
//parameter PWMClk=10000*(127*SPD_DIV)*2;  // PWM clock = PWM_cycle * (100*2)
parameter PWMClk=1000*(127*SPD_DIV);

//===========================================================================
// PORT declarations
//===========================================================================
input       iClk;
input       iRst_n;
input [7:0] iDuty;
output      oPWM;
output      oSampClk;
output      oErrorValue;

//=============================================================================
// REG/WIRE declarations
//=============================================================================
reg	[7:0]	rPWMCnt;
wire				wClk;
wire				wTempPWM;

//=============================================================================
// Structural coding
//=============================================================================

//Clock Divsor
Clkdiv #(
	.EXCEPTCLK	(PWMClk)
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
        if(rPWMCnt >= (127*SPD_DIV) )	// rPWMCnt range is 0~127
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
assign oErrorValue = (iDuty > (127*SPD_DIV)) ? 1'b1: 1'b0;
assign oPWM = wTempPWM & (~oErrorValue);

endmodule
