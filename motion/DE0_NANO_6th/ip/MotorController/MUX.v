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

module MUX (
//===========================================================================
// PORT declarations
//===========================================================================
input			iSel,
input	[7:0]	iPWM0,
//input 	[7:0]	iPWM1,
input	[1:0]	iDIR0,
input	[1:0]	iDIR1,	
output	[7:0]	oPWM,
output	[1:0]	oDIR
);

assign oPWM = iSel ? iPWM0 : 8'b00000000;
assign oDIR = iSel ? iDIR0 : iDIR1;

endmodule
