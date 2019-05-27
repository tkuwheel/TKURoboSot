// --------------------------------------------------------------------
// Copyright (c) 2012 by Intelligent Control Lab. of Tamkang University. 
// --------------------------------------------------------------------
//
//
// Major Functions: Normoalize speed and feedback
//
// --------------------------------------------------------------------
//
// Revision History :
// --------------------------------------------------------------------
//   Ver  :| Author            :| Mod. Date  :|  Changes Made:
//   0.1  :| Chih-en Wu        :| 2012/08/01 :|  Initial Version
// --------------------------------------------------------------------
// --------------------------------------------------------------------
//   Ver  :| Author            :| Mod. Date  :|  Changes Made:
//   1.0  :| Chun-Jui Huang   :| 201*/05/27 :|  Transfer feedback value into rpm
// --------------------------------------------------------------------

module Normalize #(
//===========================================================================
// PARAMETER declarations
//===========================================================================
parameter	SPD_DIV	    = 1,    // Divisor of Speed
parameter   CMD_SIZE    = 1,    // bits
parameter   FB_SIZE	    = 1 	// bits
)(
//===========================================================================
// PORT declarations
//===========================================================================
input	[FB_SIZE-1:0]	iFB,	// Feedback Input
output	[FB_SIZE-1:0]	oFB	// Feedback Output
);
`include "param.h"
//==================================================================
//  REG/WIRE declarations
//==================================================================
wire   [31:0]buffer;
wire   [31:0]compute;
//=============================================================================
// Structural coding
//=============================================================================
assign  buffer  = (iFB[FB_SIZE-1] == 1)? ~iFB+1 : iFB;
assign  oFB = (iFB[FB_SIZE-1] == 1)? ~(buffer * FEEDBACK_FREQUENCY * 60 / 2000)+1: buffer * FEEDBACK_FREQUENCY * 60 / 2000;
// iFB[CMD_SIZE-1 :0] *  FEEDBACK_FREQUENCY / 2000 * 60 ;
endmodule
