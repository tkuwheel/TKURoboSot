/*
 *  Phase accumulator clock generator:
 *   Output Frequency Fo = Fc * N / 2^bits
 *   Output Jitter = 1/Fc
 *
 *  Copyright (c) 2009,2010  Zeus Gomez Marmolejo <zeus@aluzina.org>
 *
 *  This file is part of the Zet processor. This processor is free
 *  hardware; you can redistribute it and/or modify it under the terms of
 *  the GNU General Public License as published by the Free Software
 *  Foundation; either version 3, or (at your option) any later version.
 *
 *  Zet is distrubuted in the hope that it will be useful, but WITHOUT
 *  ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
 *  or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public
 *  License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with Zet; see the file COPYING. If not, see
 *  <http://www.gnu.org/licenses/>.
 */
/*
Example:
25M*100091/(2^21)=1.193178Mhz


  clk_gen #(
    .res   (21),
    .phase (21'd100091)
    ) timerclk (
    .iClk (vga_clk),    // 25 MHz
    .iRst_n (rst),
    .oClk (timer_clk)   // 1.193178 MHz (required 1.193182 MHz)
  );

*/


module clk_gen #(
	parameter res    = 20,    // bits - bit resolution
	parameter phase  =  1     // N - phase value for the counter
	)(
	input      iClk, // Fc - input frequency
    input      iRst_n,
    output     oClk  // Fo - output frequency
);

  // Registers and nets
  reg [res-1:0] rCnt;

  // Continuous assignments
  assign oClk = rCnt[res-1];

  // Behaviour
  always @(posedge iClk or negedge iRst_n)
    if(!iRst_n)
        rCnt <= {res{1'b0}};
    else
        rCnt = (rCnt + phase);


endmodule
