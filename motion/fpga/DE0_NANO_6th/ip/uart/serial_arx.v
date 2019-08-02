/*
 *  RS-232 asynchronous RX module
 *  Copyright (C) 2010  Donna Polehn <dpolehn@verizon.net>
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

module serial_arx (
    input  clk,
    input  iRst_n,
    input  rxd,
    input  baud8tick, // Desired baud rate

    output reg [7:0] oRXd_data,
    output reg       rxd_data_ready // on clock pulse when rxd_data is valid

    // We also detect if a gap occurs in the received stream of
    // characters which can be useful if multiple characters are
    // sent in burst so that multiple characters can be treated as a "packet"
  );
  reg [7:0] rxd_data;
  reg [1:0] rxd_sync_inv;  // we invert rxd, so that the idle becomes "0", to prevent a phantom character to be received at startup
  always @(posedge clk) if(baud8tick) rxd_sync_inv <= {rxd_sync_inv[0], ~rxd};

  reg [1:0] rxd_cnt_inv;
  reg rxd_bit_inv;

  always @(posedge clk or negedge iRst_n) begin
      if(!iRst_n) begin
          rxd_cnt_inv <= 0;
          rxd_bit_inv <= 0;
      end
      else begin
          rxd_cnt_inv <= rxd_cnt_inv;
          rxd_bit_inv <= rxd_bit_inv;
          if(baud8tick) begin
              if( rxd_sync_inv[1] && rxd_cnt_inv!=2'b11) 
                  rxd_cnt_inv <= rxd_cnt_inv + 2'h1;
              else
                  if(~rxd_sync_inv[1] && rxd_cnt_inv!=2'b00) 
                      rxd_cnt_inv <= rxd_cnt_inv - 2'h1;
              if(rxd_cnt_inv==2'b00) 
                  rxd_bit_inv <= 1'b0;
              else
                  if(rxd_cnt_inv==2'b11) 
                      rxd_bit_inv <= 1'b1;
           end
      end
end

  reg [3:0] state;
  reg [3:0] bit_spacing;

  // "next_bit" controls when the data sampling occurs depending on how noisy the rxd is, different
  // values might work better with a clean connection, values from 8 to 11 work
  wire next_bit = (bit_spacing==4'd8);

  always @(posedge clk or negedge iRst_n) begin
      if(!iRst_n) begin
          bit_spacing <= 0;
      end
      else begin
          bit_spacing <= bit_spacing;
          if(state==0)
              bit_spacing <= 4'b0000;
          else
              if(baud8tick) 
                  bit_spacing <= {bit_spacing[2:0] + 4'b0001} | {bit_spacing[3], 3'b000};
      end
end
  always @(posedge clk or negedge iRst_n) begin
      if(!iRst_n) begin
          state <= 0;
      end
      else begin
        state <= state;
        if(baud8tick)
           case(state)
               4'b0000: if(rxd_bit_inv)state <= 4'b1000;  // start bit found?
               4'b1000: if(next_bit)  state <= 4'b1001;  // bit 0
               4'b1001: if(next_bit)  state <= 4'b1010;  // bit 1
               4'b1010: if(next_bit)  state <= 4'b1011;  // bit 2
               4'b1011: if(next_bit)  state <= 4'b1100;  // bit 3
               4'b1100: if(next_bit)  state <= 4'b1101;  // bit 4
               4'b1101: if(next_bit)  state <= 4'b1110;  // bit 5
               4'b1110: if(next_bit)  state <= 4'b1111;  // bit 6
               4'b1111: if(next_bit)  state <= 4'b0001;  // bit 7
               4'b0001: if(next_bit)  state <= 4'b0000;  // stop bit
               default:         state <= 4'b0000;
            endcase
       end
end

wire wFlag;
assign wFlag = (baud8tick && next_bit && state[3]);
  always @(posedge clk or negedge iRst_n) begin
      if(!iRst_n) begin
          rxd_data <= 0;
      end
      else begin
        rxd_data <= rxd_data;
        if(wFlag) 
          rxd_data <= {~rxd_bit_inv, rxd_data[7:1]};
        end
  end

  //reg rxd_data_error;
  always @(posedge clk or negedge iRst_n) begin
    if (!iRst_n) begin
        rxd_data_ready <= 0;
    end
    else begin
		rxd_data_ready <= (baud8tick && next_bit && state==4'b0001 && ~rxd_bit_inv);  // ready only if the stop bit is received
		if (rxd_data_ready) begin
			oRXd_data	<=	rxd_data;
		end
	end
		//rxd_data_error <= (baud8tick && next_bit && state==4'b0001 &&  rxd_bit_inv);  // error if the stop bit is not received
  end
endmodule
