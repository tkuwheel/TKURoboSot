module PWMReset(
input iClk,
input iRst_n,
input iDataValid,
input iPWM0,
input iPWM1,
input iPWM2,
output reg oPWM0,
output reg oPWM1,
output reg oPWM2
);
wire wClk10;
reg [7:0] counter;
reg reset;
reg rClk10;
reg rDataValid;
Clkdiv #(
	.EXPECTCLK	(10)
) Clk10Hz (
	.iClk		(iClk),	// 50Mhz clock 
	.iRst_n	(iRst_n),// Reset
  	// .oSampClk(GPIO_1_D[0]),  // multipleX expect clock, for SignalTap use
	.oClk		(wClk10)	// ExpectClk clock
);

always@(posedge iClk) begin
	if(!iRst_n)begin
		reset <= 0;
		counter <= 0;
		rDataValid <= 0;
	end
	else begin	
		
		if(~rDataValid & iDataValid)
			counter <= 0;
		else begin
			rClk10 <= wClk10;
			if(~rClk10 & wClk10 & ~reset)
				counter <= counter + 1;
			
			else
				counter <= counter;
				if(counter >= 100)
					reset <= 1;
				else
					reset <= 0;
			
		end
	end
end
always @(posedge iClk) begin
	if(!iRst_n) begin
		oPWM0 <= 0;
		oPWM1 <= 0;
		oPWM2 <= 0;
	end
	else begin
		if(reset) begin
			oPWM0 <= 0;
			oPWM1 <= 0;
			oPWM2 <= 0;
		end
		else begin
			oPWM0 <= iPWM0;
			oPWM1 <= iPWM1;
			oPWM2 <= iPWM2;
		end
	end
end
endmodule