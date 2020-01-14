module holdpwm(clk,reset,iSw,oPwm);

input clk;//1kHz
input reset;
input [6:0]iSw;//determine the output power(0~128)
output reg oPwm;
reg [7:0]counter;
	always@(posedge clk) begin
		if(!reset)begin
			counter <=0;
			oPwm<=0;
		end
		else begin	
			counter <= counter+1;	
				if(iSw>counter)begin
					oPwm <=1;
				end				
				else begin
					oPwm<=0;				
				end
				if(counter>=100) begin	
					counter<=0;		
				end
		end	  
		end
endmodule