module oshootpwm(clk,reset,oPwm);

input clk;//1kHz
input reset;
output reg oPwm;
reg [7:0]counter;


parameter iPwm=10;//determine the output power

	always@(posedge clk) begin
		if(!reset)begin
			counter <=0;
			oPwm<=0;
		end
		else begin
			counter <= counter+1;
				if(iPwm>counter)begin
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
