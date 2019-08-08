 //
//
// Major Functions: Shooting power control
//
// --------------------------------------------------------------------
//
// Revision History :
// --------------------------------------------------------------------
//   Ver  :| Author            :| Mod. Date  :|  Changes Made:
//   1.0  :| Yin-Chen,Li       :| 2019/08/06 :|  Shoot7
// --------------------------------------------------------------------  
module shoot7(clk,iSw,reset,dled,control,shoot);


input clk;//50MHz
input reset;
input [6:0]iSw;
input control;//default case

output reg dled;//direction light
output reg shoot;//shooting power
reg[31:0] counter;
reg[15:0] state;
reg[15:0]rPower;//store iSw signal
reg en;
reg resetPower;//set up a flag to close rPower signal 
reg [15:0]rStore;

wire pwmclk;
wire pwm;
wire pwm1;
		
parameter INIT=4'b1101;
parameter STANDBY=4'b1100;
parameter PRESHOOT=4'b1110; 
parameter S1=4'b0001;
parameter S2=4'b0010;
parameter S3=4'b0011;  
parameter S4=4'b0100;
parameter S5=4'b0101;
parameter S6=4'b0110;
parameter S7=4'b0111;
parameter S8=4'b1000;
parameter S9=4'b1001;

Clkdiv #(
	.EXPECTCLK (1000*100)//give pwm program 1kHz
)u1(

.iClk(clk), 
.iRst_n(reset),
.oClk(pwmclk)
	
);

ishootpwm (
.clk(pwmclk),
.reset(reset),
.iSw(rPower),
.oPwm(pwm)
);
oshootpwm (
.clk(pwmclk),
.reset(reset),
.oPwm(pwm1)
);
			
always@(posedge clk) begin
	
	if(!reset)begin
		counter <= 0;
		en<=0;
		resetPower<=0;
		state <=STANDBY;
	end
	else begin
		case(state)
		STANDBY:begin//wait for four seconds to charge the battery(00)
			shoot<=0;
			en<=0;
			resetPower<=0;
		if(counter>=200000000)begin
			state<=INIT;
			counter <= 0;
		end
		else begin
			dled<=1'b0;
			counter<=counter+1;
			state <= STANDBY;
		end
		end
		INIT:begin//(00)
			shoot<=0;
			en<=1;
			counter <= 0;
			dled<=1'b0;
			if(control)begin//a determine condition whether go to the case 6(generate a power to withdraw the motor )
				state<=S7;
			end
			else begin	
			if(rPower > 0)begin//a determine condition whether go to the case 1 to do the shooting case
				state<=PRESHOOT;
			end
			else begin
				state<=state;
			end
			end
		end
		PRESHOOT: begin
			en<=0;
			resetPower<=0;
			shoot<=0;
			if(counter>=5000000)begin//wait for 0.1 second(10)
				state <=S1;
				counter <= 0;
			end
			else begin
				dled<=1'b1;
				state <= PRESHOOT;
				counter<=counter+1;
			end			
		end	
		S1: begin
			en<=0;
			resetPower<=1;//release the rStore signal
			if(counter>=5000000)begin//output a pwm value for 0.1 second(11)
				state <=S2;
				counter <= 0;
				shoot<=0;
			end
			else begin
				dled<=1'b1;
				shoot<=pwm;
				state <= S1;
				counter<=counter+1;
			end			
		end	

		S2: begin
			  en<=0;
			  shoot<=0;
			  resetPower<=0;
			if(counter>=25000000)begin//wait for 0.5 second(10)
				state <=S3;
				counter <= 0;
			end
			else begin
				dled<=1'b1;
				state <= S2;
				counter<=counter+1;
			end			
		end
		S3: begin
				en<=0;
				shoot <= 0;
				resetPower<=0;
			if(counter>=25000000)begin//wait for 0.5 second,and change the direction signal(00)
				state <=S4;
				counter <= 0;
			end
			else begin
				dled<=1'b0;
				state <= S3;
				counter<=counter+1;
			end			
		end
		S4: begin
				en<=0;
				resetPower<=0;
			if(counter>=15000000)begin//output a pwm value for 0.3 second(01)
				state <=S5;
				counter <= 0;
				shoot <= 0;
			end
			else begin
				dled<=1'b0;
				state <= S4;
				shoot<=pwm1;
				counter<=counter+1;
			end			
		end
		S5: begin
			en<=0;
			shoot<=0;
			resetPower<=0;
			if(counter>=5000000)begin//wait for a 0.1 second,and change the direction(00)
				state <=S6;
				counter <= 0;
			end
			else begin
				counter<=counter+1;
				dled<=1'b0;
				state <= S5;
			end			
		end
		S6: begin
			en<=0;
			shoot<=0;
			resetPower<=0;
			if(counter>=50000000)begin//wait for a second,back to the INIT(10)
				state <=INIT;
				counter <= 0;
			end
			else begin
				counter<=counter+1;
				dled<=1'b1;
				state <= S6;
			end			
		end
		
		S7: begin
				en<=0;
				resetPower<=0;
			if(counter>=15000000)begin//output a pwm value for 0.3 second(01)
				state <=S8;
				counter <= 0;
				shoot <= 0;
			end
			else begin
				dled<=1'b0;
				state <= S7;
				shoot<=pwm1;
				counter<=counter+1;
			end			
		end
		S8: begin
				en<=0;
				resetPower<=0;
				shoot <= 0;
			if(counter>=5000000)begin//wait for 0.1s(00)
				state <=S9;
				counter <= 0;
			end
			else begin
				dled<=1'b0;
				state <= S8;
				counter<=counter+1;
			end			
		end
		S9: begin
			en<=0;
			shoot<=0;
			resetPower<=0;
			if(counter>=50000000)begin//wait for 1s(10)
				state <=INIT;
				counter <= 0;
			end
			else begin
				counter<=counter+1;
				dled<=1'b1;
				state <= S9;
			end			
		end


		default:begin
				shoot<=0;
				dled<=0;
				en<=0;
				resetPower<=0;
		end
		
		endcase
	
	end

end


always@(posedge clk)begin//design a latch to store iSw signal

	if(!reset)begin
		rPower<=0;
		rStore<=0;
	end
	else begin
		if(en)begin
			rPower<=iSw[6:0];
			rStore<=iSw[6:0];
		end
		else begin	
		if(resetPower)begin
			rPower<=rStore;
		end
		else begin
			rPower<=0;
			rStore<=rStore;
		end
		end
	end	
end

endmodule




