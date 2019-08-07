module holdBall(iC,oL,oR,iCMD1,iCMD2,,iCMD3,iS,oLED);
inout iC,iS;
input [7:0]iCMD1,iCMD2,iCMD3;
output reg oL,oR,oLED/*,oL1,oR1*/;
reg [1:0]cont;
reg [32:0]con;
reg [6:0]spd1,spd2,spd3;
always@(posedge iC)begin

	spd1 <= iCMD1[6:0];
	spd2 <= iCMD2[6:0];
	spd3 <= iCMD3[6:0];

	
/*	if(iS == 0)begin
		oL = 1'b1;//停止
		oR = 1'b1;
	end else begin
		if(spd1 == 0 && spd2 == 0)begin//停止
			oL = 1'b1;//停止
			oR = 1'b1;
		end else begin
			if(iCMD1[7] == 0 && iCMD2[7] == 1)begin//向前
				if((spd2 + spd1) > 100)begin
					oL <= 1'b1;
					oR <= 1'b1;
				end else begin
					oL <= 1'b0;
					oR <= 1'b0;
				end
			end else begin 
				oL <= 1'b0;
				oR <= 1'b0;
			end
		end
	end

*/
if(iS == 0)begin
	oL = 1'b1;//停止
	oR = 1'b1;
	oLED = 1'b1;
end else begin
	oLED = 1'b0;
	if(spd1 == 0 && spd2 == 0)begin//停止
		oL = 1'b1;//停止
		oR = 1'b1;
	end else begin
		oL <= 1'b0;
		oR <= 1'b0;
	end
end

end
endmodule 