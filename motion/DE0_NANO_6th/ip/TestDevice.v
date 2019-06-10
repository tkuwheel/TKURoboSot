


module TestDevice (
input				iCLK,
input				iRst_n,
input		[7:0]	iFB,
input				iFB_FREQ,
input				ienable,
output	reg[7:0]	oCMD_Motor1_Test,
output	reg[7:0]	oCMD_Motor2_Test,
output	reg[7:0]	oAX_12_Test,
output	reg			oRx_done,
output	reg			oBrush_Test,
output				oTest_Done	
);

reg 	rFREQ;
reg		ren;
reg		rdone = 0;
reg 	[31:0]	fbcnt = 0;
reg				done = 1;
reg		[3:0]	cnt1 = 0;
reg		[3:0]	cnt2 = 0;
assign	oTest_Done = done;

always @(posedge iCLK) 
begin
if ( (ren & ~ienable) & ~rdone) begin
	rdone <= 1;
end

if (!iRst_n) begin
	fbcnt <= 0;
	cnt1 <= 0;
	cnt2 <= 0;
	rdone <= 0;
end

else begin
	if (rdone == 1) begin
		if (fbcnt < 38400) begin
			if (cnt1 < 10) begin
				oRx_done <= 1;
			end
			if ( ~rFREQ & iFB_FREQ ) begin
				fbcnt <= fbcnt + iFB;
				oRx_done <= 0;
			end
			oCMD_Motor1_Test <= 8'd40;
			oCMD_Motor2_Test <= 8'd40;
			oAX_12_Test <= 8'b00100000;    //0x20	32        
			oBrush_Test <= 1;
			done <= 0;
		end
		else begin
			oCMD_Motor1_Test <= 0;
			oCMD_Motor2_Test <= 0;
			oAX_12_Test <=	8'b00110011;          //0x20	51
			oBrush_Test <= 0;
//			done <= 1;
			cnt1 <= 0;
			cnt2 <= 0;
		end
	end
	
	if (fbcnt >= 38400) begin
		rdone <= 0;
		fbcnt <= 0;
	end
	
end
rFREQ	<=	iFB_FREQ;
ren <= ienable;

end
endmodule
