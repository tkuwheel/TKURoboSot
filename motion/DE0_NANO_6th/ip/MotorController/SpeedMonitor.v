module SpeedMonitor #(
parameter CMD_SIZE  = 2, //bytes -> bits
parameter FB_SIZE   = 4	//bytes -> bits)
)(
input   iCLK,
input   iRst_n,
input   [31:0]iFB,
input   [15:0]iNew_FB,
input   iPWM,
input   [15:0]iCMD,
output  reg [31:0]oFB,
output  reg [15:0]oNew_FB,
output  reg oPWM,
output  reg [15:0]oCMD
);
wire wFREQ;
Clkdiv #(
	.EXCEPTCLK	(1000)
) Clk1K (
	.iClk		(iCLK),	// 50Mhz clock 
	.iRst_n	    (iRst_n),// Reset
	.oClk		(wFREQ)	// ExpectClk clock
);

always @(posedge wFREQ)begin
    if(!iRst_n)begin

    end
    else begin
        oFB     <=  iFB;
        oNew_FB <=  iNew_FB;
        oPWM    <=  iPWM;
        oCMD    <=  iCMD;
    end


end

endmodule