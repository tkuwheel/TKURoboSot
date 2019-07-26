module CMDTRAN (
input				iCLK,			// 50MHz, System Clock
input		[7:0]	iCMD,			// Command
output	reg	[6:0]	oSPD, // Speed of motor
output	reg			oDIR	// Direction of motor
);

always @(posedge iCLK)
begin
	if (iCMD[7]) begin		// Is iCMD[7] equal to 0
		oSPD	<=	iCMD[6:0];	// give speed iCMD[6:0]
		oDIR	<=	1'b1;			// give Positive direction of motor
	end
	else begin
		oSPD	<= iCMD[6:0];	// give speed iCMD[6:0]
		oDIR	<=	1'b0;			// give Negative direction of motor
	end
end
endmodule
