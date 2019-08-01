module CMDTRAN #(
SIZE = 16
)(
input				iCLK,	 // 50MHz, System Clock
input		[SIZE - 1:0]	iCMD,	 // Command
output	reg	[SIZE - 1:0]	oSPD,	 // Speed of motor
output	reg			oDIR	// Direction of motor
);

always @(posedge iCLK)
begin
	if (iCMD[15]) begin		// Is iCMD[7] equal to 0
		oSPD	<=	~iCMD[SIZE-1:0]+1;		// give speed iCMD[6:0]
		oDIR	<=	1'b0;			// give Positive direction of motor
	end
	else begin
		oSPD	<= iCMD[SIZE-1:0];		// give speed iCMD[6:0]
		oDIR	<=	1'b1;			// give Negative direction of motor
	end
end
endmodule
