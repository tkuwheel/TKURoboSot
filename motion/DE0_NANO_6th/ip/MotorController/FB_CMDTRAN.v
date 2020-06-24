module FB_CMDTRAN (
input				iCLK,			// 50MHz, System Clock
input				iDIR,	// Direction of motor
input				[6:0]	iSPD, // Speed of motor
output	reg	[7:0]	oCMD			// Command
);

always @(posedge iCLK)
begin
	if (oCMD[7]) begin		// Is iCMD[7] equal to 0
		oCMD[7]  	<= 1'b1;			// give Positive direction of motor
		oCMD[6:0]	<=	iSPD[6:0];	// give speed iCMD[6:0]
	end
	else begin
		oCMD[7]  	<= 1'b0;			// give Positive direction of motor
		oCMD[6:0]	<=	iSPD[6:0];	// give speed iCMD[6:0]
	end
end
endmodule 