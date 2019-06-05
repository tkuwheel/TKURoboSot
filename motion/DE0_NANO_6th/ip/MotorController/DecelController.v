`default_nettype none
module DecelController (
	input			iCLK,		// 50MHz, System Clock
	input			iRst_n,	// Reset
	input	[1:0]	iDIR,		// Direction of motor Input
	input	[7:0]	iPWM,
	output	reg	[1:0]	oDIR	// Direction of motor Output
);

reg		[5:0]	rCNT;	// Count
reg				state;
wire			wClk;


// Clock Divisor
Clkdiv #(
	.EXCEPTCLK	(10000)
) Clk10K (
	.iClk		(iCLK),			// System Clock
	.iRst_n		(iRst_n),	// Reset
	.oClk		(wClk)			// Clock	Output
);

always @(posedge wClk or negedge iRst_n)
begin
	if (!iRst_n) begin	// Reset
		rCNT	<=	0;
		oDIR	<=	2'b00;
		state	<=	0;
	end
	else begin
		case (state)
/////////////////	brake	////////////////
			0:	begin
				if	(rCNT == 6'd5) begin	// Is rCNT equal to 5
					state 	<=	1;			// let state is equal to one
					rCNT	<=	0;				// let rCNT is equal to zero
					oDIR	<=	2'b00;
				end
				else begin
					rCNT	<=	rCNT+1;
					oDIR	<=	2'b11;
				end
			end
/////////////////	release	/////////////
			1:	begin
				if	(rCNT == 6'd15) begin	// Is rCNT equal to 15
					state 	<=	0;				// let state is equal to zero
					rCNT	<=	0;					// let rCNT is equal to zero
					oDIR	<=	2'b11;
				end
				else begin
					rCNT	<=	rCNT+1;	// rCNT 
					oDIR	<=	2'b00;
					state	<=	state;	// state keep same
				end
			end
		default: begin
			rCNT	<=	rCNT;			// rCNT keep same
			oDIR	<=	iDIR;
			state	<=	state;		// state keep same
			end	
		endcase
	end
end

endmodule
