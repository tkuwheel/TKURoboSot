// --------------------------------------------------------------------
// Copyright (c) 2012 by Intelligent Control Lab. of Tamkang University. 
// --------------------------------------------------------------------
//
//
// Major Functions: Convert Package AX-12 command
//
// --------------------------------------------------------------------
//
// Revision History :
// --------------------------------------------------------------------
//   Ver  :| Author            :| Mod. Date  :|  Changes Made:
//   0.1  :| Chih-en Wu        :| 2012/08/05 :|  Initial Version
// --------------------------------------------------------------------

`default_nettype  none
module AX_12_Package (
input				iCLK,
input				iRst_n,
input				iSend_Ready,
input				iTx_busy,
input		[7:0]	iAX_12_CMD,
output	reg	[7:0]	oTx_data,
output	reg			oTx_send
);

parameter SIZE	=	3;
parameter SEND	=	3'b001;
parameter WAIT	=	3'b010;
parameter END	=	3'b100;

parameter PACKAGE_SIZE	=	10;
parameter STREAM_SIZE	=	PACKAGE_SIZE * 8;
parameter DELAY			=	1000;

reg	[STREAM_SIZE-1:0]	rData;

reg	[SIZE-1:0]	state;

reg				rSend_Ready;

reg				rSend;

reg		[7:0]	rCNT_Package;

reg		[15:0]	rCNT;

reg				rTx_busy;

reg		[7:0]	rChecksum;

always @(posedge iCLK) begin
	if ( (~rSend_Ready & iSend_Ready) & ~rSend) begin
		rSend	<=	1;
	end
	rSend_Ready	<=	iSend_Ready;

	if (!iRst_n) begin
		state			<=	SEND;
		rSend			<=	1;
		rSend_Ready		<=	0;
		rCNT_Package	<=	0;
		rCNT			<=	0;
		rTx_busy		<=	0;
		rChecksum		<=	0;
		rData[7:0]		<=	8'h FF;
		rData[15:8]		<=	8'h FF;
		rData[23:16]	<=	8'h FE;
		rData[31:24]	<=	8'h 07;
		rData[39:32]	<=	8'h 03;
		rData[47:40]	<=	8'h 1E;
		rData[63:48]	<=	16'h 200;
		rData[79:64] 	<=	16'h 200;
	end
	else if (rSend) begin
		case(state)
			SEND:
				begin
					if (rCNT_Package == PACKAGE_SIZE) begin
						oTx_data	<=	rChecksum ^ 8'hFF;
					end
					else begin
						oTx_data	<=	rData[7:0];
					end
					oTx_send	<=	1'b1;
					if (rTx_busy & ~iTx_busy) begin
						oTx_send	<=	1'b0;
						if (rCNT_Package > 1) begin
							rChecksum	<=	rChecksum + rData[7:0];
						end
						rData		<=	{rData[7:0], rData[STREAM_SIZE-1:8]};
						state		<=	WAIT;
					end
				end
			WAIT:
				begin
					if (rCNT < DELAY) begin
						rCNT	<=	rCNT + 1;
					end
					else begin
						rCNT	<=	0;
						state	<=	END;
					end
				end
			END:
				begin
					if (rCNT_Package < PACKAGE_SIZE) begin
						rCNT_Package	<=	rCNT_Package + 1;
						state			<=	SEND;
					end
					else begin
						rCNT_Package	<=	0;
						rChecksum		<=	0;
						state			<=	SEND;
						rSend			<=	0;
					end
				end
		endcase
	end
	else begin
		rData[7:0]		<=	8'h FF;
		rData[15:8]		<=	8'h FF;
		rData[23:16]	<=	8'h FE;
		rData[31:24]	<=	8'h 07;
		rData[39:32]	<=	8'h 03;
		rData[47:40]	<=	8'h 1E;
		if ( iAX_12_CMD <= 8'b00100000 ) begin		//0x20	32
			rData[63:48]	<=	16'b 0000001000000000;
		end
		else if ( iAX_12_CMD >= 8'b00110011 ) begin	// 0x33 51
			rData[63:48]	<=	16'b 0000001100110000;
		end
		else begin
			rData[63:48]	<=	{4'b0000, iAX_12_CMD, 4'b0000};
		end
		rData[79:64] 	<=	16'h 200;
	end
	rTx_busy	<=	iTx_busy;
end

endmodule
