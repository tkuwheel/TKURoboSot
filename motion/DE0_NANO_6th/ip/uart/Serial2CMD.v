// --------------------------------------------------------------------
// Copyright (c) 2012 by Intelligent Control Lab. of Tamkang University. 
// --------------------------------------------------------------------
//
//
// Major Functions: Convert serial to command
//
// --------------------------------------------------------------------
//
// Revision History :
// --------------------------------------------------------------------
//   Ver  :| Author            :| Mod. Date  :|  Changes Made:
//   0.1  :| Chih-en Wu        :| 2012/07/19 :|  Initial Version
// --------------------------------------------------------------------
// --------------------------------------------------------------------
//   Ver  :| Author            :| Mod. Date  :|  Changes Made:
//   1.8  :| Chun-Jui Huang    :| 2017/07/07 :|  Add Checksum and Shoot Control
// --------------------------------------------------------------------

module Serial2CMD (
//===========================================================================
// PORT declarations
//===========================================================================
input				iCLK, 		// 50MHz
input				iRst_n,		// Reset
input				iRx_ready,
input		[7:0]	iData,		// Data
output	reg	[7:0]	oCMD_Motor1,	// Command of motor1
output	reg	[7:0]	oCMD_Motor2,	// Command of motor2
output	reg	[7:0]	oCMD_Motor3,	// Command of motor3
output	reg	[7:0]	oSignal,			// Command of EN&STOP
output	reg	[7:0]	oAX_12,
output	reg	[7:0]	okick,			// shoot a ball at the goal
output	reg			oBrush,
output	reg			oRx_done,
output	reg			rError
);

//===========================================================================
// PARAMETER declarations
//===========================================================================
parameter SIZE	=	8;
// differentiate state in order to change state
parameter DATA0	=	8'b00000001;
parameter DATA1	=	8'b00000010;
parameter DATA2	=	8'b00000100;
parameter DATA3	=	8'b00001000;
parameter DATA4	=	8'b00010000;
parameter DATA5	=	8'b00100000;
parameter DATA6	=	8'b01000000;
parameter END	=	8'b10000000;

//=============================================================================
// REG/WIRE declarations
//=============================================================================
//	divide information to 6 part and 8 bits per part
reg		[7:0]	rData_0, rData_1, rData_2, rData_3, rData_4, rData_5,rData_6,rData_7;
reg		[7:0]	rTmpData_0, rTmpData_1, rTmpData_2, rTmpData_3, rTmpData_4, rTmpData_5, rTmpData_6, rTmpData_7;



reg		[SIZE-1:0]	state;

reg				rRx_ready;
reg				rCheck;
reg		[7:0]	rChecksum;
reg		[7:0]	null = 8'h0;
//=============================================================================
// Structural coding
//=============================================================================

always @(posedge iCLK) begin
	if(!iRst_n) begin		// Reset
		oRx_done	<=	0;
		rData_0		<=	0;
		rData_1		<=	0;
		rData_2		<=	0;
		rData_3		<=	0;
		rData_4		<=	0;
		rData_5		<=	0;
		rData_6		<=	0;
		rData_7		<=	0;
		state		<=	DATA0;
		oCMD_Motor1	<=	0;
		oCMD_Motor2	<=	0;
		oCMD_Motor3	<=	0;
		oSignal		<=	0;
		okick		<=	0;
		rError		<=	1;
		rChecksum	<=	0;
		rTmpData_0	<=	0;
		rTmpData_1	<=	0;
		rTmpData_2	<=	0;
		rTmpData_3	<=	0;
		rTmpData_4	<=	0;
		rTmpData_5	<=	0;
		rTmpData_6	<=	0;
		rTmpData_7	<=	0;
	end
	// Take apart Data
	else begin
		
		rChecksum <= rData_2 + rData_3 + rData_4 + rData_5 + rData_6;

		if(~rRx_ready & iRx_ready) begin
			case(state)
				DATA0:
					begin
						if( iData == 8'hFF ) begin	// when getting initiation packet, state jump next
							rData_0	<=	iData;
							state	<=	DATA1;
						end
					end
				DATA1:
					begin
						if( iData == 8'hFA ) begin	//when getting second initiation packet, start to receive and transmit Data
							rData_1	<=	iData;
							state	<=	DATA2;
						end
						else begin
							state	<=	DATA0;		
						end
					end
				DATA2:				
					begin
						rData_2	<=	iData;		//motor1
						state	<=	DATA3;
					end
				DATA3:				
					begin
						rData_3	<=	iData;		//motor2
						state	<=	DATA4;
					end
				DATA4:
					begin
						rData_4	<=	iData;		//motor3
						state	<=	DATA5;
					end
				DATA5:
					begin
						rData_5	<=	iData;		//enable+stop
						state	<=	DATA6;
					end
				DATA6:
					begin
						rCheck <= 0;
						rData_6	<=	iData;		//shoot
						state	<=	END;
					end
				END:
					begin
						rData_7	<=	iData;		//checksum
						oRx_done	<=	1;
						state	<=	DATA0;
					end
						

			endcase
		end
		else begin
//			oCMD_Motor1 <= rData_2;
//			oCMD_Motor2 <= rData_3;
//			oCMD_Motor3 <= rData_4;
//			oSignal 	<= rData_5;
//			okick 		<= rData_6;
//			
			rData_0 <= rData_0;
			rData_1 <= rData_1;
			rData_2 <= rData_2;
			rData_3 <= rData_3;
			rData_4 <= rData_4;
			rData_5 <= rData_5;
			rData_6 <= rData_6;
			rData_7 <= rData_7;
			rTmpData_0	<=	rData_0;
			rTmpData_1	<=	rData_1;
			rTmpData_2	<=	rData_2;
			rTmpData_3	<=	rData_3;
			rTmpData_4	<=	rData_4;
			rTmpData_5	<=	rData_5;
			if (rData_6 == 1)begin
				rTmpData_6	<=	0;
			end
			else begin
				rTmpData_6	<=	rData_6;
			end
			rTmpData_7 	<=	rData_7;
			state <= state;
			
			
			if(((rChecksum == rTmpData_7) && (rTmpData_0 == 8'hFF)) && 
				((rTmpData_1 == 8'hFA) && (rTmpData_5[1] == 1'b1)))begin
				rError <= 0;
				oCMD_Motor1 <= rTmpData_2;
				oCMD_Motor2 <= rTmpData_3;
				oCMD_Motor3 <= rTmpData_4;
				oSignal 	<= rTmpData_5;
				okick 		<= rTmpData_6;
			end
			else begin
				rError <= 1;
				oCMD_Motor1 	<= 	oCMD_Motor1;
				oCMD_Motor2 	<= 	oCMD_Motor2;
				oCMD_Motor3 	<= 	oCMD_Motor3;
				oSignal 		<= 	oSignal;
				okick			<= 	okick;
			end
			
			oRx_done	<=	0;
		end
		
		rRx_ready	<=	iRx_ready;
	end
end

endmodule
