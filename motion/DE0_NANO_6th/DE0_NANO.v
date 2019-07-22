// ============================================================================
// Copyright (c) 2011 by Terasic Technologies Inc. 
// ============================================================================
//
// Permission:
//
//   Terasic grants permission to use and modify this code for use
//   in synthesis for all Terasic Development Boards and Altera Development 
//   Kits made by Terasic.  Other use of this code, including the selling 
//   ,duplication, or modification of any portion is strictly prohibited.
//
// Disclaimer:
//
//   This VHDL/Verilog or C/C++ source code is intended as a design reference
//   which illustrates how these types of functions can be implemented.
//   It is the user's responsibility to verify their design for
//   consistency and functionality through the use of formal
//   verification methods.  Terasic provides no warranty regarding the use 
//   or functionality of this code.
//
// ============================================================================
//           
//                     Terasic Technologies Inc
//                     356 Fu-Shin E. Rd Sec. 1. JhuBei City,
//                     HsinChu County, Taiwan
//                     302
//
//                     web: http://www.terasic.com/
//                     email: support@terasic.com
//
// ============================================================================
// Major Functions/Design Description:
//
//   Please refer to DE0_Nano_User_manual.pdf in DE0_Nano system CD.
//
// ============================================================================
// Revision History:
// ============================================================================
//   Ver.: |Author:   |Mod. Date:    |Changes Made:
//   V1.0  |EricChen  |02/01/2011    |
// ============================================================================

//=======================================================
//  This code is generated by Terasic System Builder
//=======================================================
`default_nettype  none
module DE0_NANO(

	//////////// CLOCK //////////
	CLOCK_50,

	//////////// LED //////////
	LED,

	//////////// KEY //////////
	KEY,

	//////////// SW //////////
	SW,

	//////////// SDRAM //////////
	DRAM_ADDR,
	DRAM_BA,
	DRAM_CAS_N,
	DRAM_CKE,
	DRAM_CLK,
	DRAM_CS_N,
	DRAM_DQ,
	DRAM_DQM,
	DRAM_RAS_N,
	DRAM_WE_N,

	//////////// EPCS //////////
	EPCS_ASDO,
	EPCS_DATA0,
	EPCS_DCLK,
	EPCS_NCSO,

	//////////// Accelerometer and EEPROM //////////
	G_SENSOR_CS_N,
	G_SENSOR_INT,
	I2C_SCLK,
	I2C_SDAT,

	//////////// ADC //////////
	ADC_CS_N,
	ADC_SADDR,
	ADC_SCLK,
	ADC_SDAT,

	//////////// 2x13 GPIO Header //////////
	GPIO_2,
	GPIO_2_IN,

	//////////// GPIO_0, GPIO_0 connect to GPIO Default //////////
	GPIO_0_D,
	GPIO_0_IN,

	//////////// GPIO_0, GPIO_1 connect to GPIO Default //////////
	GPIO_1_D,
	GPIO_1_IN,

);

//=======================================================
//  PARAMETER declarations
//=======================================================


//=======================================================
//  PORT declarations
//=======================================================

//////////// CLOCK //////////
input 		          		CLOCK_50;

//////////// LED //////////
output		     [7:0]		LED;

//////////// KEY //////////
input 		     [1:0]		KEY;

//////////// SW //////////
input 		     [3:0]		SW;

//////////// SDRAM //////////
output		    [12:0]		DRAM_ADDR;
output		     [1:0]		DRAM_BA;
output		          		DRAM_CAS_N;
output		          		DRAM_CKE;
output		          		DRAM_CLK;
output		          		DRAM_CS_N;
inout 		    [15:0]		DRAM_DQ;
output		     [1:0]		DRAM_DQM;
output		          		DRAM_RAS_N;
output		          		DRAM_WE_N;

//////////// EPCS //////////
output		          		EPCS_ASDO;
input 		          		EPCS_DATA0;
output		          		EPCS_DCLK;
output		          		EPCS_NCSO;

//////////// Accelerometer and EEPROM //////////
output		          		G_SENSOR_CS_N;
input 		          		G_SENSOR_INT;
output		          		I2C_SCLK;
inout 		          		I2C_SDAT;

//////////// ADC //////////
output		          		ADC_CS_N;
output		          		ADC_SADDR;
output		          		ADC_SCLK;
input 		          		ADC_SDAT;

//////////// 2x13 GPIO Header //////////
inout 		    [12:0]		GPIO_2;
input 		     [2:0]		GPIO_2_IN;

//////////// GPIO_0, GPIO_0 connect to GPIO Default //////////
inout 		    [33:0]		GPIO_0_D;
input 		     [1:0]		GPIO_0_IN;

//////////// GPIO_0, GPIO_1 connect to GPIO Default //////////
inout 		    [33:0]		GPIO_1_D;
input 		     [1:0]		GPIO_1_IN;


//=======================================================
//  REG/WIRE declarations
//=======================================================
wire 			oFLASH_RST_N;
wire			iReset_n;
wire  		iMotor1_PA;
wire  		iMotor1_PB;
wire  		oMotor1_PWM;
wire  		oMotor1_DIR;
wire  		iMotor2_PA;
wire  		iMotor2_PB;
wire 			oMotor2_PWM;
wire   		oMotor2_DIR;
wire  		iMotor3_PA;
wire  		iMotor3_PB;
wire 			oMotor3_PWM;
wire  		oMotor3_DIR;
wire  		iMotor4_PA;
wire  		iMotor4_PB;
wire 			oMotor4_PWM;
wire  		oMotor4_DIR;
wire 			oMotor5_PWM;
wire 			oMotor5_DIR;
wire			oTXD;
wire			iRXD;
wire			oLight;
wire			oKick;
wire	[7:0]	wKick;
wire			wRST1ms_n, wRST2ms_n, wRST3ms_n;
wire	[7:0]	wCMD_Motor1;
wire	[7:0]	wCMD_Motor2;
wire	[7:0]	wCMD_Motor3;
wire	[7:0]	wCMD_Motor4;
wire	[7:0]	wAX_12;
wire			wBrush;

wire			wTx_send;
wire	[7:0]	wTx_data;
wire			wTx_busy;
wire			wRx_ready;
wire	[7:0]	wCMD_Motor1_RS232;
wire	[7:0]	wCMD_Motor2_RS232;
wire	[7:0]	wCMD_Motor3_RS232;
wire	[7:0]	wCMD_Motor4_RS232;
wire	[7:0]	wAX_12_RS232;
wire			wBrush_RS232;
wire	[7:0]	wRx_data;
wire			wRx_done;
wire			wRx_done_RS232;

wire			wTx_AX_12_send;
wire	[7:0]	wTx_AX_12_data;
wire			wTx_AX_12_busy;

wire	[31:0]	wFB_Motor1;
wire	[31:0]	wFB_Motor2;
wire	[31:0]	wFB_Motor3;
wire	[31:0]	wFB_Motor4;
wire	 		wDIR_Motor1;
wire			wDIR_Motor2;
wire			wDIR_Motor3;
wire			wDIR_Motor4;
wire	[7:0]	wSignal;

wire			wFB_FREQ1;
wire			wFB_FREQ2;
wire			wFB_FREQ3;
wire			wFB_FREQ4;

wire	[14:0]wFB_CNT_Motor1;
wire	[14:0]wFB_CNT_Motor2;
wire	[14:0]wFB_CNT_Motor3;
wire	[14:0]wFB_CNT_Motor4;
wire			wFB_CNT_FREQ;
wire			wFB_CNT_FREQ1;
wire			wFB_CNT_FREQ2;
wire			wFB_CNT_FREQ3;
wire			wFB_CNT_FREQ4;
wire			wFB_CNT_FREQ_AB;
wire			wFB_CNT_FREQ_CD;

wire			wTXD;
wire			wRXD;

//wire	[7:0]	wCMD_Motor1_Test;
//wire	[7:0]	wCMD_Motor2_Test;
//wire	[7:0]	wAX_12_Test;
//wire			wBrush_Test;
//wire			wRx_done_Test;
//wire			wTest_Done;

//=======================================================
//  Structural coding
//=======================================================
//assign GPIO_0_D[] = oFLASH_RST_N;
assign iReset_n = KEY[0];
assign oFLASH_RST_N = iReset_n;


wire wclk_50hz;
Clkdiv #(
	.EXCEPTCLK	(5000)
) Clk1K (
	.iClk		(CLOCK_50),	// 50Mhz clock 
	.iRst_n	(iReset_n),// Reset
  	.oSampClk(GPIO_1_D[0]),  // multipleX expect clock, for SignalTap use
//	.oClk		(wclk_50hz)	// ExpectClk clock
);

assign iMotor1_PA = GPIO_0_D[22];
assign iMotor1_PB = GPIO_0_D[28];
assign GPIO_0_D[20] = oMotor1_PWM;
assign GPIO_0_D[16] = oMotor1_DIR;
assign GPIO_0_D[18] = wSignal[7];
assign GPIO_0_D[14] = wSignal[4];


//assign GPIO_0_D[24] = wFB_CNT_Motor1;
//assign GPIO_0_D[26] = wFB_CNT_Motor2;
//assign GPIO_0_D[31] = wFB_Motor3;

MotorController MotorA (
	.iCLK		(CLOCK_50),			// 50MHz, System Clock
	.iRst_n		(iReset_n),		// Reset
	.iCMD		(wCMD_Motor1),		// Command, SPD + DIR
	.iPA		(iMotor1_PA),		// Encoder Channel A
	.iPB		(iMotor1_PB),		// Encoder Channel B
	.oPWM_Pulse	(oMotor1_PWM),	// PWM of motor
	.oDIR		(oMotor1_DIR),		// Direction of motor
	.oDIR_Now	(wDIR_Motor1),	// Direction of motor now
	.oFB		(wFB_Motor1),		// Feedback of motor
	.oFB_FREQ	(wFB_FREQ1)		
);

assign iMotor2_PA = GPIO_0_D[30];
assign iMotor2_PB = GPIO_0_D[32];
assign GPIO_0_D[21] = oMotor2_PWM;
assign GPIO_0_D[25] = oMotor2_DIR;
assign GPIO_0_D[23] = wSignal[6];
assign GPIO_0_D[27] = wSignal[3];


MotorController MotorB (
	.iCLK		(CLOCK_50),			// 50MHz, System Clock
	.iRst_n		(iReset_n),		// Reset
	.iCMD		(wCMD_Motor2),		// Command, SPD + DIR
	.iPA		(iMotor2_PA),		// Encoder Channel A
	.iPB		(iMotor2_PB),		// Encoder Channel B
	.oPWM_Pulse	(oMotor2_PWM),	// PWM of motor
	.oDIR		(oMotor2_DIR),		// Direction of motor
	.oDIR_Now	(wDIR_Motor2),	// Direction of motor now
	.oFB		(wFB_Motor2),		// Feedback of motor
	.oFB_FREQ	(wFB_FREQ2)		// Feedback renew trigger
);

assign iMotor3_PA = GPIO_0_D[17];
assign iMotor3_PB = GPIO_0_D[19];
assign GPIO_0_D[1] = oMotor3_PWM;
assign GPIO_0_D[5] = oMotor3_DIR;
assign GPIO_0_D[3] = wSignal[5];
assign GPIO_0_D[7] = wSignal[2];


MotorController MotorC (
	.iCLK		(CLOCK_50),			// 50MHz, System Clock
	.iRst_n		(iReset_n),		// Reset
	.iCMD		(wCMD_Motor3),		// Command, SPD + DIR
	.iPA		(iMotor3_PA),		// Encoder Channel A
	.iPB		(iMotor3_PB),		// Encoder Channel B
	.oPWM_Pulse	(oMotor3_PWM),	// PWM of motor
	.oDIR		(oMotor3_DIR),		// Direction of motor
	.oDIR_Now	(wDIR_Motor3),	// Direction of motor now
	.oFB		(wFB_Motor3),		// Feedback of motor
	.oFB_FREQ	(wFB_FREQ3)		// Feedback renew trigger
);

/*
assign iMotor4_PA = GPIO_0_D[22];
assign iMotor4_PB = GPIO_0_D[24];
assign GPIO_1_D[13] = oMotor4_PWM;
assign GPIO_1_D[15] = oMotor4_DIR[0];
assign GPIO_1_D[11] = oMotor4_DIR[1];

MotorController MotorD (
	.iCLK		(CLOCK_50),			// 50MHz, System Clock
	.iRst_n		(iReset_n),		// Reset
	.iCMD		(wCMD_Motor4),		// Command, SPD + DIR
	.iPA		(iMotor4_PA),		// Encoder Channel A
	.iPB		(iMotor4_PB),		// Encoder Channel B
	.oPWM_Pulse	(oMotor4_PWM), // PWM of motor
	.oDIR		(oMotor4_DIR),		// Direction of motor
	.oDIR_Now	(wDIR_Motor4),	// Direction of motor now
	.oFB		(wFB_Motor4),		// Feedback of motor
	.oFB_FREQ	(wFB_FREQ4)		// Feedback renew trigger
);
*/

//assign GPIO_0_D[] = oMotor5_DIR;
//assign GPIO_0_D[] = oMotor5_PWM;
//assign oMotor5_DIR	=	2'b01;
//assign oMotor5_PWM	=	wBrush ? 1'b1 : 1'b0;

//蹓---------------------------------------------------------------------------------------------------------

/*wire 			oRH;
wire			oLH;

assign GPIO_0_D[4] = oRH;
assign GPIO_0_D[6] = oLH;*/

holdBall(.iC(CLOCK_50),
			.oL(GPIO_0_D[4]),
			.oR(GPIO_0_D[6]),
			.iCMD1(wCMD_Motor1),
			.iCMD2(wCMD_Motor2),
			.iCMD3(wCMD_Motor3),
			.iS(wSignal[0])
//			.oLED(LED[1])
			);
//------------------------------------------------------------------------------------------------------------
//=======================================================

ShootControl (
			.iClk(CLOCK_50),
			.iRst_n(iReset_n),
			.iPower(wKick),
			.oPower(oKick)
);

assign GPIO_0_D[33] = oTXD;
assign iRXD = GPIO_0_D[11];
//RS-232 Module
UART_if RS_232 (
	.iClk		(CLOCK_50),		// Clock Input
	.iRst_n		(iReset_n),	// Reset Input

	// RS232 control signal
	.iTX_send	(wTx_send),
	.iTX_data	(wTx_data),
	.oTX_busy	(wTx_busy),
	.oRX_drdy	(wRx_ready),
	.oRX_data	(wRx_data),
	
	// RS232 interface
	.oRs232_tx	(oTXD),
	.iRs232_rx	(iRXD)
	//.oRs232_tx	(wRXD),
	//.iRs232_rx	(wTXD)
);

//assign LED = SW[0] ? wTx_data : wRx_data;
//assign GPIO_0_D[] = oLight;
assign GPIO_0_D[29] = oKick;
//assign LED[0] = okick;
assign LED[7:2] = wSignal[7:2];
assign LED[0] = oKick;
	
// Sperate package to command
Serial2CMD (
	.iCLK		(CLOCK_50),				// 50MHz, System Clock
	.iRst_n		(iReset_n),			// Reset
	.iData		(wRx_data),
	.iRx_ready	(wRx_ready),
	.oCMD_Motor1(wCMD_Motor1),		// Command of motor1
	.oCMD_Motor2(wCMD_Motor2),		// Command of motor2
	.oCMD_Motor3(wCMD_Motor3),		// Command of motor3
	.oSignal	(wSignal),			// Command of Enable & Stop signal
	.oKick		(wKick),				// shoot a ball at the goal
	.oRx_done	(wRx_done)
);


// FB_Counter Motor_A (
// 	.iCLK		(CLOCK_50),				// 50MHz, System Clock
// 	.iRst_n		(iReset_n),			// Reset
// 	.iSend		(wRx_done),
// 	.iFB_FREQ	(wFB_FREQ1),		// Feedback frequency Input
// 	.iFB		(wFB_Motor1),			// Feedback of motor1 Input
// 	.oFB		(wFB_CNT_Motor1),		// Feedback of motor1 Output
// 	.oFB_FREQ	(wFB_CNT_FREQ1)	// Feedback frequency Output
// );

// FB_Counter Motor_B (
// 	.iCLK		(CLOCK_50),				// 50MHz, System Clock
// 	.iRst_n		(iReset_n),			// Reset
// 	.iSend		(wRx_done),
// 	.iFB_FREQ	(wFB_FREQ2),		// Feedback frequency Input
// 	.iFB		(wFB_Motor2),			// Feedback of motor2 Input
// 	.oFB		(wFB_CNT_Motor2),		// Feedback of motor2 Output
// 	.oFB_FREQ	(wFB_CNT_FREQ2)	// Feedback frequency Output
// );

// FB_Counter Motor_C (
// 	.iCLK		(CLOCK_50),				// 50MHz, System Clock
// 	.iRst_n		(iReset_n),			// Reset
// 	.iSend		(wRx_done),
// 	.iFB_FREQ	(wFB_FREQ3),		// Feedback frequency Input
// 	.iFB		(wFB_Motor3),			// Feedback of motor3 Input
// 	.oFB		(wFB_CNT_Motor3),		// Feedback of motor3 Output
// 	.oFB_FREQ	(wFB_CNT_FREQ3)	// Feedback frequency Output
// );


/*
FB_Counter Motor_D (
	.iCLK		(CLOCK_50),				// 50MHz, System Clock
	.iRst_n		(iReset_n),			// Reset
	.iSend		(wRx_done),
	.iFB_FREQ	(wFB_FREQ4),		// Feedback frequency Input
	.iFB		(wFB_Motor4),			// Feedback of motor4 Input
	.oFB		(wFB_CNT_Motor4),		// Feedback of motor4 Output
	.oFB_FREQ	(wFB_CNT_FREQ4)	// Feedback frequency Output
);
*/


assign wFB_CNT_FREQ = wFB_FREQ1 & wFB_FREQ2 & wFB_FREQ3;
// assign wFB_CNT_FREQ = wFB_FREQ1;

// Combine feedbacks to package
CMD2Serial (
	.iCLK			(CLOCK_50),				// 50MHz, System Clock	
	.iRst_n			(iReset_n),			// Reset
	.iSend_Ready	(wFB_CNT_FREQ), 	// Feedback frequency
	.iTx_busy		(wTx_busy),
	.iFB_Motor1		(wFB_Motor1),	// Feedback of motor1
	.iFB_Motor2		(wFB_Motor2),	// Feedback of motor2
	.iFB_Motor3		(wFB_Motor3),	// Feedback of motor3
//	.iFB_Motor4		(wFB_CNT_Motor4),	// Feedback of motor4
	.iDIR_Motor1	(wDIR_Motor1),		// Direction of motor1
	.iDIR_Motor2	(wDIR_Motor2),		// Direction of motor2
	.iDIR_Motor3	(wDIR_Motor3),		// Direction of motor3
//	.iDIR_Motor4	(wDIR_Motor4),		// Direction of motor4
	.oTx_send		(wTx_send), //Send trigger
	.oTx_data		(wTx_data)  //Send data
);

/*
AX_12_Package (
	.iCLK			(iClk_50M),
	.iRst_n			(iReset_n),
	.iSend_Ready	(wRx_done),
	.iTx_busy		(wTx_AX_12_busy),
	.iAX_12_CMD		(wAX_12),
	.oTx_data		(wTx_AX_12_data),
	.oTx_send		(wTx_AX_12_send)
);

UART_if AX_12_TXD (
	.iClk		(iClk_50M),  // Clock Input
	.iRst_n		(iReset_n),  // Reset Input

	// RS232 control signal
	.iTX_send	(wTx_AX_12_send),
	.iTX_data	(wTx_AX_12_data),
	.oTX_busy	(wTx_AX_12_busy),
	
	// RS232 interface
	.oRs232_tx	(oTXD_AX_12),
);
*/
endmodule
