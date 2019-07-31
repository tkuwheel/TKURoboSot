// --------------------------------------------------------------------
// Copyright (c) 2012 by Intelligent Control Lab. of Tamkang University. 
// --------------------------------------------------------------------
//
// Major Functions: SKS Hardware Controller
//
// --------------------------------------------------------------------
//
// Revision History :
// --------------------------------------------------------------------
//   Ver  :| Author            :| Mod. Date  :|  Changes Made:
//   0.1  :| Shih-An Li        :| 2012/04/25 :|  Initial Version, create with Quartus II 9.1 SP2
//   0.2  :| Chih-En Wu        :| 2012/07/24 :|  Add RS232 Module
// --------------------------------------------------------------------
`default_nettype  none
module GFEC_TOP (
////////////////////////    Clock Input ////////////////////////
                iClk_50M,               // 50 MHz

////////////////////////    Push Button     ////////////////////////
                iButton,                //  Pushbutton[3:0], S3~S6
                iReset_n,               //  Reset_n, S1 button
////////////////////////    Switch      ////////////////////////
                iSW,			 //  Toggle Switch[3:0], 
////////////////////////////  LED Green ////////////////////////////
                oLEDG,                  //  LED Green[7:0]
////////////////////////////    UART    ////////////////////////////
                oUART_TXD,              //  UART Transmitter
                iUART_RXD,              //  UART Receiver
///////////////////////   SDRAM Interface (16Mbyte, 32bit wide)////////////////////////
                DRAM_DQ,                //  SDRAM Data bus 32 Bits
                oDRAM_A,                //  SDRAM Address bus 13 Bits
                oDRAM_DQM,              //  SDRAM Data Mask 
                oDRAM_WE_N,             //  SDRAM Write Enable
                oDRAM_CAS_N,            //  SDRAM Column Address Strobe
                oDRAM_RAS_N,            //  SDRAM Row Address Strobe
                oDRAM_CS_N,             //  SDRAM Chip Select
                oDRAM_BA,               //  SDRAM Bank Address
                oDRAM_CLK,              //  SDRAM Clock
                oDRAM_CKE,              //  SDRAM Clock Enable
////////////////////////    Flash Interface (8Mbyte)////////////////////////
                FLASH_DQ,               //  FLASH Data bus 15 Bits (0 to 14)
                oFLASH_A,               //  FLASH Address bus 22 Bits
                oFLASH_WE_N,            //  FLASH Write Enable
                oFLASH_RST_N,           //  FLASH Reset
                iFLASH_RY_N,            //  FLASH Ready/Busy output 
                oFLASH_OE_N,            //  FLASH Output Enable
                oFLASH_CE_N,            //  FLASH Chip Enable
////////////////////    LCD Module 16X2 ////////////////////////////
                LCD_D,                  //  LCD Data bus 8 bits
                oLCD_RW,                //  LCD Read/Write Select, 0 = Write, 1 = Read
                oLCD_EN,                //  LCD Enable
                oLCD_RS,                //  LCD Command/Data Select, 0 = Command, 1 = Data
////////////////////////    PS2   ////////////////////////////////
                PS2_DAT,                //  PS2 Data
                PS2_CLK,                //  PS2 Clock
                
////////////////////////	GPIO	////////////////////////////////
                GPIO_1,                 //  CON1 GPIO Connection 1 I/O
                GPIO_2,                 //  CON2 GPIO Connection 2 I/O
                GPIO_3,                 //  CON3 GPIO Connection 3 I/O
                GPIO_4,                 //  CON4 GPIO Connection 4 I/O
/////////////////////// Motors Feedback ////////////////////////////
				iMotor1_PA,
				iMotor1_PB,
				iMotor2_PA,
				iMotor2_PB,
				iMotor3_PA,
				iMotor3_PB,
				iMotor4_PA,
				iMotor4_PB,
/////////////////////// Motors Control /////////////////////////////
				oMotor1_PWM,
				oMotor1_DIR,
				oMotor2_PWM,
				oMotor2_DIR,
				oMotor3_PWM,
				oMotor3_DIR,
				oMotor4_PWM,	//PIN_R22
				oMotor4_DIR,	//DIR[1]:PIN_R20, DIR[0]:PIN_R21
////////////////////// RS-232 //////////////////////////////////////
				oTXD,
				iRXD,
////////////////// AX-12 Control ///////////////////////////////////
				oTXD_AX_12,
////////////////////// Others //////////////////////////////////////
				oLight
);

//===========================================================================
// PARAMETER declarations
//===========================================================================


//===========================================================================
// PORT declarations
//===========================================================================
////////////////////////    Clock Input ////////////////////////
input           iClk_50M;                // 50 MHz

////////////////////////    Push Button     ////////////////////////
input   [3:0]   iButton;                //  Pushbutton[3:0]
input           iReset_n;               //  Nios CPU Reset
////////////////////////    Switch      ////////////////////////
input   [3:0]   iSW;					//  Toggle Switch
////////////////////////////  LED Green ////////////////////////////
output  [7:0]   oLEDG;                  //  LED Green
////////////////////////////    UART    ////////////////////////////
output          oUART_TXD;              //  UART Transmitter
input           iUART_RXD;              //  UART Receiver
///////////////////////   SDRAM Interface (16Mbyte, 32bit wide)////////////////////////
inout   [31:0]  DRAM_DQ;                //  SDRAM Data bus 32 Bits
output  [11:0]  oDRAM_A;                //  SDRAM Address bus 13 Bits
output  [3:0]   oDRAM_DQM;              //  SDRAM Data Mask 
output          oDRAM_WE_N;             //  SDRAM Write Enable
output          oDRAM_CAS_N;            //  SDRAM Column Address Strobe
output          oDRAM_RAS_N;            //  SDRAM Row Address Strobe
output          oDRAM_CS_N;             //  SDRAM Chip Select
output  [1:0]   oDRAM_BA;               //  SDRAM Bank Address
output          oDRAM_CLK;              //  SDRAM Clock
output          oDRAM_CKE;              //  SDRAM Clock Enable
////////////////////////    Flash Interface (8Mbyte)////////////////////////
inout   [7:0]   FLASH_DQ;               //  FLASH Data bus 15 Bits (0 to 14)
output  [22:0]  oFLASH_A;               //  FLASH Address bus 22 Bits
output          oFLASH_WE_N;            //  FLASH Write Enable
output          oFLASH_RST_N;           //  FLASH Reset
input           iFLASH_RY_N;            //  FLASH Ready/Busy output 
output          oFLASH_OE_N;            //  FLASH Output Enable
output          oFLASH_CE_N;            //  FLASH Chip Enable

////////////////////    LCD Module 16X2 ////////////////////////////
inout   [7:0]   LCD_D;                  //  LCD Data bus 8 bits
output          oLCD_RW;                //  LCD Read/Write Select, 0 = Write, 1 = Read
output          oLCD_EN;                //  LCD Enable
output          oLCD_RS;                //  LCD Command/Data Select, 0 = Command, 1 = Data
////////////////////////    PS2   ////////////////////////////////
inout           PS2_DAT;                //  PS2 Data
inout           PS2_CLK;                //  PS2 Clock

////////////////////////    GPIO  need to add  ////////////////////////////////
inout   [19:0]  GPIO_1;                 //  CON1 GPIO Connection 1 I/O
inout   [19:0]  GPIO_2;                 //  CON2 GPIO Connection 2 I/O
inout   [19:0]  GPIO_3;                 //  CON3 GPIO Connection 3 I/O
inout   [19:1]  GPIO_4;                 //  CON4 GPIO Connection 4 I/O

/////////////////////// Motors Feedback ////////////////////////
input			iMotor1_PA;
input			iMotor1_PB;
input			iMotor2_PA;
input			iMotor2_PB;
input			iMotor3_PA;
input			iMotor3_PB;
input			iMotor4_PA;
input			iMotor4_PB;
/////////////////////// Motors Control ////////////////////////
output			oMotor1_PWM;
output	[1:0]	oMotor1_DIR;
output			oMotor2_PWM;
output	[1:0]	oMotor2_DIR;
output			oMotor3_PWM;
output	[1:0]	oMotor3_DIR;
output			oMotor4_PWM;
output	[1:0]	oMotor4_DIR;
////////////////////// RS-232 /////////////////////////////////
output			oTXD;
input				iRXD;
////////////////// AX-12 Control ///////////////////////////////////
output			oTXD_AX_12;
////////////////////// Others //////////////////////////////////////
output			oLight;

//=======================================================
//  REG/WIRE declarations
//=======================================================
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

wire	[7:0]	wFB_Motor1;
wire	[7:0]	wFB_Motor2;
wire			wDIR_Motor1;
wire			wDIR_Motor2;
wire			wFB_FREQ1;
wire			wFB_FREQ2;

wire	[14:0]	wFB_CNT_Motor1;
wire	[14:0]	wFB_CNT_Motor2;
wire			wFB_CNT_FREQ1;
wire			wFB_CNT_FREQ2;
wire			wFB_CNT_FREQ;

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
assign oFLASH_RST_N = iReset_n;

//Reset_Delay (
//	.iClk_50M(iClk_50M),
//	.iRst_n(iReset_n),
//	.oRST1ms_n(wRST1ms_n),
//	.oRST2ms_n(wRST2ms_n),
//	.oRST3ms_n(wRST3ms_n)
//);
//
//GFECSOPC (
//				// global signals:
//				.clk_50M(iClk_50M),
//				.dramclk(oDRAM_CLK),
//				.reset_n(wRST3ms_n),
//				//.systemclk,
//
//				// the_button_pio
//				.in_port_to_the_button_pio(iButton),
//
//				// the_dip_switch_pio
//				.in_port_to_the_dip_switch_pio(iSW),
//
//				// the_ext_ram_bus_avalon_slave
//				.address_to_the_cfi_flash(oFLASH_A),
//				.data_to_and_from_the_cfi_flash(FLASH_DQ),
//				.read_n_to_the_cfi_flash(oFLASH_OE_N),
//				.select_n_to_the_cfi_flash(oFLASH_CE_N),
//				.write_n_to_the_cfi_flash(oFLASH_WE_N),
//
//				// the_lcd_display
//				.LCD_E_from_the_lcd_display(oLCD_EN),
//				.LCD_RS_from_the_lcd_display(oLCD_RS),
//				.LCD_RW_from_the_lcd_display(oLCD_RW),
//				.LCD_data_to_and_from_the_lcd_display(LCD_D),
//
//				// the_led_pio
//				//.out_port_from_the_led_pio(oLEDG),
//
//				// the_sdram
//				.zs_addr_from_the_sdram(oDRAM_A),
//				.zs_ba_from_the_sdram(oDRAM_BA),
//				.zs_cas_n_from_the_sdram(oDRAM_CAS_N),
//				.zs_cke_from_the_sdram(oDRAM_CKE),
//				.zs_cs_n_from_the_sdram(oDRAM_CS_N),
//				.zs_dq_to_and_from_the_sdram(DRAM_DQ),
//				.zs_dqm_from_the_sdram(oDRAM_DQM),
//				.zs_ras_n_from_the_sdram(oDRAM_RAS_N),
//				.zs_we_n_from_the_sdram(oDRAM_WE_N),
//
//				// the_uart
//				//.rxd_to_the_uart(iUART_RXD),
//				//.txd_from_the_uart(oUART_TXD),
//				.rxd_to_the_uart(wRXD),
//				.txd_from_the_uart(wTXD),
//
//                   
//				// motor test
//				//.out_port_from_the_test_motor(wCMD_FL),
//                   
//				// RS-232 Test
//				//.out_port_from_the_test_tx_data(wTx_data),
//				//.out_port_from_the_test_tx_send(wTx_send),
//				//.in_port_to_the_test_rx_data(wRx_data),
//				//.in_port_to_the_test_rx_ready(wRx_ready),
//);

//TestDevice (
//	.iCLK				(iClk_50M),
//	.iRst_n				(iReset_n),
//	.iFB				(wFB_Motor2),
//	.iFB_FREQ			(wFB_FREQ2),
//	.ienable			(iButton[0]),
//	.oCMD_Motor1_Test	(wCMD_Motor1_Test),
//	.oCMD_Motor2_Test	(wCMD_Motor2_Test),
//	.oAX_12_Test		(wAX_12_Test),
//	.oRx_done			(wRx_done_Test),
//	.oBrush_Test		(wBrush_Test),
//	.oTest_Done			(wTest_Done)
//);
//
//assign wCMD_Motor1 = wTest_Done ? wCMD_Motor1_RS232 : wCMD_Motor1_Test;
//assign wCMD_Motor2 = wTest_Done ? wCMD_Motor2_RS232 : wCMD_Motor2_Test;
////assign wCMD_Motor3 = wTest_Done ? wCMD_Motor3_RS232 : wCMD_Motor3_Test;
////assign wCMD_Motor4 = wTest_Done ? wCMD_Motor4_RS232 : wCMD_Motor4_Test;
//assign wAX_12 = wTest_Done ? wAX_12_RS232 : wAX_12_Test;
//assign wBrush = wTest_Done ? wBrush_RS232 : wBrush_Test;
//assign wRx_done = wTest_Done ? wRx_done_RS232 : wRx_done_Test;

MotorController MotorLeft (
	.iCLK		(iClk_50M),
	.iRst_n		(iReset_n),
	.iCMD		(wCMD_Motor1),	// Command, SPD + DIR
	.iPA		(iMotor1_PA),	// Encoder Channel A
	.iPB		(iMotor1_PB),	// Encoder Channel B
	.oPWM_Pulse	(oMotor1_PWM),
	.oDIR		(oMotor1_DIR),
	.oDIR_Now	(wDIR_Motor1),
	.oFB		(wFB_Motor1),
	.oFB_FREQ	(wFB_FREQ1)
);

MotorController MotorRight (
	.iCLK		(iClk_50M),
	.iRst_n		(iReset_n),
	.iCMD		(wCMD_Motor2),
	.iPA		(iMotor2_PA),
	.iPB		(iMotor2_PB),
	.oPWM_Pulse	(oMotor2_PWM),
	.oDIR		(oMotor2_DIR),
	.oDIR_Now	(wDIR_Motor2),
	.oFB		(wFB_Motor2),
	.oFB_FREQ	(wFB_FREQ2)
);

assign oMotor3_DIR	=	2'b01;
assign oMotor3_PWM	=	wBrush ? 1'b1 : 1'b0;

//RS-232 Module
UART_if RS_232 (
	.iClk		(iClk_50M),	// Clock Input
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

assign oLEDG = iSW[0] ? wTx_data : wRx_data;

// Sperate package to command
Serial2CMD (
	.iCLK		(iClk_50M),
	.iRst_n		(iReset_n),
	.iData		(wRx_data),
	.iRx_ready	(wRx_ready),
//	.oCMD_Motor1(wCMD_Motor1_RS232),
//	.oCMD_Motor2(wCMD_Motor2_RS232),
//	.oAX_12		(wAX_12_RS232),
	.oCMD_Motor1(wCMD_Motor1),
	.oCMD_Motor2(wCMD_Motor2),
	.oAX_12		(wAX_12),
	.oLight		(oLight),
//	.oBrush		(wBrush_RS232),
//	.oRx_done	(wRx_done_RS232)
	.oBrush		(wBrush),
	.oRx_done	(wRx_done)
);

FB_Counter LeftMotor (
	.iCLK		(iClk_50M),
	.iRst_n		(iReset_n),
	.iSend		(wRx_done),
	.iFB_FREQ	(wFB_FREQ1),
	.iFB		(wFB_Motor1),
	.oFB		(wFB_CNT_Motor1),
	.oFB_FREQ	(wFB_CNT_FREQ1)
);

FB_Counter RightMotor (
	.iCLK		(iClk_50M),
	.iRst_n		(iReset_n),
	.iSend		(wRx_done),
	.iFB_FREQ	(wFB_FREQ2),
	.iFB		(wFB_Motor2),
	.oFB		(wFB_CNT_Motor2),
	.oFB_FREQ	(wFB_CNT_FREQ2)
);

assign wFB_CNT_FREQ = wFB_CNT_FREQ1 & wFB_CNT_FREQ2;

// Combine feedbacks to package
CMD2Serial (
	.iCLK			(iClk_50M),
	.iRst_n			(iReset_n),
	.iSend_Ready	(wFB_CNT_FREQ), //Feedback frequency
	.iTx_busy		(wTx_busy),
	.iFB_Motor1		(wFB_CNT_Motor1),
	.iFB_Motor2		(wFB_CNT_Motor2),
	.iDIR_Motor1	(wDIR_Motor1),
	.iDIR_Motor2	(wDIR_Motor2),
	.oTx_send		(wTx_send), //Send trigger
	.oTx_data		(wTx_data)  //Send data
);

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

endmodule
