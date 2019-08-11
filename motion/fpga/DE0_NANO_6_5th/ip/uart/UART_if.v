module UART_if #(
parameter BaudRate	=	115200/1
)(
           iClk,       // Clock Input
           iRst_n,     // Reset Input
           oSampclk,   // for debug,

    // RS232 control signal
           iTX_send,  // a send trigger
           iTX_data,
           oTX_busy,  
           oTX_done,  // ~ oTX_busy
    
           oRX_drdy,  // when receive a data, set to 1
           oRX_data,  // receive data
    
    // RS232 interface
           oRs232_tx,  // RS232 output
           iRs232_rx   // RS232 input
);

  input         iClk;  // Clock Input
  input         iRst_n;  // Reset Input
  output        oSampclk;
    
    // RS232 control signal
  input         iTX_send;
  input [7:0]   iTX_data;
  output        oTX_busy;
  output        oTX_done;
    
  output        oRX_drdy;
  output reg [7:0]  oRX_data;
    
    
    // RS232 interface
  output        oRs232_tx;   // RS232 output
  input         iRs232_rx;   // RS232 input

  // --------------------------------------------------------------------
  //  8250A Registers
  // --------------------------------------------------------------------
  wire [7:0] wRX_data;           // Wired to receiver
  reg  [7:0] wTX_data;         // Transmit register

  // --------------------------------------------------------------------
  // Instantiate the UART
  // --------------------------------------------------------------------
  wire    wRX_drdy;               // Indicates new data has come in, a trigger
  reg     rTX_send;                // Signal to send data
  wire    wTX_done;
  wire    wTX_busy;                // Signal transmitter is busy
  wire    wClk1p8432M;


  // --------------------------------------------------------------------
  // Transmit behavior
  // --------------------------------------------------------------------
reg rDly_iTX_send;
always @(posedge wClk1p8432M or negedge iRst_n) begin    // Synchrounous
    if(!iRst_n) begin
        rTX_send = 0;
        rDly_iTX_send <= 0;
    end
    else begin
        rDly_iTX_send <= iTX_send;
        if({rDly_iTX_send, iTX_send}==2'b01)  // check a trigger for TX send signal
            rTX_send <= 1;
        else 
            rTX_send <= 0;
    end
end
  
clk_gen #(
    .res	(19),
    .phase	(19'd19327)
) UartTxclk (
    .iClk	(iClk),         // 50MHz
    .iRst_n	(iRst_n),
    .oClk	(wClk1p8432M)   // 1.8431 MHz (required 1.8432 MHz)
);

 // --------------------------------------------------------------------
  //  Baud Rate Generator:
  //  Once we have our little 1.8432Mhz Baud Clock, deriving the bauds is
  //  simple simon. Just divide by 16 to get the 1x baud for transmitting
  //  and divide by 2 to get the 8x oversampling clock for receiving.
  //
  // Baud Clock = 1.8432Mhz
  // Divisor    = 16
  //
  //   Baud   Divsr %Error
  // ------   ----- -----
  //     50  2304  0.000%
  //     75  1536  0.000%
  //    110  1047  0.026%
  //    150   768  0.000%
  //    300   384  0.000%
  //    600   192  0.000%
  //   1200    96  0.000%
  //   2400    48  0.000%
  //   4800    24  0.000%
  //   7200    16  0.000%
  //   9600    12  0.000%
  //  14400     8  0.000%
  //  19200     6  0.000%
  //  28800     4  0.000%
  //  38400     3  0.000%
  //  57600     2  0.000%
  // 115200     1  0.000%
  //
  // --------------------------------------------------------------------
reg [19:0] rDivFreqCnt1, rDivFreqCnt2;

reg rBaudRate8XTick, rBaudRate1XTick;

// Generate request Baud rate 
always@(posedge wClk1p8432M or negedge iRst_n) begin
    if(!iRst_n) begin
        rDivFreqCnt1 <= 0;
        rDivFreqCnt2 <= 0;
        rBaudRate8XTick<=0;
        rBaudRate1XTick<=0;
    end
    else begin
        if( rDivFreqCnt1 >= (1843200/(BaudRate)-1)) begin
            rBaudRate1XTick <= 1;
            rDivFreqCnt1 <= 0;
        end
        else begin
            rBaudRate1XTick <= 0;
            rDivFreqCnt1 <= rDivFreqCnt1 +1;	// up count
        end
        
        if( rDivFreqCnt2 >= (1843200/(BaudRate*8)-1)) begin
            rBaudRate8XTick <= 1;
            rDivFreqCnt2 <= 0;
        end
        else begin
            rBaudRate8XTick <= 0;
            rDivFreqCnt2 <= rDivFreqCnt2 +1;	// up count
        end
    end
end

serial_atx atx (
    .clk		(wClk1p8432M),
    .iRst_n		(iRst_n),
    .baud1tick	(rBaudRate1XTick),
    .txd		(oRs232_tx),
    .txd_start	(rTX_send),
    .txd_data 	(iTX_data), //input_data
    .txd_busy	(wTX_busy)
);

serial_arx arx (
    .clk				(wClk1p8432M),
    .iRst_n				(iRst_n),
    .baud8tick			(rBaudRate8XTick),
    .rxd				(iRs232_rx),
    .rxd_data_ready		(wRX_drdy),
    .oRXd_data			(wRX_data),
);
assign oRX_drdy = wRX_drdy;
//  assign oRX_data = wRX_data;

always@(posedge wClk1p8432M) begin
    oRX_data <= wRX_data;
end

// Combinatorial logic
assign wTX_done = ~wTX_busy;     // Signal command finished sending
assign oTX_busy = wTX_busy;

endmodule
