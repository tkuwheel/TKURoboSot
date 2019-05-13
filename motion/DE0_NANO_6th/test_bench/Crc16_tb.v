`timescale 10 ns/ 1 ns
module Crc16_tb;
wire clk;
// wire wclk;
wire [15:0] crc;
// wire [15:0] tmpcrc;
// wire [7:0] counter;
// wire [71:0] data;
wire    success;
wire    finish;
// wire [71:0] tmpdata;
reg [127:0] iData;
reg reset;
reg ready;
// initial begin
//     # 0 ready = 0;
//         reset = 0;
//         iData = 72'hFFFA8080800000767d;
//     # 2 ready = 1;
//         reset = 1;
//         iData = {72'hFFFA8080800000767d};
//     # 2 ready = 0;
//         iData = {72'hFFFA80808000001234};
//     # 2 ready = 1;
//         iData = {72'hFFFA80808000001234};
//     # 2 ready = 0;
//         iData = {72'hFFFA171717e0004d8a};
//     # 2 ready = 1;
//         iData = {72'hFFFA171717e0004d8a};
//     # 2 ready = 0;
//         iData = {72'hFFFA171717e0005678};
//     # 2 ready = 1;
//         iData = {72'hFFFA171717e0005678};
//     # 2 ready = 0;
//         iData = {72'hFFFA1b9022e0ff7a11};
//     # 2 ready = 1;
//         iData = {72'hFFFA1b9022e0ff7a11};
//     # 2 ready = 0;
//         iData = {72'hFFFA1b9022e0ff7a12};
//     # 2 ready = 1;
//         iData = {72'hFFFA1b9022e0ff7a12};
// end

initial begin
    # 0 ready = 0;
        reset = 0;
        iData = 72'hFFFA8080800000767d;
    # 2 ready = 1;
        reset = 1;
        iData = {8'hff, 8'hfa, 112'h0};
    
end
Clk_50M CLOCK_50(.clk(clk));
// assign wclk =  clk;

// Crc16 #(
//     .PACKAGE_SIZE(9),
//     .STREAM_SIZE(72)
//     )Crc_RX(
//       .iClk(clk), 
//       .iRst_n(reset), 
//       .iDataValid(ready), 
//       .iData(iData),
//       .oSuccess(success), 
//       .oFinish(finish), 
//       .oCrc(crc)
//     );

Crc16 #(
    .PACKAGE_SIZE(16),
    .STREAM_SIZE(128)
    )Crc_TX(
      .iClk(clk), 
      .iRst_n(reset), 
      .iDataValid(ready), 
      .iData(iData),
      .oSuccess(success), 
      .oFinish(finish), 
      .oCrc(crc)
    );
endmodule