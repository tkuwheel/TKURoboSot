`timescale 10 ns/ 1 ns

module Serial2CMD_tb();


wire clk;
wire [7:0] motor1;
wire [7:0] motor2;
wire [7:0] motor3;
wire [7:0] en;
wire [7:0] shoot;
wire [7:0] debug;
wire done;
wire CRCsuccess;
wire [15:0] Crc;


reg reset;
reg ready;
reg [7:0] data;
initial begin
    #0  reset = 0;          //reset
        ready = 0;
        data = 0;
    #3  reset = ~reset;
        ready = ~ready;
        data = 8'h0;
    // Data 1
    #3  ready = ~ready;     //ff
        data = 8'hff;
    #3  ready = ~ready;
        data = 8'hff;
    #3  ready = ~ready;     //fa
        data = 8'hfa;
    #3  ready = ~ready;
        data = 8'hfa;
    #3  ready = ~ready;     //w1
        data = 8'h1b;
    #3  ready = ~ready;
        data = 8'h1b;
    #3  ready = ~ready;     //w2
        data = 8'h90;
    #3  ready = ~ready;
        data = 8'h90;
    #3  ready = ~ready;     //w3
        data = 8'h22;
    #3  ready = ~ready;
        data = 8'h22;
    #3  ready = ~ready;     //en+stop
        data = 8'he0;
    #3  ready = ~ready;
        data = 8'he0;
    #3  ready = ~ready;     //shoot
        data = 8'hff;
    #3  ready = ~ready;
        data = 8'hff;
    #3  ready = ~ready;     //crc1
        data = 8'h7a;
    #3  ready = ~ready;
        data = 8'h7a;
    #3  ready = ~ready;     //crc2
        data = 8'h11;
    #3  ready = ~ready;
        data = 8'h11;
    #3  ready = ~ready;     //end
        data = 8'h0;
    #3  ready = ~ready;
        data = 8'h0;
    // Data 2
    #3  ready = ~ready;     //ff
        data = 8'hff;
    #3  ready = ~ready;
        data = 8'hff;
    #3  ready = ~ready;     //fa
        data = 8'hfa;
    #3  ready = ~ready;
        data = 8'hfa;
    #3  ready = ~ready;     //w1
        data = 8'h16;
    #3  ready = ~ready;
        data = 8'h16;
    #3  ready = ~ready;     //w2
        data = 8'h96;
    #3  ready = ~ready;
        data = 8'h96;
    #3  ready = ~ready;     //w3
        data = 8'h2d;
    #3  ready = ~ready;
        data = 8'h2d;
    #3  ready = ~ready;     //en+stop
        data = 8'he0;
    #3  ready = ~ready;
        data = 8'he0;
    #3  ready = ~ready;     //shoot
        data = 8'h1;
    #3  ready = ~ready;
        data = 8'h1;
    #3  ready = ~ready;     //crc1
        data = 8'h5e;
    #3  ready = ~ready;
        data = 8'h5e;
    #3  ready = ~ready;     //crc2
        data = 8'h12;
    #3  ready = ~ready;
        data = 8'h12;
    #3  ready = ~ready;     //end
        data = 8'h0;
    #3  ready = ~ready;
        data = 8'h0;
    // Data 3
    #3  ready = ~ready;     //ff
        data = 8'hff;
    #3  ready = ~ready;
        data = 8'hff;
    #3  ready = ~ready;     //fa
        data = 8'hfa;
    #3  ready = ~ready;
        data = 8'hfa;
    #3  ready = ~ready;     //w1
        data = 8'h16;
    #3  ready = ~ready;
        data = 8'h16;
    #3  ready = ~ready;     //w2
        data = 8'h96;
    #3  ready = ~ready;
        data = 8'h96;
    #3  ready = ~ready;     //w3
        data = 8'h2d;
    #3  ready = ~ready;
        data = 8'h2d;
    #3  ready = ~ready;     //en+stop
        data = 8'he0;
    #3  ready = ~ready;
        data = 8'he0;
    #3  ready = ~ready;     //shoot
        data = 8'h7f;
    #3  ready = ~ready;
        data = 8'h7f;
    #3  ready = ~ready;     //crc1
        data = 8'hc1;
    #3  ready = ~ready;
        data = 8'hc1;
    #3  ready = ~ready;     //crc2
        data = 8'h4b;
    #3  ready = ~ready;
        data = 8'h4b;
    #3  ready = ~ready;     //end
        data = 8'h0;
    #3  ready = ~ready;
        data = 8'h0;
    // Data 4
    #3  ready = ~ready;     //ff
        data = 8'hff;
    #3  ready = ~ready;
        data = 8'hff;
    #3  ready = ~ready;     //fa
        data = 8'hfa;
    #3  ready = ~ready;
        data = 8'hfa;
    #3  ready = ~ready;     //w1
        data = 8'h41;
    #3  ready = ~ready;
        data = 8'h41;
    #3  ready = ~ready;     //w2
        data = 8'h2e;
    #3  ready = ~ready;
        data = 8'h2e;
    #3  ready = ~ready;     //w3
        data = 8'h58;
    #3  ready = ~ready;
        data = 8'h58;
    #3  ready = ~ready;     //en+stop
        data = 8'he0;
    #3  ready = ~ready;
        data = 8'he0;
    #3  ready = ~ready;     //shoot
        data = 8'h7f;
    #3  ready = ~ready;
        data = 8'h7f;
    #3  ready = ~ready;     //crc1
        data = 8'hf4;
    #3  ready = ~ready;
        data = 8'hf4;
    #3  ready = ~ready;     //crc2
        data = 8'h47;
    #3  ready = ~ready;
        data = 8'h47;
    #3  ready = ~ready;     //end
        data = 8'h0;
    #3  ready = ~ready;
        data = 8'h0;
end
Clk_50M CLOCK_50(.clk(clk));

Serial2CMD serial(
.iCLK(clk), 		// 50MHz
.iRst_n(reset),		// Reset
.iRx_ready(ready),
.iData(data),		// Data

.oCMD_Motor1(motor1),	// Command of motor1
.oCMD_Motor2(motor2),	// Command of motor2
.oCMD_Motor3(motor3),	// Command of motor3
.oSignal(en),			// Command of EN&STOP
.oKick(shoot),			// shoot a ball 

.oRx_done(done),
.oCrcSuccess(CRCsuccess),
.debug(debug),
.oCrc(Crc)
);

endmodule