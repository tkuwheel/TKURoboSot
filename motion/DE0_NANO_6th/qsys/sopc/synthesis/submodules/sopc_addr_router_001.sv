// (C) 2001-2013 Altera Corporation. All rights reserved.
// Your use of Altera Corporation's design tools, logic functions and other 
// software and tools, and its AMPP partner logic functions, and any output 
// files any of the foregoing (including device programming or simulation 
// files), and any associated documentation or information are expressly subject 
// to the terms and conditions of the Altera Program License Subscription 
// Agreement, Altera MegaCore Function License Agreement, or other applicable 
// license agreement, including, without limitation, that your use is for the 
// sole purpose of programming logic devices manufactured by Altera and sold by 
// Altera or its authorized distributors.  Please refer to the applicable 
// agreement for further details.


// (C) 2001-2013 Altera Corporation. All rights reserved.
// Your use of Altera Corporation's design tools, logic functions and other 
// software and tools, and its AMPP partner logic functions, and any output 
// files any of the foregoing (including device programming or simulation 
// files), and any associated documentation or information are expressly subject 
// to the terms and conditions of the Altera Program License Subscription 
// Agreement, Altera MegaCore Function License Agreement, or other applicable 
// license agreement, including, without limitation, that your use is for the 
// sole purpose of programming logic devices manufactured by Altera and sold by 
// Altera or its authorized distributors.  Please refer to the applicable 
// agreement for further details.


// $Id: //acds/rel/13.0sp1/ip/merlin/altera_merlin_router/altera_merlin_router.sv.terp#1 $
// $Revision: #1 $
// $Date: 2013/03/07 $
// $Author: swbranch $

// -------------------------------------------------------
// Merlin Router
//
// Asserts the appropriate one-hot encoded channel based on 
// either (a) the address or (b) the dest id. The DECODER_TYPE
// parameter controls this behaviour. 0 means address decoder,
// 1 means dest id decoder.
//
// In the case of (a), it also sets the destination id.
// -------------------------------------------------------

`timescale 1 ns / 1 ns

module sopc_addr_router_001_default_decode
  #(
     parameter DEFAULT_CHANNEL = 1,
               DEFAULT_WR_CHANNEL = -1,
               DEFAULT_RD_CHANNEL = -1,
               DEFAULT_DESTID = 9 
   )
  (output [89 - 86 : 0] default_destination_id,
   output [13-1 : 0] default_wr_channel,
   output [13-1 : 0] default_rd_channel,
   output [13-1 : 0] default_src_channel
  );

  assign default_destination_id = 
    DEFAULT_DESTID[89 - 86 : 0];

  generate begin : default_decode
    if (DEFAULT_CHANNEL == -1) begin
      assign default_src_channel = '0;
    end
    else begin
      assign default_src_channel = 13'b1 << DEFAULT_CHANNEL;
    end
  end
  endgenerate

  generate begin : default_decode_rw
    if (DEFAULT_RD_CHANNEL == -1) begin
      assign default_wr_channel = '0;
      assign default_rd_channel = '0;
    end
    else begin
      assign default_wr_channel = 13'b1 << DEFAULT_WR_CHANNEL;
      assign default_rd_channel = 13'b1 << DEFAULT_RD_CHANNEL;
    end
  end
  endgenerate

endmodule


module sopc_addr_router_001
(
    // -------------------
    // Clock & Reset
    // -------------------
    input clk,
    input reset,

    // -------------------
    // Command Sink (Input)
    // -------------------
    input                       sink_valid,
    input  [100-1 : 0]    sink_data,
    input                       sink_startofpacket,
    input                       sink_endofpacket,
    output                      sink_ready,

    // -------------------
    // Command Source (Output)
    // -------------------
    output                          src_valid,
    output reg [100-1    : 0] src_data,
    output reg [13-1 : 0] src_channel,
    output                          src_startofpacket,
    output                          src_endofpacket,
    input                           src_ready
);

    // -------------------------------------------------------
    // Local parameters and variables
    // -------------------------------------------------------
    localparam PKT_ADDR_H = 60;
    localparam PKT_ADDR_L = 36;
    localparam PKT_DEST_ID_H = 89;
    localparam PKT_DEST_ID_L = 86;
    localparam PKT_PROTECTION_H = 93;
    localparam PKT_PROTECTION_L = 91;
    localparam ST_DATA_W = 100;
    localparam ST_CHANNEL_W = 13;
    localparam DECODER_TYPE = 0;

    localparam PKT_TRANS_WRITE = 63;
    localparam PKT_TRANS_READ  = 64;

    localparam PKT_ADDR_W = PKT_ADDR_H-PKT_ADDR_L + 1;
    localparam PKT_DEST_ID_W = PKT_DEST_ID_H-PKT_DEST_ID_L + 1;



    // -------------------------------------------------------
    // Figure out the number of bits to mask off for each slave span
    // during address decoding
    // -------------------------------------------------------
    localparam PAD0 = log2ceil(64'h1000000 - 64'h800000); 
    localparam PAD1 = log2ceil(64'h1001000 - 64'h1000800); 
    localparam PAD2 = log2ceil(64'h1001020 - 64'h1001000); 
    localparam PAD3 = log2ceil(64'h1001040 - 64'h1001020); 
    localparam PAD4 = log2ceil(64'h1001050 - 64'h1001040); 
    localparam PAD5 = log2ceil(64'h1001060 - 64'h1001050); 
    localparam PAD6 = log2ceil(64'h1001070 - 64'h1001060); 
    localparam PAD7 = log2ceil(64'h1001080 - 64'h1001070); 
    localparam PAD8 = log2ceil(64'h1001090 - 64'h1001080); 
    localparam PAD9 = log2ceil(64'h10010a0 - 64'h1001090); 
    localparam PAD10 = log2ceil(64'h10010b0 - 64'h10010a0); 
    localparam PAD11 = log2ceil(64'h10010c0 - 64'h10010b0); 
    localparam PAD12 = log2ceil(64'h10010c8 - 64'h10010c0); 
    // -------------------------------------------------------
    // Work out which address bits are significant based on the
    // address range of the slaves. If the required width is too
    // large or too small, we use the address field width instead.
    // -------------------------------------------------------
    localparam ADDR_RANGE = 64'h10010c8;
    localparam RANGE_ADDR_WIDTH = log2ceil(ADDR_RANGE);
    localparam OPTIMIZED_ADDR_H = (RANGE_ADDR_WIDTH > PKT_ADDR_W) ||
                                  (RANGE_ADDR_WIDTH == 0) ?
                                        PKT_ADDR_H :
                                        PKT_ADDR_L + RANGE_ADDR_WIDTH - 1;

    localparam RG = RANGE_ADDR_WIDTH-1;

      wire [PKT_ADDR_W-1 : 0] address = sink_data[OPTIMIZED_ADDR_H : PKT_ADDR_L];

    // -------------------------------------------------------
    // Pass almost everything through, untouched
    // -------------------------------------------------------
    assign sink_ready        = src_ready;
    assign src_valid         = sink_valid;
    assign src_startofpacket = sink_startofpacket;
    assign src_endofpacket   = sink_endofpacket;

    wire [PKT_DEST_ID_W-1:0] default_destid;
    wire [13-1 : 0] default_src_channel;





    sopc_addr_router_001_default_decode the_default_decode(
      .default_destination_id (default_destid),
      .default_wr_channel   (),
      .default_rd_channel   (),
      .default_src_channel  (default_src_channel)
    );

    always @* begin
        src_data    = sink_data;
        src_channel = default_src_channel;
        src_data[PKT_DEST_ID_H:PKT_DEST_ID_L] = default_destid;

        // --------------------------------------------------
        // Address Decoder
        // Sets the channel and destination ID based on the address
        // --------------------------------------------------

    // ( 0x800000 .. 0x1000000 )
    if ( {address[RG:PAD0],{PAD0{1'b0}}} == 25'h800000   ) begin
            src_channel = 13'b0000000000010;
            src_data[PKT_DEST_ID_H:PKT_DEST_ID_L] = 9;
    end

    // ( 0x1000800 .. 0x1001000 )
    if ( {address[RG:PAD1],{PAD1{1'b0}}} == 25'h1000800   ) begin
            src_channel = 13'b0000000000001;
            src_data[PKT_DEST_ID_H:PKT_DEST_ID_L] = 8;
    end

    // ( 0x1001000 .. 0x1001020 )
    if ( {address[RG:PAD2],{PAD2{1'b0}}} == 25'h1001000   ) begin
            src_channel = 13'b0000100000000;
            src_data[PKT_DEST_ID_H:PKT_DEST_ID_L] = 10;
    end

    // ( 0x1001020 .. 0x1001040 )
    if ( {address[RG:PAD3],{PAD3{1'b0}}} == 25'h1001020   ) begin
            src_channel = 13'b0000010000000;
            src_data[PKT_DEST_ID_H:PKT_DEST_ID_L] = 12;
    end

    // ( 0x1001040 .. 0x1001050 )
    if ( {address[RG:PAD4],{PAD4{1'b0}}} == 25'h1001040   ) begin
            src_channel = 13'b1000000000000;
            src_data[PKT_DEST_ID_H:PKT_DEST_ID_L] = 11;
    end

    // ( 0x1001050 .. 0x1001060 )
    if ( {address[RG:PAD5],{PAD5{1'b0}}} == 25'h1001050   ) begin
            src_channel = 13'b0100000000000;
            src_data[PKT_DEST_ID_H:PKT_DEST_ID_L] = 6;
    end

    // ( 0x1001060 .. 0x1001070 )
    if ( {address[RG:PAD6],{PAD6{1'b0}}} == 25'h1001060   ) begin
            src_channel = 13'b0010000000000;
            src_data[PKT_DEST_ID_H:PKT_DEST_ID_L] = 4;
    end

    // ( 0x1001070 .. 0x1001080 )
    if ( {address[RG:PAD7],{PAD7{1'b0}}} == 25'h1001070   ) begin
            src_channel = 13'b0001000000000;
            src_data[PKT_DEST_ID_H:PKT_DEST_ID_L] = 2;
    end

    // ( 0x1001080 .. 0x1001090 )
    if ( {address[RG:PAD8],{PAD8{1'b0}}} == 25'h1001080   ) begin
            src_channel = 13'b0000001000000;
            src_data[PKT_DEST_ID_H:PKT_DEST_ID_L] = 7;
    end

    // ( 0x1001090 .. 0x10010a0 )
    if ( {address[RG:PAD9],{PAD9{1'b0}}} == 25'h1001090   ) begin
            src_channel = 13'b0000000100000;
            src_data[PKT_DEST_ID_H:PKT_DEST_ID_L] = 5;
    end

    // ( 0x10010a0 .. 0x10010b0 )
    if ( {address[RG:PAD10],{PAD10{1'b0}}} == 25'h10010a0   ) begin
            src_channel = 13'b0000000010000;
            src_data[PKT_DEST_ID_H:PKT_DEST_ID_L] = 3;
    end

    // ( 0x10010b0 .. 0x10010c0 )
    if ( {address[RG:PAD11],{PAD11{1'b0}}} == 25'h10010b0   ) begin
            src_channel = 13'b0000000000100;
            src_data[PKT_DEST_ID_H:PKT_DEST_ID_L] = 0;
    end

    // ( 0x10010c0 .. 0x10010c8 )
    if ( {address[RG:PAD12],{PAD12{1'b0}}} == 25'h10010c0   ) begin
            src_channel = 13'b0000000001000;
            src_data[PKT_DEST_ID_H:PKT_DEST_ID_L] = 1;
    end

end


    // --------------------------------------------------
    // Ceil(log2()) function
    // --------------------------------------------------
    function integer log2ceil;
        input reg[65:0] val;
        reg [65:0] i;

        begin
            i = 1;
            log2ceil = 0;

            while (i < val) begin
                log2ceil = log2ceil + 1;
                i = i << 1;
            end
        end
    endfunction

endmodule


