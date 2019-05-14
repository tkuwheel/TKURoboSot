// --------------------------------------------------------------------
// Copyright (c) 2012 by inteligent Control of Tamkang University. 
// --------------------------------------------------------------------
//
// Major Functions:	Reset_Delay 
//            
// --------------------------------------------------------------------
//
// Revision History :
// --------------------------------------------------------------------
//   Ver  :| Author            		:| Mod. Date  :| Changes Made:
//   V1.0 :| Shih An Li   				:| 2012/04/26 :| Initial Revision
// --------------------------------------------------------------------

module Reset_Delay( 
                     iClk_50M,
                     iRst_n,
                     oRST1ms_n,
                     oRST2ms_n,
                     oRST3ms_n
                     
                    );
                    
parameter CLKFreq = 50000000;
parameter Time1ms = CLKFreq/1000;
parameter Time2ms = Time1ms * 2;
parameter Time3ms = Time1ms * 3;
                    
input       iClk_50M;
input       iRst_n;


output reg  oRST1ms_n;
output reg  oRST2ms_n;
output reg  oRST3ms_n;

reg [31:0] rCont;

always@(posedge iClk_50M or negedge iRst_n) begin
    if(!iRst_n) begin
        rCont    <= 0;
        oRST1ms_n  <=  0;
        oRST2ms_n  <=  0;
        oRST3ms_n  <=  0;
    end
    else begin
        rCont <= rCont;
        oRST1ms_n  <=  oRST1ms_n;
        oRST2ms_n  <=  oRST2ms_n;
        oRST3ms_n  <=  oRST3ms_n;
        if(rCont <= Time3ms)
            rCont <= rCont+1;
        if(rCont >= Time1ms )
            oRST1ms_n <= 1;
        if(rCont >= Time2ms)
            oRST2ms_n <= 1;
        if(rCont >= Time3ms)  // about 0.3s
            oRST3ms_n <= 1;
    end
end

endmodule
