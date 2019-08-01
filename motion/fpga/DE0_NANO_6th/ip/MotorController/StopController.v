module StopController(
input iClk,
input iRst_n,
output oDuty
);

always @(posedge wClk)begin
    if(!iRst_n)
        oDuty <= 0;
    else begin
        oDuty <= 0;
    end
end

endmodule