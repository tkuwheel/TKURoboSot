//
//
// Major Functions: Shoot control
//
// --------------------------------------------------------------------
//
// Revision History :
// --------------------------------------------------------------------
//   Ver  :| Author            :| Mod. Date  :|  Changes Made:
//   1.8  :| Chun-Jui Huang    :| 2017/07/07 :|  Shoot Control
// --------------------------------------------------------------------
`default_nettype  none
module ShootControl (
input				iClk,
input				iRst_n,
input		[7:0]	iPower,
output	reg			oPower	
);


reg				rClk_Shoot;				//shoot clock.
reg				rShoot_EN;
reg				rShootFinish;

reg		[31:0]	rCount_ShootWaitConst;	//wiat counter
reg		[15:0]	rCount_ShootConst;		//shoot counter
reg		[7:0]	rShoot_power;
reg		[7:0]	rPower;


parameter SHOOTCONST = 5000;	//Shoot Time Const => 0.2 ms
parameter WAITSHOOTCONST = 250000000;	//Shoot Wait Time Const => (50M * 1) => 5s 

wire			wClk_Shoot;
always@(posedge iClk)begin	// 50MHz -> ShootConst
	if(!iRst_n)begin
		rCount_ShootConst <= 0;
		rClk_Shoot <= 0;
	end
	else begin
		if(rCount_ShootConst < SHOOTCONST) begin
			rCount_ShootConst <= rCount_ShootConst+1;
		end
		else begin
			rCount_ShootConst <= 0;
			rClk_Shoot <= ~rClk_Shoot;
		end
	end
end

assign wClk_Shoot = rClk_Shoot;

always@(posedge iClk)begin	// 50MHz -> 50Mhz/(2*WAITSHOOTCONST) CLOCK
	if(!iRst_n)begin
		rCount_ShootWaitConst 	<= 0;
		rShoot_EN				<= 0;
	end
	else begin
		if(rShootFinish)begin
			if(rCount_ShootWaitConst < WAITSHOOTCONST) begin
				rCount_ShootWaitConst <= rCount_ShootWaitConst+1;
				rShoot_EN <= 0;
			end
			else begin
				rCount_ShootWaitConst <= rCount_ShootWaitConst;
				rShoot_EN <= 1;
			end
		end
		else begin
			rCount_ShootWaitConst 	<= 0;
			rShoot_EN <= 1;
			
		end
	end
end

always@(posedge wClk_Shoot or negedge iRst_n)begin	// control shoot power
	if(!iRst_n)begin
		rShoot_power <= 0;
		oPower 		 <= 0;
		rShootFinish <= 0;
		rPower		 <= 0;
	end
	else begin
		rPower <= iPower;
		if(rShoot_EN && rPower)begin
			if(rShoot_power < rPower)begin
				rShoot_power <= rShoot_power+1;
				oPower <= 1;
				rShootFinish <= 0;
			end
			else begin
				oPower <= 0;
				rShoot_power <= rShoot_power;
				rShootFinish <= 1;
			end
		end
		else begin
			oPower <= 0;
			rShoot_power <= 0;
			rShootFinish <= 1;
		end
	end
end
endmodule