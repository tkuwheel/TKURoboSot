library verilog;
use verilog.vl_types.all;
entity Packet2CMD is
    port(
        iClk            : in     vl_logic;
        iRst_n          : in     vl_logic;
        iDataValid      : in     vl_logic;
        iPacket         : in     vl_logic_vector(71 downto 0);
        oMotor1         : out    vl_logic_vector(7 downto 0);
        oMotor2         : out    vl_logic_vector(7 downto 0);
        oMotor3         : out    vl_logic_vector(7 downto 0);
        oEN             : out    vl_logic_vector(7 downto 0);
        oShoot          : out    vl_logic_vector(7 downto 0)
    );
end Packet2CMD;
