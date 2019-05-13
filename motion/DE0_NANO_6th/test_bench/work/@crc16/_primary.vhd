library verilog;
use verilog.vl_types.all;
entity Crc16 is
    generic(
        PACKAGE_SIZE    : integer := 1;
        STREAM_SIZE     : integer := 8
    );
    port(
        iClk            : in     vl_logic;
        iRst_n          : in     vl_logic;
        iDataValid      : in     vl_logic;
        iData           : in     vl_logic_vector;
        oCrc            : out    vl_logic_vector(15 downto 0);
        oSuccess        : out    vl_logic;
        oFinish         : out    vl_logic
    );
    attribute mti_svvh_generic_type : integer;
    attribute mti_svvh_generic_type of PACKAGE_SIZE : constant is 1;
    attribute mti_svvh_generic_type of STREAM_SIZE : constant is 1;
end Crc16;
