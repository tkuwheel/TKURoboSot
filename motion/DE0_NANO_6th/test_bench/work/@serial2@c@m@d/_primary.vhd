library verilog;
use verilog.vl_types.all;
entity Serial2CMD is
    generic(
        DATA0           : integer := 0;
        DATA1           : integer := 1;
        DATA2           : integer := 2;
        DATA3           : integer := 3;
        DATA4           : integer := 4;
        DATA5           : integer := 5;
        DATA6           : integer := 6;
        DATA7           : integer := 7;
        DATA8           : integer := 8;
        \END\           : vl_logic_vector(0 to 7) := (Hi1, Hi1, Hi1, Hi1, Hi1, Hi1, Hi1, Hi1)
    );
    port(
        iCLK            : in     vl_logic;
        iRst_n          : in     vl_logic;
        iRx_ready       : in     vl_logic;
        iData           : in     vl_logic_vector(7 downto 0);
        oCMD_Motor1     : out    vl_logic_vector(7 downto 0);
        oCMD_Motor2     : out    vl_logic_vector(7 downto 0);
        oCMD_Motor3     : out    vl_logic_vector(7 downto 0);
        oSignal         : out    vl_logic_vector(7 downto 0);
        oKick           : out    vl_logic_vector(7 downto 0);
        oRx_done        : out    vl_logic;
        oCrc            : out    vl_logic_vector(15 downto 0);
        oCrcSuccess     : out    vl_logic;
        debug           : out    vl_logic_vector(7 downto 0)
    );
    attribute mti_svvh_generic_type : integer;
    attribute mti_svvh_generic_type of DATA0 : constant is 1;
    attribute mti_svvh_generic_type of DATA1 : constant is 1;
    attribute mti_svvh_generic_type of DATA2 : constant is 1;
    attribute mti_svvh_generic_type of DATA3 : constant is 1;
    attribute mti_svvh_generic_type of DATA4 : constant is 1;
    attribute mti_svvh_generic_type of DATA5 : constant is 1;
    attribute mti_svvh_generic_type of DATA6 : constant is 1;
    attribute mti_svvh_generic_type of DATA7 : constant is 1;
    attribute mti_svvh_generic_type of DATA8 : constant is 1;
    attribute mti_svvh_generic_type of \END\ : constant is 1;
end Serial2CMD;
