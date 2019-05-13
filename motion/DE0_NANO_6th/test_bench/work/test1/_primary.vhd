library verilog;
use verilog.vl_types.all;
entity test1 is
    port(
        iA              : in     vl_logic_vector(3 downto 0);
        iB              : in     vl_logic_vector(3 downto 0);
        oSum            : out    vl_logic_vector(3 downto 0);
        oCarry          : out    vl_logic
    );
end test1;
