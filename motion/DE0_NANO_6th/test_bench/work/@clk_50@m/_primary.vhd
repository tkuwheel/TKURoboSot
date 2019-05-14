library verilog;
use verilog.vl_types.all;
entity Clk_50M is
    generic(
        clkper          : integer := 20
    );
    port(
        clk             : out    vl_logic
    );
    attribute mti_svvh_generic_type : integer;
    attribute mti_svvh_generic_type of clkper : constant is 1;
end Clk_50M;
