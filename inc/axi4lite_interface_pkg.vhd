-- =========================================================================
-- axi4lite_interface_pkg.vhd
--
-- Shared AXI4-Lite channel record types for the DCF77 slave.  See
-- doc/axi4lite_interface.md for the interface structure and coding style.
-- =========================================================================

library ieee;
use ieee.std_logic_1164.all;

package axi4lite_interface_pkg is

    --A3.2.1          Handshake process
    -- All five transaction channels use the same VALID / READY
    -- handshake process to transfer address, data, and control information.
    -- The source generates VALID when the data is available; the destination
    -- generates READY when it can accept it; transfer occurs when both are
    -- HIGH at a rising clock edge.  A source must not wait for READY before
    -- asserting VALID, and VALID must remain asserted until the handshake
    -- completes.

    constant AXILITE_AXI_DATA_WIDTH : integer := 32;
    constant AXILITE_AXI_ADDR_WIDTH : integer := 8;

    constant AXI_RESPONSE_OKAY   : std_logic_vector(1 downto 0) := "00";
    -- AXI_RESPONSE_EXOKAY "01" is not supported by AXI4-Lite.
    constant AXI_RESPONSE_SLVERR : std_logic_vector(1 downto 0) := "10";
    constant AXI_RESPONSE_DECERR : std_logic_vector(1 downto 0) := "11";

    type axi4lite_global_input_type is record
        aclk    : std_logic;            -- Global clock
        aresetn : std_logic;            -- Global active-low reset
    end record;

    type axi4lite_address_channel_input_type is record
        valid : std_logic;
        addr  : std_logic_vector(AXILITE_AXI_ADDR_WIDTH - 1 downto 0);
        prot  : std_logic_vector(2 downto 0);
    end record;

    type axi4lite_address_channel_output_type is record
        ready : std_logic;
    end record;

    type axi4lite_write_data_channel_input_type is record
        valid : std_logic;
        data  : std_logic_vector(AXILITE_AXI_DATA_WIDTH - 1 downto 0);
        strb  : std_logic_vector((AXILITE_AXI_DATA_WIDTH / 8) - 1 downto 0);
    end record;

    type axi4lite_write_data_channel_output_type is record
        ready : std_logic;
    end record;

    type axi4lite_write_response_channel_input_type is record
        ready : std_logic;
    end record;

    type axi4lite_write_response_channel_output_type is record
        valid : std_logic;
        resp  : std_logic_vector(1 downto 0);
    end record;

    type axi4lite_read_data_channel_input_type is record
        ready : std_logic;
    end record;

    type axi4lite_read_data_channel_output_type is record
        valid : std_logic;
        data  : std_logic_vector(AXILITE_AXI_DATA_WIDTH - 1 downto 0);
        resp  : std_logic_vector(1 downto 0);
    end record;

    -- Lean slave-side bundles WITHOUT the global (clock / reset)
    -- fields: per the two-process design guide, clocks and resets stay
    -- scalar entity ports (a clock inside a record port picks up a
    -- delta delay and confuses clock-tree / timing tools).  These are
    -- the port types of dcf77_axi_lite; flattening to the s_axi_*
    -- signals Vivado IP Integrator expects happens in dcf77_top_bd.
    type axi4lite_si_type is record    -- master -> slave
        aw : axi4lite_address_channel_input_type;
        w  : axi4lite_write_data_channel_input_type;
        b  : axi4lite_write_response_channel_input_type;
        ar : axi4lite_address_channel_input_type;
        r  : axi4lite_read_data_channel_input_type;
    end record;

    type axi4lite_so_type is record    -- slave -> master
        aw : axi4lite_address_channel_output_type;
        w  : axi4lite_write_data_channel_output_type;
        b  : axi4lite_write_response_channel_output_type;
        ar : axi4lite_address_channel_output_type;
        r  : axi4lite_read_data_channel_output_type;
    end record;

    type axi4lite_slave_input_type is record
        global : axi4lite_global_input_type;
        aw_in  : axi4lite_address_channel_input_type;
        w_in   : axi4lite_write_data_channel_input_type;
        b_in   : axi4lite_write_response_channel_input_type;
        ar_in  : axi4lite_address_channel_input_type;
        r_in   : axi4lite_read_data_channel_input_type;
    end record;

    type axi4lite_slave_output_type is record
        aw_out : axi4lite_address_channel_output_type;
        w_out  : axi4lite_write_data_channel_output_type;
        b_out  : axi4lite_write_response_channel_output_type;
        ar_out : axi4lite_address_channel_output_type;
        r_out  : axi4lite_read_data_channel_output_type;
    end record;

    type axi4lite_slave_type is record
        toSlave   : axi4lite_slave_input_type;
        fromSlave : axi4lite_slave_output_type;
    end record;

    type axi4lite_master_output_type is record
        aw : axi4lite_address_channel_input_type;
        w  : axi4lite_write_data_channel_input_type;
        b  : axi4lite_write_response_channel_input_type;
        ar : axi4lite_address_channel_input_type;
        r  : axi4lite_read_data_channel_input_type;
    end record;

    type axi4lite_master_input_type is record
        global : axi4lite_global_input_type;
        aw     : axi4lite_address_channel_output_type;
        w      : axi4lite_write_data_channel_output_type;
        b      : axi4lite_write_response_channel_output_type;
        ar     : axi4lite_address_channel_output_type;
        r      : axi4lite_read_data_channel_output_type;
    end record;

end package axi4lite_interface_pkg;
