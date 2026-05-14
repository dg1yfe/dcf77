-- =========================================================================
-- axi_readmux.vhd
--
-- AXI4-Lite read-channel arbiter and address decoder for the DCF77 slave.
-- Adapted from doc/axi4lite_interface.md; bugs fixed and the
-- dcf77 register map (per dcf77_pkg) wired in.
--
-- Inputs come from the dcf77 core (status, time, last_frame) and from the
-- writemux (current control value, so the host can read it back).
-- =========================================================================

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

use work.axi4lite_interface_pkg.all;
use work.dcf77_pkg.all;

entity axi_readmux is
    port (
        ACLK    : in  std_logic;
        ARESETN : in  std_logic;

        ARInput  : in  axi4lite_address_channel_input_type;
        AROutput : out axi4lite_address_channel_output_type;

        RInput  : in  axi4lite_read_data_channel_input_type;
        ROutput : out axi4lite_read_data_channel_output_type;

        -- Only two data sources feed the readmux: the comprehensive
        -- decoder status, and the host-writable control (echoed back from
        -- the writemux so the host can read what it wrote).
        FromStatus  : in dcf77_status_type;
        FromControl : in dcf77_control_type
    );
end entity axi_readmux;

architecture rtl of axi_readmux is

    type rmux_reg_type is record
        ari : axi4lite_address_channel_input_type;
        aro : axi4lite_address_channel_output_type;
        ri  : axi4lite_read_data_channel_input_type;
        ro  : axi4lite_read_data_channel_output_type;
        st  : dcf77_status_type;
        ct  : dcf77_control_type;
    end record;

    constant ARI_RESET : axi4lite_address_channel_input_type := (
        valid => '0',
        addr  => (others => '0'),
        prot  => (others => '0')
    );
    constant ARO_RESET : axi4lite_address_channel_output_type := (ready => '1');
    constant RI_RESET  : axi4lite_read_data_channel_input_type := (ready => '0');
    constant RO_RESET  : axi4lite_read_data_channel_output_type := (
        valid => '0',
        data  => (others => '0'),
        resp  => AXI_RESPONSE_SLVERR
    );

    constant R_RESET : rmux_reg_type := (
        ari => ARI_RESET,
        aro => ARO_RESET,
        ri  => RI_RESET,
        ro  => RO_RESET,
        st  => DCF77_STATUS_RESET,
        ct  => DCF77_CONTROL_RESET
    );

    signal r, rin : rmux_reg_type;

begin

    AROutput <= r.aro;
    ROutput  <= r.ro;

    comb : process (all)
        variable v       : rmux_reg_type;
        variable word_ix : integer range 0 to 63;
    begin
        v := r;

        -- Register incoming AXI signals.
        v.ari := ARInput;
        v.ri  := RInput;

        -- Snapshot the input records on every cycle so they're stable when
        -- we read them out (the readmux's two-process pipeline guarantees the
        -- decoder fires on r.* values which were captured on the previous
        -- clock).
        v.st := FromStatus;
        v.ct := FromControl;

        -- AR handshake: ready stays low until we see a valid address and
        -- nothing else is outstanding on the read-data channel.
        v.aro.ready := '0';
        if r.ari.valid = '1' and r.ro.valid = '0' then
            v.aro.ready := '1';
        end if;
        if r.ari.valid = '1' and r.aro.ready = '1' then
            v.aro.ready := '0';
        end if;

        -- Address decode + read-data presentation.  Fires one cycle after
        -- the AR handshake, when both r.ari.valid and r.aro.ready are high.
        if (r.ari.valid = '1' and r.aro.ready = '1') or r.ro.valid = '1' then
            word_ix := to_integer(unsigned(r.ari.addr(7 downto 2)));
            v.ro.valid := '1';
            v.ro.resp  := AXI_RESPONSE_OKAY;

            case word_ix is
                when REG_CTRL        => v.ro.data := ctrl_to_slv(r.ct);
                when REG_STATUS      => v.ro.data := status_to_slv(r.st);
                when REG_TIME_HMS    => v.ro.data := time_hms_to_slv(r.st);
                when REG_DATE        => v.ro.data := date_to_slv(r.st);
                when REG_FLAGS       => v.ro.data := flags_to_slv(r.st);
                when REG_RSSI        =>
                    v.ro.data := (31 downto 16 => std_logic_vector(r.st.rssi),
                                  others => '0');
                when REG_LAST_FRAME0 => v.ro.data := last_frame0_to_slv(r.st);
                when REG_LAST_FRAME1 => v.ro.data := last_frame1_to_slv(r.st);
                when REG_ENV_MIN =>
                    v.ro.data := (31 downto 16 => '0',
                                  others => '0');
                    v.ro.data(15 downto 0) := std_logic_vector(r.st.env_min);
                when REG_NOISE =>
                    v.ro.data := (31 downto 16 => '0',
                                  others => '0');
                    v.ro.data(15 downto 0) := std_logic_vector(r.st.noise);
                when others =>
                    v.ro.data := (others => '0');
                    v.ro.resp := AXI_RESPONSE_DECERR;
            end case;
        end if;

        -- Drop RVALID once the master has accepted the response.
        if r.ri.ready = '1' and r.ro.valid = '1' then
            v.ro.valid := '0';
        end if;

        if ARESETN = '0' then
            v := R_RESET;
        end if;

        rin <= v;
    end process;

    seq : process (ACLK)
    begin
        if rising_edge(ACLK) then
            r <= rin;
        end if;
    end process;

end architecture rtl;
