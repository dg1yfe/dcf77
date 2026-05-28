-- =========================================================================
-- axi_writemux.vhd
--
-- AXI4-Lite write-channel arbiter and decoder for the DCF77 slave.  Adapted
-- from doc/axi4lite_interface.md; bugs fixed and the memory-
-- controller leftovers stripped.  Decodes writes to REG_CTRL into a
-- dcf77_control_type record that drives the receiver core.  Writes to any
-- other offset return SLVERR (no other registers are writable).
--
-- Byte-strobe support: write data is byte-merged into the registered word
-- so partial-word writes (e.g. axi_wstrb = "0001") keep the other bytes
-- intact.
-- =========================================================================

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

use work.axi4lite_interface_pkg.all;
use work.dcf77_pkg.all;

entity axi_writemux is
    port (
        ACLK    : in  std_logic;
        ARESETN : in  std_logic;

        AWInput  : in  axi4lite_address_channel_input_type;
        AWOutput : out axi4lite_address_channel_output_type;

        WInput  : in  axi4lite_write_data_channel_input_type;
        WOutput : out axi4lite_write_data_channel_output_type;

        BInput  : in  axi4lite_write_response_channel_input_type;
        BOutput : out axi4lite_write_response_channel_output_type;

        ToControl : out dcf77_control_type;

        -- One-cycle pulse: AXI host wrote '1' to SOFT_IRQ[0].  The
        -- containing axi_lite block uses this to clear its sticky
        -- second_tick flop.
        SoftIrqClear : out std_logic
    );
end entity axi_writemux;

architecture rtl of axi_writemux is

    type wmux_reg_type is record
        awi : axi4lite_address_channel_input_type;
        awo : axi4lite_address_channel_output_type;
        wi  : axi4lite_write_data_channel_input_type;
        wo  : axi4lite_write_data_channel_output_type;
        bi  : axi4lite_write_response_channel_input_type;
        bo  : axi4lite_write_response_channel_output_type;
        ctrl: dcf77_control_type;
        soft_irq_clear : std_logic;       -- one-cycle pulse
    end record;

    constant AWI_RESET : axi4lite_address_channel_input_type := (
        valid => '0',
        addr  => (others => '0'),
        prot  => (others => '0')
    );
    constant AWO_RESET : axi4lite_address_channel_output_type := (ready => '0');
    constant WI_RESET  : axi4lite_write_data_channel_input_type := (
        valid => '0',
        data  => (others => '0'),
        strb  => (others => '0')
    );
    constant WO_RESET  : axi4lite_write_data_channel_output_type := (ready => '0');
    constant BI_RESET  : axi4lite_write_response_channel_input_type := (ready => '0');
    constant BO_RESET  : axi4lite_write_response_channel_output_type := (
        valid => '0',
        resp  => AXI_RESPONSE_OKAY
    );

    constant R_RESET : wmux_reg_type := (
        awi  => AWI_RESET,
        awo  => AWO_RESET,
        wi   => WI_RESET,
        wo   => WO_RESET,
        bi   => BI_RESET,
        bo   => BO_RESET,
        ctrl => DCF77_CONTROL_RESET,
        soft_irq_clear => '0'
    );

    signal r, rin : wmux_reg_type;

begin

    AWOutput  <= r.awo;
    WOutput   <= r.wo;
    BOutput   <= r.bo;
    ToControl <= r.ctrl;
    SoftIrqClear <= r.soft_irq_clear;

    comb : process (all)
        variable v       : wmux_reg_type;
        variable word_ix : integer range 0 to 63;
        variable merged  : std_logic_vector(31 downto 0);
    begin
        v := r;

        -- Pulsed register-bit defaults.  Following the pattern from the
        -- AXI example ("Reset pulsed signals to zero here"), default a
        -- self-clearing bit to '0' at the top of the comb process.  The
        -- write decoder below sets it to '1' only on the matching write
        -- transaction, so the registered output is high for exactly one
        -- clock.
        v.ctrl.soft_reset := '0';
        v.soft_irq_clear  := '0';

        -- Register incoming AXI signals.
        v.awi := AWInput;
        v.bi  := BInput;

        -- Byte-strobe merging of write data.
        if WInput.valid = '1' then
            merged := r.wi.data;
            for byte in 0 to 3 loop
                if WInput.strb(byte) = '1' then
                    merged((byte * 8 + 7) downto byte * 8)
                        := WInput.data((byte * 8 + 7) downto byte * 8);
                end if;
            end loop;
            v.wi.data  := merged;
            v.wi.strb  := WInput.strb;
            v.wi.valid := '1';
        else
            v.wi.valid := '0';
        end if;

        -- AW handshake: wait for write address AND write data both valid.
        v.awo.ready := '0';
        if r.awi.valid = '1' and r.wi.valid = '1' then
            v.awo.ready := '1';
        end if;
        if r.awi.valid = '1' and r.awo.ready = '1' then
            v.awo.ready := '0';
        end if;

        -- W handshake: assert WREADY once we have an address.
        v.wo.ready := '0';
        if r.wi.valid = '1' and r.awi.valid = '1' then
            v.wo.ready := '1';
        end if;

        -- Decode and apply the write on the cycle that closes the W handshake.
        if r.wi.valid = '1' and r.awo.ready = '1' then
            word_ix := to_integer(unsigned(r.awi.addr(7 downto 2)));
            case word_ix is
                when REG_CTRL =>
                    -- Field-by-field update so persistent and pulsed bits
                    -- can be handled differently.  enable, env_mode
                    -- sticky; soft_reset is a one-cycle pulse cleared
                    -- by the default above.
                    v.ctrl.enable     := r.wi.data(0);
                    v.ctrl.soft_reset := r.wi.data(1);
                    v.ctrl.env_mode   := r.wi.data(2);
                    v.bo.resp := AXI_RESPONSE_OKAY;
                when REG_SOFT_IRQ =>
                    -- W1C: a write of '1' to bit 0 clears the sticky
                    -- second_tick in the containing axi_lite block.
                    if r.wi.data(0) = '1' then
                        v.soft_irq_clear := '1';
                    end if;
                    v.bo.resp := AXI_RESPONSE_OKAY;
                when others =>
                    v.bo.resp := AXI_RESPONSE_SLVERR;
            end case;
            v.bo.valid := '1';
        end if;

        -- Hold BVALID until master accepts it.
        if r.bo.valid = '1' and r.bi.ready = '1' then
            v.bo.valid := '0';
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
