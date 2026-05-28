-- =========================================================================
-- dcf77_axi_lite.vhd
--
-- Top-level wrapper: dcf77_core plus the AXI4-Lite slave (axi_readmux /
-- axi_writemux).  Exposes the conventional flat std_logic AXI4-Lite ports
-- at the entity boundary; internally re-aggregates them into the record
-- types defined by axi4lite_interface_pkg.
--
-- Register map (per dcf77_pkg):
--   0x00 RW   CTRL         enable, soft_reset, env_mode
--   0x04 R    STATUS       sync_state, time_valid, bit_index, RSSI
--   0x08 R    TIME_HMS     minute, hour
--   0x0C R    DATE         year, month, day, weekday
--   0x10 R    FLAGS        dst_active, dst_announce, leap_announce
--   0x14 R    RSSI         16-b magnitude (mirrors STATUS bits 31..16)
--   0x18 R    LAST_FRAME0  frame bits 0..31
--   0x1C R    LAST_FRAME1  frame bits 32..58
--   0x20 R    ENV_MIN      env_min top 16 b
--   0x24 R    NOISE        noise_iir top 16 b
--   0x28 R    SYNC_DIAG    sync_q / mm_q / cycle_in_min
--   0x2C R    SOFT_AM      am_soft + mm_soft + sec_index + valid + mm
--   0x30 R    SOFT_PM      placeholder (PSK correlator TBD)
--   0x34 R/W1C SOFT_IRQ    [0] second_tick (W1C), [1] pll_lock, [2] anchor_held
-- =========================================================================

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

use work.dcf77_pkg.all;
use work.axi4lite_interface_pkg.all;

entity dcf77_axi_lite is
    generic (
        -- Forward the bit_det timing into the embedded core so the host
        -- can override them at integration time for fast simulation.
        -- Defaults match the production 2 kS/s envelope tick rate (post-
        -- boxcar + 20 Hz IIR), not the legacy 100 S/s defaults.
        K_ATK         : positive := 8;
        K_DEC         : positive := 14;
        K_NOISE       : positive := 14;
        DERIV_DELAY   : positive := 16;
        K_SLOPE       : positive := 2;
        K_REF         : positive := 2;
        SAMPLE_RATE_HZ: positive := 2000;
        T_HOLD_SEC    : positive := 120;
        T_SEARCH_SEC  : positive := 300;
        NCO_PHASE_INCR: natural  := 10_399_018
    );
    port (
        -- AXI4-Lite slave (system clock = ACLK; matches dcf77_core's clk).
        s_axi_aclk    : in  std_logic;
        s_axi_aresetn : in  std_logic;

        s_axi_awaddr  : in  std_logic_vector(AXILITE_AXI_ADDR_WIDTH - 1 downto 0);
        s_axi_awprot  : in  std_logic_vector(2 downto 0);
        s_axi_awvalid : in  std_logic;
        s_axi_awready : out std_logic;

        s_axi_wdata   : in  std_logic_vector(AXILITE_AXI_DATA_WIDTH - 1 downto 0);
        s_axi_wstrb   : in  std_logic_vector((AXILITE_AXI_DATA_WIDTH / 8) - 1 downto 0);
        s_axi_wvalid  : in  std_logic;
        s_axi_wready  : out std_logic;

        s_axi_bresp   : out std_logic_vector(1 downto 0);
        s_axi_bvalid  : out std_logic;
        s_axi_bready  : in  std_logic;

        s_axi_araddr  : in  std_logic_vector(AXILITE_AXI_ADDR_WIDTH - 1 downto 0);
        s_axi_arprot  : in  std_logic_vector(2 downto 0);
        s_axi_arvalid : in  std_logic;
        s_axi_arready : out std_logic;

        s_axi_rdata   : out std_logic_vector(AXILITE_AXI_DATA_WIDTH - 1 downto 0);
        s_axi_rresp   : out std_logic_vector(1 downto 0);
        s_axi_rvalid  : out std_logic;
        s_axi_rready  : in  std_logic;

        -- ADC stream (separate from the AXI clock domain by convention; the
        -- core assumes they're synchronous to s_axi_aclk).
        adc_sample    : in  adc_sample_t;
        adc_valid     : in  std_logic
    );
end entity dcf77_axi_lite;

architecture rtl of dcf77_axi_lite is

    -- Record-typed AXI channel wires for the readmux / writemux.
    signal ar_in   : axi4lite_address_channel_input_type;
    signal ar_out  : axi4lite_address_channel_output_type;
    signal r_in    : axi4lite_read_data_channel_input_type;
    signal r_out   : axi4lite_read_data_channel_output_type;

    signal aw_in   : axi4lite_address_channel_input_type;
    signal aw_out  : axi4lite_address_channel_output_type;
    signal w_in    : axi4lite_write_data_channel_input_type;
    signal w_out   : axi4lite_write_data_channel_output_type;
    signal b_in    : axi4lite_write_response_channel_input_type;
    signal b_out   : axi4lite_write_response_channel_output_type;

    -- Core <-> AXI signals.
    signal control     : dcf77_control_type;
    signal status      : dcf77_status_type;

    -- Sticky second_tick (set on every bit_valid pulse from the core's
    -- bit_det, cleared when the host writes '1' to SOFT_IRQ[0]).
    signal soft_irq_q    : std_logic := '0';
    signal soft_irq_clr  : std_logic;     -- pulse from writemux

    -- Combined active-high reset for the core: external !aresetn OR the
    -- host-controllable soft_reset bit.
    signal core_rst    : std_logic;

    -- Host-gateable input-valid: drops to zero when CTRL.enable is cleared
    -- so the decimation chain stops advancing.  The core's state freezes
    -- but is not reset; the host can re-enable to resume.
    signal core_valid  : std_logic;

begin

    ------------------------------------------------------------------ --
    -- Flatten the AXI ports into the record types.                       --
    ------------------------------------------------------------------ --
    ar_in.valid <= s_axi_arvalid;
    ar_in.addr  <= s_axi_araddr;
    ar_in.prot  <= s_axi_arprot;
    s_axi_arready <= ar_out.ready;

    r_in.ready    <= s_axi_rready;
    s_axi_rvalid  <= r_out.valid;
    s_axi_rdata   <= r_out.data;
    s_axi_rresp   <= r_out.resp;

    aw_in.valid <= s_axi_awvalid;
    aw_in.addr  <= s_axi_awaddr;
    aw_in.prot  <= s_axi_awprot;
    s_axi_awready <= aw_out.ready;

    w_in.valid <= s_axi_wvalid;
    w_in.data  <= s_axi_wdata;
    w_in.strb  <= s_axi_wstrb;
    s_axi_wready <= w_out.ready;

    b_in.ready  <= s_axi_bready;
    s_axi_bvalid <= b_out.valid;
    s_axi_bresp  <= b_out.resp;

    ------------------------------------------------------------------ --
    -- Read-side decoder                                                  --
    ------------------------------------------------------------------ --
    u_readmux : entity work.axi_readmux
        port map (
            ACLK    => s_axi_aclk,
            ARESETN => s_axi_aresetn,

            ARInput  => ar_in,
            AROutput => ar_out,
            RInput   => r_in,
            ROutput  => r_out,

            FromStatus  => status,
            FromControl => control,
            FromSoftIrq => soft_irq_q
        );

    ------------------------------------------------------------------ --
    -- Write-side decoder                                                 --
    ------------------------------------------------------------------ --
    u_writemux : entity work.axi_writemux
        port map (
            ACLK    => s_axi_aclk,
            ARESETN => s_axi_aresetn,

            AWInput  => aw_in,
            AWOutput => aw_out,
            WInput   => w_in,
            WOutput  => w_out,
            BInput   => b_in,
            BOutput  => b_out,

            ToControl    => control,
            SoftIrqClear => soft_irq_clr
        );

    ------------------------------------------------------------------ --
    -- Combined core reset: external !aresetn OR host-issued soft_reset.  --
    -- (Both signals are already in the s_axi_aclk domain.)               --
    ------------------------------------------------------------------ --
    core_rst   <= (not s_axi_aresetn) or control.soft_reset;
    core_valid <= adc_valid and control.enable;

    ------------------------------------------------------------------ --
    -- Second-tick sticky bit.  Set on every am_soft_valid pulse from   --
    -- the core (i.e. on each end_of_bit).  Cleared by a host W1C       --
    -- write to SOFT_IRQ[0].  Read by the AXI readmux.                  --
    ------------------------------------------------------------------ --
    soft_irq_proc : process(s_axi_aclk)
    begin
        if rising_edge(s_axi_aclk) then
            if s_axi_aresetn = '0' then
                soft_irq_q <= '0';
            elsif soft_irq_clr = '1' then
                soft_irq_q <= '0';
            elsif status.am_soft_valid = '1' then
                soft_irq_q <= '1';
            end if;
        end if;
    end process;

    ------------------------------------------------------------------ --
    -- DCF77 receiver core                                                --
    ------------------------------------------------------------------ --
    u_core : entity work.dcf77_core
        generic map (
            K_ATK          => K_ATK,
            K_DEC          => K_DEC,
            K_NOISE        => K_NOISE,
            DERIV_DELAY    => DERIV_DELAY,
            K_SLOPE        => K_SLOPE,
            K_REF          => K_REF,
            SAMPLE_RATE_HZ => SAMPLE_RATE_HZ,
            T_HOLD_SEC     => T_HOLD_SEC,
            T_SEARCH_SEC   => T_SEARCH_SEC,
            NCO_PHASE_INCR => NCO_PHASE_INCR
        )
        port map (
            clk             => s_axi_aclk,
            rst             => core_rst,
            in_sample       => adc_sample,
            in_valid        => core_valid,
            env_mode        => control.env_mode,
            phase_err_out   => open,
            phase_err_valid => open,
            status_out      => status
        );

end architecture rtl;
