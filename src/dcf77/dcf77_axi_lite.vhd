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
--   0x04 R    STATUS       sync_state, time_valid, pll_lock, anchor_held, bit_index, RSSI
--   0x08 R    TIME_HMS     minute, hour     -- read latches SHADOW = DATE
--   0x0C R    DATE         year/month/day/weekday + DST flags packed in bits 29..27
--   0x10                   reserved (was FLAGS; flags now packed into DATE)
--   0x14 R    RSSI         16-b magnitude (mirrors STATUS bits 31..16)
--   0x18 R    LAST_FRAME0  frame bits 0..31
--   0x1C R    LAST_FRAME1  frame bits 32..58
--   0x20 R    ENV_MIN      env_min top 16 b
--   0x24 R    NOISE        noise_iir top 16 b
--   0x28 R    SYNC_DIAG    sync_q / mm_q / cycle_in_min
--   0x2C R    SOFT_AM         am_soft + mm_soft + sec_index + valid + mm
--   0x30 R    SOFT_PM         pm_soft + sec_index + valid + mm    -- read latches SHADOW = SOFT_AM
--   0x34 R/W1C IRQ_FLAG       per-source sticky flags (see IRQ_*_BIT in dcf77_pkg)
--   0x38 R    PM_STATUS       pm_bit_index + sync_state + time_valid + lock_qual
--   0x3C R    PM_TIME_HMS     PM-decoded HH:MM (layout = TIME_HMS) -- read latches SHADOW = PM_DATE
--   0x40 R    PM_DATE         PM-decoded date + flags (layout = DATE)
--   0x44                   reserved (was PM_FLAGS; flags now packed into PM_DATE)
--   0x48 R    PM_LAST_FRAME0  PM frame bits 0..31
--   0x4C R    PM_LAST_FRAME1  PM frame bits 32..58
-- The 0x30, 0x38-0x4C block is gated by WITH_PM (reads return DECERR
-- when false).  REG_STATUS bit 3 (PM_PRESENT) signals availability
-- to software at runtime.
--   0x50 R    PLL_PHASE_FILT  carrier-PLL LPF state, snapshot, signed 29 b
--   0x54 R/W  PLL_I_STATE     PI integrator >> I_FRAC, signed 28 b
--                             R = snapshot (sampled on pm_valid)
--                             W = jam integrator with bits[27:0]
--   0x58 R/W  PLL_OFFSET      NCO phase_inc_offset, signed 24 b
--                             R = live offset
--                             W = jam offset with bits[23:0]
--   0x5C R/W  IRQ_MASK        per-source enable for the `irq` output
--                             (bit positions match IRQ_FLAG one-for-one)
--   0x60 R    SHADOW          shared atomic-pair shadow.  Latched from
--                             dcf77_status_type on the same readmux
--                             comb cycle that a "primary" register is
--                             read.  Triggers:
--                                * SOFT_PM      → SHADOW ← SOFT_AM
--                                * TIME_HMS     → SHADOW ← DATE
--                                * PM_TIME_HMS  → SHADOW ← PM_DATE
--                             Lets the host obtain self-consistent
--                             AM+PM LLR pairs / TIME+DATE+flags
--                             snapshots without race retries.
--
-- Interrupt: a level-sensitive `irq` output goes high while any
-- unmasked IRQ_FLAG bit is set (OR(IRQ_FLAG & IRQ_MASK)).  Hardware
-- edge-detects nine source signals and latches the rising edges into
-- IRQ_FLAG until the host W1Cs them.  Reset value of IRQ_MASK is 0,
-- so the host opts in by writing each desired source.
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
        NCO_PHASE_INCR: natural  := 10_399_018;

        -- Synthesis switches.
        --
        -- WITH_PM = false drops the entire PM decoder stack:
        --   * dcf77_pm_correlator is not instantiated (saves 1 BRAM
        --     tile + ~900 flops + ~600 LUTs).
        --   * Second dcf77_frame_dec (PM_MODE=true) is not
        --     instantiated.
        --   * AXI registers 0x30, 0x38-0x4C return DECERR.
        --   * REG_STATUS bit 3 (PM_PRESENT) reads 0.
        --   * Receiver still does full AM decode (time/date/parity
        --     via the bit_det path).
        --
        -- WITH_PM_DEBUG = false drops the carrier-PLL telemetry only
        -- (already documented above).  Has no effect when WITH_PM is
        -- already false -- the PLL registers (0x50-0x58) DECERR
        -- either way, but the debug bit reads 0 so software can
        -- distinguish "no telemetry" from "no PM stack".
        --
        -- Defaults: WITH_PM=true (project's primary purpose),
        -- WITH_PM_DEBUG=false (production-area defaults).
        WITH_PM       : boolean  := true;
        WITH_PM_DEBUG : boolean  := false
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
        adc_valid     : in  std_logic;

        -- Level-sensitive interrupt output.  High while any unmasked
        -- IRQ_FLAG bit is set; cleared when the host W1Cs the flag(s).
        -- Wire to your CPU's interrupt controller; if not used, leave
        -- open and synthesis will prune the unused logic.
        irq           : out std_logic
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
    signal control      : dcf77_control_type;
    signal pll_override : dcf77_pll_override_type;
    signal status       : dcf77_status_type;

    -- IRQ aggregator state.  irq_flag_q holds the 32-bit sticky flag
    -- register that the host reads via REG_IRQ_FLAG.  Bits are set on
    -- the rising edge of each source signal and cleared by the W1C
    -- pulse from the writemux.  Reset to all-zero.
    signal irq_flag_q   : std_logic_vector(31 downto 0) := (others => '0');
    signal irq_flag_clr : std_logic_vector(31 downto 0); -- per-bit pulse from writemux

    -- Edge-detect: previous-cycle samples of each source signal.  The
    -- comb logic compares (source AND NOT prev) to obtain a one-clock
    -- pulse per rising edge.
    signal prev_am_soft_valid : std_logic := '0';
    signal prev_minute_mark   : std_logic := '0';
    signal prev_pm_minute_mark: std_logic := '0';
    signal prev_time_valid    : std_logic := '0';
    signal prev_pm_time_valid : std_logic := '0';
    signal prev_sync_state    : sync_state_t := sync_search;
    signal prev_pm_sync_lock  : std_logic := '0';    -- 1 while pm_sync_state = "10"
    signal prev_pll_lock      : std_logic := '0';

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
        generic map (
            WITH_PM       => WITH_PM,
            WITH_PM_DEBUG => WITH_PM_DEBUG
        )
        port map (
            ACLK    => s_axi_aclk,
            ARESETN => s_axi_aresetn,

            ARInput  => ar_in,
            AROutput => ar_out,
            RInput   => r_in,
            ROutput  => r_out,

            FromStatus  => status,
            FromControl => control,
            FromIrqFlag => irq_flag_q
        );

    ------------------------------------------------------------------ --
    -- Write-side decoder                                                 --
    ------------------------------------------------------------------ --
    u_writemux : entity work.axi_writemux
        generic map (
            WITH_PM_DEBUG => WITH_PM_DEBUG
        )
        port map (
            ACLK    => s_axi_aclk,
            ARESETN => s_axi_aresetn,

            AWInput  => aw_in,
            AWOutput => aw_out,
            WInput   => w_in,
            WOutput  => w_out,
            BInput   => b_in,
            BOutput  => b_out,

            ToControl     => control,
            ToPllOverride => pll_override,
            IrqFlagClear  => irq_flag_clr
        );

    ------------------------------------------------------------------ --
    -- Combined core reset: external !aresetn OR host-issued soft_reset.  --
    -- (Both signals are already in the s_axi_aclk domain.)               --
    ------------------------------------------------------------------ --
    core_rst   <= (not s_axi_aresetn) or control.soft_reset;
    core_valid <= adc_valid and control.enable;

    ------------------------------------------------------------------ --
    -- Interrupt-flag aggregator.                                       --
    --                                                                  --
    -- Edge-detects nine source signals from the core's status record   --
    -- and sets the matching sticky bit on each rising edge.  Bits stay --
    -- set until the host W1Cs them via REG_IRQ_FLAG.  The `irq` output --
    -- is level-sensitive: high while any unmasked flag is set.         --
    --                                                                  --
    -- Reset behaviour: all sources sampled as '0' / sync_search, so    --
    -- the first non-reset cycle does not generate spurious edges.      --
    ------------------------------------------------------------------ --
    irq_proc : process(s_axi_aclk)
        variable v_irq_flag        : std_logic_vector(31 downto 0);
        variable e_second          : std_logic;
        variable e_am_mm           : std_logic;
        variable e_pm_mm           : std_logic;
        variable e_am_time_valid   : std_logic;
        variable e_pm_time_valid   : std_logic;
        variable e_am_sync_lost    : std_logic;
        variable cur_pm_sync_lock  : std_logic;
        variable e_pm_sync_lost    : std_logic;
        variable e_pll_lock_rise   : std_logic;
        variable e_pll_lock_fall   : std_logic;
    begin
        if rising_edge(s_axi_aclk) then
            if s_axi_aresetn = '0' then
                irq_flag_q          <= (others => '0');
                prev_am_soft_valid  <= '0';
                prev_minute_mark    <= '0';
                prev_pm_minute_mark <= '0';
                prev_time_valid     <= '0';
                prev_pm_time_valid  <= '0';
                prev_sync_state     <= sync_search;
                prev_pm_sync_lock   <= '0';
                prev_pll_lock       <= '0';
            else
                -- Rising-edge detect on each source.
                e_second        := status.am_soft_valid and not prev_am_soft_valid;
                e_am_mm         := status.minute_mark    and not prev_minute_mark;
                e_pm_mm         := status.pm_minute_mark and not prev_pm_minute_mark;
                e_am_time_valid := status.time_valid     and not prev_time_valid;
                e_pm_time_valid := status.pm_time_valid  and not prev_pm_time_valid;

                -- AM "sync lost" = was LOCK last cycle, no longer LOCK.
                if prev_sync_state = sync_lock and status.sync_state /= sync_lock then
                    e_am_sync_lost := '1';
                else
                    e_am_sync_lost := '0';
                end if;

                -- PM "sync lost" = was LOCK ("10") last cycle, no longer LOCK.
                if status.pm_sync_state = "10" then
                    cur_pm_sync_lock := '1';
                else
                    cur_pm_sync_lock := '0';
                end if;
                e_pm_sync_lost := prev_pm_sync_lock and not cur_pm_sync_lock;

                e_pll_lock_rise := status.pll_lock and not prev_pll_lock;
                e_pll_lock_fall := prev_pll_lock   and not status.pll_lock;

                -- Build the per-bit "set" vector for this cycle.
                v_irq_flag := (others => '0');
                v_irq_flag(IRQ_SECOND_TICK_BIT)    := e_second;
                v_irq_flag(IRQ_AM_MINUTE_MARK_BIT) := e_am_mm;
                v_irq_flag(IRQ_PM_MINUTE_MARK_BIT) := e_pm_mm;
                v_irq_flag(IRQ_AM_TIME_VALID_BIT)  := e_am_time_valid;
                v_irq_flag(IRQ_PM_TIME_VALID_BIT)  := e_pm_time_valid;
                v_irq_flag(IRQ_AM_SYNC_LOST_BIT)   := e_am_sync_lost;
                v_irq_flag(IRQ_PM_SYNC_LOST_BIT)   := e_pm_sync_lost;
                v_irq_flag(IRQ_PLL_LOCK_RISE_BIT)  := e_pll_lock_rise;
                v_irq_flag(IRQ_PLL_LOCK_FALL_BIT)  := e_pll_lock_fall;

                -- Sticky: OR in new sets, AND-mask off the W1C clears.
                -- If a set and a clear collide on the same bit / same
                -- cycle, the set wins (don't lose the event).
                irq_flag_q <= (irq_flag_q and not irq_flag_clr) or v_irq_flag;

                -- Sample previous-cycle source values.
                prev_am_soft_valid  <= status.am_soft_valid;
                prev_minute_mark    <= status.minute_mark;
                prev_pm_minute_mark <= status.pm_minute_mark;
                prev_time_valid     <= status.time_valid;
                prev_pm_time_valid  <= status.pm_time_valid;
                prev_sync_state     <= status.sync_state;
                prev_pm_sync_lock   <= cur_pm_sync_lock;
                prev_pll_lock       <= status.pll_lock;
            end if;
        end if;
    end process;

    -- Level-sensitive irq output: OR-reduce(flag AND mask).
    irq <= '1' when (irq_flag_q and control.irq_mask) /= (irq_flag_q'range => '0')
           else '0';

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
            NCO_PHASE_INCR => NCO_PHASE_INCR,
            WITH_PM        => WITH_PM,
            WITH_PM_DEBUG  => WITH_PM_DEBUG
        )
        port map (
            clk             => s_axi_aclk,
            rst             => core_rst,
            in_sample       => adc_sample,
            in_valid        => core_valid,
            env_mode        => control.env_mode,
            pll_override    => pll_override,
            phase_err_out   => open,
            phase_err_valid => open,
            status_out      => status
        );

end architecture rtl;
