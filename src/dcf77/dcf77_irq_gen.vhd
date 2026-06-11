-- =========================================================================
-- dcf77_irq_gen.vhd
--
-- Interrupt generator for the DCF77 AXI4-Lite slave, modelled on the
-- axircb_irq_gen pattern (doc/axircb_irq_gen.vhd): a self-contained
-- two-process entity owning the sticky flag register, the mask
-- read-back, the unmasked-interrupt set and the registered IRQ line.
--
-- Sources (edge-detected here from the core's live status record; bit
-- positions = the IRQ_*_BIT constants in dcf77_pkg):
--   second_tick      rising  am_soft_valid (1 Hz bit_valid pulse)
--   am_minute_mark   rising  minute_mark
--   pm_minute_mark   rising  pm_minute_mark
--   am_time_valid    rising  time_valid
--   pm_time_valid    rising  pm_time_valid
--   am_sync_lost     falling (sync_state = sync_lock)
--   pm_sync_lost     falling (pm_sync_state = LOCK)
--   pll_lock_rise    rising  pll_lock
--   pll_lock_fall    falling pll_lock
--
-- Flag semantics: W1C.  FromWritemux.flag carries one-cycle clear
-- pulses (a bit is '1' for exactly one clk after the host wrote '1');
-- new events OR in after the clear, so a set and a clear colliding on
-- the same bit in the same cycle keep the event.
--
-- Mask semantics (template-conform): the IRQ line latches per event.
-- An interrupt enters the unmasked set when its flag is pending while
-- its mask bit is set; once in the set it stays until its FLAG is
-- cleared -- disabling the mask bit afterwards does NOT deassert IRQ.
-- Enabling the mask while a flag is already pending does raise IRQ.
--
-- IRQ output is registered.  Latency event -> irq is 3 clk (event ->
-- recent, recent -> flag, flag/unmasked -> irq); W1C -> deassert is
-- 2 clk.  Level-sensitive: wire to a level-triggered controller input.
-- =========================================================================

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

use work.dcf77_pkg.all;

entity dcf77_irq_gen is
    port (
        clk          : in  std_logic;
        rst          : in  std_logic;

        -- Live source signals from the core.
        FromStatus   : in  dcf77_status_type;

        -- Host writes: flag = one-cycle W1C clear pulses, mask =
        -- sticky enable word (held by the writemux's register).
        FromWritemux : in  dcf77_irq_register_type;

        -- Read-back: live sticky flags + mask echo.  Served by the
        -- readmux at REG_IRQ_FLAG / REG_IRQ_MASK.
        ToReadmux    : out dcf77_irq_register_type;

        irq          : out std_logic
    );
end entity dcf77_irq_gen;

architecture rtl of dcf77_irq_gen is

    -- Registered source levels for edge detection.  The two sync-state
    -- sources are pre-folded to "currently in LOCK" booleans so the
    -- edge logic below is a uniform rising/falling test.
    type src_type is record
        am_soft_valid  : std_logic;
        minute_mark    : std_logic;
        pm_minute_mark : std_logic;
        time_valid     : std_logic;
        pm_time_valid  : std_logic;
        am_in_lock     : std_logic;
        pm_in_lock     : std_logic;
        pll_lock       : std_logic;
    end record;

    constant SRC_RESET : src_type := (others => '0');

    type reg_type is record
        host_in  : dcf77_irq_register_type;  -- registered writemux input
        out_regs : dcf77_irq_register_type;  -- flag sticky + mask echo
        src_prev : src_type;                 -- previous source levels
        recent   : dcf77_irq_source_type;    -- events detected this cycle
        unmasked : dcf77_irq_source_type;    -- latched per-event IRQ set
        irq      : std_logic;
    end record;

    constant R_RESET : reg_type := (
        host_in  => DCF77_IRQ_REGISTER_RESET,
        out_regs => DCF77_IRQ_REGISTER_RESET,
        src_prev => SRC_RESET,
        recent   => DCF77_IRQ_SOURCE_RESET,
        unmasked => DCF77_IRQ_SOURCE_RESET,
        irq      => '0'
    );

    signal r, rin : reg_type;

begin

    ToReadmux <= r.out_regs;
    irq       <= r.irq;

    comb : process (all)
        variable v   : reg_type;
        variable cur : src_type;
    begin
        v := r;

        -- Default: no interrupt request.
        v.irq := '0';

        -- Register the host inputs; mask echoes through to read-back.
        v.host_in       := FromWritemux;
        v.out_regs.mask := v.host_in.mask;

        -- Sample the current source levels.
        cur.am_soft_valid  := FromStatus.am_soft_valid;
        cur.minute_mark    := FromStatus.minute_mark;
        cur.pm_minute_mark := FromStatus.pm_minute_mark;
        cur.time_valid     := FromStatus.time_valid;
        cur.pm_time_valid  := FromStatus.pm_time_valid;
        if FromStatus.sync_state = sync_lock then
            cur.am_in_lock := '1';
        else
            cur.am_in_lock := '0';
        end if;
        if FromStatus.pm_sync_state = "10" then
            cur.pm_in_lock := '1';
        else
            cur.pm_in_lock := '0';
        end if;
        cur.pll_lock := FromStatus.pll_lock;

        -- Edge detection: rising for the event pulses, falling for the
        -- sync-lost / lock-fall sources.
        v.recent.second_tick    := cur.am_soft_valid  and not r.src_prev.am_soft_valid;
        v.recent.am_minute_mark := cur.minute_mark    and not r.src_prev.minute_mark;
        v.recent.pm_minute_mark := cur.pm_minute_mark and not r.src_prev.pm_minute_mark;
        v.recent.am_time_valid  := cur.time_valid     and not r.src_prev.time_valid;
        v.recent.pm_time_valid  := cur.pm_time_valid  and not r.src_prev.pm_time_valid;
        v.recent.am_sync_lost   := r.src_prev.am_in_lock and not cur.am_in_lock;
        v.recent.pm_sync_lost   := r.src_prev.pm_in_lock and not cur.pm_in_lock;
        v.recent.pll_lock_rise  := cur.pll_lock and not r.src_prev.pll_lock;
        v.recent.pll_lock_fall  := r.src_prev.pll_lock and not cur.pll_lock;
        v.src_prev := cur;

        -- Sticky flags: host W1C clears first, new events OR in after
        -- (a set and a clear colliding on the same bit keep the event).
        v.out_regs.flag := slv_to_irq(
            (irq_to_slv(r.out_regs.flag) and not irq_to_slv(r.host_in.flag))
            or irq_to_slv(r.recent));

        -- Unmasked-interrupt set (template semantics): a bit stays in
        -- the set while its flag is pending, regardless of later mask
        -- changes; it enters the set whenever flag and mask are both
        -- high.  Clearing the FLAG is the only way out.
        v.unmasked := slv_to_irq(
            (irq_to_slv(r.unmasked)     and irq_to_slv(v.out_regs.flag)) or
            (irq_to_slv(r.host_in.mask) and irq_to_slv(v.out_regs.flag)));

        -- Registered IRQ: high while any bit is in the unmasked set.
        if irq_to_slv(v.unmasked) /= (31 downto 0 => '0') then
            v.irq := '1';
        end if;

        if rst = '1' then
            v := R_RESET;
        end if;

        rin <= v;
    end process;

    seq : process (clk)
    begin
        if rising_edge(clk) then
            r <= rin;
        end if;
    end process;

end architecture rtl;
