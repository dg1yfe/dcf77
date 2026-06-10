-- =========================================================================
-- dcf77_pkg.vhd
--
-- Shared types and helper functions for the DCF77 receiver / decoder.
--
-- Convention: signal-processing nets are sfixed / ufixed with explicit
-- (high downto low) ranges so the LSB scale of every word is encoded in the
-- type.  Pure modular counters (phase accumulator, bit-position counter,
-- pulse-width timer) stay on unsigned, and frame-buffer style storage stays
-- on std_logic_vector.
-- =========================================================================

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use ieee.fixed_pkg.all;

package dcf77_pkg is

    ---------------------------------------------------------------------------
    -- Signal-processing subtypes.
    --
    -- All sfixed ranges below use LSB position 0 (integer-valued) for the
    -- top-level interfaces.  Internal CIC stages choose their own
    -- (high downto low) ranges via dcf77_cic_widths_pkg.
    ---------------------------------------------------------------------------
    subtype adc_sample_t is sfixed(13 downto 0);   -- raw signed ADC sample
    subtype iq_word_t    is sfixed(21 downto 0);   -- 22 b signed I/Q at 244 kS/s

    -- Position of the next bit to be received within the minute.  0..58 are
    -- the valid received-bit slots; 59 is the "frame complete, awaiting
    -- minute mark" sentinel.  Range-limited so it can't silently overflow.
    subtype bit_index_t  is natural range 0 to 59;
    -- Envelope power = top18(I)*top18(I) + top18(Q)*top18(Q).  The mixer
    -- env_word_t carries the AM envelope magnitude computed by dcf77_env via
    -- the |I| + 3/8*|Q| approximation (alpha=1, beta=3/8).  The iq_word_t
    -- operands fit in 22 b at LSB 0, so the magnitude max is
    -- (1 + 3/8) * 2^21 ~= 2.88 M, well within the 25 b unsigned range of
    -- env_word_t.  sfixed (signed) is kept so the bit_det IIRs can reuse
    -- the same arithmetic operators across env_in, env_max, env_min and the
    -- difference terms, but the value is by construction non-negative.
    subtype env_word_t   is sfixed(25 downto 0);   -- 26 b magnitude, LSB 2^0
    subtype rssi_word_t  is ufixed(15 downto 0);   -- 16 b slow-IIR RSSI

    ---------------------------------------------------------------------------
    -- Input record from the host (or test bench) into the decoder core.
    ---------------------------------------------------------------------------
    type dcf77_in_type is record
        sample       : adc_sample_t;
        sample_valid : std_logic;
    end record;

    constant DCF77_IN_RESET : dcf77_in_type := (
        sample       => (others => '0'),
        sample_valid => '0'
    );

    ---------------------------------------------------------------------------
    -- Decoded time-of-frame.  All fields are BCD-decoded integer values.
    -- The transmitted time is the wall-clock minute that begins at the
    -- next minute mark (see dcf77_format_spec.md §1.5).
    ---------------------------------------------------------------------------
    type dcf77_time_type is record
        year         : unsigned(7 downto 0);  -- 00..99
        month        : unsigned(3 downto 0);  -- 01..12
        day          : unsigned(4 downto 0);  -- 01..31
        weekday      : unsigned(2 downto 0);  -- 1=Mon .. 7=Sun
        hour         : unsigned(4 downto 0);  -- 00..23
        minute       : unsigned(5 downto 0);  -- 00..59
        dst_active   : std_logic;             -- bit 17 (MESZ, UTC+2)
        dst_announce : std_logic;             -- bit 16 (DST change next hour)
        leap_announce: std_logic;             -- bit 19 (leap second next hour)
    end record;

    constant DCF77_TIME_RESET : dcf77_time_type := (
        year          => (others => '0'),
        month         => (others => '0'),
        day           => (others => '0'),
        weekday       => (others => '0'),
        hour          => (others => '0'),
        minute        => (others => '0'),
        dst_active    => '0',
        dst_announce  => '0',
        leap_announce => '0'
    );

    ---------------------------------------------------------------------------
    -- Decoder synchronisation state.  The enum is the natural form for the
    -- RTL; the explicit conversion function pins the AXI register encoding
    -- so it is independent of how synthesis assigns the enum.
    ---------------------------------------------------------------------------
    type sync_state_t is (sync_search, sync_lock, sync_hold);

    function sync_state_to_slv(s : sync_state_t)
        return std_logic_vector;
    function slv_to_sync_state(s : std_logic_vector)
        return sync_state_t;

    -- Status carries *everything* the decoder has produced: the full decoded
    -- time, the validity / sync flags, the raw 59-bit frame buffer, and the
    -- RSSI.  The dcf77_core aggregates this from its sub-blocks and presents
    -- a single record to the AXI readmux (the only other input to the
    -- readmux is the dcf77_control_type echoed from the writemux).
    type dcf77_status_type is record
        time         : dcf77_time_type;               -- decoded time fields
        time_valid   : std_logic;                     -- last frame fully parity-OK
        sync_state   : sync_state_t;
        sync_quality : unsigned(5 downto 0);          -- 0..63 in-guard streak counter
        mm_quality   : unsigned(1 downto 0);          -- 0..3 minute-mark confidence
        cycle_in_min : unsigned(5 downto 0);          -- 0..59 predicted sec-in-minute
        rssi         : rssi_word_t;                   -- top bits of env_max
        env_min      : rssi_word_t;                   -- top bits of env_min
        noise        : rssi_word_t;                   -- top bits of noise IIR (MAD)
        bit_index    : bit_index_t;                   -- current second 0..58 (59 = frame full)
        last_frame   : std_logic_vector(0 to 58);     -- raw bits of last minute

        -- Soft-metric fields (1 Hz, latched at end_of_bit).  Feed the
        -- SOFT_AM / SOFT_MM registers; PM fields are placeholders
        -- pending the PSK correlator and return zero.
        am_soft      : signed(11 downto 0);           -- (acc_W2 - th_new) >> K, saturated
        am_soft_valid: std_logic;                     -- bit_valid pulse latched
        mm_soft      : signed(11 downto 0);           -- (acc_W1 - mm_thr) >> K, saturated
        minute_mark  : std_logic;                     -- last end_of_bit was an mm
        pll_lock     : std_logic;                     -- carrier_pll lock_state
        anchor_held  : std_logic;                     -- bit_det has anchor

        -- PM (PSK/SIK) path fields.  Sourced by dcf77_pm_correlator +
        -- a second instance of dcf77_frame_dec configured with
        -- PM_MODE=true.  All values default to zero before the first
        -- PM lock is achieved.
        pm_soft         : signed(11 downto 0);        -- SIK bit LLR
        pm_valid        : std_logic;                  -- pm_soft / pm_sec_index just updated
        pm_minute_mark  : std_logic;                  -- SIK signature was just detected
        pm_sec_index    : bit_index_t;                -- 0..59, latched at pm_valid
        pm_bit_index    : bit_index_t;                -- live frame_dec bit_idx
        pm_sync_state   : std_logic_vector(1 downto 0);  -- correlator sync state
        pm_lock_qual    : unsigned(15 downto 0);      -- correlator lock-quality IIR
        pm_time         : dcf77_time_type;            -- PM-decoded time
        pm_time_valid   : std_logic;                  -- PM frame decoded OK
        pm_last_frame   : std_logic_vector(0 to 58);  -- raw PM frame bits

        -- Carrier-PLL snapshots, latched on each pm_valid (= once per
        -- locked second).  Both are in NCO-offset units (i_state has
        -- been right-shifted by I_FRAC_BITS internally) so software
        -- can compare/subtract directly.
        --   pll_phase_filt : 20-Hz LPF state, signed 29 b
        --   pll_i_state    : PI integrator >> I_FRAC, signed 28 b
        --   pll_offset     : current phase_inc_offset (live, not
        --                    snapshotted -- it's the actual NCO drive,
        --                    useful for the host's REG_PLL_OFFSET
        --                    readback)
        pll_phase_filt  : signed(28 downto 0);
        pll_i_state     : signed(27 downto 0);
        pll_offset      : signed(23 downto 0);
    end record;

    constant DCF77_STATUS_RESET : dcf77_status_type := (
        time         => DCF77_TIME_RESET,
        time_valid   => '0',
        sync_state   => sync_search,
        sync_quality => (others => '0'),
        mm_quality   => (others => '0'),
        cycle_in_min => (others => '0'),
        rssi         => (others => '0'),
        env_min      => (others => '0'),
        noise        => (others => '0'),
        bit_index    => 0,
        last_frame   => (others => '0'),
        am_soft      => (others => '0'),
        am_soft_valid=> '0',
        mm_soft      => (others => '0'),
        minute_mark  => '0',
        pll_lock     => '0',
        anchor_held  => '0',
        pm_soft         => (others => '0'),
        pm_valid        => '0',
        pm_minute_mark  => '0',
        pm_sec_index    => 0,
        pm_bit_index    => 0,
        pm_sync_state   => (others => '0'),
        pm_lock_qual    => (others => '0'),
        pm_time         => DCF77_TIME_RESET,
        pm_time_valid   => '0',
        pm_last_frame   => (others => '0'),
        pll_phase_filt  => (others => '0'),
        pll_i_state     => (others => '0'),
        pll_offset      => (others => '0')
    );

    ---------------------------------------------------------------------------
    -- Aggregate core output: just the decoded status (the post-mixer I/Q
    -- tap that used to live here was removed alongside the CIC3 stage).
    ---------------------------------------------------------------------------
    type dcf77_out_type is record
        status : dcf77_status_type;
    end record;

    constant DCF77_OUT_RESET : dcf77_out_type := (
        status => DCF77_STATUS_RESET
    );

    ---------------------------------------------------------------------------
    -- PM-correlator debug stream.  Output-only record from
    -- dcf77_pm_correlator.  acquire_v pulses for one clock cycle each time
    -- the engine completes one hypothesis evaluation in ACQUIRE mode;
    -- track_v likewise pulses for each TRACK pass (T_MINUS / T_CENTER /
    -- T_PLUS).  Lets a TB dump the full autocorrelation profile across
    -- all 2510 ACQUIRE hypotheses without exposing internal regs through
    -- a thicket of individual ports.  Production wiring leaves the whole
    -- record open / unobserved (synthesis drops the unused logic).
    ---------------------------------------------------------------------------
    type dcf77_pm_debug_type is record
        -- ACQUIRE event: one pulse per hypothesis
        acquire_v          : std_logic;
        acquire_hyp_idx    : unsigned(11 downto 0);
        acquire_abs_acc    : unsigned(18 downto 0);
        acquire_sec_phase  : unsigned(11 downto 0);
        -- TRACK event: one pulse per pass (T_MINUS/T_CENTER/T_PLUS)
        track_v            : std_logic;
        track_step         : unsigned(1 downto 0);    -- 0=M, 1=C, 2=P
        track_abs_acc      : unsigned(18 downto 0);
        track_locked_phase : unsigned(11 downto 0);
        -- Buffer base address used to read this pass (= chip-0
        -- position the engine integrated from).  Lets the TB verify
        -- TRACK alignment vs ACQUIRE's ac_best_phase.
        track_next_offset  : unsigned(11 downto 0);
        -- '1' while the per-hypothesis engine is running (E_REQ /
        -- E_RECV / E_DONE).  Used by the gated-clock TB to keep the
        -- clock toggling only while there's actual engine work to do
        -- -- in idle gaps between phase_err_v pulses the clock can
        -- pause and GHDL skips ahead in sim time without burning
        -- per-cycle event-processing wall time.
        engine_active      : std_logic;
    end record;

    constant DCF77_PM_DEBUG_RESET : dcf77_pm_debug_type := (
        acquire_v          => '0',
        acquire_hyp_idx    => (others => '0'),
        acquire_abs_acc    => (others => '0'),
        acquire_sec_phase  => (others => '0'),
        track_v            => '0',
        track_step         => (others => '0'),
        track_abs_acc      => (others => '0'),
        track_locked_phase => (others => '0'),
        track_next_offset  => (others => '0'),
        engine_active      => '0'
    );

    ---------------------------------------------------------------------------
    -- Host-writable control word.  Plumbed from the AXI4-Lite writemux into
    -- the core; the readmux can also read it back so the host can verify a
    -- write took effect.
    ---------------------------------------------------------------------------
    type dcf77_control_type is record
        enable     : std_logic;     -- master enable
        soft_reset : std_logic;     -- pulse this high to reset the core
        env_mode   : std_logic;     -- 0 = abs(I) only, 1 = |I| + 3/8*|Q|
        irq_mask   : std_logic_vector(31 downto 0); -- per-bit irq enables
    end record;

    constant DCF77_CONTROL_RESET : dcf77_control_type := (
        enable     => '1',          -- default-on
        soft_reset => '0',
        env_mode   => '0',          -- abs(I) by default; host can set to
                                    -- '1' for |I|+3/8|Q| (interferer-robust
                                    -- at ~0.5 dB sensitivity cost).
        irq_mask   => (others => '0') -- all sources masked off at reset
                                    -- (host opts in by writing IRQ_MASK).
    );

    ---------------------------------------------------------------------------
    -- Carrier-PLL software overrides.  Both writes generate a one-cycle
    -- pulse on the corresponding _wr signal; the carrier_pll latches the
    -- value immediately on that clk (no phase_err_valid sync, no held-
    -- override semantics).  Read-back goes through dcf77_status_type and
    -- always reports the current HW state -- the register IS the
    -- hardware, no software-side buffering.
    ---------------------------------------------------------------------------
    type dcf77_pll_override_type is record
        offset_wr  : std_logic;                     -- 1-cycle pulse
        offset_val : signed(23 downto 0);
        i_state_wr : std_logic;                     -- 1-cycle pulse
        i_state_val: signed(27 downto 0);
    end record;

    constant DCF77_PLL_OVERRIDE_RESET : dcf77_pll_override_type := (
        offset_wr   => '0',
        offset_val  => (others => '0'),
        i_state_wr  => '0',
        i_state_val => (others => '0')
    );

    ---------------------------------------------------------------------------
    -- AXI4-Lite register map (word index = addr(7 downto 2)).
    ---------------------------------------------------------------------------
    constant REG_CTRL        : natural := 0;   -- 0x00 RW
    constant REG_STATUS      : natural := 1;   -- 0x04 R
    constant REG_TIME_HMS    : natural := 2;   -- 0x08 R
    constant REG_DATE        : natural := 3;   -- 0x0C R
    constant REG_FLAGS       : natural := 4;   -- 0x10 R
    constant REG_RSSI        : natural := 5;   -- 0x14 R   env_max top 16 b
    constant REG_LAST_FRAME0 : natural := 6;   -- 0x18 R
    constant REG_LAST_FRAME1 : natural := 7;   -- 0x1C R
    constant REG_ENV_MIN     : natural := 8;   -- 0x20 R   env_min top 16 b
    constant REG_NOISE       : natural := 9;   -- 0x24 R   noise_iir top 16 b
    constant REG_SYNC_DIAG   : natural := 10;  -- 0x28 R   sync_q / mm_q / cycle_in_min
    constant REG_SOFT_AM        : natural := 11;  -- 0x2C R   AM soft metric + flags
    constant REG_SOFT_PM        : natural := 12;  -- 0x30 R   PM soft metric + flags
    -- 0x34: per-source sticky interrupt flags (W1C).  See the
    -- IRQ_*_BIT constants below for the bit layout.  Reading returns
    -- the live sticky state; writing '1' to a bit clears that flag.
    -- The `irq` output of dcf77_axi_lite is OR(IRQ_FLAG & IRQ_MASK).
    constant REG_IRQ_FLAG       : natural := 13;  -- 0x34 R/W1C interrupt sources
    constant REG_PM_STATUS      : natural := 14;  -- 0x38 R   PM sync/lock diagnostics
    constant REG_PM_TIME_HMS    : natural := 15;  -- 0x3C R   PM-decoded HH:MM
    constant REG_PM_DATE        : natural := 16;  -- 0x40 R   PM-decoded date
    constant REG_PM_FLAGS       : natural := 17;  -- 0x44 R   PM-decoded flags
    constant REG_PM_LAST_FRAME0 : natural := 18;  -- 0x48 R   PM frame bits 0..31
    constant REG_PM_LAST_FRAME1 : natural := 19;  -- 0x4C R   PM frame bits 32..58
    -- Carrier-PLL telemetry & overrides (read snapshot, write = jam).
    --
    -- PLL_PHASE_FILT : R-only.  Snapshot of 20-Hz LPF state, signed
    --                 29 b sign-extended to 32 b.  Sampled on pm_valid.
    -- PLL_I_STATE    : R/W.
    --                 R = snapshot of integrator >> I_FRAC_BITS,
    --                     signed 28 b sign-extended to 32 b.
    --                 W = jam the integrator with bits[27:0].  The
    --                     write event itself is the trigger -- the
    --                     value is latched in carrier_pll and applied
    --                     on the next phase_err_valid pulse.
    -- PLL_OFFSET     : R/W.
    --                 R = current phase_inc_offset (signed 24 b in
    --                     bits[23:0], sign-extended to 32 b).
    --                 W = jam the NCO offset with bits[23:0] for one
    --                     phase_err_valid cycle (PI resumes the cycle
    --                     after).  Useful for bench testing & for
    --                     bumping the PLL with a known step.
    constant REG_PLL_PHASE_FILT : natural := 20;  -- 0x50 R
    constant REG_PLL_I_STATE    : natural := 21;  -- 0x54 RW
    constant REG_PLL_OFFSET     : natural := 22;  -- 0x58 RW
    -- IRQ_MASK: per-bit enable for the irq output line.  Bit positions
    -- match REG_IRQ_FLAG one-for-one.  Reset value is 0 → host opts in.
    constant REG_IRQ_MASK       : natural := 23;  -- 0x5C RW

    ---------------------------------------------------------------------------
    -- Interrupt source bit positions for REG_IRQ_FLAG and REG_IRQ_MASK.
    -- The dcf77_axi_lite shell edge-detects each source signal from the
    -- core's dcf77_status_type, sets the corresponding sticky bit on
    -- the rising edge, and asserts the `irq` output while any
    -- unmasked sticky bit is set.  Bits not listed here are reserved
    -- (read 0; writes ignored on the flag register, ignored on the mask
    -- register since they don't drive anything).
    constant IRQ_SECOND_TICK_BIT    : natural := 0;
        -- 1 Hz rising edge on status.am_soft_valid (= bit_valid pulse
        -- from the AM bit_det).  The canonical "fresh soft metrics are
        -- ready" interrupt.
    constant IRQ_AM_MINUTE_MARK_BIT : natural := 1;
        -- Rising edge on status.minute_mark (AM frame just completed).
    constant IRQ_PM_MINUTE_MARK_BIT : natural := 2;
        -- Rising edge on status.pm_minute_mark (PM SIK signature edge).
        -- Tied off when WITH_PM = false.
    constant IRQ_AM_TIME_VALID_BIT  : natural := 3;
        -- Rising edge on status.time_valid (fresh AM decode passed parity).
    constant IRQ_PM_TIME_VALID_BIT  : natural := 4;
        -- Rising edge on status.pm_time_valid.  Tied off when WITH_PM = false.
    constant IRQ_AM_SYNC_LOST_BIT   : natural := 5;
        -- AM sync_state transitioned out of sync_lock.
    constant IRQ_PM_SYNC_LOST_BIT   : natural := 6;
        -- PM sync_state transitioned out of LOCK (value "10").
        -- Tied off when WITH_PM = false.
    constant IRQ_PLL_LOCK_RISE_BIT  : natural := 7;
        -- Carrier-PLL lock_state 0 -> 1.
    constant IRQ_PLL_LOCK_FALL_BIT  : natural := 8;
        -- Carrier-PLL lock_state 1 -> 0.

    ---------------------------------------------------------------------------
    -- 32-b register pack / unpack helpers.  The bit layouts below match the
    -- plan in /Users/eligs/.claude/plans/this-project-aims-to-toasty-pelican.md.
    ---------------------------------------------------------------------------

    -- CTRL register: bit 0 enable, bit 1 soft_reset, bit 2 env_mode, rest reserved.
    function ctrl_to_slv (c : dcf77_control_type) return std_logic_vector;
    function slv_to_ctrl (s : std_logic_vector) return dcf77_control_type;

    -- All read-side registers take the full dcf77_status_type since that is
    -- the only data source from the decoder.

    -- STATUS register:
    --   bits  1..0  : sync_state
    --   bit   2     : time_valid
    --   bit   3     : pm_present       (1 = WITH_PM       was enabled at synth)
    --   bit   4     : pm_debug_present (1 = WITH_PM_DEBUG was enabled at synth)
    --   bit   5     : pll_lock         (live)
    --   bit   6     : anchor_held      (bit_det's AM anchor state, live)
    --   bits 13..8  : bit_index
    --   bits 31..16 : rssi
    -- Bits 3 and 4 are not part of dcf77_status_type (they're synthesis
    -- constants); the axi_readmux ORs them in directly from its own
    -- generic.  Bits 5 and 6 come from the status record's live
    -- pll_lock / anchor_held fields (host-readable replacements for the
    -- defunct SOFT_IRQ bits).
    constant REG_STATUS_PM_PRESENT_BIT       : natural := 3;
    constant REG_STATUS_PM_DEBUG_PRESENT_BIT : natural := 4;
    constant REG_STATUS_PLL_LOCK_BIT         : natural := 5;
    constant REG_STATUS_ANCHOR_HELD_BIT      : natural := 6;
    function status_to_slv (s : dcf77_status_type) return std_logic_vector;

    -- TIME_HMS register: bits 5..0 minute, 12..8 hour.
    function time_hms_to_slv (s : dcf77_status_type) return std_logic_vector;

    -- DATE register: 7..0 year, 11..8 month, 20..16 day, 26..24 weekday.
    function date_to_slv (s : dcf77_status_type) return std_logic_vector;

    -- FLAGS register: bit 0 dst_active, bit 1 dst_announce, bit 2 leap_announce.
    function flags_to_slv (s : dcf77_status_type) return std_logic_vector;

    -- LAST_FRAME 0 / 1: frame bits 0..31 / 32..58.
    function last_frame0_to_slv (s : dcf77_status_type) return std_logic_vector;
    function last_frame1_to_slv (s : dcf77_status_type) return std_logic_vector;

    -- SYNC_DIAG register:
    --   bits  5..0  : sync_quality (0..63 saturating in-guard streak)
    --   bits  7..6  : mm_quality   (0..3 minute-mark confidence counter)
    --   bits 13..8  : cycle_in_min (0..59 predicted second-in-minute)
    --   bits 31..14 : reserved
    function sync_diag_to_slv (s : dcf77_status_type) return std_logic_vector;

    -- SOFT_AM register (both AM and MM LLRs in one word):
    --   [11:0]  am_soft (signed 12 b, = (acc_W2 - th_new) >> K, sat)
    --   [23:12] mm_soft (signed 12 b, = (acc_W1 - mm_thr) >> K, sat)
    --   [29:24] second_index (bit_index 0..58)
    --   [30]    am_valid (also covers mm_soft -- both update together)
    --   [31]    minute_mark (last end_of_bit produced an mm)
    function soft_am_to_slv  (s : dcf77_status_type) return std_logic_vector;

    -- SOFT_PM register:
    --   [5:0]    pm_sec_index   (snapshot — second pm_soft was latched for)
    --   [17:6]   pm_soft        (signed 12 b, SIK bit LLR)
    --   [29:18]  reserved       (was pm_mm_soft; software computes its own)
    --   [30]     pm_valid
    --   [31]     pm_minute_mark
    function soft_pm_to_slv  (s : dcf77_status_type) return std_logic_vector;

    -- PM_STATUS register:
    --   [5:0]    pm_bit_index   (live 0..59 counter from frame_dec_pm)
    --   [7:6]    pm_sync_state  (0=FILL, 1=ACQUIRE, 2=LOCK)
    --   [8]      pm_time_valid
    --   [31:16]  pm_lock_qual   (slow IIR of |pm_soft|)
    function pm_status_to_slv (s : dcf77_status_type) return std_logic_vector;

    -- PM_TIME_HMS / PM_DATE / PM_FLAGS: same layouts as TIME_HMS / DATE /
    -- FLAGS but populated from the PM-side decoded time.
    function pm_time_hms_to_slv (s : dcf77_status_type) return std_logic_vector;
    function pm_date_to_slv     (s : dcf77_status_type) return std_logic_vector;
    function pm_flags_to_slv    (s : dcf77_status_type) return std_logic_vector;

    -- PM_LAST_FRAME0 / 1: PM frame bits 0..31 / 32..58.
    function pm_last_frame0_to_slv (s : dcf77_status_type) return std_logic_vector;
    function pm_last_frame1_to_slv (s : dcf77_status_type) return std_logic_vector;

    -- PLL_PHASE_FILT register: sign-extended 29-b filt state in bits 31..0.
    -- PLL_I_STATE    register: sign-extended 28-b integrator in  bits 31..0.
    -- PLL_OFFSET     register: sign-extended 24-b NCO offset in  bits 31..0.
    function pll_phase_filt_to_slv (s : dcf77_status_type) return std_logic_vector;
    function pll_i_state_to_slv    (s : dcf77_status_type) return std_logic_vector;
    function pll_offset_to_slv     (s : dcf77_status_type) return std_logic_vector;

    ---------------------------------------------------------------------------
    -- Helper functions.
    ---------------------------------------------------------------------------

    -- Even parity over an arbitrary-length std_logic_vector.
    function even_parity(v : std_logic_vector) return std_logic;

    -- BCD decode a contiguous slice of the 59-bit frame, LSB-first
    -- (per dcf77_format_spec.md §1.3).  weight_lo is the value of the
    -- lowest-order bit (typically 1).  Returns the sum-of-weights as an
    -- unsigned of width `result_width`.
    function bcd_decode_lsb_first(slice       : std_logic_vector;
                                  result_width: natural)
        return unsigned;

end package dcf77_pkg;


package body dcf77_pkg is

    function even_parity(v : std_logic_vector) return std_logic is
        variable acc : std_logic := '0';
    begin
        for i in v'range loop
            acc := acc xor v(i);
        end loop;
        return acc;
    end function;

    function bcd_decode_lsb_first(slice       : std_logic_vector;
                                  result_width: natural)
        return unsigned
    is
        variable acc    : unsigned(result_width - 1 downto 0) := (others => '0');
        variable weight : unsigned(result_width - 1 downto 0);
    begin
        -- DCF77 BCD bit weights are 1,2,4,8 for the units decade then
        -- 10,20,40,80 for the tens decade (year reaches all 8 positions).
        -- The slice is interpreted LSB-first per dcf77_format_spec.md §1.3.
        for i in 0 to slice'length - 1 loop
            case i is
                when 0 => weight := to_unsigned(1,  result_width);
                when 1 => weight := to_unsigned(2,  result_width);
                when 2 => weight := to_unsigned(4,  result_width);
                when 3 => weight := to_unsigned(8,  result_width);
                when 4 => weight := to_unsigned(10, result_width);
                when 5 => weight := to_unsigned(20, result_width);
                when 6 => weight := to_unsigned(40, result_width);
                when 7 => weight := to_unsigned(80, result_width);
                when others => weight := (others => '0');
            end case;
            if slice(slice'low + i) = '1' then
                acc := acc + weight;
            end if;
        end loop;
        return acc;
    end function;

    ---------------------------------------------------------------------------
    -- Register pack / unpack
    ---------------------------------------------------------------------------

    function sync_state_to_slv(s : sync_state_t) return std_logic_vector is
    begin
        case s is
            when sync_search => return "00";
            when sync_lock   => return "01";
            when sync_hold   => return "10";
        end case;
    end function;

    function slv_to_sync_state(s : std_logic_vector) return sync_state_t is
    begin
        case s is
            when "00"   => return sync_search;
            when "01"   => return sync_lock;
            when "10"   => return sync_hold;
            when others => return sync_search;
        end case;
    end function;

    function ctrl_to_slv (c : dcf77_control_type) return std_logic_vector is
        variable r : std_logic_vector(31 downto 0) := (others => '0');
    begin
        r(0) := c.enable;
        r(1) := c.soft_reset;
        r(2) := c.env_mode;
        return r;
    end function;

    function slv_to_ctrl (s : std_logic_vector) return dcf77_control_type is
        variable c : dcf77_control_type;
    begin
        c.enable     := s(s'low + 0);
        c.soft_reset := s(s'low + 1);
        c.env_mode   := s(s'low + 2);
        return c;
    end function;

    function status_to_slv (s : dcf77_status_type) return std_logic_vector is
        variable r : std_logic_vector(31 downto 0) := (others => '0');
    begin
        r(1 downto 0)   := sync_state_to_slv(s.sync_state);
        r(2)            := s.time_valid;
        -- bits 3 and 4 are ORed in by axi_readmux from its WITH_PM /
        -- WITH_PM_DEBUG generics (synthesis-time capability bits).
        r(REG_STATUS_PLL_LOCK_BIT)    := s.pll_lock;
        r(REG_STATUS_ANCHOR_HELD_BIT) := s.anchor_held;
        r(13 downto 8)  := std_logic_vector(to_unsigned(s.bit_index, 6));
        r(31 downto 16) := std_logic_vector(s.rssi);
        return r;
    end function;

    function time_hms_to_slv (s : dcf77_status_type) return std_logic_vector is
        variable r : std_logic_vector(31 downto 0) := (others => '0');
    begin
        r(5 downto 0)  := std_logic_vector(s.time.minute);
        r(12 downto 8) := std_logic_vector(s.time.hour);
        return r;
    end function;

    function date_to_slv (s : dcf77_status_type) return std_logic_vector is
        variable r : std_logic_vector(31 downto 0) := (others => '0');
    begin
        r(7 downto 0)   := std_logic_vector(s.time.year);
        r(11 downto 8)  := std_logic_vector(s.time.month);
        r(20 downto 16) := std_logic_vector(s.time.day);
        r(26 downto 24) := std_logic_vector(s.time.weekday);
        return r;
    end function;

    function flags_to_slv (s : dcf77_status_type) return std_logic_vector is
        variable r : std_logic_vector(31 downto 0) := (others => '0');
    begin
        r(0) := s.time.dst_active;
        r(1) := s.time.dst_announce;
        r(2) := s.time.leap_announce;
        return r;
    end function;

    function last_frame0_to_slv (s : dcf77_status_type) return std_logic_vector is
        variable r : std_logic_vector(31 downto 0) := (others => '0');
    begin
        for i in 0 to 31 loop
            r(i) := s.last_frame(i);
        end loop;
        return r;
    end function;

    function last_frame1_to_slv (s : dcf77_status_type) return std_logic_vector is
        variable r : std_logic_vector(31 downto 0) := (others => '0');
    begin
        for i in 32 to 58 loop
            r(i - 32) := s.last_frame(i);
        end loop;
        return r;
    end function;

    function sync_diag_to_slv (s : dcf77_status_type) return std_logic_vector is
        variable r : std_logic_vector(31 downto 0) := (others => '0');
    begin
        r( 5 downto  0) := std_logic_vector(s.sync_quality);
        r( 7 downto  6) := std_logic_vector(s.mm_quality);
        r(13 downto  8) := std_logic_vector(s.cycle_in_min);
        return r;
    end function;

    function soft_am_to_slv (s : dcf77_status_type) return std_logic_vector is
        variable r : std_logic_vector(31 downto 0) := (others => '0');
    begin
        r(11 downto  0) := std_logic_vector(s.am_soft);
        r(23 downto 12) := std_logic_vector(s.mm_soft);
        r(29 downto 24) := std_logic_vector(to_unsigned(s.bit_index, 6));
        r(30)           := s.am_soft_valid;
        r(31)           := s.minute_mark;
        return r;
    end function;

    function soft_pm_to_slv (s : dcf77_status_type) return std_logic_vector is
        variable r : std_logic_vector(31 downto 0) := (others => '0');
    begin
        r( 5 downto  0) := std_logic_vector(to_unsigned(s.pm_sec_index, 6));
        r(17 downto  6) := std_logic_vector(s.pm_soft);
        -- bits 29..18 reserved (was pm_mm_soft)
        r(30)           := s.pm_valid;
        r(31)           := s.pm_minute_mark;
        return r;
    end function;

    function pm_status_to_slv (s : dcf77_status_type) return std_logic_vector is
        variable r : std_logic_vector(31 downto 0) := (others => '0');
    begin
        r( 5 downto  0) := std_logic_vector(to_unsigned(s.pm_bit_index, 6));
        r( 7 downto  6) := s.pm_sync_state;
        r(8)            := s.pm_time_valid;
        r(31 downto 16) := std_logic_vector(s.pm_lock_qual);
        return r;
    end function;

    function pm_time_hms_to_slv (s : dcf77_status_type) return std_logic_vector is
        variable r : std_logic_vector(31 downto 0) := (others => '0');
    begin
        r( 5 downto  0) := std_logic_vector(s.pm_time.minute);
        r(12 downto  8) := std_logic_vector(s.pm_time.hour);
        return r;
    end function;

    function pm_date_to_slv (s : dcf77_status_type) return std_logic_vector is
        variable r : std_logic_vector(31 downto 0) := (others => '0');
    begin
        r( 7 downto  0) := std_logic_vector(s.pm_time.year);
        r(11 downto  8) := std_logic_vector(s.pm_time.month);
        r(20 downto 16) := std_logic_vector(s.pm_time.day);
        r(26 downto 24) := std_logic_vector(s.pm_time.weekday);
        return r;
    end function;

    function pm_flags_to_slv (s : dcf77_status_type) return std_logic_vector is
        variable r : std_logic_vector(31 downto 0) := (others => '0');
    begin
        r(0) := s.pm_time.dst_active;
        r(1) := s.pm_time.dst_announce;
        r(2) := s.pm_time.leap_announce;
        return r;
    end function;

    function pm_last_frame0_to_slv (s : dcf77_status_type) return std_logic_vector is
        variable r : std_logic_vector(31 downto 0) := (others => '0');
    begin
        for i in 0 to 31 loop
            r(i) := s.pm_last_frame(i);
        end loop;
        return r;
    end function;

    function pm_last_frame1_to_slv (s : dcf77_status_type) return std_logic_vector is
        variable r : std_logic_vector(31 downto 0) := (others => '0');
    begin
        for i in 32 to 58 loop
            r(i - 32) := s.pm_last_frame(i);
        end loop;
        return r;
    end function;

    function pll_phase_filt_to_slv (s : dcf77_status_type) return std_logic_vector is
    begin
        -- Sign-extend 29-b signed to 32 b.
        return std_logic_vector(resize(s.pll_phase_filt, 32));
    end function;

    function pll_i_state_to_slv (s : dcf77_status_type) return std_logic_vector is
    begin
        -- Sign-extend 28-b signed to 32 b.
        return std_logic_vector(resize(s.pll_i_state, 32));
    end function;

    function pll_offset_to_slv (s : dcf77_status_type) return std_logic_vector is
    begin
        -- Sign-extend 24-b signed to 32 b.
        return std_logic_vector(resize(s.pll_offset, 32));
    end function;

end package body dcf77_pkg;
