-- =========================================================================
-- dcf77_bit_det.vhd
--
-- Single-phase symbol-structural AM bit detector at 100 S/s.  Anchors
-- the bit cadence to the first significant falling envelope edge after
-- reset / loss-of-anchor, then runs a free-running modulo-BIT_PERIOD
-- sample counter from there.  No internal phase-locked loop -- the
-- carrier-tracking PLL upstream provides cesium-locked time stability
-- through the NCO; in replay-mode test flow the system-clock /
-- env_valid cadence is good enough over the test duration (sample-
-- rate drift of 10 ppm yields ~6 ms over a minute, well within the
-- 100 ms bit width).
--
-- Symbol-structural windows (in env_valid ticks, called "bins"):
--   W1 = [0..W1_END)           = dip / minute-mark sense (10 bins)
--   W2 = [W1_END..W2_END)      = bit-value discriminator (10 bins)
--   W3 = [W2_END..BIT_PERIOD)  = carrier reference -> iir_high (80 bins)
--
-- Decision (identical to the previous implementation):
--   threshold := iir_high * 37/64          ~= 0.578 * iir_high
--   mm_thr    := 7 * env_max
--   candidate := acc_W1 > mm_thr           (mm-candidate when carrier
--                                            stayed high for all 10 W1
--                                            samples)
--   bit_value := '1' if acc_W2 < threshold else '0'
--
-- Outputs are NOT gated by any internal lock state; the caller
-- (dcf77_core for real-mode, replay TB for back-end-only test) decides
-- whether to consume them.  `sync_state` is derived from have_anchor +
-- mm_counter for backward-compatible status reporting.
-- =========================================================================

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use ieee.fixed_float_types.all;
use ieee.fixed_pkg.all;

use work.dcf77_pkg.all;

entity dcf77_bit_det is
    generic (
        -- Slow trackers (SNR estimator) -- shifts at the env_valid rate.
        -- Defaults below are scaled for SAMPLE_RATE_HZ = 2000 (= +5 over
        -- the legacy 100 S/s defaults so the continuous-time TCs match).
        K_ATK          : positive := 8;
        K_DEC          : positive := 14;
        K_NOISE        : positive := 14;

        -- Falling-edge amplitude detector.
        -- DERIV_DELAY = ~ one IIR-TC of the upstream 20 Hz LPF.
        DERIV_DELAY    : positive := 16;
        K_SLOPE        : positive := 2;

        -- Symbol windows (in env_valid ticks at SAMPLE_RATE_HZ).
        -- 100 ms / 200 ms windows at 2 kS/s.
        BIT_PERIOD     : positive := 2000;
        W1_END         : positive := 200;
        W2_END         : positive := 400;
        W3_END         : positive := 2000;

        -- Structural bit decision.  K_REF is per-bit (= per-second),
        -- independent of SAMPLE_RATE_HZ.
        K_REF          : positive := 2;

        -- Anchor give-up: clear have_anchor after this many env_valid
        -- ticks with no falling edge (~10 s @ SAMPLE_RATE_HZ).
        GIVE_UP_TICKS  : positive := 20000;

        -- Diagnostic counter caps.
        SYNC_Q_MAX     : positive := 63;
        MM_Q_MAX       : positive := 3;

        -- Legacy generics retained for compatibility with the dcf77_core
        -- generic forwarding; unused under the simplified anchor design.
        SAMPLE_RATE_HZ : positive := 2000;
        T_HOLD_SEC     : positive := 120;
        T_SEARCH_SEC   : positive := 300;

        -- Per-cycle diagnostic dump at every end_of_bit (acc_W1, acc_W2,
        -- mm_thr, th_new, candidate, bit_value).  Off by default; flip
        -- to true in a TB when chasing false candidate / wrong bit value.
        DEBUG_TRACE    : boolean := false;

        -- AM-LLR right shift applied to (acc_W2 - th_new) before saturating
        -- into 12-bit signed.  Default 7 keeps the dynamic range of a
        -- ~3000 env_peak corpus inside ±2 k LSB.  Software is told this
        -- constant and treats am_soft as a fixed-point LLR.
        AM_LLR_SHIFT   : positive := 7;

        -- MM-LLR right shift applied to (acc_W1 - mm_thr) before saturating
        -- into 12-bit signed.  Same scale rationale as AM_LLR_SHIFT;
        -- acc_W1 and mm_thr live on the same acc10_t / iir_high range.
        MM_LLR_SHIFT   : positive := 7
    );
    port (
        clk         : in  std_logic;
        rst         : in  std_logic;

        env_in      : in  env_word_t;
        env_valid   : in  std_logic;

        bit_value     : out std_logic;
        bit_valid     : out std_logic;
        minute_mark   : out std_logic;
        sync_state    : out sync_state_t;
        sync_quality  : out unsigned(5 downto 0);
        mm_quality    : out unsigned(1 downto 0);
        cycle_in_min  : out unsigned(5 downto 0);
        env_max       : out env_word_t;
        env_min       : out env_word_t;
        noise         : out env_word_t;
        threshold     : out env_word_t;

        -- Latched AM-LLR (signed 12 b): acc_W2 - th_new, right-shifted
        -- by AM_LLR_SHIFT, saturated.  Updated at end_of_bit; holds
        -- between updates.  Negative => bit '1' favoured, positive =>
        -- bit '0' favoured.  am_soft_valid pulses with bit_valid.
        am_soft       : out signed(11 downto 0);
        am_soft_valid : out std_logic;

        -- Latched MM-LLR (signed 12 b): acc_W1 - mm_thr, right-shifted
        -- by MM_LLR_SHIFT, saturated.  Positive => minute-mark
        -- candidate (W1 carrier-on, no dip), negative => normal bit.
        mm_soft       : out signed(11 downto 0);
        anchor_held   : out std_logic;

        edge_accepted : out std_logic
    );
end entity dcf77_bit_det;

architecture rtl of dcf77_bit_det is

    function clog2(n : positive) return natural is
        variable acc : natural := 0;
        variable v   : natural := n - 1;
    begin
        while v > 0 loop
            acc := acc + 1;
            v   := v / 2;
        end loop;
        return acc;
    end function;

    function maxp(a, b : positive) return positive is
    begin
        if a > b then return a; else return b; end if;
    end function;

    -- Window accumulator widths grow with the number of samples
    -- summed.  acc12_t (W1, W2) sized for the longer of the two
    -- windows.  acc3_t (W3) sized for BIT_PERIOD - W2_END samples.
    constant ACC12_GROWTH : natural := clog2(maxp(W1_END,
                                                  W2_END - W1_END));
    constant ACC3_GROWTH  : natural := clog2(W3_END - W2_END);

    subtype acc12_t is sfixed(env_word_t'high + ACC12_GROWTH
                              downto env_word_t'low);
    subtype acc3_t  is sfixed(env_word_t'high + ACC3_GROWTH
                              downto env_word_t'low);

    -- Compatibility aliases: the rest of the architecture used to
    -- reference acc10_t (W1/W2 scale) and acc80_t (W3 scale).
    subtype acc10_t is acc12_t;
    subtype acc80_t is acc3_t;

    constant DELAY_DEPTH : natural := DERIV_DELAY + 1;
    type env_delay_t is array (0 to DELAY_DEPTH - 1) of env_word_t;

    type reg_type is record
        -- Slow trackers.
        env_max     : env_word_t;
        env_min     : env_word_t;
        noise_iir   : env_word_t;
        carrier_thr : env_word_t;
        carrier_hys : env_word_t;
        carrier     : std_logic;

        -- Falling-edge detector.
        env_dly     : env_delay_t;
        slope_below : std_logic;

        -- Free-running modulo-BIT_PERIOD sample counter.
        sample_in_bit : natural range 0 to BIT_PERIOD - 1;
        have_anchor   : std_logic;
        give_up_cnt   : natural range 0 to GIVE_UP_TICKS;

        -- Run-on mm-candidate detector: an mm candidate firing in
        -- consecutive cycles means our anchor is off-phase and W1 is
        -- landing on the carrier portion of every bit.  Drop the
        -- anchor after RUN_ON_LIMIT in a row so the next falling edge
        -- re-anchors at the (presumably) correct bit boundary.
        cand_run_on  : natural range 0 to 7;

        -- Warm-up counter: mm-candidate evaluation is suppressed for
        -- the first few cycles after anchoring so iir_high (which is
        -- the reference for mm_thr) can settle to a meaningful value.
        -- Cleared on every cold/give-up anchor reset.
        warmup_cnt : natural range 0 to 15;

        -- sync_q saturating "seconds anchored" counter.
        sync_q : natural range 0 to SYNC_Q_MAX;

        -- Window accumulators.
        acc_W1 : acc10_t;
        acc_W2 : acc10_t;
        acc_W3 : acc80_t;

        -- Structural decision state.
        iir_high    : acc10_t;
        threshold_o : acc10_t;

        -- Minute-mark state.
        mm_counter   : natural range 0 to MM_Q_MAX;
        cycle_in_min : natural range 0 to 59;

        -- Registered outputs.
        bit_value     : std_logic;
        bit_valid     : std_logic;
        minute_mark   : std_logic;
        edge_accepted : std_logic;
        am_soft       : signed(11 downto 0);
        mm_soft       : signed(11 downto 0);
    end record;

    constant ZERO_ENV   : env_word_t := (others => '0');
    constant ZERO_ACC10 : acc10_t    := (others => '0');
    constant ZERO_ACC80 : acc80_t    := (others => '0');

    constant R_RESET : reg_type := (
        env_max         => ZERO_ENV,
        env_min         => ZERO_ENV,
        noise_iir       => ZERO_ENV,
        carrier_thr     => ZERO_ENV,
        carrier_hys     => ZERO_ENV,
        carrier         => '1',
        env_dly         => (others => ZERO_ENV),
        slope_below     => '0',
        sample_in_bit   => 0,
        have_anchor     => '0',
        give_up_cnt     => 0,
        cand_run_on     => 0,
        warmup_cnt      => 0,
        sync_q          => 0,
        acc_W1          => ZERO_ACC10,
        acc_W2          => ZERO_ACC10,
        acc_W3          => ZERO_ACC80,
        iir_high        => ZERO_ACC10,
        threshold_o     => ZERO_ACC10,
        mm_counter      => 0,
        cycle_in_min    => 0,
        bit_value       => '0',
        bit_valid       => '0',
        minute_mark     => '0',
        edge_accepted   => '0',
        am_soft         => (others => '0'),
        mm_soft         => (others => '0')
    );

    signal r, rin : reg_type;

begin

    bit_value     <= r.bit_value;
    bit_valid     <= r.bit_valid;
    minute_mark   <= r.minute_mark;
    am_soft       <= r.am_soft;
    am_soft_valid <= r.bit_valid;
    mm_soft       <= r.mm_soft;
    anchor_held   <= r.have_anchor;
    sync_quality  <= to_unsigned(r.sync_q,       6);
    mm_quality    <= to_unsigned(r.mm_counter,   2);
    cycle_in_min  <= to_unsigned(r.cycle_in_min, 6);
    env_max       <= r.env_max;
    env_min       <= r.env_min;
    noise         <= r.noise_iir;
    edge_accepted <= r.edge_accepted;
    threshold     <= resize(r.threshold_o,
                            env_word_t'high, env_word_t'low,
                            fixed_saturate, fixed_truncate);

    process (r.have_anchor, r.mm_counter)
    begin
        if r.have_anchor = '0' then
            sync_state <= sync_search;
        elsif r.mm_counter > 0 then
            sync_state <= sync_lock;
        else
            sync_state <= sync_hold;
        end if;
    end process;

    comb : process (all)
        variable v          : reg_type;

        -- Slow-tracker scratch.
        variable delta_e    : sfixed(env_word_t'high + 1 downto env_word_t'low);
        variable abs_dev    : sfixed(env_word_t'high + 1 downto env_word_t'low);
        variable shifted_e  : sfixed(env_word_t'high + 1 downto env_word_t'low);
        variable th_hi      : sfixed(env_word_t'high + 1 downto env_word_t'low);
        variable th_lo      : sfixed(env_word_t'high + 1 downto env_word_t'low);
        variable new_max    : sfixed(env_word_t'high + 1 downto env_word_t'low);
        variable new_min    : sfixed(env_word_t'high + 1 downto env_word_t'low);
        variable new_noise  : sfixed(env_word_t'high + 1 downto env_word_t'low);

        -- Falling-edge detector scratch.
        variable d1         : sfixed(env_word_t'high + 1 downto env_word_t'low);
        variable slope_thr  : sfixed(env_word_t'high + 1 downto env_word_t'low);
        variable neg_thr    : sfixed(env_word_t'high + 1 downto env_word_t'low);
        variable amp_ok     : std_logic;

        -- Bit-counter scratch.
        variable current_bin  : natural range 0 to BIT_PERIOD - 1;
        variable end_of_bit   : std_logic;

        -- End-of-bit decision scratch.
        variable candidate     : std_logic;
        variable do_fire_mm    : std_logic;
        variable do_fire_bit   : std_logic;
        variable acc_W3_d3     : acc10_t;
        variable iir_delta     : sfixed(acc10_t'high + 1 downto acc10_t'low);
        variable iir_shifted   : sfixed(acc10_t'high + 1 downto acc10_t'low);
        variable new_iir       : sfixed(acc10_t'high + 1 downto acc10_t'low);
        variable th_new        : acc10_t;
        variable th_h1, th_h4, th_h6 : acc10_t;
        variable am_soft_full  : acc10_t;
        variable mm_soft_full  : acc10_t;
        variable em_acc, em_x4, em_x2, mm_thr : acc10_t;
    begin
        v := r;

        v.bit_valid     := '0';
        v.minute_mark   := '0';
        v.edge_accepted := '0';

        -- Default scratch values.
        current_bin := r.sample_in_bit;
        end_of_bit  := '0';

        if env_valid = '1' then

            -- ============================================================
            -- (1) Slow trackers (env_max / env_min / noise_iir / carrier).
            -- ============================================================
            if env_in > r.env_max then
                delta_e   := resize(env_in - r.env_max,
                                    delta_e'high, delta_e'low);
                shifted_e := shift_right(delta_e, K_ATK);
                new_max   := resize(r.env_max + shifted_e,
                                    new_max'high, new_max'low);
            else
                shifted_e := shift_right(resize(r.env_max,
                                                shifted_e'high, shifted_e'low),
                                         K_DEC);
                new_max   := resize(r.env_max - shifted_e,
                                    new_max'high, new_max'low);
            end if;
            v.env_max := resize(new_max, env_word_t'high, env_word_t'low,
                                fixed_saturate, fixed_truncate);

            if env_in < r.env_min then
                delta_e   := resize(r.env_min - env_in,
                                    delta_e'high, delta_e'low);
                shifted_e := shift_right(delta_e, K_ATK);
                new_min   := resize(r.env_min - shifted_e,
                                    new_min'high, new_min'low);
            else
                delta_e   := resize(r.env_max - r.env_min,
                                    delta_e'high, delta_e'low);
                shifted_e := shift_right(delta_e, K_DEC);
                new_min   := resize(r.env_min + shifted_e,
                                    new_min'high, new_min'low);
            end if;
            v.env_min := resize(new_min, env_word_t'high, env_word_t'low,
                                fixed_saturate, fixed_truncate);

            v.carrier_thr := resize(shift_right(v.env_max, 1),
                                    env_word_t'high, env_word_t'low,
                                    fixed_saturate, fixed_truncate);
            v.carrier_hys := resize(shift_right(v.env_max, 3),
                                    env_word_t'high, env_word_t'low,
                                    fixed_saturate, fixed_truncate);
            th_hi := resize(v.carrier_thr + v.carrier_hys,
                            th_hi'high, th_hi'low);
            th_lo := resize(v.carrier_thr - v.carrier_hys,
                            th_lo'high, th_lo'low);
            if env_in >= th_hi then
                v.carrier := '1';
            elsif env_in <= th_lo then
                v.carrier := '0';
            else
                v.carrier := r.carrier;
            end if;

            if v.carrier = '1' then
                delta_e := resize(env_in - r.env_max,
                                  delta_e'high, delta_e'low);
            else
                delta_e := resize(env_in - r.env_min,
                                  delta_e'high, delta_e'low);
            end if;
            if delta_e(delta_e'high) = '1' then
                abs_dev := resize(- delta_e, abs_dev'high, abs_dev'low);
            else
                abs_dev := delta_e;
            end if;
            shifted_e := shift_right(resize(r.noise_iir,
                                            shifted_e'high, shifted_e'low),
                                     K_NOISE);
            new_noise := resize(r.noise_iir
                                + resize(shift_right(abs_dev, K_NOISE),
                                         new_noise'high, new_noise'low)
                                - shifted_e,
                                new_noise'high, new_noise'low);
            v.noise_iir := resize(new_noise, env_word_t'high, env_word_t'low,
                                  fixed_saturate, fixed_truncate);

            -- ============================================================
            -- (2) Falling-edge amplitude detector (one-shot debounced).
            -- ============================================================
            d1 := resize(env_in - r.env_dly(DERIV_DELAY - 1),
                         d1'high, d1'low);
            slope_thr := resize(shift_right(v.env_max, K_SLOPE),
                                slope_thr'high, slope_thr'low);
            neg_thr   := resize(- slope_thr,
                                neg_thr'high, neg_thr'low);

            amp_ok := '0';
            if slope_thr > to_sfixed(0, slope_thr'high, slope_thr'low)
               and d1 < neg_thr then
                if r.slope_below = '0' then
                    amp_ok := '1';
                end if;
                v.slope_below := '1';
            else
                v.slope_below := '0';
            end if;

            for i in DELAY_DEPTH - 1 downto 1 loop
                v.env_dly(i) := r.env_dly(i - 1);
            end loop;
            v.env_dly(0) := env_in;

            -- ============================================================
            -- (3) Anchor + counter advance.
            --     Cold anchor on first amp_ok after no-anchor.
            --     No re-anchor on subsequent edges -- a noise edge mid-
            --     cycle would otherwise snap the counter to the wrong
            --     phase, and small alignment errors fall well within
            --     the W1 window's tolerance.  Anchor is only re-acquired
            --     after a give_up timeout clears have_anchor.
            -- ============================================================
            if amp_ok = '1' and r.have_anchor = '0' then
                v.have_anchor   := '1';
                v.give_up_cnt   := 0;
                v.warmup_cnt    := 0;
                current_bin     := 0;
                v.sample_in_bit := 1;
                v.acc_W1        := ZERO_ACC10;
                v.acc_W2        := ZERO_ACC10;
                v.acc_W3        := ZERO_ACC80;
                end_of_bit      := '0';
                v.edge_accepted := '1';

            elsif r.have_anchor = '1' then
                current_bin := r.sample_in_bit;
                if r.sample_in_bit = BIT_PERIOD - 1 then
                    v.sample_in_bit := 0;
                    end_of_bit      := '1';
                else
                    v.sample_in_bit := r.sample_in_bit + 1;
                end if;

                if amp_ok = '1' then
                    v.give_up_cnt   := 0;
                    v.edge_accepted := '1';
                elsif r.give_up_cnt < GIVE_UP_TICKS then
                    v.give_up_cnt := r.give_up_cnt + 1;
                end if;

            else
                -- Not anchored and no edge: stay idle, accs empty.
                current_bin := 0;
                end_of_bit  := '0';
            end if;

            -- Give-up: lose anchor after a long gap of no edges.
            if v.give_up_cnt = GIVE_UP_TICKS then
                v.have_anchor   := '0';
                v.give_up_cnt   := 0;
                v.warmup_cnt    := 0;
                v.sample_in_bit := 0;
                v.acc_W1        := ZERO_ACC10;
                v.acc_W2        := ZERO_ACC10;
                v.acc_W3        := ZERO_ACC80;
            end if;

            -- ============================================================
            -- (4) Accumulator update for THIS sample's bin.
            --     Only meaningful while anchored; uses v.have_anchor
            --     (which is post-update for the cold-anchor case).
            -- ============================================================
            if v.have_anchor = '1' then
                if current_bin < W1_END then
                    v.acc_W1 := resize(v.acc_W1 + env_in,
                                       acc10_t'high, acc10_t'low,
                                       fixed_saturate, fixed_truncate);
                elsif current_bin < W2_END then
                    v.acc_W2 := resize(v.acc_W2 + env_in,
                                       acc10_t'high, acc10_t'low,
                                       fixed_saturate, fixed_truncate);
                else
                    v.acc_W3 := resize(v.acc_W3 + env_in,
                                       acc80_t'high, acc80_t'low,
                                       fixed_saturate, fixed_truncate);
                end if;
            end if;

            -- ============================================================
            -- (5) End-of-bit decision.
            -- ============================================================
            if end_of_bit = '1' then

                -- Threshold = 0.578125 * iir_high.
                th_h1 := resize(shift_right(r.iir_high, 1),
                                acc10_t'high, acc10_t'low,
                                fixed_saturate, fixed_truncate);
                th_h4 := resize(shift_right(r.iir_high, 4),
                                acc10_t'high, acc10_t'low,
                                fixed_saturate, fixed_truncate);
                th_h6 := resize(shift_right(r.iir_high, 6),
                                acc10_t'high, acc10_t'low,
                                fixed_saturate, fixed_truncate);
                th_new := resize(th_h1 + th_h4 + th_h6,
                                 acc10_t'high, acc10_t'low,
                                 fixed_saturate, fixed_truncate);
                v.threshold_o := th_new;

                -- mm-candidate threshold ~= 0.6875 * iir_high.  iir_high
                -- tracks acc_W3 / 8 = (W3_END - W2_END) / 8 envelope
                -- samples; with the W1/W2/W3 ratio kept across rates
                -- (W1_END * 8 = W3_END - W2_END) this equals one W1
                -- window of pure carrier.  Old "7 * env_max" worked at
                -- W1_END=10 because iir_high happened to track 10*env_max;
                -- expressing the threshold as a fraction of iir_high makes
                -- it rate-agnostic.
                em_x4 := resize(shift_right(r.iir_high, 1),
                                acc10_t'high, acc10_t'low,
                                fixed_saturate, fixed_truncate);
                em_x2 := resize(shift_right(r.iir_high, 3),
                                acc10_t'high, acc10_t'low,
                                fixed_saturate, fixed_truncate);
                em_acc := resize(shift_right(r.iir_high, 4),
                                 acc10_t'high, acc10_t'low,
                                 fixed_saturate, fixed_truncate);
                mm_thr := resize(em_x4 + em_x2 + em_acc,
                                 acc10_t'high, acc10_t'low,
                                 fixed_saturate, fixed_truncate);
                -- Warm-up counter: bump once per end_of_bit until
                -- iir_high has had time to converge.  Until then, the
                -- mm-candidate threshold (a fraction of iir_high) is
                -- too small and would trigger on bit-cycle W1 dips.
                if r.warmup_cnt < 15 then
                    v.warmup_cnt := r.warmup_cnt + 1;
                end if;

                candidate := '0';
                if r.warmup_cnt >= 8 and v.acc_W1 > mm_thr then
                    candidate := '1';
                end if;

                -- Run-on detector: mm-candidate firing in N consecutive
                -- cycles means our anchor is off-phase and W1 is landing
                -- on the carrier portion of every bit.  Real signal has
                -- one mm per minute, so two in a row is already wrong;
                -- threshold N=3 leaves margin for transient confusion
                -- around the real mm boundary.
                if candidate = '1' then
                    if r.cand_run_on < 7 then
                        v.cand_run_on := r.cand_run_on + 1;
                    end if;
                else
                    v.cand_run_on := 0;
                end if;

                do_fire_mm  := '0';
                do_fire_bit := '0';

                if r.mm_counter = 0 then
                    if candidate = '1' then
                        v.mm_counter   := 1;
                        v.cycle_in_min := 0;
                        do_fire_mm     := '1';
                    end if;
                elsif r.cycle_in_min = 59 then
                    if candidate = '1' then
                        if r.mm_counter < MM_Q_MAX then
                            v.mm_counter := r.mm_counter + 1;
                        end if;
                    else
                        v.mm_counter := r.mm_counter - 1;
                    end if;
                    v.cycle_in_min := 0;
                    do_fire_mm     := '1';
                else
                    if candidate = '1' and r.mm_counter > 0 then
                        v.mm_counter := r.mm_counter - 1;
                    end if;
                end if;

                if do_fire_mm = '0' then
                    if v.acc_W2 < th_new then
                        v.bit_value := '1';
                    else
                        v.bit_value := '0';
                    end if;
                    do_fire_bit := '1';
                end if;

                -- AM-LLR: (acc_W2 - th_new) >> AM_LLR_SHIFT, saturated to
                -- 12-bit signed.  Sign convention: positive = bit '0'
                -- favoured (carrier present in W2), negative = bit '1'.
                am_soft_full := resize(v.acc_W2 - th_new,
                                       am_soft_full'high, am_soft_full'low,
                                       fixed_saturate, fixed_truncate);
                v.am_soft := signed(to_slv(resize(
                    shift_right(am_soft_full, AM_LLR_SHIFT),
                    11, 0, fixed_saturate, fixed_truncate)));

                -- MM-LLR: (acc_W1 - mm_thr) >> MM_LLR_SHIFT, saturated.
                -- Positive = minute-mark candidate (no dip in W1),
                -- negative = normal bit (W1 contains the dip).
                mm_soft_full := resize(v.acc_W1 - mm_thr,
                                       mm_soft_full'high, mm_soft_full'low,
                                       fixed_saturate, fixed_truncate);
                v.mm_soft := signed(to_slv(resize(
                    shift_right(mm_soft_full, MM_LLR_SHIFT),
                    11, 0, fixed_saturate, fixed_truncate)));

                v.bit_valid   := do_fire_bit;
                v.minute_mark := do_fire_mm;

                if DEBUG_TRACE then
                    report "trace cyc_in_min=" & integer'image(r.cycle_in_min)
                        & " mm_cnt=" & integer'image(r.mm_counter)
                        & " acc_W1=" & integer'image(to_integer(signed(to_slv(v.acc_W1))))
                        & " acc_W2=" & integer'image(to_integer(signed(to_slv(v.acc_W2))))
                        & " mm_thr=" & integer'image(to_integer(signed(to_slv(mm_thr))))
                        & " th_new=" & integer'image(to_integer(signed(to_slv(th_new))))
                        & " iir_high=" & integer'image(to_integer(signed(to_slv(r.iir_high))))
                        & " cand=" & std_logic'image(candidate)
                        & " fire_mm=" & std_logic'image(do_fire_mm)
                        & " bit_val=" & std_logic'image(v.bit_value)
                        severity note;
                end if;

                -- iir_high update (W3 is full carrier on every cycle type).
                acc_W3_d3 := resize(shift_right(v.acc_W3, 3),
                                    acc10_t'high, acc10_t'low,
                                    fixed_saturate, fixed_truncate);
                iir_delta := resize(acc_W3_d3 - r.iir_high,
                                    iir_delta'high, iir_delta'low);
                iir_shifted := shift_right(iir_delta, K_REF);
                new_iir := resize(r.iir_high + iir_shifted,
                                  new_iir'high, new_iir'low);
                v.iir_high := resize(new_iir,
                                     acc10_t'high, acc10_t'low,
                                     fixed_saturate, fixed_truncate);

                if do_fire_mm = '0' then
                    if r.cycle_in_min < 59 then
                        v.cycle_in_min := r.cycle_in_min + 1;
                    else
                        v.cycle_in_min := 0;
                    end if;
                end if;

                -- sync_q ticks once per anchored end-of-bit.
                if r.sync_q < SYNC_Q_MAX then
                    v.sync_q := r.sync_q + 1;
                end if;

                v.acc_W1 := ZERO_ACC10;
                v.acc_W2 := ZERO_ACC10;
                v.acc_W3 := ZERO_ACC80;

                -- Wrong-anchor recovery: drop anchor when too many mm
                -- candidates fire back-to-back.  Next falling edge
                -- will re-anchor; iir_high / env_max state is kept so
                -- the re-acquired anchor doesn't have to re-warm.
                if v.cand_run_on >= 3 then
                    v.have_anchor   := '0';
                    v.give_up_cnt   := 0;
                    v.cand_run_on   := 0;
                    v.sample_in_bit := 0;
                    v.mm_counter    := 0;
                    v.cycle_in_min  := 0;
                    v.sync_q        := 0;
                    -- Suppress this cycle's outputs -- they were
                    -- almost certainly garbage.
                    v.bit_valid     := '0';
                    v.minute_mark   := '0';
                end if;

            end if;

            -- Anchor lost -> reset sync_q.
            if v.have_anchor = '0' then
                v.sync_q := 0;
            end if;

        end if;

        rin <= v;
    end process;

    seq : process (clk)
    begin
        if rising_edge(clk) then
            if rst = '1' then
                r <= R_RESET;
            else
                r <= rin;
            end if;
        end if;
    end process;

end architecture rtl;
