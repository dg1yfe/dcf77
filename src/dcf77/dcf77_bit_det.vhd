-- =========================================================================
-- dcf77_bit_det.vhd
--
-- Single-phase symbol-structural AM bit detector at 100 S/s with a
-- 1st-order PLL implemented as a 22-b fixed-point NCO.
--
-- Sync architecture
-- -----------------
-- phase_acc (22-b unsigned) advances by NOMINAL_STEP = 2^22 / 100 = 41943
-- each env_dec_v tick.  After 100 ticks the accumulator has advanced
-- 100 * 41943 = 4_194_300, four units short of 2^22 = 4_194_304, so the
-- natural unsigned wrap occurs every ~100.0095 ticks — a 0.0095 %
-- residual cycle-length error the PLL absorbs trivially.
--
-- The wrap of phase_acc is the end-of-bit event.  Window boundaries are
-- expressed in phase_acc units (= W_END * NOMINAL_STEP), so the
-- accumulator-region check is a direct unsigned comparison.
--
-- Loop filter is a regular-clocked 1st-order IIR on the most recent
-- phase-error measurement (sample-and-hold of phase_acc reinterpreted as
-- signed at the most recent accepted falling edge).  PLL correction to
-- the step is just a right-shift of the filter output: thanks to the
-- 22-b scale, both filter TC and correction TC come out as exact
-- power-of-two divisions of phase_acc units — no multipliers, no
-- Bresenham fractional accumulator.
--
--   filt[n+1]   = filt[n] + (last_phase_err - filt[n]) >> K_LOOP
--   step[n]     = NOMINAL_STEP - (filt[n] >> K_PLL)
--   phase_acc   = phase_acc + step  (mod 2^22)
--   end_of_bit  = wrap of phase_acc (= unsigned add carry-out)
--
-- K_LOOP and K_PLL are independent generics.  Matched (K_PLL=K_LOOP)
-- gives a critically-damped 1st-order PLL.  Decoupled (K_PLL<K_LOOP)
-- gives a smoothly-averaging filter with a faster phase-counter chase.
--
-- Edge gating: after each accepted edge, edge_lockout suppresses further
-- amp-qualified edges for EDGE_LOCKOUT_TICKS, but only while
-- lock_state='1'.  Cold-start sees every amplitude-qualified edge.
--
-- Cold-start: first edge after no-anchor (have_anchor='0') seeds
-- phase_acc := NOMINAL_STEP, filt := 0, last_phase_err := 0, clears
-- accumulators, sets have_anchor.  Subsequent edges sample-and-hold
-- phase_acc into last_phase_err.
--
-- Symbol-structural windows (in env_dec ticks, converted to phase_acc
-- boundaries internally):
--   W1 = [0..W1_END)   = dip / minute-mark sense
--   W2 = [W1_END..W2_END) = bit-value discriminator
--   W3 = [W2_END..BIT_PERIOD) = carrier reference -> iir_high
--
-- Threshold and decision: identical to the previous implementation.
--   threshold := iir_high * 37/64
--   candidate := acc_W1 > 7*env_max  (mm-candidate)
--   bit_value := '1' if acc_W2 < threshold else '0'
--   Outputs gated on lock_state = '1'.
-- =========================================================================

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use ieee.fixed_float_types.all;
use ieee.fixed_pkg.all;

use work.dcf77_pkg.all;

entity dcf77_bit_det is
    generic (
        -- Slow trackers (SNR estimator).
        K_ATK          : positive := 3;
        K_DEC          : positive := 9;
        K_NOISE        : positive := 9;

        -- Falling-edge amplitude detector.
        DERIV_DELAY    : positive := 2;
        K_SLOPE        : positive := 2;

        -- Symbol windows (in env_dec ticks).
        BIT_PERIOD     : positive := 100;
        W1_END         : positive := 10;
        W2_END         : positive := 20;
        W3_END         : positive := 100;

        -- Structural bit decision.
        K_REF          : positive := 2;

        -- PLL fixed-point NCO.  phase_acc width = PHASE_W (22 by default;
        -- gives nominal step = 2^22/100 ≈ 41943 with 0.01 % cycle error).
        PHASE_W        : positive := 22;

        -- PLL loop bandwidth (regular-clocked filter, 100 S/s).
        K_LOOP_FAST    : positive := 8;     -- pre-lock filter shift  (TC ~2.5 s)
        K_LOOP_SLOW    : positive := 10;    -- post-lock filter shift (TC ~10 s)

        -- Correction shift (right-shift of filt -> step adjustment).
        -- Matched (= K_LOOP_SLOW) gives critically-damped loop.
        K_PLL          : positive := 10;

        -- Lock detector hysteresis.
        LOCK_IN_THR    : positive := 5;     -- |filt_in_ticks| < this -> lock-eligible
        LOCK_OUT_THR   : positive := 15;    -- |filt_in_ticks| > this -> unlock-eligible
        LOCK_HOLD_TICKS: positive := 128;   -- consecutive ticks of in/out-of-band

        -- Post-lock edge-detection gate.
        EDGE_LOCKOUT_TICKS : positive := 80;

        -- Anchor give-up.
        GIVE_UP_TICKS  : positive := 1000;

        -- Diagnostic counter caps.
        SYNC_Q_MAX     : positive := 63;
        MM_Q_MAX       : positive := 3;

        -- Legacy (unused under lock_state-driven sync_state mapping).
        SAMPLE_RATE_HZ : positive := 100;
        T_HOLD_SEC     : positive := 120;
        T_SEARCH_SEC   : positive := 300
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

        edge_accepted : out std_logic
    );
end entity dcf77_bit_det;

architecture rtl of dcf77_bit_det is

    function clog2(n : natural) return natural is
        variable acc : natural := 0;
        variable v   : natural := n;
    begin
        while v > 0 loop
            acc := acc + 1;
            v   := v / 2;
        end loop;
        return acc;
    end function;

    -- 10-sample-sum / 80-sample-sum scales for the window accumulators.
    subtype acc10_t is sfixed(env_word_t'high + 4 downto env_word_t'low);
    subtype acc80_t is sfixed(env_word_t'high + 7 downto env_word_t'low);

    -- Phase / filter types.
    constant FILT_W       : positive := PHASE_W + 1;     -- 23 b: one bit headroom
    subtype phase_acc_t   is unsigned(PHASE_W - 1 downto 0);
    subtype phase_signed_t is signed(PHASE_W - 1 downto 0);
    subtype filt_t        is signed(FILT_W - 1 downto 0);

    -- Nominal step makes phase_acc wrap roughly every BIT_PERIOD ticks.
    constant NOMINAL_STEP : positive := 2 ** PHASE_W / BIT_PERIOD;

    -- Window boundaries in phase_acc units.
    constant W1_END_PA : positive := W1_END * NOMINAL_STEP;
    constant W2_END_PA : positive := W2_END * NOMINAL_STEP;
    -- W3 ends at the wrap, no explicit boundary needed.

    -- Lock detector thresholds in phase_acc units.
    constant LOCK_IN_PA  : positive := LOCK_IN_THR  * NOMINAL_STEP;
    constant LOCK_OUT_PA : positive := LOCK_OUT_THR * NOMINAL_STEP;

    constant DELAY_DEPTH : natural := DERIV_DELAY + 1;
    type env_delay_t is array (0 to DELAY_DEPTH - 1) of env_word_t;

    type reg_type is record
        -- Slow trackers (preserve for SNR estimator port).
        env_max     : env_word_t;
        env_min     : env_word_t;
        noise_iir   : env_word_t;
        carrier_thr : env_word_t;
        carrier_hys : env_word_t;
        carrier     : std_logic;

        -- Falling-edge detector.
        env_dly     : env_delay_t;
        slope_below : std_logic;

        -- Phase NCO + PLL.
        phase_acc      : phase_acc_t;
        filt           : filt_t;
        last_phase_err : phase_signed_t;
        lock_state     : std_logic;
        lock_dur       : natural range 0 to LOCK_HOLD_TICKS;

        -- Sync acquisition / lockout.
        have_anchor   : std_logic;
        edge_tick_cnt : natural range 0 to GIVE_UP_TICKS;
        edge_lockout  : natural range 0 to EDGE_LOCKOUT_TICKS;

        -- sync_q saturating "seconds locked" counter.
        sync_q          : natural range 0 to SYNC_Q_MAX;
        sync_q_tick_cnt : natural range 0 to BIT_PERIOD - 1;

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
    end record;

    constant ZERO_ENV   : env_word_t  := (others => '0');
    constant ZERO_ACC10 : acc10_t     := (others => '0');
    constant ZERO_ACC80 : acc80_t     := (others => '0');
    constant ZERO_PHASE : phase_acc_t := (others => '0');
    constant ZERO_PS    : phase_signed_t := (others => '0');
    constant ZERO_FILT  : filt_t      := (others => '0');

    constant R_RESET : reg_type := (
        env_max         => ZERO_ENV,
        env_min         => ZERO_ENV,
        noise_iir       => ZERO_ENV,
        carrier_thr     => ZERO_ENV,
        carrier_hys     => ZERO_ENV,
        carrier         => '1',
        env_dly         => (others => ZERO_ENV),
        slope_below     => '0',
        phase_acc       => ZERO_PHASE,
        filt            => ZERO_FILT,
        last_phase_err  => ZERO_PS,
        lock_state      => '0',
        lock_dur        => 0,
        have_anchor     => '0',
        edge_tick_cnt   => 0,
        edge_lockout    => 0,
        sync_q          => 0,
        sync_q_tick_cnt => 0,
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
        edge_accepted   => '0'
    );

    signal r, rin : reg_type;

begin

    bit_value    <= r.bit_value;
    bit_valid    <= r.bit_valid;
    minute_mark  <= r.minute_mark;
    sync_quality <= to_unsigned(r.sync_q,       6);
    mm_quality   <= to_unsigned(r.mm_counter,   2);
    cycle_in_min <= to_unsigned(r.cycle_in_min, 6);
    env_max      <= r.env_max;
    env_min      <= r.env_min;
    noise        <= r.noise_iir;
    edge_accepted <= r.edge_accepted;
    threshold    <= resize(r.threshold_o,
                           env_word_t'high, env_word_t'low,
                           fixed_saturate, fixed_truncate);

    process (r.lock_state, r.have_anchor)
    begin
        if r.lock_state = '1' then
            sync_state <= sync_lock;
        elsif r.have_anchor = '1' then
            sync_state <= sync_hold;
        else
            sync_state <= sync_search;
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

        -- Edge processing.
        variable edge_event : std_logic;
        variable cold_snap  : std_logic;

        -- PLL / NCO scratch.
        variable delta_f      : signed(FILT_W downto 0);
        variable corr_f       : signed(FILT_W downto 0);
        variable filt_new     : signed(FILT_W downto 0);
        variable correction   : filt_t;
        variable step_signed  : filt_t;
        variable step_us      : phase_acc_t;
        variable new_acc_ext  : unsigned(PHASE_W downto 0);
        variable wrap         : std_logic;
        variable filt_pos     : signed(FILT_W downto 0);
        variable filt_neg     : signed(FILT_W downto 0);
        variable in_band      : boolean;
        variable out_band     : boolean;

        -- Phase region for accumulator.
        variable phase_for_acc : phase_acc_t;
        variable end_of_bit    : std_logic;

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
        variable em_acc, em_x4, em_x2, mm_thr : acc10_t;

        variable k_loop_sel : natural;
    begin
        v := r;

        v.bit_valid     := '0';
        v.minute_mark   := '0';
        v.edge_accepted := '0';

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
            -- (3) Edge gating: post-lock edge lockout suppresses noise
            --     edges within 800 ms of the last accepted edge.
            -- ============================================================
            edge_event := amp_ok;
            if r.lock_state = '1' and r.edge_lockout > 0 then
                edge_event := '0';
            end if;

            -- ============================================================
            -- (4) Edge processing.
            --     - cold_snap: first edge after no-anchor: seed phase_acc
            --       so the current sample is bin 0 of a fresh bit cycle.
            --       Reset filter and accumulators.
            --     - subsequent edges: sample-and-hold last_phase_err
            --       (signed reinterpretation of phase_acc gives the
            --       centred ±2^(PHASE_W-1) phase offset for free).
            -- ============================================================
            cold_snap := '0';
            if edge_event = '1' then
                v.edge_accepted := '1';
                v.edge_lockout  := EDGE_LOCKOUT_TICKS;
                v.edge_tick_cnt := 0;

                if r.have_anchor = '0' then
                    cold_snap        := '1';
                    v.have_anchor    := '1';
                    v.last_phase_err := ZERO_PS;
                    v.filt           := ZERO_FILT;
                else
                    v.last_phase_err := signed(r.phase_acc);
                end if;
            end if;

            -- ============================================================
            -- (5) Loop filter (regular-clocked).  Scheduled K_LOOP.
            -- ============================================================
            if r.lock_state = '1' then
                k_loop_sel := K_LOOP_SLOW;
            else
                k_loop_sel := K_LOOP_FAST;
            end if;

            delta_f  := resize(v.last_phase_err, FILT_W + 1)
                      - resize(r.filt,           FILT_W + 1);
            corr_f   := shift_right(delta_f, k_loop_sel);
            filt_new := resize(r.filt, FILT_W + 1) + corr_f;
            v.filt   := resize(filt_new, FILT_W);

            -- ============================================================
            -- (6) Step computation.  step = NOMINAL_STEP - filt >> K_PLL.
            --     Clamp negative steps (only possible if K_PLL very low
            --     and filter is at extreme; included for safety).
            -- ============================================================
            correction  := shift_right(v.filt, K_PLL);
            step_signed := to_signed(NOMINAL_STEP, FILT_W) - correction;
            if step_signed < to_signed(0, FILT_W) then
                step_signed := to_signed(0, FILT_W);
            end if;
            step_us := unsigned(step_signed(PHASE_W - 1 downto 0));

            -- ============================================================
            -- (7) Phase advance (unsigned modular add of step).
            --     Carry-out = wrap = end-of-bit.
            -- ============================================================
            new_acc_ext := ('0' & r.phase_acc) + ('0' & step_us);
            wrap        := new_acc_ext(PHASE_W);

            -- ============================================================
            -- (8) Cold-snap override or natural phase advance.
            -- ============================================================
            end_of_bit := '0';
            if cold_snap = '1' then
                -- Current sample is bin 0 of new dip; v.phase_acc set to
                -- NOMINAL_STEP so next sample lands at bin ~1.28.
                phase_for_acc := ZERO_PHASE;
                v.phase_acc   := to_unsigned(NOMINAL_STEP, PHASE_W);
                v.acc_W1      := ZERO_ACC10;
                v.acc_W2      := ZERO_ACC10;
                v.acc_W3      := ZERO_ACC80;
            else
                phase_for_acc := r.phase_acc;
                v.phase_acc   := new_acc_ext(PHASE_W - 1 downto 0);
                end_of_bit    := wrap;
            end if;

            -- ============================================================
            -- (9) Accumulator update by phase_acc region.
            -- ============================================================
            if phase_for_acc < to_unsigned(W1_END_PA, PHASE_W) then
                v.acc_W1 := resize(v.acc_W1 + env_in,
                                   acc10_t'high, acc10_t'low,
                                   fixed_saturate, fixed_truncate);
            elsif phase_for_acc < to_unsigned(W2_END_PA, PHASE_W) then
                v.acc_W2 := resize(v.acc_W2 + env_in,
                                   acc10_t'high, acc10_t'low,
                                   fixed_saturate, fixed_truncate);
            else
                v.acc_W3 := resize(v.acc_W3 + env_in,
                                   acc80_t'high, acc80_t'low,
                                   fixed_saturate, fixed_truncate);
            end if;

            -- ============================================================
            -- (10) Lock detector hysteresis on |filt|.  Compare in
            --      phase_acc-scale units (= LOCK_*_THR * NOMINAL_STEP).
            -- ============================================================
            filt_pos := resize(v.filt, FILT_W + 1);
            filt_neg := - filt_pos;
            in_band  := filt_pos < to_signed(LOCK_IN_PA,  FILT_W + 1)
                    and filt_neg < to_signed(LOCK_IN_PA,  FILT_W + 1);
            out_band := filt_pos >= to_signed(LOCK_OUT_PA, FILT_W + 1)
                    or  filt_neg >= to_signed(LOCK_OUT_PA, FILT_W + 1);

            if r.lock_state = '0' then
                if in_band then
                    if r.lock_dur >= LOCK_HOLD_TICKS - 1 then
                        v.lock_state := '1';
                        v.lock_dur   := 0;
                    else
                        v.lock_dur := r.lock_dur + 1;
                    end if;
                else
                    v.lock_dur := 0;
                end if;
            else
                if out_band then
                    if r.lock_dur >= LOCK_HOLD_TICKS - 1 then
                        v.lock_state := '0';
                        v.lock_dur   := 0;
                    else
                        v.lock_dur := r.lock_dur + 1;
                    end if;
                else
                    v.lock_dur := 0;
                end if;
            end if;

            -- ============================================================
            -- (11) Edge lockout countdown (only consumed while locked).
            -- ============================================================
            if edge_event = '0' and r.edge_lockout > 0 then
                v.edge_lockout := r.edge_lockout - 1;
            end if;

            -- ============================================================
            -- (12) Give-up timer.
            -- ============================================================
            if edge_event = '0' then
                if r.edge_tick_cnt < GIVE_UP_TICKS then
                    v.edge_tick_cnt := r.edge_tick_cnt + 1;
                end if;
            end if;
            if v.edge_tick_cnt = GIVE_UP_TICKS then
                v.have_anchor    := '0';
                v.lock_state     := '0';
                v.lock_dur       := 0;
                v.filt           := ZERO_FILT;
                v.last_phase_err := ZERO_PS;
            end if;

            -- ============================================================
            -- (13) End-of-bit decision.
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

                -- mm-candidate threshold = 7 * env_max in 10-sample scale.
                -- 7 = 4 + 2 + 1 = (em<<2) + (em<<1) + em.
                em_acc := resize(r.env_max,
                                 acc10_t'high, acc10_t'low,
                                 fixed_saturate, fixed_truncate);
                em_x4  := resize(shift_left(em_acc, 2),
                                 acc10_t'high, acc10_t'low,
                                 fixed_saturate, fixed_truncate);
                em_x2  := resize(shift_left(em_acc, 1),
                                 acc10_t'high, acc10_t'low,
                                 fixed_saturate, fixed_truncate);
                mm_thr := resize(em_x4 + em_x2 + em_acc,
                                 acc10_t'high, acc10_t'low,
                                 fixed_saturate, fixed_truncate);
                candidate := '0';
                if r.acc_W1 > mm_thr then
                    candidate := '1';
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
                    if r.acc_W2 < th_new then
                        v.bit_value := '1';
                    else
                        v.bit_value := '0';
                    end if;
                    do_fire_bit := '1';
                end if;

                if r.lock_state = '1' then
                    v.bit_valid   := do_fire_bit;
                    v.minute_mark := do_fire_mm;
                end if;

                -- iir_high update (W3 is full carrier on every cycle type).
                acc_W3_d3 := resize(shift_right(r.acc_W3, 3),
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

                v.acc_W1 := ZERO_ACC10;
                v.acc_W2 := ZERO_ACC10;
                v.acc_W3 := ZERO_ACC80;

            end if;

            -- ============================================================
            -- (14) sync_q saturating "seconds locked" counter.
            -- ============================================================
            if v.lock_state = '1' then
                if r.sync_q_tick_cnt < BIT_PERIOD - 1 then
                    v.sync_q_tick_cnt := r.sync_q_tick_cnt + 1;
                else
                    v.sync_q_tick_cnt := 0;
                    if r.sync_q < SYNC_Q_MAX then
                        v.sync_q := r.sync_q + 1;
                    end if;
                end if;
            else
                v.sync_q          := 0;
                v.sync_q_tick_cnt := 0;
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
