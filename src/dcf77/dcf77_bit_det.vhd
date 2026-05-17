-- =========================================================================
-- dcf77_bit_det.vhd
--
-- Adaptive-threshold pulse-width AM bit detector.  Consumes the envelope
-- power at 1 kS/s (post-mixer / post-CIC3+CIC4) and produces bit /
-- minute-mark events.
--
-- Threshold tracking:
--   env_max is an IIR peak detector with fast attack (K_ATK) and slow
--   decay (K_DEC).  env_min is the mirrored trough tracker (fast attack
--   on low excursions, slow drift back up).  The DCF77 amplitude drops
--   to ~15 % during the absenkung; the threshold sits midway between
--   env_max and env_min in effect, with ±env_max/8 hysteresis.
--
--   A noise estimate (noise_iir) tracks the running mean-absolute
--   deviation of env_in from the currently-tracked level (env_max while
--   carrier='1', env_min while carrier='0').  The host can compute
--   SNR_dB ≈ 20·log10((env_max - env_min)/noise_iir).
--
-- Pulse-width FSM @ 1 kS/s:
--   carrier='1' (high) -> count high duration; if it lasts beyond the
--                         per-second window, latch a pending minute mark.
--   carrier='0' (low ) -> count low duration; on the rising edge classify
--                         the low pulse as bit-0 / bit-1 / invalid and
--                         emit the pending minute mark.
--
-- Per spec §1.1: 100 ms ±10 ms -> bit 0, 200 ms ±10 ms -> bit 1.
-- At 1 000 ticks/s:
--    90 ms =    90 ticks
--   110 ms =   110 ticks
--   190 ms =   190 ticks
--   210 ms =   210 ticks
--  1500 ms =  1500 ticks   (minute-mark gap)
-- =========================================================================

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use ieee.fixed_float_types.all;
use ieee.fixed_pkg.all;

use work.dcf77_pkg.all;

entity dcf77_bit_det is
    generic (
        K_ATK         : positive := 3;          -- env_max/min attack shift (~80 ms @ 100 S/s)
        K_DEC         : positive := 9;          -- env_max/min decay shift (~5 s @ 100 S/s)
        K_NOISE       : positive := 9;          -- noise-MAD IIR shift (~5 s @ 100 S/s)
        -- Bit-pulse-width thresholds in env_valid ticks.  At 100 S/s, 1 tick = 10 ms.
        -- The CIC_ENV (R=10, N=3) impulse response is ~27 input samples wide
        -- = ~2.7 output samples, so a rectangular 100 ms dip arrives as an
        -- 8-10 tick pulse and a 200 ms dip as 18-20 ticks.  Windows widened
        -- accordingly with a guard gap in [13..14] between bit-0 and bit-1.
        T_LOW_MIN     : positive := 6;          -- min low-pulse for any bit (60 ms)
        T_BIT0_MAX    : positive := 13;         -- bit 0 upper bound (130 ms)
        T_BIT1_MIN    : positive := 14;         -- bit 1 lower bound (140 ms)
        T_BIT1_MAX    : positive := 23;         -- bit 1 upper bound (230 ms)
        T_GAP_MINUTE  : positive := 150;        -- minute-mark gap (1500 ms)
        -- Sync-state demotion is on minute-scale: defining the thresholds
        -- in seconds keeps the numbers human-readable.  The actual tick
        -- counts are SAMPLE_RATE_HZ * the relevant seconds value.
        SAMPLE_RATE_HZ : positive := 100;       -- env_valid rate at the real Fs
        T_HOLD_SEC     : positive := 120;       -- lock -> hold (~2 min)
        T_SEARCH_SEC   : positive := 300        -- any -> search (~5 min)
    );
    port (
        clk         : in  std_logic;
        rst         : in  std_logic;

        env_in      : in  env_word_t;
        env_valid   : in  std_logic;

        bit_value   : out std_logic;
        bit_valid   : out std_logic;          -- one-cycle strobe
        minute_mark : out std_logic;          -- one-cycle strobe
        sync_state  : out sync_state_t;
        env_max     : out env_word_t;
        env_min     : out env_word_t;
        noise       : out env_word_t;
        threshold   : out env_word_t
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

    -- Counter must cover the longest expected pulse-width measurement,
    -- (T_GAP_MINUTE) plus headroom.
    constant T_MAX_COUNT : natural := 2 * T_GAP_MINUTE;
    constant CNT_W       : natural := clog2(T_MAX_COUNT);

    -- Derived tick counts for the sync-state thresholds.
    constant T_HOLD_TICKS   : positive := T_HOLD_SEC   * SAMPLE_RATE_HZ;
    constant T_SEARCH_TICKS : positive := T_SEARCH_SEC * SAMPLE_RATE_HZ;
    constant SYNC_CNT_W     : natural  := clog2(T_SEARCH_TICKS + 1);

    type state_t is (S_HIGH, S_LOW);

    type reg_type is record
        -- Adaptive threshold trackers and noise estimate
        env_max     : env_word_t;
        env_min     : env_word_t;
        noise_iir   : env_word_t;
        threshold   : env_word_t;
        hyst        : env_word_t;
        carrier     : std_logic;          -- hysteretic envelope decision

        -- Pulse-width FSM
        state       : state_t;
        low_cnt     : unsigned(CNT_W - 1 downto 0);
        high_cnt    : unsigned(CNT_W - 1 downto 0);
        pending_mm  : std_logic;          -- minute mark pending on next low->high

        -- Time since the last minute-mark, in env_valid ticks.  Saturating
        -- counter driving the sync state machine.
        sync_cnt    : unsigned(SYNC_CNT_W - 1 downto 0);

        -- Outputs (registered, one-cycle strobes)
        bit_value   : std_logic;
        bit_valid   : std_logic;
        minute_mark : std_logic;
        sync_state  : sync_state_t;
    end record;

    constant ZERO_ENV : env_word_t := (others => '0');
    constant R_RESET  : reg_type := (
        env_max     => ZERO_ENV,
        env_min     => ZERO_ENV,
        noise_iir   => ZERO_ENV,
        threshold   => ZERO_ENV,
        hyst        => ZERO_ENV,
        carrier     => '1',
        state       => S_HIGH,
        low_cnt     => (others => '0'),
        high_cnt    => (others => '0'),
        pending_mm  => '0',
        sync_cnt    => (others => '0'),
        bit_value   => '0',
        bit_valid   => '0',
        minute_mark => '0',
        sync_state  => sync_search
    );

    signal r, rin : reg_type;

begin

    bit_value   <= r.bit_value;
    bit_valid   <= r.bit_valid;
    minute_mark <= r.minute_mark;
    sync_state  <= r.sync_state;
    env_max     <= r.env_max;
    env_min     <= r.env_min;
    noise       <= r.noise_iir;
    threshold   <= r.threshold;

    comb : process (all)
        variable v        : reg_type;
        variable delta    : sfixed(env_word_t'high + 1 downto env_word_t'low);
        variable abs_dev  : sfixed(env_word_t'high + 1 downto env_word_t'low);
        variable shifted  : sfixed(env_word_t'high + 1 downto env_word_t'low);
        variable th_hi    : sfixed(env_word_t'high + 1 downto env_word_t'low);
        variable th_lo    : sfixed(env_word_t'high + 1 downto env_word_t'low);
        variable new_max  : sfixed(env_word_t'high + 1 downto env_word_t'low);
        variable new_min  : sfixed(env_word_t'high + 1 downto env_word_t'low);
        variable new_noise: sfixed(env_word_t'high + 1 downto env_word_t'low);
    begin
        v := r;

        -- Default: deassert one-cycle strobes.
        v.bit_valid   := '0';
        v.minute_mark := '0';

        if env_valid = '1' then

            -----------------------------------------------------------------
            -- Adaptive peak tracker (env_max): fast attack toward larger,
            -- slow decay toward zero.
            -----------------------------------------------------------------
            if env_in > r.env_max then
                delta   := resize(env_in - r.env_max,
                                  delta'high, delta'low);
                shifted := shift_right(delta, K_ATK);
                new_max := resize(r.env_max + shifted,
                                  new_max'high, new_max'low);
            else
                shifted := shift_right(resize(r.env_max,
                                              shifted'high, shifted'low),
                                       K_DEC);
                new_max := resize(r.env_max - shifted,
                                  new_max'high, new_max'low);
            end if;
            v.env_max := resize(new_max, env_word_t'high, env_word_t'low,
                                fixed_saturate, fixed_truncate);

            -----------------------------------------------------------------
            -- Adaptive trough tracker (env_min): fast attack toward smaller,
            -- slow drift back up toward env_max.
            -----------------------------------------------------------------
            if env_in < r.env_min then
                delta   := resize(r.env_min - env_in,
                                  delta'high, delta'low);
                shifted := shift_right(delta, K_ATK);
                new_min := resize(r.env_min - shifted,
                                  new_min'high, new_min'low);
            else
                delta   := resize(r.env_max - r.env_min,
                                  delta'high, delta'low);
                shifted := shift_right(delta, K_DEC);
                new_min := resize(r.env_min + shifted,
                                  new_min'high, new_min'low);
            end if;
            v.env_min := resize(new_min, env_word_t'high, env_word_t'low,
                                fixed_saturate, fixed_truncate);

            -----------------------------------------------------------------
            -- Noise estimator: running mean-absolute-deviation from the
            -- currently-tracked level (env_max while carrier='1',
            -- env_min while carrier='0').  Slow IIR with shift K_NOISE.
            -----------------------------------------------------------------
            if r.carrier = '1' then
                delta := resize(env_in - r.env_max,
                                delta'high, delta'low);
            else
                delta := resize(env_in - r.env_min,
                                delta'high, delta'low);
            end if;
            if delta(delta'high) = '1' then
                abs_dev := resize(- delta, abs_dev'high, abs_dev'low);
            else
                abs_dev := delta;
            end if;
            shifted   := shift_right(resize(r.noise_iir,
                                            shifted'high, shifted'low),
                                     K_NOISE);
            new_noise := resize(r.noise_iir +
                                resize(shift_right(abs_dev, K_NOISE),
                                       new_noise'high, new_noise'low) -
                                shifted,
                                new_noise'high, new_noise'low);
            v.noise_iir := resize(new_noise, env_word_t'high, env_word_t'low,
                                  fixed_saturate, fixed_truncate);

            -- Derive threshold and hysteresis from the updated env_max.
            v.threshold := resize(shift_right(v.env_max, 1),
                                  env_word_t'high, env_word_t'low,
                                  fixed_saturate, fixed_truncate);
            v.hyst      := resize(shift_right(v.env_max, 3),
                                  env_word_t'high, env_word_t'low,
                                  fixed_saturate, fixed_truncate);

            -----------------------------------------------------------------
            -- Hysteretic carrier decision.
            -----------------------------------------------------------------
            th_hi := resize(v.threshold + v.hyst, th_hi'high, th_hi'low);
            th_lo := resize(v.threshold - v.hyst, th_lo'high, th_lo'low);
            if env_in >= th_hi then
                v.carrier := '1';
            elsif env_in <= th_lo then
                v.carrier := '0';
            else
                v.carrier := r.carrier;
            end if;

            -----------------------------------------------------------------
            -- Pulse-width FSM.
            -----------------------------------------------------------------
            case r.state is

                when S_HIGH =>
                    if v.carrier = '0' and r.carrier = '1' then
                        -- Falling edge: enter low pulse.
                        v.state   := S_LOW;
                        v.low_cnt := (others => '0');
                        if r.high_cnt >= to_unsigned(T_GAP_MINUTE, CNT_W) then
                            v.pending_mm := '1';
                        end if;
                        v.high_cnt := (others => '0');
                    else
                        if r.high_cnt < to_unsigned(T_MAX_COUNT, CNT_W) then
                            v.high_cnt := r.high_cnt + 1;
                        end if;
                    end if;

                when S_LOW =>
                    if v.carrier = '1' and r.carrier = '0' then
                        -- Rising edge: classify low pulse.
                        v.state    := S_HIGH;
                        v.high_cnt := (others => '0');
                        if r.low_cnt >= to_unsigned(T_LOW_MIN, CNT_W) and
                           r.low_cnt <= to_unsigned(T_BIT0_MAX, CNT_W) then
                            v.bit_value := '0';
                            v.bit_valid := '1';
                        elsif r.low_cnt >= to_unsigned(T_BIT1_MIN, CNT_W) and
                              r.low_cnt <= to_unsigned(T_BIT1_MAX, CNT_W) then
                            v.bit_value := '1';
                            v.bit_valid := '1';
                        end if;
                        if r.pending_mm = '1' then
                            v.minute_mark := '1';
                            v.pending_mm  := '0';
                        end if;
                    else
                        if r.low_cnt < to_unsigned(T_MAX_COUNT, CNT_W) then
                            v.low_cnt := r.low_cnt + 1;
                        end if;
                    end if;

            end case;

            -----------------------------------------------------------------
            -- Sync state machine.
            --   * Every minute_mark resets sync_cnt to zero and pulls the
            --     state into sync_lock.
            --   * If sync_cnt exceeds T_HOLD_TICKS without a minute_mark,
            --     drop from sync_lock to sync_hold (signal recently degraded).
            --   * If it exceeds T_SEARCH_TICKS, drop to sync_search (lost).
            -----------------------------------------------------------------
            if v.minute_mark = '1' then
                v.sync_cnt   := (others => '0');
                v.sync_state := sync_lock;
            else
                if r.sync_cnt < to_unsigned(T_SEARCH_TICKS, SYNC_CNT_W) then
                    v.sync_cnt := r.sync_cnt + 1;
                end if;
                if r.sync_cnt >= to_unsigned(T_SEARCH_TICKS, SYNC_CNT_W) then
                    v.sync_state := sync_search;
                elsif r.sync_cnt >= to_unsigned(T_HOLD_TICKS, SYNC_CNT_W)
                      and r.sync_state = sync_lock then
                    v.sync_state := sync_hold;
                end if;
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
