-- =========================================================================
-- dcf77_bit_det.vhd
--
-- Adaptive-threshold pulse-width AM bit detector.  Consumes the envelope
-- power at 244.140625 kS/s and produces bit / minute-mark events.
--
-- Threshold tracking:
--   env_max is a slow IIR peak detector with fast attack (k_atk) and slow
--   decay (k_dec).  The DCF77 amplitude drops to ~15% during the absenkung,
--   so a half-peak threshold gives a comfortable separation.  Hysteresis at
--   ±env_max/8 prevents chattering on noise.
--
-- Pulse-width FSM @ 244 kS/s:
--   carrier='1' (high) -> count high duration; if it lasts beyond the
--                         per-second window, latch a pending minute mark.
--   carrier='0' (low ) -> count low duration; on the rising edge classify
--                         the low pulse as bit-0 / bit-1 / invalid and
--                         emit the pending minute mark.
--
-- Per spec §1.1: 100 ms ±10 ms -> bit 0, 200 ms ±10 ms -> bit 1.
-- At 244 140.625 ticks/s:
--   90 ms  =  21 973 ticks
--   110 ms =  26 855 ticks
--   190 ms =  46 387 ticks
--   210 ms =  51 269 ticks
--   1500 ms = 366 211 ticks  (used for minute-mark gap detect)
-- =========================================================================

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use ieee.fixed_float_types.all;
use ieee.fixed_pkg.all;

use work.dcf77_pkg.all;

entity dcf77_bit_det is
    generic (
        K_ATK         : positive := 6;        -- attack shift (1/2^K_ATK per sample)
        K_DEC         : positive := 20;       -- decay shift
        T_LOW_MIN     : positive := 21_973;   -- min low-pulse for any bit (90 ms)
        T_BIT0_MAX    : positive := 26_855;   -- bit 0 upper bound (110 ms)
        T_BIT1_MIN    : positive := 46_387;   -- bit 1 lower bound (190 ms)
        T_BIT1_MAX    : positive := 51_269;   -- bit 1 upper bound (210 ms)
        T_GAP_MINUTE  : positive := 366_211   -- minute-mark gap (1500 ms)
    );
    port (
        clk         : in  std_logic;
        rst         : in  std_logic;

        env_in      : in  env_word_t;
        env_valid   : in  std_logic;

        bit_value   : out std_logic;
        bit_valid   : out std_logic;          -- one-cycle strobe
        minute_mark : out std_logic;          -- one-cycle strobe
        sync_state  : out std_logic_vector(1 downto 0);
        env_max     : out env_word_t;
        threshold   : out env_word_t
    );
end entity dcf77_bit_det;

architecture rtl of dcf77_bit_det is

    -- Counter must cover the longest expected duration (T_GAP_MINUTE)
    -- plus headroom.
    constant T_MAX_COUNT : natural := 2 * T_GAP_MINUTE;
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
    constant CNT_W : natural := clog2(T_MAX_COUNT);

    type state_t is (S_HIGH, S_LOW);

    type reg_type is record
        -- Adaptive threshold tracker
        env_max     : env_word_t;
        threshold   : env_word_t;
        hyst        : env_word_t;
        carrier     : std_logic;          -- hysteretic envelope decision

        -- Pulse-width FSM
        state       : state_t;
        low_cnt     : unsigned(CNT_W - 1 downto 0);
        high_cnt    : unsigned(CNT_W - 1 downto 0);
        pending_mm  : std_logic;          -- minute mark pending on next low->high

        -- Outputs (registered, one-cycle strobes)
        bit_value   : std_logic;
        bit_valid   : std_logic;
        minute_mark : std_logic;
        sync_state  : std_logic_vector(1 downto 0);
    end record;

    constant ZERO_ENV : env_word_t := (others => '0');
    constant R_RESET  : reg_type := (
        env_max     => ZERO_ENV,
        threshold   => ZERO_ENV,
        hyst        => ZERO_ENV,
        carrier     => '1',
        state       => S_HIGH,
        low_cnt     => (others => '0'),
        high_cnt    => (others => '0'),
        pending_mm  => '0',
        bit_value   => '0',
        bit_valid   => '0',
        minute_mark => '0',
        sync_state  => DCF77_SYNC_SEARCH
    );

    signal r, rin : reg_type;

begin

    bit_value   <= r.bit_value;
    bit_valid   <= r.bit_valid;
    minute_mark <= r.minute_mark;
    sync_state  <= r.sync_state;
    env_max     <= r.env_max;
    threshold   <= r.threshold;

    comb : process (all)
        variable v        : reg_type;
        variable delta    : sfixed(env_word_t'high + 1 downto env_word_t'low);
        variable shifted  : sfixed(env_word_t'high + 1 downto env_word_t'low);
        variable th_hi    : sfixed(env_word_t'high + 1 downto env_word_t'low);
        variable th_lo    : sfixed(env_word_t'high + 1 downto env_word_t'low);
        variable new_max  : sfixed(env_word_t'high + 1 downto env_word_t'low);
    begin
        v := r;

        -- Default: deassert one-cycle strobes.
        v.bit_valid   := '0';
        v.minute_mark := '0';

        if env_valid = '1' then

            -----------------------------------------------------------------
            -- Adaptive peak tracker (env_max).
            -----------------------------------------------------------------
            if env_in > r.env_max then
                -- Fast attack: env_max += (env_in - env_max) >> K_ATK
                delta   := resize(env_in - r.env_max,
                                  delta'high, delta'low);
                shifted := shift_right(delta, K_ATK);
                new_max := resize(r.env_max + shifted,
                                  new_max'high, new_max'low);
            else
                -- Slow decay: env_max -= env_max >> K_DEC
                shifted := shift_right(resize(r.env_max,
                                              shifted'high, shifted'low),
                                       K_DEC);
                new_max := resize(r.env_max - shifted,
                                  new_max'high, new_max'low);
            end if;
            v.env_max := resize(new_max, env_word_t'high, env_word_t'low,
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
            -- Sync state transitions: SEARCH -> LOCK on first minute mark.
            -----------------------------------------------------------------
            if v.minute_mark = '1' then
                v.sync_state := DCF77_SYNC_LOCK;
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
