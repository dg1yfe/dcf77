-- =========================================================================
-- dcf77_env_tracker.vhd
--
-- Envelope statistics + falling-edge detector, carved out of bit_det.
-- Updates per clock_en tick:
--
--   * env_max:  attack-fast / decay-slow IIR tracking the carrier-on
--                amplitude (Kp = 2^-K_ATK / 2^-K_DEC).
--   * env_min:  symmetric attack-fast / decay-slow tracker for the
--                AM-dip amplitude during bit '1' / minute-mark seconds.
--   * noise_iir: MAD (median absolute deviation) IIR around env_max
--                when carrier is up, env_min when carrier is down;
--                Kp = 2^-K_NOISE.
--   * carrier:  carrier-up/-down state machine with hysteresis
--                (carrier_thr = env_max/2, carrier_hys = env_max/8).
--   * falling_edge: 1-cycle pulse when a sufficiently sharp negative
--                envelope slope is detected (derivative below
--                -env_max / 2^K_SLOPE).  Debounced via slope_below
--                state so a single negative-derivative event fires
--                exactly one pulse.
--
-- All outputs are registered and present on the cycle immediately
-- following the clock_en tick they were computed from.  Same hardware
-- as the original bit_det implementation; just split out so bit_det
-- can be slimmed to ~16 record fields focused on bit-cadence logic.
-- =========================================================================

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use ieee.fixed_float_types.all;
use ieee.fixed_pkg.all;

use work.dcf77_pkg.all;

entity dcf77_env_tracker is
    generic (
        -- Slow trackers (SNR estimator) -- shifts at the clock_en rate.
        K_ATK       : positive := 8;
        K_DEC       : positive := 14;
        K_NOISE     : positive := 14;

        -- Falling-edge amplitude detector.
        DERIV_DELAY : positive := 16;
        K_SLOPE     : positive := 2
    );
    port (
        clk           : in  std_logic;
        rst           : in  std_logic;

        env_in        : in  env_word_t;
        clock_en     : in  std_logic;

        env_max       : out env_word_t;
        env_min       : out env_word_t;
        noise         : out env_word_t;
        -- 1-cycle pulse aligned with clock_en when a true falling
        -- edge crosses the slope threshold (debounced).
        falling_edge  : out std_logic
    );
end entity dcf77_env_tracker;

architecture rtl of dcf77_env_tracker is

    constant DELAY_DEPTH : natural := DERIV_DELAY + 1;
    type env_delay_t is array (0 to DELAY_DEPTH - 1) of env_word_t;

    type reg_type is record
        env_max     : env_word_t;
        env_min     : env_word_t;
        noise_iir   : env_word_t;
        carrier_thr : env_word_t;
        carrier_hys : env_word_t;
        carrier     : std_logic;
        env_dly     : env_delay_t;
        slope_below : std_logic;
    end record;

    constant ZERO_ENV : env_word_t := (others => '0');

    constant R_RESET : reg_type := (
        env_max     => ZERO_ENV,
        env_min     => ZERO_ENV,
        noise_iir   => ZERO_ENV,
        carrier_thr => ZERO_ENV,
        carrier_hys => ZERO_ENV,
        carrier     => '1',
        env_dly     => (others => ZERO_ENV),
        slope_below => '0'
    );

    signal r, rin : reg_type;

    -- Combinational falling_edge: must be aligned with the SAME clock_en
    -- tick that produced it, so bit_det's "if clock_en = '1'" gate sees
    -- it within the same cycle.  Driven by the comb process below.
    signal falling_edge_comb : std_logic;

begin

    env_max      <= r.env_max;
    env_min      <= r.env_min;
    noise        <= r.noise_iir;
    falling_edge <= falling_edge_comb;

    comb : process (all)
        variable v          : reg_type;
        variable delta_e    : sfixed(env_word_t'high + 1 downto env_word_t'low);
        variable abs_dev    : sfixed(env_word_t'high + 1 downto env_word_t'low);
        variable shifted_e  : sfixed(env_word_t'high + 1 downto env_word_t'low);
        variable th_hi      : sfixed(env_word_t'high + 1 downto env_word_t'low);
        variable th_lo      : sfixed(env_word_t'high + 1 downto env_word_t'low);
        variable new_max    : sfixed(env_word_t'high + 1 downto env_word_t'low);
        variable new_min    : sfixed(env_word_t'high + 1 downto env_word_t'low);
        variable new_noise  : sfixed(env_word_t'high + 1 downto env_word_t'low);
        variable d1         : sfixed(env_word_t'high + 1 downto env_word_t'low);
        variable slope_thr  : sfixed(env_word_t'high + 1 downto env_word_t'low);
        variable neg_thr    : sfixed(env_word_t'high + 1 downto env_word_t'low);
    begin
        v := r;
        falling_edge_comb <= '0';

        if clock_en = '1' then

            ------------------------------------------------------------
            -- env_max (attack-fast / decay-slow)
            ------------------------------------------------------------
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

            ------------------------------------------------------------
            -- env_min (symmetric)
            ------------------------------------------------------------
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

            ------------------------------------------------------------
            -- Carrier hysteresis state machine
            ------------------------------------------------------------
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

            ------------------------------------------------------------
            -- noise_iir (MAD around the relevant reference)
            ------------------------------------------------------------
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

            ------------------------------------------------------------
            -- Falling-edge detector (debounced)
            ------------------------------------------------------------
            d1 := resize(env_in - r.env_dly(DERIV_DELAY - 1),
                         d1'high, d1'low);
            slope_thr := resize(shift_right(v.env_max, K_SLOPE),
                                slope_thr'high, slope_thr'low);
            neg_thr   := resize(- slope_thr,
                                neg_thr'high, neg_thr'low);

            if slope_thr > to_sfixed(0, slope_thr'high, slope_thr'low)
               and d1 < neg_thr then
                if r.slope_below = '0' then
                    falling_edge_comb <= '1';
                end if;
                v.slope_below := '1';
            else
                v.slope_below := '0';
            end if;

            for i in DELAY_DEPTH - 1 downto 1 loop
                v.env_dly(i) := r.env_dly(i - 1);
            end loop;
            v.env_dly(0) := env_in;
        end if;

        rin <= v;
    end process;

    -- Clock-enable seq.  r updates only on clock_en='1' clock edges,
    -- so the comb process upstream re-fires when r changes = once per
    -- env tick instead of every clock cycle.  Same hardware (FF CE
    -- input on Xilinx/Intel is free); just lets GHDL skip dead cycles.
    --
    -- Reset deliberately stays in this seq process (deviating from the
    -- project's reset-in-comb convention): with the clock-enable form,
    -- a comb-side `v := R_RESET` would only reach r on clock_en='1'
    -- edges, leaving the registers un-reset during rst when the env
    -- tick is idle.
    seq : process (clk)
    begin
        if rising_edge(clk) then
            if rst = '1' then
                r <= R_RESET;
            elsif clock_en = '1' then
                r <= rin;
            end if;
        end if;
    end process;

end architecture rtl;
