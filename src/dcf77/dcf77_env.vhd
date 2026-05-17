-- =========================================================================
-- dcf77_env.vhd
--
-- AM envelope magnitude via the "alpha max + beta min" approximation:
--     |I+jQ| ~= alpha * max(|I|,|Q|) + beta * min(|I|,|Q|)
-- with alpha = 1, beta = 3/8.  Max approximation error ~3.5 % at 22.5 deg,
-- which sits well below the noise floor of every signal in this pipeline.
--
-- Replaces the earlier I*I + Q*Q ("power") implementation.  Motivation:
--   * Squaring doubles the envelope bandwidth (cross-terms at 2*f_mod),
--     so a steep CIC_ENV at 100 S/s aliases the upper half of the power
--     envelope's spectrum.  Magnitude keeps the original ~50 Hz baseband.
--   * No multiplier.  The previous power calc time-shared one DSP48E1
--     between I*I and Q*Q; this version is pure abs/compare/shift/add,
--     freeing that DSP slice entirely.
--   * Noise on a single carrier sample is amplitude-additive on
--     magnitude but variance-additive on power, so the magnitude form
--     produces a flatter noise floor and cleaner dip discrimination.
--
-- Pipeline (single-cycle compute, two-cycle latency):
--   S_IDLE -- on in_valid, latch I, Q, advance.
--   S_EMIT -- compute |I|, |Q|, pick max/min, magnitude = max + (3*min)/8,
--             present out_valid for one cycle, back to IDLE.
-- =========================================================================

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use ieee.fixed_float_types.all;
use ieee.fixed_pkg.all;

use work.dcf77_pkg.all;

entity dcf77_env is
    port (
        clk        : in  std_logic;
        rst        : in  std_logic;

        i_in       : in  iq_word_t;
        q_in       : in  iq_word_t;
        in_valid   : in  std_logic;

        power      : out env_word_t;
        out_valid  : out std_logic
    );
end entity dcf77_env;

architecture rtl of dcf77_env is

    -- abs() on sfixed grows by one MSB (the corner case
    -- -2^(N-1) becomes +2^(N-1), which doesn't fit in the original width).
    subtype abs_t is sfixed(iq_word_t'high + 1 downto iq_word_t'low);

    -- (1 + 3/8)*2^N peak; +1 b headroom over max(|I|,|Q|).
    subtype mag_t is sfixed(iq_word_t'high + 2 downto iq_word_t'low);

    type state_t is (S_IDLE, S_EMIT);

    type reg_type is record
        state     : state_t;
        i_lat     : iq_word_t;
        q_lat     : iq_word_t;
        power     : env_word_t;
        out_valid : std_logic;
    end record;

    constant ZERO_IQ  : iq_word_t  := (others => '0');
    constant ZERO_ENV : env_word_t := (others => '0');

    constant R_RESET : reg_type := (
        state     => S_IDLE,
        i_lat     => ZERO_IQ,
        q_lat     => ZERO_IQ,
        power     => ZERO_ENV,
        out_valid => '0'
    );

    signal r, rin : reg_type;

begin

    power     <= r.power;
    out_valid <= r.out_valid;

    comb : process (all)
        variable v        : reg_type;
        variable i_abs    : abs_t;
        variable q_abs    : abs_t;
        variable mx       : abs_t;
        variable mn       : abs_t;
        variable mag      : mag_t;
    begin
        v := r;
        v.out_valid := '0';

        case r.state is
            when S_IDLE =>
                if in_valid = '1' then
                    v.i_lat := i_in;
                    v.q_lat := q_in;
                    v.state := S_EMIT;
                end if;

            when S_EMIT =>
                -- |I|, |Q|.  fixed_pkg's abs grows the width by one MSB.
                i_abs := abs(r.i_lat);
                q_abs := abs(r.q_lat);

                -- max(|I|,|Q|), min(|I|,|Q|).
                if i_abs > q_abs then
                    mx := i_abs;
                    mn := q_abs;
                else
                    mx := q_abs;
                    mn := i_abs;
                end if;

                -- alpha=1, beta=3/8.  3/8 = 1/4 + 1/8, so beta*min is
                -- (min >> 2) + (min >> 3) -- two free shifts, one add.
                -- Sum needs 1 b headroom above max; mag_t provides it.
                mag := resize(mx
                              + shift_right(mn, 2)
                              + shift_right(mn, 3),
                              mag_t'high, mag_t'low,
                              fixed_saturate, fixed_truncate);

                -- env_word_t holds the magnitude directly at LSB 2^0.
                v.power := resize(mag,
                                  env_word_t'high, env_word_t'low,
                                  fixed_saturate, fixed_truncate);
                v.out_valid := '1';
                v.state := S_IDLE;

        end case;

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
