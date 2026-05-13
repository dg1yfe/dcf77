-- =========================================================================
-- dcf77_env.vhd
--
-- Envelope-power computation: power = I^2 + Q^2 at 244 kS/s.  One DSP48E1
-- in MAC mode is time-shared between the two squares.  The top 18 bits of
-- each iq_word_t operand are routed into the multiplier (the bottom 4 bits
-- sit below the realistic ADC ENOB-12 noise floor anyway).
--
-- FSM:
--   IDLE  -- wait for in_valid; latch I, Q.
--   MUL_I -- compute I*I, store in acc.
--   MUL_Q -- compute Q*Q, add to acc.
--   EMIT  -- present the resized power and out_valid for one cycle.
--
-- With ~512 clock cycles per sample at 125 MHz, the 4-state path leaves
-- ample idle time; the DSP MAC handles the rest.
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

    -- Top 18 b operand for the DSP multiplier.  LSB position 4 of the
    -- 22-b iq_word_t (high 21, low 0) -> sfixed(21 downto 4).
    subtype mul_op_t is sfixed(21 downto 4);

    -- Multiplier output: sfixed(21+21+1 downto 4+4) = sfixed(43 downto 8),
    -- 36 b.  Both squares produce non-negative values; we carry the sign
    -- bit through the sfixed multiply and ignore it (it stays 0).
    subtype product_t is sfixed(43 downto 8);

    -- Accumulator: sum of two squares fits in product_t'high + 1 bits.
    subtype acc_t is sfixed(44 downto 8);

    type state_t is (S_IDLE, S_MUL_I, S_MUL_Q, S_EMIT);

    type reg_type is record
        state     : state_t;
        i_lat     : iq_word_t;
        q_lat     : iq_word_t;
        -- Shared multiplier operand and accumulator.  Vivado infers a single
        -- DSP48E1 in MAC mode for the (operand * operand + acc) expression
        -- below because the state machine guarantees only one multiply per
        -- clock and the operand register is reused.
        operand   : mul_op_t;
        acc       : acc_t;
        power     : env_word_t;
        out_valid : std_logic;
    end record;

    constant ZERO_OP  : mul_op_t  := (others => '0');
    constant ZERO_IQ  : iq_word_t := (others => '0');
    constant ZERO_ACC : acc_t     := (others => '0');
    constant ZERO_PW  : env_word_t := (others => '0');

    constant R_RESET : reg_type := (
        state     => S_IDLE,
        i_lat     => ZERO_IQ,
        q_lat     => ZERO_IQ,
        operand   => ZERO_OP,
        acc       => ZERO_ACC,
        power     => ZERO_PW,
        out_valid => '0'
    );

    signal r, rin : reg_type;

    function top18(x : iq_word_t) return mul_op_t is
    begin
        return resize(x, 21, 4, fixed_saturate, fixed_truncate);
    end function;

begin

    power     <= r.power;
    out_valid <= r.out_valid;

    comb : process (all)
        variable v       : reg_type;
        variable squared : product_t;
    begin
        v := r;
        v.out_valid := '0';

        case r.state is
            when S_IDLE =>
                if in_valid = '1' then
                    v.i_lat   := i_in;
                    v.q_lat   := q_in;
                    v.operand := top18(i_in);
                    v.acc     := ZERO_ACC;
                    v.state   := S_MUL_I;
                end if;

            when S_MUL_I =>
                -- squared <- I_top * I_top (single multiply this cycle).
                squared := resize(r.operand * r.operand,
                                  product_t'high, product_t'low,
                                  fixed_saturate, fixed_truncate);
                v.acc   := resize(squared,
                                  acc_t'high, acc_t'low,
                                  fixed_saturate, fixed_truncate);
                v.operand := top18(r.q_lat);
                v.state   := S_MUL_Q;

            when S_MUL_Q =>
                squared := resize(r.operand * r.operand,
                                  product_t'high, product_t'low,
                                  fixed_saturate, fixed_truncate);
                v.acc   := resize(r.acc + squared,
                                  acc_t'high, acc_t'low,
                                  fixed_saturate, fixed_truncate);
                v.state := S_EMIT;

            when S_EMIT =>
                -- r.acc is non-negative (sum of two squares); resize directly
                -- into env_word_t (sfixed) with saturation on overflow.
                v.power := resize(r.acc,
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
