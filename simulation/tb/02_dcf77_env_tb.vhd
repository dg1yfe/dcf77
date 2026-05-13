-- =========================================================================
-- 02_dcf77_env_tb.vhd
--
-- VUnit testbench for dcf77_env (time-shared I^2 + Q^2 power).
-- =========================================================================

library vunit_lib;
context vunit_lib.vunit_context;

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use ieee.fixed_pkg.all;

library dcf77;
use dcf77.dcf77_pkg.all;

entity dcf77_env_tb is
    generic (runner_cfg : string);
end entity;

architecture tb of dcf77_env_tb is

    constant CLK_PERIOD : time := 8 ns;

    signal clk      : std_logic := '0';
    signal rst      : std_logic := '1';
    signal i_in     : iq_word_t := (others => '0');
    signal q_in     : iq_word_t := (others => '0');
    signal in_valid : std_logic := '0';
    signal power    : env_word_t;
    signal out_valid: std_logic;

    -- Make an iq_word_t from a plain integer (LSB at 2^0).
    function iq(v : integer) return iq_word_t is
    begin
        return to_sfixed(v, iq_word_t'high, iq_word_t'low);
    end function;

    -- Read the power output as an integer at its native LSB position.
    function pwr_int(p : env_word_t) return integer is
    begin
        return to_integer(signed(to_slv(p)));
    end function;

    -- Expected power output for a given (I, Q) input pair.
    --   top18(x) = x shifted right by 4 bits, then taken as 18 b
    --   product LSB = 2^8 (LSB of mul_op = 2^4)
    --   accumulator LSB = 2^8
    --   env LSB = 2^10  -> we shift product right by 2 more
    --   So env_int = floor(top18(I)^2 + top18(Q)^2) >> 2
    function expected(i_val, q_val : integer) return integer is
        variable i_top : integer := i_val / 16;   -- top 18 b == shift right 4
        variable q_top : integer := q_val / 16;
    begin
        return (i_top * i_top + q_top * q_top) / 4;  -- shift right 2 more for LSB 2^10
    end function;

begin

    clk <= not clk after CLK_PERIOD / 2;

    dut : entity dcf77.dcf77_env
        port map (
            clk       => clk,
            rst       => rst,
            i_in      => i_in,
            q_in      => q_in,
            in_valid  => in_valid,
            power     => power,
            out_valid => out_valid
        );

    test_runner : process
        procedure pulse_in(i_val, q_val : integer) is
        begin
            i_in     <= iq(i_val);
            q_in     <= iq(q_val);
            in_valid <= '1';
            wait until rising_edge(clk);
            in_valid <= '0';
            i_in <= (others => '0');
            q_in <= (others => '0');
            -- Wait for out_valid (FSM takes 3 clocks after in_valid -> emit).
            for i in 1 to 16 loop
                wait until rising_edge(clk);
                exit when out_valid = '1';
            end loop;
        end procedure;

        variable exp : integer;
    begin
        test_runner_setup(runner, runner_cfg);

        rst <= '1';
        wait for 4 * CLK_PERIOD;
        wait until rising_edge(clk);
        rst <= '0';
        wait until rising_edge(clk);

        while test_suite loop

            -----------------------------------------------------------------
            if run("zero") then
                pulse_in(0, 0);
                check_equal(pwr_int(power), 0, "power of (0,0)");

            -----------------------------------------------------------------
            elsif run("pure_i") then
                -- I=2^16, Q=0  -> top18(I)=2^12, top18(Q)=0
                -- (2^12)^2 = 2^24 -> /4 -> 2^22
                pulse_in(65536, 0);
                exp := expected(65536, 0);
                check_equal(pwr_int(power), exp, "power of (65536, 0)");

            -----------------------------------------------------------------
            elsif run("pure_q") then
                pulse_in(0, 65536);
                exp := expected(0, 65536);
                check_equal(pwr_int(power), exp, "power of (0, 65536)");

            -----------------------------------------------------------------
            elsif run("equal_iq") then
                pulse_in(32768, 32768);
                exp := expected(32768, 32768);
                check_equal(pwr_int(power), exp, "power of (32768, 32768)");

            -----------------------------------------------------------------
            elsif run("negative_i") then
                -- Squaring removes sign, so (-32768, 0) == (32768, 0).
                pulse_in(-32768, 0);
                exp := expected(-32768, 0);
                check_equal(pwr_int(power), exp,
                            "power of (-32768, 0) equals power of (32768, 0)");

            end if;
        end loop;

        test_runner_cleanup(runner);
        wait;
    end process;

    test_runner_watchdog(runner, 1 ms);

end architecture;
