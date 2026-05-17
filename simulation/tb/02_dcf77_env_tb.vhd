-- =========================================================================
-- 02_dcf77_env_tb.vhd
--
-- VUnit testbench for dcf77_env (alpha-max + beta-min magnitude).
-- |I+jQ| ~= max(|I|,|Q|) + (3/8)*min(|I|,|Q|).
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

    -- Read the magnitude output as an integer at its native LSB position
    -- (env_word_t is sfixed(25 downto 0), so the bit pattern == the value).
    function env_int(p : env_word_t) return integer is
    begin
        return to_integer(signed(to_slv(p)));
    end function;

    -- alpha=1, beta=3/8 expected magnitude.  Integer arithmetic matches
    -- the RTL: (min >> 2) + (min >> 3), each truncating toward zero.
    function expected(i_val, q_val : integer) return integer is
        variable ia : integer := abs(i_val);
        variable qa : integer := abs(q_val);
        variable mx : integer;
        variable mn : integer;
    begin
        if ia > qa then
            mx := ia;  mn := qa;
        else
            mx := qa;  mn := ia;
        end if;
        return mx + (mn / 4) + (mn / 8);
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
            -- 2-cycle pipeline: latch then emit.  16 is comfortable headroom.
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
                check_equal(env_int(power), 0, "magnitude of (0,0)");

            -----------------------------------------------------------------
            elsif run("pure_i") then
                -- Q=0  -> magnitude = |I|
                pulse_in(65536, 0);
                exp := expected(65536, 0);
                check_equal(env_int(power), exp, "magnitude of (65536, 0)");

            -----------------------------------------------------------------
            elsif run("pure_q") then
                pulse_in(0, 65536);
                exp := expected(0, 65536);
                check_equal(env_int(power), exp, "magnitude of (0, 65536)");

            -----------------------------------------------------------------
            elsif run("equal_iq") then
                -- |I| = |Q| -> max + 3/8 * min  (within ~5.5 % of sqrt 2)
                pulse_in(32768, 32768);
                exp := expected(32768, 32768);
                check_equal(env_int(power), exp,
                            "magnitude of (32768, 32768)");

            -----------------------------------------------------------------
            elsif run("negative_i") then
                -- abs() removes sign, so (-32768, 0) == (32768, 0).
                pulse_in(-32768, 0);
                exp := expected(-32768, 0);
                check_equal(env_int(power), exp,
                            "magnitude of (-32768, 0) equals (32768, 0)");

            end if;
        end loop;

        test_runner_cleanup(runner);
        wait;
    end process;

    test_runner_watchdog(runner, 1 ms);

end architecture;
