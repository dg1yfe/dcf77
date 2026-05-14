-- =========================================================================
-- 03_dcf77_bit_det_tb.vhd
--
-- VUnit testbench for dcf77_bit_det.  Uses scaled-down timing generics so
-- each test runs in tens of milliseconds of simulator time while still
-- exercising the same FSM logic that runs against real DCF77 ticks.
--
--   T_BIT0 window: 80..110 ticks  (mid-point 100)
--   T_BIT1 window: 180..220 ticks (mid-point 200)
--   minute-mark gap: >= 1500 ticks
--
-- Tests:
--   bit0       : 100-tick low pulse -> bit_valid=1, bit_value='0'
--   bit1       : 200-tick low pulse -> bit_valid=1, bit_value='1'
--   too_short  : 50-tick low pulse  -> bit_valid stays '0'
--   minute     : long high gap then a low pulse -> minute_mark='1' on the
--                bit_valid transition
-- =========================================================================

library vunit_lib;
context vunit_lib.vunit_context;

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use ieee.fixed_pkg.all;

library dcf77;
use dcf77.dcf77_pkg.all;

entity dcf77_bit_det_tb is
    generic (runner_cfg : string);
end entity;

architecture tb of dcf77_bit_det_tb is

    constant CLK_PERIOD : time := 8 ns;

    -- Scaled-down timing.
    constant TG_LOW_MIN    : positive := 80;
    constant TG_BIT0_MAX   : positive := 110;
    constant TG_BIT1_MIN   : positive := 180;
    constant TG_BIT1_MAX   : positive := 220;
    constant TG_GAP_MINUTE : positive := 1500;
    -- Faster attack/decay so env_max settles in the short test window.
    constant TG_K_ATK : positive := 2;
    constant TG_K_DEC : positive := 14;

    -- Envelope magnitudes.  Carrier-high ~ ENV_HIGH, absenkung ~ ENV_LOW.
    function env(v : integer) return env_word_t is
    begin
        return to_sfixed(v, env_word_t'high, env_word_t'low);
    end function;
    constant ENV_HIGH : env_word_t := env(1_000_000);
    constant ENV_LOW  : env_word_t := env(  150_000);

    signal clk         : std_logic := '0';
    signal rst         : std_logic := '1';
    signal env_in      : env_word_t := (others => '0');
    signal env_valid   : std_logic := '0';

    signal bit_value   : std_logic;
    signal bit_valid   : std_logic;
    signal minute_mark : std_logic;
    signal sync_state  : sync_state_t;
    signal env_max_o   : env_word_t;
    signal env_min_o   : env_word_t;
    signal noise_o     : env_word_t;
    signal threshold_o : env_word_t;

begin

    clk <= not clk after CLK_PERIOD / 2;

    dut : entity dcf77.dcf77_bit_det
        generic map (
            K_ATK        => TG_K_ATK,
            K_DEC        => TG_K_DEC,
            T_LOW_MIN    => TG_LOW_MIN,
            T_BIT0_MAX   => TG_BIT0_MAX,
            T_BIT1_MIN   => TG_BIT1_MIN,
            T_BIT1_MAX   => TG_BIT1_MAX,
            T_GAP_MINUTE => TG_GAP_MINUTE
        )
        port map (
            clk         => clk,
            rst         => rst,
            env_in      => env_in,
            env_valid   => env_valid,
            bit_value   => bit_value,
            bit_valid   => bit_valid,
            minute_mark => minute_mark,
            sync_state  => sync_state,
            env_max     => env_max_o,
            env_min     => env_min_o,
            noise       => noise_o,
            threshold   => threshold_o
        );

    test_runner : process

        -- Drive `n` ticks of env_in = level (env_valid='1' once per clk).
        procedure drive_env(level : env_word_t; n : positive) is
        begin
            env_in    <= level;
            env_valid <= '1';
            for i in 1 to n loop
                wait until rising_edge(clk);
            end loop;
            env_valid <= '0';
        end procedure;

        -- Drive `n` ticks while watching for bit_valid / minute_mark events.
        procedure drive_env_capture(level : env_word_t;
                                    n     : positive;
                                    seen_bit, seen_mm : out boolean;
                                    captured_bit      : out std_logic) is
            variable b : boolean := false;
            variable m : boolean := false;
            variable c : std_logic := 'X';
        begin
            env_in    <= level;
            env_valid <= '1';
            for i in 1 to n loop
                wait until rising_edge(clk);
                if bit_valid = '1' then
                    b := true;
                    c := bit_value;
                end if;
                if minute_mark = '1' then
                    m := true;
                end if;
            end loop;
            env_valid <= '0';
            seen_bit     := b;
            seen_mm      := m;
            captured_bit := c;
        end procedure;

        variable seen_bit, seen_mm : boolean;
        variable cap_bit : std_logic;

    begin
        test_runner_setup(runner, runner_cfg);

        rst <= '1';
        wait for 4 * CLK_PERIOD;
        wait until rising_edge(clk);
        rst <= '0';
        wait until rising_edge(clk);

        while test_suite loop

            -----------------------------------------------------------------
            if run("bit0") then
                -- Prime env_max with the HIGH level.
                drive_env(ENV_HIGH, 600);
                -- 100-tick low pulse, then back to high.
                drive_env(ENV_LOW, 100);
                drive_env_capture(ENV_HIGH, 200, seen_bit, seen_mm, cap_bit);
                check(seen_bit, "bit_valid asserted for 100-tick low pulse");
                check_equal(cap_bit, '0', "bit_value = 0");

            -----------------------------------------------------------------
            elsif run("bit1") then
                drive_env(ENV_HIGH, 600);
                drive_env(ENV_LOW, 200);
                drive_env_capture(ENV_HIGH, 200, seen_bit, seen_mm, cap_bit);
                check(seen_bit, "bit_valid asserted for 200-tick low pulse");
                check_equal(cap_bit, '1', "bit_value = 1");

            -----------------------------------------------------------------
            elsif run("too_short") then
                drive_env(ENV_HIGH, 600);
                drive_env(ENV_LOW, 50);
                drive_env_capture(ENV_HIGH, 200, seen_bit, seen_mm, cap_bit);
                check(not seen_bit, "bit_valid stays low for sub-90 tick pulse");

            -----------------------------------------------------------------
            elsif run("minute_mark") then
                -- Bring up carrier, then have a 1600-tick high gap (no low
                -- pulse) followed by a normal 100-tick low pulse.  Minute mark
                -- should fire on the bit_valid event that closes this pulse.
                drive_env(ENV_HIGH, 600);
                -- First low pulse (any short pulse just to set state).
                drive_env(ENV_LOW, 100);
                drive_env(ENV_HIGH, 1600);   -- long gap >= 1500 ticks
                drive_env(ENV_LOW, 100);
                drive_env_capture(ENV_HIGH, 200, seen_bit, seen_mm, cap_bit);
                check(seen_bit, "bit_valid asserted after minute-mark gap");
                check(seen_mm,  "minute_mark asserted on bit after long gap");

            end if;
        end loop;

        test_runner_cleanup(runner);
        wait;
    end process;

    test_runner_watchdog(runner, 10 ms);

end architecture;
