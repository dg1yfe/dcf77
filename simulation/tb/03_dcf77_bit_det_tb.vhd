-- =========================================================================
-- 03_dcf77_bit_det_tb.vhd
--
-- VUnit testbench for the single-phase symbol-structural bit detector.
-- Drives sequences of synthetic 100-tick "second" cycles at env_valid='1'
-- and watches bit_valid / minute_mark / sync_quality / mm_quality outputs.
--
-- Tests:
--   bit0          : repeated 10/90 dip-carrier pattern -> bit_value=0
--   bit1          : repeated 20/80 dip-carrier pattern -> bit_value=1
--   mm_cold_start : a single no-dip "gap" cycle -> minute_mark fires
--   mm_confirm    : 60 bit-0 cycles around a gap cycle -> mm_q climbs
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

    -- Detector defaults are 100 S/s; keep them.  Drive each test bit with
    -- BIT_PERIOD = 100 env_valid ticks (= one "second" at the design rate).
    constant BIT_PERIOD : positive := 100;
    constant DIP_BIT0   : positive := 10;     -- 100 ms equivalent
    constant DIP_BIT1   : positive := 20;     -- 200 ms equivalent

    -- Envelope magnitudes.  Carrier ~ ENV_HIGH, absenkung ~ 0.15 * HIGH
    -- (matches DCF77 spec; the structural threshold sits at 0.578 of HIGH).
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

    signal bit_value     : std_logic;
    signal bit_valid     : std_logic;
    signal minute_mark   : std_logic;
    signal sync_state    : sync_state_t;
    signal sync_quality  : unsigned(5 downto 0);
    signal mm_quality    : unsigned(1 downto 0);
    signal cycle_in_min  : unsigned(5 downto 0);
    signal env_max_o     : env_word_t;
    signal env_min_o     : env_word_t;
    signal noise_o       : env_word_t;
    signal threshold_o   : env_word_t;

begin

    clk <= not clk after CLK_PERIOD / 2;

    -- Keep this TB on the legacy 100 S/s timings even though bit_det's
    -- production defaults moved to 2 kS/s.  The dip / warm-up stimulus
    -- in this TB is sized for 100-tick bit cycles.
    dut : entity dcf77.dcf77_bit_det
        generic map (
            K_ATK          => 3,
            K_DEC          => 9,
            K_NOISE        => 9,
            DERIV_DELAY    => 2,
            K_SLOPE        => 2,
            K_REF          => 2,
            BIT_PERIOD     => 100,
            W1_END         => 10,
            W2_END         => 20,
            W3_END         => 100,
            GIVE_UP_TICKS  => 1000,
            SAMPLE_RATE_HZ => 100
        )
        port map (
            clk           => clk,
            rst           => rst,
            env_in        => env_in,
            env_valid     => env_valid,
            bit_value     => bit_value,
            bit_valid     => bit_valid,
            minute_mark   => minute_mark,
            sync_state    => sync_state,
            sync_quality  => sync_quality,
            mm_quality    => mm_quality,
            cycle_in_min  => cycle_in_min,
            env_max       => env_max_o,
            env_min       => env_min_o,
            noise         => noise_o,
            threshold     => threshold_o,
            edge_accepted => open
        );

    test_runner : process

        -- Drive a single bit cycle: `dip_len` samples at ENV_LOW followed
        -- by (BIT_PERIOD - dip_len) samples at ENV_HIGH.  dip_len=0 means
        -- "no dip" (minute-mark candidate).
        procedure drive_cycle(dip_len : natural;
                              n_seen_bit, n_seen_mm : inout natural;
                              last_bit              : inout std_logic) is
        begin
            for i in 1 to BIT_PERIOD loop
                if i <= dip_len then
                    env_in <= ENV_LOW;
                else
                    env_in <= ENV_HIGH;
                end if;
                env_valid <= '1';
                wait until rising_edge(clk);
                if bit_valid = '1' then
                    n_seen_bit := n_seen_bit + 1;
                    last_bit   := bit_value;
                end if;
                if minute_mark = '1' then
                    n_seen_mm := n_seen_mm + 1;
                end if;
            end loop;
            -- Settle: one extra clock so the registered output of
            -- sample BIT_PERIOD's end-of-bit is observable.  env_valid='0'
            -- so the bit_det doesn't process another sample.
            env_valid <= '0';
            wait until rising_edge(clk);
            if bit_valid = '1' then
                n_seen_bit := n_seen_bit + 1;
                last_bit   := bit_value;
            end if;
            if minute_mark = '1' then
                n_seen_mm := n_seen_mm + 1;
            end if;
        end procedure;

        -- Drive N copies of the same cycle, throwing away events (warm-up).
        procedure drive_cycles(dip_len : natural; n : natural) is
            variable nb, nm : natural := 0;
            variable lb     : std_logic := 'X';
        begin
            for k in 1 to n loop
                drive_cycle(dip_len, nb, nm, lb);
            end loop;
        end procedure;

        variable n_bits   : natural := 0;
        variable n_mms    : natural := 0;
        variable last_bit : std_logic := 'X';

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
                -- 20 bit-0 cycles to warm up env_max and iir_high; on the
                -- last few the structural detector should classify bit-0.
                n_bits := 0; n_mms := 0;
                for k in 1 to 20 loop
                    drive_cycle(DIP_BIT0, n_bits, n_mms, last_bit);
                end loop;
                check(n_bits >= 10, "at least 10 bit_valid pulses fired in 20 bit-0 cycles");
                check_equal(last_bit, '0', "last bit_value was 0");

            -----------------------------------------------------------------
            elsif run("bit1") then
                n_bits := 0; n_mms := 0;
                for k in 1 to 20 loop
                    drive_cycle(DIP_BIT1, n_bits, n_mms, last_bit);
                end loop;
                check(n_bits >= 10, "at least 10 bit_valid pulses fired in 20 bit-1 cycles");
                check_equal(last_bit, '1', "last bit_value was 1");

            -----------------------------------------------------------------
            elsif run("mm_cold_start") then
                -- 20 bit-0 warm-up cycles, then one "no dip" gap cycle.
                -- The gap fires the cold-start minute_mark (counter 0 -> 1).
                n_bits := 0; n_mms := 0;
                for k in 1 to 20 loop
                    drive_cycle(DIP_BIT0, n_bits, n_mms, last_bit);
                end loop;
                info("after 20 warm-up: n_bits=" & integer'image(n_bits)
                     & " n_mms=" & integer'image(n_mms)
                     & " mm_q=" & integer'image(to_integer(mm_quality))
                     & " cycle=" & integer'image(to_integer(cycle_in_min))
                     & " sync_q=" & integer'image(to_integer(sync_quality)));
                n_bits := 0; n_mms := 0;
                drive_cycle(0, n_bits, n_mms, last_bit);
                info("after gap: n_bits=" & integer'image(n_bits)
                     & " n_mms=" & integer'image(n_mms)
                     & " mm_q=" & integer'image(to_integer(mm_quality))
                     & " cycle=" & integer'image(to_integer(cycle_in_min)));
                check(n_mms >= 1, "minute_mark fired on no-dip cycle");
                check_equal(n_bits, 0, "no bit_valid in the mm cycle");

            -----------------------------------------------------------------
            elsif run("mm_confirm") then
                -- Bring up sync, fire one cold-start mm, then drive a full
                -- minute (60 cycles, with bit-0 in 0..58 and no-dip in 59).
                -- The second mm should fire on the predicted gap second and
                -- mm_quality should climb past 1.
                drive_cycles(DIP_BIT0, 20);

                -- First gap cycle to anchor (counter 0 -> 1).
                n_bits := 0; n_mms := 0;
                drive_cycle(0, n_bits, n_mms, last_bit);
                check(n_mms = 1, "anchor mm fired");

                -- 59 bit-0 cycles (cycle_in_min 0..58).
                for k in 1 to 59 loop
                    drive_cycle(DIP_BIT0, n_bits, n_mms, last_bit);
                end loop;

                -- 60th cycle: predicted second 59, no dip -> mm fires AND
                -- counter climbs to 2.
                drive_cycle(0, n_bits, n_mms, last_bit);
                check(n_mms = 2, "predicted-sec-59 mm fired");
                check(mm_quality >= "10",
                      "mm_quality climbs after second confirmation");

            end if;
        end loop;

        test_runner_cleanup(runner);
        wait;
    end process;

    -- 60 cycles * 100 ticks * 8 ns = 48 us per minute; mm_confirm runs
    -- ~80 cycles total at 100 ticks each = 64 us simulated.  Bump watchdog
    -- generously.
    test_runner_watchdog(runner, 100 ms);

end architecture;
