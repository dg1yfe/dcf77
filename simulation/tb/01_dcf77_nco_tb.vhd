-- =========================================================================
-- 01_dcf77_nco_tb.vhd
--
-- VUnit testbench for dcf77_nco.  The NCO must produce sign-only cos/sin
-- waveforms whose frequency is 77.5 kHz at the 244.140 kS/s tick rate, and
-- whose phase relationship is quadrature (sin lags cos by pi/2 -- so over a
-- long capture the XOR of the two sign bits has 50% duty).
-- =========================================================================

library vunit_lib;
context vunit_lib.vunit_context;

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library dcf77;

entity dcf77_nco_tb is
    generic (runner_cfg : string);
end entity;

architecture tb of dcf77_nco_tb is

    constant CLK_PERIOD       : time     := 8 ns;       -- 125 MHz
    constant TICK_PERIOD_CLKS : positive := 500;        -- 250 kS/s
    constant PHASE_WIDTH      : positive := 18;
    constant PHASE_INCR       : natural  := 81_245;     -- ~77.5 kHz @ 250 kS/s

    -- Reference cycle count for the cos waveform over the capture window:
    -- 77.5 kHz at the 250 kS/s tick rate -> 0.31 cycles per tick.
    -- 4096 ticks = ~16.4 ms -> ~1270 cycles -> 2540 edges (cos has two
    -- sign flips per cycle).  Tolerate +/- 4 edges for the small phase
    -- error from rounding K to the nearest 18-b value.
    constant CAPTURE_TICKS    : positive := 4096;

    signal clk      : std_logic := '0';
    signal rst      : std_logic := '1';
    signal tick     : std_logic := '0';
    signal cos_sign : std_logic;
    signal sin_sign : std_logic;

begin

    clk <= not clk after CLK_PERIOD / 2;

    dut : entity dcf77.dcf77_nco
        generic map (
            PHASE_WIDTH => PHASE_WIDTH,
            PHASE_INCR  => PHASE_INCR
        )
        port map (
            clk      => clk,
            rst      => rst,
            tick     => tick,
            cos_sign => cos_sign,
            sin_sign => sin_sign
        );

    test_runner : process

        procedure gen_tick(n_ticks : natural;
                           cos_edges : out natural;
                           sin_edges : out natural;
                           xor_high  : out natural) is
            variable cos_prev : std_logic := cos_sign;
            variable sin_prev : std_logic := sin_sign;
            variable c        : natural := 0;
            variable s        : natural := 0;
            variable xh       : natural := 0;
        begin
            for i in 1 to n_ticks loop
                -- Hold tick='0' for TICK_PERIOD_CLKS - 1 cycles, then pulse high.
                for j in 1 to TICK_PERIOD_CLKS - 1 loop
                    wait until rising_edge(clk);
                end loop;
                tick <= '1';
                wait until rising_edge(clk);
                tick <= '0';

                -- Sample one cycle after the pulse so the new phase is visible.
                wait until rising_edge(clk);
                if cos_sign /= cos_prev then
                    c := c + 1;
                end if;
                if sin_sign /= sin_prev then
                    s := s + 1;
                end if;
                if (cos_sign xor sin_sign) = '1' then
                    xh := xh + 1;
                end if;
                cos_prev := cos_sign;
                sin_prev := sin_sign;
            end loop;
            cos_edges := c;
            sin_edges := s;
            xor_high  := xh;
        end procedure;

        variable cos_edges, sin_edges, xor_high : natural;

    begin
        test_runner_setup(runner, runner_cfg);

        -- Reset.
        rst <= '1';
        wait for 4 * CLK_PERIOD;
        wait until rising_edge(clk);
        rst <= '0';
        wait until rising_edge(clk);

        while test_suite loop

            -----------------------------------------------------------------
            if run("frequency") then
                gen_tick(CAPTURE_TICKS, cos_edges, sin_edges, xor_high);
                -- Each LO cycle has two sign-flips, so cos_edges should be
                -- 2 * (77500 / 250000) * CAPTURE_TICKS ~= 2539 with our
                -- 4096-tick window.  Tolerate +/- 6 edges for initial
                -- transient and 18-b NCO rounding error.
                check(cos_edges >= 2533 and cos_edges <= 2545,
                      "cos edge count: " & integer'image(cos_edges) &
                      " (expected ~2539)");
                check(sin_edges >= 2533 and sin_edges <= 2545,
                      "sin edge count: " & integer'image(sin_edges) &
                      " (expected ~2539)");

            -----------------------------------------------------------------
            elsif run("quadrature") then
                gen_tick(CAPTURE_TICKS, cos_edges, sin_edges, xor_high);
                -- For perfect quadrature, cos and sin sign-bits agree for 1/2
                -- of the cycle and disagree for the other 1/2.  Expect XOR-
                -- high count near CAPTURE_TICKS/2, tolerating a few percent.
                check(xor_high >= (CAPTURE_TICKS / 2) - 80
                      and xor_high <= (CAPTURE_TICKS / 2) + 80,
                      "XOR high count: " & integer'image(xor_high) &
                      " (expected ~" & integer'image(CAPTURE_TICKS / 2) & ")");

            end if;
        end loop;

        test_runner_cleanup(runner);
        wait;
    end process;

    test_runner_watchdog(runner, 100 ms);

end architecture;
