-- =========================================================================
-- 00_dcf77_cic_dec_tb.vhd
--
-- VUnit testbench for the generic CIC decimator dcf77_cic_dec.  Two configs
-- are registered in run.py: R=2 (matches CIC0 in the cascade) and R=16
-- (matches CIC1 / CIC2 widths).  Each config runs the same battery of
-- tests:
--   dc_gain    - constant input settles to V * R^N after the transient
--   zero       - zero input produces zero output
--   impulse    - sum of decimated outputs equals R^N
-- =========================================================================

library vunit_lib;
context vunit_lib.vunit_context;

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use ieee.fixed_pkg.all;

library dcf77;

entity dcf77_cic_dec_tb is
    generic (
        runner_cfg : string;
        DEC_R      : positive := 16;
        N_STAGES   : positive := 3;
        IN_WIDTH   : positive := 17;
        INT_WIDTH  : positive := 29;
        COMB_WIDTH : positive := 28;
        OUT_WIDTH  : positive := 24
    );
end entity;

architecture tb of dcf77_cic_dec_tb is

    constant CLK_PERIOD : time := 8 ns;

    signal clk : std_logic := '0';
    signal rst : std_logic := '1';

    signal in_sample  : sfixed(IN_WIDTH - 1 downto 0) := (others => '0');
    signal in_valid   : std_logic := '0';
    signal out_sample : sfixed(OUT_WIDTH - 1 downto 0);
    signal out_valid  : std_logic;

    function ipow(base : integer; exp : integer) return integer is
        variable acc : integer := 1;
    begin
        for i in 1 to exp loop
            acc := acc * base;
        end loop;
        return acc;
    end function;

    constant DC_GAIN : integer := ipow(DEC_R, N_STAGES);
    -- The decimated impulse response of an N-stage CIC with decimation R sums
    -- to R^(N-1), not R^N.  (The full input-rate impulse response sums to R^N;
    -- decimation by R picks every R'th tap, so the cumulative area scales
    -- by 1/R, leaving R^(N-1).)
    constant IMPULSE_AREA : integer := ipow(DEC_R, N_STAGES - 1);

begin

    clk <= not clk after CLK_PERIOD / 2;

    dut : entity dcf77.dcf77_cic_dec
        generic map (
            N          => N_STAGES,
            DEC_R      => DEC_R,
            M          => 1,
            IN_WIDTH   => IN_WIDTH,
            INT_WIDTH  => INT_WIDTH,
            COMB_WIDTH => COMB_WIDTH,
            OUT_WIDTH  => OUT_WIDTH
        )
        port map (
            clk        => clk,
            rst        => rst,
            in_sample  => in_sample,
            in_valid   => in_valid,
            out_sample => out_sample,
            out_valid  => out_valid
        );

    -------------------------------------------------------------------------
    test_runner : process

        -- Apply one input sample (synchronous to clk) with in_valid='1'.
        procedure drive_sample(value : integer) is
        begin
            in_sample <= to_sfixed(value, IN_WIDTH - 1, 0);
            in_valid  <= '1';
            wait until rising_edge(clk);
            in_valid  <= '0';
            in_sample <= (others => '0');
        end procedure;

        -- Drive N_drive samples of value V and capture how many output samples
        -- arrived and the LAST out_sample value.
        procedure drive_and_capture(value     : in  integer;
                                    n_drive   : in  natural;
                                    last_out  : out integer;
                                    sum_out   : out integer) is
            variable cnt : natural := 0;
            variable s   : integer := 0;
            variable la  : integer := 0;
        begin
            for i in 1 to n_drive loop
                in_sample <= to_sfixed(value, IN_WIDTH - 1, 0);
                in_valid  <= '1';
                wait until rising_edge(clk);
                if out_valid = '1' then
                    la := to_integer(signed(to_slv(out_sample)));
                    s  := s + la;
                    cnt := cnt + 1;
                end if;
            end loop;
            in_valid  <= '0';
            in_sample <= (others => '0');
            last_out := la;
            sum_out  := s;
        end procedure;

        variable last_v : integer;
        variable sum_v  : integer;

    begin
        test_runner_setup(runner, runner_cfg);

        -- Reset for a few cycles.
        rst <= '1';
        wait for 4 * CLK_PERIOD;
        wait until rising_edge(clk);
        rst <= '0';
        wait until rising_edge(clk);

        while test_suite loop

            -----------------------------------------------------------------
            if run("zero") then
                -- 4*N*R samples of zero input -> zero output throughout.
                drive_and_capture(0, 4 * N_STAGES * DEC_R, last_v, sum_v);
                check_equal(last_v, 0, "last output sample with zero input");
                check_equal(sum_v,  0, "sum of decimated outputs with zero input");

            -----------------------------------------------------------------
            elsif run("dc_gain") then
                -- Apply constant 1 for enough samples that the integrator
                -- ramps up and the comb chain has produced at least 4 fully-
                -- settled output samples.  After settling, each decimated
                -- output should equal R^N exactly.
                drive_and_capture(1, 8 * N_STAGES * DEC_R, last_v, sum_v);
                check_equal(last_v, DC_GAIN,
                            "DC step settles to R^N");

            -----------------------------------------------------------------
            elsif run("impulse") then
                -- Single impulse at t=0.  Sum of all decimated outputs over
                -- the (N*R+1) impulse response support equals R^N.
                drive_sample(1);
                drive_and_capture(0, N_STAGES * DEC_R * 2, last_v, sum_v);
                check_equal(sum_v, IMPULSE_AREA,
                            "decimated impulse area equals R^(N-1)");

            end if;
        end loop;

        test_runner_cleanup(runner);
        wait;
    end process;

    test_runner_watchdog(runner, 1 ms);

end architecture;
