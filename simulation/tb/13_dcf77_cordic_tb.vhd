-- =========================================================================
-- 13_dcf77_cordic_tb.vhd
--
-- VUnit testbench for dcf77_cordic_atan2.  Drives a handful of known
-- complex inputs and verifies that:
--   1. out_mag is within tolerance of K·√(I²+Q²) (K ≈ 1.6467).
--   2. out_phase is within ±2 LSB (≈ ±0.18° at PHASE_WIDTH=12) of the
--      true atan2(Q, I).
--   3. all four quadrants plus the principal axes resolve correctly.
-- =========================================================================

library vunit_lib;
context vunit_lib.vunit_context;

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use ieee.math_real.all;

entity dcf77_cordic_tb is
    generic (runner_cfg : string);
end entity;

architecture tb of dcf77_cordic_tb is

    constant CLK_PERIOD  : time     := 8 ns;
    constant IQ_WIDTH    : positive := 18;
    constant PHASE_WIDTH : positive := 12;
    constant ITERATIONS  : positive := 10;

    -- CORDIC gain K = prod(sqrt(1 + 2^-2i)) for i = 0..ITERATIONS-1
    constant CORDIC_K : real := 1.6467602;

    -- LSB-per-radian for the phase output (2π = 2^PHASE_WIDTH).
    constant LSB_PER_RAD : real := real(2 ** PHASE_WIDTH) / (2.0 * MATH_PI);

    signal clk : std_logic := '0';
    signal rst : std_logic := '1';

    signal in_i      : signed(IQ_WIDTH - 1 downto 0) := (others => '0');
    signal in_q      : signed(IQ_WIDTH - 1 downto 0) := (others => '0');
    signal in_valid  : std_logic := '0';

    signal out_mag   : unsigned(IQ_WIDTH downto 0);
    signal out_phase : signed(PHASE_WIDTH - 1 downto 0);
    signal out_valid : std_logic;

begin

    clk <= not clk after CLK_PERIOD / 2;

    dut : entity work.dcf77_cordic_atan2
        generic map (
            IQ_WIDTH    => IQ_WIDTH,
            PHASE_WIDTH => PHASE_WIDTH,
            ITERATIONS  => ITERATIONS
        )
        port map (
            clk       => clk,
            rst       => rst,
            in_i      => in_i,
            in_q      => in_q,
            in_valid  => in_valid,
            out_mag   => out_mag,
            out_phase => out_phase,
            out_valid => out_valid
        );

    test_runner : process

        -- Drive (i, q), wait for out_valid, return (mag, phase) as integers.
        procedure run_one(i_val   : integer;
                          q_val   : integer;
                          mag_out : out integer;
                          phs_out : out integer) is
        begin
            in_i     <= to_signed(i_val, IQ_WIDTH);
            in_q     <= to_signed(q_val, IQ_WIDTH);
            in_valid <= '1';
            wait until rising_edge(clk);
            in_valid <= '0';
            wait until rising_edge(clk) and out_valid = '1';
            mag_out := to_integer(out_mag);
            phs_out := to_integer(out_phase);
        end procedure;

        -- Tolerances:
        --   * Phase: caller-specified LSB.  ±2 LSB (≈ ±0.18°) for normal
        --     signal levels; larger for very-small inputs where the
        --     CORDIC shifts run out of bits below the magnitude.
        --   * Magnitude: ±3 % relative (CORDIC gain rounding + integer ops)
        procedure check_one(i_val, q_val : integer;
                            tag : string;
                            phs_tol_lsb : integer := 2) is
            variable m, p             : integer;
            variable expected_phs_lsb : real;
            variable expected_mag     : real;
            variable phs_err          : integer;
            variable mag_err          : real;
            variable tolerated_mag_err: real;
        begin
            run_one(i_val, q_val, m, p);

            expected_mag := CORDIC_K * sqrt(real(i_val * i_val + q_val * q_val));
            expected_phs_lsb := arctan(real(q_val), real(i_val)) * LSB_PER_RAD;
            -- Phase wraps at ±2^(PHASE_WIDTH-1).  Normalise the comparison.
            phs_err := p - integer(expected_phs_lsb);
            if phs_err > 2 ** (PHASE_WIDTH - 1) then
                phs_err := phs_err - 2 ** PHASE_WIDTH;
            elsif phs_err < -(2 ** (PHASE_WIDTH - 1)) then
                phs_err := phs_err + 2 ** PHASE_WIDTH;
            end if;
            check(abs(phs_err) <= phs_tol_lsb,
                  tag & ": phase err " & integer'image(phs_err) &
                  " LSB (got " & integer'image(p) & ", expected " &
                  integer'image(integer(expected_phs_lsb)) & ")");

            mag_err := real(m) - expected_mag;
            tolerated_mag_err := expected_mag * 0.03 + 5.0;
            check(abs(mag_err) <= tolerated_mag_err,
                  tag & ": mag err " & integer'image(integer(mag_err)) &
                  " LSB (got " & integer'image(m) & ", expected " &
                  integer'image(integer(expected_mag)) & ")");
        end procedure;

    begin
        test_runner_setup(runner, runner_cfg);

        rst <= '1';
        wait for 4 * CLK_PERIOD;
        wait until rising_edge(clk);
        rst <= '0';
        wait until rising_edge(clk);

        while test_suite loop

            -----------------------------------------------------------------
            if run("axes") then
                check_one(  10000,      0, "positive x");
                check_one(      0,  10000, "positive y (+90°)");
                check_one( -10000,      0, "negative x (±180°)");
                check_one(      0, -10000, "negative y (-90°)");

            -----------------------------------------------------------------
            elsif run("quadrant_1_45deg") then
                check_one(10000, 10000, "Q1 45°");
                check_one(20000, 10000, "Q1 ~26.57°");

            -----------------------------------------------------------------
            elsif run("quadrant_2") then
                check_one(-10000, 10000, "Q2 135°");
                check_one(-20000, 10000, "Q2 ~153.43°");

            -----------------------------------------------------------------
            elsif run("quadrant_3") then
                check_one(-10000, -10000, "Q3 -135°");
                check_one(-10000,  -5000, "Q3 ~-153.43°");

            -----------------------------------------------------------------
            elsif run("quadrant_4") then
                check_one(10000, -10000, "Q4 -45°");
                check_one(20000, -10000, "Q4 ~-26.57°");

            -----------------------------------------------------------------
            elsif run("small_psk_deviation") then
                -- A signal at carrier amplitude ~10000 with PM deviation
                -- ±15.6° corresponds to Q = ±10000 * sin(15.6°) ≈ ±2691.
                check_one(10000,  2691, "PSK +15.6° on |I|=10000");
                check_one(10000, -2691, "PSK -15.6° on |I|=10000");

            -----------------------------------------------------------------
            elsif run("near_zero") then
                -- Small but non-zero inputs: CORDIC precision degrades as
                -- the shifts run out of bits below the magnitude.  Real
                -- DCF77 reception has post-CIC I/Q in the hundreds-of-LSB
                -- range or above; this test documents behaviour at the
                -- edge rather than expecting strict precision.
                check_one(100,  50, "small input, ~26.57°", phs_tol_lsb => 4);
                check_one( 30,  30, "very small input, 45°", phs_tol_lsb => 20);

            end if;
        end loop;

        test_runner_cleanup(runner);
        wait;
    end process;

    test_runner_watchdog(runner, 2 ms);

end architecture;
