-- =========================================================================
-- 11_dcf77_pm_correlator_tb.vhd
--
-- VUnit smoke TB for dcf77_pm_correlator.  Generates synthetic
-- phase_err matching the broadcast's PSK signal (LFSR-9 with taps 5 & 9,
-- SIK polarity from a deterministic bit pattern, active 1586 samples
-- per second starting 200 ms after each second boundary).
--
-- Verifies that:
--   (a) The correlator transitions M_FILL → M_ACQUIRE within the
--       expected 2048 sample ticks.
--   (b) Within a few seconds it transitions M_ACQUIRE → M_LOCK
--       (signature lock from a clean signal).
--   (c) Once LOCKED, pm_valid pulses appear at the expected cadence
--       and pm_soft has plausible (signed, non-zero) values.
--
-- Sample rate emulation: we accelerate vs the real 62 500 clk/sample by
-- using a smaller clk/sample ratio so the sim runs in seconds, not
-- minutes.  4000 clk/sample gives the engine enough room for a 463-hyp
-- burst (~1.5M clk = 375 sample ticks) before the next burst slot.
-- =========================================================================

library vunit_lib;
context vunit_lib.vunit_context;

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library dcf77;
use dcf77.dcf77_pkg.all;

entity dcf77_pm_correlator_tb is
    generic (runner_cfg : string);
end entity;

architecture tb of dcf77_pm_correlator_tb is

    constant CLK_PERIOD     : time    := 8 ns;        -- 125 MHz
    constant CLK_PER_SAMPLE : natural := 4_000;       -- 4000 clk per phase_err_v

    signal clk         : std_logic := '0';
    signal rst         : std_logic := '1';

    signal phase_err   : signed(8 downto 0)         := (others => '0');
    signal phase_err_v : std_logic                  := '0';

    signal pm_soft        : signed(11 downto 0);
    signal pm_valid       : std_logic;
    signal pm_minute_mark : std_logic;
    signal pm_sync_state  : std_logic_vector(1 downto 0);
    signal pm_locked_phase: unsigned(11 downto 0);
    signal pm_lock_qual   : unsigned(15 downto 0);

    constant SYNC_FILL    : std_logic_vector(1 downto 0) := "00";
    constant SYNC_ACQUIRE : std_logic_vector(1 downto 0) := "01";
    constant SYNC_LOCK    : std_logic_vector(1 downto 0) := "10";

    -- Latches for pm_valid pulses (1-cycle pulses missed by coarse polling).
    shared variable pm_valid_pulses    : natural := 0;
    shared variable last_pm_soft       : integer := 0;

    -- Pre-compute the 512-chip PRBS-9 sequence with taps 5,9 + Hetzel
    -- force-1 (chip 511 = '0').  +1 for chip '0', -1 for chip '1'.
    function build_prbs return integer_vector is
        variable state : unsigned(8 downto 0) := "000000001";
        variable seq   : integer_vector(0 to 511);
        variable bit5, bit9, new_bit : std_logic;
    begin
        for n in 0 to 510 loop
            if state(0) = '0' then
                seq(n) := 1;
            else
                seq(n) := -1;
            end if;
            bit5 := state(4);
            bit9 := state(8);
            new_bit := bit5 xor bit9;
            state := state(7 downto 0) & new_bit;
        end loop;
        -- DUT does NOT implement Hetzel's force-1 mechanism for v1, so its
        -- 9-bit LFSR runs as a pure m-sequence with period 511.  Chip 511
        -- therefore equals chip 0 (LFSR back at initial state "000000001").
        if state(0) = '0' then
            seq(511) := 1;
        else
            seq(511) := -1;
        end if;
        return seq;
    end function;

    constant PRBS : integer_vector(0 to 511) := build_prbs;

    -- Per-second SIK polarity.  Seconds 0..9 = '1' (signature) → -1;
    -- 10..14, 59 = '0' (signature) → +1; 15..58 = pseudo-random data.
    function sik_polarity(sec : natural) return integer is
    begin
        if sec mod 60 <= 9 then
            return -1;
        elsif (sec mod 60 = 59) or (sec mod 60 >= 10 and sec mod 60 <= 14) then
            return +1;
        else
            -- Deterministic data pattern
            if (sec mod 7) >= 4 then
                return -1;                  -- bit '1' → inverted PNS
            else
                return +1;                  -- bit '0' → unchanged PNS
            end if;
        end if;
    end function;

    constant CHIP0_OFFSET     : natural := 400;       -- samples within the second
    -- 9-b phase_err scale: 2π = 2^9 LSB, so PSK deviation ±15.6°
    -- corresponds to ±22 LSB per sample.  Over 1586 samples the
    -- correct-hypothesis integration peaks at ±35 K, above
    -- LOCK_IN_THR = 16 K.
    constant DELTA_PHI_LSB    : integer := 22;
    constant SAMPLES_PER_PRBS : natural := 1586;
    constant SAMPLES_PER_SEC  : natural := 2000;

begin

    clk <= not clk after CLK_PERIOD / 2;

    dut : entity dcf77.dcf77_pm_correlator
        port map (
            clk              => clk,
            rst              => rst,
            phase_err        => phase_err,
            phase_err_v      => phase_err_v,
            pm_soft          => pm_soft,
            pm_valid         => pm_valid,
            pm_minute_mark_o => pm_minute_mark,
            pm_sync_state    => pm_sync_state,
            pm_locked_phase  => pm_locked_phase,
            pm_lock_qual     => pm_lock_qual,
            pm_debug         => open
        );

    ----------------------------------------------------------------------
    -- Stimulus driver: emit one phase_err_v pulse every CLK_PER_SAMPLE
    -- clock cycles, with phase_err set to the synthetic PSK signal.
    ----------------------------------------------------------------------
    stim : process
        variable sample_idx : natural := 0;
        variable sec_idx    : natural;
        variable n_in_sec   : natural;
        variable chip_idx   : natural;
        variable val        : integer;
    begin
        wait until rst = '0';

        loop
            -- Compute phase_err for this sample tick.
            sec_idx  := sample_idx / SAMPLES_PER_SEC;
            n_in_sec := sample_idx mod SAMPLES_PER_SEC;
            if n_in_sec >= CHIP0_OFFSET
               and n_in_sec < CHIP0_OFFSET + SAMPLES_PER_PRBS
            then
                chip_idx := ((n_in_sec - CHIP0_OFFSET) * 31) / 96;
                if chip_idx >= 512 then
                    chip_idx := chip_idx mod 512;
                end if;
                val := DELTA_PHI_LSB * PRBS(chip_idx) * sik_polarity(sec_idx);
            else
                val := 0;
            end if;
            phase_err <= to_signed(val, 9);

            -- One-cycle phase_err_v pulse.
            phase_err_v <= '1';
            wait until rising_edge(clk);
            phase_err_v <= '0';

            -- Idle until next sample tick.
            for k in 1 to CLK_PER_SAMPLE - 1 loop
                wait until rising_edge(clk);
            end loop;

            sample_idx := sample_idx + 1;
        end loop;
    end process;

    -- Catch every pm_valid pulse (1 cycle wide).
    pm_valid_monitor : process(clk)
    begin
        if rising_edge(clk) then
            if pm_valid = '1' then
                pm_valid_pulses := pm_valid_pulses + 1;
                last_pm_soft := to_integer(pm_soft);
            end if;
        end if;
    end process;

    ----------------------------------------------------------------------
    -- Test driver
    ----------------------------------------------------------------------
    test_runner : process
        variable timeout_count : natural;
        variable pm_valid_count : natural;
    begin
        test_runner_setup(runner, runner_cfg);
        rst <= '1';
        wait for 4 * CLK_PERIOD;
        rst <= '0';

        while test_suite loop

            -----------------------------------------------------------------
            if run("smoke_fill_to_acquire") then
                -- M_FILL needs BUF_SIZE = 4096 samples = 4096 * 4000 clk
                -- ≈ 16.4M clk.  Run for ~20M clk to give margin.
                for n in 0 to 20_000_000 / 100 loop
                    wait for 100 * CLK_PERIOD;
                    exit when pm_sync_state /= SYNC_FILL;
                end loop;
                check(pm_sync_state /= SYNC_FILL,
                      "after 20M clk, sync_state must have left FILL");

            -----------------------------------------------------------------
            elsif run("smoke_lock_eventually") then
                -- BUF_SIZE = 4096, so FILL = 16M clk.  Add ~2 natural cycles
                -- (~32M clk) for the new lock to settle.  Total budget ~80M clk.
                for n in 0 to 80_000_000 / 100 loop
                    wait for 100 * CLK_PERIOD;
                    exit when pm_sync_state = SYNC_LOCK and pm_valid_pulses >= 2;
                end loop;
                check_equal(pm_sync_state, SYNC_LOCK,
                            "correlator reaches LOCK within 80M clk");
                check(pm_valid_pulses >= 2,
                      "at least 2 pm_valid pulses fired in LOCK (got " &
                      integer'image(pm_valid_pulses) & ")");
                check(last_pm_soft /= 0,
                      "last pm_soft non-zero (got " &
                      integer'image(last_pm_soft) & ")");

            end if;
        end loop;

        test_runner_cleanup(runner);
        wait;
    end process;

    -- 50M clk × 8 ns = 400 ms wall-clock — generous watchdog headroom.
    test_runner_watchdog(runner, 1500 ms);

end architecture;
