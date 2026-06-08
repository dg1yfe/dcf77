-- =========================================================================
-- 15_dcf77_pm_correlator_replay_tb.vhd
--
-- Standalone replay TB for dcf77_pm_correlator.  Reads a pre-captured
-- phase_err stream from disk (produced by 14_pm_if_replay_tb's
-- PHASE_ERR_DUMP generic) and drives the correlator directly,
-- bypassing the IF chain (mixer, CIC, PLL, CORDIC).
--
-- Decoupling the upstream chain from the correlator lets us
--   (a) skip the upstream sim cost entirely
--   (b) drive phase_err_v at arbitrary periods without rate-coupling
--       the engine to the writer (the buffer race seen in 14_pm at
--       CLOCKS_PER_TICK=8, where the writer ticks 65x per engine
--       hypothesis and the BRAM is rewritten 40x per burst)
--
-- File format (2 bytes per baseband sample, int16 little-endian):
--   bits 11:0  : signed cordic_phase (12 b)
--   bits 14:12 : reserved (zero)
--   bit  15    : pll_lock
--
-- The replay TB extracts phase_err = bits 11:3 (= top 9 of 12), matching
-- the production wiring in dcf77_core / 14_pm_if_replay_tb.
-- =========================================================================

library vunit_lib;
context vunit_lib.vunit_context;

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library std;
use std.textio.all;

library dcf77;
use dcf77.dcf77_pkg.all;

entity dcf77_pm_correlator_replay_tb is
    generic (
        runner_cfg          : string;
        REPLAY_FILE         : string  := "";
        SAMPLE_COUNT        : natural := 0;       -- # of baseband samples in file
        -- Clocks between consecutive phase_err_v='1' pulses.  This
        -- sets the writer-vs-engine race ratio inside the PM
        -- correlator's BRAM ring buffer:
        --
        --   * The engine takes ~3174 clk to evaluate one hypothesis
        --     (E_REQ + E_RECV × 1586 samples + E_DONE).
        --   * The writer fires once every PHASE_ERR_V_CLOCKS clk.
        --   * So writes-per-hyp = 3174 / PHASE_ERR_V_CLOCKS, and
        --     race-impacted samples per integration window = the
        --     same.
        --
        -- **Recommended default: 3174 (= 1 write/hyp).**  At that
        -- value, the writer touches at most ONE of the 1586 read
        -- positions per hypothesis (≈ 0.06 % of the integration),
        -- which is mathematically negligible for PRBS correlation
        -- magnitude.  Lower values introduce race noise that scales
        -- linearly; higher values cost simulation wall time without
        -- any further correlation improvement.
        --
        -- Why not lower for speed?
        --
        --   PHASE_ERR_V_CLOCKS | writes/hyp | race fraction
        --   48                 | 66         | 4.2 %  (matches 14_pm
        --                                              CLOCKS_PER_TICK=8 --
        --                                              breaks ACQUIRE on
        --                                              the corpus)
        --   200                | 16         | 1.0 %  (marginal on noisy
        --                                              recordings)
        --   1000               |  3         | 0.2 %  (still safe; saves
        --                                              ~30 s wall time)
        --   3174               |  1         | 0.06 % (theoretical floor
        --                                              for non-zero race)
        --   31740              |  0.1       | 0       (10× over-engineered;
        --                                              wall time grows 10×)
        --
        -- Why 3174 specifically?  It's the engine's per-hyp clock
        -- cost.  Anything > 3174 means the engine OUTRUNS the writer
        -- (engine pulls ahead, never races); anything < 3174 means
        -- the writer catches up and overwrites samples mid-read.
        -- 3174 is the inflection point where the race noise is at
        -- its natural minimum (= exactly 1 write per hyp, write hits
        -- exactly 1 read position per hyp).  See
        -- feedback_pm_phase_err_v_clocks memory for the full
        -- analysis.
        PHASE_ERR_V_CLOCKS  : positive := 3174;
        EXPECT_PM_VALID_MIN : natural := 30;
        ACQUIRE_LOG_CSV     : string  := "";
        TRACK_LOG_CSV       : string  := ""
    );
end entity;

architecture tb of dcf77_pm_correlator_replay_tb is

    constant CLK_PERIOD : time := 8 ns;

    constant SYNC_FILL    : std_logic_vector(1 downto 0) := "00";
    constant SYNC_ACQUIRE : std_logic_vector(1 downto 0) := "01";
    constant SYNC_LOCK    : std_logic_vector(1 downto 0) := "10";

    signal clk : std_logic := '0';
    signal rst : std_logic := '1';

    signal phase_err   : signed(8 downto 0) := (others => '0');
    signal phase_err_v : std_logic          := '0';
    signal pll_locked  : std_logic          := '0';

    signal pm_soft_s         : signed(11 downto 0);
    signal pm_valid_s        : std_logic;
    signal pm_corr_mm_s      : std_logic;
    signal pm_sync_state_s   : std_logic_vector(1 downto 0);
    signal pm_locked_phase_s : unsigned(11 downto 0);
    signal pm_lock_qual_s    : unsigned(15 downto 0);
    signal pm_debug_s        : dcf77_pm_debug_type;

    signal pm_time_d   : dcf77_time_type;
    signal pm_time_v_d : std_logic;
    signal pm_last_frm : std_logic_vector(0 to 58);
    signal pm_bit_ix   : bit_index_t;
    signal pm_frame_mm : std_logic;

    signal stim_done : std_logic := '0';

    shared variable pm_valid_count     : natural := 0;
    shared variable pm_mm_count        : natural := 0;
    shared variable pm_state_changes   : natural := 0;
    shared variable last_pm_sync_state : std_logic_vector(1 downto 0) := "00";

    function pm_state_str(s : std_logic_vector) return string is
    begin
        if    s = SYNC_FILL    then return "FILL";
        elsif s = SYNC_ACQUIRE then return "ACQUIRE";
        elsif s = SYNC_LOCK    then return "LOCK";
        else                        return "?";
        end if;
    end function;

begin

    clk <= not clk after CLK_PERIOD / 2;

    dut : entity dcf77.dcf77_pm_correlator
        port map (
            clk              => clk,
            rst              => rst,
            phase_err        => phase_err,
            phase_err_v      => phase_err_v,
            pll_locked       => pll_locked,
            pm_soft          => pm_soft_s,
            pm_valid         => pm_valid_s,
            pm_minute_mark_o => pm_corr_mm_s,
            pm_sync_state    => pm_sync_state_s,
            pm_locked_phase  => pm_locked_phase_s,
            pm_lock_qual     => pm_lock_qual_s,
            pm_debug         => pm_debug_s
        );

    u_pm_frame : entity dcf77.dcf77_frame_dec
        generic map (
            PM_MODE => true
        )
        port map (
            clk             => clk,
            rst             => rst,
            bit_value       => pm_soft_s(11),
            bit_valid       => pm_valid_s,
            minute_mark     => '0',
            time_out        => pm_time_d,
            time_valid      => pm_time_v_d,
            last_frame      => pm_last_frm,
            bit_index       => pm_bit_ix,
            minute_mark_out => pm_frame_mm
        );

    -- ------------------------------------------------------------------ --
    -- Stimulus: read int16-LE per baseband sample; drive phase_err +    --
    -- pll_locked from file; pulse phase_err_v for one clock then idle   --
    -- for (PHASE_ERR_V_CLOCKS - 1) clocks before the next sample.        --
    -- ------------------------------------------------------------------ --
    stim : process

        type byte_file_t is file of character;
        file replay_handle : byte_file_t;

        variable status     : file_open_status;
        variable lo, hi     : character;
        variable val_slv    : std_logic_vector(15 downto 0);
        variable feed       : natural := 0;
    begin
        wait until rst = '0';
        wait until rising_edge(clk);

        if REPLAY_FILE = "" then
            stim_done <= '1';
            wait;
        end if;

        info("pm-corr-replay start: file=" & REPLAY_FILE
             & " samples=" & integer'image(SAMPLE_COUNT)
             & " phase_err_v_clocks=" & integer'image(PHASE_ERR_V_CLOCKS));

        file_open(status, replay_handle, REPLAY_FILE, read_mode);
        assert status = open_ok
            report "cannot open replay file " & REPLAY_FILE
            severity failure;

        while feed < SAMPLE_COUNT loop
            read(replay_handle, lo);
            read(replay_handle, hi);
            val_slv := std_logic_vector(to_unsigned(character'pos(hi), 8))
                     & std_logic_vector(to_unsigned(character'pos(lo), 8));

            -- phase_err = top 9 of the 12-b cordic_phase (= bits 11..3)
            phase_err   <= signed(val_slv(11 downto 3));
            pll_locked  <= val_slv(15);
            phase_err_v <= '1';
            wait until rising_edge(clk);
            phase_err_v <= '0';

            for k in 1 to PHASE_ERR_V_CLOCKS - 1 loop
                wait until rising_edge(clk);
            end loop;

            feed := feed + 1;
        end loop;

        file_close(replay_handle);
        info("pm-corr-replay end: pm_valid=" & integer'image(pm_valid_count)
             & " pm_mm=" & integer'image(pm_mm_count)
             & " state_changes=" & integer'image(pm_state_changes));
        stim_done <= '1';
        wait;
    end process;

    -- ------------------------------------------------------------------ --
    -- PM monitor (mirrors 14_pm).                                         --
    -- ------------------------------------------------------------------ --
    pm_monitor : process(clk)
        variable bit_str : string(1 to 1);
    begin
        if rising_edge(clk) then
            if rst = '1' then
                pm_valid_count     := 0;
                pm_mm_count        := 0;
                pm_state_changes   := 0;
                last_pm_sync_state := SYNC_FILL;
            else
                if pm_valid_s = '1' then
                    pm_valid_count := pm_valid_count + 1;
                    if pm_valid_count <= 200 then
                        if pm_soft_s(11) = '1' then
                            bit_str(1) := '1';
                        else
                            bit_str(1) := '0';
                        end if;
                        info("pm_bit #" & integer'image(pm_valid_count)
                             & " sign=" & bit_str
                             & " soft=" & integer'image(to_integer(pm_soft_s))
                             & " bit_ix=" & integer'image(pm_bit_ix));
                    end if;
                end if;
                if pm_frame_mm = '1' then
                    pm_mm_count := pm_mm_count + 1;
                end if;
                if pm_sync_state_s /= last_pm_sync_state then
                    pm_state_changes := pm_state_changes + 1;
                    info("pm_sync_state " & pm_state_str(last_pm_sync_state)
                         & " -> " & pm_state_str(pm_sync_state_s)
                         & " at " & time'image(now));
                    last_pm_sync_state := pm_sync_state_s;
                end if;
            end if;
        end if;
    end process;

    -- ------------------------------------------------------------------ --
    -- ACQUIRE-debug logger (mirrors 14_pm).                               --
    -- ------------------------------------------------------------------ --
    acquire_logger : process(clk)
        file     fh        : text;
        variable lbuf      : line;
        variable status    : file_open_status;
        variable opened    : boolean := false;
    begin
        if rising_edge(clk) then
            if ACQUIRE_LOG_CSV /= "" then
                if not opened and rst = '0' then
                    file_open(status, fh, ACQUIRE_LOG_CSV, write_mode);
                    if status = open_ok then
                        write(lbuf, string'("t_us,hyp_idx,abs_acc,sec_phase"));
                        writeline(fh, lbuf);
                        opened := true;
                    end if;
                end if;
                if opened and pm_debug_s.acquire_v = '1' then
                    write(lbuf, integer'image(now / 1 us));
                    write(lbuf, ',');
                    write(lbuf, integer'image(
                        to_integer(pm_debug_s.acquire_hyp_idx)));
                    write(lbuf, ',');
                    write(lbuf, integer'image(
                        to_integer(pm_debug_s.acquire_abs_acc)));
                    write(lbuf, ',');
                    write(lbuf, integer'image(
                        to_integer(pm_debug_s.acquire_sec_phase)));
                    writeline(fh, lbuf);
                end if;
            end if;
        end if;
    end process;

    -- ------------------------------------------------------------------ --
    -- TRACK-debug logger (mirrors 14_pm).  One line per dbg_track_v       --
    -- pulse with step / |acc| / next_offset / locked_phase.               --
    -- ------------------------------------------------------------------ --
    track_logger : process(clk)
        file     fh        : text;
        variable lbuf      : line;
        variable status    : file_open_status;
        variable opened    : boolean := false;
    begin
        if rising_edge(clk) then
            if TRACK_LOG_CSV /= "" then
                if not opened and rst = '0' then
                    file_open(status, fh, TRACK_LOG_CSV, write_mode);
                    if status = open_ok then
                        write(lbuf, string'(
                            "t_us,step,abs_acc,next_offset,locked_phase"));
                        writeline(fh, lbuf);
                        opened := true;
                    end if;
                end if;
                if opened and pm_debug_s.track_v = '1' then
                    write(lbuf, integer'image(now / 1 us));
                    write(lbuf, ',');
                    write(lbuf, integer'image(
                        to_integer(pm_debug_s.track_step)));
                    write(lbuf, ',');
                    write(lbuf, integer'image(
                        to_integer(pm_debug_s.track_abs_acc)));
                    write(lbuf, ',');
                    write(lbuf, integer'image(
                        to_integer(pm_debug_s.track_next_offset)));
                    write(lbuf, ',');
                    write(lbuf, integer'image(
                        to_integer(pm_debug_s.track_locked_phase)));
                    writeline(fh, lbuf);
                end if;
            end if;
        end if;
    end process;

    -- ------------------------------------------------------------------ --
    -- Test runner                                                          --
    -- ------------------------------------------------------------------ --
    test_runner : process
    begin
        test_runner_setup(runner, runner_cfg);

        rst <= '1';
        wait for 4 * CLK_PERIOD;
        wait until rising_edge(clk);
        rst <= '0';
        wait until rising_edge(clk);

        while test_suite loop

            if run("pm_smoke") then
                if REPLAY_FILE = "" then
                    info("no REPLAY_FILE configured, skipping");
                else
                    wait until stim_done = '1';
                    wait for 1 ms;

                    info("final state: " & pm_state_str(pm_sync_state_s)
                         & "  pm_lock_qual = "
                         & integer'image(to_integer(pm_lock_qual_s))
                         & "  pm_valid pulses = "
                         & integer'image(pm_valid_count)
                         & "  pm_mm = " & integer'image(pm_mm_count));

                    check(pm_state_changes >= 1,
                          "pm_sync_state advanced past FILL (changes = "
                          & integer'image(pm_state_changes) & ")");

                    check(pm_valid_count >= EXPECT_PM_VALID_MIN,
                          "at least " & integer'image(EXPECT_PM_VALID_MIN)
                          & " pm_valid pulses (got "
                          & integer'image(pm_valid_count) & ")");
                end if;
            end if;
        end loop;

        test_runner_cleanup(runner);
        wait;
    end process;

    -- Generous watchdog: a LIVE-rate run of 60 broadcast-sec at
    -- PHASE_ERR_V_CLOCKS=62500 is 60 sec sim; allow margin for slow GHDL.
    test_runner_watchdog(runner, 5 min);

end architecture;
