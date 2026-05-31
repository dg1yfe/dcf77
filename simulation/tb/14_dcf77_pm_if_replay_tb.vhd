-- =========================================================================
-- 14_dcf77_pm_if_replay_tb.vhd
--
-- PM (PSK / SIK) IF replay testbench.  Streams a real-valued int16 sample
-- file through the new CORDIC-based receiver and asserts that the PM
-- correlator's sync state machine progresses past FILL and that pm_valid
-- pulses fire while the broadcast is being processed.
--
-- This TB doesn't try to validate the decoded PM time (no cached PM
-- truth yet) — it's a smoke test that demonstrates the full PM path
-- runs on real signal.  Cold lock of the PM correlator on a clean
-- broadcast is expected within a few broadcast-seconds (= ~1-2 ms of
-- wall-sim time per second of broadcast at CLOCKS_PER_TICK=8).
--
-- Adapted from 07_dcf77_if_replay_tb; uses the same NCO/mixer/boxcar
-- input pipeline.  Post-boxcar: CORDIC → magnitude IIR (unused here)
-- + phase × 2^11 adapter → carrier_pll q_sample; phase direct → PM
-- correlator → sign(pm_soft) → PM-mode frame_dec.
-- =========================================================================

library vunit_lib;
context vunit_lib.vunit_context;

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use ieee.fixed_float_types.all;
use ieee.fixed_pkg.all;

library std;
use std.textio.all;

library dcf77;
use dcf77.dcf77_pkg.all;

entity dcf77_pm_if_replay_tb is
    generic (
        runner_cfg     : string;
        REPLAY_FILE    : string  := "";
        SAMPLE_COUNT   : natural := 0;     -- # of real samples in the file
        MAX_SAMPLES    : natural := 0;     -- 0 = feed every sample
        IF_HZ          : natural := 5_000;
        SAMPLE_RATE_HZ : natural := 12_000;
        CLOCKS_PER_TICK : positive := 8;
        -- Minimum pm_valid pulses we expect by end of recording.
        EXPECT_PM_VALID_MIN : natural := 30;

        -- Optional CSV telemetry dump (same idea as 07_if_replay).
        -- Columns: t_ms, nco_offset, pll_qfilt, pll_qlock, pll_lock,
        -- cordic_phase, pm_soft, pm_valid, pm_lock_qual, pm_sync_state.
        -- One row per TELEM_PERIOD_MS of broadcast time.
        TELEM_CSV       : string  := "";
        TELEM_PERIOD_MS : positive := 10
    );
end entity;

architecture tb of dcf77_pm_if_replay_tb is

    constant CLK_PERIOD : time := 8 ns;

    constant NCO_PHASE_W  : positive := 25;
    constant NCO_OFFSET_W : positive := 24;
    constant NCO_PHASE_INCR_VAL : natural :=
        natural(real(2 ** NCO_PHASE_W) * real(IF_HZ) / real(SAMPLE_RATE_HZ));

    constant POST_BOX_RATE  : positive := SAMPLE_RATE_HZ / 2000;
    constant POST_W         : positive := iq_word_t'length + 3;
    constant LOOP_IIR_SHIFT : positive := 4;

    constant CORDIC_IQ_W    : positive := 18;
    constant CORDIC_PHASE_W : positive := 12;

    constant PLL_PHASE_ADAPTER_SHIFT : natural := 11;

    constant SYNC_FILL    : std_logic_vector(1 downto 0) := "00";
    constant SYNC_ACQUIRE : std_logic_vector(1 downto 0) := "01";
    constant SYNC_LOCK    : std_logic_vector(1 downto 0) := "10";

    signal clk : std_logic := '0';
    signal rst : std_logic := '1';

    signal sample_in : iq_word_t := (others => '0');
    signal sample_v  : std_logic := '0';

    -- NCO + mixer
    signal cos_sign, sin_sign : std_logic;
    signal nco_phase_acc      : unsigned(NCO_PHASE_W - 1 downto 0);
    signal nco_offset         : signed(NCO_OFFSET_W - 1 downto 0);
    signal pll_offset         : signed(NCO_OFFSET_W - 1 downto 0);

    signal i_mix : iq_word_t;
    signal q_mix : iq_word_t;
    signal mix_v : std_logic;

    -- Post-mixer boxcar
    signal i_box, q_box     : signed(POST_W - 1 downto 0);
    signal i_box_v, q_box_v : std_logic;

    -- CORDIC + IIR
    signal cordic_mag   : unsigned(CORDIC_IQ_W downto 0);
    signal cordic_phase : signed(CORDIC_PHASE_W - 1 downto 0);
    signal cordic_v     : std_logic;

    signal mag_signed : signed(CORDIC_IQ_W + 1 downto 0);
    signal mag_filt   : signed(CORDIC_IQ_W + 1 downto 0);
    signal mag_filt_v : std_logic;

    -- PLL inputs
    signal phase_for_pll   : signed(POST_W - 1 downto 0);
    signal phase_for_pll_v : std_logic;

    signal pll_lock        : std_logic;
    signal pll_qfilt       : signed(POST_W - 1 downto 0);
    signal pll_qlock       : signed(POST_W - 1 downto 0);

    -- PM correlator outputs
    signal pm_soft_s          : signed(11 downto 0);
    signal pm_valid_s         : std_logic;
    signal pm_corr_mm_s       : std_logic;
    signal pm_sync_state_s    : std_logic_vector(1 downto 0);
    signal pm_locked_phase_s  : unsigned(11 downto 0);
    signal pm_lock_qual_s     : unsigned(15 downto 0);

    -- PM frame_dec
    signal pm_time_d   : dcf77_time_type;
    signal pm_time_v_d : std_logic;
    signal pm_last_frm : std_logic_vector(0 to 58);
    signal pm_bit_ix   : bit_index_t;
    signal pm_frame_mm : std_logic;

    signal stim_done : std_logic := '0';

    shared variable pm_valid_count    : natural := 0;
    shared variable pm_mm_count       : natural := 0;
    shared variable pm_state_changes  : natural := 0;
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

    -- NCO + sign-LO mixer.
    u_nco : entity dcf77.dcf77_nco
        generic map (
            PHASE_WIDTH  => NCO_PHASE_W,
            PHASE_INCR   => NCO_PHASE_INCR_VAL,
            OFFSET_WIDTH => NCO_OFFSET_W
        )
        port map (
            clk              => clk,
            rst              => rst,
            tick             => sample_v,
            phase_inc_offset => nco_offset,
            cos_sign         => cos_sign,
            sin_sign         => sin_sign,
            phase_acc_out    => nco_phase_acc
        );

    u_mixer : entity dcf77.dcf77_mixer
        generic map (
            DATA_WIDTH => iq_word_t'length
        )
        port map (
            clk       => clk,
            rst       => rst,
            in_sample => sample_in,
            in_valid  => sample_v,
            cos_sign  => cos_sign,
            sin_sign  => sin_sign,
            i_out     => i_mix,
            q_out     => q_mix,
            out_valid => mix_v
        );

    -- Post-mixer boxcar to 2 kS/s.
    u_i_boxcar : entity dcf77.dcf77_boxcar
        generic map (
            IN_WIDTH  => iq_word_t'length,
            RATE      => POST_BOX_RATE,
            OUT_WIDTH => POST_W
        )
        port map (
            clk        => clk,
            rst        => rst,
            in_sample  => i_mix,
            in_valid   => mix_v,
            out_sample => i_box,
            out_valid  => i_box_v
        );

    u_q_boxcar : entity dcf77.dcf77_boxcar
        generic map (
            IN_WIDTH  => iq_word_t'length,
            RATE      => POST_BOX_RATE,
            OUT_WIDTH => POST_W
        )
        port map (
            clk        => clk,
            rst        => rst,
            in_sample  => q_mix,
            in_valid   => mix_v,
            out_sample => q_box,
            out_valid  => q_box_v
        );

    -- CORDIC: post-boxcar I/Q → magnitude + phase.
    u_cordic : entity dcf77.dcf77_cordic_atan2
        generic map (
            IQ_WIDTH    => CORDIC_IQ_W,
            PHASE_WIDTH => CORDIC_PHASE_W,
            ITERATIONS  => 10
        )
        port map (
            clk       => clk,
            rst       => rst,
            in_i      => i_box(POST_W - 1 downto POST_W - CORDIC_IQ_W),
            in_q      => q_box(POST_W - 1 downto POST_W - CORDIC_IQ_W),
            in_valid  => i_box_v,
            out_mag   => cordic_mag,
            out_phase => cordic_phase,
            out_valid => cordic_v
        );

    -- Magnitude IIR (unused in this TB, but kept for symmetry with core).
    mag_signed <= signed('0' & std_logic_vector(cordic_mag));

    u_mag_iir : entity dcf77.dcf77_iir_lpf
        generic map (
            WIDTH => CORDIC_IQ_W + 2,
            SHIFT => LOOP_IIR_SHIFT
        )
        port map (
            clk        => clk,
            rst        => rst,
            in_sample  => mag_signed,
            in_valid   => cordic_v,
            out_sample => mag_filt,
            out_valid  => mag_filt_v
        );

    -- Carrier-PLL adapter: left-shift CORDIC phase by 11 to match the
    -- PI gains calibrated for the old amplitude-weighted detector.
    pll_adapter_proc : process(clk)
    begin
        if rising_edge(clk) then
            if rst = '1' then
                phase_for_pll   <= (others => '0');
                phase_for_pll_v <= '0';
            else
                phase_for_pll   <= shift_left(resize(cordic_phase, POST_W),
                                              PLL_PHASE_ADAPTER_SHIFT);
                phase_for_pll_v <= cordic_v;
            end if;
        end if;
    end process;

    u_carrier_pll : entity dcf77.dcf77_carrier_pll
        generic map (
            Q_WIDTH         => POST_W,
            OFFSET_WIDTH    => NCO_OFFSET_W,
            KP_SHIFT        => 7,
            KI_SHIFT        => 20,
            LOOP_IIR_SHIFT  => LOOP_IIR_SHIFT,
            LOCK_IIR_SHIFT  => 14,
            LOCK_IN_THR     => 2**18,
            LOCK_OUT_THR    => 2**21,
            LOCK_HOLD_TICKS => 256
        )
        port map (
            clk              => clk,
            rst              => rst,
            q_sample         => phase_for_pll,
            q_valid          => phase_for_pll_v,
            phase_inc_offset => pll_offset,
            lock_state       => pll_lock,
            q_filt_out       => pll_qfilt,
            q_lock_out       => pll_qlock
        );

    nco_offset <= pll_offset;

    -- ------------------------------------------------------------------ --
    -- PM correlator + PM frame_dec.                                       --
    -- ------------------------------------------------------------------ --
    u_pm_corr : entity dcf77.dcf77_pm_correlator
        port map (
            clk              => clk,
            rst              => rst,
            phase_err        => cordic_phase(CORDIC_PHASE_W - 1
                                              downto CORDIC_PHASE_W - 9),
            phase_err_v      => cordic_v,
            pll_locked       => pll_lock,
            pm_soft          => pm_soft_s,
            pm_valid         => pm_valid_s,
            pm_minute_mark_o => pm_corr_mm_s,
            pm_sync_state    => pm_sync_state_s,
            pm_locked_phase  => pm_locked_phase_s,
            pm_lock_qual     => pm_lock_qual_s
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
    -- PM monitor: count pm_valid pulses, PM minute_marks, and state      --
    -- transitions.                                                        --
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
                    -- Log per-bit pm_soft to diagnose SIK signature detection.
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
    -- CSV telemetry: dense dump of PLL + PM correlator state.  Enabled
    -- via the TELEM_CSV generic; render with
    -- simulation/scripts/plot_pll_telem.py.
    -- ------------------------------------------------------------------ --
    csv_telemetry : process
        constant TICK_INTERVAL : time :=
            TELEM_PERIOD_MS * SAMPLE_RATE_HZ * CLOCKS_PER_TICK * CLK_PERIOD
            / 1_000;
        file     fh        : text;
        variable lbuf      : line;
        variable t_ms      : natural := 0;
        variable status    : file_open_status;
        variable lock_int  : integer;
        variable pmv_int   : integer;
    begin
        if TELEM_CSV = "" then
            wait;
        end if;
        wait until rst = '0';
        file_open(status, fh, TELEM_CSV, write_mode);
        assert status = open_ok
            report "csv_telemetry: cannot open " & TELEM_CSV
            severity failure;
        write(lbuf, string'("t_ms,nco_offset,pll_qfilt,pll_qlock,"
                            & "pll_lock,cordic_phase,pm_soft,pm_valid,"
                            & "pm_lock_qual,pm_sync_state"));
        writeline(fh, lbuf);
        loop
            wait for TICK_INTERVAL;
            t_ms := t_ms + TELEM_PERIOD_MS;
            if pll_lock = '1'  then lock_int := 1; else lock_int := 0; end if;
            if pm_valid_s = '1' then pmv_int  := 1; else pmv_int  := 0; end if;
            write(lbuf, integer'image(t_ms));
            write(lbuf, ',');
            write(lbuf, integer'image(to_integer(pll_offset)));
            write(lbuf, ',');
            write(lbuf, integer'image(to_integer(pll_qfilt)));
            write(lbuf, ',');
            write(lbuf, integer'image(to_integer(pll_qlock)));
            write(lbuf, ',');
            write(lbuf, integer'image(lock_int));
            write(lbuf, ',');
            write(lbuf, integer'image(to_integer(cordic_phase)));
            write(lbuf, ',');
            write(lbuf, integer'image(to_integer(pm_soft_s)));
            write(lbuf, ',');
            write(lbuf, integer'image(pmv_int));
            write(lbuf, ',');
            write(lbuf, integer'image(to_integer(pm_lock_qual_s)));
            write(lbuf, ',');
            write(lbuf, integer'image(to_integer(unsigned(pm_sync_state_s))));
            writeline(fh, lbuf);
            exit when stim_done = '1';
        end loop;
        file_close(fh);
        wait;
    end process;

    -- ------------------------------------------------------------------ --
    -- IF stream driver (mirrors 07_dcf77_if_replay_tb).                   --
    -- ------------------------------------------------------------------ --
    stim : process

        type byte_file_t is file of character;
        file replay_handle : byte_file_t;

        procedure read_int16(signal out_sample : out iq_word_t) is
            variable lo, hi   : character;
            variable lo_b, hi_b : std_logic_vector(7 downto 0);
            variable s16      : signed(15 downto 0);
        begin
            read(replay_handle, lo);
            read(replay_handle, hi);
            lo_b := std_logic_vector(to_unsigned(character'pos(lo), 8));
            hi_b := std_logic_vector(to_unsigned(character'pos(hi), 8));
            s16  := signed(hi_b & lo_b);
            out_sample <= resize(to_sfixed(s16, 15, 0),
                                 iq_word_t'high, iq_word_t'low);
        end procedure;

        variable status : file_open_status;
        variable feed   : natural := 0;
        variable limit  : natural;
    begin
        wait until rst = '0';
        wait until rising_edge(clk);

        if REPLAY_FILE = "" then
            stim_done <= '1';
            wait;
        end if;

        if MAX_SAMPLES = 0 or MAX_SAMPLES > SAMPLE_COUNT then
            limit := SAMPLE_COUNT;
        else
            limit := MAX_SAMPLES;
        end if;

        info("pm-if-replay start: file=" & REPLAY_FILE
             & " samples=" & integer'image(limit));

        file_open(status, replay_handle, REPLAY_FILE, read_mode);
        assert status = open_ok
            report "cannot open replay file " & REPLAY_FILE
            severity failure;

        while feed < limit loop
            read_int16(sample_in);
            sample_v <= '1';
            wait until rising_edge(clk);
            sample_v <= '0';

            for i in 1 to CLOCKS_PER_TICK - 1 loop
                wait until rising_edge(clk);
            end loop;

            feed := feed + 1;
        end loop;

        file_close(replay_handle);
        info("pm-if-replay end: pm_valid=" & integer'image(pm_valid_count)
             & " pm_mm=" & integer'image(pm_mm_count)
             & " state_changes=" & integer'image(pm_state_changes));
        stim_done <= '1';
        wait;
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

            -----------------------------------------------------------------
            if run("pm_smoke") then
                -- Run the full recording through the PM chain.  Verify:
                --   1. pm_sync_state advances past FILL (= buffer filled
                --      and ACQUIRE bursts started)
                --   2. at least EXPECT_PM_VALID_MIN pm_valid pulses fired
                --      somewhere during the run (proves PM correlator
                --      eventually finds something to integrate against)
                if REPLAY_FILE = "" then
                    info("no REPLAY_FILE configured, skipping pm_smoke");
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

    test_runner_watchdog(runner, 2 sec);

end architecture;
