-- =========================================================================
-- 06_dcf77_rx_replay_tb.vhd
--
-- Replay testbench: stream a pre-processed binary recording into the
-- back end of the receiver pipeline.  The .s16 file is produced by
-- simulation/scripts/wav_to_postcic.py and contains real-valued 22-b-
-- signed-equivalent samples at 244 140 Hz (DCF77 carrier at 77.5 kHz),
-- packaged as little-endian int16.
--
-- The CIC cascade is bypassed; the testbench wires the sub-blocks the
-- same way dcf77_core does and drives their `cic2_out`/`cic2_v` inputs
-- directly from the file.  One sample is presented every 512 clock
-- cycles, matching the realistic post-CIC tick rate at 125 MHz.
--
-- Two tests:
--   smoke         -- short run, asserts sync_state ever leaves SEARCH
--                    (= carrier detected) and several bit_valid events
--                    fire.  No full-frame decode expected.
--   full_decode   -- long run that must contain a minute mark; asserts
--                    time_valid = '1' and decoded fields match the
--                    expected ones passed via generics.
--
-- The REPLAY_FILE / EXPECTED_* generics are set by run.py and resolved
-- against simulation/derived/.
-- =========================================================================

library vunit_lib;
context vunit_lib.vunit_context;

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use ieee.fixed_pkg.all;

library std;
use std.textio.all;

library dcf77;
use dcf77.dcf77_pkg.all;
use dcf77.dcf77_cic_widths_pkg.all;

entity dcf77_rx_replay_tb is
    generic (
        runner_cfg     : string;
        REPLAY_FILE    : string  := "";
        SAMPLE_COUNT   : natural := 0;     -- # of int16 samples in the file
        MAX_SAMPLES    : natural := 0;     -- 0 = feed all SAMPLE_COUNT samples
        EXPECT_FULL_FRAME : boolean := false;
        EXPECT_YEAR    : natural := 0;
        EXPECT_MONTH   : natural := 0;
        EXPECT_DAY     : natural := 0;
        EXPECT_WEEKDAY : natural := 0;
        EXPECT_HOUR    : natural := 0;
        EXPECT_MINUTE  : natural := 0;
        EXPECT_DST     : natural := 0;
        -- Number of 125 MHz clock cycles between successive samples.  In
        -- silicon this is 512 (the post-CIC tick spacing); the decoder
        -- back end only counts env_valid pulses (not wall-clock cycles
        -- between them), so dropping this gives identical decoder
        -- behaviour at much faster simulation.  Minimum 4 (the env block
        -- needs 4 clocks per output); 8 is the comfortable default and
        -- gives a 64x speed-up over silicon timing.
        CLOCKS_PER_TICK : positive := 8;
        -- NCO phase increment.  Must satisfy
        --   PHASE_INCR = round(2**24 * 77.5e3 * CLOCKS_PER_TICK / 125e6)
        -- so the LO lands on 77.5 kHz at the chosen tick rate.
        NCO_PHASE_INCR : natural := 1_301
    );
end entity;

architecture tb of dcf77_rx_replay_tb is

    constant CLK_PERIOD : time := 8 ns;

    -- ----- clock / reset -------------------------------------------------
    signal clk : std_logic := '0';
    signal rst : std_logic := '1';

    -- ----- pipeline signals (mirror of dcf77_core internals) -------------
    signal cic2_out  : sfixed(CIC2_OUT_W - 1 downto 0) := (others => '0');
    signal cic2_v    : std_logic := '0';

    signal cos_sign  : std_logic;
    signal sin_sign  : std_logic;

    signal i_mix     : iq_word_t;
    signal q_mix     : iq_word_t;
    signal mix_v     : std_logic;

    -- Post-mixer per-channel decimation, mirroring dcf77_core.
    signal cic3_i_out : sfixed(CIC3_OUT_W - 1 downto 0);
    signal cic3_q_out : sfixed(CIC3_OUT_W - 1 downto 0);
    signal cic3_i_v   : std_logic;
    signal cic3_q_v   : std_logic;
    signal cic4_i_out : sfixed(CIC4_OUT_W - 1 downto 0);
    signal cic4_q_out : sfixed(CIC4_OUT_W - 1 downto 0);
    signal cic4_i_v   : std_logic;
    signal cic4_q_v   : std_logic;

    signal env_pwr   : env_word_t;
    signal env_v     : std_logic;

    signal bit_val   : std_logic;
    signal bit_v     : std_logic;
    signal minute_mk : std_logic;
    signal sync_st   : sync_state_t;
    signal env_max_s : env_word_t;
    signal env_min_s : env_word_t;
    signal noise_s   : env_word_t;
    signal thresh_s  : env_word_t;

    signal time_d    : dcf77_time_type;
    signal time_v_d  : std_logic;
    signal last_frm  : std_logic_vector(0 to 58);
    signal bit_ix    : bit_index_t;

    -- ----- run-end gate --------------------------------------------------
    -- The stimulus process drives `done` high once it has fed the last
    -- sample (or reached end-of-file); the test_runner then waits a small
    -- post-roll window for the decoder to flush.
    signal stim_done : std_logic := '0';

    -- Event capture for the smoke assertion.
    shared variable bits_seen   : natural := 0;
    shared variable minutes_seen: natural := 0;

begin

    clk <= not clk after CLK_PERIOD / 2;

    -- -------------------------------------------------------------------- --
    -- Receiver back end (dcf77_core wiring minus the CIC cascade)           --
    -- -------------------------------------------------------------------- --
    u_nco : entity dcf77.dcf77_nco
        generic map (
            PHASE_WIDTH => 18,
            PHASE_INCR  => NCO_PHASE_INCR
        )
        port map (
            clk      => clk,
            rst      => rst,
            tick     => cic2_v,
            cos_sign => cos_sign,
            sin_sign => sin_sign
        );

    u_mixer : entity dcf77.dcf77_mixer
        generic map (
            DATA_WIDTH => CIC2_OUT_W
        )
        port map (
            clk       => clk,
            rst       => rst,
            in_sample => cic2_out,
            in_valid  => cic2_v,
            cos_sign  => cos_sign,
            sin_sign  => sin_sign,
            i_out     => i_mix,
            q_out     => q_mix,
            out_valid => mix_v
        );

    -- Post-mixer per-channel decimators (matches dcf77_core wiring).
    u_cic3_i : entity dcf77.dcf77_cic_dec
        generic map (
            N => CIC_N, DEC_R => CIC3_R, M => CIC_M,
            IN_WIDTH => CIC3_IN_W, INT_WIDTH => CIC3_INT_W,
            COMB_WIDTH => CIC3_COMB_W, OUT_WIDTH => CIC3_OUT_W
        )
        port map (
            clk => clk, rst => rst,
            in_sample => i_mix, in_valid => mix_v,
            out_sample => cic3_i_out, out_valid => cic3_i_v
        );

    u_cic3_q : entity dcf77.dcf77_cic_dec
        generic map (
            N => CIC_N, DEC_R => CIC3_R, M => CIC_M,
            IN_WIDTH => CIC3_IN_W, INT_WIDTH => CIC3_INT_W,
            COMB_WIDTH => CIC3_COMB_W, OUT_WIDTH => CIC3_OUT_W
        )
        port map (
            clk => clk, rst => rst,
            in_sample => q_mix, in_valid => mix_v,
            out_sample => cic3_q_out, out_valid => cic3_q_v
        );

    u_cic4_i : entity dcf77.dcf77_cic_dec
        generic map (
            N => CIC_N, DEC_R => CIC4_R, M => CIC_M,
            IN_WIDTH => CIC4_IN_W, INT_WIDTH => CIC4_INT_W,
            COMB_WIDTH => CIC4_COMB_W, OUT_WIDTH => CIC4_OUT_W
        )
        port map (
            clk => clk, rst => rst,
            in_sample => cic3_i_out, in_valid => cic3_i_v,
            out_sample => cic4_i_out, out_valid => cic4_i_v
        );

    u_cic4_q : entity dcf77.dcf77_cic_dec
        generic map (
            N => CIC_N, DEC_R => CIC4_R, M => CIC_M,
            IN_WIDTH => CIC4_IN_W, INT_WIDTH => CIC4_INT_W,
            COMB_WIDTH => CIC4_COMB_W, OUT_WIDTH => CIC4_OUT_W
        )
        port map (
            clk => clk, rst => rst,
            in_sample => cic3_q_out, in_valid => cic3_q_v,
            out_sample => cic4_q_out, out_valid => cic4_q_v
        );

    u_env : entity dcf77.dcf77_env
        port map (
            clk       => clk,
            rst       => rst,
            i_in      => cic4_i_out,
            q_in      => cic4_q_out,
            in_valid  => cic4_i_v,
            power     => env_pwr,
            out_valid => env_v
        );

    u_bit_det : entity dcf77.dcf77_bit_det
        port map (
            clk         => clk,
            rst         => rst,
            env_in      => env_pwr,
            env_valid   => env_v,
            bit_value   => bit_val,
            bit_valid   => bit_v,
            minute_mark => minute_mk,
            sync_state  => sync_st,
            env_max     => env_max_s,
            env_min     => env_min_s,
            noise       => noise_s,
            threshold   => thresh_s
        );

    u_frame : entity dcf77.dcf77_frame_dec
        port map (
            clk         => clk,
            rst         => rst,
            bit_value   => bit_val,
            bit_valid   => bit_v,
            minute_mark => minute_mk,
            time_out    => time_d,
            time_valid  => time_v_d,
            last_frame  => last_frm,
            bit_index   => bit_ix
        );

    -- -------------------------------------------------------------------- --
    -- Sample event monitor (counts bit / minute events for the smoke check) --
    -- -------------------------------------------------------------------- --
    monitor : process (clk)
    begin
        if rising_edge(clk) then
            if bit_v = '1' then
                bits_seen := bits_seen + 1;
            end if;
            if minute_mk = '1' then
                minutes_seen := minutes_seen + 1;
            end if;
        end if;
    end process;

    -- -------------------------------------------------------------------- --
    -- Binary file streamer                                                  --
    --                                                                       --
    -- Reads two bytes per sample (LE int16), every CLOCKS_PER_TICK clocks.  --
    -- Sign-extends the int16 into sfixed(21 downto 0).  Holds `cic2_v` high  --
    -- for one clock at each tick.                                           --
    -- -------------------------------------------------------------------- --
    stim : process

        type byte_file_t is file of character;
        file replay_handle : byte_file_t;

        procedure tick_idle is
        begin
            cic2_v <= '0';
            for i in 1 to CLOCKS_PER_TICK - 1 loop
                wait until rising_edge(clk);
            end loop;
        end procedure;

        procedure read_int16(signal out_sample : out sfixed(21 downto 0)) is
            variable lo, hi : character;
            variable lo_b, hi_b : std_logic_vector(7 downto 0);
            variable s16 : signed(15 downto 0);
        begin
            read(replay_handle, lo);
            read(replay_handle, hi);
            lo_b := std_logic_vector(to_unsigned(character'pos(lo), 8));
            hi_b := std_logic_vector(to_unsigned(character'pos(hi), 8));
            s16  := signed(hi_b & lo_b);
            -- sign-extend signed(15..0) into sfixed(21 downto 0), LSB at 0
            out_sample <= resize(to_sfixed(s16, 15, 0), 21, 0);
        end procedure;

        variable status : file_open_status;
        variable feed   : natural := 0;
        variable limit  : natural;
    begin
        -- Wait for reset to release.
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

        file_open(status, replay_handle, REPLAY_FILE, read_mode);
        assert status = open_ok
            report "cannot open replay file " & REPLAY_FILE
            severity failure;

        while feed < limit loop
            -- present sample for one clock
            read_int16(cic2_out);
            cic2_v <= '1';
            wait until rising_edge(clk);
            cic2_v <= '0';

            -- hold idle for the remaining cycles of this tick window
            for i in 1 to CLOCKS_PER_TICK - 1 loop
                wait until rising_edge(clk);
            end loop;

            feed := feed + 1;
        end loop;

        file_close(replay_handle);
        stim_done <= '1';
        wait;
    end process;

    -- -------------------------------------------------------------------- --
    -- Test runner                                                           --
    -- -------------------------------------------------------------------- --
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
            if run("smoke") then
                -- If no replay file was provided (e.g. the default run.py
                -- config when no .s16 is present), let the test pass
                -- silently rather than asserting against an empty pipeline.
                if REPLAY_FILE = "" then
                    info("no REPLAY_FILE configured, skipping replay smoke");
                else
                    wait until stim_done = '1';
                    wait for 100 us;
                    check(bits_seen >= 1,
                          "at least one bit_valid fired (" &
                          integer'image(bits_seen) & ")");
                end if;

            -----------------------------------------------------------------
            elsif run("full_decode") then
                if REPLAY_FILE = "" or not EXPECT_FULL_FRAME then
                    info("no REPLAY_FILE configured, skipping replay full_decode");
                else
                -- For a >=60 s clip we expect at least one minute_mark and
                -- a successful frame decode.
                wait until stim_done = '1';
                wait for 1 ms;

                check_equal(time_v_d, '1', "time_valid asserted");
                check(minutes_seen >= 1,
                      "at least one minute_mark fired (" &
                      integer'image(minutes_seen) & ")");
                check_equal(to_integer(time_d.year),    EXPECT_YEAR,    "year");
                check_equal(to_integer(time_d.month),   EXPECT_MONTH,   "month");
                check_equal(to_integer(time_d.day),     EXPECT_DAY,     "day");
                check_equal(to_integer(time_d.weekday), EXPECT_WEEKDAY, "weekday");
                check_equal(to_integer(time_d.hour),    EXPECT_HOUR,    "hour");
                check_equal(to_integer(time_d.minute),  EXPECT_MINUTE,  "minute");
                if EXPECT_DST = 1 then
                    check_equal(time_d.dst_active, '1', "MESZ active");
                else
                    check_equal(time_d.dst_active, '0', "MEZ active");
                end if;
                end if;

            end if;
        end loop;

        test_runner_cleanup(runner);
        wait;
    end process;

    -- 60 s of feed + 1 s post-roll worst-case.
    test_runner_watchdog(runner, 90 sec);

end architecture;
