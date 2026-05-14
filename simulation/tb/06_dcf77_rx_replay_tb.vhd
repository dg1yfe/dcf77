-- =========================================================================
-- 06_dcf77_rx_replay_tb.vhd
--
-- Replay testbench: stream pre-decimated complex baseband (I/Q at 1 kS/s)
-- straight into the back end of the receiver pipeline.  The .s16 file is
-- produced by simulation/scripts/wav_to_iq.py and contains interleaved
-- little-endian int16 I,Q pairs.  The TB sign-extends each pair into
-- iq_word_t and drives the dcf77_env input -- bypassing the NCO, mixer,
-- and the post-mixer CIC stages entirely.
--
-- Two tests:
--   smoke         -- run the first ~60 s of the recording, assert that
--                    bit_valid fires at least once (carrier was tracked).
--                    No timestamp validation.
--   full_decode   -- feed the whole recording and check the decoded
--                    fields match the reference time the preprocessor's
--                    Python decoder recovered from the same audio.
--                    Gated on EXPECT_FULL_FRAME + EXPECT_VALIDATE_TIME.
--
-- Verbose logging: the monitor process emits info() lines on each bit,
-- minute mark, sync-state change, and time_valid pulse.  VUnit
-- suppresses test stdout for passing tests; pass -v on the run.py
-- command line to see them.
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

entity dcf77_rx_replay_tb is
    generic (
        runner_cfg     : string;
        REPLAY_FILE    : string  := "";
        SAMPLE_COUNT   : natural := 0;     -- # of (I,Q) pairs in the file
        MAX_SAMPLES    : natural := 0;     -- 0 = feed every pair
        EXPECT_FULL_FRAME    : boolean := false;
        EXPECT_VALIDATE_TIME : boolean := false;
        -- When true, the smoke test asserts the receiver tracked at least
        -- 30 bits and one minute mark.  Set false for weak / interfered
        -- recordings so the WAV still exercises the pipeline without
        -- failing the regression on the receiver's decode limits.
        EXPECT_DECODE_PROGRESS : boolean := true;
        EXPECT_YEAR    : natural := 0;
        EXPECT_MONTH   : natural := 0;
        EXPECT_DAY     : natural := 0;
        EXPECT_WEEKDAY : natural := 0;
        EXPECT_HOUR    : natural := 0;
        EXPECT_MINUTE  : natural := 0;
        EXPECT_DST     : natural := 0;
        -- 125 MHz clock cycles between successive (I,Q) pairs.  The env /
        -- bit_det / frame_dec chain is strobe-driven, so this just sets
        -- how fast simulated time advances; 8 is comfortable.
        CLOCKS_PER_TICK : positive := 8
    );
end entity;

architecture tb of dcf77_rx_replay_tb is

    constant CLK_PERIOD : time := 8 ns;

    signal clk : std_logic := '0';
    signal rst : std_logic := '1';

    signal i_in_s : iq_word_t := (others => '0');
    signal q_in_s : iq_word_t := (others => '0');
    signal iq_v   : std_logic := '0';

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

    signal stim_done : std_logic := '0';

    shared variable bits_seen     : natural := 0;
    shared variable minutes_seen  : natural := 0;
    shared variable time_valids   : natural := 0;

    function sync_state_str(s : sync_state_t) return string is
    begin
        case s is
            when sync_search => return "SEARCH";
            when sync_lock   => return "LOCK";
            when sync_hold   => return "HOLD";
        end case;
    end function;

    function weekday_str(wd : natural) return string is
    begin
        case wd is
            when 1 => return "Mon"; when 2 => return "Tue";
            when 3 => return "Wed"; when 4 => return "Thu";
            when 5 => return "Fri"; when 6 => return "Sat";
            when 7 => return "Sun";
            when others => return "???";
        end case;
    end function;

begin

    clk <= not clk after CLK_PERIOD / 2;

    -- -------------------------------------------------------------------- --
    -- Receiver back end (env -> bit_det -> frame_dec)                       --
    -- -------------------------------------------------------------------- --
    u_env : entity dcf77.dcf77_env
        port map (
            clk       => clk,
            rst       => rst,
            i_in      => i_in_s,
            q_in      => q_in_s,
            in_valid  => iq_v,
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
    -- Event monitor: counts + verbose info() logging (visible with -v)      --
    -- -------------------------------------------------------------------- --
    monitor : process (clk)
        variable prev_sync   : sync_state_t       := sync_search;
        variable prev_time_v : std_logic          := '0';
        variable prev_time   : dcf77_time_type    := DCF77_TIME_RESET;
    begin
        if rising_edge(clk) then
            if rst = '1' then
                prev_sync   := sync_search;
                prev_time_v := '0';
                prev_time   := DCF77_TIME_RESET;
            else
                if sync_st /= prev_sync then
                    info("sync_state " & sync_state_str(prev_sync)
                         & " -> " & sync_state_str(sync_st)
                         & " at " & time'image(now));
                    prev_sync := sync_st;
                end if;

                if bit_v = '1' then
                    bits_seen := bits_seen + 1;
                    info("bit " & integer'image(bit_ix)
                         & " = " & std_logic'image(bit_val)
                         & " sync=" & sync_state_str(sync_st)
                         & " (#" & integer'image(bits_seen) & ")");
                end if;

                if minute_mk = '1' then
                    minutes_seen := minutes_seen + 1;
                    info("minute_mark (#" & integer'image(minutes_seen)
                         & ", bits_seen=" & integer'image(bits_seen) & ")");
                end if;

                -- Log time_valid only on a fresh decode: either the level
                -- just rose, or the decoded time advanced while the level
                -- stayed high.  Otherwise time_v_d=1 every clock would
                -- flood the log.
                if time_v_d = '1'
                    and (prev_time_v = '0' or time_d /= prev_time)
                then
                    time_valids := time_valids + 1;
                    info("time_valid: "
                         & integer'image(to_integer(time_d.year)) & "-"
                         & integer'image(to_integer(time_d.month)) & "-"
                         & integer'image(to_integer(time_d.day)) & " ("
                         & weekday_str(to_integer(time_d.weekday)) & ") "
                         & integer'image(to_integer(time_d.hour)) & ":"
                         & integer'image(to_integer(time_d.minute))
                         & " dst=" & std_logic'image(time_d.dst_active));
                end if;
                prev_time_v := time_v_d;
                prev_time   := time_d;
            end if;
        end if;
    end process;

    -- -------------------------------------------------------------------- --
    -- Binary I/Q streamer                                                   --
    --                                                                       --
    -- Reads four bytes per tick (LE int16 I, then LE int16 Q), every        --
    -- CLOCKS_PER_TICK clocks.  Holds iq_v high for one clock per tick.      --
    -- -------------------------------------------------------------------- --
    stim : process

        type byte_file_t is file of character;
        file replay_handle : byte_file_t;

        procedure read_int16(signal out_sample : out iq_word_t) is
            variable lo, hi : character;
            variable lo_b, hi_b : std_logic_vector(7 downto 0);
            variable s16 : signed(15 downto 0);
        begin
            read(replay_handle, lo);
            read(replay_handle, hi);
            lo_b := std_logic_vector(to_unsigned(character'pos(lo), 8));
            hi_b := std_logic_vector(to_unsigned(character'pos(hi), 8));
            s16  := signed(hi_b & lo_b);
            out_sample <= resize(to_sfixed(s16, 15, 0), 21, 0);
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

        info("replay start: file=" & REPLAY_FILE
             & " pairs=" & integer'image(SAMPLE_COUNT)
             & " feed=" & integer'image(limit)
             & " validate_time=" & boolean'image(EXPECT_VALIDATE_TIME));

        file_open(status, replay_handle, REPLAY_FILE, read_mode);
        assert status = open_ok
            report "cannot open replay file " & REPLAY_FILE
            severity failure;

        while feed < limit loop
            read_int16(i_in_s);
            read_int16(q_in_s);
            iq_v <= '1';
            wait until rising_edge(clk);
            iq_v <= '0';

            for i in 1 to CLOCKS_PER_TICK - 1 loop
                wait until rising_edge(clk);
            end loop;

            feed := feed + 1;
        end loop;

        file_close(replay_handle);
        info("replay end: fed " & integer'image(feed)
             & " I/Q pairs, bits_seen=" & integer'image(bits_seen)
             & " minutes_seen=" & integer'image(minutes_seen)
             & " time_valids=" & integer'image(time_valids));
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
                if REPLAY_FILE = "" then
                    info("no REPLAY_FILE configured, skipping replay smoke");
                else
                    wait until stim_done = '1';
                    wait for 1 ms;
                    if EXPECT_DECODE_PROGRESS then
                        check(bits_seen >= 30,
                              "bit_valid fired enough times (" &
                              integer'image(bits_seen) & " >= 30)");
                        check(minutes_seen >= 1,
                              "at least one minute_mark fired (" &
                              integer'image(minutes_seen) & ")");
                    else
                        info("EXPECT_DECODE_PROGRESS=false; "
                             & "weak/interfered recording, no decode "
                             & "assertion (bits_seen="
                             & integer'image(bits_seen)
                             & " minutes_seen="
                             & integer'image(minutes_seen) & ")");
                    end if;
                end if;

            -----------------------------------------------------------------
            elsif run("full_decode") then
                if REPLAY_FILE = "" or not EXPECT_FULL_FRAME then
                    info("no REPLAY_FILE configured, skipping replay full_decode");
                else
                    wait until stim_done = '1';
                    wait for 1 ms;

                    check_equal(time_v_d, '1', "time_valid asserted");
                    check(minutes_seen >= 1,
                          "at least one minute_mark fired (" &
                          integer'image(minutes_seen) & ")");

                    if EXPECT_VALIDATE_TIME then
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

            end if;
        end loop;

        test_runner_cleanup(runner);
        wait;
    end process;

    test_runner_watchdog(runner, 30 sec);

end architecture;
