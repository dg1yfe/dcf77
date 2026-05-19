-- =========================================================================
-- 07_dcf77_if_replay_tb.vhd
--
-- IF replay testbench.  Streams a real-valued int16 sample file (DCF77 at
-- a known IF, default 5 kHz, at 12 kS/s) straight into the receiver's
-- sign-LO mixer with the NCO retuned to that IF/Fs ratio.  This exercises
-- the full carrier-PLL loop closure -- which the baseband replay in
-- simulation/tb/06_dcf77_rx_replay_tb.vhd cannot do because it consumes
-- already-mixed I/Q.
--
-- Pipeline (matches dcf77_core topology with scaled R values):
--
--   .if.s16 stream (real, int16 LE @ 12 kS/s)
--      ▼  sign-extend to iq_word_t (22 b)
--   dcf77_mixer  ◀── dcf77_nco (25 b acc, PHASE_INCR for IF / Fs)
--      ▼  I, Q @ 12 kS/s                              ▲
--      ├─────────────────────┐                        │
--      ▼                     ▼                        │
--   CIC₃_I R=4              CIC₃_Q R=4                │
--      ▼ 3 kS/s                ▼ 3 kS/s               │
--   CIC₄_I R=3              q_boxcar R=3              │
--      ▼ I @ 1 kS/s            ▼ Q @ 1 kS/s, 24 b      │
--   abs(I) -> env_word_t   dcf77_carrier_pll ─────────┘
--      ▼                       phase_inc_offset
--   CIC_ENV R=10
--      ▼ env @ 100 S/s
--   dcf77_bit_det -> dcf77_frame_dec
--
-- Two tests, matching 06_replay:
--   smoke         -- first 130 s; asserts bit / minute_mark counts.
--   full_decode   -- whole recording; validates decoded time fields.
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
use dcf77.dcf77_cic_widths_pkg.all;

entity dcf77_if_replay_tb is
    generic (
        runner_cfg     : string;
        REPLAY_FILE    : string  := "";
        SAMPLE_COUNT   : natural := 0;     -- # of real samples in the file
        MAX_SAMPLES    : natural := 0;     -- 0 = feed every sample
        IF_HZ          : natural := 5_000; -- DCF77 IF in the input stream
        SAMPLE_RATE_HZ : natural := 12_000; -- input stream rate
        EXPECT_FULL_FRAME    : boolean := false;
        EXPECT_VALIDATE_TIME : boolean := false;
        EXPECT_DECODE_PROGRESS : boolean := true;
        EXPECT_YEAR    : natural := 0;
        EXPECT_MONTH   : natural := 0;
        EXPECT_DAY     : natural := 0;
        EXPECT_WEEKDAY : natural := 0;
        EXPECT_HOUR    : natural := 0;
        EXPECT_MINUTE  : natural := 0;
        EXPECT_DST     : natural := 0;
        -- 125 MHz clock cycles between successive samples.  At 12 kS/s with
        -- CLOCKS_PER_TICK = 8 the simulator runs ~125 MHz / 12 kHz / 8 =
        -- ~1300x faster than real time.
        CLOCKS_PER_TICK : positive := 8
    );
end entity;

architecture tb of dcf77_if_replay_tb is

    constant CLK_PERIOD : time := 8 ns;

    -- 25-b NCO accumulator + 24-b signed offset matches dcf77_core.
    constant NCO_PHASE_W  : positive := 25;
    constant NCO_OFFSET_W : positive := 24;
    -- PHASE_INCR = round(2^NCO_PHASE_W * IF_HZ / SAMPLE_RATE_HZ).
    -- For 5 kHz at 12 kS/s: 2^25 * 5 / 12 = 13_981_013.
    constant NCO_PHASE_INCR_VAL : natural :=
        natural(real(2 ** NCO_PHASE_W) * real(IF_HZ) / real(SAMPLE_RATE_HZ));

    constant CIC3_DEC : positive := 4;
    constant CIC4_DEC : positive := 3;

    -- Generous integrator / comb widths; R=4 and R=3 are gentler than the
    -- production R=10/R=25 so the production widths are safe over-provisions.
    constant POST_IN_W   : positive := iq_word_t'length;   -- 22
    constant POST_INT_W  : positive := 33;
    constant POST_COMB_W : positive := 26;
    constant POST_OUT_W  : positive := iq_word_t'length;   -- 22

    -- Q boxcar width: 22 + ceil(log2(3)) = 22 + 2 = 24.
    constant Q_PLL_W : positive := 24;

    signal clk : std_logic := '0';
    signal rst : std_logic := '1';

    signal sample_in : iq_word_t := (others => '0');
    signal sample_v  : std_logic := '0';

    -- NCO + mixer.
    signal cos_sign      : std_logic;
    signal sin_sign      : std_logic;
    signal nco_phase_acc : unsigned(NCO_PHASE_W - 1 downto 0);
    signal nco_offset    : signed(NCO_OFFSET_W - 1 downto 0);

    signal i_mix : iq_word_t;
    signal q_mix : iq_word_t;
    signal mix_v : std_logic;

    -- Post-mixer per-channel decimators.
    signal cic3_i_out : sfixed(POST_OUT_W - 1 downto 0);
    signal cic3_q_out : sfixed(POST_OUT_W - 1 downto 0);
    signal cic3_i_v   : std_logic;
    signal cic3_q_v   : std_logic;
    signal cic4_i_out : sfixed(POST_OUT_W - 1 downto 0);
    signal cic4_i_v   : std_logic;

    -- Q -> PLL.
    signal q_pll   : signed(Q_PLL_W - 1 downto 0);
    signal q_pll_v : std_logic;
    signal pll_lock: std_logic;
    signal pll_qiir: unsigned(Q_PLL_W - 1 downto 0);

    -- Envelope @ 1 kS/s -> CIC_ENV -> 100 S/s.
    signal env_pwr     : env_word_t;
    signal env_v       : std_logic;
    signal env_pwr_cic : sfixed(CIC_ENV_IN_W - 1 downto 0);
    signal env_dec_raw : sfixed(CIC_ENV_OUT_W - 1 downto 0);
    signal env_dec_out : env_word_t;
    signal env_dec_v   : std_logic;

    -- bit_det / frame_dec outputs.
    signal bit_val_raw, mm_raw   : std_logic;
    signal bit_v_raw             : std_logic;
    signal bit_val, bit_v        : std_logic;
    signal minute_mk             : std_logic;
    signal sync_st_raw           : sync_state_t;
    signal sync_st               : sync_state_t;
    signal env_max_s, env_min_s  : env_word_t;
    signal noise_s, thresh_s     : env_word_t;
    signal time_d                : dcf77_time_type;
    signal time_v_d              : std_logic;
    signal last_frm              : std_logic_vector(0 to 58);
    signal bit_ix                : bit_index_t;

    signal stim_done : std_logic := '0';

    shared variable bits_seen    : natural := 0;
    shared variable minutes_seen : natural := 0;
    shared variable time_valids  : natural := 0;

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

    -- ------------------------------------------------------------------ --
    -- NCO + sign-LO mixer.                                                 --
    -- ------------------------------------------------------------------ --
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

    -- ------------------------------------------------------------------ --
    -- Post-mixer: I/Q -> 3 kS/s -> 1 kS/s.                                 --
    -- I lands on the envelope path; Q feeds the carrier PLL via a boxcar.  --
    -- ------------------------------------------------------------------ --
    u_cic3_i : entity dcf77.dcf77_cic_dec
        generic map (
            N          => CIC_N,
            DEC_R      => CIC3_DEC,
            M          => CIC_M,
            IN_WIDTH   => POST_IN_W,
            INT_WIDTH  => POST_INT_W,
            COMB_WIDTH => POST_COMB_W,
            OUT_WIDTH  => POST_OUT_W
        )
        port map (
            clk        => clk,
            rst        => rst,
            in_sample  => i_mix,
            in_valid   => mix_v,
            out_sample => cic3_i_out,
            out_valid  => cic3_i_v
        );

    u_cic3_q : entity dcf77.dcf77_cic_dec
        generic map (
            N          => CIC_N,
            DEC_R      => CIC3_DEC,
            M          => CIC_M,
            IN_WIDTH   => POST_IN_W,
            INT_WIDTH  => POST_INT_W,
            COMB_WIDTH => POST_COMB_W,
            OUT_WIDTH  => POST_OUT_W
        )
        port map (
            clk        => clk,
            rst        => rst,
            in_sample  => q_mix,
            in_valid   => mix_v,
            out_sample => cic3_q_out,
            out_valid  => cic3_q_v
        );

    u_cic4_i : entity dcf77.dcf77_cic_dec
        generic map (
            N          => CIC_N,
            DEC_R      => CIC4_DEC,
            M          => CIC_M,
            IN_WIDTH   => POST_IN_W,
            INT_WIDTH  => POST_INT_W,
            COMB_WIDTH => POST_COMB_W,
            OUT_WIDTH  => POST_OUT_W
        )
        port map (
            clk        => clk,
            rst        => rst,
            in_sample  => cic3_i_out,
            in_valid   => cic3_i_v,
            out_sample => cic4_i_out,
            out_valid  => cic4_i_v
        );

    u_q_boxcar : entity dcf77.dcf77_q_boxcar
        generic map (
            IN_WIDTH  => POST_OUT_W,
            RATE      => CIC4_DEC,
            OUT_WIDTH => Q_PLL_W
        )
        port map (
            clk        => clk,
            rst        => rst,
            in_sample  => cic3_q_out,
            in_valid   => cic3_q_v,
            out_sample => q_pll,
            out_valid  => q_pll_v
        );

    u_carrier_pll : entity dcf77.dcf77_carrier_pll
        generic map (
            Q_WIDTH         => Q_PLL_W,
            OFFSET_WIDTH    => NCO_OFFSET_W,
            K_LOCK_SHIFT    => 6,
            LOCK_IN_THR     => 2**18,
            LOCK_OUT_THR    => 2**21,
            LOCK_HOLD_TICKS => 256
        )
        port map (
            clk              => clk,
            rst              => rst,
            q_sample         => q_pll,
            q_valid          => q_pll_v,
            phase_inc_offset => nco_offset,
            lock_state       => pll_lock,
            q_iir_out        => pll_qiir
        );

    -- ------------------------------------------------------------------ --
    -- Synchronously-detected envelope: abs(I) at 1 kS/s.                   --
    -- After PLL lock, all AM amplitude lands on I and abs(I) is the
    -- envelope.  The 180-deg phase ambiguity is absorbed by the abs.
    -- ------------------------------------------------------------------ --
    env_pwr <= resize(abs(cic4_i_out),
                      env_word_t'high, env_word_t'low,
                      fixed_saturate, fixed_truncate);
    env_v   <= cic4_i_v;

    -- ------------------------------------------------------------------ --
    -- CIC_ENV (R=10) -> 100 S/s.  Same bit-pattern reinterpretation as
    -- dcf77_core.
    -- ------------------------------------------------------------------ --
    env_pwr_cic <= to_sfixed(to_slv(env_pwr), CIC_ENV_IN_W - 1, 0);
    env_dec_out <= to_sfixed(to_slv(env_dec_raw),
                             env_word_t'high, env_word_t'low);

    u_env_dec : entity dcf77.dcf77_cic_dec
        generic map (
            N          => CIC_N,
            DEC_R      => CIC_ENV_R,
            M          => CIC_M,
            IN_WIDTH   => CIC_ENV_IN_W,
            INT_WIDTH  => CIC_ENV_INT_W,
            COMB_WIDTH => CIC_ENV_COMB_W,
            OUT_WIDTH  => CIC_ENV_OUT_W
        )
        port map (
            clk        => clk,
            rst        => rst,
            in_sample  => env_pwr_cic,
            in_valid   => env_v,
            out_sample => env_dec_raw,
            out_valid  => env_dec_v
        );

    -- ------------------------------------------------------------------ --
    -- bit_det + carrier-PLL output gating (same as dcf77_core).            --
    -- ------------------------------------------------------------------ --
    u_bit_det : entity dcf77.dcf77_bit_det
        port map (
            clk           => clk,
            rst           => rst,
            env_in        => env_dec_out,
            env_valid     => env_dec_v,
            bit_value     => bit_val_raw,
            bit_valid     => bit_v_raw,
            minute_mark   => mm_raw,
            sync_state    => sync_st_raw,
            sync_quality  => open,
            mm_quality    => open,
            cycle_in_min  => open,
            env_max       => env_max_s,
            env_min       => env_min_s,
            noise         => noise_s,
            threshold     => thresh_s,
            edge_accepted => open
        );

    bit_v <= bit_v_raw and pll_lock;
    minute_mk <= mm_raw and pll_lock;
    bit_val <= bit_val_raw;
    sync_st <= sync_search when pll_lock = '0' else sync_st_raw;

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

    -- ------------------------------------------------------------------ --
    -- Event monitor.                                                       --
    -- ------------------------------------------------------------------ --
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

    -- ------------------------------------------------------------------ --
    -- Real-int16 streamer @ SAMPLE_RATE_HZ pace.                           --
    -- ------------------------------------------------------------------ --
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

        info("if-replay start: file=" & REPLAY_FILE
             & " samples=" & integer'image(SAMPLE_COUNT)
             & " feed=" & integer'image(limit)
             & " if_hz=" & integer'image(IF_HZ)
             & " fs=" & integer'image(SAMPLE_RATE_HZ)
             & " nco_phase_incr=" & integer'image(NCO_PHASE_INCR_VAL)
             & " validate_time=" & boolean'image(EXPECT_VALIDATE_TIME));

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
        info("if-replay end: fed " & integer'image(feed)
             & " samples, bits_seen=" & integer'image(bits_seen)
             & " minutes_seen=" & integer'image(minutes_seen)
             & " time_valids=" & integer'image(time_valids));
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
            if run("smoke") then
                if REPLAY_FILE = "" then
                    info("no REPLAY_FILE configured, skipping if-replay smoke");
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
                             & "no decode assertion (bits_seen="
                             & integer'image(bits_seen)
                             & " minutes_seen="
                             & integer'image(minutes_seen) & ")");
                    end if;
                end if;

            -----------------------------------------------------------------
            elsif run("full_decode") then
                if REPLAY_FILE = "" or not EXPECT_FULL_FRAME then
                    info("no REPLAY_FILE configured, skipping if-replay full_decode");
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

    -- ~12 kS/s * 8 clocks/sample * 8 ns/clock = ~125 MHz / 12 kHz = ~10x
    -- real-time slower than wall clock, so 500 s recording -> ~5 s ws sim
    -- time -> ~3-5 min wall.  Keep a generous watchdog for full_decode.
    test_runner_watchdog(runner, 30 sec);

end architecture;
