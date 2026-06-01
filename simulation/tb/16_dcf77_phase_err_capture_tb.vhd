-- =========================================================================
-- 16_dcf77_phase_err_capture_tb.vhd
--
-- Capture testbench for the DCF77 PM upstream chain.  Reads an .if.s16
-- recording, runs it through the same NCO + mixer + I/Q boxcars +
-- CORDIC + carrier-PLL pipeline as 14_pm_if_replay_tb, and dumps the
-- resulting `cordic_phase + pll_lock` stream to a binary file.  Strictly
-- a producer for 15_pm_correlator_replay_tb -- no PM correlator, no
-- frame_dec, no smoke assertions are instantiated, so the wall-time
-- cost of the capture is exactly that of the IF chain itself (no
-- 40-buffer-rewrite-per-burst BRAM activity).
--
-- File format (2 bytes per baseband sample, int16 little-endian):
--   bits 11:0  : signed cordic_phase (12 b)
--   bits 14:12 : reserved (zero)
--   bit  15    : pll_lock
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

entity dcf77_phase_err_capture_tb is
    generic (
        runner_cfg      : string;
        REPLAY_FILE     : string  := "";
        SAMPLE_COUNT    : natural := 0;
        MAX_SAMPLES     : natural := 0;     -- 0 = feed every sample
        IF_HZ           : natural := 5_000;
        SAMPLE_RATE_HZ  : natural := 12_000;
        CLOCKS_PER_TICK : positive := 8;
        PHASE_ERR_DUMP  : string  := "";

        -- Carrier-PLL PI gain right-shifts.  Same defaults as 14_pm.
        PLL_KP_SHIFT    : positive := 10;
        PLL_KI_SHIFT    : positive := 23
    );
end entity;

architecture tb of dcf77_phase_err_capture_tb is

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

    signal clk : std_logic := '0';
    signal rst : std_logic := '1';

    signal sample_in : iq_word_t := (others => '0');
    signal sample_v  : std_logic := '0';

    signal cos_sign, sin_sign : std_logic;
    signal nco_phase_acc      : unsigned(NCO_PHASE_W - 1 downto 0);
    signal nco_offset         : signed(NCO_OFFSET_W - 1 downto 0);
    signal pll_offset         : signed(NCO_OFFSET_W - 1 downto 0);

    signal i_mix : iq_word_t;
    signal q_mix : iq_word_t;
    signal mix_v : std_logic;

    signal i_box, q_box     : signed(POST_W - 1 downto 0);
    signal i_box_v, q_box_v : std_logic;

    signal cordic_mag   : unsigned(CORDIC_IQ_W downto 0);
    signal cordic_phase : signed(CORDIC_PHASE_W - 1 downto 0);
    signal cordic_v     : std_logic;

    signal mag_signed : signed(CORDIC_IQ_W + 1 downto 0);
    signal mag_filt   : signed(CORDIC_IQ_W + 1 downto 0);
    signal mag_filt_v : std_logic;

    signal phase_for_pll   : signed(POST_W - 1 downto 0);
    signal phase_for_pll_v : std_logic;

    signal pll_lock    : std_logic;
    signal pll_qfilt   : signed(POST_W - 1 downto 0);
    signal pll_qlock   : signed(POST_W - 1 downto 0);

    signal stim_done : std_logic := '0';

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
        generic map (DATA_WIDTH => iq_word_t'length)
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

    -- Magnitude IIR kept for symmetry with the production core (PLL
    -- timing depends on the same set of IIR-driven nets toggling).
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
            KP_SHIFT        => PLL_KP_SHIFT,
            KI_SHIFT        => PLL_KI_SHIFT,
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
    -- Phase-error dump: 2 bytes int16-LE per cordic_v baseband sample.   --
    -- ------------------------------------------------------------------ --
    phase_err_dump_proc : process(clk)
        type byte_file_t is file of character;
        file     fh        : byte_file_t;
        variable status    : file_open_status;
        variable opened    : boolean := false;
        variable val_slv   : std_logic_vector(15 downto 0);
    begin
        if rising_edge(clk) then
            if PHASE_ERR_DUMP /= "" then
                if not opened and rst = '0' then
                    file_open(status, fh, PHASE_ERR_DUMP, write_mode);
                    if status = open_ok then
                        opened := true;
                    end if;
                end if;
                if opened and cordic_v = '1' then
                    val_slv := (others => '0');
                    val_slv(11 downto 0) := std_logic_vector(cordic_phase);
                    val_slv(15)          := pll_lock;
                    write(fh, character'val(
                        to_integer(unsigned(val_slv(7 downto 0)))));
                    write(fh, character'val(
                        to_integer(unsigned(val_slv(15 downto 8)))));
                end if;
            end if;
        end if;
    end process;

    -- ------------------------------------------------------------------ --
    -- IF stream driver (same pattern as 07/14_if_replay).                 --
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

        info("phase-err capture start: file=" & REPLAY_FILE
             & " samples=" & integer'image(limit)
             & " dump=" & PHASE_ERR_DUMP);

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
        info("phase-err capture end");
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
            if run("capture") then
                if REPLAY_FILE = "" then
                    info("no REPLAY_FILE configured, skipping");
                else
                    wait until stim_done = '1';
                    wait for 100 us;
                    info("capture complete");
                end if;
            end if;
        end loop;

        test_runner_cleanup(runner);
        wait;
    end process;

    test_runner_watchdog(runner, 30 min);

end architecture;
