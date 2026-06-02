-- =========================================================================
-- dcf77_core.vhd
--
-- Top-level wiring of the DCF77 receiver / decoder:
--
--   adc -> CIC0(R=2) -> CIC1(R=10) -> CIC2(R=25) -+-> mixer @250 kS/s
--                                                  |       |
--                                                  NCO     +-+--> CIC3_I (R=10)
--                                                          |        |    25 kS/s
--                                                          +--> CIC3_Q   ---> iq_tap
--                                                                   |
--                                                                   +--> CIC4_I (R=25)
--                                                                          1 kS/s
--                                                                   +--> CIC4_Q
--                                                                          |
--                                                                env (I²+Q²)
--                                                                          |
--                                                                       bit_det
--                                                                          |
--                                                                       frame_dec --> status_out
--
-- All blocks live in the 125 MHz clock domain; inner CIC stages advance on
-- valid strobes.  XDC multicycle constraints are needed at synthesis:
--   CIC1: factor 2,   CIC2: factor 20,
--   CIC3_*: factor 500, CIC4_*: factor 5000.
-- =========================================================================

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use ieee.fixed_float_types.all;
use ieee.fixed_pkg.all;

use work.dcf77_pkg.all;
use work.dcf77_cic_widths_pkg.all;

entity dcf77_core is
    generic (
        -- AM-chain defaults: 200 S/s tick rate at env_tracker / bit_det
        -- (CORDIC magnitude is decimated /10 from 2 kS/s by an inline
        -- boxcar averager).  IIR time constants scaled vs the legacy
        -- 2 kS/s defaults so continuous-time TCs are unchanged.
        --   K_ATK   = 5  ~ Tau =  32/200 = 160 ms  (was   256/2000 = 128 ms)
        --   K_DEC   = 11 ~ Tau = 2048/200 = 10.2 s (was 16384/2000 =  8.2 s)
        --   K_NOISE = 11 ~ same as K_DEC.
        --   DERIV_DELAY = 2 samples = 10 ms slope-detection window
        --     (was 16 samples = 8 ms at 2 kS/s).  K_SLOPE unchanged
        --     (relative threshold env_max/4).
        K_ATK          : positive := 5;
        K_DEC          : positive := 11;
        K_NOISE        : positive := 11;
        DERIV_DELAY    : positive := 2;
        K_SLOPE        : positive := 2;
        K_REF          : positive := 2;
        SAMPLE_RATE_HZ : positive := 200;
        T_HOLD_SEC     : positive := 120;
        T_SEARCH_SEC   : positive := 300;

        -- NCO frequency increment for 25 b acc @ 250 kS/s mixer tick.
        -- 2^25 * 77500 / 250000 = 10_399_018.6...
        NCO_PHASE_INCR : natural := 10_399_018
    );
    port (
        clk       : in  std_logic;
        rst       : in  std_logic;

        in_sample : in  adc_sample_t;
        in_valid  : in  std_logic;

        -- 0 = abs(I) only (cheaper, ~0.5 dB more sensitive when PLL is
        -- locked on a single carrier),
        -- 1 = |I| + 3/8 * |Q| (magnitude approx; robust against in-band
        -- CW interferers the PLL doesn't track).  Default '1'.
        env_mode  : in  std_logic := '1';

        -- Raw phase-error tap: sign(I_iir) * Q_box at boxcar rate.
        -- This is the input to carrier_pll's internal IIR; it carries
        -- the full post-boxcar bandwidth (≈ 1 kHz), suitable for a
        -- future PM/PSK correlator.  Width matches the post-mixer
        -- boxcar (= iq_word_t'length + 7 = 29 b signed).
        -- Amplitude-independent phase tap (CORDIC atan2 output).
        -- Width is the CORDIC PHASE_WIDTH; 2π = 2^12, so 1 LSB ≈ 0.088°.
        -- Replaces the old 29-b amplitude-weighted sign(I)·q_box tap.
        phase_err_out   : out signed(11 downto 0);
        phase_err_valid : out std_logic;

        -- Comprehensive decoder status: time, validity, last frame, RSSI,
        -- sync state, current bit index.  See dcf77_pkg.dcf77_status_type.
        status_out: out dcf77_status_type
    );
end entity dcf77_core;

architecture rtl of dcf77_core is

    -- Inter-stage signals (pre-mixer).
    signal cic0_out   : sfixed(CIC0_OUT_W - 1 downto 0);
    signal cic0_v     : std_logic;
    signal cic1_out   : sfixed(CIC1_OUT_W - 1 downto 0);
    signal cic1_v     : std_logic;
    signal cic2_out   : sfixed(CIC2_OUT_W - 1 downto 0);  -- 22 b signed @ 250 kS/s
    signal cic2_v     : std_logic;

    signal cos_sign   : std_logic;
    signal sin_sign   : std_logic;

    -- NCO phase accumulator (25 b unsigned) and signed PLL offset (24 b).
    constant NCO_PHASE_W  : positive := 25;
    constant NCO_OFFSET_W : positive := 24;
    signal nco_phase_acc        : unsigned(NCO_PHASE_W  - 1 downto 0);
    signal nco_phase_inc_offset : signed  (NCO_OFFSET_W - 1 downto 0)
                                := (others => '0');

    signal i_mix      : iq_word_t;                        -- 22 b signed @ 250 kS/s
    signal q_mix      : iq_word_t;
    signal mix_v      : std_logic;

    -- Post-mixer flat boxcar (R=125, 250 kS/s -> 2 kS/s).  Width =
    -- iq_word_t'length + ceil(log2(125)) = 22 + 7 = 29 b signed.
    constant POST_BOX_RATE : positive := 125;
    constant POST_W        : positive := iq_word_t'length + 7;  -- 29
    signal i_box      : signed(POST_W - 1 downto 0);
    signal q_box      : signed(POST_W - 1 downto 0);
    signal i_box_v    : std_logic;
    signal q_box_v    : std_logic;

    -- 20 Hz IIR LPF.  lambda = 2^-4 at 2 kS/s.  Applied to the CORDIC
    -- magnitude output to recover the noise-bandwidth limit the old
    -- per-channel i_iir provided before envelope detection.
    constant LOOP_IIR_SHIFT : positive := 4;

    -- ------------------------------------------------------------------ --
    -- CORDIC I/Q-to-polar conversion (Phase 4.5 consolidation).          --
    --                                                                    --
    -- Inputs: top 18 bits of the 29-b post-boxcar I/Q (drops 11 LSBs).   --
    -- Outputs: 19-b unsigned magnitude (= K·√(I²+Q²), K ≈ 1.6467) and    --
    -- 12-b signed phase (atan2(Q, I); 2π = 2^12 so 1 LSB ≈ 0.088°).      --
    --                                                                    --
    -- Replaces three pieces of the old chain:                            --
    --   * the i_iir 20 Hz LPF (now applied to CORDIC magnitude instead) --
    --   * the env_proc α-max-β-min envelope detector                    --
    --   * the phase_err_proc sign(I)·q_box phase detector               --
    -- ------------------------------------------------------------------ --
    constant CORDIC_IQ_W    : positive := 18;
    constant CORDIC_PHASE_W : positive := 12;
    signal cordic_mag   : unsigned(CORDIC_IQ_W downto 0);    -- 19-b
    signal cordic_phase : signed(CORDIC_PHASE_W - 1 downto 0);
    signal cordic_v     : std_logic;

    -- 20-b signed wrapping of CORDIC magnitude (prepend 0 sign bit) for
    -- the magnitude IIR LPF.
    signal mag_signed   : signed(CORDIC_IQ_W + 1 downto 0);  -- 20-b
    signal mag_filt     : signed(CORDIC_IQ_W + 1 downto 0);
    signal mag_filt_v   : std_logic;

    -- /10 decimating boxcar after the magnitude IIR.  Brings the AM
    -- chain (env_tracker + bit_det) down to 200 S/s.  Output is the
    -- sum of 10 consecutive mag_filt samples, naturally a 24-b signed
    -- value (20-b input + 4 b growth).  Sinc nulls at 200/400/... Hz
    -- coincide exactly with the new sample-rate harmonics, so the
    -- worst alias band is notched; first sidelobe ~ -13 dB.
    constant ENV_DEC_RATE : positive := 10;
    signal mag_dec_sum  : signed(CORDIC_IQ_W + 1 + 4 downto 0);  -- 24-b
    signal mag_dec_v    : std_logic;

    -- Carrier-PLL phase input.  The CORDIC phase is amplitude-
    -- independent in 12-b at 2π=2^12 scale; the carrier_pll's PI gains
    -- (KP_SHIFT=7, KI_SHIFT=20) were calibrated for the OLD amplitude-
    -- weighted sign(I)·q_box detector with magnitude A·Δφ.  Left-shift
    -- the CORDIC phase by 11 bits to scale it into the same magnitude
    -- envelope the PI filter was designed for (~1.33 M LSB per radian
    -- = midway between old A_medium and A_strong cases).  Net result:
    -- amplitude-independent phase detector + unchanged PLL loop bandwidth.
    constant PLL_PHASE_ADAPTER_SHIFT : natural := 11;
    signal phase_for_pll   : signed(POST_W - 1 downto 0);
    signal phase_for_pll_v : std_logic;

    -- Synchronously-detected envelope at 2 kS/s.  abs(i_iir) post-
    -- boxcar can reach ~ 2^28; we right-shift by 4 before resizing into
    -- env_word_t so the IIR's slow trackers stay inside the 26-b range
    -- without saturation.
    constant ENV_RSHIFT : natural := 4;
    signal env_pwr    : env_word_t;
    signal env_v      : std_logic;

    -- Carrier-PLL lock indicator (real-mode output gating).
    signal pll_lock   : std_logic;
    signal pll_qfilt  : signed(POST_W - 1 downto 0);
    signal pll_qlock  : signed(POST_W - 1 downto 0);

    signal bit_val    : std_logic;
    signal bit_v      : std_logic;
    signal minute_mk  : std_logic;
    signal sync_st    : sync_state_t;
    signal sync_q_s   : unsigned(5 downto 0);
    signal mm_q_s     : unsigned(1 downto 0);
    signal cycle_s    : unsigned(5 downto 0);

    -- Carrier-PLL-gated decoder outputs.
    signal bit_v_gated     : std_logic;
    signal minute_mk_gated : std_logic;

    signal am_soft_s       : signed(11 downto 0);
    signal am_soft_v_s     : std_logic;
    signal mm_soft_s       : signed(11 downto 0);
    signal anchor_held_s   : std_logic;
    signal sync_st_gated   : sync_state_t;
    signal env_max_s  : env_word_t;
    signal env_min_s  : env_word_t;
    signal noise_s    : env_word_t;
    signal thresh_s   : env_word_t;
    signal et_falling_edge_s : std_logic;

    signal time_d     : dcf77_time_type;
    signal time_v_d   : std_logic;
    signal last_frm   : std_logic_vector(0 to 58);
    signal bit_ix     : bit_index_t;

    -- ------------------------------------------------------------------ --
    -- PM (PSK/SIK) demod chain.                                          --
    -- ------------------------------------------------------------------ --
    signal pm_soft_s         : signed(11 downto 0);
    signal pm_valid_s        : std_logic;
    signal pm_corr_mm_s      : std_logic;             -- correlator's per-second pulse
    signal pm_sync_state_s   : std_logic_vector(1 downto 0);
    signal pm_locked_phase_s : unsigned(11 downto 0);
    signal pm_lock_qual_s    : unsigned(15 downto 0);

    signal pm_time_d         : dcf77_time_type;
    signal pm_time_v_d       : std_logic;
    signal pm_last_frm       : std_logic_vector(0 to 58);
    signal pm_bit_ix         : bit_index_t;
    signal pm_frame_mm_s     : std_logic;             -- frame_dec's SIK-signature edge

    -- pm_sec_index = bit_ix snapshot at the moment pm_valid pulses.
    -- pm_bit_index = the live frame_dec counter (= pm_bit_ix above).
    signal pm_sec_index_s    : bit_index_t;

    -- Cast top 16 b of an env_word_t (sfixed, but always >= 0 in this
    -- design) to rssi_word_t.  Skips the sign bit and takes the 16 next
    -- bits.
    function env_to_rssi(x : env_word_t) return rssi_word_t is
        constant TOP : natural := x'high - 1;          -- skip sign bit
        constant BOT : natural := TOP - 15;
        variable slv : std_logic_vector(env_word_t'length - 1 downto 0);
    begin
        slv := to_slv(x);
        return ufixed(slv(TOP - x'low downto BOT - x'low));
    end function;

begin

    -- ------------------------------------------------------------------ --
    -- CIC cascade                                                          --
    -- ------------------------------------------------------------------ --
    u_cic0 : entity work.dcf77_cic_dec
        generic map (
            N          => CIC_N,
            DEC_R      => CIC0_R,
            M          => CIC_M,
            IN_WIDTH   => CIC0_IN_W,
            INT_WIDTH  => CIC0_INT_W,
            COMB_WIDTH => CIC0_COMB_W,
            OUT_WIDTH  => CIC0_OUT_W
        )
        port map (
            clk        => clk,
            rst        => rst,
            in_sample  => in_sample,
            in_valid   => in_valid,
            out_sample => cic0_out,
            out_valid  => cic0_v
        );

    u_cic1 : entity work.dcf77_cic_dec
        generic map (
            N          => CIC_N,
            DEC_R      => CIC1_R,
            M          => CIC_M,
            IN_WIDTH   => CIC1_IN_W,
            INT_WIDTH  => CIC1_INT_W,
            COMB_WIDTH => CIC1_COMB_W,
            OUT_WIDTH  => CIC1_OUT_W
        )
        port map (
            clk        => clk,
            rst        => rst,
            in_sample  => cic0_out,
            in_valid   => cic0_v,
            out_sample => cic1_out,
            out_valid  => cic1_v
        );

    u_cic2 : entity work.dcf77_cic_dec
        generic map (
            N          => CIC_N,
            DEC_R      => CIC2_R,
            M          => CIC_M,
            IN_WIDTH   => CIC2_IN_W,
            INT_WIDTH  => CIC2_INT_W,
            COMB_WIDTH => CIC2_COMB_W,
            OUT_WIDTH  => CIC2_OUT_W
        )
        port map (
            clk        => clk,
            rst        => rst,
            in_sample  => cic1_out,
            in_valid   => cic1_v,
            out_sample => cic2_out,
            out_valid  => cic2_v
        );

    -- ------------------------------------------------------------------ --
    -- NCO + mixer                                                          --
    -- ------------------------------------------------------------------ --
    u_nco : entity work.dcf77_nco
        generic map (
            PHASE_WIDTH => NCO_PHASE_W,
            PHASE_INCR  => NCO_PHASE_INCR
        )
        port map (
            clk              => clk,
            rst              => rst,
            tick             => cic2_v,
            phase_inc_offset => nco_phase_inc_offset,
            cos_sign         => cos_sign,
            sin_sign         => sin_sign,
            phase_acc_out    => nco_phase_acc
        );

    u_mixer : entity work.dcf77_mixer
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

    -- ------------------------------------------------------------------ --
    -- Post-mixer chain (flat single-stage): boxcar R=125 collapses both
    -- channels from 250 kS/s to 2 kS/s in one decimation step.  The
    -- previous 4-stage CIC3/CIC4/CIC_ENV cascade is gone.
    -- ------------------------------------------------------------------ --
    u_i_boxcar : entity work.dcf77_boxcar
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

    u_q_boxcar : entity work.dcf77_boxcar
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

    -- ------------------------------------------------------------------ --
    -- CORDIC I/Q-to-polar.  Consumes the top 18 b of the post-boxcar    --
    -- I/Q (drops the bottom 11 LSBs).  Produces 19-b unsigned magnitude --
    -- (= K·√(I²+Q²)) and 12-b signed phase (= atan2(Q, I)).             --
    -- ------------------------------------------------------------------ --
    u_cordic : entity work.dcf77_cordic_atan2
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

    -- ------------------------------------------------------------------ --
    -- Magnitude IIR (= the 20 Hz noise-bandwidth limit before envelope  --
    -- detection that the OLD i_iir provided).  CORDIC magnitude is      --
    -- naturally non-negative so we just prepend a 0 to feed the signed  --
    -- IIR module.                                                        --
    -- ------------------------------------------------------------------ --
    mag_signed <= signed('0' & std_logic_vector(cordic_mag));

    u_mag_iir : entity work.dcf77_iir_lpf
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

    -- /10 decimating boxcar.  Accumulates ENV_DEC_RATE mag_filt samples
    -- then emits the running sum.  Free 13-dB anti-alias notches at
    -- 200/400/... Hz; group delay ~ 4.5 input samples = 2.25 ms.
    env_dec_proc : process(clk)
        variable cnt : natural range 0 to ENV_DEC_RATE - 1 := 0;
        variable acc : signed(CORDIC_IQ_W + 1 + 4 downto 0);
    begin
        if rising_edge(clk) then
            if rst = '1' then
                mag_dec_sum <= (others => '0');
                mag_dec_v   <= '0';
                cnt := 0;
                acc := (others => '0');
            else
                mag_dec_v <= '0';
                if mag_filt_v = '1' then
                    if cnt = ENV_DEC_RATE - 1 then
                        mag_dec_sum <= acc + resize(mag_filt, mag_dec_sum'length);
                        mag_dec_v   <= '1';
                        cnt := 0;
                        acc := (others => '0');
                    else
                        acc := acc + resize(mag_filt, acc'length);
                        cnt := cnt + 1;
                    end if;
                end if;
            end if;
        end if;
    end process;

    -- env_pwr scale: legacy env_pwr = 128 x mag_filt (= shift_left by 7).
    -- After /10 decimation: mag_dec_sum = 10 x mag_filt on average.
    -- shift_left by 4 -> env_pwr = 16 x sum = 160 x mag_filt = 1.25 x
    -- legacy.  Slight scale shift but within env_tracker's relative-
    -- threshold tolerance (env_max-based hysteresis).
    env_pwr <= resize(
        to_sfixed(std_logic_vector(
            resize(shift_left(resize(mag_dec_sum, CORDIC_IQ_W + 6 + 4), 4),
                   CORDIC_IQ_W + 6 + 4)),
            CORDIC_IQ_W + 6 + 4 - 1, 0),
        env_word_t'high, env_word_t'low,
        fixed_saturate, fixed_truncate);
    env_v <= mag_dec_v;

    -- ------------------------------------------------------------------ --
    -- Carrier-PLL adapter: left-shift CORDIC phase by 11 bits to match  --
    -- the magnitude scale the PI gains were calibrated against.  Note    --
    -- the gains used here (KP=10, KI=23) are *not* the legacy 7/20 from --
    -- the amplitude-weighted sign(I)·q_box detector — see the           --
    -- u_carrier_pll instantiation below + the carrier-pll-tuning memo.   --
    -- ------------------------------------------------------------------ --
    -- Sign inversion: the carrier_pll loop's "q ≈ A · φ" convention
    -- comes from the legacy sign(I)·q_box detector.  CORDIC's
    -- atan2(Q, I) output has the opposite sign for the same physical
    -- phase shift through our mixer + boxcar geometry.  Without this
    -- negation the loop runs positive-feedback and limit-cycles
    -- around its IIR pole (locked-σ stuck near uniform [-π, +π]).
    -- With negation, KP=10/KI=23 converges as designed.
    pll_adapter_proc : process(clk)
    begin
        if rising_edge(clk) then
            if rst = '1' then
                phase_for_pll   <= (others => '0');
                phase_for_pll_v <= '0';
            else
                phase_for_pll   <= shift_left(resize(-cordic_phase, POST_W),
                                              PLL_PHASE_ADAPTER_SHIFT);
                phase_for_pll_v <= cordic_v;
            end if;
        end if;
    end process;

    -- External tap: the amplitude-independent CORDIC phase directly
    -- (no adapter shift; consumers like the PM correlator want phase
    -- units, not the PLL-internal amplitude-weighted scale).
    phase_err_out   <= cordic_phase;
    phase_err_valid <= cordic_v;

    -- ------------------------------------------------------------------ --
    -- Q-channel -> carrier PLL.  The PLL's internal IIRs handle both
    -- noise filtering (LOOP_IIR_SHIFT) and lock detection (LOCK_IIR_SHIFT).
    -- ------------------------------------------------------------------ --
    u_carrier_pll : entity work.dcf77_carrier_pll
        generic map (
            Q_WIDTH         => POST_W,          -- 29
            OFFSET_WIDTH    => NCO_OFFSET_W,    -- 24
            -- PI gains: KP=10/KI=23 → ω_n ≈ 0.14 Hz, ζ ≈ 1.76
            -- (heavily overdamped, narrow tracking).  Stays well below
            -- the chip rate (645 Hz) so the PI cannot react to PSK
            -- transitions.  These gains paired with the *negated*
            -- cordic_phase feed (see pll_adapter_proc above) pass the
            -- locked-σ < 400 LSB sanity check on the corpus.
            KP_SHIFT        => 10,
            KI_SHIFT        => 23,
            -- Internal IIRs.
            LOOP_IIR_SHIFT  => LOOP_IIR_SHIFT,  -- 4 -> ~ 20 Hz
            LOCK_IIR_SHIFT  => 14,              -- ~ 8.2 s
            -- Lock thresholds on the slow-IIR |phase error|.  Tune per
            -- deployment SNR; defaults sized for ~2^20 carrier amplitude.
            LOCK_IN_THR     => 2**18,
            LOCK_OUT_THR    => 2**21,
            LOCK_HOLD_TICKS => 256
        )
        port map (
            clk              => clk,
            rst              => rst,
            -- Amplitude-independent CORDIC phase, scaled into the
            -- magnitude range the PI gains were calibrated for.
            q_sample         => phase_for_pll,
            q_valid          => phase_for_pll_v,
            phase_inc_offset => nco_phase_inc_offset,
            lock_state       => pll_lock,
            q_filt_out       => pll_qfilt,
            q_lock_out       => pll_qlock
        );

    -- ------------------------------------------------------------------ --
    -- Envelope statistics + falling-edge detector.  Carved out of        --
    -- bit_det so the slow-tracker / edge-detect machinery doesn't bloat  --
    -- bit_det's reg_type record (= faster GHDL simulation, cleaner       --
    -- separation of concerns).                                            --
    -- ------------------------------------------------------------------ --
    u_env_tracker : entity work.dcf77_env_tracker
        generic map (
            K_ATK       => K_ATK,
            K_DEC       => K_DEC,
            K_NOISE     => K_NOISE,
            DERIV_DELAY => DERIV_DELAY,
            K_SLOPE     => K_SLOPE
        )
        port map (
            clk          => clk,
            rst          => rst,
            env_in       => env_pwr,
            clock_en     => env_v,
            env_max      => env_max_s,
            env_min      => env_min_s,
            noise        => noise_s,
            falling_edge => et_falling_edge_s
        );

    u_bit_det : entity work.dcf77_bit_det
        generic map (
            -- 200 S/s window sizes (10x smaller than bit_det's 2 kS/s
            -- defaults).  Symbol decision logic is unchanged; only the
            -- sample counts shrink.  LLR shifts drop by 3 b to absorb
            -- the 10x smaller W1/W2 accumulator sums.
            BIT_PERIOD     => 200,
            W1_END         => 20,
            W2_END         => 40,
            W3_END         => 200,
            GIVE_UP_TICKS  => 2000,
            AM_LLR_SHIFT   => 4,
            MM_LLR_SHIFT   => 4,
            K_REF          => K_REF,
            SAMPLE_RATE_HZ => SAMPLE_RATE_HZ,
            T_HOLD_SEC     => T_HOLD_SEC,
            T_SEARCH_SEC   => T_SEARCH_SEC
        )
        port map (
            clk           => clk,
            rst           => rst,
            env_in        => env_pwr,
            clock_en      => env_v,
            env_max_in    => env_max_s,
            env_min_in    => env_min_s,
            noise_in      => noise_s,
            falling_edge  => et_falling_edge_s,
            bit_value     => bit_val,
            bit_valid     => bit_v,
            minute_mark   => minute_mk,
            sync_state    => sync_st,
            sync_quality  => sync_q_s,
            mm_quality    => mm_q_s,
            cycle_in_min  => cycle_s,
            env_max       => open,         -- already exposed from env_tracker
            env_min       => open,
            noise         => open,
            threshold     => thresh_s,
            am_soft       => am_soft_s,
            am_soft_valid => am_soft_v_s,
            mm_soft       => mm_soft_s,
            anchor_held   => anchor_held_s,
            edge_accepted => open
        );

    -- ------------------------------------------------------------------ --
    -- Carrier-PLL gating: outputs only flow through the frame decoder
    -- while the PLL is locked.  Pre-lock sync_state is forced to
    -- sync_search so a stale lock indication can't leak through the
    -- AXI status register before the receiver has actually acquired.
    -- ------------------------------------------------------------------ --
    bit_v_gated     <= bit_v     and pll_lock;
    minute_mk_gated <= minute_mk and pll_lock;

    process (pll_lock, sync_st)
    begin
        if pll_lock = '0' then
            sync_st_gated <= sync_search;
        else
            sync_st_gated <= sync_st;
        end if;
    end process;

    -- ------------------------------------------------------------------ --
    -- Frame decoder                                                        --
    -- ------------------------------------------------------------------ --
    u_frame : entity work.dcf77_frame_dec
        generic map (
            PM_MODE => false
        )
        port map (
            clk             => clk,
            rst             => rst,
            bit_value       => bit_val,
            bit_valid       => bit_v_gated,
            minute_mark     => minute_mk_gated,
            time_out        => time_d,
            time_valid      => time_v_d,
            last_frame      => last_frm,
            bit_index       => bit_ix,
            minute_mark_out => open
        );

    -- ------------------------------------------------------------------ --
    -- PM (PSK/SIK) correlator: time-multiplexed matched filter over the --
    -- CORDIC phase stream.  Emits one pm_soft (SIK bit LLR) per locked  --
    -- second; sync state machine handles ACQUIRE → LOCK → re-acquire.   --
    -- ------------------------------------------------------------------ --
    u_pm_corr : entity work.dcf77_pm_correlator
        port map (
            clk              => clk,
            rst              => rst,
            -- Drop the 3 LSBs of the 12-b CORDIC phase to feed the
            -- 9-b PM correlator input (= 1 LSB ≈ 0.7°, plenty given
            -- the matched filter's 32 dB processing gain).
            phase_err        => cordic_phase(CORDIC_PHASE_W - 1
                                              downto CORDIC_PHASE_W - 9),
            phase_err_v      => cordic_v,
            pll_locked       => pll_lock,
            pm_soft          => pm_soft_s,
            pm_valid         => pm_valid_s,
            pm_minute_mark_o => pm_corr_mm_s,
            pm_sync_state    => pm_sync_state_s,
            pm_locked_phase  => pm_locked_phase_s,
            pm_lock_qual     => pm_lock_qual_s,
            pm_debug         => open
        );

    -- PM frame decoder.  PM_MODE=true → START_INDEX=15 + internal SIK
    -- signature detector; ignores the external minute_mark port.
    -- The bit_value fed in is the sign of pm_soft (12-b signed; sign
    -- bit '1' = negative = SIK '1' bit; sign bit '0' = positive = SIK '0').
    u_pm_frame : entity work.dcf77_frame_dec
        generic map (
            PM_MODE => true
        )
        port map (
            clk             => clk,
            rst             => rst,
            bit_value       => pm_soft_s(11),       -- sign bit of pm_soft
            bit_valid       => pm_valid_s,
            minute_mark     => '0',                 -- ignored for PM mode
            time_out        => pm_time_d,
            time_valid      => pm_time_v_d,
            last_frame      => pm_last_frm,
            bit_index       => pm_bit_ix,
            minute_mark_out => pm_frame_mm_s
        );

    -- Latch pm_bit_ix into pm_sec_index on each pm_valid pulse so the
    -- AXI SOFT_PM register's [5:0] field reports the second-of-minute
    -- that the pm_soft value belongs to, atomically with the LLR.
    pm_sec_index_proc : process(clk)
    begin
        if rising_edge(clk) then
            if rst = '1' then
                pm_sec_index_s <= 0;
            elsif pm_valid_s = '1' then
                pm_sec_index_s <= pm_bit_ix;
            end if;
        end if;
    end process;

    -- ------------------------------------------------------------------ --
    -- Aggregate the comprehensive status record from the sub-blocks.      --
    -- ------------------------------------------------------------------ --
    status_out.time         <= time_d;
    status_out.time_valid   <= time_v_d;
    status_out.sync_state   <= sync_st_gated;
    status_out.sync_quality <= sync_q_s;
    status_out.mm_quality   <= mm_q_s;
    status_out.cycle_in_min <= cycle_s;
    status_out.last_frame   <= last_frm;
    status_out.bit_index    <= bit_ix;

    -- RSSI, ENV_MIN, NOISE: top 16 bits of the respective sfixed values.
    -- Each is non-negative by construction (env_max and env_min track
    -- magnitudes; noise_iir is a MAD), so we skip the sign bit and take
    -- the next 16 b as a ufixed magnitude.
    status_out.rssi    <= env_to_rssi(env_max_s);
    status_out.env_min <= env_to_rssi(env_min_s);
    status_out.noise   <= env_to_rssi(noise_s);

    -- Soft metric and per-bit status flags.  bit_index in the status
    -- record provides the second_index for the SOFT_AM register.
    status_out.am_soft       <= am_soft_s;
    status_out.am_soft_valid <= am_soft_v_s;
    status_out.mm_soft       <= mm_soft_s;
    status_out.minute_mark   <= minute_mk_gated;
    status_out.pll_lock      <= pll_lock;
    status_out.anchor_held   <= anchor_held_s;

    -- PM-side status fields.
    status_out.pm_soft        <= pm_soft_s;
    status_out.pm_valid       <= pm_valid_s;
    status_out.pm_minute_mark <= pm_frame_mm_s;
    status_out.pm_sec_index   <= pm_sec_index_s;
    status_out.pm_bit_index   <= pm_bit_ix;
    status_out.pm_sync_state  <= pm_sync_state_s;
    status_out.pm_lock_qual   <= pm_lock_qual_s;
    status_out.pm_time        <= pm_time_d;
    status_out.pm_time_valid  <= pm_time_v_d;
    status_out.pm_last_frame  <= pm_last_frm;

end architecture rtl;
