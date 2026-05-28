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
        -- bit_det defaults: 2 kS/s tick rate (one envelope sample per
        -- 500 us).  All time-constant generics scaled vs the legacy
        -- 100 S/s defaults so continuous-time TCs are unchanged.
        K_ATK          : positive := 8;
        K_DEC          : positive := 14;
        K_NOISE        : positive := 14;
        DERIV_DELAY    : positive := 16;
        K_SLOPE        : positive := 2;
        K_REF          : positive := 2;
        SAMPLE_RATE_HZ : positive := 2000;
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
        phase_err_out   : out signed(28 downto 0);
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

    -- 20 Hz IIR LPF on the I channel (matches the IIR inside
    -- carrier_pll on the loop-filter path).  lambda = 2^-4 at 2 kS/s.
    constant LOOP_IIR_SHIFT : positive := 4;
    signal i_iir      : signed(POST_W - 1 downto 0);
    signal i_iir_v    : std_logic;

    -- Phase-error signal: sign(I_iir) * Q_box, computed at the
    -- boxcar valid tick (2 kS/s).  Feeds carrier_pll's input in
    -- place of the raw Q_box, so the PLL's internal IIR averages
    -- a polarity-resolved phase-error rather than raw Q.  Also
    -- exposed as a wider-bandwidth tap for downstream PM demod.
    signal phase_err   : signed(POST_W - 1 downto 0);
    signal phase_err_v : std_logic;

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

    signal time_d     : dcf77_time_type;
    signal time_v_d   : std_logic;
    signal last_frm   : std_logic_vector(0 to 58);
    signal bit_ix     : bit_index_t;

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
    -- I-channel 20 Hz IIR (lambda = 2^-4 at 2 kS/s) -> abs() ->          --
    -- env_word_t.  The IIR provides the noise-bandwidth limit before     --
    -- envelope detection.                                                 --
    -- ------------------------------------------------------------------ --
    u_i_iir : entity work.dcf77_iir_lpf
        generic map (
            WIDTH => POST_W,
            SHIFT => LOOP_IIR_SHIFT
        )
        port map (
            clk        => clk,
            rst        => rst,
            in_sample  => i_box,
            in_valid   => i_box_v,
            out_sample => i_iir,
            out_valid  => i_iir_v
        );

    -- ------------------------------------------------------------------ --
    -- Phase-error: sign(I_iir) * Q_box.  Multiplying Q by sign(I) makes  --
    -- the phase detector respond symmetrically to both stable lock      --
    -- points (Δφ=0 and Δφ=π) and gives a polarity-resolved error signal --
    -- whose magnitude tracks A·|sin(Δφ)| regardless of I's quadrant.    --
    -- Sample tick is i_iir_v so I and Q are aligned to the same         --
    -- 2 kS/s grid.                                                       --
    -- ------------------------------------------------------------------ --
    phase_err_proc : process(clk)
    begin
        if rising_edge(clk) then
            if rst = '1' then
                phase_err   <= (others => '0');
                phase_err_v <= '0';
            else
                phase_err_v <= '0';
                if i_iir_v = '1' then
                    if i_iir(POST_W - 1) = '1' then  -- I_iir negative
                        phase_err <= -q_box;
                    else
                        phase_err <=  q_box;
                    end if;
                    phase_err_v <= '1';
                end if;
            end if;
        end if;
    end process;

    phase_err_out   <= phase_err;
    phase_err_valid <= phase_err_v;

    -- Envelope detector.  Two software-selectable modes:
    --   env_mode='0': abs(I_iir).  Cheapest; relies on PLL lock to keep
    --                amplitude in I and zero in Q.  ~0.5 dB more
    --                sensitive than the magnitude mode in clean cases.
    --   env_mode='1': |I_iir| + 3/8 * |Q_iir|  (alpha=1, beta=3/8
    --                magnitude approximation; max error ±7 % over
    --                angle).  Phase-agnostic: robust against in-band
    --                CW interferers the PLL doesn't track.
    -- Q_iir == pll_qfilt = the same 20 Hz LPF that runs on I, applied
    -- internally inside dcf77_carrier_pll.
    --
    -- ENV_RSHIFT = 4 keeps env_pwr's peak around 2^24 (env_word_t = sfixed
    -- 25 downto 0) so bit_det's slow IIR running headroom is preserved.
    env_proc : process (i_iir, pll_qfilt, env_mode)
        variable i_abs, q_abs : unsigned(POST_W - 1 downto 0);
        variable i_ext, q_ext : signed(POST_W downto 0);
        variable a_max, a_min : unsigned(POST_W - 1 downto 0);
        variable min_term     : unsigned(POST_W - 1 downto 0);
        variable env_sum      : unsigned(POST_W downto 0);
        variable env_sh       : unsigned(POST_W downto 0);
    begin
        i_ext := resize(i_iir, POST_W + 1);
        if i_ext(POST_W) = '1' then i_ext := -i_ext; end if;
        i_abs := unsigned(i_ext(POST_W - 1 downto 0));

        q_ext := resize(pll_qfilt, POST_W + 1);
        if q_ext(POST_W) = '1' then q_ext := -q_ext; end if;
        q_abs := unsigned(q_ext(POST_W - 1 downto 0));

        -- alpha-max + beta-min magnitude approximation, alpha=1, beta=3/8.
        -- |V| ≈ max(|I|,|Q|) + 3/8 * min(|I|,|Q|).  Max error ±7 %.
        if i_abs >= q_abs then
            a_max := i_abs;
            a_min := q_abs;
        else
            a_max := q_abs;
            a_min := i_abs;
        end if;
        min_term := shift_right(a_min, 2) + shift_right(a_min, 3);

        if env_mode = '1' then
            env_sum := ('0' & a_max) + ('0' & min_term);
        else
            env_sum := ('0' & i_abs);
        end if;

        env_sh := shift_right(env_sum, ENV_RSHIFT);
        env_pwr <= resize(to_sfixed(std_logic_vector(env_sh(POST_W - 1 downto 0)),
                                    POST_W - 1, 0),
                          env_word_t'high, env_word_t'low,
                          fixed_saturate, fixed_truncate);
    end process;
    env_v <= i_iir_v;

    -- ------------------------------------------------------------------ --
    -- Q-channel -> carrier PLL.  The PLL's internal IIRs handle both
    -- noise filtering (LOOP_IIR_SHIFT) and lock detection (LOCK_IIR_SHIFT).
    -- ------------------------------------------------------------------ --
    u_carrier_pll : entity work.dcf77_carrier_pll
        generic map (
            Q_WIDTH         => POST_W,          -- 29
            OFFSET_WIDTH    => NCO_OFFSET_W,    -- 24
            -- PI gains (alpha = 2^-7, beta = 2^-25).
            KP_SHIFT        => 7,
            KI_SHIFT        => 20,
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
            -- PLL now sees the polarity-resolved phase-error rather
            -- than raw Q.  The PLL's internal 20 Hz IIR therefore
            -- filters phase_err -> pe_filt (still exposed via the
            -- existing q_filt_out port for backward-compat).
            q_sample         => phase_err,
            q_valid          => phase_err_v,
            phase_inc_offset => nco_phase_inc_offset,
            lock_state       => pll_lock,
            q_filt_out       => pll_qfilt,
            q_lock_out       => pll_qlock
        );

    u_bit_det : entity work.dcf77_bit_det
        generic map (
            K_ATK          => K_ATK,
            K_DEC          => K_DEC,
            K_NOISE        => K_NOISE,
            DERIV_DELAY    => DERIV_DELAY,
            K_SLOPE        => K_SLOPE,
            K_REF          => K_REF,
            SAMPLE_RATE_HZ => SAMPLE_RATE_HZ,
            T_HOLD_SEC     => T_HOLD_SEC,
            T_SEARCH_SEC   => T_SEARCH_SEC
        )
        port map (
            clk           => clk,
            rst           => rst,
            env_in        => env_pwr,
            env_valid     => env_v,
            bit_value     => bit_val,
            bit_valid     => bit_v,
            minute_mark   => minute_mk,
            sync_state    => sync_st,
            sync_quality  => sync_q_s,
            mm_quality    => mm_q_s,
            cycle_in_min  => cycle_s,
            env_max       => env_max_s,
            env_min       => env_min_s,
            noise         => noise_s,
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
        port map (
            clk         => clk,
            rst         => rst,
            bit_value   => bit_val,
            bit_valid   => bit_v_gated,
            minute_mark => minute_mk_gated,
            time_out    => time_d,
            time_valid  => time_v_d,
            last_frame  => last_frm,
            bit_index   => bit_ix
        );

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

end architecture rtl;
