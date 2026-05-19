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
        -- bit_det defaults: 100 S/s tick rate (one envelope sample per 10 ms).
        K_ATK          : positive := 3;
        K_DEC          : positive := 9;
        K_NOISE        : positive := 9;
        DERIV_DELAY    : positive := 2;
        K_SLOPE        : positive := 2;
        K_REF          : positive := 2;
        SAMPLE_RATE_HZ : positive := 100;
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

        -- Comprehensive decoder status: time, validity, last frame, RSSI,
        -- sync state, current bit index.  See dcf77_pkg.dcf77_status_type.
        status_out: out dcf77_status_type;
        iq_tap_out: out dcf77_iq_tap_type
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

    -- Post-mixer per-channel decimators.
    signal cic3_i_out : sfixed(CIC3_OUT_W - 1 downto 0);  -- 22 b @ 25 kS/s
    signal cic3_q_out : sfixed(CIC3_OUT_W - 1 downto 0);
    signal cic3_i_v   : std_logic;
    signal cic3_q_v   : std_logic;
    signal cic4_i_out : sfixed(CIC4_OUT_W - 1 downto 0);  -- 22 b @ 1 kS/s
    signal cic4_i_v   : std_logic;
    -- CIC4_Q removed: the carrier PLL consumes the post-mixer Q via a
    -- 1-stage boxcar on cic3_q_out (25 kS/s -> 1 kS/s), not via a
    -- second 3-stage CIC.  AM envelope after PLL lock is abs(I).

    -- Q-channel boxcar @ 1 kS/s feeding the carrier PLL.  R=25 from
    -- cic3_q_out @ 25 kS/s; width = 22 + ceil(log2(25)) = 27 b signed.
    constant Q_PLL_W  : positive := 27;
    signal q_pll      : signed(Q_PLL_W - 1 downto 0);
    signal q_pll_v    : std_logic;

    -- Carrier-PLL lock indicator (real-mode output gating).
    signal pll_lock   : std_logic;
    signal pll_qiir   : unsigned(Q_PLL_W - 1 downto 0);

    signal env_pwr    : env_word_t;
    signal env_v      : std_logic;

    -- Envelope decimator (CIC_ENV): 1 kS/s -> 100 S/s.  Reinterprets the
    -- env_word_t (sfixed(35..10)) bit pattern as sfixed(IN_W-1..0) on the
    -- way in and back to env_word_t on the way out.  The CIC's arithmetic
    -- doesn't depend on LSB position, only on bit count.
    signal env_pwr_cic    : sfixed(CIC_ENV_IN_W - 1 downto 0);
    signal env_dec_raw    : sfixed(CIC_ENV_OUT_W - 1 downto 0);
    signal env_dec_out    : env_word_t;
    signal env_dec_v      : std_logic;

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
    -- Post-mixer cascade (per I/Q channel): R=10 then R=25 = 250            --
    -- 250 kS/s -> 25 kS/s (PM tap) -> 1 kS/s                                --
    -- ------------------------------------------------------------------ --
    u_cic3_i : entity work.dcf77_cic_dec
        generic map (
            N          => CIC_N,
            DEC_R      => CIC3_R,
            M          => CIC_M,
            IN_WIDTH   => CIC3_IN_W,
            INT_WIDTH  => CIC3_INT_W,
            COMB_WIDTH => CIC3_COMB_W,
            OUT_WIDTH  => CIC3_OUT_W
        )
        port map (
            clk        => clk,
            rst        => rst,
            in_sample  => i_mix,
            in_valid   => mix_v,
            out_sample => cic3_i_out,
            out_valid  => cic3_i_v
        );

    u_cic3_q : entity work.dcf77_cic_dec
        generic map (
            N          => CIC_N,
            DEC_R      => CIC3_R,
            M          => CIC_M,
            IN_WIDTH   => CIC3_IN_W,
            INT_WIDTH  => CIC3_INT_W,
            COMB_WIDTH => CIC3_COMB_W,
            OUT_WIDTH  => CIC3_OUT_W
        )
        port map (
            clk        => clk,
            rst        => rst,
            in_sample  => q_mix,
            in_valid   => mix_v,
            out_sample => cic3_q_out,
            out_valid  => cic3_q_v
        );

    -- PM hook: post-CIC3 I/Q at 25 kS/s.  Either channel's valid will do
    -- (they tick together on the same mix_v stream).
    iq_tap_out.i_data <= cic3_i_out;
    iq_tap_out.q_data <= cic3_q_out;
    iq_tap_out.valid  <= cic3_i_v;

    u_cic4_i : entity work.dcf77_cic_dec
        generic map (
            N          => CIC_N,
            DEC_R      => CIC4_R,
            M          => CIC_M,
            IN_WIDTH   => CIC4_IN_W,
            INT_WIDTH  => CIC4_INT_W,
            COMB_WIDTH => CIC4_COMB_W,
            OUT_WIDTH  => CIC4_OUT_W
        )
        port map (
            clk        => clk,
            rst        => rst,
            in_sample  => cic3_i_out,
            in_valid   => cic3_i_v,
            out_sample => cic4_i_out,
            out_valid  => cic4_i_v
        );

    -- CIC4_Q is intentionally absent: after carrier-PLL lock the AM
    -- envelope is synchronously detected as abs(I) on the I-channel
    -- alone, and the PLL itself consumes the Q channel via the boxcar
    -- (cic3_q_out @ 25 kS/s) below.  The replay testbench instantiates
    -- its own dcf77_env (magnitude |I|+(3/8)|Q|) directly on the file
    -- I/Q because no NCO loop closure happens there.

    -- ------------------------------------------------------------------ --
    -- Q-channel boxcar + carrier-tracking PLL.
    -- 1-stage boxcar (R=25) on cic3_q_out @ 25 kS/s -> 1 kS/s, 27 b
    -- signed Q sample drives the 2nd-order PI loop, which feeds a 24 b
    -- signed phase-increment offset back into the NCO.  Lock state gates
    -- the bit-detector's outputs.
    -- ------------------------------------------------------------------ --
    u_q_boxcar : entity work.dcf77_q_boxcar
        generic map (
            IN_WIDTH  => CIC3_OUT_W,        -- 22
            RATE      => CIC4_R,            -- 25
            OUT_WIDTH => Q_PLL_W            -- 27
        )
        port map (
            clk        => clk,
            rst        => rst,
            in_sample  => cic3_q_out,
            in_valid   => cic3_q_v,
            out_sample => q_pll,
            out_valid  => q_pll_v
        );

    u_carrier_pll : entity work.dcf77_carrier_pll
        generic map (
            Q_WIDTH         => Q_PLL_W,         -- 27
            OFFSET_WIDTH    => NCO_OFFSET_W,    -- 24
            -- Lock thresholds in |q|_iir units; |q| peaks at ~ A * RATE
            -- where A is the post-CIC carrier amplitude.  For a strong
            -- carrier A ~ 2^20, |q|_peak ~ 25*2^20 = 2^24.6.  In-band
            -- ~ 1/16 of peak = 2^20; out-of-band ~ 1/4 = 2^23.  These
            -- defaults are starting points; tune per deployment.
            K_LOCK_SHIFT    => 6,
            LOCK_IN_THR     => 2**20,
            LOCK_OUT_THR    => 2**23,
            LOCK_HOLD_TICKS => 256
        )
        port map (
            clk              => clk,
            rst              => rst,
            q_sample         => q_pll,
            q_valid          => q_pll_v,
            phase_inc_offset => nco_phase_inc_offset,
            lock_state       => pll_lock,
            q_iir_out        => pll_qiir
        );

    -- ------------------------------------------------------------------ --
    -- Synchronously-detected AM envelope.
    --
    -- After carrier-PLL lock, Q -> 0 and all AM-modulation amplitude
    -- collapses onto I.  abs(I) is then the envelope, with the +3 dB
    -- SNR benefit of synchronous detection over magnitude detection.
    -- The 180-deg phase ambiguity (PLL locks at either of two phase-
    -- stable points) is handled by taking the absolute value.  No DSP,
    -- LUT-only.
    -- ------------------------------------------------------------------ --
    env_pwr <= resize(abs(cic4_i_out),
                      env_word_t'high, env_word_t'low,
                      fixed_saturate, fixed_truncate);
    env_v   <= cic4_i_v;

    -- env_word_t has LSB at 2^10 while dcf77_cic_dec's ports use LSB at 0;
    -- the bits transfer 1-for-1.  The CIC's Hogenauer-pruned output drops
    -- the gain's LSBs, so OUT_W=26 holds the natural value and no input
    -- pre-scaling is needed.
    env_pwr_cic <= to_sfixed(to_slv(env_pwr), CIC_ENV_IN_W - 1, 0);
    env_dec_out <= to_sfixed(to_slv(env_dec_raw),
                             env_word_t'high, env_word_t'low);

    u_env_dec : entity work.dcf77_cic_dec
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
            env_in        => env_dec_out,
            env_valid     => env_dec_v,
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

end architecture rtl;
