-- =========================================================================
-- dcf77_core.vhd
--
-- Top-level wiring of the DCF77 receiver / decoder:
--
--   adc -> CIC0(R=2) -> CIC1(R=16) -> CIC2(R=16) ---+--> mixer -+-> env -> bit_det
--                                                    |          |
--                                                    NCO(244k)  +-> iq_tap (PM hook)
--                                                                       |
--                                                              bit_det -+-> frame_dec
--                                                                          |
--                                                                          time_out / status
--
-- All blocks live in the 125 MHz clock domain; inner CIC stages advance on
-- valid strobes, so XDC multicycle constraints (factor 2 for CIC1, factor
-- 32 for CIC2) are required at synthesis to actually relax their timing.
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
        -- bit_det timing: defaults match the real 244.140 kS/s rate.  Tests
        -- can scale these down to fit a short simulation budget.
        K_ATK         : positive := 6;
        K_DEC         : positive := 20;
        T_LOW_MIN     : positive := 21_973;
        T_BIT0_MAX    : positive := 26_855;
        T_BIT1_MIN    : positive := 46_387;
        T_BIT1_MAX    : positive := 51_269;
        T_GAP_MINUTE  : positive := 366_211;

        -- NCO frequency increment (per 244 kS/s tick).
        NCO_PHASE_INCR : natural := 5_326_192
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

    -- Inter-stage signals.
    signal cic0_out  : sfixed(CIC0_OUT_W - 1 downto 0);
    signal cic0_v    : std_logic;
    signal cic1_out  : sfixed(CIC1_OUT_W - 1 downto 0);
    signal cic1_v    : std_logic;
    signal cic2_out  : sfixed(CIC2_OUT_W - 1 downto 0);  -- 22 b signed @ 244 kS/s
    signal cic2_v    : std_logic;

    signal cos_sign  : std_logic;
    signal sin_sign  : std_logic;

    signal i_mix     : iq_word_t;
    signal q_mix     : iq_word_t;
    signal mix_v     : std_logic;

    signal env_pwr   : env_word_t;
    signal env_v     : std_logic;

    signal bit_val   : std_logic;
    signal bit_v     : std_logic;
    signal minute_mk : std_logic;
    signal sync_st   : sync_state_t;
    signal env_max_s : env_word_t;
    signal thresh_s  : env_word_t;

    signal time_d    : dcf77_time_type;
    signal time_v_d  : std_logic;
    signal last_frm  : std_logic_vector(0 to 58);
    signal bit_ix    : unsigned(5 downto 0);

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
            PHASE_WIDTH => 24,
            PHASE_INCR  => NCO_PHASE_INCR
        )
        port map (
            clk      => clk,
            rst      => rst,
            tick     => cic2_v,
            cos_sign => cos_sign,
            sin_sign => sin_sign
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

    -- PM hook: expose the post-mixer I/Q at 244 kS/s.
    iq_tap_out.i_data <= i_mix;
    iq_tap_out.q_data <= q_mix;
    iq_tap_out.valid  <= mix_v;

    -- ------------------------------------------------------------------ --
    -- Envelope power + bit detector                                        --
    -- ------------------------------------------------------------------ --
    u_env : entity work.dcf77_env
        port map (
            clk       => clk,
            rst       => rst,
            i_in      => i_mix,
            q_in      => q_mix,
            in_valid  => mix_v,
            power     => env_pwr,
            out_valid => env_v
        );

    u_bit_det : entity work.dcf77_bit_det
        generic map (
            K_ATK        => K_ATK,
            K_DEC        => K_DEC,
            T_LOW_MIN    => T_LOW_MIN,
            T_BIT0_MAX   => T_BIT0_MAX,
            T_BIT1_MIN   => T_BIT1_MIN,
            T_BIT1_MAX   => T_BIT1_MAX,
            T_GAP_MINUTE => T_GAP_MINUTE
        )
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
            threshold   => thresh_s
        );

    -- ------------------------------------------------------------------ --
    -- Frame decoder                                                        --
    -- ------------------------------------------------------------------ --
    u_frame : entity work.dcf77_frame_dec
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
    -- Aggregate the comprehensive status record from the sub-blocks.      --
    -- ------------------------------------------------------------------ --
    status_out.time       <= time_d;
    status_out.time_valid <= time_v_d;
    status_out.sync_state <= sync_st;
    status_out.last_frame <= last_frm;
    status_out.bit_index  <= bit_ix;

    -- RSSI = top 16 bits of the sfixed env_max value (always non-negative,
    -- so reinterpreting the raw bits as ufixed is safe).  The MSB of env_max
    -- is the sign bit and is always '0' for a sum of squares.
    rssi_slice : block
        constant TOP : natural := env_max_s'high - 1;  -- skip sign bit
        constant BOT : natural := TOP - 15;
        signal slv : std_logic_vector(env_word_t'length - 1 downto 0);
    begin
        slv <= to_slv(env_max_s);
        status_out.rssi <= ufixed(slv(TOP - env_max_s'low downto BOT - env_max_s'low));
    end block;

end architecture rtl;
