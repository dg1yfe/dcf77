-- =========================================================================
-- dcf77_carrier_pll.vhd
--
-- Carrier-tracking 2nd-order PI PLL.  Reads the scaled CORDIC phase
-- error (signed) and produces a signed phase-increment offset that the
-- upstream NCO adds to its nominal step.
--
-- Lock / signal-quality indication is a separate concern handled by
-- dcf77_phase_quality, which operates on the raw 12-b CORDIC phase
-- rather than the PI-scaled / 20-Hz-LPFed phase_filt.  Keeping the
-- two separate lets the PI loop be a pure tracker and the lock signal
-- be a deliberate SNR call.
--
-- Phase detector
-- --------------
-- The phase error is produced upstream by CORDIC atan2 of the mixer's
-- post-boxcar I/Q vector, scaled into a signed PHASE_ERR_WIDTH-bit
-- amplitude-independent unit (legacy: ~1.33 M LSB per radian after the
-- ×2^11 adapter shift in dcf77_core).  Driving phase_err -> 0 locks
-- the LO to the carrier.
--
-- Signal chain inside the PLL
-- ---------------------------
--   phase_err ─► loop_iir (1st-order IIR, SHIFT=LOOP_IIR_SHIFT, ~20 Hz)
--                    │
--                    ▼  phase_filt
--                  PI loop filter  ─►  phase_inc_offset
--
-- Loop filter (shift-only, no multipliers)
-- ----------------------------------------
--   alpha = 2^-KP_SHIFT     (Kp,  proportional gain)
--   beta  = 2^-KI_SHIFT     (Ki,  integral gain)
--
-- The integral path needs full-precision accumulation: a per-sample
-- `phase_filt >> KI_SHIFT` would arithmetic-shift small samples to 0/-1
-- asymmetrically, biasing the accumulator by -0.5 LSB/sample.  Instead
-- the per-sample shift is only KI_SHIFT - I_FRAC_BITS, and the resulting
-- i_state carries I_FRAC_BITS extra fractional bits below the offset
-- LSB; those bits absorb the per-sample rounding so the residual bias
-- scales as ~fs / 2^(I_FRAC_BITS+1) LSB/s.
--
--   i_step           = phase_filt >> (KI_SHIFT - I_FRAC_BITS)
--   i_state         += i_step                    (ISTATE_W b, saturated)
--   p_path           = phase_filt >> KP_SHIFT
--   phase_inc_offset = sat( (i_state >> I_FRAC_BITS) + p_path )
-- =========================================================================

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity dcf77_carrier_pll is
    generic (
        PHASE_ERR_WIDTH : positive := 29;     -- signed phase_err input
        OFFSET_WIDTH    : positive := 24;     -- signed NCO phase_inc_offset

        -- PI gains:  alpha = 2^-KP_SHIFT, beta = 2^-KI_SHIFT.
        KP_SHIFT       : positive := 7;
        KI_SHIFT       : positive := 20;

        -- Fractional bits carried in i_state below the offset LSB
        -- (per-sample integrator shift is KI_SHIFT - I_FRAC_BITS).
        I_FRAC_BITS    : positive := 10;

        -- Pre-loop IIR low-pass (cutoff ≈ fs / (2π · 2^SHIFT)).  SHIFT=0
        -- bypasses the filter (phase_filt tracks phase_err instantaneously).
        LOOP_IIR_SHIFT : natural  := 4        -- ~ 20 Hz @ 2 kS/s
    );
    port (
        clk              : in  std_logic;
        rst              : in  std_logic;

        phase_err        : in  signed(PHASE_ERR_WIDTH - 1 downto 0);
        phase_err_valid  : in  std_logic;

        phase_inc_offset : out signed(OFFSET_WIDTH - 1 downto 0);

        -- Diagnostic output: 20-Hz LPF state (live, not sampled).
        phase_filt_out   : out signed(PHASE_ERR_WIDTH - 1 downto 0);

        -- Snapshot outputs.  Updated on snap_tick='1' (typically driven
        -- by pm_valid, = once per locked second).  Software polls these
        -- via AXI for per-second carrier-state telemetry.
        --   * phase_filt_snap : current 20-Hz LPF state (1st-order signal).
        --   * i_state_snap    : PI integrator state, shifted right by
        --                       I_FRAC_BITS so the units match
        --                       phase_inc_offset (= NCO phase-increment
        --                       units, same as i_part contributes).
        snap_tick        : in  std_logic := '0';
        phase_filt_snap  : out signed(PHASE_ERR_WIDTH - 1 downto 0);
        i_state_snap     : out signed(OFFSET_WIDTH + 3 downto 0);

        -- Writable PI jams.  On each *_wr='1' clk pulse (driven by
        -- the AXI writemux on a register write), the corresponding
        -- internal register is overwritten with *_val.  Jams take
        -- effect on the same clk (any clk, not gated by
        -- phase_err_valid -- so AXI writes can't fall between sample
        -- ticks and be missed).  PI loop resumes normally on the
        -- next phase_err_valid using the jammed value as its new
        -- starting point.
        --   * offset_wr / offset_val
        --       Jam phase_inc_offset.  Single-cycle effect until
        --       the next phase_err_valid, then PI overwrites.  Mostly
        --       useful for bench-bumping the NCO.
        --   * i_state_wr / i_state_val
        --       Jam the PI integrator.  i_state_val is in NCO-offset
        --       units; HW shifts left by I_FRAC_BITS to land in the
        --       integer part of i_state (fractional zeroed).  PI
        --       continues to accumulate from there.
        offset_wr        : in  std_logic := '0';
        offset_val       : in  signed(OFFSET_WIDTH - 1 downto 0) := (others => '0');
        i_state_wr       : in  std_logic := '0';
        i_state_val      : in  signed(OFFSET_WIDTH + 3 downto 0) := (others => '0')
    );
end entity dcf77_carrier_pll;

architecture rtl of dcf77_carrier_pll is

    -- Per-sample shift on the integral path (KI_SHIFT - I_FRAC_BITS).
    constant K_PRE    : natural  := KI_SHIFT - I_FRAC_BITS;

    -- i_state width: OFFSET_WIDTH integer bits + I_FRAC_BITS fractional
    -- bits + a few bits of transient headroom.
    constant ISTATE_W : positive := OFFSET_WIDTH + I_FRAC_BITS + 4;

    -- Width used to build the (i_part + p_path) sum before saturation.
    constant SUM_W    : positive := OFFSET_WIDTH + 4;

    -- Convenience alias for the phase-error data width.
    constant PE_W     : positive := PHASE_ERR_WIDTH;

    -- i_state expressed in NCO-offset units (= shifted right by
    -- I_FRAC_BITS).  Both for the read snapshot and the jam-write input.
    constant ISNAP_W  : positive := OFFSET_WIDTH + 4;

    type reg_type is record
        phase_filt   : signed(PE_W - 1 downto 0);
        i_state      : signed(ISTATE_W - 1 downto 0);
        offset       : signed(OFFSET_WIDTH - 1 downto 0);
        phase_filt_s : signed(PE_W - 1 downto 0);          -- snapshot
        i_state_s    : signed(ISNAP_W - 1 downto 0);       -- snapshot (shifted)
    end record;

    constant ZERO_IS  : signed(ISTATE_W - 1 downto 0)     := (others => '0');
    constant ZERO_OFF : signed(OFFSET_WIDTH - 1 downto 0) := (others => '0');
    constant ZERO_PE  : signed(PE_W - 1 downto 0)         := (others => '0');
    constant ZERO_SN  : signed(ISNAP_W - 1 downto 0)      := (others => '0');

    constant R_RESET : reg_type := (
        phase_filt   => ZERO_PE,
        i_state      => ZERO_IS,
        offset       => ZERO_OFF,
        phase_filt_s => ZERO_PE,
        i_state_s    => ZERO_SN
    );

    signal r, rin : reg_type;

    -- Symmetric saturation of `x` into a signed of width `W`.
    function sat_to(x : signed; W : natural) return signed is
        variable hi  : signed(W - 1 downto 0);
        variable lo  : signed(W - 1 downto 0);
        variable msb : std_logic;
    begin
        hi := (others => '1');
        hi(W - 1) := '0';
        lo := (others => '0');
        lo(W - 1) := '1';

        if x'length <= W then
            return resize(x, W);
        end if;
        msb := x(x'high);
        for i in W - 1 to x'high loop
            if x(i) /= msb then
                if msb = '1' then
                    return lo;
                else
                    return hi;
                end if;
            end if;
        end loop;
        return x(W - 1 downto 0);
    end function;

begin

    phase_inc_offset <= r.offset;
    phase_filt_out   <= r.phase_filt;
    phase_filt_snap  <= r.phase_filt_s;
    i_state_snap     <= r.i_state_s;

    comb : process (all)
        variable v          : reg_type;

        -- LPF scratch (one extra bit for the diff).
        variable loop_diff  : signed(PE_W downto 0);
        variable loop_step  : signed(PE_W downto 0);
        variable loop_new   : signed(PE_W downto 0);

        -- Integral path.
        variable i_step     : signed(PE_W - 1 downto 0);
        variable i_sum      : signed(ISTATE_W downto 0);
        variable i_part     : signed(SUM_W - 1 downto 0);

        -- Proportional path + offset sum.
        variable p_path     : signed(PE_W - 1 downto 0);
        variable sum_ext    : signed(SUM_W - 1 downto 0);
    begin
        v := r;

        if phase_err_valid = '1' then

            -- ===== Pre-loop IIR (20 Hz LPF on phase_err) ================
            loop_diff := resize(phase_err, PE_W + 1)
                       - resize(r.phase_filt, PE_W + 1);
            loop_step := shift_right(loop_diff, LOOP_IIR_SHIFT);
            loop_new  := resize(r.phase_filt, PE_W + 1) + loop_step;
            v.phase_filt := resize(loop_new, PE_W);

            -- ===== Integral path (wide accumulator) =====================
            i_step    := shift_right(v.phase_filt, K_PRE);
            i_sum     := resize(r.i_state, ISTATE_W + 1)
                       + resize(i_step,    ISTATE_W + 1);
            v.i_state := sat_to(i_sum, ISTATE_W);

            -- ===== Proportional path + offset ===========================
            p_path  := shift_right(v.phase_filt, KP_SHIFT);
            i_part  := resize(shift_right(v.i_state, I_FRAC_BITS), SUM_W);
            sum_ext := i_part + resize(p_path, SUM_W);
            v.offset := sat_to(sum_ext, OFFSET_WIDTH);

        end if;

        -- AXI write jams.  Applied OUTSIDE the phase_err_valid gate so
        -- a single-clk pulse from the writemux always lands.  On the
        -- next phase_err_valid the PI loop continues from the jammed
        -- state -- the override is one-clk; it doesn't hold.
        if i_state_wr = '1' then
            v.i_state := resize(shift_left(resize(i_state_val, ISTATE_W),
                                           I_FRAC_BITS),
                                ISTATE_W);
        end if;
        if offset_wr = '1' then
            v.offset := offset_val;
        end if;

        -- Snapshot capture (independent of phase_err_valid -- snap_tick
        -- is asynchronous to the PI loop's 2 kS/s rate; both i_state
        -- and phase_filt are slow-varying so a one-cycle skew is fine).
        if snap_tick = '1' then
            v.phase_filt_s := r.phase_filt;
            v.i_state_s    := resize(shift_right(r.i_state, I_FRAC_BITS),
                                     ISNAP_W);
        end if;

        rin <= v;
    end process;

    seq : process (clk)
    begin
        if rising_edge(clk) then
            if rst = '1' then
                r <= R_RESET;
            else
                r <= rin;
            end if;
        end if;
    end process;

end architecture rtl;
