-- =========================================================================
-- dcf77_carrier_pll.vhd
--
-- Carrier-tracking 2nd-order PI PLL.  Reads the scaled CORDIC phase
-- error (signed) and produces a signed phase-increment offset that the
-- upstream NCO adds to its nominal step, plus a lock-state indicator
-- the receiver uses to gate downstream bit outputs.
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
--                    │
--                    ▼
--                lock_iir = slow_IIR(|phase_filt|)
--                    │
--                    ▼
--                lock_iir compared to LOCK_IN_THR / LOCK_OUT_THR with
--                LOCK_HOLD_TICKS hysteresis -> lock_state
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
        LOOP_IIR_SHIFT : natural  := 4;       -- ~ 20 Hz @ 2 kS/s
        -- Slow IIR for lock detection (Tau = 2^SHIFT / fs seconds).
        LOCK_IIR_SHIFT : positive := 14;      -- ~ 8.2 s @ 2 kS/s

        -- Lock detector hysteresis thresholds on lock_iir =
        -- slow_IIR(|phase_filt|).  Locked: PSK component on phase_filt
        -- is ±177 LSB × 2^11 ÷ ~30 dB(20 Hz LPF on 645 Hz chip rate)
        -- ≈ ±11 K LSB; rectified mean ≈ 7-9 K.  Unlocked / wrapping:
        -- phase_filt ramps ~uniform over ±π × 2^11 between wraps, so
        -- rectified mean is 0.5-2 M LSB.  Thresholds 2^14/2^16 sit in
        -- the gap with a 4× hysteresis margin.
        LOCK_IN_THR    : natural  := 2**14;
        LOCK_OUT_THR   : natural  := 2**16;
        -- Sustained-state requirement.  Pick LOCK_HOLD_TICKS ≥ 3 *
        -- 2^LOCK_IIR_SHIFT so the slow IIR has actually converged
        -- before a transition.
        LOCK_HOLD_TICKS: positive := 49152
    );
    port (
        clk              : in  std_logic;
        rst              : in  std_logic;

        phase_err        : in  signed(PHASE_ERR_WIDTH - 1 downto 0);
        phase_err_valid  : in  std_logic;

        phase_inc_offset : out signed(OFFSET_WIDTH - 1 downto 0);
        lock_state       : out std_logic;

        -- Diagnostic outputs.
        phase_filt_out   : out signed(PHASE_ERR_WIDTH - 1 downto 0);
        lock_iir_out     : out signed(PHASE_ERR_WIDTH - 1 downto 0)
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

    type reg_type is record
        phase_filt : signed(PE_W - 1 downto 0);
        lock_iir   : signed(PE_W - 1 downto 0);
        i_state    : signed(ISTATE_W - 1 downto 0);
        offset     : signed(OFFSET_WIDTH - 1 downto 0);
        lock_state : std_logic;
        lock_dur   : natural range 0 to LOCK_HOLD_TICKS;
    end record;

    constant ZERO_IS : signed(ISTATE_W - 1 downto 0)     := (others => '0');
    constant ZERO_OFF: signed(OFFSET_WIDTH - 1 downto 0) := (others => '0');
    constant ZERO_PE : signed(PE_W - 1 downto 0)         := (others => '0');

    -- Seed the slow lock IIR at LOCK_OUT_THR so a cold start doesn't
    -- satisfy `in_band` immediately and false-lock.
    function lock_iir_reset return signed is
        variable v : signed(PE_W - 1 downto 0);
    begin
        v := to_signed(LOCK_OUT_THR, PE_W);
        return v;
    end function;

    constant R_RESET : reg_type := (
        phase_filt => ZERO_PE,
        lock_iir   => lock_iir_reset,
        i_state    => ZERO_IS,
        offset     => ZERO_OFF,
        lock_state => '0',
        lock_dur   => 0
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

    function abs_signed(x : signed) return signed is
    begin
        if x(x'high) = '1' then
            return -x;
        else
            return x;
        end if;
    end function;

begin

    phase_inc_offset <= r.offset;
    lock_state       <= r.lock_state;
    phase_filt_out   <= r.phase_filt;
    lock_iir_out     <= r.lock_iir;

    comb : process (all)
        variable v          : reg_type;

        -- LPF scratch (one extra bit for the diff).
        variable loop_diff  : signed(PE_W downto 0);
        variable loop_step  : signed(PE_W downto 0);
        variable loop_new   : signed(PE_W downto 0);

        variable lock_diff  : signed(PE_W downto 0);
        variable lock_step  : signed(PE_W downto 0);
        variable lock_new   : signed(PE_W downto 0);
        variable pf_abs     : signed(PE_W - 1 downto 0);

        -- Integral path.
        variable i_step     : signed(PE_W - 1 downto 0);
        variable i_sum      : signed(ISTATE_W downto 0);
        variable i_part     : signed(SUM_W - 1 downto 0);

        -- Proportional path + offset sum.
        variable p_path     : signed(PE_W - 1 downto 0);
        variable sum_ext    : signed(SUM_W - 1 downto 0);

        -- Lock detector scratch.
        variable in_band    : boolean;
        variable out_band   : boolean;
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

            -- ===== Slow lock IIR =======================================
            -- Track the long-time mean of |phase_filt|.  Locked: PSK
            -- swings cancel after rectification + IIR → small value
            -- (~150 LSB on dcf_strong).  Unlocked: phase_filt tracks
            -- the residual Δf beat and the rectified mean grows to
            -- hundreds-to-thousands of LSB.
            pf_abs    := abs_signed(v.phase_filt);
            lock_diff := resize(pf_abs, PE_W + 1)
                       - resize(r.lock_iir, PE_W + 1);
            lock_step := shift_right(lock_diff, LOCK_IIR_SHIFT);
            lock_new  := resize(r.lock_iir, PE_W + 1) + lock_step;
            v.lock_iir := resize(lock_new, PE_W);

            -- ===== Lock-state hysteresis ===============================
            in_band    := v.lock_iir <  to_signed(LOCK_IN_THR,  PE_W);
            out_band   := v.lock_iir >= to_signed(LOCK_OUT_THR, PE_W);

            if r.lock_state = '0' then
                if in_band then
                    if r.lock_dur >= LOCK_HOLD_TICKS - 1 then
                        v.lock_state := '1';
                        v.lock_dur   := 0;
                    else
                        v.lock_dur := r.lock_dur + 1;
                    end if;
                else
                    v.lock_dur := 0;
                end if;
            else
                if out_band then
                    if r.lock_dur >= LOCK_HOLD_TICKS - 1 then
                        v.lock_state := '0';
                        v.lock_dur   := 0;
                    else
                        v.lock_dur := r.lock_dur + 1;
                    end if;
                else
                    v.lock_dur := 0;
                end if;
            end if;

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
