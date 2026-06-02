-- =========================================================================
-- dcf77_carrier_pll.vhd
--
-- Carrier-tracking 2nd-order PI PLL.  Reads the post-boxcar Q sample
-- (signed) and produces a signed phase-increment offset that the
-- upstream NCO adds to its nominal step, plus a lock-state indicator
-- the receiver uses to gate downstream bit outputs.
--
-- Phase detector
-- --------------
-- Under small-angle approximation `sin(phi) ~= phi`, the Q sample after
-- the mixer carries the residual carrier-phase error scaled by the
-- I-side amplitude: `q ~= A * phi`.  Driving q -> 0 locks the LO to
-- the carrier.
--
-- Signal chain inside the PLL
-- ---------------------------
--   q_sample ─► loop_iir (1st-order IIR, SHIFT=LOOP_IIR_SHIFT, ~20 Hz)
--                    │
--                    ▼  q_filt
--                  PI loop filter  ─►  phase_inc_offset
--                    │
--                    ▼
--                lock_iir (1st-order IIR, SHIFT=LOCK_IIR_SHIFT, ~8 s)
--                    │
--                    ▼  q_lock
--                |q_lock| compared to LOCK_IN_THR / LOCK_OUT_THR with
--                LOCK_HOLD_TICKS hysteresis -> lock_state
--
-- Loop filter (shift-only, no multipliers)
-- ----------------------------------------
--   alpha = 2^-KP_SHIFT     (Kp,  proportional gain, default 2^-7)
--   beta  = 2^-KI_SHIFT     (Ki,  integral gain,    default 2^-25)
--
-- The integral path needs full-precision accumulation: a per-sample
-- `q_filt >> KI_SHIFT` would arithmetic-shift small samples to 0/-1
-- asymmetrically, biasing the accumulator by -0.5 LSB/sample = ~1000
-- LSB/s @ 2 kS/s.  Instead the per-sample shift is only KI_SHIFT -
-- I_FRAC_BITS (=15 by default), and the resulting i_state carries
-- I_FRAC_BITS extra fractional bits below the offset LSB; those bits
-- absorb the per-sample rounding so the residual bias scales as
-- ~fs / 2^(I_FRAC_BITS+1) LSB/s (-1 LSB/s at default).
--
--   i_step    = q_filt >> (KI_SHIFT - I_FRAC_BITS)
--   i_state  += i_step                          (ISTATE_W b, saturated)
--   p_path    = q_filt >> KP_SHIFT
--   phase_inc_offset = sat( (i_state >> I_FRAC_BITS) + p_path )
-- =========================================================================

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity dcf77_carrier_pll is
    generic (
        Q_WIDTH        : positive := 29;     -- signed Q post-boxcar
        OFFSET_WIDTH   : positive := 24;     -- signed NCO phase_inc_offset

        -- PI gains:  alpha = 2^-KP_SHIFT, beta = 2^-KI_SHIFT.
        -- Defaults at 2 kS/s give f_n ≈ 55 mHz, dominant proportional
        -- term (very overdamped) -- a narrow tracking loop.
        KP_SHIFT       : positive := 7;
        KI_SHIFT       : positive := 20;

        -- Fractional bits carried in i_state below the offset LSB
        -- (per-sample integrator shift is KI_SHIFT - I_FRAC_BITS).
        -- Larger values reduce the per-sample rounding bias at the
        -- cost of a wider accumulator; residual bias scales as
        -- ~fs / 2^(I_FRAC_BITS+1) LSB/s.  Default 10 -> -1 LSB/s @ 2 kS/s.
        I_FRAC_BITS    : positive := 10;

        -- Pre-loop IIR low-pass (cutoff ≈ fs / (2π · 2^SHIFT)).  SHIFT=0
        -- bypasses the filter (q_loop_iir tracks q_sample instantaneously).
        LOOP_IIR_SHIFT : natural  := 4;       -- ~ 20 Hz @ 2 kS/s
        -- Slow IIR for lock detection (Tau = 2^SHIFT / fs seconds).
        LOCK_IIR_SHIFT : positive := 14;      -- ~ 8.2 s @ 2 kS/s

        -- Lock detector hysteresis thresholds on |q_lock_iir|.
        LOCK_IN_THR    : natural  := 2**18;
        LOCK_OUT_THR   : natural  := 2**21;
        -- Sustained-state requirement.  Pick LOCK_HOLD_TICKS ≥ 3 *
        -- 2^LOCK_IIR_SHIFT so the slow IIR (Tau = 2^LOCK_IIR_SHIFT
        -- samples) has actually converged before a transition.
        LOCK_HOLD_TICKS: positive := 49152
    );
    port (
        clk              : in  std_logic;
        rst              : in  std_logic;

        q_sample         : in  signed(Q_WIDTH - 1 downto 0);
        q_valid          : in  std_logic;

        phase_inc_offset : out signed(OFFSET_WIDTH - 1 downto 0);
        lock_state       : out std_logic;

        -- Diagnostic outputs.
        q_filt_out       : out signed(Q_WIDTH - 1 downto 0);
        q_lock_out       : out signed(Q_WIDTH - 1 downto 0)
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

    type reg_type is record
        q_loop_iir : signed(Q_WIDTH - 1 downto 0);
        q_lock_iir : signed(Q_WIDTH - 1 downto 0);
        i_state    : signed(ISTATE_W - 1 downto 0);
        offset     : signed(OFFSET_WIDTH - 1 downto 0);
        lock_state : std_logic;
        lock_dur   : natural range 0 to LOCK_HOLD_TICKS;
    end record;

    constant ZERO_IS : signed(ISTATE_W - 1 downto 0)     := (others => '0');
    constant ZERO_OFF: signed(OFFSET_WIDTH - 1 downto 0) := (others => '0');
    constant ZERO_Q  : signed(Q_WIDTH - 1 downto 0)      := (others => '0');

    -- Seed the slow lock IIR at LOCK_OUT_THR so a cold start doesn't
    -- satisfy `in_band` immediately and false-lock.
    function lock_iir_reset return signed is
        variable v : signed(Q_WIDTH - 1 downto 0);
    begin
        v := to_signed(LOCK_OUT_THR, Q_WIDTH);
        return v;
    end function;

    constant R_RESET : reg_type := (
        q_loop_iir => ZERO_Q,
        q_lock_iir => lock_iir_reset,
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

    function abs_signed(x : signed) return unsigned is
    begin
        if x(x'high) = '1' then
            return unsigned(-x);
        else
            return unsigned(x);
        end if;
    end function;

begin

    phase_inc_offset <= r.offset;
    lock_state       <= r.lock_state;
    q_filt_out       <= r.q_loop_iir;
    q_lock_out       <= r.q_lock_iir;

    comb : process (all)
        variable v          : reg_type;

        -- IIR scratch (one extra bit for the diff).
        variable loop_diff  : signed(Q_WIDTH downto 0);
        variable loop_step  : signed(Q_WIDTH downto 0);
        variable loop_new   : signed(Q_WIDTH downto 0);

        variable lock_diff  : signed(Q_WIDTH downto 0);
        variable lock_step  : signed(Q_WIDTH downto 0);
        variable lock_new   : signed(Q_WIDTH downto 0);

        -- Integral path.
        variable i_step     : signed(Q_WIDTH - 1 downto 0);
        variable i_sum      : signed(ISTATE_W downto 0);     -- +1 for overflow
        variable i_part     : signed(SUM_W - 1 downto 0);

        -- Proportional path + offset sum.
        variable p_path     : signed(Q_WIDTH - 1 downto 0);
        variable sum_ext    : signed(SUM_W - 1 downto 0);

        -- Lock detector scratch.
        variable q_lock_abs : unsigned(Q_WIDTH - 1 downto 0);
        variable in_band    : boolean;
        variable out_band   : boolean;
    begin
        v := r;

        if q_valid = '1' then

            -- ===== Pre-loop IIR (20 Hz LPF on q_sample) =================
            loop_diff := resize(q_sample, Q_WIDTH + 1)
                       - resize(r.q_loop_iir, Q_WIDTH + 1);
            loop_step := shift_right(loop_diff, LOOP_IIR_SHIFT);
            loop_new  := resize(r.q_loop_iir, Q_WIDTH + 1) + loop_step;
            v.q_loop_iir := resize(loop_new, Q_WIDTH);

            -- ===== Integral path (wide accumulator) =====================
            -- Per-sample shift is only K_PRE bits; the remaining
            -- I_FRAC_BITS of right-shift happens once on the way into
            -- the offset sum.  i_state therefore carries I_FRAC_BITS
            -- below the offset LSB and absorbs the rounding asymmetry.
            i_step    := shift_right(v.q_loop_iir, K_PRE);
            i_sum     := resize(r.i_state, ISTATE_W + 1)
                       + resize(i_step,    ISTATE_W + 1);
            v.i_state := sat_to(i_sum, ISTATE_W);

            -- ===== Proportional path + offset ===========================
            p_path  := shift_right(v.q_loop_iir, KP_SHIFT);
            i_part  := resize(shift_right(v.i_state, I_FRAC_BITS), SUM_W);
            sum_ext := i_part + resize(p_path, SUM_W);
            v.offset := sat_to(sum_ext, OFFSET_WIDTH);

            -- ===== Slow lock IIR ========================================
            lock_diff := resize(v.q_loop_iir, Q_WIDTH + 1)
                       - resize(r.q_lock_iir, Q_WIDTH + 1);
            lock_step := shift_right(lock_diff, LOCK_IIR_SHIFT);
            lock_new  := resize(r.q_lock_iir, Q_WIDTH + 1) + lock_step;
            v.q_lock_iir := resize(lock_new, Q_WIDTH);

            -- ===== Lock-state hysteresis ===============================
            q_lock_abs := abs_signed(v.q_lock_iir);
            in_band    := q_lock_abs <  to_unsigned(LOCK_IN_THR,  Q_WIDTH);
            out_band   := q_lock_abs >= to_unsigned(LOCK_OUT_THR, Q_WIDTH);

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
