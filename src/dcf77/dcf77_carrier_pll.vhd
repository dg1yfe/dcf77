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
-- The pre-loop IIR is also the noise filter for the PI loop (effective
-- 3rd-order overall).  The slow lock IIR sees the same filtered phase
-- error and decides lock based on whether the long-term mean is small.
--
-- Loop filter (shift-only, no multipliers)
-- ----------------------------------------
-- Sample rate fs = 2 kS/s (default), tracking f_n ≈ 55 mHz:
--
--   alpha = 2^-KP_SHIFT     (Kp,  proportional gain)
--   beta  = 2^-KI_SHIFT     (Ki,  integral gain)
--
--   P      = q_filt >> KP_SHIFT
--   i_step = q_filt >> KI_SHIFT
--   i_state += i_step                        (24+4-bit headroom)
--   phase_inc_offset = sat( P + i_state )    (saturate to OFFSET_WIDTH)
--
-- Acquisition vs tracking: while ever_locked='0', the loop uses the
-- wider *_ACQ shift values (faster, captures phase quickly); once the
-- loop has locked at least once the ever_locked latch holds and the
-- narrower tracking shifts dominate.
-- =========================================================================

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity dcf77_carrier_pll is
    generic (
        Q_WIDTH        : positive := 29;     -- signed Q post-boxcar
        OFFSET_WIDTH   : positive := 24;     -- signed NCO phase_inc_offset

        -- Tracking PI gains:  alpha = 2^-KP_SHIFT, beta = 2^-KI_SHIFT.
        -- Defaults at 2 kS/s give f_n ≈ 55 mHz, dominant proportional
        -- term (very overdamped).
        KP_SHIFT       : positive := 7;       -- tracking alpha = 2^-7
        KI_SHIFT       : positive := 25;      -- tracking beta  = 2^-25

        -- Acquisition PI gains (used until ever_locked latches).
        KP_SHIFT_ACQ   : positive := 5;       -- acquisition alpha = 2^-5
        KI_SHIFT_ACQ   : positive := 14;      -- acquisition beta  = 2^-14

        -- Pre-loop IIR low-pass (cutoff ≈ fs / (2π · 2^SHIFT)).
        LOOP_IIR_SHIFT : positive := 4;       -- ~ 20 Hz @ 2 kS/s
        -- Slow IIR for lock detection (Tau = 2^SHIFT / fs seconds).
        LOCK_IIR_SHIFT : positive := 14;      -- ~ 8.2 s @ 2 kS/s

        -- Lock detector hysteresis thresholds on |q_lock_iir|.
        LOCK_IN_THR    : natural  := 2**18;
        LOCK_OUT_THR   : natural  := 2**21;
        LOCK_HOLD_TICKS: positive := 256
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

    constant ISTATE_W : positive := OFFSET_WIDTH + 4;   -- headroom over OFFSET_WIDTH

    type reg_type is record
        q_loop_iir   : signed(Q_WIDTH - 1 downto 0);
        q_lock_iir   : signed(Q_WIDTH - 1 downto 0);
        i_state      : signed(ISTATE_W - 1 downto 0);
        offset       : signed(OFFSET_WIDTH - 1 downto 0);
        lock_state   : std_logic;
        ever_locked  : std_logic;
        lock_dur     : natural range 0 to LOCK_HOLD_TICKS;
    end record;

    constant ZERO_IS : signed(ISTATE_W - 1 downto 0)     := (others => '0');
    constant ZERO_OFF: signed(OFFSET_WIDTH - 1 downto 0) := (others => '0');
    constant ZERO_Q  : signed(Q_WIDTH - 1 downto 0)      := (others => '0');

    -- Seed the slow lock IIR at LOCK_OUT_THR so a cold start doesn't
    -- satisfy `in_band` immediately and false-lock.  Real signal pushes
    -- the IIR off this seed within one TC.
    function lock_iir_reset return signed is
        variable v : signed(Q_WIDTH - 1 downto 0);
    begin
        v := to_signed(LOCK_OUT_THR, Q_WIDTH);
        return v;
    end function;

    constant R_RESET : reg_type := (
        q_loop_iir  => ZERO_Q,
        q_lock_iir  => lock_iir_reset,
        i_state     => ZERO_IS,
        offset      => ZERO_OFF,
        lock_state  => '0',
        ever_locked => '0',
        lock_dur    => 0
    );

    signal r, rin : reg_type;

    function sat_offset(x : signed) return signed is
        constant W   : natural := OFFSET_WIDTH;
        variable hi  : signed(W - 1 downto 0);
        variable lo  : signed(W - 1 downto 0);
        variable xs  : signed(x'length - 1 downto 0) := x;
        variable msb : std_logic;
    begin
        hi := (others => '1');
        hi(W - 1) := '0';
        lo := (others => '0');
        lo(W - 1) := '1';

        if xs'length <= W then
            return resize(xs, W);
        end if;
        msb := xs(xs'high);
        for i in W - 1 to xs'high loop
            if xs(i) /= msb then
                if msb = '1' then
                    return lo;
                else
                    return hi;
                end if;
            end if;
        end loop;
        return xs(W - 1 downto 0);
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

        -- PI loop scratch.
        variable kp_shift   : positive;
        variable ki_shift   : positive;
        variable p_path     : signed(Q_WIDTH - 1 downto 0);
        variable i_step     : signed(Q_WIDTH - 1 downto 0);
        variable sum_ext    : signed(ISTATE_W - 1 downto 0);

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

            -- ===== PI loop filter (shift-only) ==========================
            -- ever_locked latches narrow tracking gains after first lock.
            if r.ever_locked = '0' then
                kp_shift := KP_SHIFT_ACQ;
                ki_shift := KI_SHIFT_ACQ;
            else
                kp_shift := KP_SHIFT;
                ki_shift := KI_SHIFT;
            end if;
            p_path := shift_right(v.q_loop_iir, kp_shift);
            i_step := shift_right(v.q_loop_iir, ki_shift);

            v.i_state := r.i_state + resize(i_step, ISTATE_W);
            sum_ext   := v.i_state + resize(p_path, ISTATE_W);
            v.offset  := sat_offset(sum_ext);

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
                        v.lock_state  := '1';
                        v.ever_locked := '1';
                        v.lock_dur    := 0;
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
