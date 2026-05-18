-- =========================================================================
-- dcf77_carrier_pll.vhd
--
-- Carrier-tracking 2nd-order PI PLL.  Operates on the boxcar-decimated Q
-- channel post-mixer (1 kS/s, signed); produces a signed phase-increment
-- offset that the upstream NCO adds to its nominal step, plus a lock-state
-- indicator the receiver uses to gate downstream bit outputs.
--
-- Phase detector
-- --------------
-- Under small-angle approximation `sin(phi) ~= phi`, the Q sample after
-- the mixer carries the residual carrier-phase error scaled by the I-side
-- amplitude: `q ~= A * phi`.  Driving q -> 0 locks the LO to the carrier.
--
-- Loop filter (shift-only, no multipliers)
-- ----------------------------------------
-- Sample rate fs = 1 kS/s, target bandwidth ~0.77 Hz, damping ~0.71:
--   Kp = 1/128 - 1/1024 = 7/1024  ~= 0.00684
--   Ki = 1/65536 + 1/131072 = 3/131072  ~= 0.0000229   (~ 1/50_700)
--
--   P     = (q >> 7) - (q >> 10)         -- proportional path
--   i_step= (q >> 16) + (q >> 17)        -- integral increment
--   i_state += i_step                    -- bounded by saturation
--   phase_inc_offset = sat( P + i_state )
--
-- Equivalent continuous-time natural frequency f_n = sqrt(Ki*fs^2)/(2pi)
-- ~= 0.76 Hz; well below the boxcar's first null at 1 kHz and well above
-- DCF77's bit-rate frequencies.
--
-- Lock detector
-- -------------
-- |q| is fed into a 1st-order IIR (right-shift K_LOCK_SHIFT) and compared
-- against two hysteretic thresholds.  Below LOCK_IN_THR for at least
-- LOCK_HOLD_TICKS consecutive q_valid pulses -> lock_state asserts.
-- Above LOCK_OUT_THR for the same duration -> de-asserts.  Thresholds
-- are absolute (q is in iq_word_t * RATE scale = ~2^29 range), pick them
-- as a fraction of the expected post-lock noise floor for the deployment
-- carrier amplitude.
-- =========================================================================

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity dcf77_carrier_pll is
    generic (
        Q_WIDTH        : positive := 30;     -- signed Q post-boxcar
        OFFSET_WIDTH   : positive := 24;     -- signed NCO phase_inc_offset

        -- Lock detector parameters.
        K_LOCK_SHIFT   : positive := 6;      -- |q| IIR shift; TC = 2^6 * 1 ms = 64 ms
        LOCK_IN_THR    : natural  := 2**24;  -- |q|_iir below this -> lock-eligible
        LOCK_OUT_THR   : natural  := 2**26;  -- |q|_iir above this -> unlock-eligible
        LOCK_HOLD_TICKS: positive := 256     -- consecutive in/out-of-band ticks
    );
    port (
        clk              : in  std_logic;
        rst              : in  std_logic;

        q_sample         : in  signed(Q_WIDTH - 1 downto 0);
        q_valid          : in  std_logic;

        phase_inc_offset : out signed(OFFSET_WIDTH - 1 downto 0);
        lock_state       : out std_logic;

        -- Diagnostic: filtered |q|, expose top bits for AXI status if
        -- useful.  Drive `open` when not needed.
        q_iir_out        : out unsigned(Q_WIDTH - 1 downto 0)
    );
end entity dcf77_carrier_pll;

architecture rtl of dcf77_carrier_pll is

    constant ISTATE_W : positive := OFFSET_WIDTH + 4;     -- headroom over OFFSET_WIDTH

    type reg_type is record
        i_state      : signed(ISTATE_W - 1 downto 0);
        offset       : signed(OFFSET_WIDTH - 1 downto 0);
        q_abs_iir    : unsigned(Q_WIDTH - 1 downto 0);
        lock_state   : std_logic;
        lock_dur     : natural range 0 to LOCK_HOLD_TICKS;
    end record;

    constant ZERO_IS  : signed(ISTATE_W - 1 downto 0)   := (others => '0');
    constant ZERO_OFF : signed(OFFSET_WIDTH - 1 downto 0):= (others => '0');
    constant ZERO_QU  : unsigned(Q_WIDTH - 1 downto 0)  := (others => '0');

    constant R_RESET : reg_type := (
        i_state    => ZERO_IS,
        offset     => ZERO_OFF,
        q_abs_iir  => ZERO_QU,
        lock_state => '0',
        lock_dur   => 0
    );

    signal r, rin : reg_type;

    -- Saturate an arbitrary signed `x` into OFFSET_WIDTH bits.
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

begin

    phase_inc_offset <= r.offset;
    lock_state       <= r.lock_state;
    q_iir_out        <= r.q_abs_iir;

    comb : process (all)
        variable v        : reg_type;
        variable p_path   : signed(Q_WIDTH - 1 downto 0);
        variable i_step   : signed(Q_WIDTH - 1 downto 0);
        variable q7       : signed(Q_WIDTH - 1 downto 0);
        variable q10      : signed(Q_WIDTH - 1 downto 0);
        variable q16      : signed(Q_WIDTH - 1 downto 0);
        variable q17      : signed(Q_WIDTH - 1 downto 0);
        variable sum_ext  : signed(ISTATE_W - 1 downto 0);
        variable q_abs    : unsigned(Q_WIDTH - 1 downto 0);
        variable abs_delta: signed(Q_WIDTH downto 0);
        variable iir_step : signed(Q_WIDTH downto 0);
        variable new_iir  : signed(Q_WIDTH downto 0);
        variable in_band  : boolean;
        variable out_band : boolean;
    begin
        v := r;

        if q_valid = '1' then
            -- ----- Loop filter (PI), shift-only gains -----------------
            q7  := shift_right(q_sample, 7);
            q10 := shift_right(q_sample, 10);
            q16 := shift_right(q_sample, 16);
            q17 := shift_right(q_sample, 17);

            p_path := q7  - q10;
            i_step := q16 + q17;

            -- Integral accumulator with headroom; saturate when
            -- collapsing to OFFSET_WIDTH.
            v.i_state := r.i_state + resize(i_step, ISTATE_W);
            sum_ext   := v.i_state + resize(p_path, ISTATE_W);
            v.offset  := sat_offset(sum_ext);

            -- ----- |q| IIR ------------------------------------------
            if q_sample(Q_WIDTH - 1) = '1' then
                q_abs := unsigned(-q_sample);
            else
                q_abs := unsigned(q_sample);
            end if;

            -- new_iir = iir + (|q| - iir) >> K_LOCK_SHIFT
            abs_delta := signed('0' & std_logic_vector(q_abs))
                       - signed('0' & std_logic_vector(r.q_abs_iir));
            iir_step  := shift_right(abs_delta, K_LOCK_SHIFT);
            new_iir   := signed('0' & std_logic_vector(r.q_abs_iir))
                       + iir_step;
            -- new_iir is non-negative under correct shifting; clamp.
            if new_iir(new_iir'high) = '1' then
                v.q_abs_iir := (others => '0');
            else
                v.q_abs_iir := unsigned(new_iir(Q_WIDTH - 1 downto 0));
            end if;

            -- ----- Lock-state hysteresis ----------------------------
            in_band  := v.q_abs_iir <  to_unsigned(LOCK_IN_THR,  Q_WIDTH);
            out_band := v.q_abs_iir >= to_unsigned(LOCK_OUT_THR, Q_WIDTH);

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
