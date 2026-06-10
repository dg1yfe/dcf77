-- =========================================================================
-- dcf77_phase_quality.vhd
--
-- Signal-quality / lock indicator for the carrier PLL.  Operates on the
-- raw 12-b CORDIC phase rather than any PLL-internal nets so the lock
-- decision is decoupled from the PI loop's noise filter.
--
-- Metric: saturating signed up/down counter, +1 when |cordic_phase| sits
-- in the low half of the range (|p| < 2^(PHASE_WIDTH-1)) and -1 when in
-- the high half.  Pure noise gives a 50/50 split — counter random-walks
-- around 0.  Any coherent signal pulls more samples into the low bin
-- and drives the counter toward +CMAX; signal-free spectrum drives it
-- toward -CMAX.  See the corpus calibration table below for empirical
-- bin fractions.
--
-- Why a counter rather than an IIR:
--   * Input is ±1, well above any per-sample shift-truncation bias —
--     no FRAC_BITS / wide-accumulator workaround needed.
--   * Counter dynamics are linear in time, so the saturation time
--     scales as CMAX / drift.  Every locked-on signal (frac_low > 0.5)
--     eventually rails at +CMAX regardless of SNR; only the time-to-
--     saturation differs.
--   * Hysteresis is a plain integer compare.
--
-- Corpus calibration on 60 s captures, post 4 s warmup:
--
--   recording        σ(p)   |p|    frac_low   E[step] = 2·frac − 1
--   --------------   ----   ----   --------   ---------------------
--   dcf_strong       190    159    1.000       +1.000   (rails fast)
--   synth            226    176    1.000       +1.000
--   dcf_3_1          359    294    0.996       +0.992
--   dcf_3_6          617    466    0.907       +0.814
--   dcf_3_4          812    638    0.791       +0.582
--   dcf_3_3          957    778    0.685       +0.370
--   dcf_3_7         1090    920    0.578       +0.156
--   dcf_3_5         1146    979    0.529       +0.058
--   noise (1750 Hz) 1190   1033    0.492       -0.016   (rails neg.)
--
-- Defaults with COUNTER_W=12 (CMAX = 2^11 = 2048):
--   LOCK_IN_THR  = +1024 (≈ half-saturation) — lock when counter is
--                  unambiguously in the signal-coherence region.
--   LOCK_OUT_THR = -512  — only unlock when counter has been driven
--                  well below the noise random-walk range.
--
-- Saturation-time at frac_low = f: CMAX / (2f − 1) samples.  At fs =
-- 2 kS/s:  strong → 1.0 s, dcf_3_3 → 2.8 s, dcf_3_5 → 17.6 s.  Noise
-- rails at -CMAX over a similar time scale (|drift| ≈ 0.016 → ~64 s).
-- =========================================================================

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity dcf77_phase_quality is
    generic (
        PHASE_WIDTH    : positive := 12;        -- signed cordic_phase
        -- Signed counter width (incl. sign bit).  Rails at ±2^(W-1).
        COUNTER_W      : positive := 12;
        -- Hysteresis thresholds on the signed counter.  Asymmetric:
        -- LOCK_IN is positive (clear signal coherence), LOCK_OUT is
        -- negative (counter has been pushed well below noise's typical
        -- random-walk position around 0).
        LOCK_IN_THR    : integer := 1024;
        LOCK_OUT_THR   : integer := -512;
        -- Edge-debounce on the threshold crossing.  The counter itself
        -- carries the integration, so this is just sample-jitter
        -- rejection; 256 = 128 ms @ 2 kS/s.
        LOCK_HOLD_TICKS: positive := 256
    );
    port (
        clk             : in  std_logic;
        rst             : in  std_logic;

        cordic_phase    : in  signed(PHASE_WIDTH - 1 downto 0);
        cordic_valid    : in  std_logic;

        lock_state      : out std_logic;
        -- Diagnostic: current counter value (signed, COUNTER_W bits).
        lock_counter_out: out signed(COUNTER_W - 1 downto 0)
    );
end entity dcf77_phase_quality;

architecture rtl of dcf77_phase_quality is

    -- Positive saturation = 2^(COUNTER_W-1) - 1; negative = -2^(COUNTER_W-1).
    constant CMAX_POS : integer := 2 ** (COUNTER_W - 1) - 1;
    constant CMAX_NEG : integer := -(2 ** (COUNTER_W - 1));

    type reg_type is record
        counter    : signed(COUNTER_W - 1 downto 0);
        lock_state : std_logic;
        lock_dur   : natural range 0 to LOCK_HOLD_TICKS;
    end record;

    constant R_RESET : reg_type := (
        counter    => to_signed(0, COUNTER_W),
        lock_state => '0',
        lock_dur   => 0
    );

    signal r, rin : reg_type;

begin

    lock_state       <= r.lock_state;
    lock_counter_out <= r.counter;

    comb : process (all)
        variable v          : reg_type;
        -- in_low_bin: |cordic_phase| < 2^(PHASE_WIDTH-1).  Equivalent
        -- to "the two MSBs of the signed phase are equal" — single XOR.
        --   phase = +0..+(2^(PW-1)-1):  bits[PW-1]=0, bits[PW-2]=0  → eq
        --   phase = +2^(PW-1)..+(2^PW-1)-1: bits = 0,1              → ne
        --   phase = -1..-(2^(PW-1)+1):  bits = 1,1                  → eq
        --   phase = -2^(PW-1)..-2^PW:   bits = 1,0                  → ne
        variable in_low_bin : boolean;
        variable in_band    : boolean;
        variable out_band   : boolean;
    begin
        v := r;

        if cordic_valid = '1' then

            in_low_bin := cordic_phase(PHASE_WIDTH - 1) =
                          cordic_phase(PHASE_WIDTH - 2);

            -- Saturating ±1 step.
            if in_low_bin then
                if r.counter < to_signed(CMAX_POS, COUNTER_W) then
                    v.counter := r.counter + 1;
                end if;
            else
                if r.counter > to_signed(CMAX_NEG, COUNTER_W) then
                    v.counter := r.counter - 1;
                end if;
            end if;

            in_band  := v.counter >= to_signed(LOCK_IN_THR,  COUNTER_W);
            out_band := v.counter <= to_signed(LOCK_OUT_THR, COUNTER_W);

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

        if rst = '1' then
            v := R_RESET;
        end if;

        rin <= v;
    end process;

    seq : process (clk)
    begin
        if rising_edge(clk) then
            r <= rin;
        end if;
    end process;

end architecture rtl;
