-- =========================================================================
-- dcf77_cordic_atan2.vhd
--
-- Time-shared vectoring CORDIC.  Converts a complex sample (I, Q) into
-- magnitude (= K · √(I²+Q²), K = CORDIC gain ≈ 1.6467) and phase
-- (= atan2(Q, I)).
--
-- Used by dcf77_core to replace:
--   * the α-max+β-min envelope approximation (uses `mag` output)
--   * the sign(I)·q_box phase detector for the carrier PLL (uses `phase`)
--   * the new PM correlator's phase input (also uses `phase`)
--
-- Algorithm: standard vectoring CORDIC, 10 iterations.
--
--   1. Quadrant pre-fold rotates (I, Q) into the right half-plane
--      (|angle| ≤ π/2) and seeds the angle accumulator z with
--      ±π/2 as needed so the final z = atan2(Q_orig, I_orig).
--
--   2. Per-iteration update (i = 0..9):
--         if y ≥ 0:
--           x' = x + (y >> i)
--           y' = y - (x >> i)
--           z' = z + atan(2^-i)
--         else:
--           x' = x - (y >> i)
--           y' = y + (x >> i)
--           z' = z - atan(2^-i)
--      drives y → 0, leaving x → K·magnitude and z → angle.
--
-- Phase output scaling: the full circle 2π = 2^PHASE_WIDTH (so
-- 12-b phase → 1 LSB ≈ 0.088° ≈ 1.534 mrad).  Angular resolution
-- after 10 iterations is ~0.1°, plenty for the matched-filter consumer.
--
-- Magnitude output: K · √(I²+Q²) at IQ_WIDTH+1 bits (= 19 b for
-- 18-b inputs).  Includes the CORDIC gain K ≈ 1.6467; downstream
-- consumers absorb that as a constant scaling.
--
-- Latency: ITERATIONS + 2 clock cycles from in_valid to out_valid.
-- At 125 MHz this is ~96 ns — far below the 500 µs sample period
-- of the 2 kS/s consumers, so single-instance time-sharing covers
-- all three downstream consumers with ample margin.
-- =========================================================================

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity dcf77_cordic_atan2 is
    generic (
        IQ_WIDTH    : positive := 18;   -- signed I/Q input width
        PHASE_WIDTH : positive := 12;   -- signed phase output width
                                         -- (full circle 2π = 2^PHASE_WIDTH)
        ITERATIONS  : positive := 10    -- CORDIC iteration count
    );
    port (
        clk      : in  std_logic;
        rst      : in  std_logic;

        in_i     : in  signed(IQ_WIDTH - 1 downto 0);
        in_q     : in  signed(IQ_WIDTH - 1 downto 0);
        in_valid : in  std_logic;       -- 1-cycle pulse to start a conversion

        out_mag   : out unsigned(IQ_WIDTH downto 0);     -- K·|I+jQ|
        out_phase : out signed(PHASE_WIDTH - 1 downto 0); -- atan2(Q, I)
        out_valid : out std_logic       -- 1-cycle pulse when out_mag/phase ready
    );
end entity dcf77_cordic_atan2;

architecture rtl of dcf77_cordic_atan2 is

    -- Internal X/Y width: +2 b over input for CORDIC gain headroom.
    -- Peak |I+jQ| = √2·2^(IQ_WIDTH-1); after K ≈ 1.6467 it becomes
    -- K·√2·2^(IQ_WIDTH-1) ≈ 2^(IQ_WIDTH+1.2), which fits in
    -- IQ_WIDTH+2 signed bits with ~0.8 b of headroom.
    constant INTERN_W : positive := IQ_WIDTH + 2;

    -- π/2 in our phase units (where 2π = 2^PHASE_WIDTH).
    constant HALF_PI : signed(PHASE_WIDTH - 1 downto 0) :=
        to_signed(2 ** (PHASE_WIDTH - 2), PHASE_WIDTH);

    -- atan(2^-i) LUT in PHASE_WIDTH-bit signed (where 2π = 2^PHASE_WIDTH).
    -- Pre-computed for PHASE_WIDTH = 12: 1 rad = 4096/(2π) ≈ 651.9 LSB.
    --   atan(1)      = 0.7854 rad → 512  LSB (= 45°)
    --   atan(1/2)    = 0.4636 rad → 302
    --   atan(1/4)    = 0.2450 rad → 160
    --   atan(1/8)    = 0.1244 rad →  81
    --   atan(1/16)   = 0.0624 rad →  41
    --   atan(1/32)   = 0.0312 rad →  20
    --   atan(1/64)   = 0.0156 rad →  10
    --   atan(1/128)  = 0.0078 rad →   5
    --   atan(1/256)  = 0.0039 rad →   3
    --   atan(1/512)  = 0.0020 rad →   1
    type atan_lut_t is array (0 to ITERATIONS - 1) of
        signed(PHASE_WIDTH - 1 downto 0);
    constant ATAN_LUT : atan_lut_t := (
        to_signed(512, PHASE_WIDTH),
        to_signed(302, PHASE_WIDTH),
        to_signed(160, PHASE_WIDTH),
        to_signed( 81, PHASE_WIDTH),
        to_signed( 41, PHASE_WIDTH),
        to_signed( 20, PHASE_WIDTH),
        to_signed( 10, PHASE_WIDTH),
        to_signed(  5, PHASE_WIDTH),
        to_signed(  3, PHASE_WIDTH),
        to_signed(  1, PHASE_WIDTH)
    );

    type state_t is (S_IDLE, S_ITER, S_DONE);

    type reg_type is record
        state     : state_t;
        iter      : natural range 0 to ITERATIONS;
        x         : signed(INTERN_W - 1 downto 0);
        y         : signed(INTERN_W - 1 downto 0);
        z         : signed(PHASE_WIDTH - 1 downto 0);
        out_mag   : unsigned(IQ_WIDTH downto 0);
        out_phase : signed(PHASE_WIDTH - 1 downto 0);
        out_valid : std_logic;
    end record;

    constant R_RESET : reg_type := (
        state     => S_IDLE,
        iter      => 0,
        x         => (others => '0'),
        y         => (others => '0'),
        z         => (others => '0'),
        out_mag   => (others => '0'),
        out_phase => (others => '0'),
        out_valid => '0'
    );

    signal r, rin : reg_type;

begin

    out_mag   <= r.out_mag;
    out_phase <= r.out_phase;
    out_valid <= r.out_valid;

    comb : process (all)
        variable v         : reg_type;
        variable shifted_x : signed(INTERN_W - 1 downto 0);
        variable shifted_y : signed(INTERN_W - 1 downto 0);
        variable y_nonneg  : boolean;
    begin
        v := r;
        v.out_valid := '0';

        case r.state is
            when S_IDLE =>
                if in_valid = '1' then
                    -- Quadrant pre-fold: bring (I, Q) into right half-plane.
                    if in_i(IQ_WIDTH - 1) = '0' then
                        -- I ≥ 0: no fold, angle is in (-π/2, π/2].
                        v.x := resize(in_i, INTERN_W);
                        v.y := resize(in_q, INTERN_W);
                        v.z := (others => '0');
                    elsif in_q(IQ_WIDTH - 1) = '0' then
                        -- I < 0, Q ≥ 0: angle in (π/2, π]; rotate by -π/2
                        -- → (x', y') = (Q, -I), seed z = +π/2.
                        v.x :=  resize(in_q, INTERN_W);
                        v.y := -resize(in_i, INTERN_W);
                        v.z := HALF_PI;
                    else
                        -- I < 0, Q < 0: angle in [-π, -π/2); rotate by +π/2
                        -- → (x', y') = (-Q, I), seed z = -π/2.
                        v.x := -resize(in_q, INTERN_W);
                        v.y :=  resize(in_i, INTERN_W);
                        v.z := -HALF_PI;
                    end if;
                    v.iter  := 0;
                    v.state := S_ITER;
                end if;

            when S_ITER =>
                -- Arithmetic right-shift by r.iter (synthesises as a small
                -- barrel shifter since r.iter ∈ [0, ITERATIONS-1]).
                shifted_x := shift_right(r.x, r.iter);
                shifted_y := shift_right(r.y, r.iter);

                y_nonneg := r.y(INTERN_W - 1) = '0';

                if y_nonneg then
                    v.x := r.x + shifted_y;
                    v.y := r.y - shifted_x;
                    v.z := r.z + ATAN_LUT(r.iter);
                else
                    v.x := r.x - shifted_y;
                    v.y := r.y + shifted_x;
                    v.z := r.z - ATAN_LUT(r.iter);
                end if;

                if r.iter = ITERATIONS - 1 then
                    v.state := S_DONE;
                else
                    v.iter := r.iter + 1;
                end if;

            when S_DONE =>
                -- After the final iteration the registered r.x holds
                -- K·magnitude (non-negative, since folded into the right
                -- half-plane).  Take the low IQ_WIDTH+1 bits as unsigned.
                v.out_mag   := unsigned(r.x(IQ_WIDTH downto 0));
                v.out_phase := r.z;
                v.out_valid := '1';
                v.state     := S_IDLE;

        end case;

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
