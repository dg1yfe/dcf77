-- =========================================================================
-- dcf77_nco.vhd
--
-- Sign-only LO generator at 77.5 kHz for the DCF77 receiver.  A single
-- PHASE_WIDTH-bit accumulator advances by PHASE_INCR each `tick` strobe; the
-- top two bits of the accumulator define the quadrant, from which the sign
-- of cos and sin are derived.  No ROM, no DSP -- the cos/sin outputs feed
-- directly into the conditional-negate mixer.
--
-- Quadrant encoding (phase[hi-1 .. hi-2]):
--   00 -> Q1: cos+, sin+
--   01 -> Q2: cos-, sin+
--   10 -> Q3: cos-, sin-
--   11 -> Q4: cos+, sin-
--
-- Therefore:
--   sin_sign = phase(hi)        ('1' means negative)
--   cos_sign = phase(hi) xor phase(hi-1)
--
-- The outputs are combinational from the registered phase, so for an input
-- sample arriving in the same clock as `tick = '1'`, the mixer sees the
-- pre-increment sign (i.e. the LO value at the moment of that sample).
-- =========================================================================

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity dcf77_nco is
    generic (
        PHASE_WIDTH : positive := 24;
        PHASE_INCR  : natural  := 5_326_192   -- 2^24 * 77.5e3 / 244140.625
    );
    port (
        clk      : in  std_logic;
        rst      : in  std_logic;
        tick     : in  std_logic;       -- 244 kS/s advance strobe
        cos_sign : out std_logic;       -- '1' = negative
        sin_sign : out std_logic
    );
end entity dcf77_nco;

architecture rtl of dcf77_nco is

    type reg_type is record
        phase : unsigned(PHASE_WIDTH - 1 downto 0);
    end record;

    constant R_RESET : reg_type := (
        phase => (others => '0')
    );

    signal r, rin : reg_type;

begin

    cos_sign <= r.phase(PHASE_WIDTH - 1) xor r.phase(PHASE_WIDTH - 2);
    sin_sign <= r.phase(PHASE_WIDTH - 1);

    comb : process (all)
        variable v : reg_type;
    begin
        v := r;
        if tick = '1' then
            v.phase := r.phase + to_unsigned(PHASE_INCR, PHASE_WIDTH);
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
