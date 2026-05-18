-- =========================================================================
-- dcf77_nco.vhd
--
-- Sign-only LO generator at ~77.5 kHz for the DCF77 receiver.  A single
-- PHASE_WIDTH-bit accumulator advances by (PHASE_INCR + phase_inc_offset)
-- on every `tick` strobe; the top two bits of the accumulator define the
-- quadrant, from which the sign of cos and sin are derived.  No ROM, no
-- DSP -- the cos/sin outputs feed directly into the conditional-negate
-- mixer.
--
-- Quadrant encoding (phase[hi .. hi-1]):
--   00 -> Q1: cos+, sin+
--   01 -> Q2: cos-, sin+
--   10 -> Q3: cos-, sin-
--   11 -> Q4: cos+, sin-
--
--   sin_sign = phase(hi)            ('1' means negative)
--   cos_sign = phase(hi) xor phase(hi-1)
--
-- A widened accumulator (default 25 b at 250 kS/s tick = ~7.45 mHz step
-- resolution) plus a signed `phase_inc_offset` port lets a carrier-tracking
-- PLL trim the LO frequency by an arbitrary signed amount around the
-- nominal PHASE_INCR.  Offset = 0 reproduces the legacy static-NCO
-- behaviour.  The full unsigned accumulator is exposed on `phase_acc_out`
-- so a downstream "second-tick" generator can count carrier rollovers.
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
        PHASE_WIDTH      : positive := 25;
        -- 2^25 * 77.5e3 / 250000 = 10_399_018.6...
        PHASE_INCR       : natural  := 10_399_018;
        -- Width of the signed phase_inc_offset input.  Sized so the PI
        -- loop's full output range (~ +-2^23 at our gains) fits without
        -- premature saturation when sign-extended into the PHASE_WIDTH+1
        -- adder.
        OFFSET_WIDTH     : positive := 24
    );
    port (
        clk              : in  std_logic;
        rst              : in  std_logic;
        tick             : in  std_logic;       -- 250 kS/s advance strobe
        phase_inc_offset : in  signed(OFFSET_WIDTH - 1 downto 0) :=
                                (others => '0');
        cos_sign         : out std_logic;       -- '1' = negative
        sin_sign         : out std_logic;
        phase_acc_out    : out unsigned(PHASE_WIDTH - 1 downto 0)
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

    cos_sign      <= r.phase(PHASE_WIDTH - 1) xor r.phase(PHASE_WIDTH - 2);
    sin_sign      <= r.phase(PHASE_WIDTH - 1);
    phase_acc_out <= r.phase;

    comb : process (all)
        variable v        : reg_type;
        variable nominal  : signed(PHASE_WIDTH + 1 downto 0);
        variable offset_s : signed(PHASE_WIDTH + 1 downto 0);
        variable step_s   : signed(PHASE_WIDTH + 1 downto 0);
    begin
        v := r;
        if tick = '1' then
            -- Cast both contributions to a wider signed before summing
            -- so a worst-case negative offset can't underflow.  The
            -- modular advance still wraps cleanly at PHASE_WIDTH bits.
            nominal  := to_signed(PHASE_INCR, PHASE_WIDTH + 2);
            offset_s := resize(phase_inc_offset, PHASE_WIDTH + 2);
            step_s   := nominal + offset_s;
            v.phase  := r.phase
                      + unsigned(step_s(PHASE_WIDTH - 1 downto 0));
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
