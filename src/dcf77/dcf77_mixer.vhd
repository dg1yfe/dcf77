-- =========================================================================
-- dcf77_mixer.vhd
--
-- Sign-only I/Q mixer: conditional two's-complement negation of the input
-- sample driven by the cos_sign / sin_sign outputs of the NCO.  No
-- multipliers; the harmonics of the square LO fall outside the narrow
-- post-CIC passband at this point in the chain.
--
-- Both I and Q outputs share the same `valid` strobe (= input `valid`),
-- with one cycle of registered delay through this entity.
-- =========================================================================

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use ieee.fixed_float_types.all;
use ieee.fixed_pkg.all;

entity dcf77_mixer is
    generic (
        DATA_WIDTH : positive := 22
    );
    port (
        clk      : in  std_logic;
        rst      : in  std_logic;

        in_sample : in  sfixed(DATA_WIDTH - 1 downto 0);
        in_valid  : in  std_logic;
        cos_sign  : in  std_logic;       -- '1' = negate -> -input
        sin_sign  : in  std_logic;

        i_out     : out sfixed(DATA_WIDTH - 1 downto 0);
        q_out     : out sfixed(DATA_WIDTH - 1 downto 0);
        out_valid : out std_logic
    );
end entity dcf77_mixer;

architecture rtl of dcf77_mixer is

    type reg_type is record
        i_out     : sfixed(DATA_WIDTH - 1 downto 0);
        q_out     : sfixed(DATA_WIDTH - 1 downto 0);
        out_valid : std_logic;
    end record;

    constant ZERO_S : sfixed(DATA_WIDTH - 1 downto 0) := (others => '0');
    constant R_RESET : reg_type := (
        i_out     => ZERO_S,
        q_out     => ZERO_S,
        out_valid => '0'
    );

    signal r, rin : reg_type;

    function cond_negate(x : sfixed; do_negate : std_logic)
        return sfixed
    is
    begin
        if do_negate = '1' then
            -- Saturating negate avoids the -2^(W-1) wrap corner case.
            return resize(- x, x'high, x'low,
                          fixed_saturate, fixed_truncate);
        else
            return x;
        end if;
    end function;

begin

    i_out     <= r.i_out;
    q_out     <= r.q_out;
    out_valid <= r.out_valid;

    comb : process (all)
        variable v : reg_type;
    begin
        v := r;
        v.out_valid := '0';

        if in_valid = '1' then
            v.i_out     := cond_negate(in_sample, cos_sign);
            v.q_out     := cond_negate(in_sample, sin_sign);
            v.out_valid := '1';
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
