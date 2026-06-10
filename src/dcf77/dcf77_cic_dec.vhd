-- =========================================================================
-- dcf77_cic_dec.vhd
--
-- Single-stage CIC decimator with N integrators, decimation factor R, and
-- comb differential delay M (= 1 only).  Integrator and comb widths are
-- chosen by the caller (typically sourced from dcf77_cic_widths_pkg) so the
-- entity can be instantiated multiple times in the DCF77 cascade with
-- different widths and decimation factors from a single source file.
--
-- Structure (Hogenauer 1981):
--
--   in_sample -> I_1 -> I_2 -> ... -> I_N -> dec(R) -> C_1 -> C_2 -> ... -> C_N -> out
--
-- Each stage truncates **LSBs** (per the Hogenauer noise-budget analysis in
-- simulation/scripts/cic_pruning.py), anchored at the same natural-MSB
-- position throughout the chain.  This is the canonical Hogenauer-pruned
-- structure: the kept bits are the top portion of the natural-scale value
-- and the dropped LSBs are bounded-L2 truncation noise.
--
-- Internally every adder operates at the natural MSB (= IN_WIDTH +
-- N*ceil(log2(R*M)) - 1).  Integrator wrap is modular (fixed_wrap); the
-- comb section's subtraction recovers the signal modulo 2^COMB_WIDTH at
-- its (higher) LSB position.  The OUTPUT port still presents the bit
-- pattern at sfixed(OUT_WIDTH-1 downto 0) (LSB=0) so downstream
-- consumers don't need to know the internal LSB scaling; the bit pattern
-- represents the natural-scale value shifted right by (natural_msb -
-- OUT_WIDTH + 1) bits.
-- =========================================================================

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use ieee.fixed_float_types.all;
use ieee.fixed_pkg.all;
use ieee.math_real.all;

entity dcf77_cic_dec is
    generic (
        N          : positive := 3;     -- integrator / comb stages
        DEC_R      : positive;          -- decimation factor (>= 2)
        M          : positive := 1;     -- comb diff. delay (1 supported)
        IN_WIDTH   : positive;          -- input sample width (bits)
        INT_WIDTH  : positive;          -- integrator carrier width
        COMB_WIDTH : positive;          -- comb carrier width
        OUT_WIDTH  : positive           -- output sample width
    );
    port (
        clk        : in  std_logic;
        rst        : in  std_logic;
        in_sample  : in  sfixed(IN_WIDTH - 1 downto 0);
        in_valid   : in  std_logic;
        out_sample : out sfixed(OUT_WIDTH - 1 downto 0);
        out_valid  : out std_logic
    );
end entity dcf77_cic_dec;

architecture rtl of dcf77_cic_dec is

    constant M_OK : boolean := (M = 1);

    function clog2(n : positive) return natural is
        variable acc : natural := 0;
        variable v   : natural := n - 1;
    begin
        while v > 0 loop
            acc := acc + 1;
            v   := v / 2;
        end loop;
        return acc;
    end function;

    constant DEC_CNT_W : natural :=
        maximum(1, clog2(DEC_R));

    -- Natural-MSB position: the kept signal range is anchored at this
    -- bit through the entire chain.  Every stage shares the same hi
    -- index; only the LSB position varies (more LSBs dropped at deeper
    -- pruning points).
    constant LOG2_RM : natural := clog2(DEC_R * M);
    constant NAT_MSB : integer := IN_WIDTH + N * LOG2_RM - 1;

    constant INT_LSB  : integer := NAT_MSB - INT_WIDTH  + 1;
    constant COMB_LSB : integer := NAT_MSB - COMB_WIDTH + 1;
    constant OUT_LSB  : integer := NAT_MSB - OUT_WIDTH  + 1;

    subtype integ_t is sfixed(NAT_MSB downto INT_LSB);
    subtype comb_t  is sfixed(NAT_MSB downto COMB_LSB);
    subtype outl_t  is sfixed(NAT_MSB downto OUT_LSB);

    type integ_array_t is array (1 to N) of integ_t;
    type comb_array_t  is array (1 to N) of comb_t;

    constant ZERO_INT   : integ_t := (others => '0');
    constant ZERO_COMB  : comb_t  := (others => '0');
    constant ZERO_OUTL  : outl_t  := (others => '0');
    constant ZERO_OUT   : sfixed(OUT_WIDTH - 1 downto 0) := (others => '0');

    type reg_type is record
        integ      : integ_array_t;
        dec_cnt    : unsigned(DEC_CNT_W - 1 downto 0);
        -- comb_prev(1) holds the previous integ(N) snapshot at the comb
        -- LSB scale; comb_prev(k) for k >= 2 holds the previous output
        -- of comb stage k-1.
        comb_prev  : comb_array_t;
        out_local  : outl_t;
        out_valid  : std_logic;
    end record;

    constant R_RESET : reg_type := (
        integ      => (others => ZERO_INT),
        dec_cnt    => (others => '0'),
        comb_prev  => (others => ZERO_COMB),
        out_local  => ZERO_OUTL,
        out_valid  => '0'
    );

    signal r, rin : reg_type;

begin

    assert M_OK
        report "dcf77_cic_dec: only M=1 is supported"
        severity failure;

    comb : process (all)
        variable v          : reg_type;
        variable widened_in : integ_t;
        variable dec_tick   : std_logic;
        variable cv         : comb_array_t;
        variable cv_in      : comb_t;
    begin
        v := r;
        v.out_valid := '0';

        -----------------------------------------------------------------------
        -- Pipelined integrators @ in_valid.  Operate at the natural MSB
        -- (INT_LSB > 0 means we discard the input's bottom INT_LSB bits as
        -- pruning noise).  Wrap is modular per CIC theory; the comb section
        -- recovers the signal regardless of any wraps that occurred here.
        -----------------------------------------------------------------------
        if in_valid = '1' then
            widened_in := resize(in_sample, NAT_MSB, INT_LSB,
                                 fixed_wrap, fixed_truncate);
            v.integ(1) := resize(r.integ(1) + widened_in,
                                 NAT_MSB, INT_LSB,
                                 fixed_wrap, fixed_truncate);
            for k in 2 to N loop
                v.integ(k) := resize(r.integ(k) + r.integ(k - 1),
                                     NAT_MSB, INT_LSB,
                                     fixed_wrap, fixed_truncate);
            end loop;

            -- Decimation counter.
            if r.dec_cnt = to_unsigned(DEC_R - 1, DEC_CNT_W) then
                v.dec_cnt := (others => '0');
                dec_tick  := '1';
            else
                v.dec_cnt := r.dec_cnt + 1;
                dec_tick  := '0';
            end if;
        else
            dec_tick := '0';
        end if;

        -----------------------------------------------------------------------
        -- Comb section.  Each stage drops further LSBs (COMB_LSB > INT_LSB),
        -- still anchored at NAT_MSB.  Modular subtraction (fixed_wrap) so
        -- residual integrator wraps cancel correctly.
        -----------------------------------------------------------------------
        if dec_tick = '1' then
            cv_in := resize(v.integ(N), NAT_MSB, COMB_LSB,
                            fixed_wrap, fixed_truncate);
            cv(1) := resize(cv_in - r.comb_prev(1),
                            NAT_MSB, COMB_LSB,
                            fixed_wrap, fixed_truncate);
            for k in 2 to N loop
                cv(k) := resize(cv(k - 1) - r.comb_prev(k),
                                NAT_MSB, COMB_LSB,
                                fixed_wrap, fixed_truncate);
            end loop;
            v.comb_prev(1) := cv_in;
            for k in 2 to N loop
                v.comb_prev(k) := cv(k - 1);
            end loop;
            -- Final output: drop the remaining LSBs down to OUT_LSB.
            v.out_local := resize(cv(N), NAT_MSB, OUT_LSB,
                                  fixed_wrap, fixed_truncate);
            v.out_valid := '1';
        end if;

        if rst = '1' then
            v := R_RESET;
        end if;

        rin <= v;
    end process;

    -- Transfer the output bit pattern to the port-shaped sfixed (LSB=0)
    -- without altering the bits.  Downstream consumers see a plain
    -- OUT_WIDTH-bit signed number; the natural-scale LSB position
    -- (NAT_MSB - OUT_WIDTH + 1) is implicit in the chain's overall scaling.
    out_sample <= to_sfixed(to_slv(r.out_local), OUT_WIDTH - 1, 0);
    out_valid  <= r.out_valid;

    seq : process (clk)
    begin
        if rising_edge(clk) then
            r <= rin;
        end if;
    end process;

end architecture rtl;
