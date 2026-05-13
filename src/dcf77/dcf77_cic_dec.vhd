-- =========================================================================
-- dcf77_cic_dec.vhd
--
-- Single-stage CIC decimator with N integrators, decimation factor R, and
-- comb differential delay M (= 1 only).  Integrator and comb widths are
-- chosen by the caller (typically sourced from dcf77_cic_widths_pkg) so the
-- entity can be instantiated three times in the DCF77 cascade with different
-- widths and decimation factors but a single source file.
--
-- Structure (Hogenauer 1981):
--
--   in_sample -> I_1 -> I_2 -> ... -> I_N -> dec(R) -> C_1 -> C_2 -> ... -> C_N -> out
--
-- Pipelining: each integrator is followed by a register so the integrator
-- chain has N register stages at full input rate.  The combs are computed
-- in a single combinational pass at decimation tick and registered into the
-- comb_prev delay line and the output.  All adders use wrap-around overflow
-- (CIC modular arithmetic).
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

    -- M=1 is the only differential delay supported; trapped at elaboration.
    constant M_OK : boolean := (M = 1);

    constant DEC_CNT_W : natural :=
        maximum(1, integer(ceil(log2(real(DEC_R)))));

    type integ_array_t is array (1 to N) of sfixed(INT_WIDTH - 1 downto 0);
    type comb_array_t  is array (1 to N) of sfixed(COMB_WIDTH - 1 downto 0);

    constant ZERO_INT  : sfixed(INT_WIDTH - 1 downto 0)  := (others => '0');
    constant ZERO_COMB : sfixed(COMB_WIDTH - 1 downto 0) := (others => '0');
    constant ZERO_OUT  : sfixed(OUT_WIDTH - 1 downto 0)  := (others => '0');

    type reg_type is record
        integ      : integ_array_t;
        dec_cnt    : unsigned(DEC_CNT_W - 1 downto 0);
        -- comb_prev(k) is the previous-sample input to comb k at the decimated
        -- rate.  comb_prev(1) stores the previous integ(N) snapshot; combs 2..N
        -- store the previous output of their own input chain (see comb section).
        comb_prev  : comb_array_t;
        out_sample : sfixed(OUT_WIDTH - 1 downto 0);
        out_valid  : std_logic;
    end record;

    constant R_RESET : reg_type := (
        integ      => (others => ZERO_INT),
        dec_cnt    => (others => '0'),
        comb_prev  => (others => ZERO_COMB),
        out_sample => ZERO_OUT,
        out_valid  => '0'
    );

    signal r, rin : reg_type;

begin

    assert M_OK
        report "dcf77_cic_dec: only M=1 is supported"
        severity failure;

    comb : process (all)
        variable v          : reg_type;
        variable widened_in : sfixed(INT_WIDTH - 1 downto 0);
        variable dec_tick   : std_logic;
        variable cv         : comb_array_t;
        variable cv_in      : sfixed(COMB_WIDTH - 1 downto 0);
    begin
        v := r;
        v.out_valid := '0';

        -----------------------------------------------------------------------
        -- Pipelined integrators @ in_valid.
        --
        -- Each integrator uses the REGISTERED value of the previous stage
        -- (r.integ(k-1)), so the chain forms an N-deep pipeline.  The
        -- magnitude response is unchanged; only a constant group delay of
        -- (N-1) input samples is added.
        -----------------------------------------------------------------------
        if in_valid = '1' then
            widened_in := resize(in_sample, INT_WIDTH - 1, 0,
                                 fixed_saturate, fixed_truncate);
            v.integ(1) := resize(r.integ(1) + widened_in,
                                 INT_WIDTH - 1, 0,
                                 fixed_wrap, fixed_truncate);
            for k in 2 to N loop
                v.integ(k) := resize(r.integ(k) + r.integ(k - 1),
                                     INT_WIDTH - 1, 0,
                                     fixed_wrap, fixed_truncate);
            end loop;

            -- Decimation counter: tick once every R input samples.
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
        -- Comb section.  Fires on dec_tick at the decimated rate.
        --
        -- comb_prev(1) holds the previous integ(N) snapshot (truncated to
        -- COMB_WIDTH).  comb_prev(k) for k >= 2 holds the previous value of
        -- the (k-1)'th comb chain output, i.e. cv(k-1) from the previous
        -- decimation cycle.
        -----------------------------------------------------------------------
        if dec_tick = '1' then
            cv_in := resize(v.integ(N), COMB_WIDTH - 1, 0,
                            fixed_saturate, fixed_truncate);
            cv(1) := resize(cv_in - r.comb_prev(1),
                            COMB_WIDTH - 1, 0,
                            fixed_wrap, fixed_truncate);
            for k in 2 to N loop
                cv(k) := resize(cv(k - 1) - r.comb_prev(k),
                                COMB_WIDTH - 1, 0,
                                fixed_wrap, fixed_truncate);
            end loop;
            v.comb_prev(1) := cv_in;
            for k in 2 to N loop
                v.comb_prev(k) := cv(k - 1);
            end loop;
            v.out_sample := resize(cv(N), OUT_WIDTH - 1, 0,
                                   fixed_saturate, fixed_truncate);
            v.out_valid  := '1';
        end if;

        rin <= v;
    end process;

    out_sample <= r.out_sample;
    out_valid  <= r.out_valid;

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
