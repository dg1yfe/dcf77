-- =========================================================================
-- dcf77_boxcar.vhd
--
-- 1-stage CIC / running boxcar accumulator with decimation.  Sums `RATE`
-- consecutive in_valid samples then emits the sum (signed) at 1 / RATE
-- the input rate and clears the accumulator.  Equivalent to
-- `dcf77_cic_dec` with N=1, M=1, DEC_R=RATE; output width =
-- IN_WIDTH + ceil(log2(RATE)) bits.
--
-- Used twice in dcf77_core (one I, one Q) with RATE=125 to bring the
-- 22-b post-mixer signals from 250 kS/s down to 2 kS/s.  The first-null
-- of the boxcar lands at fs/RATE = 2 kHz; out-of-band rejection comes
-- from the dcf77_iir_lpf (or PLL) downstream.
--
-- For IN_WIDTH=22 and RATE=125 the accumulator needs
-- 22 + ceil(log2(125)) = 22 + 7 = 29 b.
-- =========================================================================

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use ieee.fixed_float_types.all;
use ieee.fixed_pkg.all;

entity dcf77_boxcar is
    generic (
        IN_WIDTH  : positive := 22;
        RATE      : positive := 125;
        OUT_WIDTH : positive := 29          -- IN_WIDTH + ceil(log2(RATE))
    );
    port (
        clk        : in  std_logic;
        rst        : in  std_logic;

        in_sample  : in  sfixed(IN_WIDTH - 1 downto 0);
        in_valid   : in  std_logic;

        out_sample : out signed(OUT_WIDTH - 1 downto 0);
        out_valid  : out std_logic
    );
end entity dcf77_boxcar;

architecture rtl of dcf77_boxcar is

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

    constant CNT_W : natural := maximum(1, clog2(RATE));

    type reg_type is record
        acc       : signed(OUT_WIDTH - 1 downto 0);
        cnt       : unsigned(CNT_W - 1 downto 0);
        out_reg   : signed(OUT_WIDTH - 1 downto 0);
        out_valid : std_logic;
    end record;

    constant ZERO_ACC : signed(OUT_WIDTH - 1 downto 0) := (others => '0');
    constant ZERO_CNT : unsigned(CNT_W - 1 downto 0)   := (others => '0');

    constant R_RESET : reg_type := (
        acc       => ZERO_ACC,
        cnt       => ZERO_CNT,
        out_reg   => ZERO_ACC,
        out_valid => '0'
    );

    signal r, rin : reg_type;

begin

    out_sample <= r.out_reg;
    out_valid  <= r.out_valid;

    comb : process (all)
        variable v          : reg_type;
        variable in_signed  : signed(IN_WIDTH - 1 downto 0);
        variable sum        : signed(OUT_WIDTH - 1 downto 0);
    begin
        v := r;
        v.out_valid := '0';

        if in_valid = '1' then
            in_signed := signed(to_slv(in_sample));
            sum := r.acc + resize(in_signed, OUT_WIDTH);

            if r.cnt = to_unsigned(RATE - 1, CNT_W) then
                v.out_reg   := sum;
                v.out_valid := '1';
                v.acc       := ZERO_ACC;
                v.cnt       := ZERO_CNT;
            else
                v.acc := sum;
                v.cnt := r.cnt + 1;
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
