-- =========================================================================
-- dcf77_iir_lpf.vhd
--
-- Generic 1st-order IIR low-pass with a power-of-two coefficient:
--
--     y[n+1] = y[n] + (x[n] - y[n]) >> SHIFT
--
-- Pure shift-add, no multipliers, single-precision width (output width
-- equals input width -- the IIR doesn't grow).  Continuous-time time
-- constant TC ≈ 2^SHIFT samples, so for a sample rate `fs`:
--     TC_seconds ≈ 2^SHIFT / fs
--     f_3dB     ≈ fs / (2π · 2^SHIFT)
--
-- Example: SHIFT=4 at 2 kS/s -> TC ≈ 8 ms, f_3dB ≈ 19.9 Hz.
--          SHIFT=14 at 2 kS/s -> TC ≈ 8.2 s.
-- =========================================================================

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity dcf77_iir_lpf is
    generic (
        WIDTH : positive := 24;
        SHIFT : positive := 4
    );
    port (
        clk        : in  std_logic;
        rst        : in  std_logic;

        in_sample  : in  signed(WIDTH - 1 downto 0);
        in_valid   : in  std_logic;

        out_sample : out signed(WIDTH - 1 downto 0);
        out_valid  : out std_logic
    );
end entity dcf77_iir_lpf;

architecture rtl of dcf77_iir_lpf is

    type reg_type is record
        y         : signed(WIDTH - 1 downto 0);
        out_valid : std_logic;
    end record;

    constant R_RESET : reg_type := (
        y         => (others => '0'),
        out_valid => '0'
    );

    signal r, rin : reg_type;

begin

    out_sample <= r.y;
    out_valid  <= r.out_valid;

    comb : process (all)
        variable v        : reg_type;
        variable diff_ext : signed(WIDTH downto 0);
        variable step     : signed(WIDTH downto 0);
        variable y_ext    : signed(WIDTH downto 0);
        variable y_new    : signed(WIDTH downto 0);
    begin
        v := r;
        v.out_valid := '0';

        if in_valid = '1' then
            -- One extra bit for the subtraction.
            diff_ext := resize(in_sample, WIDTH + 1)
                      - resize(r.y, WIDTH + 1);
            step     := shift_right(diff_ext, SHIFT);
            y_ext    := resize(r.y, WIDTH + 1) + step;
            v.y         := resize(y_ext, WIDTH);
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
