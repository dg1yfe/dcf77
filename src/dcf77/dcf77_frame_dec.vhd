-- =========================================================================
-- dcf77_frame_dec.vhd
--
-- 59-bit frame buffer + BCD/parity decoder.  On each minute_mark the buffer
-- holding the just-completed minute is decoded according to
-- doc/dcf77_format_spec.md and presented as a dcf77_time_type record.
-- time_valid is held high while the latched frame's parity/range checks
-- pass; it drops only when a fresh minute fails the checks.
--
-- Per the FSM in dcf77_bit_det, the rising edge that ends the long gap
-- (1500 ms+) emits bit_valid (= bit 0 of the new minute) AND minute_mark in
-- the same clock cycle.  Decoding is therefore done on the OLD buffer
-- before the new bit overwrites position 0.
-- =========================================================================

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

use work.dcf77_pkg.all;

entity dcf77_frame_dec is
    port (
        clk         : in  std_logic;
        rst         : in  std_logic;

        bit_value   : in  std_logic;
        bit_valid   : in  std_logic;
        minute_mark : in  std_logic;

        time_out    : out dcf77_time_type;
        time_valid  : out std_logic;
        last_frame  : out std_logic_vector(0 to 58)
    );
end entity dcf77_frame_dec;

architecture rtl of dcf77_frame_dec is

    type reg_type is record
        frame_buf  : std_logic_vector(0 to 58);
        bit_index  : unsigned(5 downto 0);
        time_out   : dcf77_time_type;
        time_valid : std_logic;
        last_frame : std_logic_vector(0 to 58);
    end record;

    constant ZERO_FRAME : std_logic_vector(0 to 58) := (others => '0');
    constant R_RESET : reg_type := (
        frame_buf  => ZERO_FRAME,
        bit_index  => (others => '0'),
        time_out   => DCF77_TIME_RESET,
        time_valid => '0',
        last_frame => ZERO_FRAME
    );

    signal r, rin : reg_type;

begin

    time_out   <= r.time_out;
    time_valid <= r.time_valid;
    last_frame <= r.last_frame;

    comb : process (all)
        variable v       : reg_type;
        variable frm     : std_logic_vector(0 to 58);
        variable min_u   : unsigned(6 downto 0);
        variable hour_u  : unsigned(5 downto 0);
        variable day_u   : unsigned(5 downto 0);
        variable wd_u    : unsigned(2 downto 0);
        variable mon_u   : unsigned(4 downto 0);
        variable year_u  : unsigned(7 downto 0);
        variable par_min, par_hour, par_date : std_logic;
        variable ok      : boolean;
        variable t       : dcf77_time_type;
    begin
        v := r;

        -- Process bit_valid and minute_mark.  They may co-occur (rising edge
        -- after the minute-mark gap).
        if minute_mark = '1' then
            -- Decode the OLD frame_buf as the just-completed minute.
            frm := r.frame_buf;
            v.last_frame := frm;

            -- Parity over each protected region.  Per spec each region is
            -- even-parity *including* the parity bit, so the XOR of the
            -- whole protected region should be 0.
            par_min  := even_parity(frm(21 to 28));
            par_hour := even_parity(frm(29 to 35));
            par_date := even_parity(frm(36 to 58));

            -- BCD decode the value-carrying slices (LSB-first).
            min_u  := bcd_decode_lsb_first(frm(21 to 27), 7);
            hour_u := bcd_decode_lsb_first(frm(29 to 34), 6);
            day_u  := bcd_decode_lsb_first(frm(36 to 41), 6);
            wd_u   := bcd_decode_lsb_first(frm(42 to 44), 3);
            mon_u  := bcd_decode_lsb_first(frm(45 to 49), 5);
            year_u := bcd_decode_lsb_first(frm(50 to 57), 8);

            t.minute       := min_u(5 downto 0);
            t.hour         := hour_u(4 downto 0);
            t.day          := day_u(4 downto 0);
            t.weekday      := wd_u;
            t.month        := mon_u(3 downto 0);
            t.year         := year_u;
            t.dst_active   := frm(17);
            t.dst_announce := frm(16);
            t.leap_announce:= frm(19);

            -- Plausibility checks (spec §1.4).
            ok :=     frm(0)  = '0'
                  and frm(20) = '1'
                  and (frm(17) xor frm(18)) = '1'
                  and par_min  = '0'
                  and par_hour = '0'
                  and par_date = '0'
                  and to_integer(min_u)  <= 59
                  and to_integer(hour_u) <= 23
                  and to_integer(day_u)  >= 1
                  and to_integer(day_u)  <= 31
                  and to_integer(mon_u)  >= 1
                  and to_integer(mon_u)  <= 12
                  and to_integer(wd_u)   >= 1
                  and to_integer(wd_u)   <= 7;

            if ok then
                v.time_out   := t;
                v.time_valid := '1';
            else
                v.time_valid := '0';
            end if;

            -- Start the new minute's buffer.  If bit_valid co-occurs with
            -- minute_mark, the new bit is bit 0 of the next minute.
            v.frame_buf := ZERO_FRAME;
            if bit_valid = '1' then
                v.frame_buf(0) := bit_value;
                v.bit_index    := to_unsigned(1, 6);
            else
                v.bit_index := (others => '0');
            end if;

        elsif bit_valid = '1' then
            if r.bit_index < to_unsigned(59, 6) then
                v.frame_buf(to_integer(r.bit_index)) := bit_value;
                v.bit_index := r.bit_index + 1;
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
