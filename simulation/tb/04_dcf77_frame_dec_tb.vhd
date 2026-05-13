-- =========================================================================
-- 04_dcf77_frame_dec_tb.vhd
--
-- VUnit testbench for dcf77_frame_dec.  Builds known-good 59-bit DCF77
-- frames from (year, month, day, weekday, hour, minute) inputs and drives
-- them via bit_valid pulses followed by a minute_mark.  Checks the decoded
-- time fields and time_valid flag.  Also tests intentionally bad parity to
-- confirm rejection.
-- =========================================================================

library vunit_lib;
context vunit_lib.vunit_context;

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library dcf77;
use dcf77.dcf77_pkg.all;

entity dcf77_frame_dec_tb is
    generic (runner_cfg : string);
end entity;

architecture tb of dcf77_frame_dec_tb is

    constant CLK_PERIOD : time := 8 ns;

    signal clk         : std_logic := '0';
    signal rst         : std_logic := '1';

    signal bit_value   : std_logic := '0';
    signal bit_valid   : std_logic := '0';
    signal minute_mark : std_logic := '0';

    signal time_out    : dcf77_time_type;
    signal time_valid  : std_logic;
    signal last_frame  : std_logic_vector(0 to 58);

    -- DCF77 bit weights for the LSB-first BCD slices.  The array length
    -- equals the slice length and the i'th entry is the weight of position
    -- slice'low + i.
    constant W_MIN  : integer_vector(0 to 6) := (1, 2, 4, 8, 10, 20, 40);
    constant W_HR   : integer_vector(0 to 5) := (1, 2, 4, 8, 10, 20);
    constant W_DAY  : integer_vector(0 to 5) := (1, 2, 4, 8, 10, 20);
    constant W_WD   : integer_vector(0 to 2) := (1, 2, 4);
    constant W_MON  : integer_vector(0 to 4) := (1, 2, 4, 8, 10);
    constant W_YR   : integer_vector(0 to 7) := (1, 2, 4, 8, 10, 20, 40, 80);

    -- Encode a value into a BCD-LSB-first bit-slice using the given weights.
    -- Greedy from the highest weight downwards (DCF77 representation is
    -- unique for any value in the valid range).
    function bcd_encode(value : integer; weights : integer_vector)
        return std_logic_vector
    is
        variable result : std_logic_vector(0 to weights'high) := (others => '0');
        variable rem_v  : integer := value;
    begin
        for k in weights'high downto 0 loop
            if rem_v >= weights(k) then
                result(k) := '1';
                rem_v := rem_v - weights(k);
            end if;
        end loop;
        return result;
    end function;

    function xor_slice(s : std_logic_vector) return std_logic is
        variable acc : std_logic := '0';
    begin
        for i in s'range loop
            acc := acc xor s(i);
        end loop;
        return acc;
    end function;

    -- Build a full 59-bit frame.  dst_active selects MESZ ('1') vs. MEZ ('0').
    function build_frame(year, month, day, weekday, hour, minute : integer;
                         dst_active : std_logic) return std_logic_vector
    is
        variable f       : std_logic_vector(0 to 58) := (others => '0');
        variable enc_min : std_logic_vector(0 to 6);
        variable enc_hr  : std_logic_vector(0 to 5);
        variable enc_day : std_logic_vector(0 to 5);
        variable enc_wd  : std_logic_vector(0 to 2);
        variable enc_mon : std_logic_vector(0 to 4);
        variable enc_yr  : std_logic_vector(0 to 7);
    begin
        f(0)  := '0';                                  -- start bit
        f(15) := '0';                                  -- call bit
        f(16) := '0';                                  -- DST announce
        f(17) := dst_active;
        f(18) := not dst_active;                       -- invariant: 17 XOR 18 = 1
        f(19) := '0';                                  -- no leap second
        f(20) := '1';                                  -- start of time info

        enc_min := bcd_encode(minute, W_MIN);
        for i in 0 to 6 loop f(21 + i) := enc_min(i); end loop;
        f(28) := xor_slice(enc_min);                   -- even parity for bits 21..28

        enc_hr := bcd_encode(hour, W_HR);
        for i in 0 to 5 loop f(29 + i) := enc_hr(i); end loop;
        f(35) := xor_slice(enc_hr);

        enc_day := bcd_encode(day, W_DAY);
        for i in 0 to 5 loop f(36 + i) := enc_day(i); end loop;
        enc_wd  := bcd_encode(weekday, W_WD);
        for i in 0 to 2 loop f(42 + i) := enc_wd(i); end loop;
        enc_mon := bcd_encode(month, W_MON);
        for i in 0 to 4 loop f(45 + i) := enc_mon(i); end loop;
        enc_yr  := bcd_encode(year, W_YR);
        for i in 0 to 7 loop f(50 + i) := enc_yr(i); end loop;

        -- Date parity over bits 36..58 (even).
        f(58) := xor_slice(f(36 to 57));

        return f;
    end function;

begin

    clk <= not clk after CLK_PERIOD / 2;

    dut : entity dcf77.dcf77_frame_dec
        port map (
            clk         => clk,
            rst         => rst,
            bit_value   => bit_value,
            bit_valid   => bit_valid,
            minute_mark => minute_mark,
            time_out    => time_out,
            time_valid  => time_valid,
            last_frame  => last_frame
        );

    test_runner : process

        procedure send_frame(frm : std_logic_vector(0 to 58)) is
        begin
            for i in 0 to 58 loop
                bit_value <= frm(i);
                bit_valid <= '1';
                wait until rising_edge(clk);
                bit_valid <= '0';
                wait until rising_edge(clk);
            end loop;
            -- Now signal minute_mark to trigger decode.
            minute_mark <= '1';
            wait until rising_edge(clk);
            minute_mark <= '0';
            -- Allow one cycle for the registered output to update.
            wait until rising_edge(clk);
        end procedure;

        variable frm : std_logic_vector(0 to 58);

    begin
        test_runner_setup(runner, runner_cfg);

        rst <= '1';
        wait for 4 * CLK_PERIOD;
        wait until rising_edge(clk);
        rst <= '0';
        wait until rising_edge(clk);

        while test_suite loop

            -----------------------------------------------------------------
            if run("decode_example") then
                -- 2026-05-12, Tuesday (weekday=2), 23:42, summer time.
                frm := build_frame(year=>26, month=>5, day=>12, weekday=>2,
                                   hour=>23, minute=>42, dst_active=>'1');
                send_frame(frm);
                check_equal(time_valid, '1', "time_valid for clean frame");
                check_equal(to_integer(time_out.minute), 42, "minute = 42");
                check_equal(to_integer(time_out.hour),   23, "hour = 23");
                check_equal(to_integer(time_out.day),    12, "day = 12");
                check_equal(to_integer(time_out.weekday), 2, "weekday = 2");
                check_equal(to_integer(time_out.month),   5, "month = 5");
                check_equal(to_integer(time_out.year),   26, "year = 26");
                check_equal(time_out.dst_active, '1', "MESZ active");

            -----------------------------------------------------------------
            elsif run("midnight") then
                -- 2026-01-01, Thursday (1=Mon..7=Sun -> Thursday=4), 00:00, MEZ.
                frm := build_frame(year=>26, month=>1, day=>1, weekday=>4,
                                   hour=>0, minute=>0, dst_active=>'0');
                send_frame(frm);
                check_equal(time_valid, '1', "time_valid for midnight frame");
                check_equal(to_integer(time_out.minute), 0, "minute = 0");
                check_equal(to_integer(time_out.hour),   0, "hour = 0");
                check_equal(to_integer(time_out.day),    1, "day = 1");
                check_equal(to_integer(time_out.month),  1, "month = 1");
                check_equal(time_out.dst_active, '0', "MEZ active");

            -----------------------------------------------------------------
            elsif run("bad_parity") then
                -- Build a valid frame, then corrupt the minute parity bit.
                frm := build_frame(year=>26, month=>5, day=>12, weekday=>2,
                                   hour=>23, minute=>42, dst_active=>'1');
                frm(28) := not frm(28);
                send_frame(frm);
                check_equal(time_valid, '0',
                            "time_valid clears on bad minute parity");

            -----------------------------------------------------------------
            elsif run("bad_start_bit") then
                frm := build_frame(year=>26, month=>5, day=>12, weekday=>2,
                                   hour=>10, minute=>5, dst_active=>'1');
                frm(0) := '1';   -- start bit MUST be 0
                send_frame(frm);
                check_equal(time_valid, '0',
                            "time_valid clears on bad start bit");

            end if;
        end loop;

        test_runner_cleanup(runner);
        wait;
    end process;

    test_runner_watchdog(runner, 2 ms);

end architecture;
