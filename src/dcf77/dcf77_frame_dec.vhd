-- =========================================================================
-- dcf77_frame_dec.vhd
--
-- DCF77 frame buffer + BCD/parity decoder, shared between the AM and PM
-- paths.
--
-- Storage:
--   * 43-bit LSB-input shift register holding s16..s58 after all data
--     bit_valids have been processed.  Bit s_(58-i) sits at shift_reg(i)
--     so the newest bit (s58) is at shift_reg(0).
--   * Separate 1-bit s0_buf latched when bit_idx = 0 (AM only — PM never
--     visits index 0 during its data phase, so s0_buf trivially stays
--     '0' and satisfies the spec's "s0 = '0'" plausibility check by
--     construction).
--   * 6-bit mod-60 bit_idx representing the second-within-minute the
--     incoming bit_value belongs to (the *current* bit, not "next").
--
-- Two modes selected by the PM_MODE generic:
--   * AM (PM_MODE = false, START_INDEX = 0).  External minute_mark drives
--     the reset; bit_det fires it during the missing-s59 second.
--   * PM (PM_MODE = true,  START_INDEX = 15).  The SIK minute-mark
--     signature ("111111111100000") is detected combinationally on the
--     bottom 15 bits of the same shift_reg, edge-detected into a
--     single-cycle pulse, and used in lieu of the external minute_mark.
--     `pm_mm_detect` has been folded in via this generic.
--
-- Decode triggers when bit_idx = 58 AND bit_valid (= "s58 was just
-- shifted in").  Decode reads the virtual 59-bit frame view of the
-- storage, applies the existing plausibility checks (s0 = '0', s20 = '1',
-- DST mutex, three even-parities, BCD-range checks) and presents
-- dcf77_time_type when ok.
--
-- The bit_valid pulse and the (external or internal) minute_mark never
-- co-occur in any path used in this project: dcf77_bit_det's FSM emits
-- them mutually exclusively (line 611 of dcf77_bit_det.vhd) and the
-- SIK signature match is edge-detected one cycle after the bit_valid
-- that completed the pattern.
-- =========================================================================

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

use work.dcf77_pkg.all;

entity dcf77_frame_dec is
    generic (
        -- false = AM (external minute_mark, START_INDEX = 0)
        -- true  = PM (SIK signature detector internal, START_INDEX = 15)
        PM_MODE : boolean := false;
        -- PM-mode SIK signature tolerance.  Max bit errors allowed in
        -- the 12 non-edge bits of the 16-bit extended signature
        -- (sec 59 prefix + sec 0..14 = "0_111111111_1_00000").  The 4
        -- edge bits flanking the two transitions ('0'→'1' at sec59/sec0
        -- and '1'→'0' at sec9/sec10) ALWAYS require an exact match --
        -- this protects against off-by-1 false alignments where naive
        -- T-tolerance would collide with the +1 shift 50 % of the time.
        --
        -- Defaults to 1.  At a weak-signal regime of ε≈0.13 bit-error
        -- per second (= dcf_3_5 in the corpus), this gives ~29 %
        -- detect-per-minute vs ~12 % at T=0, while the false-positive
        -- MTBF against random data is ~84 min (vs ~33 min for naive
        -- T=1, ~55 h for T=0).
        MM_SIG_T_NON_EDGE : natural := 1
    );
    port (
        clk             : in  std_logic;
        rst             : in  std_logic;

        bit_value       : in  std_logic;
        bit_valid       : in  std_logic;
        minute_mark     : in  std_logic;     -- ignored when PM_MODE = true

        time_out        : out dcf77_time_type;
        time_valid      : out std_logic;
        last_frame      : out std_logic_vector(0 to 58);
        -- Current second within the minute (0..59).  Mod-60 counter
        -- reset to START_INDEX on the (external or internal) minute_mark.
        bit_index       : out bit_index_t;
        -- The effective minute_mark used inside the decoder.  Equals the
        -- input minute_mark for AM, equals the internally-generated
        -- SIK-signature edge pulse for PM.  Exposed so the AXI status
        -- register can mirror the PM-side minute_mark flag.
        minute_mark_out : out std_logic
    );
end entity dcf77_frame_dec;

architecture rtl of dcf77_frame_dec is

    function start_index_value return bit_index_t is
    begin
        if PM_MODE then
            return 15;
        else
            return 0;
        end if;
    end function;

    constant START_INDEX  : bit_index_t := start_index_value;

    -- Edge-protected SIK signature.  Extended to 16 bits including sec
    -- 59 (= '0' in PM, fixed by spec) as a prefix.  shift_reg position
    -- ↔ broadcast second (newest = bit_value just shifted in):
    --
    --   pos 15: sec 59 = '0'  ──┐
    --   pos 14: sec 0  = '1'  ──┘  EDGE bits ('0'→'1' transition anchor)
    --   pos 13..6: sec 1..8 = "11111111"  (non-edge, ≤T errors allowed)
    --   pos 5 : sec 9  = '1'  ──┐
    --   pos 4 : sec 10 = '0'  ──┘  EDGE bits ('1'→'0' transition anchor)
    --   pos 3..0: sec 11..14 = "0000"  (non-edge, shares the same T budget)
    --
    -- Off-by-1 alignments always violate >=1 edge bit (independent of
    -- data bits), so they reject regardless of T.  Off-by-N for N>=2
    -- has even larger edge violations.  T applies only to symbol
    -- errors strictly inside the constant-run regions.

    type reg_type is record
        shift_reg   : std_logic_vector(42 downto 0);
        s0_buf      : std_logic;
        bit_idx     : bit_index_t;
        match_prev  : std_logic;       -- used only when PM_MODE = true
        time_out    : dcf77_time_type;
        time_valid  : std_logic;
        last_frame  : std_logic_vector(0 to 58);
        mm_out      : std_logic;       -- registered effective mm output
    end record;

    constant R_RESET : reg_type := (
        shift_reg  => (others => '0'),
        s0_buf     => '0',
        bit_idx    => START_INDEX,
        match_prev => '0',
        time_out   => DCF77_TIME_RESET,
        time_valid => '0',
        last_frame => (others => '0'),
        mm_out     => '0'
    );

    signal r, rin : reg_type;

    -- Build the virtual 59-bit frame view from (shift_reg, s0_buf).
    -- Position k of the result is the s_k bit:
    --   s0  → result(0)        = s0_buf
    --   s_k → result(k)        = shift_reg(58 - k)   for k in [16, 58]
    --   s1..s15 are unused; left as '0'.
    function frame_view(sr : std_logic_vector(42 downto 0);
                        s0 : std_logic) return std_logic_vector
    is
        variable result : std_logic_vector(0 to 58) := (others => '0');
    begin
        result(0) := s0;
        for i in 0 to 42 loop
            result(58 - i) := sr(i);
        end loop;
        return result;
    end function;

begin

    time_out        <= r.time_out;
    time_valid      <= r.time_valid;
    last_frame      <= r.last_frame;
    bit_index       <= r.bit_idx;
    minute_mark_out <= r.mm_out;

    comb : process (all)
        variable v             : reg_type;
        variable pattern_match : std_logic;
        variable edge_ok       : boolean;
        variable non_edge_errs : natural range 0 to 12;
        variable mm_eff        : std_logic;
        variable do_decode     : boolean;
        variable frm           : std_logic_vector(0 to 58);
        variable min_u         : unsigned(6 downto 0);
        variable hour_u        : unsigned(5 downto 0);
        variable day_u         : unsigned(5 downto 0);
        variable wd_u          : unsigned(2 downto 0);
        variable mon_u         : unsigned(4 downto 0);
        variable year_u        : unsigned(7 downto 0);
        variable par_min, par_hour, par_date : std_logic;
        variable ok            : boolean;
        variable t             : dcf77_time_type;
    begin
        v := r;
        do_decode := false;

        -- Edge-protected SIK signature comparator on REGISTERED shift_reg
        -- (deliberately one-cycle delayed so the mm_internal pulse is
        -- non-coincident with the bit_valid that completed the signature).
        edge_ok :=     r.shift_reg(15) = '0'   -- sec 59
                   and r.shift_reg(14) = '1'   -- sec 0
                   and r.shift_reg(5)  = '1'   -- sec 9
                   and r.shift_reg(4)  = '0';  -- sec 10

        non_edge_errs := 0;
        for i in 6 to 13 loop                   -- sec 1..8 expected '1'
            if r.shift_reg(i) = '0' then
                non_edge_errs := non_edge_errs + 1;
            end if;
        end loop;
        for i in 0 to 3 loop                    -- sec 11..14 expected '0'
            if r.shift_reg(i) = '1' then
                non_edge_errs := non_edge_errs + 1;
            end if;
        end loop;

        if edge_ok and non_edge_errs <= MM_SIG_T_NON_EDGE then
            pattern_match := '1';
        else
            pattern_match := '0';
        end if;
        v.match_prev := pattern_match;

        -- Effective minute_mark: external for AM, edge-of-pattern for PM.
        if PM_MODE then
            mm_eff := pattern_match and not r.match_prev;
        else
            mm_eff := minute_mark;
        end if;

        v.mm_out := mm_eff;

        -- Process bit_valid using the registered r.bit_idx so that the
        -- mm-eff branch below cleanly OVERRIDES this when both fire in
        -- the (defensive) same cycle.
        if bit_valid = '1' then
            v.shift_reg := r.shift_reg(41 downto 0) & bit_value;

            if r.bit_idx = 0 then
                v.s0_buf := bit_value;
            end if;

            if r.bit_idx = 58 then
                do_decode := true;
            end if;

            -- mod-60 advance
            if r.bit_idx = 59 then
                v.bit_idx := 0;
            else
                v.bit_idx := r.bit_idx + 1;
            end if;
        end if;

        -- Decode happens BEFORE the reset takes effect because v.shift_reg /
        -- v.s0_buf at this point reflect the just-shifted-in s58.
        if do_decode then
            frm := frame_view(v.shift_reg, v.s0_buf);
            v.last_frame := frm;

            par_min  := even_parity(frm(21 to 28));
            par_hour := even_parity(frm(29 to 35));
            par_date := even_parity(frm(36 to 58));

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
        end if;

        -- mm reset: clears storage and re-seeds bit_idx to the per-mode
        -- starting index.  Sequenced AFTER the bit_valid shift / decode so
        -- the decode (if any in this same cycle) gets to see v.shift_reg
        -- before it's wiped.  Also clears match_prev so the internal mm
        -- can re-arm cleanly on the next minute's signature.
        if mm_eff = '1' then
            v.shift_reg  := (others => '0');
            v.s0_buf     := '0';
            v.bit_idx    := START_INDEX;
            v.match_prev := '0';
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
