-- =========================================================================
-- dcf77_pkg.vhd
--
-- Shared types and helper functions for the DCF77 receiver / decoder.
--
-- Convention: signal-processing nets are sfixed / ufixed with explicit
-- (high downto low) ranges so the LSB scale of every word is encoded in the
-- type.  Pure modular counters (phase accumulator, bit-position counter,
-- pulse-width timer) stay on unsigned, and frame-buffer style storage stays
-- on std_logic_vector.
-- =========================================================================

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use ieee.fixed_pkg.all;

package dcf77_pkg is

    ---------------------------------------------------------------------------
    -- Signal-processing subtypes.
    --
    -- All sfixed ranges below use LSB position 0 (integer-valued) for the
    -- top-level interfaces.  Internal CIC stages choose their own
    -- (high downto low) ranges via dcf77_cic_widths_pkg.
    ---------------------------------------------------------------------------
    subtype adc_sample_t is sfixed(13 downto 0);   -- raw signed ADC sample
    subtype iq_word_t    is sfixed(21 downto 0);   -- 22 b signed I/Q at 244 kS/s

    -- Position of the next bit to be received within the minute.  0..58 are
    -- the valid received-bit slots; 59 is the "frame complete, awaiting
    -- minute mark" sentinel.  Range-limited so it can't silently overflow.
    subtype bit_index_t  is natural range 0 to 59;
    -- Envelope power = top18(I)*top18(I) + top18(Q)*top18(Q).  The mixer
    -- env_word_t carries the AM envelope magnitude computed by dcf77_env via
    -- the |I| + 3/8*|Q| approximation (alpha=1, beta=3/8).  The iq_word_t
    -- operands fit in 22 b at LSB 0, so the magnitude max is
    -- (1 + 3/8) * 2^21 ~= 2.88 M, well within the 25 b unsigned range of
    -- env_word_t.  sfixed (signed) is kept so the bit_det IIRs can reuse
    -- the same arithmetic operators across env_in, env_max, env_min and the
    -- difference terms, but the value is by construction non-negative.
    subtype env_word_t   is sfixed(25 downto 0);   -- 26 b magnitude, LSB 2^0
    subtype rssi_word_t  is ufixed(15 downto 0);   -- 16 b slow-IIR RSSI

    ---------------------------------------------------------------------------
    -- Input record from the host (or test bench) into the decoder core.
    ---------------------------------------------------------------------------
    type dcf77_in_type is record
        sample       : adc_sample_t;
        sample_valid : std_logic;
    end record;

    constant DCF77_IN_RESET : dcf77_in_type := (
        sample       => (others => '0'),
        sample_valid => '0'
    );

    ---------------------------------------------------------------------------
    -- Decoded time-of-frame.  All fields are BCD-decoded integer values.
    -- The transmitted time is the wall-clock minute that begins at the
    -- next minute mark (see dcf77_format_spec.md §1.5).
    ---------------------------------------------------------------------------
    type dcf77_time_type is record
        year         : unsigned(7 downto 0);  -- 00..99
        month        : unsigned(3 downto 0);  -- 01..12
        day          : unsigned(4 downto 0);  -- 01..31
        weekday      : unsigned(2 downto 0);  -- 1=Mon .. 7=Sun
        hour         : unsigned(4 downto 0);  -- 00..23
        minute       : unsigned(5 downto 0);  -- 00..59
        dst_active   : std_logic;             -- bit 17 (MESZ, UTC+2)
        dst_announce : std_logic;             -- bit 16 (DST change next hour)
        leap_announce: std_logic;             -- bit 19 (leap second next hour)
    end record;

    constant DCF77_TIME_RESET : dcf77_time_type := (
        year          => (others => '0'),
        month         => (others => '0'),
        day           => (others => '0'),
        weekday       => (others => '0'),
        hour          => (others => '0'),
        minute        => (others => '0'),
        dst_active    => '0',
        dst_announce  => '0',
        leap_announce => '0'
    );

    ---------------------------------------------------------------------------
    -- Decoder synchronisation state.  The enum is the natural form for the
    -- RTL; the explicit conversion function pins the AXI register encoding
    -- so it is independent of how synthesis assigns the enum.
    ---------------------------------------------------------------------------
    type sync_state_t is (sync_search, sync_lock, sync_hold);

    function sync_state_to_slv(s : sync_state_t)
        return std_logic_vector;
    function slv_to_sync_state(s : std_logic_vector)
        return sync_state_t;

    -- Status carries *everything* the decoder has produced: the full decoded
    -- time, the validity / sync flags, the raw 59-bit frame buffer, and the
    -- RSSI.  The dcf77_core aggregates this from its sub-blocks and presents
    -- a single record to the AXI readmux (the only other input to the
    -- readmux is the dcf77_control_type echoed from the writemux).
    type dcf77_status_type is record
        time         : dcf77_time_type;               -- decoded time fields
        time_valid   : std_logic;                     -- last frame fully parity-OK
        sync_state   : sync_state_t;
        sync_quality : unsigned(5 downto 0);          -- 0..63 in-guard streak counter
        mm_quality   : unsigned(1 downto 0);          -- 0..3 minute-mark confidence
        cycle_in_min : unsigned(5 downto 0);          -- 0..59 predicted sec-in-minute
        rssi         : rssi_word_t;                   -- top bits of env_max
        env_min      : rssi_word_t;                   -- top bits of env_min
        noise        : rssi_word_t;                   -- top bits of noise IIR (MAD)
        bit_index    : bit_index_t;                   -- current second 0..58 (59 = frame full)
        last_frame   : std_logic_vector(0 to 58);     -- raw bits of last minute
    end record;

    constant DCF77_STATUS_RESET : dcf77_status_type := (
        time         => DCF77_TIME_RESET,
        time_valid   => '0',
        sync_state   => sync_search,
        sync_quality => (others => '0'),
        mm_quality   => (others => '0'),
        cycle_in_min => (others => '0'),
        rssi         => (others => '0'),
        env_min      => (others => '0'),
        noise        => (others => '0'),
        bit_index    => 0,
        last_frame   => (others => '0')
    );

    ---------------------------------------------------------------------------
    -- Aggregate core output: just the decoded status (the post-mixer I/Q
    -- tap that used to live here was removed alongside the CIC3 stage).
    ---------------------------------------------------------------------------
    type dcf77_out_type is record
        status : dcf77_status_type;
    end record;

    constant DCF77_OUT_RESET : dcf77_out_type := (
        status => DCF77_STATUS_RESET
    );

    ---------------------------------------------------------------------------
    -- Host-writable control word.  Plumbed from the AXI4-Lite writemux into
    -- the core; the readmux can also read it back so the host can verify a
    -- write took effect.
    ---------------------------------------------------------------------------
    type dcf77_control_type is record
        enable     : std_logic;     -- master enable
        soft_reset : std_logic;     -- pulse this high to reset the core
        env_mode   : std_logic;     -- 0 = abs(I) only, 1 = |I| + 3/8*|Q|
    end record;

    constant DCF77_CONTROL_RESET : dcf77_control_type := (
        enable     => '1',          -- default-on
        soft_reset => '0',
        env_mode   => '0'           -- abs(I) by default; host can set to
                                    -- '1' for |I|+3/8|Q| (interferer-robust
                                    -- at ~0.5 dB sensitivity cost).
    );

    ---------------------------------------------------------------------------
    -- AXI4-Lite register map (word index = addr(7 downto 2)).
    ---------------------------------------------------------------------------
    constant REG_CTRL        : natural := 0;   -- 0x00 RW
    constant REG_STATUS      : natural := 1;   -- 0x04 R
    constant REG_TIME_HMS    : natural := 2;   -- 0x08 R
    constant REG_DATE        : natural := 3;   -- 0x0C R
    constant REG_FLAGS       : natural := 4;   -- 0x10 R
    constant REG_RSSI        : natural := 5;   -- 0x14 R   env_max top 16 b
    constant REG_LAST_FRAME0 : natural := 6;   -- 0x18 R
    constant REG_LAST_FRAME1 : natural := 7;   -- 0x1C R
    constant REG_ENV_MIN     : natural := 8;   -- 0x20 R   env_min top 16 b
    constant REG_NOISE       : natural := 9;   -- 0x24 R   noise_iir top 16 b
    constant REG_SYNC_DIAG   : natural := 10;  -- 0x28 R   sync_q / mm_q / cycle_in_min

    ---------------------------------------------------------------------------
    -- 32-b register pack / unpack helpers.  The bit layouts below match the
    -- plan in /Users/eligs/.claude/plans/this-project-aims-to-toasty-pelican.md.
    ---------------------------------------------------------------------------

    -- CTRL register: bit 0 enable, bit 1 soft_reset, bit 2 env_mode, rest reserved.
    function ctrl_to_slv (c : dcf77_control_type) return std_logic_vector;
    function slv_to_ctrl (s : std_logic_vector) return dcf77_control_type;

    -- All read-side registers take the full dcf77_status_type since that is
    -- the only data source from the decoder.

    -- STATUS register:
    --   bits  1..0  : sync_state
    --   bit   2     : time_valid
    --   bits 13..8  : bit_index
    --   bits 31..16 : rssi
    function status_to_slv (s : dcf77_status_type) return std_logic_vector;

    -- TIME_HMS register: bits 5..0 minute, 12..8 hour.
    function time_hms_to_slv (s : dcf77_status_type) return std_logic_vector;

    -- DATE register: 7..0 year, 11..8 month, 20..16 day, 26..24 weekday.
    function date_to_slv (s : dcf77_status_type) return std_logic_vector;

    -- FLAGS register: bit 0 dst_active, bit 1 dst_announce, bit 2 leap_announce.
    function flags_to_slv (s : dcf77_status_type) return std_logic_vector;

    -- LAST_FRAME 0 / 1: frame bits 0..31 / 32..58.
    function last_frame0_to_slv (s : dcf77_status_type) return std_logic_vector;
    function last_frame1_to_slv (s : dcf77_status_type) return std_logic_vector;

    -- SYNC_DIAG register:
    --   bits  5..0  : sync_quality (0..63 saturating in-guard streak)
    --   bits  7..6  : mm_quality   (0..3 minute-mark confidence counter)
    --   bits 13..8  : cycle_in_min (0..59 predicted second-in-minute)
    --   bits 31..14 : reserved
    function sync_diag_to_slv (s : dcf77_status_type) return std_logic_vector;

    ---------------------------------------------------------------------------
    -- Helper functions.
    ---------------------------------------------------------------------------

    -- Even parity over an arbitrary-length std_logic_vector.
    function even_parity(v : std_logic_vector) return std_logic;

    -- BCD decode a contiguous slice of the 59-bit frame, LSB-first
    -- (per dcf77_format_spec.md §1.3).  weight_lo is the value of the
    -- lowest-order bit (typically 1).  Returns the sum-of-weights as an
    -- unsigned of width `result_width`.
    function bcd_decode_lsb_first(slice       : std_logic_vector;
                                  result_width: natural)
        return unsigned;

end package dcf77_pkg;


package body dcf77_pkg is

    function even_parity(v : std_logic_vector) return std_logic is
        variable acc : std_logic := '0';
    begin
        for i in v'range loop
            acc := acc xor v(i);
        end loop;
        return acc;
    end function;

    function bcd_decode_lsb_first(slice       : std_logic_vector;
                                  result_width: natural)
        return unsigned
    is
        variable acc    : unsigned(result_width - 1 downto 0) := (others => '0');
        variable weight : unsigned(result_width - 1 downto 0);
    begin
        -- DCF77 BCD bit weights are 1,2,4,8 for the units decade then
        -- 10,20,40,80 for the tens decade (year reaches all 8 positions).
        -- The slice is interpreted LSB-first per dcf77_format_spec.md §1.3.
        for i in 0 to slice'length - 1 loop
            case i is
                when 0 => weight := to_unsigned(1,  result_width);
                when 1 => weight := to_unsigned(2,  result_width);
                when 2 => weight := to_unsigned(4,  result_width);
                when 3 => weight := to_unsigned(8,  result_width);
                when 4 => weight := to_unsigned(10, result_width);
                when 5 => weight := to_unsigned(20, result_width);
                when 6 => weight := to_unsigned(40, result_width);
                when 7 => weight := to_unsigned(80, result_width);
                when others => weight := (others => '0');
            end case;
            if slice(slice'low + i) = '1' then
                acc := acc + weight;
            end if;
        end loop;
        return acc;
    end function;

    ---------------------------------------------------------------------------
    -- Register pack / unpack
    ---------------------------------------------------------------------------

    function sync_state_to_slv(s : sync_state_t) return std_logic_vector is
    begin
        case s is
            when sync_search => return "00";
            when sync_lock   => return "01";
            when sync_hold   => return "10";
        end case;
    end function;

    function slv_to_sync_state(s : std_logic_vector) return sync_state_t is
    begin
        case s is
            when "00"   => return sync_search;
            when "01"   => return sync_lock;
            when "10"   => return sync_hold;
            when others => return sync_search;
        end case;
    end function;

    function ctrl_to_slv (c : dcf77_control_type) return std_logic_vector is
        variable r : std_logic_vector(31 downto 0) := (others => '0');
    begin
        r(0) := c.enable;
        r(1) := c.soft_reset;
        r(2) := c.env_mode;
        return r;
    end function;

    function slv_to_ctrl (s : std_logic_vector) return dcf77_control_type is
        variable c : dcf77_control_type;
    begin
        c.enable     := s(s'low + 0);
        c.soft_reset := s(s'low + 1);
        c.env_mode   := s(s'low + 2);
        return c;
    end function;

    function status_to_slv (s : dcf77_status_type) return std_logic_vector is
        variable r : std_logic_vector(31 downto 0) := (others => '0');
    begin
        r(1 downto 0)   := sync_state_to_slv(s.sync_state);
        r(2)            := s.time_valid;
        r(13 downto 8)  := std_logic_vector(to_unsigned(s.bit_index, 6));
        r(31 downto 16) := std_logic_vector(s.rssi);
        return r;
    end function;

    function time_hms_to_slv (s : dcf77_status_type) return std_logic_vector is
        variable r : std_logic_vector(31 downto 0) := (others => '0');
    begin
        r(5 downto 0)  := std_logic_vector(s.time.minute);
        r(12 downto 8) := std_logic_vector(s.time.hour);
        return r;
    end function;

    function date_to_slv (s : dcf77_status_type) return std_logic_vector is
        variable r : std_logic_vector(31 downto 0) := (others => '0');
    begin
        r(7 downto 0)   := std_logic_vector(s.time.year);
        r(11 downto 8)  := std_logic_vector(s.time.month);
        r(20 downto 16) := std_logic_vector(s.time.day);
        r(26 downto 24) := std_logic_vector(s.time.weekday);
        return r;
    end function;

    function flags_to_slv (s : dcf77_status_type) return std_logic_vector is
        variable r : std_logic_vector(31 downto 0) := (others => '0');
    begin
        r(0) := s.time.dst_active;
        r(1) := s.time.dst_announce;
        r(2) := s.time.leap_announce;
        return r;
    end function;

    function last_frame0_to_slv (s : dcf77_status_type) return std_logic_vector is
        variable r : std_logic_vector(31 downto 0) := (others => '0');
    begin
        for i in 0 to 31 loop
            r(i) := s.last_frame(i);
        end loop;
        return r;
    end function;

    function last_frame1_to_slv (s : dcf77_status_type) return std_logic_vector is
        variable r : std_logic_vector(31 downto 0) := (others => '0');
    begin
        for i in 32 to 58 loop
            r(i - 32) := s.last_frame(i);
        end loop;
        return r;
    end function;

    function sync_diag_to_slv (s : dcf77_status_type) return std_logic_vector is
        variable r : std_logic_vector(31 downto 0) := (others => '0');
    begin
        r( 5 downto  0) := std_logic_vector(s.sync_quality);
        r( 7 downto  6) := std_logic_vector(s.mm_quality);
        r(13 downto  8) := std_logic_vector(s.cycle_in_min);
        return r;
    end function;

end package body dcf77_pkg;
