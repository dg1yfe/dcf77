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
    -- Envelope power = top18(I)*top18(I) + top18(Q)*top18(Q).  The mixer
    -- operands have LSB position 4 after the top-18 truncation, so the
    -- product has LSB position 8 and natural full range sfixed(43 downto 8).
    -- env_word_t is sfixed (carries the sign bit so the same arithmetic
    -- operators work in the threshold IIRs) but the value is by construction
    -- non-negative.  We drop the bottom 2 bits and the top overflow bits of
    -- the natural sum, keeping a compact 26 b magnitude with LSB 2^10.
    subtype env_word_t   is sfixed(35 downto 10);  -- 26 b power, LSB 2^10
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
    -- Post-decimation I/Q tap (244 kS/s).  Exposed at the core boundary so a
    -- future PM correlator can attach without reaching inside the receiver.
    ---------------------------------------------------------------------------
    type dcf77_iq_tap_type is record
        i_data : iq_word_t;
        q_data : iq_word_t;
        valid  : std_logic;
    end record;

    constant DCF77_IQ_TAP_RESET : dcf77_iq_tap_type := (
        i_data => (others => '0'),
        q_data => (others => '0'),
        valid  => '0'
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
    -- Decoder status.  sync_state enum is mapped onto a 2 b vector for the
    -- benefit of the AXI register map.
    ---------------------------------------------------------------------------
    constant DCF77_SYNC_SEARCH : std_logic_vector(1 downto 0) := "00";
    constant DCF77_SYNC_LOCK   : std_logic_vector(1 downto 0) := "01";
    constant DCF77_SYNC_HOLD   : std_logic_vector(1 downto 0) := "10";

    type dcf77_status_type is record
        time_valid : std_logic;                       -- last frame fully parity-OK
        sync_state : std_logic_vector(1 downto 0);
        rssi       : rssi_word_t;
        bit_index  : unsigned(5 downto 0);            -- current second 0..58
    end record;

    constant DCF77_STATUS_RESET : dcf77_status_type := (
        time_valid => '0',
        sync_state => DCF77_SYNC_SEARCH,
        rssi       => (others => '0'),
        bit_index  => (others => '0')
    );

    ---------------------------------------------------------------------------
    -- Aggregate core output.
    ---------------------------------------------------------------------------
    type dcf77_out_type is record
        time   : dcf77_time_type;
        status : dcf77_status_type;
        iq_tap : dcf77_iq_tap_type;
    end record;

    constant DCF77_OUT_RESET : dcf77_out_type := (
        time   => DCF77_TIME_RESET,
        status => DCF77_STATUS_RESET,
        iq_tap => DCF77_IQ_TAP_RESET
    );

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
            -- Decode bit position i -> weight 2^(i mod 4) * 10^(i / 4)
            -- Compute weight as integer to avoid a runtime power op.
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

end package body dcf77_pkg;
