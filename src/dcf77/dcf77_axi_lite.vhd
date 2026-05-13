-- =========================================================================
-- dcf77_axi_lite.vhd
--
-- AXI4-Lite slave wrapper for the DCF77 receiver core.  This is a
-- placeholder skeleton: the user will supply a project AXI4-Lite reference
-- implementation; the entity and register layout below match the plan and
-- will be wired up once that example is available.
--
-- Register map (offsets are byte addresses; all registers are 32 b):
--   0x00 RW  CTRL         enable, soft-reset, threshold-mode bits
--   0x04 R   STATUS       time_valid, sync_state, parity flags, bit_index
--   0x08 R   TIME_HMS     hour | minute | seconds
--   0x0C R   DATE         year | month | day | weekday
--   0x10 R   FLAGS        dst_active, dst_announce, leap_announce, call_bit
--   0x14 R   RSSI         adaptive max / min / threshold
--   0x18 R   LAST_FRAME0  bits 0..31 of last 59-b frame
--   0x1C R   LAST_FRAME1  bits 32..58
--
-- The combinational read-mux + write-decoder + handshakes will be filled in
-- against the user's AXI4-Lite stencil.  The entity port list below is
-- the conventional AXI4-Lite slave subset.
-- =========================================================================

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

use work.dcf77_pkg.all;

entity dcf77_axi_lite is
    generic (
        ADDR_WIDTH : positive := 5;    -- 32 B address space -> 5 bits
        DATA_WIDTH : positive := 32
    );
    port (
        s_axi_aclk    : in  std_logic;
        s_axi_aresetn : in  std_logic;

        -- Write address channel
        s_axi_awaddr  : in  std_logic_vector(ADDR_WIDTH - 1 downto 0);
        s_axi_awvalid : in  std_logic;
        s_axi_awready : out std_logic;
        -- Write data channel
        s_axi_wdata   : in  std_logic_vector(DATA_WIDTH - 1 downto 0);
        s_axi_wstrb   : in  std_logic_vector((DATA_WIDTH / 8) - 1 downto 0);
        s_axi_wvalid  : in  std_logic;
        s_axi_wready  : out std_logic;
        -- Write response channel
        s_axi_bresp   : out std_logic_vector(1 downto 0);
        s_axi_bvalid  : out std_logic;
        s_axi_bready  : in  std_logic;
        -- Read address channel
        s_axi_araddr  : in  std_logic_vector(ADDR_WIDTH - 1 downto 0);
        s_axi_arvalid : in  std_logic;
        s_axi_arready : out std_logic;
        -- Read data channel
        s_axi_rdata   : out std_logic_vector(DATA_WIDTH - 1 downto 0);
        s_axi_rresp   : out std_logic_vector(1 downto 0);
        s_axi_rvalid  : out std_logic;
        s_axi_rready  : in  std_logic;

        -- Connections to the dcf77_core instance
        core_time   : in  dcf77_time_type;
        core_status : in  dcf77_status_type;
        core_last_frame : in  std_logic_vector(0 to 58)
    );
end entity dcf77_axi_lite;

architecture stub of dcf77_axi_lite is
begin

    -- Skeleton tie-offs.  The real AXI4-Lite handshake + register decode
    -- will replace this once the user's example wrapper is wired up.
    s_axi_awready <= '0';
    s_axi_wready  <= '0';
    s_axi_bresp   <= "00";
    s_axi_bvalid  <= '0';
    s_axi_arready <= '0';
    s_axi_rdata   <= (others => '0');
    s_axi_rresp   <= "00";
    s_axi_rvalid  <= '0';

end architecture stub;
