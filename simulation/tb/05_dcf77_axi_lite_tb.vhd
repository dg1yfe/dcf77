-- =========================================================================
-- 05_dcf77_axi_lite_tb.vhd
--
-- VUnit testbench for the dcf77_axi_lite top-level wrapper.  A small BFM
-- of AXI4-Lite read / write procedures drives the slave and checks the
-- decoded responses.  The ADC stream is held silent so the core stays in
-- reset state and STATUS/TIME registers read back as zero.
-- =========================================================================

library vunit_lib;
context vunit_lib.vunit_context;

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use ieee.fixed_pkg.all;

library dcf77;
use dcf77.dcf77_pkg.all;
use dcf77.axi4lite_interface_pkg.all;

entity dcf77_axi_lite_tb is
    generic (runner_cfg : string);
end entity;

architecture tb of dcf77_axi_lite_tb is

    constant CLK_PERIOD : time := 8 ns;

    signal aclk    : std_logic := '0';
    signal aresetn : std_logic := '0';

    signal awaddr  : std_logic_vector(7 downto 0) := (others => '0');
    signal awprot  : std_logic_vector(2 downto 0) := (others => '0');
    signal awvalid : std_logic := '0';
    signal awready : std_logic;
    signal wdata   : std_logic_vector(31 downto 0) := (others => '0');
    signal wstrb   : std_logic_vector(3 downto 0) := "1111";
    signal wvalid  : std_logic := '0';
    signal wready  : std_logic;
    signal bresp   : std_logic_vector(1 downto 0);
    signal bvalid  : std_logic;
    signal bready  : std_logic := '0';

    signal araddr  : std_logic_vector(7 downto 0) := (others => '0');
    signal arprot  : std_logic_vector(2 downto 0) := (others => '0');
    signal arvalid : std_logic := '0';
    signal arready : std_logic;
    signal rdata   : std_logic_vector(31 downto 0);
    signal rresp   : std_logic_vector(1 downto 0);
    signal rvalid  : std_logic;
    signal rready  : std_logic := '0';

    signal adc_sample : adc_sample_t := (others => '0');
    signal adc_valid  : std_logic := '0';

    signal irq        : std_logic;

    -- Record bundles for dcf77_axi_lite's record ports.  The flat BFM
    -- signals above are aggregated into / split out of these, so the AXI
    -- BFM and assertions below stay unchanged.
    signal axi_in  : axi4lite_si_type;
    signal axi_out : axi4lite_so_type;
    signal dcf_in  : dcf77_in_type;

begin

    aclk <= not aclk after CLK_PERIOD / 2;

    -- Aggregate the flat BFM drive signals into the master->slave record.
    axi_in.aw.valid <= awvalid;
    axi_in.aw.addr  <= awaddr;
    axi_in.aw.prot  <= awprot;
    axi_in.w.valid  <= wvalid;
    axi_in.w.data   <= wdata;
    axi_in.w.strb   <= wstrb;
    axi_in.b.ready  <= bready;
    axi_in.ar.valid <= arvalid;
    axi_in.ar.addr  <= araddr;
    axi_in.ar.prot  <= arprot;
    axi_in.r.ready  <= rready;

    -- Split the slave->master record back out for the BFM to observe.
    awready <= axi_out.aw.ready;
    wready  <= axi_out.w.ready;
    bvalid  <= axi_out.b.valid;
    bresp   <= axi_out.b.resp;
    arready <= axi_out.ar.ready;
    rvalid  <= axi_out.r.valid;
    rdata   <= axi_out.r.data;
    rresp   <= axi_out.r.resp;

    -- ADC stream into the input record (adc_sample is already sfixed).
    dcf_in.sample       <= adc_sample;
    dcf_in.sample_valid <= adc_valid;

    dut : entity dcf77.dcf77_axi_lite
        generic map (
            WITH_PM_DEBUG => true
        )
        port map (
            clk     => aclk,
            rst_n   => aresetn,
            axi_in  => axi_in,
            axi_out => axi_out,
            dcf_in  => dcf_in,
            irq     => irq
        );

    test_runner : process

        procedure wait_clk(n : natural := 1) is
        begin
            for i in 1 to n loop
                wait until rising_edge(aclk);
            end loop;
        end procedure;

        procedure axi_write(addr : std_logic_vector(7 downto 0);
                            data : std_logic_vector(31 downto 0);
                            expect_resp : std_logic_vector(1 downto 0)) is
        begin
            -- Drive AW and W in parallel.
            awaddr  <= addr;
            wdata   <= data;
            wstrb   <= "1111";
            awvalid <= '1';
            wvalid  <= '1';
            bready  <= '1';

            -- Wait for both handshakes.
            loop
                wait_clk;
                exit when awready = '1';
            end loop;
            awvalid <= '0';

            loop
                exit when wready = '1';
                wait_clk;
            end loop;
            wvalid  <= '0';

            loop
                wait_clk;
                exit when bvalid = '1';
            end loop;
            check_equal(bresp, expect_resp,
                        "AXI write response at 0x" & to_hstring(addr));
            wait_clk;
            bready <= '0';
        end procedure;

        procedure axi_read(addr : std_logic_vector(7 downto 0);
                           data : out std_logic_vector(31 downto 0);
                           expect_resp : std_logic_vector(1 downto 0)) is
        begin
            araddr  <= addr;
            arvalid <= '1';
            rready  <= '1';
            loop
                wait_clk;
                exit when arready = '1';
            end loop;
            arvalid <= '0';

            loop
                wait_clk;
                exit when rvalid = '1';
            end loop;
            data := rdata;
            check_equal(rresp, expect_resp,
                        "AXI read response at 0x" & to_hstring(addr));
            wait_clk;
            rready <= '0';
        end procedure;

        -- Drive a 77.5 kHz square wave on adc_sample for n_clocks cycles.
        -- 125 MHz / 1612 cycles = 77 544 Hz, ~0.06 % off the LO frequency,
        -- which is well within the post-mixer CIC passband.  The chain
        -- band-pass-filters this to a strong DC component on the I/Q
        -- channels, so env_max climbs in bit_det.  Constant-DC stimulus
        -- (which the old tests used) is correctly rejected by the
        -- properly-pruned Hogenauer CIC + sign-LO mixer combo, so RSSI
        -- never moves with DC -- hence this tone.
        procedure drive_tone(n_clocks : natural) is
            variable tone_phase : integer range 0 to 1611 := 0;
        begin
            adc_valid <= '1';
            for i in 1 to n_clocks loop
                if tone_phase < 806 then
                    adc_sample <= to_sfixed(4000, adc_sample_t'high,
                                            adc_sample_t'low);
                else
                    adc_sample <= to_sfixed(-4000, adc_sample_t'high,
                                            adc_sample_t'low);
                end if;
                if tone_phase = 1611 then
                    tone_phase := 0;
                else
                    tone_phase := tone_phase + 1;
                end if;
                wait_clk;
            end loop;
        end procedure;

        variable rd : std_logic_vector(31 downto 0);

    begin
        test_runner_setup(runner, runner_cfg);

        aresetn <= '0';
        wait_clk(8);
        aresetn <= '1';
        wait_clk(8);

        while test_suite loop

            -----------------------------------------------------------------
            if run("ctrl_roundtrip") then
                -- Default CTRL after reset has enable='1', soft_reset='0'.
                axi_read(x"00", rd, AXI_RESPONSE_OKAY);
                check_equal(rd(0), '1', "CTRL.enable defaults to '1'");
                check_equal(rd(1), '0', "CTRL.soft_reset defaults to '0'");

                -- enable is persistent; soft_reset is a one-cycle pulse, so
                -- after a write of bit1='1', the read-back must show '0'
                -- because by then the bit has self-cleared.
                axi_write(x"00", x"00000003", AXI_RESPONSE_OKAY);
                axi_read(x"00", rd, AXI_RESPONSE_OKAY);
                check_equal(rd(0), '1', "enable bit reads back '1'");
                check_equal(rd(1), '0',
                            "soft_reset cleared after one-cycle pulse");

                -- Write enable=0 (no soft_reset bit).  Enable stays sticky.
                axi_write(x"00", x"00000000", AXI_RESPONSE_OKAY);
                axi_read(x"00", rd, AXI_RESPONSE_OKAY);
                check_equal(rd(0), '0', "enable=0 reads back sticky");
                check_equal(rd(1), '0', "soft_reset stays cleared");

                -- Re-enable for subsequent tests.
                axi_write(x"00", x"00000001", AXI_RESPONSE_OKAY);

            -----------------------------------------------------------------
            elsif run("status_default") then
                -- With the ADC silent and no minute mark seen, STATUS reads 0.
                axi_read(x"04", rd, AXI_RESPONSE_OKAY);
                check_equal(rd(1 downto 0),
                            std_logic_vector'("00"),
                            "sync_state = SEARCH");
                check_equal(rd(2), '0', "time_valid = 0");

            -----------------------------------------------------------------
            elsif run("date_default") then
                axi_read(x"0C", rd, AXI_RESPONSE_OKAY);
                check_equal(rd, std_logic_vector'(x"00000000"),
                            "DATE reads 0 before any decode");

            -----------------------------------------------------------------
            elsif run("write_readonly_slverr") then
                -- Writing to STATUS (read-only) returns SLVERR.
                axi_write(x"04", x"DEADBEEF", AXI_RESPONSE_SLVERR);

            -----------------------------------------------------------------
            elsif run("read_unmapped_decerr") then
                -- 0x64 is past the last mapped register (REG_SOFT_AM_SHADOW
                -- at 0x60) -> DECERR.
                axi_read(x"64", rd, AXI_RESPONSE_DECERR);

            -----------------------------------------------------------------
            elsif run("pm_registers_reset") then
                -- After reset, every PM-side register reads 0.  Verifies the
                -- wires from the new PM-side status_out fields through
                -- axi_readmux.  pm_valid='0' and pm_minute_mark='0' in
                -- SOFT_PM mean bits 30 and 31 are zero, so the whole word
                -- is zero.
                axi_read(x"30", rd, AXI_RESPONSE_OKAY);
                check_equal(rd, std_logic_vector'(x"00000000"),
                            "SOFT_PM reads 0 at reset");
                -- PM_STATUS at reset has pm_bit_index = 15 (= START_INDEX
                -- of the PM-side frame_dec) → bits [5:0] = 0x0F.  Other
                -- fields are zero, so the full word is 0x0000_000F.
                axi_read(x"38", rd, AXI_RESPONSE_OKAY);
                check_equal(rd, std_logic_vector'(x"0000000F"),
                            "PM_STATUS reads pm_bit_index=15 at reset");
                axi_read(x"3C", rd, AXI_RESPONSE_OKAY);
                check_equal(rd, std_logic_vector'(x"00000000"),
                            "PM_TIME_HMS reads 0 at reset");
                axi_read(x"40", rd, AXI_RESPONSE_OKAY);
                check_equal(rd, std_logic_vector'(x"00000000"),
                            "PM_DATE reads 0 at reset");
                axi_read(x"44", rd, AXI_RESPONSE_DECERR);
                axi_read(x"48", rd, AXI_RESPONSE_OKAY);
                check_equal(rd, std_logic_vector'(x"00000000"),
                            "PM_LAST_FRAME0 reads 0 at reset");
                axi_read(x"4C", rd, AXI_RESPONSE_OKAY);
                check_equal(rd, std_logic_vector'(x"00000000"),
                            "PM_LAST_FRAME1 reads 0 at reset");

            -----------------------------------------------------------------
            elsif run("last_frame_reset") then
                -- After reset, last_frame is all zeros throughout
                -- dcf77_frame_dec, so both LAST_FRAME registers read 0.
                -- This implicitly verifies the wire from frame_dec ->
                -- dcf77_core.status_out.last_frame -> AXI register.
                axi_read(x"18", rd, AXI_RESPONSE_OKAY);
                check_equal(rd, std_logic_vector'(x"00000000"),
                            "LAST_FRAME0 reads 0 at reset");
                axi_read(x"1C", rd, AXI_RESPONSE_OKAY);
                check_equal(rd, std_logic_vector'(x"00000000"),
                            "LAST_FRAME1 reads 0 at reset");

            -----------------------------------------------------------------
            elsif run("soft_reset_pulse_resets_core") then
                -- Drive a 77.5 kHz tone, let RSSI climb, fire soft_reset,
                -- verify RSSI is zero again (i.e. the core saw a reset
                -- pulse).  env_dec_v fires every 1.25M clocks (after the
                -- new CIC_ENV 1 kS/s -> 100 S/s decimator), so 2.5M
                -- clocks gives ~2 env_dec_v pulses -- enough for the
                -- env_max IIR (K_ATK=3) to come well off zero given the
                -- huge CIC gain on a 77.5 kHz tone.
                drive_tone(10_000_000);
                axi_read(x"04", rd, AXI_RESPONSE_OKAY);
                check(rd(31 downto 16) /= std_logic_vector'(x"0000"),
                      "RSSI rose before soft_reset");

                -- Write CTRL with soft_reset=1, enable=1.  Pulse should be
                -- one cycle wide; core resets synchronously.
                axi_write(x"00", x"00000003", AXI_RESPONSE_OKAY);
                wait_clk(8);
                axi_read(x"04", rd, AXI_RESPONSE_OKAY);
                check_equal(rd(31 downto 16),
                            std_logic_vector'(x"0000"),
                            "RSSI cleared after soft_reset pulse");

                adc_valid <= '0';

            -----------------------------------------------------------------
            elsif run("enable_gates_adc") then
                -- Drive a strong 77.5 kHz tone with enable=0 and verify
                -- the decoder doesn't track: env_max (STATUS bits 31..16)
                -- stays at zero.  Then enable and confirm env_max climbs
                -- above zero with the same tone.  See drive_tone() for
                -- why DC stimulus does not work under the properly-pruned
                -- Hogenauer CIC chain.
                axi_write(x"00", x"00000000", AXI_RESPONSE_OKAY);  -- enable=0
                drive_tone(10_000_000);
                axi_read(x"04", rd, AXI_RESPONSE_OKAY);
                check_equal(rd(31 downto 16),
                            std_logic_vector'(x"0000"),
                            "RSSI stays zero while CTRL.enable=0");

                axi_write(x"00", x"00000001", AXI_RESPONSE_OKAY);  -- enable=1
                drive_tone(10_000_000);
                axi_read(x"04", rd, AXI_RESPONSE_OKAY);
                check(rd(31 downto 16) /= std_logic_vector'(x"0000"),
                      "RSSI rises once CTRL.enable goes high");

                adc_valid <= '0';

            -----------------------------------------------------------------
            elsif run("irq_registers_reset") then
                -- IRQ_FLAG and IRQ_MASK both read 0 at reset; the irq
                -- output is correspondingly low.
                axi_read(x"34", rd, AXI_RESPONSE_OKAY);
                check_equal(rd, std_logic_vector'(x"00000000"),
                            "IRQ_FLAG reads 0 at reset");
                axi_read(x"5C", rd, AXI_RESPONSE_OKAY);
                check_equal(rd, std_logic_vector'(x"00000000"),
                            "IRQ_MASK reads 0 at reset");
                check_equal(irq, '0',
                            "irq output stays low at reset (all masked)");

            -----------------------------------------------------------------
            elsif run("irq_mask_roundtrip") then
                -- IRQ_MASK is R/W for the nine defined source bits;
                -- reserved bits 31..9 read as 0 (the mask is stored as
                -- a dcf77_irq_source_type record -- undefined positions
                -- have no storage).  The irq output must not move while
                -- no flags are set, regardless of mask value.
                axi_write(x"5C", x"DEADBEEF", AXI_RESPONSE_OKAY);
                axi_read(x"5C", rd, AXI_RESPONSE_OKAY);
                check_equal(rd, std_logic_vector'(x"000000EF"),
                            "IRQ_MASK round-trips DEADBEEF & 0x1FF");
                check_equal(irq, '0',
                            "irq stays low while flag = 0 (mask irrelevant)");
                axi_write(x"5C", x"FFFFFFFF", AXI_RESPONSE_OKAY);
                axi_read(x"5C", rd, AXI_RESPONSE_OKAY);
                check_equal(rd, std_logic_vector'(x"000001FF"),
                            "IRQ_MASK all-ones reads back as the 9 defined bits");
                axi_write(x"5C", x"00000000", AXI_RESPONSE_OKAY);
                axi_read(x"5C", rd, AXI_RESPONSE_OKAY);
                check_equal(rd, std_logic_vector'(x"00000000"),
                            "IRQ_MASK cleared back to 0");

            -----------------------------------------------------------------
            elsif run("shadow_reset_and_triggers") then
                -- SHADOW at reset reads 0 (never latched).
                axi_read(x"60", rd, AXI_RESPONSE_OKAY);
                check_equal(rd, std_logic_vector'(x"00000000"),
                            "SHADOW reads 0 at reset");
                -- All triggers below produce a 0 shadow because the
                -- status record is at its reset value (everything zero).
                -- That's fine -- the test is about the latch firing,
                -- not the latched value.  Full-stimulus content checks
                -- live in the 10_core / 07_replay TBs.
                axi_read(x"30", rd, AXI_RESPONSE_OKAY);    -- SOFT_PM read
                axi_read(x"60", rd, AXI_RESPONSE_OKAY);
                check_equal(rd, std_logic_vector'(x"00000000"),
                            "SHADOW reflects SOFT_AM at PM-read time");
                axi_read(x"08", rd, AXI_RESPONSE_OKAY);    -- TIME_HMS read
                axi_read(x"60", rd, AXI_RESPONSE_OKAY);
                check_equal(rd, std_logic_vector'(x"00000000"),
                            "SHADOW reflects DATE at TIME-read time");
                axi_read(x"3C", rd, AXI_RESPONSE_OKAY);    -- PM_TIME_HMS read
                axi_read(x"60", rd, AXI_RESPONSE_OKAY);
                check_equal(rd, std_logic_vector'(x"00000000"),
                            "SHADOW reflects PM_DATE at PM_TIME-read time");
                -- Non-trigger reads leave the shadow unchanged.  Sanity:
                -- read STATUS (not a trigger), then SHADOW, expect 0.
                axi_read(x"04", rd, AXI_RESPONSE_OKAY);
                axi_read(x"60", rd, AXI_RESPONSE_OKAY);
                check_equal(rd, std_logic_vector'(x"00000000"),
                            "SHADOW unchanged by non-trigger read");

            -----------------------------------------------------------------
            elsif run("flags_register_decerr") then
                -- The former REG_FLAGS at 0x10 and REG_PM_FLAGS at 0x44
                -- are now unmapped (flags packed into DATE / PM_DATE).
                axi_read(x"10", rd, AXI_RESPONSE_DECERR);
                axi_read(x"44", rd, AXI_RESPONSE_DECERR);

            -----------------------------------------------------------------
            elsif run("irq_flag_w1c_no_effect_at_reset") then
                -- Writing W1C bits when no flag is set must leave both
                -- IRQ_FLAG and irq unchanged.  Verifies the readmux
                -- accepts writes to 0x34 (no SLVERR) and the clear path
                -- doesn't accidentally promote any bits.
                axi_write(x"34", x"FFFFFFFF", AXI_RESPONSE_OKAY);
                axi_read(x"34", rd, AXI_RESPONSE_OKAY);
                check_equal(rd, std_logic_vector'(x"00000000"),
                            "IRQ_FLAG stays 0 after spurious W1C");
                check_equal(irq, '0', "irq stays low");

            -----------------------------------------------------------------
            -- End-to-end edge-detect → sticky-flag → irq behaviour is
            -- exercised by the higher-level 10_/07_/14_/15_ TBs where a
            -- real signal chain produces real source edges.  Adding it
            -- here would require multi-second drive_tone() calls, which
            -- would balloon the 05_tb runtime.

            end if;
        end loop;

        test_runner_cleanup(runner);
        wait;
    end process;

    -- Some tests need ~10 M clocks (80 ms) per drive_tone() call for the
    -- env CIC + env_max IIR to climb measurably; enable_gates_adc runs
    -- two such drives, so allow generous margin.
    test_runner_watchdog(runner, 500 ms);

end architecture;
