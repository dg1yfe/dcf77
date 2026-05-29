-- =========================================================================
-- dcf77_pm_correlator.vhd
--
-- Self-synchronizing DCF77 PM (PSK/SIK) correlator.  Consumes the
-- carrier-PLL's phase_err stream at 2 kS/s and emits one signed pm_soft
-- per second (SIK bit LLR, sign convention: positive = SIK '0' =
-- broadcast bit '0' favoured).
--
-- Single-LFSR + single-ALU design: a 2048×18 BRAM ring buffer holds the
-- last ≈1 s of phase_err; a 9-bit LFSR (taps 5 & 9, init "000000001"
-- post-Hetzel-force-1) drives the chip sign sequence through a 7-bit
-- Bresenham mod-96 chip clock (exact 31/96 of fs); a single 29-b signed
-- accumulator is time-shared across all hypotheses.
--
-- Two-phase synchronization:
--   ACQUIRE: 4 evaluation bursts per second (every 500 sample ticks),
--     K=463 hypotheses per burst, running-max for the best.  Once any
--     second's best score exceeds LOCK_IN_THR the broadcast's chip-0
--     phase is locked in as `locked_phase`.
--   LOCK:    3 hypotheses per second (locked_phase −1 / center / +1)
--     for early-late DLL tracking.  pm_soft is latched from the center
--     hypothesis's signed accumulator; locked_phase is adjusted by ±1
--     based on the running-max winner across all three.
--   LOCK loss: K_LOSS consecutive seconds with |pm_soft| below
--     LOSS_THR → back to ACQUIRE.
-- =========================================================================

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

use work.dcf77_pkg.all;

entity dcf77_pm_correlator is
    generic (
        -- Right-shift before saturating to 12-b pm_soft output.
        -- With 9-b phase input, per-sample PSK signal is ±22 LSB
        -- (= ±15.6° in 2π=2^9 scale).  Σ over 1586 samples maxes at
        -- ±35 K (~16 b); shift 4 brings the peak into the 12-b range
        -- with light saturation on max-amplitude alignment.
        PM_LLR_SHIFT      : natural := 4;
        -- |acc| threshold to confirm an ACQUIRE lock.  At the expected
        -- ±35 K signal peak, 16 K ≈ 46 % of peak — well above the
        -- random-correlation noise floor of √1586 · 148 ≈ 5.9 K.
        LOCK_IN_THR       : natural := 16384;
        -- |pm_soft| threshold below which a TRACK second is "weak".
        LOSS_THR          : natural := 8;
        -- Consecutive weak seconds in TRACK before falling back to ACQUIRE.
        K_LOSS            : natural := 5
    );
    port (
        clk              : in  std_logic;
        rst              : in  std_logic;

        -- Input: 9-b signed phase stream (= top 9 b of CORDIC's 12-b
        -- output).  Amplitude-independent; 2π = 2^9, so 1 LSB ≈ 0.7°.
        -- Per-sample quantisation noise (σ = 0.29 LSB) sits ~36 dB
        -- below the matched filter's false-correlation noise floor
        -- and far below typical post-PLL phase noise on DCF77.
        phase_err        : in  signed(8 downto 0);
        phase_err_v      : in  std_logic;

        -- Held high while the carrier PLL is locked.  Buffer fills and
        -- ACQUIRE / LOCK only run while this is asserted; otherwise the
        -- correlator stays in M_FILL with fill_count = 0 (the
        -- pre-PLL-lock phase signal is free-running and meaningless to
        -- the matched filter).  When this drops mid-LOCK, the state
        -- machine falls back to M_FILL.
        pll_locked       : in  std_logic := '1';

        -- Outputs (registered).
        pm_soft          : out signed(11 downto 0);
        pm_valid         : out std_logic;          -- 1-cycle pulse per locked second
        pm_minute_mark_o : out std_logic;          -- = pm_valid (signature work in frame_dec)
        pm_sync_state    : out std_logic_vector(1 downto 0);  -- "00"=FILL "01"=ACQ "10"=LOCK
        pm_locked_phase  : out unsigned(11 downto 0);
        pm_lock_qual     : out unsigned(15 downto 0)
    );
end entity dcf77_pm_correlator;

architecture rtl of dcf77_pm_correlator is

    ----------------------------------------------------------------------
    -- Constants
    ----------------------------------------------------------------------
    constant BUF_SIZE         : natural := 4096;
    constant SAMPLES_PER_PRBS : natural := 1586;
    constant SAMPLES_PER_SEC  : natural := 2000;
    -- K = BUF_SIZE - SAMPLES_PER_PRBS: every chip-0 candidate whose
    -- 1586-sample integration window fits non-wrapped inside the buffer
    -- is tested in a SINGLE burst.  No coverage gaps.
    constant K_HYPOTHESES     : natural := 2510;
    constant BURSTS_PER_CYCLE : natural := 1;
    -- One burst per natural-wrap cycle (= 2.048 broadcast-seconds at
    -- 4096 / 2000).  Burst triggers at sec_phase = 0.
    -- K=2510 hypotheses × ~3175 clk ≈ 7.97 M clk ≈ 128 sample-ticks of
    -- compute per cycle.  Plenty of slack inside the 4096-tick cycle.
    constant SAMPLES_PER_BURST_SLOT : natural := BUF_SIZE;
    constant CHIP_RATE_NUM    : natural := 31;
    constant CHIP_RATE_DEN    : natural := 96;
    -- LFSR init.  v1 uses pure m-sequence (no Hetzel force-1): init to
    -- "000000001" and run continuously.  This is offset by 1 chip from the
    -- broadcast's Hetzel sequence (which prepends a '0' chip from the
    -- all-zero state), giving ~0.4 % SNR loss but consistent self-
    -- correlation.  v2 may add the force-1 start-FF for exact Hetzel
    -- timing alignment.
    constant LFSR_INIT        : std_logic_vector(8 downto 0) := "000000001";
    -- Buffer holds 9-b signed phase (2π = 2^9, 1 LSB ≈ 0.7°).
    -- 4096 × 9 b = 36.9 kbit → fits in a single Xilinx 36-kbit BRAM tile.
    constant BUF_DATA_W       : natural := 9;
    constant PHASE_ERR_W      : natural := 9;

    constant SYNC_STATE_FILL    : std_logic_vector(1 downto 0) := "00";
    constant SYNC_STATE_ACQUIRE : std_logic_vector(1 downto 0) := "01";
    constant SYNC_STATE_LOCK    : std_logic_vector(1 downto 0) := "10";

    ----------------------------------------------------------------------
    -- BRAM: 2048 × 18-bit, separate sync read / sync write
    ----------------------------------------------------------------------
    type ram_t is array (0 to BUF_SIZE - 1) of std_logic_vector(BUF_DATA_W - 1 downto 0);
    signal buf      : ram_t := (others => (others => '0'));

    -- Registered read port: address registered combinationally as ram_raddr,
    -- data appears one clock later in ram_rdata_r.
    signal ram_raddr : unsigned(11 downto 0);
    signal ram_rdata_r : std_logic_vector(BUF_DATA_W - 1 downto 0);

    ----------------------------------------------------------------------
    -- FSM types
    ----------------------------------------------------------------------
    type mode_t       is (M_FILL, M_ACQUIRE, M_LOCK);
    type eng_state_t  is (E_IDLE, E_REQ, E_RECV, E_DONE);
    type track_step_t is (T_MINUS, T_CENTER, T_PLUS);

    ----------------------------------------------------------------------
    -- Register record
    ----------------------------------------------------------------------
    type reg_type is record
        ---- Sample-buffer writer ----
        w_ptr            : unsigned(11 downto 0);     -- 0..4095 (= BUF_SIZE-1)
        fill_count       : unsigned(12 downto 0);     -- 0..4096 (saturating)

        ---- Per-second timing ----
        sec_phase        : unsigned(11 downto 0);     -- natural 12-b wrap @ 4096

        ---- Outer FSM ----
        mode             : mode_t;

        ---- ACQUIRE bookkeeping ----
        ac_burst_armed   : std_logic;                 -- '1' = burst pending
        ac_burst_count   : unsigned(0 downto 0);      -- 0..1 (one burst per cycle)
        ac_hyp_idx       : unsigned(11 downto 0);     -- 0..2509
        ac_best_score    : unsigned(18 downto 0);     -- |acc| running max (19-b)
        ac_best_phase    : unsigned(11 downto 0);     -- sec_phase at chip-0
        ac_w_ptr_snap    : unsigned(11 downto 0);     -- w_ptr at start of burst
        ac_sec_phase_snap: unsigned(11 downto 0);     -- sec_phase at start of burst

        ---- LOCK / TRACK ----
        locked_phase     : unsigned(11 downto 0);     -- sec_phase at chip-0 arrival
        target_phase     : unsigned(11 downto 0);     -- sec_phase to dispatch TRACK
                                                      -- (advanced by +2000 each
                                                      -- broadcast-second; natural
                                                      -- 12-b wrap absorbs the
                                                      -- 96-sample slip)
        track_step       : track_step_t;
        track_armed      : std_logic;
        track_best_idx   : unsigned(1 downto 0);      -- 0=undecided 1=- 2=center 3=+
        track_best_score : unsigned(18 downto 0);
        track_center_acc : signed(19 downto 0);       -- 20-b signed acc snapshot
        loss_count       : unsigned(3 downto 0);
        pm_lock_qual     : unsigned(15 downto 0);

        ---- Engine state (shared across ACQUIRE and TRACK) ----
        eng_state        : eng_state_t;
        eng_base_ptr     : unsigned(11 downto 0);     -- (w_ptr_snap + offset) mod 4096
        sample_idx       : unsigned(10 downto 0);     -- 0..1585
        acc              : signed(19 downto 0);       -- 20-b signed (max 1586·256 ≈ 406K)
        chip_acc         : unsigned(6 downto 0);
        lfsr             : std_logic_vector(8 downto 0);

        ---- Outputs ----
        pm_soft          : signed(11 downto 0);
        pm_valid         : std_logic;
        pm_minute_mark   : std_logic;
        sync_state_o     : std_logic_vector(1 downto 0);
    end record;

    constant R_RESET : reg_type := (
        w_ptr            => (others => '0'),
        fill_count       => (others => '0'),
        sec_phase        => (others => '0'),
        mode             => M_FILL,
        ac_burst_armed   => '0',
        ac_burst_count   => (others => '0'),
        ac_hyp_idx       => (others => '0'),
        ac_best_score    => (others => '0'),
        ac_best_phase    => (others => '0'),
        ac_w_ptr_snap    => (others => '0'),
        ac_sec_phase_snap=> (others => '0'),
        locked_phase     => (others => '0'),
        target_phase     => (others => '0'),
        track_step       => T_MINUS,
        track_armed      => '0',
        track_best_idx   => (others => '0'),
        track_best_score => (others => '0'),
        track_center_acc => (others => '0'),
        loss_count       => (others => '0'),
        pm_lock_qual     => (others => '0'),
        eng_state        => E_IDLE,
        eng_base_ptr     => (others => '0'),
        sample_idx       => (others => '0'),
        acc              => (others => '0'),
        chip_acc         => (others => '0'),
        lfsr             => LFSR_INIT,
        pm_soft          => (others => '0'),
        pm_valid         => '0',
        pm_minute_mark   => '0',
        sync_state_o     => SYNC_STATE_FILL
    );

    signal r, rin : reg_type;

    -- |x| for a signed 20-b value, returned as unsigned(18 downto 0).
    function abs20(x : signed(19 downto 0)) return unsigned is
        variable a : signed(19 downto 0);
    begin
        if x(19) = '1' then
            a := -x;
        else
            a := x;
        end if;
        return unsigned(std_logic_vector(a(18 downto 0)));
    end function;

    -- Saturating right-shift to 12-b signed for pm_soft output.
    function sat12_shr(x : signed(19 downto 0); shr : natural) return signed is
        variable shifted : signed(19 downto 0);
        constant POS_MAX : signed(19 downto 0) := to_signed( 2047, 20);
        constant NEG_MAX : signed(19 downto 0) := to_signed(-2048, 20);
    begin
        shifted := shift_right(x, shr);
        if shifted > POS_MAX then
            return to_signed(2047, 12);
        elsif shifted < NEG_MAX then
            return to_signed(-2048, 12);
        else
            return resize(shifted, 12);
        end if;
    end function;

begin

    pm_soft          <= r.pm_soft;
    pm_valid         <= r.pm_valid;
    pm_minute_mark_o <= r.pm_minute_mark;
    pm_sync_state    <= r.sync_state_o;
    pm_locked_phase  <= r.locked_phase;
    pm_lock_qual     <= r.pm_lock_qual;

    ----------------------------------------------------------------------
    -- BRAM read/write process.  Synchronous: write on phase_err_v,
    -- read every clock; data appears in ram_rdata_r one cycle later.
    ----------------------------------------------------------------------
    ram_proc : process (clk)
    begin
        if rising_edge(clk) then
            if phase_err_v = '1' and pll_locked = '1' then
                buf(to_integer(r.w_ptr))
                    <= std_logic_vector(phase_err(PHASE_ERR_W - 1 downto
                                                   PHASE_ERR_W - BUF_DATA_W));
            end if;
            ram_rdata_r <= buf(to_integer(ram_raddr));
        end if;
    end process;

    ----------------------------------------------------------------------
    -- Combinational FSM
    ----------------------------------------------------------------------
    comb : process (all)
        variable v             : reg_type;
        variable chip_sign     : std_logic;
        variable sample_in     : signed(BUF_DATA_W - 1 downto 0);
        variable new_lfsr_bit  : std_logic;
        variable chip_acc_next : unsigned(7 downto 0);
        variable burst_offset  : unsigned(11 downto 0);
        variable abs_acc       : unsigned(18 downto 0);
        variable engine_done   : boolean;
        variable start_engine  : boolean;
        variable next_offset   : unsigned(11 downto 0);
    begin
        v := r;
        v.pm_valid       := '0';
        v.pm_minute_mark := '0';
        engine_done      := false;
        start_engine     := false;
        next_offset      := (others => '0');

        ----------------------------------------------------------------
        -- PLL-lock gate: if the carrier PLL isn't locked, the phase
        -- input is meaningless to the matched filter.  Stay in M_FILL
        -- with fill_count = 0 and don't advance any state.  When the
        -- PLL locks, we start fresh.  This avoids polluting the buffer
        -- with free-running phase samples.
        ----------------------------------------------------------------
        if pll_locked = '0' then
            v.mode           := M_FILL;
            v.fill_count     := (others => '0');
            v.w_ptr          := (others => '0');
            v.sec_phase      := (others => '0');
            v.ac_burst_armed := '0';
            v.ac_burst_count := (others => '0');
            v.ac_best_score  := (others => '0');
            v.track_armed    := '0';
            v.eng_state      := E_IDLE;
            v.sync_state_o   := SYNC_STATE_FILL;
        else
            ------------------------------------------------------------
            -- Sample-buffer write pointer update (on each phase_err_v)
            ------------------------------------------------------------
            if phase_err_v = '1' then
                if r.w_ptr = BUF_SIZE - 1 then
                    v.w_ptr := (others => '0');
                else
                    v.w_ptr := r.w_ptr + 1;
                end if;
                if r.fill_count < BUF_SIZE then
                    v.fill_count := r.fill_count + 1;
                end if;

                -- sec_phase wraps naturally at 2048 (= 1.024 broadcast seconds).
                -- The 48-sample slip vs the 2000-sample broadcast period is
                -- absorbed into the target_phase += SAMPLES_PER_SEC advance below.
                v.sec_phase := r.sec_phase + 1;
            end if;
        end if;

        ----------------------------------------------------------------
        -- Engine (single ALU, shared across ACQUIRE and TRACK)
        --
        -- Per hypothesis, walks 1586 samples reading from
        -- buf[(eng_base_ptr + sample_idx) mod 2048], driving chip_sign
        -- from the LFSR's stage 1, Bresenham-advancing the chip clock.
        ----------------------------------------------------------------
        case r.eng_state is
            when E_IDLE =>
                -- Wait for a "start" command from the outer FSM.
                null;

            when E_REQ =>
                -- Address is presented to BRAM combinationally via the
                -- ram_raddr signal below.  Move to E_RECV next cycle.
                v.eng_state := E_RECV;

            when E_RECV =>
                -- ram_rdata_r now holds buf[(eng_base_ptr + sample_idx)].
                sample_in := signed(ram_rdata_r);
                chip_sign := r.lfsr(0);     -- '0' chip → +1, '1' chip → -1
                if chip_sign = '0' then
                    v.acc := r.acc + resize(sample_in, 20);
                else
                    v.acc := r.acc - resize(sample_in, 20);
                end if;

                -- Bresenham chip advance: chip_acc += 31; if ≥ 96, sub 96
                -- and shift the LFSR.
                chip_acc_next := resize(r.chip_acc, 8) + to_unsigned(CHIP_RATE_NUM, 8);
                if chip_acc_next >= to_unsigned(CHIP_RATE_DEN, 8) then
                    v.chip_acc := resize(chip_acc_next - to_unsigned(CHIP_RATE_DEN, 8), 7);
                    new_lfsr_bit := r.lfsr(4) xor r.lfsr(8);
                    v.lfsr := r.lfsr(7 downto 0) & new_lfsr_bit;
                else
                    v.chip_acc := resize(chip_acc_next, 7);
                end if;

                if r.sample_idx = SAMPLES_PER_PRBS - 1 then
                    v.eng_state := E_DONE;
                else
                    v.sample_idx := r.sample_idx + 1;
                    v.eng_state := E_REQ;
                end if;

            when E_DONE =>
                engine_done := true;
                v.eng_state := E_IDLE;
        end case;

        ----------------------------------------------------------------
        -- Outer FSM
        ----------------------------------------------------------------
        case r.mode is
            when M_FILL =>
                v.sync_state_o := SYNC_STATE_FILL;
                if r.fill_count >= BUF_SIZE then
                    v.mode := M_ACQUIRE;
                    v.ac_burst_count := (others => '0');
                    v.ac_best_score  := (others => '0');
                    v.sync_state_o := SYNC_STATE_ACQUIRE;
                end if;

            when M_ACQUIRE =>
                v.sync_state_o := SYNC_STATE_ACQUIRE;

                -- Trigger a burst every SAMPLES_PER_BURST_SLOT samples
                -- (0, 500, 1000, 1500 within the second).
                -- Single burst per natural-wrap cycle, triggered when
                -- sec_phase = 0.  K = 2510 hypotheses cover every chip-0
                -- candidate whose 1586-sample integration window fits
                -- non-wrapped inside the 4096-sample buffer.
                if phase_err_v = '1'
                   and r.eng_state = E_IDLE
                   and r.ac_burst_armed = '0'
                   and r.ac_burst_count < BURSTS_PER_CYCLE
                   and r.sec_phase = 0
                then
                    v.ac_burst_armed    := '1';
                    v.ac_hyp_idx        := (others => '0');
                    v.ac_w_ptr_snap     := r.w_ptr;
                    v.ac_sec_phase_snap := r.sec_phase;
                end if;

                if r.ac_burst_armed = '1' and r.eng_state = E_IDLE then
                    start_engine := true;
                    -- Hypothesis k: integration window starts at
                    -- buf[(w_ptr_snap + k) mod BUF_SIZE].  Engine base ptr
                    -- = w_ptr_snap + ac_hyp_idx (mod BUF_SIZE via 12-b resize).
                    next_offset := resize(r.ac_w_ptr_snap + r.ac_hyp_idx, 12);
                end if;

                if engine_done then
                    abs_acc := abs20(v.acc);
                    if abs_acc > r.ac_best_score then
                        v.ac_best_score := abs_acc;
                        -- chip-0 was at buf[(w_ptr_snap + k) mod BUF_SIZE] = the
                        -- sample written (BUF_SIZE - k) sample ticks before
                        -- burst start.  At that moment sec_phase was
                        -- (sec_phase_snap - (BUF_SIZE - k)) mod BUF_SIZE
                        --   = (sec_phase_snap + k - BUF_SIZE) mod BUF_SIZE
                        --   = (sec_phase_snap + k)            mod BUF_SIZE
                        -- (since "- BUF_SIZE" is a no-op in 12-bit unsigned).
                        v.ac_best_phase := r.ac_sec_phase_snap
                                         + resize(r.ac_hyp_idx, 12);
                    end if;

                    if r.ac_hyp_idx = K_HYPOTHESES - 1 then
                        v.ac_burst_armed := '0';
                        v.ac_burst_count := r.ac_burst_count + 1;
                    else
                        v.ac_hyp_idx := r.ac_hyp_idx + 1;
                    end if;
                end if;

                -- LOCK decision at the natural-wrap boundary (sec_phase
                -- wraps 4095 → 0), AFTER the single big burst's K=2510
                -- hypotheses have all been evaluated.  ac_best_score is
                -- the true absolute peak across every chip-0 candidate
                -- whose 1586-sample integration window fits non-wrapped
                -- inside the buffer.
                --
                -- target_phase = locked_phase + 1586 + 4 × 2000 (mod 4096):
                --   By end-of-cycle the chip-0 we found is ~5683–8192
                --   sample ticks in the past (= 2.8–4.1 broadcast-seconds),
                --   so we need to skip 4 future broadcast-second window-
                --   ends to land on the first one in the future.  In
                --   12-b natural wrap, (1586 + 4·2000) mod 4096 = 1394.
                if phase_err_v = '1' and v.sec_phase = 0 then
                    if r.ac_best_score >= to_unsigned(LOCK_IN_THR, 19) then
                        v.mode := M_LOCK;
                        v.locked_phase := r.ac_best_phase;
                        v.target_phase := r.ac_best_phase
                                        + to_unsigned(
                                              (SAMPLES_PER_PRBS
                                               + 4 * SAMPLES_PER_SEC)
                                              mod BUF_SIZE,
                                              12);
                        v.track_step  := T_MINUS;
                        v.track_armed := '0';
                        v.track_best_idx := (others => '0');
                        v.track_best_score := (others => '0');
                        v.loss_count := (others => '0');
                        v.pm_lock_qual := (others => '0');
                        v.sync_state_o := SYNC_STATE_LOCK;
                    end if;
                    v.ac_best_score  := (others => '0');
                    v.ac_burst_count := (others => '0');
                end if;

            when M_LOCK =>
                v.sync_state_o := SYNC_STATE_LOCK;

                -- Trigger the TRACK 3-hypothesis sequence when sec_phase reaches
                -- locked_phase + SAMPLES_PER_PRBS (= end of the integration window
                -- whose chip-0 is at sec_phase = locked_phase).  In other words,
                -- at sec_phase = (locked_phase + 1586) mod 2000, the most recent
                -- 1586 samples in the buffer are exactly the on-time integration
                -- window.
                -- TRACK trigger: when sec_phase reaches target_phase.  Both
                -- are 11-bit unsigned with natural wrap.
                if phase_err_v = '1'
                   and r.track_armed = '0'
                   and r.eng_state = E_IDLE
                   and r.sec_phase = r.target_phase
                then
                    v.track_armed := '1';
                    v.track_step := T_MINUS;
                    v.track_best_idx := (others => '0');
                    v.track_best_score := (others => '0');
                end if;

                if r.track_armed = '1' and r.eng_state = E_IDLE then
                    -- The engine reads buf at base = w_ptr - 1586 + DLL_OFFSET.
                    -- DLL_OFFSET is -1 for T_MINUS, 0 for T_CENTER, +1 for T_PLUS.
                    -- In unsigned mod-4096 arithmetic: subtract 1586, ±1.
                    case r.track_step is
                        when T_MINUS  =>
                            next_offset := resize(r.w_ptr + BUF_SIZE
                                                  - SAMPLES_PER_PRBS - 1, 12);
                        when T_CENTER =>
                            next_offset := resize(r.w_ptr + BUF_SIZE
                                                  - SAMPLES_PER_PRBS, 12);
                        when T_PLUS   =>
                            next_offset := resize(r.w_ptr + BUF_SIZE
                                                  - SAMPLES_PER_PRBS + 1, 12);
                    end case;
                    start_engine := true;
                end if;

                if engine_done then
                    abs_acc := abs20(v.acc);
                    if abs_acc > r.track_best_score then
                        v.track_best_score := abs_acc;
                        case r.track_step is
                            when T_MINUS  => v.track_best_idx := "01";
                            when T_CENTER => v.track_best_idx := "10";
                            when T_PLUS   => v.track_best_idx := "11";
                        end case;
                    end if;
                    if r.track_step = T_CENTER then
                        v.track_center_acc := v.acc;
                    end if;

                    case r.track_step is
                        when T_MINUS  => v.track_step := T_CENTER;
                        when T_CENTER => v.track_step := T_PLUS;
                        when T_PLUS   =>
                            -- End of the 3-hypothesis sequence.  Advance
                            -- target_phase by SAMPLES_PER_SEC for the next
                            -- broadcast-second's TRACK firing; natural 11-b
                            -- wrap absorbs the 48-sample-per-second slip vs
                            -- the BUF_SIZE counter cycle.
                            v.track_armed := '0';
                            v.target_phase := r.target_phase
                                            + to_unsigned(SAMPLES_PER_SEC, 12);
                            -- DLL: ±1 sample adjustment on the winning offset.
                            -- Applied to BOTH locked_phase (for diagnostics)
                            -- and target_phase (for next trigger).
                            case v.track_best_idx is
                                when "01" =>
                                    v.locked_phase := r.locked_phase - 1;
                                    v.target_phase := v.target_phase - 1;
                                when "11" =>
                                    v.locked_phase := r.locked_phase + 1;
                                    v.target_phase := v.target_phase + 1;
                                when others => null;  -- center → no change
                            end case;
                            -- Latch output
                            v.pm_soft  := sat12_shr(r.track_center_acc, PM_LLR_SHIFT);
                            v.pm_valid := '1';
                            v.pm_minute_mark := '1';
                            -- Lock-quality IIR: q ← q + (|center_acc|>>shift - q)>>4
                            v.pm_lock_qual := r.pm_lock_qual
                                + resize(shift_right(abs20(r.track_center_acc), 4),
                                         16)
                                - shift_right(r.pm_lock_qual, 4);
                            -- Loss tracking
                            if abs20(r.track_center_acc) <
                                   to_unsigned(LOSS_THR * (2**PM_LLR_SHIFT), 19)
                            then
                                if r.loss_count = K_LOSS - 1 then
                                    v.mode := M_ACQUIRE;
                                    v.sync_state_o := SYNC_STATE_ACQUIRE;
                                    v.ac_burst_count := (others => '0');
                                    v.ac_best_score  := (others => '0');
                                    v.loss_count := (others => '0');
                                else
                                    v.loss_count := r.loss_count + 1;
                                end if;
                            else
                                v.loss_count := (others => '0');
                            end if;
                    end case;
                end if;
        end case;

        ----------------------------------------------------------------
        -- Engine start (initialise per-hypothesis state)
        ----------------------------------------------------------------
        if start_engine then
            v.eng_state    := E_REQ;
            v.eng_base_ptr := next_offset;
            v.sample_idx   := (others => '0');
            v.acc          := (others => '0');
            v.chip_acc     := (others => '0');
            v.lfsr         := LFSR_INIT;
        end if;

        ----------------------------------------------------------------
        -- BRAM read address (combinational from the next state's view)
        ----------------------------------------------------------------
        if start_engine then
            ram_raddr <= resize(next_offset, 12);
        else
            ram_raddr <= resize(r.eng_base_ptr + r.sample_idx, 12);
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
