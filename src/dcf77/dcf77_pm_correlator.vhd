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
        pm_lock_qual     : out unsigned(15 downto 0);

        -- Debug / instrumentation: bundled per dcf77_pkg's
        -- dcf77_pm_debug_type record.  Pulses fire on each engine
        -- completion in ACQUIRE / TRACK; lets TBs log the full
        -- autocorrelation profile + early-late energy trace.
        pm_debug         : out dcf77_pm_debug_type
    );
end entity dcf77_pm_correlator;

architecture rtl of dcf77_pm_correlator is

    ----------------------------------------------------------------------
    -- Constants
    ----------------------------------------------------------------------
    constant BUF_SIZE         : natural := 4096;
    constant SAMPLES_PER_PRBS : natural := 1586;
    constant SAMPLES_PER_SEC  : natural := 2000;
    -- K = SAMPLES_PER_SEC: chip-0 occurs once per broadcast-second, so
    -- searching 2000 candidate positions covers exactly one inter-chip-0
    -- spacing -- guaranteed to find chip-0 in a single burst without
    -- redundancy.  Buffer minimum is K + SAMPLES_PER_PRBS - 1 = 3585
    -- samples; we use BUF_SIZE=4096 (one BRAM tile) leaving the natural
    -- 12-b wrap-around for sec_phase / target_phase arithmetic.
    constant K_HYPOTHESES     : natural := SAMPLES_PER_SEC;
    constant BURSTS_PER_CYCLE : natural := 1;
    -- Sigma-delta DLL threshold.  ±DLL_THRESH ticks of net agreement on
    -- T_MINUS/T_PLUS = one ±1-sample shift of locked_phase.  4 gives
    -- ~4s time constant against the per-second random ±1 noise; bumping
    -- higher slows drift tracking, lower brings back the random walk.
    constant DLL_THRESH       : integer := 4;
    -- One burst per natural-wrap cycle (= 2.048 broadcast-seconds at
    -- 4096 / 2000).  Burst triggers at sec_phase = 0.
    -- K=2000 hypotheses × ~3175 clk ≈ 6.35 M clk ≈ 50.8 ms wall.
    constant SAMPLES_PER_BURST_SLOT : natural := BUF_SIZE;
    constant CHIP_RATE_NUM    : natural := 31;
    constant CHIP_RATE_DEN    : natural := 96;
    -- LFSR per external analysis of the DCF77 broadcast:
    --   Fibonacci LFSR, polynomial x^9 + x^5 + 1, taps 5 & 9 (XOR of stage
    --   5 = bit 4 and stage 9 = bit 8 feeds back into bit 0 on left shift).
    --   Seed = 0x011 = 0b000010001 = 17 (decimal).
    --   Output bit = MSB (bit 8) of the state, taken BEFORE the shift.
    --   Frame extension: one '0' chip PREPENDED before chip 1 .. chip 511,
    --   giving exactly 512 chips with 256 zeros and 256 ones (DC balance).
    -- The engine carries a `first_chip` flag that holds chip_sign='0' for
    -- the duration of the prepended chip; on the first Bresenham crossing
    -- the flag clears and the LFSR-driven chips take over.
    constant LFSR_INIT        : std_logic_vector(8 downto 0) := "000010001";
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

        -- Debug / instrumentation registers (drive corresponding ports
        -- via plain `r.field <= ...` below; one-clock pulses are cleared
        -- to '0' at the top of the comb body).
        dbg_acquire_v        : std_logic;
        dbg_acquire_hyp_idx  : unsigned(11 downto 0);
        dbg_acquire_abs_acc  : unsigned(18 downto 0);
        dbg_acquire_sec_phase: unsigned(11 downto 0);
        dbg_track_v          : std_logic;
        dbg_track_step       : unsigned(1 downto 0);
        dbg_track_abs_acc    : unsigned(18 downto 0);

        ---- LOCK / TRACK ----
        locked_phase     : unsigned(11 downto 0);     -- sec_phase at chip-0 arrival
        target_phase     : unsigned(11 downto 0);     -- sec_phase to dispatch TRACK
                                                      -- (advanced by +2000 each
                                                      -- broadcast-second; natural
                                                      -- 12-b wrap absorbs the
                                                      -- 96-sample slip)
        track_step       : track_step_t;
        track_armed      : std_logic;
        -- w_ptr snapshot captured at TRACK trigger.  All three passes
        -- (T_MINUS / T_CENTER / T_PLUS) compute next_offset from this
        -- frozen value so they integrate over the SAME broadcast PRBS
        -- window shifted by ±1 sample.  Without snapshotting, the
        -- ~396-tick engine latency lets w_ptr drift between passes,
        -- and each pass ends up reading a window ~400 samples offset
        -- from the previous — randomized correlation, dead DLL.
        track_w_ptr_snap : unsigned(11 downto 0);
        track_best_idx   : unsigned(1 downto 0);      -- 0=undecided 1=- 2=center 3=+
        track_best_score : unsigned(18 downto 0);
        track_center_acc : signed(19 downto 0);       -- 20-b signed acc snapshot
        loss_count       : unsigned(3 downto 0);
        pm_lock_qual     : unsigned(15 downto 0);
        -- Sigma-delta DLL accumulator.  Each TRACK cycle increments
        -- (T_PLUS) / decrements (T_MINUS) by 1.  Actual ±1 sample shift
        -- of locked_phase / target_phase only fires when the accumulator
        -- reaches ±DLL_THRESH ("MSB carries out"); residual is preserved
        -- so cumulative bias still gets tracked.  Effective time-constant
        -- is DLL_THRESH × second-rate -> filters out per-cycle noise that
        -- was driving locked_phase into a random walk on the 3 kHz corpus.
        dll_acc          : signed(4 downto 0);        -- range ±15

        ---- Engine state (shared across ACQUIRE and TRACK) ----
        eng_state        : eng_state_t;
        eng_base_ptr     : unsigned(11 downto 0);     -- (w_ptr_snap + offset) mod 4096
        sample_idx       : unsigned(10 downto 0);     -- 0..1585
        acc              : signed(19 downto 0);       -- 20-b signed (max 1586·256 ≈ 406K)
        chip_acc         : unsigned(6 downto 0);
        lfsr             : std_logic_vector(8 downto 0);
        first_chip       : std_logic;                  -- '1' during the prepended '0' chip

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
        track_w_ptr_snap => (others => '0'),
        track_best_idx   => (others => '0'),
        track_best_score => (others => '0'),
        track_center_acc => (others => '0'),
        loss_count       => (others => '0'),
        pm_lock_qual     => (others => '0'),
        dll_acc          => (others => '0'),
        dbg_acquire_v        => '0',
        dbg_acquire_hyp_idx  => (others => '0'),
        dbg_acquire_abs_acc  => (others => '0'),
        dbg_acquire_sec_phase=> (others => '0'),
        dbg_track_v          => '0',
        dbg_track_step       => (others => '0'),
        dbg_track_abs_acc    => (others => '0'),
        eng_state        => E_IDLE,
        eng_base_ptr     => (others => '0'),
        sample_idx       => (others => '0'),
        acc              => (others => '0'),
        chip_acc         => (others => '0'),
        lfsr             => LFSR_INIT,
        first_chip       => '0',
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

    pm_debug.acquire_v          <= r.dbg_acquire_v;
    pm_debug.acquire_hyp_idx    <= r.dbg_acquire_hyp_idx;
    pm_debug.acquire_abs_acc    <= r.dbg_acquire_abs_acc;
    pm_debug.acquire_sec_phase  <= r.dbg_acquire_sec_phase;
    pm_debug.track_v            <= r.dbg_track_v;
    pm_debug.track_step         <= r.dbg_track_step;
    pm_debug.track_abs_acc      <= r.dbg_track_abs_acc;
    pm_debug.track_locked_phase <= r.locked_phase;

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
        v.dbg_acquire_v  := '0';
        v.dbg_track_v    := '0';
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
                -- chip_sign is '0' during the prepended force-1 chip,
                -- LFSR MSB output once the engine has crossed the first
                -- Bresenham boundary.
                --
                -- Polarity mapping per observed broadcast convention
                -- (Python decoder cross-check on dcf_strong: 86.8 % SIK
                -- signature match with this assignment vs 71.7 % with
                -- the originally-assumed mapping).  Somewhere in the
                -- chain (mixer / CORDIC / ...) the phase ends up inverted
                -- relative to the planning-doc convention; absorbing the
                -- inversion here keeps the rest of the chain unchanged.
                --   chip='0' (PNS)  -> multiply sample by -1
                --   chip='1' (PNS̄)  -> multiply sample by +1
                -- Net result (after the chain's own sign flip):
                --   broadcast SIK '0' bit (data '0') -> pm_soft > 0
                --                                    -> bit_value = '0'
                --   broadcast SIK '1' bit (data '1') -> pm_soft < 0
                --                                    -> bit_value = '1'
                -- which is exactly what frame_dec's MM_SIGNATURE =
                -- "111111111100000" expects at the signature seconds.
                sample_in := signed(ram_rdata_r);
                if r.first_chip = '1' then
                    chip_sign := '0';        -- prepended chip = '0'
                else
                    chip_sign := r.lfsr(8);  -- MSB output, before shift
                end if;
                if chip_sign = '0' then
                    v.acc := r.acc - resize(sample_in, 20);
                else
                    v.acc := r.acc + resize(sample_in, 20);
                end if;

                -- Bresenham chip advance: chip_acc += 31; if ≥ 96, sub 96.
                -- On the first crossing, drop the first_chip flag and let
                -- the LFSR start driving chip_sign.  Subsequent crossings
                -- shift the LFSR left (feedback = stage5 XOR stage9).
                chip_acc_next := resize(r.chip_acc, 8) + to_unsigned(CHIP_RATE_NUM, 8);
                if chip_acc_next >= to_unsigned(CHIP_RATE_DEN, 8) then
                    v.chip_acc := resize(chip_acc_next - to_unsigned(CHIP_RATE_DEN, 8), 7);
                    if r.first_chip = '1' then
                        v.first_chip := '0';   -- transition to LFSR-driven chips
                    else
                        new_lfsr_bit := r.lfsr(4) xor r.lfsr(8);
                        v.lfsr := r.lfsr(7 downto 0) & new_lfsr_bit;
                    end if;
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
                    -- Up-count: start at 0, walk to K-1.  Each
                    -- hypothesis k reads slots [w_ptr_snap+k ..
                    -- w_ptr_snap+k+1585].  The writer follows behind
                    -- (also writing slots w_ptr_snap, w_ptr_snap+1,
                    -- ... at one slot per phase_err_v pulse).  As
                    -- long as t_writer >= t_engine_per_hyp (= writes/
                    -- hypothesis <= 1), the writer reaches slot
                    -- w_ptr_snap+i only AFTER the last hypothesis
                    -- that needs to read it (= hyp k=i) has consumed
                    -- it -- so no slot is overwritten before its last
                    -- read.  Down-count would concentrate corruption
                    -- in the last-evaluated (low-k) hypotheses; up-
                    -- count spreads it uniformly at ~1 slot/hyp at
                    -- the boundary.
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
                    -- Per-hypothesis ACQUIRE debug pulse: records |acc|
                    -- + hyp_idx + sec_phase_snap so the TB can dump the
                    -- whole autocorrelation profile.
                    v.dbg_acquire_v         := '1';
                    v.dbg_acquire_hyp_idx   := r.ac_hyp_idx;
                    v.dbg_acquire_abs_acc   := abs_acc;
                    v.dbg_acquire_sec_phase := r.ac_sec_phase_snap;
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

                    -- Burst complete: evaluate LOCK_IN_THR against the
                    -- true running max across all K=2510 hypotheses.
                    -- Previously this fired at the next sec_phase=0
                    -- wrap, but in accelerated TBs the wrap occurs
                    -- before the burst finishes -- the check then
                    -- evaluated a partial-burst max and was misleading.
                    -- Tying the check to burst-complete decouples LOCK
                    -- from buffer-wrap timing and matches the semantic
                    -- intent ("decide once you have a full running max").
                    -- Behaviour in live operation is unchanged: burst
                    -- finishes ~63 ms into a 2.048 s buffer cycle, the
                    -- old wait-until-wrap was idle time.
                    --
                    -- target_phase = (ac_best_phase + SAMPLES_PER_PRBS
                    -- + SAMPLES_PER_SEC) mod BUF_SIZE.  Fires TRACK at
                    -- the END of the NEXT broadcast PRBS:
                    --   1. wait SAMPLES_PER_SEC for the next chip-0
                    --      to arrive in the buffer
                    --   2. wait SAMPLES_PER_PRBS for the whole PRBS
                    --      to be written
                    -- Engine reads buf[w_ptr - 1586 .. w_ptr - 1], so
                    -- w_ptr at fire = chip-0 + SAMPLES_PER_PRBS.
                    -- The previous formula +(SAMPLES_PER_PRBS +
                    -- 4*SAMPLES_PER_SEC) mod BUF_SIZE was off by 192
                    -- samples: SAMPLES_PER_SEC=2000 mod BUF_SIZE=4096
                    -- ≠ 0, so "skip 4 seconds" inside the mod doesn't
                    -- preserve alignment.  12-b natural wrap handles
                    -- future-ness automatically.
                    if r.ac_hyp_idx = K_HYPOTHESES - 1 then
                        v.ac_burst_armed := '0';
                        if v.ac_best_score >= to_unsigned(LOCK_IN_THR, 19) then
                            v.mode := M_LOCK;
                            v.locked_phase := v.ac_best_phase;
                            -- track_w_ptr_snap captures w_ptr at the
                            -- trigger cycle (where r.sec_phase =
                            -- r.target_phase), so no pipeline-delay
                            -- compensation is needed.  target_phase
                            -- = ac_best_phase + SAMPLES_PER_PRBS +
                            -- SAMPLES_PER_SEC lands w_ptr_snap at the
                            -- end of the next PRBS window exactly.
                            v.target_phase := v.ac_best_phase
                                            + to_unsigned(
                                                  SAMPLES_PER_PRBS
                                                + SAMPLES_PER_SEC, 12);
                            v.track_step  := T_MINUS;
                            v.track_armed := '0';
                            v.track_best_idx := (others => '0');
                            v.track_best_score := (others => '0');
                            v.loss_count := (others => '0');
                            v.pm_lock_qual := (others => '0');
                            v.dll_acc      := (others => '0');
                            v.sync_state_o := SYNC_STATE_LOCK;
                        end if;
                        v.ac_best_score  := (others => '0');
                        v.ac_burst_count := (others => '0');
                    else
                        v.ac_hyp_idx := r.ac_hyp_idx + 1;
                    end if;
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
                    -- Snapshot w_ptr at trigger.  All 3 passes
                    -- (M / C / P) read from this frozen value plus
                    -- (BUF_SIZE - SAMPLES_PER_PRBS ± 1), so they
                    -- integrate over the SAME PRBS window with a
                    -- ±1-sample shift.  Without this, the engine's
                    -- ~396-tick latency lets w_ptr drift between
                    -- passes -- each pass reads a ~+400-sample
                    -- offset window, randomizing correlation.
                    v.track_w_ptr_snap := r.w_ptr;
                end if;

                if r.track_armed = '1' and r.eng_state = E_IDLE then
                    -- The engine reads buf at base = w_ptr_snap - 1586
                    -- + DLL_OFFSET.  DLL_OFFSET is -1 for T_MINUS, 0
                    -- for T_CENTER, +1 for T_PLUS.  In unsigned mod-
                    -- 4096 arithmetic: subtract 1586, ±1.  BUF_SIZE -
                    -- SAMPLES_PER_PRBS ± 1 fits in 12 bits
                    -- (2509/2510/2511 < 4096); folding into one
                    -- to_unsigned avoids the intermediate (w_ptr_snap
                    -- + BUF_SIZE) that truncates because BUF_SIZE =
                    -- 2^12 = 4096 doesn't fit.
                    case r.track_step is
                        when T_MINUS  =>
                            next_offset := r.track_w_ptr_snap + to_unsigned(
                                BUF_SIZE - SAMPLES_PER_PRBS - 1, 12);
                        when T_CENTER =>
                            next_offset := r.track_w_ptr_snap + to_unsigned(
                                BUF_SIZE - SAMPLES_PER_PRBS, 12);
                        when T_PLUS   =>
                            next_offset := r.track_w_ptr_snap + to_unsigned(
                                BUF_SIZE - SAMPLES_PER_PRBS + 1, 12);
                    end case;
                    start_engine := true;
                end if;

                if engine_done then
                    abs_acc := abs20(v.acc);
                    -- TRACK debug pulse: |acc| per pass (M/C/P).  Lets
                    -- the TB watch the early-late energy ratio over time.
                    v.dbg_track_v       := '1';
                    v.dbg_track_abs_acc := abs_acc;
                    case r.track_step is
                        when T_MINUS  => v.dbg_track_step := "00";
                        when T_CENTER => v.dbg_track_step := "01";
                        when T_PLUS   => v.dbg_track_step := "10";
                    end case;
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
                            -- Sigma-delta DLL.  Each cycle ±1 LSB into
                            -- dll_acc; an actual sample shift fires only
                            -- when the accumulator reaches ±DLL_THRESH
                            -- ("MSB carries out").  Residual is preserved
                            -- (subtract DLL_THRESH, don't reset) so any
                            -- cumulative bias still gets tracked but
                            -- per-cycle noise averages out -- replaces the
                            -- ±1 sample/sec random walk seen with the
                            -- old unfiltered DLL.
                            case v.track_best_idx is
                                when "01" =>  -- T_MINUS won
                                    v.dll_acc := r.dll_acc - 1;
                                when "11" =>  -- T_PLUS won
                                    v.dll_acc := r.dll_acc + 1;
                                when others => null;
                            end case;
                            if v.dll_acc >= DLL_THRESH then
                                v.locked_phase := r.locked_phase + 1;
                                v.target_phase := v.target_phase  + 1;
                                v.dll_acc      := v.dll_acc - DLL_THRESH;
                            elsif v.dll_acc <= -DLL_THRESH then
                                v.locked_phase := r.locked_phase - 1;
                                v.target_phase := v.target_phase  - 1;
                                v.dll_acc      := v.dll_acc + DLL_THRESH;
                            end if;
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
            v.first_chip   := '1';      -- prepended force-1 '0' chip
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
