# AXI4-Lite Slave — Structure & Coding Style

Canonical reference for the DCF77 receiver's AXI4-Lite register interface and
the two-process coding style used throughout the RTL. It supersedes the ad-hoc
VHDL example skeleton the project used to carry under `doc/`: the receiver's own
files (`inc/axi4lite_interface_pkg.vhd`, `src/dcf77/axi_readmux.vhd`,
`src/dcf77/axi_writemux.vhd`, `src/dcf77/dcf77_axi_lite.vhd`) are the live,
corrected, domain-specific implementation of everything described here.

Read this before touching the register map, adding a register, or writing a new
two-process entity.

---

## 1. Two-process coding style

Every synchronous entity in `src/dcf77/` follows the Gaisler structured
two-process method (J. Gaisler, "A structured VHDL design method"; [vhdl2proc](https://www.gaisler.com/doc/vhdl2proc.pdf)): **all** state
lives in one record `r`, a single combinational process computes the next-state
variable `v` (published as `rin`), and a single sequential process clocks
`rin → r`.

```vhdl
architecture rtl of some_entity is
    type reg_type is record
        -- every flip-flop in the entity lives here
        state   : state_t;
        counter : natural range 0 to N;      -- range-limited, not unsigned
        pulse   : std_logic;
    end record;

    constant R_RESET : reg_type := (state => idle, counter => 0, pulse => '0');
    signal r, rin : reg_type;
begin
    Output <= r.pulse;                        -- outputs driven from r, never from inputs

    comb : process (all)
        variable v : reg_type;
    begin
        v := r;                               -- default: hold all state
        v.pulse := '0';                       -- self-clearing bits default low each cycle

        -- next-state logic: read r.* and the input ports, write v.*
        -- ... set v.pulse := '1' only where the one-cycle event fires ...

        if ARESETN = '0' then
            v := R_RESET;                     -- synchronous reset
        end if;
        rin <= v;
    end process;

    seq : process (ACLK)
    begin
        if rising_edge(ACLK) then
            r <= rin;
        end if;
    end process;
end architecture;
```

Invariants (enforced across the codebase):

- **No combinational input→output path.** Outputs are driven from `r`
  (registered) or from `v` (next `r`), never straight from an input. This keeps
  the slave AXI-timing-clean.
- **Clocks and resets stay scalar entity ports** — never inside a record. A
  clock in a record picks up a delta delay and confuses clock-tree/timing tools
  (this is why the `axi4lite_si_type`/`so_type` bundles omit the global fields).
- **Pulsed bits default-cleared at the top of `comb`** (`v.pulse := '0'`), then
  set to `'1'` only on the matching event, so the registered output is high for
  exactly one clock.
- **Counters are range-limited `natural range 0 to N`**, not raw `unsigned`, so
  out-of-range values are a simulation error rather than a silent wrap.
- **Signal-processing nets are `sfixed`/`ufixed`** (`ieee.fixed_pkg`) with
  explicit `(high downto low)` ranges; pure counters/bit-vectors stay
  `unsigned`/`std_logic_vector`.

Indentation is 4 spaces, no tabs. Records and enums get descriptive names
(`reg_type`, `state_t`, `dcf77_status_type`).

---

## 2. Channel record types

`inc/axi4lite_interface_pkg.vhd` defines one record per AXI4-Lite channel,
**grouped by direction from the endpoint's viewpoint**:

- `*_input_type`  — signals flowing **into** this endpoint (a slave's inputs).
- `*_output_type` — signals this endpoint **drives out**.

| Channel | `_input_type` fields | `_output_type` fields |
|---------|----------------------|-----------------------|
| Address (`aw`, `ar`) | `valid`, `addr[7:0]`, `prot[2:0]` | `ready` |
| Write data (`w`)     | `valid`, `data[31:0]`, `strb[3:0]` | `ready` |
| Write response (`b`) | `ready` | `valid`, `resp[1:0]` |
| Read data (`r`)      | `ready` | `valid`, `data[31:0]`, `resp[1:0]` |

Constants: `AXILITE_AXI_DATA_WIDTH = 32`, `AXILITE_AXI_ADDR_WIDTH = 8`,
responses `AXI_RESPONSE_OKAY` (`"00"`), `AXI_RESPONSE_SLVERR` (`"10"`),
`AXI_RESPONSE_DECERR` (`"11"`). `EXOKAY` is not used (not part of AXI4-Lite).

The slave-port bundles collect the five channels by direction, **without** the
global clock/reset:

```vhdl
type axi4lite_si_type is record   -- master -> slave: aw, w, b, ar, r (all _input_type)
type axi4lite_so_type is record   -- slave -> master: aw, w, b, ar, r (all _output_type)
```

`dcf77_axi_lite` takes `axi4lite_si_type` / `axi4lite_so_type` plus scalar
`s_axi_aclk` / `s_axi_aresetn`; flattening to the discrete `s_axi_*` signals
Vivado IP Integrator expects happens in `dcf77_top_bd`.

---

## 3. Handshake rules (AMBA A3.2.1)

All five channels use the same VALID/READY handshake:

- Transfer occurs when **both VALID and READY are HIGH** at a rising clock edge.
- The **source must not wait for READY** before asserting VALID.
- **VALID must stay asserted** until the handshake completes.
- The destination may wait for VALID before asserting READY.

The muxes implement this with a small FSM held in `r` (see below); because of
the two-process discipline, the handshake decisions are always made on the
registered `r.*` values captured the previous clock — never combinationally off
the live input.

---

## 4. Read path — `axi_readmux.vhd`

Registers `ar`/`r` channels, snapshots the data sources
(`FromStatus`, `FromControl`, `FromIrqGen`) every cycle, and serves reads:

1. **AR handshake** — `aro.ready` goes high for one cycle when `ari.valid` is
   seen and no read response is outstanding (`ro.valid = '0'`), then drops.
2. **Decode** — one cycle later (`ari.valid and aro.ready`), the word index is
   `to_integer(unsigned(ari.addr(7 downto 2)))`; a `case word_ix` selects the
   pack function for that register, sets `ro.data` and `ro.resp := OKAY`, and
   asserts `ro.valid`.
3. **Release** — `ro.valid` is held until the master asserts `ri.ready`.

Unknown offsets, and any register gated off by a generic, return
`AXI_RESPONSE_DECERR` with zero data. Read is decoupled from RREADY: the
response is formed on the AR handshake, not gated on the master accepting it.

**Capability bits & atomic pairs.** `REG_STATUS` ORs in synthesis-time
capability bits (`PM_PRESENT`, `PM_DEBUG_PRESENT`) from the generics. Certain
"primary" reads latch a companion word into the shared `shadow` register on the
*same* comb cycle, so a follow-up `REG_SHADOW` read returns a value coherent
with the primary: `TIME_HMS → date`, `SOFT_PM → soft_am_paired`,
`PM_TIME_HMS → pm_date`.

---

## 5. Write path — `axi_writemux.vhd`

Registers `aw`/`w`/`b` channels and decodes writes:

1. **Byte-strobe merge** — on `WInput.valid`, each asserted `strb(byte)` merges
   that byte of `WInput.data` into the registered word, so partial-word writes
   preserve the untouched bytes.
2. **AW/W handshake** — `awo.ready` pulses when both `awi.valid` and `wi.valid`
   are present; `wo.ready` asserts once an address is present.
3. **Decode & respond** — on the cycle that closes the W handshake
   (`wi.valid and awo.ready`), `case word_ix` applies the write and sets
   `bo.resp` (default `SLVERR`, `OKAY` on a recognised writable register), then
   asserts `bo.valid`, held until `bi.ready`.

Writable registers: `REG_CTRL` (field-by-field: `enable`, `env_mode` sticky;
`soft_reset` a one-cycle pulse), `REG_IRQ_FLAG` (W1C — one-cycle per-bit clear
pulses to `dcf77_irq_gen`), `REG_IRQ_MASK` (sticky enable word), and — only when
`WITH_PM_DEBUG` — `REG_PLL_OFFSET` / `REG_PLL_I_STATE` (carrier-PLL jam
overrides). Every other offset returns `SLVERR`.

Pulsed outputs (`ctrl.soft_reset`, `pll.offset_wr`, `pll.i_state_wr`,
`irq.flag`) are default-cleared at the top of `comb` and set only on the
matching write — the canonical one-clock-pulse pattern.

---

## 6. Register map & pack/unpack pattern

The map is defined once in `inc/dcf77_pkg.vhd` as word-index constants
(`REG_CTRL = 0` … `REG_SHADOW = 24`; byte offset = index × 4). Each register has
a **pack function** `<name>_to_slv(status/ctrl) : std_logic_vector(31 downto 0)`
that serialises a domain record into the 32-bit word with a fixed bit layout
(e.g. `status_to_slv`, `time_hms_to_slv`, `date_to_slv`, `soft_am_to_slv`,
`irq_to_slv`), and — for writable registers — an **unpack** helper
(`slv_to_irq`, or field-by-field in the writemux for `REG_CTRL`). Bit positions
are named constants too (`REG_STATUS_PLL_LOCK_BIT`, `REG_DATE_DST_ACTIVE_BIT`, …)
so the layout is single-sourced between RTL, tests, and the host driver.

The readmux/writemux `case` arms bind `REG_*` index → pack/unpack function;
`WITH_PM` / `WITH_PM_DEBUG` generics gate whole register ranges (DECERR on read,
SLVERR on write) and drive the `REG_STATUS` capability bits.

The full bit-field table lives in the README and `doc/dcf77_architecture.md`;
`dcf77_pkg.vhd` is the authority.

---

## 7. Adding a new register

1. Add a `REG_<NAME> : natural := <index>;` constant in `inc/dcf77_pkg.vhd`
   (next free word index), plus any `REG_<NAME>_*_BIT` field constants.
2. Add the field(s) to the relevant domain record (`dcf77_status_type`,
   `dcf77_control_type`, …) and its `*_RESET` constant.
3. Write a `<name>_to_slv(...)` pack function (and `slv_to_<name>` if writable).
4. Add a `when REG_<NAME> =>` arm in `axi_readmux.vhd`; gate it on a generic if
   it belongs to an optional block.
5. If writable, add a `when REG_<NAME> =>` arm in `axi_writemux.vhd` (set
   `bo.resp := OKAY`; default pulsed bits at the top of `comb`).
6. Update the register-map table (README / `doc/dcf77_architecture.md`) and add
   a testbench case in `simulation/tb/05_dcf77_axi_lite_tb.vhd`.

---

## 8. Live files

| Concern | File |
|---------|------|
| Channel record types + bundles | `inc/axi4lite_interface_pkg.vhd` |
| Register map + pack/unpack helpers | `inc/dcf77_pkg.vhd` |
| Read arbiter/decoder | `src/dcf77/axi_readmux.vhd` |
| Write arbiter/decoder | `src/dcf77/axi_writemux.vhd` |
| Top-level shell (muxes + core + irq_gen) | `src/dcf77/dcf77_axi_lite.vhd` |
| Flat `s_axi_*` IP wrapper | `fpga/dcf77_top_bd.vhd` |
| AXI slave testbench | `simulation/tb/05_dcf77_axi_lite_tb.vhd` |
