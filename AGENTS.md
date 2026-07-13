# Repository Guidelines
- If a command fails due to insufficient permissions, you must elevate the command to the user for approval.
## Project Structure & Module Organization
- `inc/`: Shared VHDL packages.
- `src/dcf77/`: Core RTL.
- `simulation/tb/`: VUnit testbenches.
- `simulation/scripts/`: VUnit runner scripts; outputs default to `simulation/derived`.

## Build, Test, and Development Commands
- Run VUnit suite (GHDL backend): `python3 simulation/scripts/run.py --clean` (rebuilds and runs tests; artifacts in `simulation/derived`).
- List tests only: `python3 simulation/scripts/run.py --list`.

## Coding Style & Naming Conventions
- VHDL-2008, structured two-process style (combinational + sequential) per `doc/structdes.pdf`.
- Indentation: 4 spaces; avoid tabs. Use descriptive record/enum names (e.g., `state_t`, `r_type`).
- AXI-Stream ports use `s_axis_*` and `m_axis_*`.
- AXI4-Lite slave structure & coding style: see `doc/axi4lite_interface.md`.

## Testing Guidelines
- Framework: VUnit with GHDL. Testbench names in `simulation/tb` prefixed with two-digit indices (e.g., `00_reset...`, `01_no_error`).
- Before running tests, ensure `.venv` is active or system Python has VUnit available.

## Commit & Pull Request Guidelines
- Commit messages: short title, blank line, then concise description (see recent commits for pattern).
- Keep changes scoped; include new/updated tests for functional changes.

## Architecture Overview
- RESET is active high
- CLOCK (system clock) is 125 MHz
- Samplerate is 125 MHz
- Samplewidth is 14 Bit
- Objective is to implement a DCF77 decoder providing all decoded data, RSSI and optionally an SNR estimation via an AXI4-Lite registerset 
- Use 'DCF77-Prompt.txt' as architectural guide for this project


