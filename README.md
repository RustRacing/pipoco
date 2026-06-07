# Pipoco — a minimal Rust ECU

Pipoco is a small Rust ECU workspace built around no_std-compatible runtime crates. The split crates own trigger decoding, scheduling, board interfaces, TunerStudio protocol/page DTOs, and fuel/runtime strategy; `ecu-core` remains a legacy compatibility facade for `EcuState`, TS page-store glue, trigger primitives, and safety helpers. Board crates wire those reusable pieces to pins, ADC, timers, persistence, and transport.

Status: prototype with a verified runtime inventory. The live path currently covers trigger decoding, scheduler execution, safety gating, and base injector pulse-width lookup; several advertised control features remain built but not wired.

## Highlights

- 60‑2 trigger decoder with sync/loss handling
- Batch and sequential fuel injection; wasted‑spark and sequential ignition
- Angle‑based scheduling (per‑cylinder TDC + BTDC targets); low‑jitter tick deadlines
- TunerStudio pages for tables, calibration, and runtime diagnostics
- no_std-compatible crates with fixed-size data and integer math; board and runtime responsibilities are split by crate

## Getting Started

- Run the verification script:
  - `./tools/verify.sh`
  - It runs the lint-clean core checks plus the currently supported STM32F4 build slices.
  - Workspace-wide `cargo check --workspace --all-targets --all-features` is not a stable gate because the embedded board crates pull incompatible `critical-section` restore-state modes into one resolution, and the RP2040 example bins are still under active repair.

- Choose a firmware recipe before building or flashing:
  - List renderable board/recipe pairs: `cargo run -p ecu-firmware-resolver -- --list`
  - Render a build command: `cargo run -p ecu-firmware-resolver -- rp2040-pico ignition-only-wasted-spark-no-watchdog-bringup`
  - Render a flash command when metadata supports it: `cargo run -p ecu-firmware-resolver -- --flash-command rp2040-pico ignition-only-wasted-spark-no-watchdog-bringup`

- Manual board package builds are still useful for board bring-up:
  - RP2040 Pico: `cargo build -p ecu-rp2040-pico --release`
  - STM32F4: `rustup target add thumbv7em-none-eabihf && cargo build -p stm32f4-ecu --release`

See per-board docs for pins and features: `boards/rp2040-pico/TS-HOWTO.md` and the board README files.

### Feature/Profile Matrix

- `stm32f4-ecu`: `capture-tim` and `capture-gpio` are mutually exclusive capture profiles.
- `ecu-rp2350b`: `capture-gpio` is the available capture profile; `example-bins` gates the helper binaries (`capture-pio` was removed).
- `ecu-rp2040-pico`: `flash-kv`, `capture-pio`, `capture-cam`, and `example-bins` are documented as named feature slices, with the example binaries gated where required.
- Keep the board READMEs and `tools/verify.sh` in sync when a feature profile is added or removed.

## Board Status

- `boards/rp2040-pico`: bring-up board with live TunerStudio runtime data and flash-backed page persistence.
- `boards/stm32f4`: bring-up board with timer capture and example transport bins.
- `boards/rp2350b`: example-only / experimental VE demo and auxiliary bins; not a supported ECU board yet.

## TunerStudio Integration

- INI asset used by tests: `crates/core/tests/assets/IPW-ECU.ini` (signature “IPW‑ECU V0.1”).
- Pages in use:
  - [Sensors] (3): TPS/MAP calibration, CLT/IAT curves
  - [AE] (4): accel-response calibration and decay/lockout settings
  - [DFCO] (5): thresholds, delays, hysteresis
  - [Limits] (6): sensor clamps and optional emergency triggers
  - [Diag] (7): fault flags; [DiagLog] (8): recent events
  - [Angles] (9): per‑cylinder injection/spark references and cam timeout

## Scheduling Modes

- Angle-based scheduling using live tooth timing with per-cylinder angles is the only implemented path.

## Persistence

- Tables and angles persist via a simple key/value interface (RAM or flash‑backed on boards).
- Keys: `fuel` (512 B), `ign` (512 B), `angles` (68 B). Flash backends include CRC/versioning.

## Safety & Diagnostics

- Sensor range clamps with optional emergency mode
- Flood‑clear and sync‑loss shutdown utilities
- Diagnostics flags and recent‑event log available via TS pages

## Project Layout

```
pipoco/
├── crates/              # Reusable ECU libraries
│   ├── core/            # Legacy compatibility facade (no_std-compatible)
│   ├── domain/          # Units, ids, authority, and shared domain types
│   ├── runtime/         # Runtime orchestration
│   └── spec/            # Executable specs and formal/proof artifacts
├── boards/              # Embedded board support packages
│   ├── common/          # Shared board adapters and helpers
│   ├── rp2040-pico/     # Pico board
│   ├── rp2350b/         # RP2350B board
│   └── stm32f4/         # STM32F4 board
├── sim/                 # Simulator, FFI, and host driver crates
├── tools/               # Verification and maintenance scripts
└── tests/               # Python/tooling tests
```

## Contributing

- Run `./tools/verify.sh` before review.
- Host-side tests: `cargo test --workspace --all-features --exclude ecu-rp2040-pico --exclude stm32f4-ecu --exclude ecu-rp2350b`.
- Keep no_std-compatible crates on fixed-size data structures. Avoid panics.
- When touching TunerStudio, keep INI/page sizes in sync (tests verify signature and sizes).

## License

AGPL-3.0-or-later
