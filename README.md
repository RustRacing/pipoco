# Pipoco — a minimal Rust ECU

Pipoco is a small, no_std engine control core written in Rust. It decodes a 60‑2 trigger, schedules injection/ignition events, and exposes runtime/state and tables to TunerStudio for tuning. The core is platform‑agnostic; board‑specific crates wire up pins, ADC, and persistence.

Status: prototype with a verified runtime inventory. The live path currently covers trigger decoding, scheduler execution, safety gating, and base injector pulse-width lookup; several advertised control features remain built but not wired.

## Highlights

- 60‑2 trigger decoder with sync/loss handling
- Batch and sequential fuel injection; wasted‑spark and sequential ignition
- Angle‑based scheduling (per‑cylinder TDC + BTDC targets); low‑jitter tick deadlines
- TunerStudio pages for tables, calibration, and runtime diagnostics
- no_std core with fixed‑size data and integer math; zero dependencies in core

## Getting Started

- Run the verification script:
  - `./tools/verify.sh`
  - It runs the lint-clean core checks plus the currently supported STM32F4 build slices.
  - Workspace-wide `cargo check --workspace --all-targets --all-features` is not a stable gate because the embedded targets pull incompatible `critical-section` restore-state modes into one resolution, and the RP2040 example bins are still under active repair.

- Build a target (examples):
  - RP2040 Pico: `cargo build -p ecu-rp2040-pico --release`
  - STM32F4: `rustup target add thumbv7em-none-eabihf && cargo build -p stm32f4-ecu --release`

See per‑target docs for pins and features: `rp2040-pico/TS-HOWTO.md`, target README files.

### Feature/Profile Matrix

- `stm32f4-ecu`: `capture-tim` and `capture-gpio` are mutually exclusive capture profiles.
- `ecu-rp2350b`: `capture-gpio` and `capture-pio` are mutually exclusive capture profiles; `example-bins` gates the helper binaries.
- `ecu-rp2040-pico`: `flash-kv`, `capture-pio`, `capture-cam`, and `example-bins` are documented as named feature slices, with the example binaries gated where required.
- Keep the board READMEs and `tools/verify.sh` in sync when a feature profile is added or removed.

## Target Status

- `rp2040-pico`: bring-up target with live TunerStudio runtime data and flash-backed page persistence.
- `stm32f4`: bring-up target with timer capture and example transport bins.
- `rp2350b`: example-only / experimental VE demo and auxiliary bins; not a supported ECU target yet.
- `src/management`: experimental OODA-style control architecture, not wired into the main runtime loop.

## TunerStudio Integration

- INI asset used by tests: `tests/assets/IPW-ECU.ini` (signature “IPW‑ECU V0.1”).
- Pages in use:
  - [Sensors] (3): TPS/MAP calibration, CLT/IAT curves
  - [AE] (4): accel-response calibration and decay/lockout settings
  - [DFCO] (5): thresholds, delays, hysteresis
  - [Limits] (6): sensor clamps and optional emergency triggers
  - [Diag] (7): fault flags; [DiagLog] (8): recent events
  - [Angles] (9): per‑cylinder injection/spark references and cam timeout

## Scheduling Modes

- Default: angle‑based scheduling using live tooth timing with per‑cylinder angles.
- Fallback: `sched-angle-disable` feature — simple RPM‑based half‑revolution model for bring-up and comparison tests. This disables the default angle-based path.
  - Build: `cargo build --features sched-angle-disable`
  - Test: `cargo test --features sched-angle-disable`

## Persistence

- Tables and angles persist via a simple key/value interface (RAM or flash‑backed on targets).
- Keys: `fuel` (512 B), `ign` (512 B), `angles` (68 B). Flash backends include CRC/versioning.

## Safety & Diagnostics

- Sensor range clamps with optional emergency mode
- Flood‑clear and sync‑loss shutdown utilities
- Diagnostics flags and recent‑event log available via TS pages

## Project Layout

```
rust-ipw-ecu/
├── src/                 # Core ECU library (no_std)
│   ├── trigger.rs       # 60-2 decoder + angle tracking
│   ├── scheduler.rs     # Fixed-size event scheduler
│   ├── ignition.rs      # Timing + dwell
│   ├── tables.rs        # IPW/ignition table lookup
│   └── ts/              # TunerStudio proto, server, pages, OUTPC
├── rp2040-pico/         # Pico target (USB TS, optional flash KV)
├── stm32f4/             # STM32F4 target
├── rp2350b/             # RP2350B target (WIP)
└── tests/assets/        # TunerStudio INI asset used by tests
```

## Contributing

- Run `./tools/verify.sh` before review.
- Host-side tests should still pass with `cargo test --all-features`.
- Keep core no_std with fixed‑size data structures. Avoid panics.
- When touching TunerStudio, keep INI/page sizes in sync (tests verify signature and sizes).

## License

AGPL-3.0-or-later
