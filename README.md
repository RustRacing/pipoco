# Pipoco — a minimal Rust ECU

Pipoco is a small, no_std engine control core written in Rust. It decodes a 60‑2 trigger, schedules injection/ignition events, and exposes runtime/state and tables to TunerStudio for tuning. The core is platform‑agnostic; board‑specific crates wire up pins, ADC, and persistence.

Status: prototype, validated with a trigger simulator. Designed to run on real hardware next.

## Highlights

- 60‑2 trigger decoder with sync/loss handling
- Batch and sequential fuel injection; wasted‑spark and sequential ignition
- Angle‑based scheduling (per‑cylinder TDC + BTDC targets); low‑jitter tick deadlines
- TunerStudio pages for tables and runtime configuration (Sensors, AE, DFCO, Limits, Diagnostics, Angles)
- no_std core with fixed‑size data and integer math; zero dependencies in core

## Getting Started

- Run tests and lints (host):
  - `cargo test --all-features`
  - `cargo clippy --all-targets --all-features -- -D warnings`

- Build a target (examples):
  - RP2040 Pico: `cargo build -p ecu-rp2040-pico --release`
  - STM32F4: `rustup target add thumbv7em-none-eabihf && cargo build -p stm32f4-ecu --release`

See per‑target docs for pins and features: `rp2040-pico/TS-HOWTO.md`, target README files.

## TunerStudio Integration

- INI: `ts/IPW-ECU.ini` (signature “IPW‑ECU V0.1”).
- Pages in use:
  - [Sensors] (3): TPS/MAP calibration, CLT/IAT curves
  - [AE] (4): accel enrichment, decay, lockout
  - [DFCO] (5): thresholds, delays, hysteresis
  - [Limits] (6): sensor clamps and optional emergency triggers
  - [Diag] (7): fault flags; [DiagLog] (8): recent events
  - [Angles] (9): per‑cylinder injection/spark references and cam timeout

## Scheduling Modes

- Default: angle‑based scheduling using live tooth timing with per‑cylinder angles.
- Fallback: `sched-simple` feature — simple RPM‑based half‑revolution model.
  - Build: `cargo build --features sched-simple`
  - Test: `cargo test --features sched-simple`

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
└── ts/                  # TunerStudio INI
```

## Contributing

- Tests and clippy must pass (`--all-features`).
- Keep core no_std with fixed‑size data structures. Avoid panics.
- When touching TunerStudio, keep INI/page sizes in sync (tests verify signature and sizes).

## License

AGPL-3.0-or-later
