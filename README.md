# Pipoco, the Rust ECU

Minimal modular ECU implementation in Rust that works on pi pico.

## Features

- ✅ 60-2 trigger decoder
- ✅ IPW table lookup (16x16, no interpolation)
- ✅ Batch injection (all injectors fire together)
- ✅ Wasted spark ignition (pairs fire together)
- ✅ Fixed correction multipliers
- ✅ no_std embedded Rust
- ✅ Zero dependencies in core library
- ✅ Angle-based sequential injection and ignition (per-cylinder)
- ✅ TunerStudio pages for sensors, enrichment, DFCO, diagnostics, and angles

## Project Structure

```
├── src/               # Core ECU library (no_std, zero deps)
│   ├── lib.rs
│   ├── hal.rs        # HAL trait definitions
│   ├── trigger.rs    # 60-2 decoder
│   ├── tables.rs     # IPW table lookup
│   └── scheduler.rs  # Event scheduler
├── stm32f4/          # STM32F4 application
│   └── src/
│       ├── main.rs
│       └── hal_impl.rs
└── tests/            # Unit tests
    └── trigger_test.rs
```

## Building

### Core Library Tests (on host)

```bash
cargo test
```

### STM32F4 Application

```bash
cd stm32f4
cargo build --release
```

### Flash to Hardware

```bash
cd stm32f4
cargo flash --chip STM32F405RGTx --release
```

Or using probe-rs directly:
```bash
cd stm32f4
cargo run --release
```

## Hardware Setup

### Required Hardware
- STM32F405RGTx development board
- Trigger wheel simulator (Ardu-Stim recommended)
- LEDs or oscilloscope for testing outputs

### Pin Connections

**Inputs:**
- PA0: Trigger input (60-2 wheel signal)

**Outputs:**
- PB0: Injector 1
- PB1: Injector 2
- PB2: Ignition coil 1
- PB3: Ignition coil 2

## Testing

### Unit Tests
```bash
cargo test
```

Tests include:
- Trigger sync detection
- RPM calculation
- Table lookup
- Correction multiplication
- Fuel calculation

### Bench Testing

1. Connect Ardu-Stim to PA0
2. Set Ardu-Stim to 60-2 pattern at 1000 RPM
3. Connect LEDs to PB0-PB3
4. Flash firmware
5. Verify LEDs pulse when trigger signal applied
6. Increase RPM to 6000, verify stable operation

### First Engine Test

**Preparation:**
1. Install 60-2 trigger wheel on engine
2. Connect trigger sensor to PA0
3. Wire all 4 injectors in parallel to PB0 (batch injection)
4. Wire coils in pairs: Cyl 1+4 to PB2, Cyl 2+3 to PB3 (wasted spark)

**Procedure:**
1. Crank engine with fuel disabled
2. Verify trigger sync via debug output
3. Enable fuel pump
4. Start engine with rich IPW table
5. Adjust table values until engine runs smoothly

## Code Size

Target: <64KB flash, <8KB RAM

Actual (optimized):
```bash
cd stm32f4
cargo size --release
```

## TunerStudio Pages (selected)

- [Sensors] page (3): TPS/MAP calibration and CLT/IAT curves
- [AE] page (4), [DFCO] page (5)
- [Limits] page (6): sensor clamps and emergency triggers
- [Diag] page (7): fault flags, [DiagLog] page (8): recent events
- [Angles] page (9):
  - inj_angle_btdc_x10[16] (deg*10, u16)
  - tdc_per_cyl_x10[16] (deg*10, u16)
  - tooth0_angle_x10 (deg*10, u16)
  - cam_missing_timeout_ms (u16)

## Scheduling Modes

- Default: angle-based scheduling using live tooth timing with per-cylinder TDC and BTDC offsets. Uses tick-based deadlines for low jitter.
- Fallback: `sched-simple` feature uses a simple RPM-based half-rev model with optional per-cylinder offsets.
  - Build: `cargo build --features sched-simple`
  - Tests: `cargo test --features sched-simple` (includes a small guard test to exercise the simple scheduler path)

## Persistence

- Fuel/Ignition tables persist via the KV interface (RAM or flash-backed).
- Angle configuration ([Angles] page) persists via the same KV under key `angles` (68 bytes). RAM KV supports it; flash KV support is provided for targets enabling `flash-kv`.


## License

MIT

## Target: 720 lines of code, 4 weeks, engine running.
