# RP2350B Board Bring-Up

This crate contains the RP2350B board bring-up firmware plus a separate
runtime semantic fuel-evaluation demo. The supported flashable board path is
`ecu-rp2350b-min`; the fuel demo is explicitly non-vehicle demo code.

## Hardware Specifications

- **MCU**: RP2350B (Raspberry Pi silicon)
- **Architecture**: Dual Cortex-M33 @ 150MHz (ARMv8-M Mainline)
- **RAM**: 520KB SRAM
- **Flash**: 2MB+ (external)
- **Features**: Hardware multiply/divide, single-precision FPU (not used), TrustZone

## Features Demonstrated

The `ecu-rp2350b-demo` example showcases:

1. **Semantic fuel evaluation** using `runtime_semantic_evaluate_fuel`
2. **Fuel tune-derived calibration mapping** via `runtime_semantic_calibration_from_fuel_tune`
3. **Load-source behavior** (MAP and TPS/Alpha-N style paths)
4. **Pulse-width directional checks** under changed load conditions

## Binary Size

```
Flash (text):  7,592 bytes (~7.4KB)
RAM (data):        0 bytes (all static)
RAM (bss):         4 bytes
Total:         7,596 bytes
```

**Ultra-compact!** The entire VE engine fits in **under 8KB** with all features enabled.

## Compiling

### Prerequisites

```bash
# Install ARM Cortex-M33 target
rustup target add thumbv8m.main-none-eabihf

# Install probe-rs for flashing (optional)
cargo install probe-rs
```

### Build

```bash
cd ../..
cargo build -p ecu-rp2350b --release --target thumbv8m.main-none-eabihf --features example-bins --bin ecu-rp2350b-min
```

### Flash (with probe-rs)

```bash
# Minimal ECU target (safe outputs + scheduler loop)
cargo run --release --features example-bins --bin ecu-rp2350b-min

# Runtime semantic fuel demo (not board firmware)
cargo run --release --features ve-demo --bin ecu-rp2350b-demo
```

### Trigger Wiring Options

1) GPIO IRQ (IO_BANK0) — feature `capture-gpio`
- Edit `src/pinmap.rs`; `PinMap::defaults()` owns injector, ignition, and trigger GPIO numbers.
- Build with `--features "example-bins capture-gpio" --bin ecu-rp2350b-min` to enable the IO_IRQ_BANK0 handler.
- In code, `setup_trigger_irq(...)` configures rising-edge detection and unmasks the pin; ISR clears the flag and pushes a timestamp via `Rp2350Time::micros()`.

The minimal ECU loop feeds captured timestamps through the split trigger
adapter. There is no PIO capture feature in this crate until real PIO setup,
IRQ/DMA capture, timestamping, and decoder handoff exist.

## Supported Feature Profiles

- These are the supported named slices for this target; the cross-target
  inventory lives in `changes/runtime-architecture-migration/inventory.md`.
- `capture-gpio` enables the GPIO IRQ trigger-capture path in `src/bin/minimal_ecu.rs`.
- `capture-gpio` enables the GPIO IRQ trigger-capture path in `src/bin/minimal_ecu.rs`.
- `ve-demo` gates the runtime semantic fuel demo binary in `src/main.rs`.
- `example-bins` gates the minimal board bring-up binary.

## Validation Commands

- `cargo check -p ecu-rp2350b --release --target thumbv8m.main-none-eabihf --features example-bins --bin ecu-rp2350b-min`
- `cargo check -p ecu-rp2350b --release --target thumbv8m.main-none-eabihf --features "example-bins capture-gpio" --bin ecu-rp2350b-min`
- `cargo check -p ecu-rp2350b --release --target thumbv8m.main-none-eabihf --features ve-demo --bin ecu-rp2350b-demo`

Or use standard tools:
```bash
probe-rs run --chip RP2350 target/thumbv8m.main-none-eabihf/release/ecu-rp2350b
```

## Memory Map

```
┌─────────────────────────┐ 0x10000000
│  Boot2 (256 bytes)      │
├─────────────────────────┤ 0x10000100
│  Application Code       │ ~7.4KB
│  - Runtime semantic fuel evaluator │
│  - Calibration mapping bridge      │
└─────────────────────────┘

┌─────────────────────────┐ 0x20000000
│  Static Data (4 bytes)  │
│  - VE Engine state      │
└─────────────────────────┘
```

## Code Structure

```rust
fn main() -> ! {
    // Initialize hardware
    let tune = FuelRuntimeTune::new(...);
    let calibration = runtime_semantic_calibration_from_fuel_tune(&tune);
    let obs = runtime_semantic_evaluate_fuel(&calibration, input, RuntimeSemanticState::default());

    // Main loop
    loop {
        // Update semantic input from real sensors and evaluate fuel
    }
}
```

### Pin Mapping

In `src/pinmap.rs` you can change the numeric GPIO assignment:

```rust
pub const fn defaults() -> PinMap {
    PinMap {
        inj1: 0,
        inj2: 1,
        ign1: 2,
        ign2: 3,
        trigger: 4,
    }
}
```

The minimal binary validates that map at startup and validates the trigger GPIO
before constructing IRQ bit masks. Because RP HAL GPIO fields are statically
typed, the output bridge in `src/bin/minimal_ecu.rs` must still be kept aligned
with `PinMap::defaults()` when changing injector or ignition pins.

The minimal target uses the split runtime/scheduler path:
`BoardAdapter` + `ScheduledActionExecutor` + `ScheduledOutputs4`.
Embedded-hal 1.0 pins are wrapped with `Hal1ScheduledOut`.

The current minimal binary uses deterministic bring-up sensor values and
crank-only phase until real ADC/sensor/cam plumbing is added. It is intended to
prove the split scheduler output path compiles on the RP2350 target, not to
define final production sensor IO.

### Channel Mapping and Modes

- The split runtime currently emits injector and ignition channel 1 in the
  minimal bring-up path, so `ScheduledOutputs4` maps those to the second
  injector and ignition pins.
- Wider channel maps and sequential modes remain follow-up work for the split
  board adapter path.
- Wider channel examples should use the split target-common/scheduler path and
  its board adapter output types.

## Real-World Integration

For production use, integrate with:

1. **Sensor Input**: ADC for MAP/TPS/CLT/IAT
2. **Communication**: CAN/USB/UART telemetry and tune updates
3. **Triggers**: GPIO interrupts for RPM sensing
4. **Outputs**: PWM for idle control, boost control

## Why RP2350B?

Useful for ECU bring-up:

- ✅ **Fast enough**: 150MHz handles VE calculations easily
- ✅ **Big enough**: 520KB RAM for multiple tables + history
- ✅ **Cheap**: ~$1-2 in volume
- ✅ **Flexible**: Dual cores allow dedicated injection/management split
- ✅ **Well-supported**: Excellent Rust HAL and tooling
- ✅ **Programmable I/O (PIO)**: Can handle complex trigger patterns
- ✅ **Modern architecture**: ARMv8-M with TrustZone for safety

## Next Steps

1. Add board-native semantic calibration source (instead of demo defaults)
2. Implement UART/CAN communication
3. Add trigger input via PIO
4. Create tuning interface
5. Integrate with injection module

## License

Same as parent project.
