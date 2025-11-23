# ECU VE Engine for Raspberry Pi RP2350B

This example demonstrates the **VE Engine** running on the Raspberry Pi RP2350B microcontroller.

## Hardware Specifications

- **MCU**: RP2350B (Raspberry Pi silicon)
- **Architecture**: Dual Cortex-M33 @ 150MHz (ARMv8-M Mainline)
- **RAM**: 520KB SRAM
- **Flash**: 2MB+ (external)
- **Features**: Hardware multiply/divide, single-precision FPU (not used), TrustZone

## Features Demonstrated

This example showcases all major VE engine capabilities:

1. **Speed-Density Calculations** - MAF-less air mass calculation
2. **VE → IPW Conversion** - Complete 16×16 table calculation
3. **Environmental Corrections** - CLT, IAT, voltage compensation
4. **Transformation System** - All safety modes:
   - Emergency Rich (+20% fuel)
   - Limp Mode (60% VE, conservative)
   - Cold Start Enrichment (+50% fuel)
   - Regional Trim (specific RPM/load ranges)
   - Transformation clearing (return to baseline)

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
cd rp2350b
cargo build --release # builds both VE demo and minimal ECU bin
```

### Flash (with probe-rs)

```bash
# VE demo (enable feature)
cargo run --release --features ve-demo --bin ecu-rp2350b-demo

# Minimal ECU target (safe outputs + scheduler loop)
cargo run --release --bin ecu-rp2350b-min

# Datalogger (sensors → CAN skeleton)
cargo run --release --bin rp2350b-datalogger-sensors-can

### Trigger Wiring Options

1) GPIO IRQ (IO_BANK0) — feature `capture-gpio`
- Edit TRIGGER_PIN_NUM and mapping macros at the top of `src/bin/minimal_ecu.rs`.
- Build with `--features capture-gpio` to enable the IO_IRQ_BANK0 handler.
- In code, `setup_trigger_irq(...)` configures rising-edge detection and unmasks the pin; ISR clears the flag and pushes a timestamp via `Rp2350Time::micros()`.

2) PIO-based Capture — feature `capture-pio`
- An isolated skeleton lives at `src/bin/pio_capture_example.rs`.
- Build with: `cargo run --release --features capture-pio --bin rp2350-pio-capture-example`.
- Intended flow: PIO samples the trigger line and raises IRQ/DMA on edges; in the IRQ, read TIMER0 microseconds and push to the ring buffer.

Both options feed `TriggerDecoder::tooth_edge_with_timestamp(ts)` through the minimal ECU loop.
```

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
│  - VE Engine            │
│  - Speed-Density        │
│  - Corrections          │
│  - Interpolation        │
└─────────────────────────┘

┌─────────────────────────┐ 0x20000000
│  Static Data (4 bytes)  │
│  - VE Engine state      │
└─────────────────────────┘
```

## Performance Estimates

At 150MHz (RP2350B max clock):

- **Single Cell IPW Calculation**: ~200 cycles (~1.3μs)
- **Full 16×16 Table**: ~51,200 cycles (~340μs)
- **With Transformations**: ~60,000 cycles (~400μs)

Real-time capability: Can update VE tables at **2.5kHz** or more!

## Code Structure

```rust
fn main() -> ! {
    // Initialize hardware
    let mut ve_engine = VeEngine::new_with(
        engine_config::injector_config(),
        engine_config::baseline_ve(),
        engine_config::afr_table(),
    );

    // Simulate sensors
    let sensors = SensorData { ... };

    // Calculate IPW table
    let ipw_table = ve_engine.calculate_ipw_table(&sensors);

    // Apply emergency mode
    ve_engine.apply_command(VeCommand::EmergencyRich { percent: 20 });

    // Main loop
    loop {
        // Update calculations based on real sensors
    }
}
```

### Arduino-style Pin Mapping

In `src/bin/minimal_ecu.rs` you can change the pin mapping at the top of the file:

```rust
// Change these macros to remap pins quickly
macro_rules! INJ1_GPIO { () => { gpio0 } }
macro_rules! INJ2_GPIO { () => { gpio1 } }
macro_rules! IGN1_GPIO { () => { gpio2 } }
macro_rules! IGN2_GPIO { () => { gpio3 } }
macro_rules! TRIGGER_GPIO { () => { gpio4 } }
```

These macros map directly to `pins.gpioX` fields when creating outputs, keeping the rest of the code unchanged. For sequential setups with more outputs, switch to `EcuApp::new_with_config` and provide a channel map and modes (Batch/Sequential, Wasted/Sequential).

### Channel Mapping and Modes

- Use `OutputChannels` to assign logical injector/ignition channels to GPIO pins.
- Example (V8): indices 0..7 = injectors, 8..15 = coils. Arrange your `outputs` slice in the same order so the scheduler toggles the expected pins.
- Choose modes via `EcuConfig`:
  - `InjectionMode::{Batch, Sequential}`
  - `IgnitionMode::{Wasted, Sequential}`
  - `firing_order: &[u8]` like `&[1,8,4,3,6,5,7,2]` (no trailing zeros needed).

## Real-World Integration

For production use, integrate with:

1. **Sensor Input**: ADC for MAP/TPS/CLT/IAT
2. **Communication**:
   - UART/SPI to send IPW table to injection module
   - CAN for distributed ECU architecture
   - USB for tuning interface
3. **Triggers**: GPIO interrupts for RPM sensing
4. **Outputs**: PWM for idle control, boost control

## Why RP2350B?

Perfect for ECU applications:

- ✅ **Fast enough**: 150MHz handles VE calculations easily
- ✅ **Big enough**: 520KB RAM for multiple tables + history
- ✅ **Cheap**: ~$1-2 in volume
- ✅ **Flexible**: Dual cores allow dedicated injection/management split
- ✅ **Well-supported**: Excellent Rust HAL and tooling
- ✅ **Programmable I/O (PIO)**: Can handle complex trigger patterns
- ✅ **Modern architecture**: ARMv8-M with TrustZone for safety

## Next Steps

1. Add ADC input for real sensors
2. Implement UART/CAN communication
3. Add trigger input via PIO
4. Create tuning interface
5. Integrate with injection module

## License

Same as parent project.
