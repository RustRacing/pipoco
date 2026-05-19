# RP2040 Pico: TunerStudio Gauges Example

USB CDC examples exposing live runtime data to TunerStudio. The `ts-ecu`
scheduled injector/ignition output path now uses the split
`BoardAdapter`/`ScheduledActionExecutor`/`ScheduledOutputs4` scheduler path.
TunerStudio page storage remains root `EcuState` backed until the TS
snapshot/page-store migration.

## Prerequisites
- Rust nightly or stable with embedded targets
- Targets: `rustup target add thumbv6m-none-eabi`
- Tools: `probe-rs` (optional), `elf2uf2-rs` (optional)

## Build

```
cd rust-ipw-ecu/rp2040-pico
cargo build --release --bin ts-gauges --target thumbv6m-none-eabi --features example-bins
```

## Flash
- UF2 (bootloader): hold BOOTSEL while plugging in USB, then copy the UF2 file
```
elf2uf2-rs target/thumbv6m-none-eabi/release/ts-gauges ts-gauges.uf2
# copy ts-gauges.uf2 to the RPI-RP2 drive
```
- Or with probe-rs (SWD):
```
probe-rs run --chip RP2040 target/thumbv6m-none-eabi/release/ts-gauges
```

## Connect to TunerStudio
- Open `tests/assets/IPW-ECU.ini` from the repo in TunerStudio
- Select serial port for Pico (USB CDC)
- You should see live gauges (RPM, MAP, TPS, CLT, IAT, VBATT, etc.) from the target runtime

## Notes
- Example implements a simple USB CDC loop and a live Outpc provider backed by target state
- For real data, integrate your board’s sensors and keep `EcuState` fields updated
- Tuning pages (tables) require implementing persistence on target (flash burn)
- `ts-ecu` keeps root `EcuState` for TS pages, persistence, and sensor diagnostics, but scheduled outputs are no longer driven through root `EcuApp`.
- Captured trigger timestamps are decoded through the split trigger adapter. Sensor samples and ignition control inputs use the shared split live-input snapshot for RPM/angle.
- When `capture-pio` is disabled, the local synthetic trigger fallback emits a 60-2 style missing-tooth cadence so the real trigger decoder can still reach sync during bring-up.

## Flash Burn (optional)

Enable flash-backed persistence with feature `flash-kv` to make TunerStudio “Burn” store tables in Pico flash:

```
cargo build --release -p ecu-rp2040-pico --bin ts-ecu --target thumbv6m-none-eabi --features flash-kv
```

By default, the implementation uses a reserved 4 KiB sector near the end of flash (offset 0x001F0000). Adjust for your board if needed. The storage layer remains abstract via `KvStore`, so other backends (e.g., EKV, TicKV) can be used later without changing core code.

## Supported Feature Profiles

- These are the supported named slices for this target; the cross-target
  inventory lives in `changes/runtime-architecture-migration/inventory.md`.
- `flash-kv` enables flash-backed persistence for `ts-ecu`.
- `capture-pio` enables the PIO trigger-capture path in `ts-ecu`.
- `capture-cam` enables the optional cam GPIO input path.
- `example-bins` gates standalone example binaries.

## Validation Commands

- `cargo check -p ecu-rp2040-pico --release --target thumbv6m-none-eabi --features flash-kv --bin ts-ecu`
- `cargo check -p ecu-rp2040-pico --release --target thumbv6m-none-eabi --features capture-cam --bin ts-ecu`
- `cargo check -p ecu-rp2040-pico --release --target thumbv6m-none-eabi --features example-bins --bin ts-gauges`
- `cargo check -p ecu-rp2040-pico --release --target thumbv6m-none-eabi --features "example-bins capture-pio" --bin pio_capture_example`
