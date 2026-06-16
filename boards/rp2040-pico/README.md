# RP2040 Pico: `ts-ecu` Bring-Up Firmware

USB CDC firmware exposing live runtime data to TunerStudio. The `ts-ecu`
scheduled injector/ignition output path now uses the split
`BoardAdapter`/`ScheduledActionExecutor`/`ScheduledOutputs4` scheduler path.
TunerStudio page storage remains root `EcuState` backed until the TS
snapshot/page-store migration.

## Prerequisites
- Rust nightly or stable with embedded Rust target triples installed
- Rust target triple: `rustup target add thumbv6m-none-eabi`
- Tools: `probe-rs` (optional), `elf2uf2-rs` (optional)

## Build

```
cd rust-ipw-ecu
cargo build --release -p ecu-rp2040-pico --bin ts-ecu --target thumbv6m-none-eabi --features capture-pio
```

## Flash
- UF2 (bootloader): hold BOOTSEL while plugging in USB, then copy the UF2 file
```
elf2uf2-rs target/thumbv6m-none-eabi/release/ts-ecu ts-ecu.uf2
# copy ts-ecu.uf2 to the RPI-RP2 drive
```
- Or with probe-rs (SWD):
```
probe-rs run --chip RP2040 target/thumbv6m-none-eabi/release/ts-ecu
```

## Connect to TunerStudio
- Open `crates/compat/tests/assets/IPW-ECU.ini` from the repo in TunerStudio
- Select serial port for Pico (USB CDC)
- You should see live gauges (RPM, MAP, TPS, CLT, IAT, VBATT, etc.) from the board runtime

## Notes
- Example implements a simple USB CDC loop and a live Outpc provider backed by board state
- For real data, integrate your board’s sensors and keep `EcuState` fields updated
- Tuning pages (tables) require implementing persistence on the board (flash burn)
- `ts-ecu` keeps `EcuState` for TS pages, persistence, and sensor diagnostics; scheduled outputs use the split board adapter/scheduler path.
- Captured trigger timestamps are decoded through the split trigger adapter. Sensor samples and ignition control inputs use the shared split live-input snapshot for RPM/angle.
- Flashable `ts-ecu` builds require `capture-pio`; no-hardware trigger generation is restricted to the explicit `synthetic-trigger-demo` feature.

## Flash Burn (optional)

Enable flash-backed persistence with feature `flash-kv` to make TunerStudio “Burn” store tables in Pico flash:

```
cargo build --release -p ecu-rp2040-pico --bin ts-ecu --target thumbv6m-none-eabi --features flash-kv
```

By default, the implementation uses a reserved 4 KiB sector near the end of flash (offset 0x001F0000). Adjust for your board if needed. The storage layer remains abstract via `KvStore`, so other backends (e.g., EKV, TicKV) can be used later without changing core code.

The flash path validates the fixed layout before touching ROM erase/program
routines:

- Header plus fuel, ignition, and angle pages must fit in the reserved 4 KiB
  sector.
- Erase operations must cover sector-aligned ranges.
- Program operations must be bounded by the reserved sector and page-sized ROM
  programming granularity.
- Host `flash-kv` tests exercise the sequential-storage shim with the same
  region bounds before it reaches raw XIP reads.

USB CDC reads and writes still use the `ecu_ts::serial::SerialPort` byte-count
API for compatibility, but the RP2040 adapter now keeps `CdcSerialStats` so
read errors, write errors, and short writes are observable during bring-up.

## Supported Feature Profiles

- These are the supported named slices for this board; the cross-board
  inventory lives in `changes/runtime-architecture-migration/inventory.md`.
- `flash-kv` enables flash-backed persistence for `ts-ecu`.
- `capture-pio` enables the required PIO trigger-capture path in `ts-ecu`.
- `capture-cam` enables the optional cam GPIO input path.
- `example-bins` gates standalone example binaries.
- `synthetic-trigger-demo` enables a no-hardware trigger source for local protocol demos only; do not flash it as vehicle firmware.

## Validation Commands

- `cargo check -p ecu-rp2040-pico --release --target thumbv6m-none-eabi --features "capture-pio flash-kv" --bin ts-ecu`
- `cargo check -p ecu-rp2040-pico --release --target thumbv6m-none-eabi --features "capture-pio capture-cam" --bin ts-ecu`
- `cargo check -p ecu-rp2040-pico --release --target thumbv6m-none-eabi --features "synthetic-trigger-demo" --bin ts-ecu`
- `cargo check -p ecu-rp2040-pico --release --target thumbv6m-none-eabi --features example-bins --bin ts-gauges`
- `cargo check -p ecu-rp2040-pico --release --target thumbv6m-none-eabi --features "example-bins capture-pio" --bin pio_capture_example`
