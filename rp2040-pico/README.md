# RP2040 Pico: TunerStudio Gauges Example

Minimal USB CDC example exposing live runtime data to TunerStudio using the `ecu-core` TunerStudio server.

## Prerequisites
- Rust nightly or stable with embedded targets
- Targets: `rustup target add thumbv6m-none-eabi`
- Tools: `probe-rs` (optional), `elf2uf2-rs` (optional)

## Build

```
cd rust-ipw-ecu/rp2040-pico
cargo build --release --bin ts-gauges --target thumbv6m-none-eabi
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
- Open `ts/IPW-ECU.ini` in TunerStudio
- Select serial port for Pico (USB CDC)
- You should see live gauges (RPM, MAP, etc.) from a synthetic provider

## Notes
- Example implements a simple USB CDC loop and a fake Outpc provider for demo
- For real data, integrate your board’s sensors and feed EcuState fields
- Tuning pages (tables) require implementing persistence on target (flash burn)

## Flash Burn (optional)

Enable flash-backed persistence with feature `flash-kv` to make TunerStudio “Burn” store tables in Pico flash:

```
cargo build --release -p ecu-rp2040-pico --bin ts-ecu --target thumbv6m-none-eabi --features flash-kv
```

By default, the implementation uses a reserved 4 KiB sector near the end of flash (offset 0x001F0000). Adjust for your board if needed. The storage layer remains abstract via `KvStore`, so other backends (e.g., EKV, TicKV) can be used later without changing core code.
