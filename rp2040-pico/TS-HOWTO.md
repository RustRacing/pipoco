# RP2040 + TunerStudio HOWTO

This guide shows how to build, flash, and connect TunerStudio (TS) to the RP2040 target in this workspace.

## 1) Build Features and Targets

Recommended features:
- `capture-pio`: PIO-based trigger edge capture for low-latency timestamping.
- `flash-kv`: Persist fuel/ignition tables in a small flash region.
- `vbatt-vsys` (optional): Treat ADC29 as VSYS/3 to report battery voltage (VBATT) via OUTPC. Disable if ADC29 is an IAT thermistor.

Examples:
- With PIO capture and flash persistence:
  `cargo build -p ecu-rp2040-pico --release --features "capture-pio,flash-kv"`
- With VBATT derived from VSYS/3 on ADC29:
  `cargo build -p ecu-rp2040-pico --release --features "capture-pio,flash-kv,vbatt-vsys"`

Flash with your preferred tool (e.g., picotool, probe-rs).

## 2) Default Pin Map

- Trigger input: GPIO4 (PIO0) when `capture-pio` is enabled.
- Outputs: GPIO0..GPIO3 as INJ1/INJ2/IGN1/IGN2 (see macros at top of `rp2040-pico/src/bin/ts_ecu.rs`).
- ADC channels: GPIO26 (MAP), GPIO27 (TPS), GPIO28 (CLT), GPIO29 (IAT or VBATT depending on `vbatt-vsys`).

Adjust macros/constants if your wiring differs.

## 3) TunerStudio Project Setup

- Use `ts/IPW-ECU.ini` as your project INI.
  - Signature: `IPW-ECU V0.1`
  - OUTPC fields are defined and ordered to match firmware.
  - Pages:
    - Fuel table (page 1, 16x16, 512 bytes u16 LE)
    - Ignition table (page 2, 16x16, 512 bytes i16 LE)
    - Sensors calibration (page 3, 128 bytes)
    - AE (page 4, 16 bytes)
    - DFCO (page 5, 16 bytes)

- Connect to the RP2040’s USB serial device (CDC). VID/PID used in code: 0x2E8A:0x000A.

## 4) Basic Flow in TS

1. Connect and verify SIG (controller signature). TS should display `IPW-ECU V0.1`.
2. Watch OUTPC values change on the dashboard (RPM, MAP, TPS, CLT, IAT, VBATT, etc.).
3. Read Fuel/Ign tables (page 1/2), edit a cell, then `WritePage`.
4. Click `Burn` to save to flash (with `flash-kv` enabled). Power cycle and confirm changes persist.

## 5) Notes on Persistence

- Flash KV uses a reserved flash region (64 KiB near the end of 2 MiB). See `rp2040-pico/src/seq_kv.rs` for `REGION_OFFSET` and `REGION_SIZE`.
- Ensure your firmware does not overlap this region.
- If your board has a different flash size, adjust `REGION_OFFSET/REGION_SIZE` accordingly.

## 6) Troubleshooting

- If TS doesn’t connect: confirm USB CDC enumerates and choose the correct COM/tty.
- If OUTPC fields look wrong: confirm your project uses `ts/IPW-ECU.ini` and that the signature matches.
- If Fuel/Ign pages won’t burn: confirm `flash-kv` is enabled and your flash reservation doesn’t overlap your program.
- If VBATT is zero or bogus: check the `vbatt-vsys` feature matches your wiring on ADC29.

## 7) Optional: Trigger/Cam

- `capture-pio` enables a simple PIO program to raise IRQ on trigger edges. The firmware timestamps edges and schedules events.
- `capture-cam` enables cam phase capture on a GPIO IRQ (default pin 5). Edit constants if needed.

## 8) Safety

- Injection is gated by a master safety check: sync, flood-clear, sync-loss shutdown, rev limiter.
- Outputs latch off on repeated scheduler overflows.

## 9) Next Steps

- Edit sensors calibration (page 3) to match your MAP/TPS and thermistor curves.
- Use AE/DFCO pages to tune transient enrichment and decel fuel cut.
- For more robustness, add a proper VBATT channel or enable `vbatt-vsys`.

