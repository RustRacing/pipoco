# QEMU AVR GPIO Patch

Stock QEMU 10.2.1 boots `-M mega2560`, but ATmega GPIO and ADC are
`unimplemented-device` regions. Reads always return zero and writes are
discarded, so qtest/HMP/GDB cannot drive crank/cam/MAP pins for real firmware.

Apply `atmega-gpio-v10.2.1.patch` to a QEMU 10.2.1 tree, rebuild
`qemu-system-avr`, and run the soak runner with that binary first on `PATH`.
The patch replaces `atmega-gpio-a` ... `atmega-gpio-l` stubs with stateful AVR
GPIO port devices. Each port has normal `PINx`/`DDRx`/`PORTx` behavior and a
QOM property:

```text
/machine/mcu/gpiod external-level 0x04
```

Use QMP/HMP `qom-set` to drive external input levels. For Speeduino M5x trigger
inputs, `PD2` is crank and `PD3` is cam, so set `/machine/mcu/gpiod`
`external-level` bit 2/3 high or low over time.

Run the fallback smoke when patched GPIO is unavailable:

```bash
cargo run -p ecu-qemu-soak -- --duration-secs 600 --sample-secs 10
```

Smoke-test patched GPIO:

```bash
cargo run -p ecu-qemu-soak -- --duration-secs 10 --sample-secs 1 --pin-smoke
```

Run the preferred long driven soak with patched QEMU first on `PATH`:

```bash
cargo run -p ecu-qemu-soak -- --duration-secs 28800 --sample-secs 60 --pin-smoke --pin-drive
```

The report uses a warm RSS baseline and includes both telemetry categories:

- `firmware_runtime_telemetry: unavailable: external Speeduino firmware telemetry is opaque`
- `host_runtime_telemetry: available: host_telemetry.json`

Host telemetry includes `clock_monotonicity`, `reset_count`,
`scheduler_queue_depth`, `level0_final_state`, `level1_request_final`, and
`level2_stage_counters` plus host mirror counters. Pin-drive evidence is now
captured as `pin_drive_events`, `pin_drive_set_failures`,
`pin_drive_readback_failures`, and `last_pin_external_level`.
