# AVR Harness

This crate is intentionally excluded from the main workspace. It depends on
`avr-tester` plus a locally patched `simavr-ffi`, so normal workspace builds do
not require the AVR simulator toolchain.

Run it with:

```sh
tools/check_avr_harness.sh --nocapture
```

The harness covers two paths:

- a deterministic Mega2560 stub that uses the Speeduino M5x pins and closes the
  loop into the `ecu-sim-core` M50B25TU-like plant;
- a smoke replay that loads
  `aidocs/ref/code/Speeduino-M5x-PCBs/6-cyl firmware files/202305.hex` and feeds
  the same M50B25TU plant trigger/sensor stream into the AVR simulator.

The reference HEX test is a boot/wiring smoke. The project tune is stored as
separate `.msq` data, not in the firmware HEX, so output-pulse assertions stay
on the stub until tune/eeprom loading is modeled.
