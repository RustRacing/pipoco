# ecu-atmega2560

This crate is the first AVR/ATmega2560 compile target for the HAL-independent
ECU runtime path. It is a Speeduino-M5x/M50 bridge crate targeting the
M50B25TU full sequential profile: six injector channels and six coil-on-plug
ignition channels.

The pin map is schematic-derived from
`Speeduino-M5x-PCBs/m50-m40-m60_Pnp/Rev 2.3/Schematic__speeduino compatible
PCB for bosch 88pin motronic rev2.3.pdf`. It is not bench-verified yet.

The `ecu-domain` + `ecu-board-api` + `ecu-board-profiles` + `ecu-runtime` path
can be built for an ATmega2560-class AVR target and can emit fixed-size
logical injector, ignition, auxiliary, and telemetry commands.

## Schematic-Derived CPU Pins

| Function | CPU pin |
| --- | --- |
| INJ1..INJ6 | D8, D9, D10, D11, D12, D50 |
| IGN1..IGN6 | D40, D38, D52, D48, D36, D34 |
| IAT, CLT, TPS, MAP, battery, O2 | A0, A1, A2, A3, A4, A8 |
| TACH1, TACH2 | D19, D18 |
| Low-current outputs | D45, D47, D49, D51, D53 |
| Reset control | D43 |
| Trigger inputs | Crank VR1, Cam VR2 conditioned inputs |

## Build

AVR has no prebuilt `core` artifact on the local stable toolchain, so use
nightly build-std:

```sh
RUSTFLAGS='-C target-cpu=atmega2560' \
  cargo +nightly -Z build-std=core build -p ecu-atmega2560 --target avr-none
```

The board-specific HAL crate still needs to bind:

- timer input capture for the 60-2 crank sensor,
- cam input capture,
- ADC sampling for MAP, CLT, IAT, TPS, battery, and lambda,
- output compare channels for six injectors and six coil-on-plug ignition
  outputs,
- auxiliary pins for VVT, idle valve, fuel pump, fan, tach, CEL, boost, and
  spare outputs,
- nonvolatile calibration storage.
