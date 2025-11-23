# STM32F4 ECU Target

This target runs the ecu-core on STM32F405 with TIM2 input-capture for trigger timing, safe output states, and a watchdog.

## Hardware
- MCU: STM32F405 (168 MHz)
- Trigger input: PA0 → TIM2_CH1 (AF1)
- Outputs: PB0 (INJ1), PB1 (INJ2), PB2 (IGN1), PB3 (IGN2)
  - Sequential support: map additional channels in code or keep batch/wasted default.

## Build & Flash
- Install ARM target: `rustup target add thumbv7em-none-eabihf`
- Flash via probe-rs (chip example):
  - `cargo flash -p stm32f4-ecu --chip STM32F405RGTx --release`

## Examples (bins)
- 4-cylinder batched + wasted spark (default):
  - `cargo run -p stm32f4-ecu --bin 4c-batched`
- V8 sequential injection + sequential ignition (channel-mapped):
  - `cargo run -p stm32f4-ecu --bin v8-seq`
  - Note: This demo configures 16 logical channels in software. To physically drive more than 4 pins, extend the outputs array you pass to `EcuApp::drive_outputs(now, &mut outputs)` and wire additional GPIOs.
- CAN heartbeat (skeleton transport):
  - `cargo run -p stm32f4-ecu --features transport-can,transport-can-fd --bin can-heartbeat`
  - Replace `DummyCan` with your HAL CAN implementation (implements `CanDevice`).
- TunerStudio gauges (custom protocol, USB CDC loop skeleton):
  - `cargo run -p stm32f4-ecu --bin ts-gauges`
  - Replace `DummySerial` with your USB CDC implementation, pass frames to `TunerstudioServer`.

### Choose Capture Path (features)
- Timer capture on TIM2 CH1 (default): `--features capture-tim`
- GPIO edge via EXTI0: `--features capture-gpio`

## Trigger Capture (TIM2)
- TIM2 prescaler set so CNT ticks at 1 µs.
- PA0 configured to AF1 (TIM2_CH1) input.
- CC1 input-capture on rising edge; TIM2 IRQ copies `CCR1` (timestamp, µs) into ring buffer.
- Main loop pops timestamps and calls `TriggerDecoder::tooth_edge_with_timestamp(ts)`.

## Safe Outputs & Scheduler
- On boot all outputs are driven LOW (safe state).
- The main loop executes `Scheduler::check_and_execute(now, &mut outputs)` to update pins.
- On persistent scheduler-full errors, outputs are latched LOW until reset.

## Watchdog & Reset
- IWDG started (~250 ms). Pet in main loop.
- Reset flags are cleared on boot (best-effort) and can be inspected.

## Notes
- Flash KV persistence (feature `flash-kv`):
  - The last 256 KiB of flash (sectors 10–11) is reserved for key/value storage. The linker script (`memory.x`) limits the firmware image to 768 KiB to prevent overlap.
  - Enable with `--features flash-kv`. TunerStudio Burn persists fuel/ignition pages (512 B each) across reset using a dual-sector, sequence-numbered layout with an atomic commit flag to tolerate abrupt power loss.
- To change pins, adjust the mappings in `src/main.rs` where PB0..PB3 are initialized. Boards with more outputs can pass a larger outputs array and configure `EcuApp::new_with_config`.
- To change pins, adjust the mappings in `src/main.rs` where PB0..PB3 are initialized. Boards with more outputs can pass a larger outputs array and configure `EcuApp::new_with_config`.
- Channel mapping: `OutputChannels` maps logical channel indices to physical outputs. For example, indices 0..7 can be injectors and 8..15 coils; arrange the `outputs` slice in the same order so `Scheduler` toggles the expected pins.
- To switch back to EXTI (edge GPIO IRQ) instead of TIM2 capture, set PA0 to input with EXTI0, unmask EXTI0 in NVIC, and in `EXTI0()` push timestamps from the 1 µs timer.
- Engine specs live in `src/engine_config.rs` (target-owned). Use with the VE engine like:

```rust
use ecu_core::ve_engine::{VeEngine, types::{InjectorConfig, VeTable, AfrTable}};
mod engine_config;

let mut ve = VeEngine::new_with(
    engine_config::injector_config(),
    engine_config::baseline_ve(),
    engine_config::afr_table(),
);
```
