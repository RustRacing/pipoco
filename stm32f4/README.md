# STM32F4 ECU Target

This target runs the split runtime/scheduler output path on STM32F405 with TIM2
input-capture for trigger timing, safe output states, and a watchdog.

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
  - Note: This is now a split-runtime placeholder; final V8 channel mapping
    belongs in the target-common/scheduler configuration path.
- CAN heartbeat example (no-op transport placeholder):
  - `cargo run -p stm32f4-ecu --features example-bins,transport-can,transport-can-fd --bin can-heartbeat`
  - `transport-can-fd` extends the CAN payload limit and requires `transport-can`.
- TunerStudio gauges example (USB CDC loop placeholder):
  - `cargo run -p stm32f4-ecu --bin ts-gauges`
  - Replace the `NullSerial` placeholder with your USB CDC implementation, pass frames to `TunerstudioServer`.

### Choose Capture Path (features)
- Timer capture on TIM2 CH1 (default): `--features capture-tim`
- GPIO edge via EXTI0: `--features capture-gpio`
- These are alternative capture paths for the same trigger input. Enable only one at a time.

## Supported Feature Profiles

- These are the supported named profile slices for this target; the cross-target
  inventory lives in `changes/runtime-architecture-migration/inventory.md`.
- `capture-tim` and `capture-gpio` are the two alternative trigger-capture profiles.
- `ts-usb` enables the TunerStudio serial path through `ecu-target-common`.
- `ts-usb-hw` layers the hardware bring-up on top of `ts-usb`.
- `flash-kv` enables the flash-backed KV bring-up backend.

## Validation Commands

- `cargo check -p stm32f4-ecu --release --target thumbv7em-none-eabihf --features capture-tim --bin stm32f4-ecu`
- `cargo check -p stm32f4-ecu --release --target thumbv7em-none-eabihf --features capture-gpio --bin stm32f4-ecu`
- `cargo check -p stm32f4-ecu --release --target thumbv7em-none-eabihf --features "capture-tim capture-gpio" --bin stm32f4-ecu` should fail.

## Trigger Capture (TIM2)
- TIM2 prescaler set so CNT ticks at 1 µs.
- PA0 configured to AF1 (TIM2_CH1) input.
- CC1 input-capture on rising edge; TIM2 IRQ copies `CCR1` (timestamp, µs) into ring buffer.
- Main loop pops timestamps and forwards them as split-runtime `BoardEvent::TriggerEdge` events.

## Safe Outputs & Scheduler
- On boot all outputs are driven LOW (safe state).
- The default main loop uses `BoardAdapter`, `ScheduledActionExecutor`, `ScheduledOutputs4`, and `run_split_scheduled_tick`.
- The current default main loop uses deterministic bring-up load/temperature/lambda values until real ADC/sensor plumbing is migrated.
- TunerStudio page storage remains legacy root-state-backed under TS features until the TS snapshot/page-store migration.

## Watchdog & Reset
- IWDG started (~250 ms). Pet in main loop.
- Reset flags are cleared on boot (best-effort) and can be inspected.

## Notes
- Flash KV persistence (feature `flash-kv`):
  - The last 256 KiB of flash (sectors 10–11) is reserved for key/value storage. The linker script (`memory.x`) limits the firmware image to 768 KiB to prevent overlap.
  - Enable with `--features flash-kv`. TunerStudio Burn persists fuel/ignition pages (512 B each) across reset using a dual-sector, sequence-numbered layout with an atomic commit flag to tolerate abrupt power loss.
- To change pins, adjust the mappings in `src/main.rs` where PB0..PB3 are initialized.
- The split runtime currently emits injector and ignition channel 1 in the bring-up path, so `ScheduledOutputs4` maps those to the second injector and ignition pins.
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
