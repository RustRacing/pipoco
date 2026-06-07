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
- Default features include `hardware` and `capture-tim`, so the production
  binary has a TIM2 trigger-capture path unless you explicitly disable defaults.

### Choose Capture Path (features)
- Timer capture on TIM2 CH1 (default): default features, or `--features capture-tim`
- GPIO edge via EXTI0: `--no-default-features --features "hardware capture-gpio"`
- These are alternative capture paths for the same trigger input. Enable only one at a time.
- A production build with `hardware` but neither capture feature is rejected at
  compile time so it cannot silently boot without crank/trigger capture.

## Supported Feature Profiles

- These are the supported named profile slices for this target; the cross-target
  inventory lives in `changes/runtime-architecture-migration/inventory.md`.
- `capture-tim` and `capture-gpio` are the two alternative trigger-capture profiles.
- `ts-usb` enables the TunerStudio serial path through `ecu-target-common`.
- `ts-usb-hw` layers the hardware bring-up on top of `ts-usb`.
- `flash-kv` enables the flash-backed KV bring-up backend.
- No ADC sensor feature is exposed yet; sensor input should land as concrete
  board wiring rather than synthetic target behavior.

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
- The default main loop uses `BoardAdapter`, `ScheduledActionExecutor`, `ScheduledOutputs4`, and `run_runtime_scheduled_output_tick`.
- The current default main loop uses a fixed load source until real ADC/sensor plumbing is migrated.
- TunerStudio page storage remains legacy root-state-backed under TS features until the TS snapshot/page-store migration.

## Watchdog & Reset
- IWDG starts with a documented nominal 250 ms contract: PR `/64` and RLR
  derived from a 32 kHz nominal LSI clock. Real timeout still follows STM32F4
  LSI oscillator tolerance, so hardware safety margins must account for the
  datasheet range.
- Watchdog is fed through the board adapter in the main loop.
- Reset flags are cleared on boot (best-effort) and can be inspected.

## Notes
- Flash KV persistence (feature `flash-kv`):
  - The last 256 KiB of flash (sectors 10–11) is reserved for key/value storage. The linker script (`memory.x`) limits the firmware image to 768 KiB to prevent overlap.
  - Enable with `--features flash-kv`. TunerStudio Burn persists fuel/ignition pages (512 B each) across reset using a dual-sector, sequence-numbered layout with an atomic commit flag to tolerate abrupt power loss.
  - Layout constants are centralized in `ts_support.rs` and host tests cover
    sector bounds, sequence selection, and CRC/header rejection.
- To change pins, adjust the mappings in `src/main.rs` where PB0..PB3 are initialized.
- The split runtime currently emits injector and ignition channel 1 in the bring-up path, so `ScheduledOutputs4` maps those to the second injector and ignition pins.
- To switch back to EXTI (edge GPIO IRQ) instead of TIM2 capture, set PA0 to input with EXTI0, unmask EXTI0 in NVIC, and in `EXTI0()` push timestamps from the 1 µs timer.
- The STM32 bring-up path now uses runtime semantic calibration from `EcuState`
  (`runtime_fuel_strategy_from_state`) rather than the legacy `core::ve_engine`
  injector/VE/AFR types.
