# Scope

`ecu-spec` is the no_std, no_alloc semantic oracle for v1 steady-state fueling, steady-state spark scheduling, crank-angle event generation, cut-flag suppression behavior, and the minimal visible diagnostics required by the formal methods plan.

It is pure, deterministic, side-effect free, and unsafe-free. It does not model hardware drivers, interrupts, persistence, transport, TS pages, closed-loop lambda, enrichment, torque arbitration, idle control, fan control, or any behavior outside the frozen v1 boundary.

# Units

The semantic kernel uses these exact units:

- `Rpm`: `u16`
- `Micros`: `u32`
- `Kpa10`: `u16`
- `TempC10`: `i16`
- `Millivolts`: `u16`
- `Degrees10`: `u16`
- `SignedDegrees10`: `i16`
- `PulseWidthUs`: `u32`
- `AfrX100`: `u16`
- `VePctX100`: `u16`
- `RatioX1000`: `u16`

Normalized cycle-domain angles live in `[0, 7200)`.

# Fixed-Point Scales

Frozen scales are:

- VE table values: percent x100
- correction curves: ratio x1000
- AFR targets: x100
- dwell table values: microseconds
- spark table values: degrees x10
- injection target table values: degrees x10

Examples:

- `85.50% VE` is stored as `8550`
- `1.075 correction` is stored as `1075`
- `14.70 AFR` is stored as `1470`
- `15.0 degrees` is stored as `150`

# Rounding Policy

Executable v1 semantics use floor rounding everywhere except the final pulse-width clamp.

- interpolation uses floor on the final divide
- ratio composition uses floor on each multiply-divide stage
- pressure compensation uses floor
- angle conversion from microseconds uses floor
- cyclic distance uses exact integer arithmetic

# Overflow Policy

All intermediate arithmetic widens before narrowing:

- intermediate multiply uses `u64` or `i64`
- narrowing occurs only after range checks
- only final output clamping saturates

Disallowed:

- silent wraparound
- saturating interpolation internals
- saturating segment lookup arithmetic

# Clamp Policy

Semantics-level clamping occurs only at these boundaries:

- input clipping to axis range
- final corrected pulse width clamp to `[0, pw_max_us]`
- final normalized angle wrap through `norm7200`

Outside those boundaries, integer semantics are exact.

# Input-Bounds Matrix

The following matrix is frozen for every helper in `ecu-spec/src/numeric.rs`.

| Helper | Total operand range (panic-free domain) | Rounding mode | Overflow policy | Precondition discharge source |
| --- | --- | --- | --- | --- |
| `clamp_u16(value, min, max)` | total when `min <= max` | exact (no divide) | total by bound | caller typing and axis/curve/table ordering invariants |
| `clamp_u32(value, min, max)` | total when `min <= max` | exact (no divide) | total by bound | caller typing and validated limits (`pw_max_us > 0`) |
| `clamp_i32(value, min, max)` | total when `min <= max` | exact (no divide) | total by bound | caller typing and signed-domain helper contracts |
| `mul_div_floor_u32(value, mul, div)` | total when `div != 0` and `floor((value * mul) / div) <= u32::MAX` | floor | total by bound | validation invariants (`pref_kpa10 > 0`, AFR lower bound), typed nonzero constants (`1000`, `10000`) |
| `mul_div_floor_i32(value, mul, div)` | total when `div != 0` and `i32::MIN <= floor((value * mul) / div) <= i32::MAX` | floor (Euclidean) | total by bound | interpolation/angle call-site bounds from validated axes and table ranges |
| `mul_ratio_x1000(value, ratio)` | total when `floor((value * ratio_x1000) / 1000) <= u32::MAX` | floor | total by bound | `RatioX1000` calibration bounds (`0..=4000`) plus bounded pulse-width domains |
| `norm7200(value)` | total for all `i32` | exact modulo wrap | total by bound | pure numeric identity over fixed modulus `7200` |
| `cyc7200_distance(a, b)` | total for all normalized `Degrees10` (`0..7200`) | exact integer arithmetic | total by bound | `Degrees10` normalized cycle-domain contract |
| `duration_us_to_deg10(pw_us, rpm)` | total when `floor((pw_us * rpm * 6) / 100000) <= u16::MAX` | floor | total by bound | bounded pulse-width and RPM domains from validation plus caller policy |
| `pi_integrator_step_i32(acc, i_step, min_acc, max_acc)` | total when `acc,i_step,min_acc,max_acc ∈ [-4000, 4000]` and `min_acc <= max_acc` | exact integer add then inclusive clamp | total by bound | frozen PI domains from US-FM0201C (`min_acc=-2000`, `max_acc=2000`) |
| `pid_sum_clamp_i32(base, p_term, i_term, out_min, out_max)` | total when all operands in `[-8000, 8000]` and `out_min <= out_max` | exact integer add then inclusive clamp | total by bound | idle/lambda controller bounded terms and frozen output clamps from US-FM0201C |
| `deadtime_bilerp_u16(vbat_mv, fuel_pressure_kpa10)` | total when `vbat_mv ∈ [0, 18000]`, `fuel_pressure_kpa10 ∈ [0, 10000]`, deadtime cells `∈ [0, 20000]` | floor at each lerp stage | total by widened intermediates and bounded output | US-FM0201A deadtime table dimensions/ranges and validated monotone axes |
| `steinhart_ratio_q24(adc_counts, pullup_ohm)` | total when `adc_counts ∈ [1, 4094]`, `pullup_ohm ∈ [100, 1_000_000]` | floor rational | total by nonzero denominator and widened division | US-FM0201D ADC domain (`0..=4095`) with endpoint clipping before ratio stage |
| `steinhart_beta_temp_c10_q22(ln_r_ratio_q20, beta_q10, t0_k_q10)` | total when `ln_r_ratio_q20 ∈ [-3_000_000, 3_000_000]`, `beta_q10 ∈ [200_000, 800_000]`, `t0_k_q10 ∈ [2500, 4000]` | floor fixed-point divide and affine transform | total by `i64` widened intermediates and bounded narrowing | US-FM0201D thermistor integer approximation contract |
| `slew_limit_step_i32(last, candidate, max_delta)` | total when `last,candidate ∈ [-32768, 32767]`, `max_delta ∈ [0, 20000]` | exact diff then clamp | total by bound | per-sensor frozen slew limits from US-FM0201D |
| `angle_add_deg10_wrap(base_deg10, delta_deg10)` | total when `base_deg10 ∈ [0, 7199]`, `delta_deg10 ∈ [-7200, 7200]` | exact integer add with `norm7200` wrap | total by bound | normalized cycle-angle contract from v1 and US-FM0201E trigger/schedule extension |
| `angle_delta_deg10_signed(from_deg10, to_deg10)` | total when both operands in `[0, 7199]` | exact modulo-domain subtraction | total by bound | normalized cycle-angle contract from v1 and US-FM0201E comparisons |
| `debounce_counter_step_us(counter_us, sample_dt_us, threshold_us)` | total when `counter_us,threshold_us ∈ [0, 500000]`, `sample_dt_us ∈ [0, 200000]` | exact add/subtract with floor-at-zero and saturate-at-threshold | saturating counter updates are required semantic behavior | US-FM0201D plausibility debounce (`threshold_us=500000`) and trigger timing domains |

# Overflow-Safety Table

For each helper, this table freezes the widest intermediate and the Verus Lemma Table obligation that covers the safety argument.

| Helper | Worst-case widened intermediate | Verus Lemma Table obligation |
| --- | --- | --- |
| `clamp_u16` | comparisons only (no widened multiply/divide) | `lerp_boundedness` |
| `clamp_u32` | comparisons only (no widened multiply/divide) | `lerp_boundedness` |
| `clamp_i32` | comparisons only (no widened multiply/divide) | `lerp_boundedness` |
| `mul_div_floor_u32` | `(value as u64) * (mul as u64)` then `/ (div as u64)` | `duration_nonnegative` |
| `mul_div_floor_i32` | `(value as i64) * (mul as i64)` then `div_euclid(div as i64)` | `lerp_boundedness` |
| `mul_ratio_x1000` | `(value as u64) * (ratio_x1000 as u64)` then `/ 1000` | `duration_nonnegative` |
| `norm7200` | signed modulo/remainder in `i32` then range-normalized | `norm7200_in_range` |
| `cyc7200_distance` | `rem_euclid(7200)` and `7200 - forward` in cycle domain | `cyc7200_distance_bound` |
| `duration_us_to_deg10` | `(pw_us as u64) * (rpm as u64) * 6` then `/ 100000` | `duration_nonnegative` |
| `pi_integrator_step_i32` | `(acc as i64) + (i_step as i64)` then inclusive clamp to `[min_acc, max_acc]` | `lemma_pi_integrator_step_i32_bounds` |
| `pid_sum_clamp_i32` | `(base as i64) + (p_term as i64) + (i_term as i64)` then inclusive clamp | `lemma_pid_sum_clamp_i32_bounds` |
| `deadtime_bilerp_u16` | two-stage `u32`/`u64` lerp numerators over `16x16` cell/range bounds | `lemma_deadtime_bilerp_u16_bounds` |
| `steinhart_ratio_q24` | `(adc_counts as u64) * (pullup_ohm as u64)` and divide by `(adc_max - adc_counts)` | `lemma_steinhart_ratio_q24_bounds` |
| `steinhart_beta_temp_c10_q22` | `i64` affine chain over `ln_r_ratio_q20`, reciprocal temperature term, and Kelvin->Celsius shift | `lemma_steinhart_beta_temp_c10_q22_bounds` |
| `slew_limit_step_i32` | `(candidate as i64) - (last as i64)` then bounded add/subtract by `max_delta` | `lemma_slew_limit_step_i32_bounds` |
| `angle_add_deg10_wrap` | `(base as i32) + (delta as i32)` then `norm7200` | `lemma_angle_add_deg10_wrap_bounds` |
| `angle_delta_deg10_signed` | signed modular subtraction in `i32` with half-cycle fold | `lemma_angle_delta_deg10_signed_bounds` |
| `debounce_counter_step_us` | `counter ± sample_dt_us` in `u32` with floor-at-zero and threshold saturation | `lemma_debounce_counter_step_us_bounds` |

# Validation Invariants

Raw calibration values are validated before use.

Validation enforces:

- each axis length is in `2..=16`
- all used axis prefixes are strictly increasing
- each table has matching axis lengths
- curve lengths match axis lengths
- cylinder count is in `1..=8`
- used cylinder phases are in `[0, 7200)`
- `required_fuel_us > 0`
- `pref_kpa10 > 0`
- `pw_max_us > 0`
- correction curve values are in `[0, 4000]`
- VE table values are in `[0, 30000]`
- AFR targets are in `[500, 3000]`
- dwell values are in `[1, 20000]`
- spark advance table values are in `[-7200, 7200]`
- injection target table values are in `[0, 7200)`
- `target_afr_override_x100`, when present, is in `[500, 3000]`

Only the used prefixes of axes, tables, curves, and cylinder arrays are checked. Unused tail entries are ignored and may contain zero.

# Axis Clipping

Lookup inputs are clipped before interpolation.

`clip_u16(x, lo, hi) = min(max(x, lo), hi)`

For a valid axis of length `len`:

- `rpm_clip = clip_u16(rpm, rpm_axis[0], rpm_axis[len_rpm - 1])`
- `load_clip = clip_u16(load, load_axis[0], load_axis[len_load - 1])`

Clipping is internal to the lookup helpers.

# Segment Selection

Segment selection is left-closed, right-open, with the final segment closed on the upper edge.

For valid `axis` and clipped `x`, `find_segment(axis, x)` returns `i` such that:

- `0 <= i < len - 1`
- `axis[i] <= x < axis[i + 1]`, or
- `i == len - 2` and `x == axis[len - 1]`

# Interpolation

Interpolation uses floor on the final divide.

## 1D interpolation

Executable form:

- `y = y0 + floor((y1 - y0) * num / den)`

with:

- `num = x - x0`
- `den = x1 - x0`

The signed and unsigned forms are separate helper functions.

## 2D bilinear interpolation

The frozen implementation strategy is:

1. interpolate along RPM on the lower row
2. interpolate along RPM on the upper row
3. interpolate the two row results along load

This two-stage form is the only permitted bilinear strategy in v1.

# Fuel Formula

Fuel lookup inputs use RPM and load.

- `ve_pct_x100 = bilerp_2d(ve_table, rpm, load_kpa10)`
- if `target_afr_override_x100` is present, use the clamped override value
- otherwise `target_afr_x100 = bilerp_2d(afr_target_table, rpm, load_kpa10)`

Base fuel:

- `pw_base_us = floor(required_fuel_us * ve_pct_x100 / 10000)`

Air correction:

- `pw_air_us = floor(pw_base_us * map_kpa10 / pref_kpa10)`

AFR correction:

- `afr_corr_x1000 = floor(stoich_afr_x100 * 1000 / target_afr_x100)`

Other correction lookups use:

- CLT correction from `clt_c10`
- IAT correction from `iat_c10`
- Baro correction from `baro_kpa10`
- Deadtime from `vbatt_mv / 100`, interpreted as decivolts on an integer axis

Only identity trim exists in v1:

- `trim_corr_x1000 = 1000`

Corrected fuel composition:

1. `pw1 = pw_air_us`
2. `pw2 = floor(pw1 * clt_corr_x1000 / 1000)`
3. `pw3 = floor(pw2 * iat_corr_x1000 / 1000)`
4. `pw4 = floor(pw3 * baro_corr_x1000 / 1000)`
5. `pw5 = floor(pw4 * afr_corr_x1000 / 1000)`
6. `pw6 = floor(pw5 * trim_corr_x1000 / 1000)`
7. `pw7 = pw6 + deadtime_us`
8. `pw_corr_us = clamp(pw7, 0, pw_max_us)`

# Runtime Input Normalization

`step` is total over all representable `InputSnapshot` values and never returns a runtime input validation error.

Frozen runtime normalization rules:

- `rpm` is used raw by angle-duration conversion and clipped only by table lookup helpers
- `load_kpa10` is clipped only by table lookup helpers
- `map_kpa10` is used raw in pressure compensation and only bounded by final pulse-width clamp
- `clt_c10 < 0` maps to curve input `0`; otherwise cast to `u16` and clip by curve lookup helpers
- `iat_c10 < 0` maps to curve input `0`; otherwise cast to `u16` and clip by curve lookup helpers
- `baro_kpa10` is clipped only by the baro curve lookup helper
- `vbatt_mv / 100` is interpreted as decivolts and clipped only by the deadtime curve lookup helper
- `target_afr_override_x100`, when present, is clamped to `[500, 3000]` before AFR correction
- `mode`, `sync`, `fuel_cut`, and `spark_cut` only affect event emission, cut suppression, and diagnostics

# Correction Formula

The correction pipeline is fixed and ordered:

- CLT correction
- IAT correction
- Baro correction
- AFR correction
- trim correction
- deadtime addition
- final clamp

Fuel cut forces `pw_corr_us = 0`. Spark cut leaves the spark angles computed but suppresses spark event emission.

# Cut Semantics

If `fuel_cut == true`:

- `pw_corr_us = 0`
- no injection events are emitted

If `spark_cut == true`:

- spark scheduling still computes semantic spark angles
- no spark events are emitted

This keeps diagnostics and non-cut outputs observable even when events are suppressed.

# Angle Normalization

`norm7200` is defined over signed widened input and returns a `u16` in `[0, 7200)`.

The normalization rule is to add or subtract `7200` in widened signed space until the result is in range, then return the positive representative.

`cyc7200_distance(x, y)` uses exact integer arithmetic on the normalized cycle domain.

# Scheduling Semantics

Conversion from pulse width to angle:

- `duration_deg10 = floor((pw_us * rpm * 6) / 100000)`

The same helper is used for dwell duration.

Spark advance:

- `spark_advance_deg10 = bilerp_2d(spark_advance_table_deg10, rpm, load_kpa10)`

Dwell lookup:

- `dwell_us = bilerp_2d(dwell_table_us, rpm, load_kpa10)`

Injection target lookup:

- `injection_target_deg10 = bilerp_2d(injection_target_table_deg10, rpm, load_kpa10)`

Per-cylinder scheduling:

- if the injection mode is `EndOfInjection`:
  - `eoi = norm7200(phase - injection_target_deg10)`
  - `soi = norm7200(eoi - duration_deg10)`
- if the injection mode is `StartOfInjection`:
  - `soi = norm7200(phase - injection_target_deg10)`
  - `eoi = norm7200(soi + duration_deg10)`

Spark:

- `spark = norm7200(phase - spark_advance_deg10)`
- `dwell_start = norm7200(spark - dwell_duration_deg10)`

Event emission order within one cylinder is deterministic:

1. `InjectionOpen` if fuel events are enabled
2. `InjectionClose` if fuel events are enabled
3. `CoilChargeStart` if spark events are enabled
4. `CoilFire` if spark events are enabled

Across cylinders, emit in cylinder index order.

# Event Emission

Suppressed events are omitted entirely rather than emitted with sentinel angles.

If an event push returns `EventBatchFull`, `schedule_all_cylinders` stops adding events and sets `DiagnosticCode::CalibrationInvalid`. The v1 constants make this path unreachable for valid calibration, but the defensive behavior is part of the contract.

Enable and suppression rules:

- `engine_enabled = (mode == Cranking) || (mode == Running)`
- `sync_enabled = (sync == Synced)`
- `fuel_events_enabled = engine_enabled && sync_enabled && !fuel_cut && pw_corr_us > 0`
- `spark_events_enabled = engine_enabled && sync_enabled && !spark_cut`

Consequences:

- if `mode == Off`, no events are emitted
- if `mode == Shutdown`, no events are emitted
- if `sync != Synced`, no events are emitted
- if `fuel_cut == true`, no injection events are emitted
- if `spark_cut == true`, no spark events are emitted

# State Update

At the end of `step`, `next_state` updates as follows:

## MathState

- last-valid sensor fields become the current input fields
- `trim_ratio_x1000` is preserved unchanged in v1

## SchedulerState

- `pending` becomes the newly emitted event batch
- `last_cycle_epoch` is preserved unchanged in v1

## DiagState

- `unsynced = (sync != Synced)`
- `fuel_cut_active = fuel_cut`
- `spark_cut_active = spark_cut`
- `current` uses the frozen priority:
  - `FuelCutActive` if fuel cut
  - `SparkCutActive` if spark cut
  - `Unsynced` if unsynced
  - `None` otherwise

# Diagnostics

`ObservableOutput.diagnostic` must equal `next_state.diag.current`.

No other diagnostic channel exists in v1.

`CalibrationInvalid` is excluded from the normal diagnostic priority because validation prevents it in valid inputs; it is only used by the defensive event-batch overflow path.

# Tolerance Model

The oracle is exact within the chosen integer semantics.

Comparison budgets against other implementations use:

- `eps_ve_x100 = 1`
- `eps_pw_us = 1`
- `eps_angle_deg10 = 1`

These are one least-significant unit in each domain. Any larger budget requires a written justification in code comments or documentation.

# Coverage Gate (US-FM0288)

Minimum line-coverage thresholds for formal gate pass:

- `ecu-spec`: 90.0% line coverage
- `ecu-runtime` (differential reducers): 90.0% line coverage
- `ecu-scheduler` (differential reducers): 90.0% line coverage

Coverage is measured using `cargo llvm-cov` against the host test suite (`cargo test --workspace --exclude ecu-rp2040-pico --exclude stm32f4-ecu --exclude ecu-rp2350b`).

Baseline (2026-04-23):

- ecu-spec: 97.56% line, 98.56% functions, 97.19% branches
- ecu-runtime (host tests): measured separately via `cargo test -p ecu-runtime` + `cargo llvm-cov -p ecu-runtime`
- ecu-scheduler: 88.34% line, 91.76% functions, 87.15% branches (includes ecu-spec dep)

Command: `cargo llvm-cov --package <crate> --summary-only`

Note: workspace-wide `cargo llvm-cov` fails due to conflicting `critical-section` restore-state modes between stm32f4xx-hal and rp235x-hal. Coverage is measured per-package against host-only test targets (`--workspace --exclude <embedded>`).

# Benchmark Gate (US-FM0289)

Minimum benchmark regression threshold for formal gate pass:

- `ecu-spec step()` p95 runtime regression maximum: **5.0%** vs baseline
- `ecu-runtime` differential reducer p95 regression maximum: **5.0%** vs baseline
- `ecu-scheduler` differential reducer p95 regression maximum: **5.0%** vs baseline

A regression is declared when the p95 timing of the current commit exceeds `(1 + 0.05) * baseline_p95` for the same benchmark binary and inputs.

Benchmark harness location: `ecu-spec/benches/spec_bench.rs`

Benchmark command:

```
cargo bench --package ecu-spec --bench spec_bench -- --save-baseline=<name>
cargo bench --package ecu-spec --bench spec_bench -- --baseline=<name>
```

For runtime reducer benchmarks, run `cargo bench --package ecu-runtime` and `cargo bench --package ecu-scheduler` once their bench harnesses exist, using the same baseline comparison protocol.

Baseline (2026-04-23, criterion 0.4, release mode, Intel Skylake):

- `spec_step`: 678.22 ns p95 (100 samples, ~5.3M iterations) — baseline stored in `target/` via `--save-baseline=fm0289-baseline`
- `ecu-runtime step()`: 81.049 ns p95 (100 samples, ~61M iterations) — baseline stored via `--save-baseline=fm0289-baseline`
- `ecu-scheduler step()`: not yet benchmarked

# Panic-Freedom Gate (US-FM0290)

## Gate mechanism

panic-never v0.1.0 was evaluated as the primary gate mechanism. It causes linker
failure on Rust 1.91.1 when any panicking branch is present in the binary. Testing confirmed:
- panic-never v0.1.0 linked as dependency → linker failure with `undefined symbol: error(panic-never)`
- panic-never removed → linker success
- Root cause: cortex-m-rt v0.7 interrupt handlers internally call `.unwrap()`;
  panic-never's linker script unconditionally references `__rustc::rust_begin_unwind` in the
  dependency tree on this toolchain, regardless of cfg-gating.

**Tool unavailability:** panic-never is blocked on Rust 1.91.1 stable.
Documented in: `tools/TOOLING.md`.

## Equivalent gate: embedded release build + rg audit

Given the panic-never blocker, two complementary checks are used:

### 1. Full release build (rp2040-pico only)

```
# RP2040 Pico — full release build verifies complete binary compiles
cargo build -p ecu-rp2040-pico --release --target thumbv6m-none-eabi --bin ts-ecu
cargo build -p ecu-rp2040-pico --release --target thumbv6m-none-eabi --features flash-kv --bin ts-ecu
cargo build -p ecu-rp2040-pico --release --target thumbv6m-none-eabi --bin ts-gauges
cargo build -p ecu-rp2040-pico --release --target thumbv6m-none-eabi --bin pio_capture_example
```

Note: `.expect()` in binary `main()` (Peripherals::take, clock init, PIO install) is
acceptable — these are unrecoverable fatal hardware initialization failures where panic
is the intended embedded halt behavior. The semantic goal (halt-on-fatal) is preserved.

### 2. rg-based static audit

```bash
# Embedded binary sources — scan for production-code panic/unwrap/expect
rg -n "panic!\b|\.unwrap\(\)|\bexpect\(" \
    rp2040-pico/src/bin/ stm32f4/src/bin/ stm32f4/src/main.rs \
    rp2350b/src/bin/ rp2350b/src/main.rs \
    2>/dev/null | grep -v "^\s*//" | grep -v "^\s*#\[" | grep -v "mod tests" | grep -v "cfg(test)"

# Production library sources — scan for panic/unwrap/expect
rg -n "panic!\b|\.unwrap\(\)|\bexpect\(" \
    ecu-runtime/src/ ecu-scheduler/src/ ecu-compat/src/ \
    2>/dev/null | grep -v "cfg(test)" | grep -v "mod tests" | grep -v "^\s*//"
```

Pass criteria: zero matches in production paths (non-test, non-#[cfg(test)]).

### 3. panic-never opt-in for isolated verification (when toolchain supports it)

In `tools/TOOLING.md`, a verification step is documented that can be run manually
when a compatible toolchain is available:

```bash
# Add to Cargo.toml [dependencies] of each embedded binary crate:
panic-never = "0.1"

# In each binary's main source, replace the panic handler with:
#[cfg(not(debug_assertions))]
use panic_never as _;
#[cfg(debug_assertions)]
use panic_halt as _;

# Then build — linker failure = panic found, linker success = panic-free
cargo build -p ecu-rp2040-pico --release --target thumbv6m-none-eabi --bin ts-ecu
```

Current status: linker fails on Rust 1.91.1 even with no user-space panics (cortex-m-rt issue).
This step will succeed when a toolchain-version-compatible panic-never is available.

Proof Obligations

The proof obligations for v1 are:

- segment bounds
- interpolation breakpoint exactness
- interpolation boundedness
- edge continuity
- constant-table reproduction
- bilinear-surface reproduction
- angle normalization range
- duration nonnegativity
- cut suppression
- deterministic step
- scheduler ordering

The helper and proof surfaces are frozen to support these obligations without changing production semantics.

# Phase-2 Shape Freeze (US-FM0201)

This section freezes only Phase-2 shapes, enums, dimensions, matrix headers, tolerances, and gate thresholds. Formula and behavior bodies are frozen by US-FM0201A..US-FM0201F.

## (a) LogicalState additive delta

`LogicalState` is extended additively with exactly these fields:

| Field | Type | Reset value |
| --- | --- | --- |
| `ae_state` | `AeState` | `AeState { active: false, pulse_us: 0, decay_steps_remaining: 0 }` |
| `idle_integrator_state` | `PiIntegratorState` | `PiIntegratorState::zero()` |
| `lambda_integrator_state` | `PiIntegratorState` | `PiIntegratorState::zero()` |
| `knock_state` | `KnockState` | `KnockState { retard_deg10: 0, recovery_counter: 0, detected: false }` |
| `launch_state` | `LaunchState` | `LaunchState { active: false, cut_cycle_count: 0 }` |
| `flat_shift_state` | `FlatShiftState` | `FlatShiftState { active: false, cut_cycle_count: 0 }` |
| `trigger_state` | `TriggerDecoderState` | `TriggerDecoderState::Unsynced` |
| `rpm_estimate` | `Rpm` | `0` |
| `sync_state` | `TriggerSyncState` | `TriggerSyncState::NoSync` |
| `stall_counter` | `u16` | `0` |
| `safety_latched` | `bool` | `false` |
| `cut_reason_code` | `u8` | `0` |
| `knock_intensity_x100` | `u16` | `0` |
| `idle_duty_x1000` | `u16` | `0` |
| `lambda_correction_x1000` | `u16` | `1000` |
| `torque_request_x1000` | `u16` | `1000` |
| `torque_allowed_x1000` | `u16` | `1000` |
| `torque_actuated_x1000` | `u16` | `1000` |

## (b) Frozen cut/limit priority enum

`cut_reason_code` uses this frozen priority and encoding (lower code = higher priority):

| Code | Variant | Meaning |
| --- | --- | --- |
| `0` | `None` | no cut active |
| `1` | `SafetyLatched` | safety latch forced cut |
| `2` | `HardRevLimit` | hard rev limiter cut |
| `3` | `LaunchCut` | launch control cut |
| `4` | `FlatShiftCut` | flat-shift cut |
| `5` | `DfcoCut` | decel fuel cut |
| `6` | `SoftRevSparkCut` | soft rev limiter spark cut |
| `7` | `KnockSparkRetardOnly` | knock response active (no fuel cut) |

This order is the only legal arbitration order for US-FM0217.

## (c) PI integrator state type

Idle and lambda closed-loop share this frozen type:

```rust
pub struct PiIntegratorState {
    pub acc: i32,
    pub min_acc: i32,
    pub max_acc: i32,
    pub frozen: bool,
}
```

Freeze/clamp semantics:

- update form: `acc_next = clamp(acc + error_term, min_acc, max_acc)` when `frozen == false`
- when `frozen == true`, `acc_next = acc`
- clamp is inclusive and idempotent at both limits
- `PiIntegratorState::zero()` is `acc = 0`, `min_acc = -2000`, `max_acc = 2000`, `frozen = false`

## (d) Injector deadtime table shape

Deadtime uses a 2-D table:

- dimensions: `16 x 16`
- x-axis label and unit: `vbat_mv` (`Millivolts`)
- y-axis label and unit: `fuel_pressure_kpa10` (`Kpa10`)
- cell unit: microseconds (`u16`)
- interpolation policy: two-stage floor bilinear

## (e) Sensor table and curve shapes

| Sensor path | Shape | Input axis label(s) | Output unit |
| --- | --- | --- | --- |
| CLT thermistor | 1-D curve, 16 points | `adc_counts` | `TempC10` |
| IAT thermistor | 1-D curve, 16 points | `adc_counts` | `TempC10` |
| MAP linear | 1-D linear table, 2 points | `adc_counts` | `Kpa10` |
| TPS linear | 1-D linear table, 2 points | `adc_counts` | `u16` percent x100 |
| MAF piecewise linear | 1-D table, 16 points | `adc_counts` | mass-flow x100 |
| O2/lambda wideband linear | 1-D linear table, 2 points | `adc_counts` | `AfrX100` |
| O2 narrowband switch | threshold + hysteresis scalars | `adc_counts` | `AfrX100` equivalent |
| Knock intensity map | 1-D table, 16 points | `window_energy` | x100 intensity |
| Baro linear | 1-D linear table, 2 points | `adc_counts` | `Kpa10` |
| Vbat linear | 1-D linear table, 2 points | `adc_counts` | `Millivolts` |

## (f) Trigger decoder state enum and RPM bound

Frozen decoder state:

| Variant | Meaning |
| --- | --- |
| `NoSync` | no valid wheel pattern acquired |
| `PreSync` | candidate gap observed, awaiting confirmation |
| `Synced` | 60-2 sync established |
| `SyncLoss` | prior sync invalidated pending resync |

Frozen bound:

- `rpm_estimate` semantic range: `0..=20000`

## (g) Verus Lemma Table (Phase-2 header rows, frozen order)

| Order | Spec function | Lemma name slot | Owning story |
| --- | --- | --- | --- |
| 1 | `deadtime_lookup` | `lemma_deadtime_lookup_*` | `US-FM0201A` |
| 2 | `vbat_correction` | `lemma_vbat_correction_*` | `US-FM0201A` |
| 3 | `baro_correction` | `lemma_baro_correction_*` | `US-FM0201A` |
| 4 | `cranking_enrichment` | `lemma_cranking_enrichment_*` | `US-FM0201A` |
| 5 | `afterstart_correction` | `lemma_afterstart_correction_*` | `US-FM0201A` |
| 6 | `warmup_correction` | `lemma_warmup_correction_*` | `US-FM0201A` |
| 7 | `ae_step` | `lemma_ae_step_*` | `US-FM0201A` |
| 8 | `dfco_state_step` | `lemma_dfco_state_step_*` | `US-FM0201B` |
| 9 | `rev_limit_step` | `lemma_rev_limit_step_*` | `US-FM0201B` |
| 10 | `knock_step` | `lemma_knock_step_*` | `US-FM0201B` |
| 11 | `launch_step` | `lemma_launch_step_*` | `US-FM0201B` |
| 12 | `flat_shift_step` | `lemma_flat_shift_step_*` | `US-FM0201B` |
| 13 | `arbiter_step` | `lemma_arbiter_step_*` | `US-FM0201B` |
| 14 | `idle_pi_step` | `lemma_idle_pi_step_*` | `US-FM0201C` |
| 15 | `lambda_pi_step` | `lemma_lambda_pi_step_*` | `US-FM0201C` |
| 16 | `clt_from_counts` | `lemma_clt_from_counts_*` | `US-FM0201D` |
| 17 | `iat_from_counts` | `lemma_iat_from_counts_*` | `US-FM0201D` |
| 18 | `map_from_counts` | `lemma_map_from_counts_*` | `US-FM0201D` |
| 19 | `tps_from_counts` | `lemma_tps_from_counts_*` | `US-FM0201D` |
| 20 | `maf_from_counts` | `lemma_maf_from_counts_*` | `US-FM0201D` |
| 21 | `o2_from_counts` | `lemma_o2_from_counts_*` | `US-FM0201D` |
| 22 | `knock_from_window` | `lemma_knock_from_window_*` | `US-FM0201D` |
| 23 | `baro_from_counts` | `lemma_baro_from_counts_*` | `US-FM0201D` |
| 24 | `vbat_from_counts` | `lemma_vbat_from_counts_*` | `US-FM0201D` |
| 25 | `sensor_plausibility_step` | `lemma_sensor_plausibility_step_*` | `US-FM0201D` |
| 26 | `sensor_slew_step` | `lemma_sensor_slew_step_*` | `US-FM0201D` |
| 27 | `trigger_60_2_step` | `lemma_trigger_60_2_step_*` | `US-FM0201E` |
| 28 | `trigger_sync_loss_step` | `lemma_trigger_sync_loss_step_*` | `US-FM0201E` |
| 29 | `cam_phase_step` | `lemma_cam_phase_step_*` | `US-FM0201E` |
| 30 | `stall_detection_step` | `lemma_stall_detection_step_*` | `US-FM0201E` |
| 31 | `scheduler_cancel_on_sync_loss` | `lemma_scheduler_cancel_on_sync_loss_*` | `US-FM0201E` |
| 32 | `torque_pipeline_step` | `lemma_torque_pipeline_step_*` | `US-FM0201E` |
| 33 | `persist_encode` | `lemma_persist_encode_*` | `US-FM0201F` |
| 34 | `persist_decode` | `lemma_persist_decode_*` | `US-FM0201F` |
| 35 | `persist_migrate` | `lemma_persist_migrate_*` | `US-FM0201F` |
| 36 | `persist_factory_reset` | `lemma_persist_factory_reset_*` | `US-FM0201F` |
| 37 | `ts_page_meta` | `lemma_ts_page_meta_*` | `US-FM0201F` |
| 38 | `ts_outpc_encode` | `lemma_ts_outpc_encode_*` | `US-FM0201F` |
| 39 | `ts_dispatch_step` | `lemma_ts_dispatch_step_*` | `US-FM0201F` |
| 40 | `ts_burn_save_step` | `lemma_ts_burn_save_step_*` | `US-FM0201F` |
| 41 | `ts_diag_ring_step` | `lemma_ts_diag_ring_step_*` | `US-FM0201F` |

## (h) Kani Harness Matrix (Phase-2 header rows)

| Order | Harness name | Spec function | Symbolic domain | Unwind bound | Owning story |
| --- | --- | --- | --- | --- | --- |
| 1 | `kani_deadtime_lookup_total` | `deadtime_lookup` | `vbat_mv 0..=18000`, `fuel_pressure_kpa10 0..=10000` | `16` | `US-FM0201A` |
| 2 | `kani_vbat_correction_total` | `vbat_correction` | `vbat_mv 0..=18000` | `16` | `US-FM0201A` |
| 3 | `kani_baro_correction_total` | `baro_correction` | `baro_kpa10 0..=3000` | `16` | `US-FM0201A` |
| 4 | `kani_cranking_total` | `cranking_enrichment` | symbolic cranking inputs in validated ranges | `16` | `US-FM0201A` |
| 5 | `kani_afterstart_total` | `afterstart_correction` | `cycles_since_start 0..=65535`, CLT range | `32` | `US-FM0201A` |
| 6 | `kani_warmup_total` | `warmup_correction` | CLT full range | `16` | `US-FM0201A` |
| 7 | `kani_ae_total` | `ae_step` | TPS/MAP deltas full i16 range | `32` | `US-FM0201A` |
| 8 | `kani_cuts_arbiter_total` | `arbiter_step` | all cut inputs symbolic bool/int domains | `32` | `US-FM0201B` |
| 9 | `kani_idle_pi_total` | `idle_pi_step` | signed RPM error ±4000 | `32` | `US-FM0201C` |
| 10 | `kani_lambda_pi_total` | `lambda_pi_step` | lambda error ±1000 with freeze gates | `32` | `US-FM0201C` |
| 11 | `kani_sensor_curves_total` | sensor curve functions | adc counts `0..=4095` | `16` | `US-FM0201D` |
| 12 | `kani_sensor_plausibility_total` | `sensor_plausibility_step` | symbolic sensor streams | `32` | `US-FM0201D` |
| 13 | `kani_sensor_slew_total` | `sensor_slew_step` | symbolic per-step deltas | `32` | `US-FM0201D` |
| 14 | `kani_trigger_decoder_total` | `trigger_60_2_step` | tooth intervals `50..=200000` us | `128` | `US-FM0201E` |
| 15 | `kani_trigger_sync_loss_total` | `trigger_sync_loss_step` | symbolic tooth stream with gaps | `128` | `US-FM0201E` |
| 16 | `kani_cam_phase_option_total` | `cam_phase_step` | `Option<CamTooth>` symbolic | `64` | `US-FM0201E` |
| 17 | `kani_stall_total` | `stall_detection_step` | timeout counters symbolic | `64` | `US-FM0201E` |
| 18 | `kani_torque_pipeline_total` | `torque_pipeline_step` | request/limit symbolic x1000 | `32` | `US-FM0201E` |
| 19 | `kani_persist_total` | persist functions | page bytes symbolic within page sizes | `64` | `US-FM0201F` |
| 20 | `kani_ts_proto_total` | TS functions | command bytes symbolic in `0..=64` length domain | `64` | `US-FM0201F` |

## (i) Property-Test Matrix (Phase-2 header rows)

| Order | Property row name | Generator domain | Rejections | Assertion target | Owning story |
| --- | --- | --- | --- | --- | --- |
| 1 | `prop_deadtime_monotone_voltage` | valid deadtime table + voltage sweep | invalid axis ordering | monotone/interpolated bounds | `US-FM0201A` |
| 2 | `prop_vbat_correction_bounds` | voltage sweep | none | correction in frozen bounds | `US-FM0201A` |
| 3 | `prop_baro_correction_bounds` | baro sweep | none | correction in frozen bounds | `US-FM0201A` |
| 4 | `prop_enrichment_ordering` | symbolic enrichment components | invalid calibration | frozen composition order | `US-FM0201A` |
| 5 | `prop_arbiter_priority_total` | all cut inputs symbolic | none | output matches frozen order | `US-FM0201B` |
| 6 | `prop_pi_clamp_idempotent` | integrator states/errors symbolic | none | clamp/freeze idempotence | `US-FM0201C` |
| 7 | `prop_sensor_curve_clamp` | adc counts sweep | none | outputs remain within bounds | `US-FM0201D` |
| 8 | `prop_sensor_slew_limit` | sensor streams | none | bounded per-step deltas | `US-FM0201D` |
| 9 | `prop_trigger_sync_totality` | tooth interval streams | malformed impossible streams | total sync-state transition | `US-FM0201E` |
| 10 | `prop_cam_none_passthrough` | `Option<CamTooth>` streams | none | deterministic None behavior | `US-FM0201E` |
| 11 | `prop_persist_roundtrip` | representable pages | invalid CRC in decode-only assertions | `decode(encode(x)) == x` | `US-FM0201F` |
| 12 | `prop_ts_dispatch_totality` | command byte streams | none | total response or typed error | `US-FM0201F` |

## (j) Runtime differential mapping fields and epsilon

Every runtime differential fixture for US-FM0267..US-FM0281 compares these additional fields:

- `duty_x1000`
- `correction_x1000`
- `advance_deg10_trim`
- `rpm_estimate`
- `sync_state`
- `knock_intensity_x100`
- `idle_integrator_state`
- `lambda_integrator_state`
- `cut_reason_code`
- `fuel_cut`
- `spark_cut`
- `torque_request_x1000`
- `torque_allowed_x1000`
- `torque_actuated_x1000`

Frozen tolerances for new fields:

| Field | Epsilon |
| --- | --- |
| `duty_x1000` | `1` |
| `correction_x1000` | `1` |
| `advance_deg10_trim` | `1` |
| `rpm_estimate` | `1` |
| `sync_state` | `0` (exact enum match) |
| `knock_intensity_x100` | `1` |
| `idle_integrator_state` | `0` (exact state match) |
| `lambda_integrator_state` | `0` (exact state match) |
| `cut_reason_code` | `0` (exact code match) |
| `fuel_cut` | `0` (exact bool match) |
| `spark_cut` | `0` (exact bool match) |
| `torque_request_x1000` | `1` |
| `torque_allowed_x1000` | `1` |
| `torque_actuated_x1000` | `1` |

## (k) Gate thresholds for US-FM0288 and US-FM0289

Frozen gate thresholds:

- minimum line coverage for `ecu-spec` + differential reducers: `90.0%`
- maximum allowed benchmark regression for `step()` and differential reducers (p95 runtime): `5.0%`

# Phase-2 Domain Freeze: Fuel Enrichment and Compensation (US-FM0201A)

This section freezes the full behavior for deadtime, Vbat compensation, baro compensation, cranking, after-start enrichment, warm-up enrichment, and AE.

All US-FM0201A equations are integer-only with floor semantics on every multiply-divide stage.

## US-FM0201A.1 Deadtime

- units:
  - input `vbat_mv`: `Millivolts`
  - input `fuel_pressure_kpa10`: `Kpa10`
  - output `deadtime_us`: `PulseWidthUs` (stored from `u16` table cells)
- fixed-point and shape:
  - table unit is microseconds
  - interpolation is two-stage floor bilinear over frozen `16 x 16` table
- validation invariants:
  - voltage axis and pressure axis are strictly increasing
  - both axes use lengths in `2..=16`
  - all deadtime cells are in `0..=20000`
- formula:
  - `deadtime_us = bilerp_floor(deadtime_table_us, vbat_mv, fuel_pressure_kpa10)`
- integration order into corrected pulse:
  - deadtime is added after all multiplicative corrections and additive AE pulse
  - `pw_with_deadtime = pw_after_ae + deadtime_us`
- state update:
  - none
- diagnostics:
  - invalid deadtime table shape/range produces `CalibrationInvalid` during validation
- proof obligations:
  - Verus lemma slot: `lemma_deadtime_lookup_*`
  - Kani harness: `kani_deadtime_lookup_total`
  - property row: `prop_deadtime_monotone_voltage`

## US-FM0201A.2 Vbat compensation

- units:
  - input `vbat_mv`: `Millivolts`
  - output `vbat_corr_x1000`: `RatioX1000`
- fixed-point and shape:
  - 1-D correction curve, ratio scale x1000
- validation invariants:
  - axis strictly increasing, length in `2..=16`
  - values in `0..=4000`
  - nominal point `12000 mV` maps to `1000` in the default calibration
- formula:
  - `vbat_corr_x1000 = lerp_floor(vbat_corr_curve, vbat_mv)`
- integration order into corrected pulse:
  - applied after baro correction and before cranking/after-start/warm-up/AE
  - `pw_vbat = floor(pw_baro * vbat_corr_x1000 / 1000)`
- state update:
  - none
- diagnostics:
  - invalid curve shape/range produces `CalibrationInvalid` during validation
- proof obligations:
  - Verus lemma slot: `lemma_vbat_correction_*`
  - Kani harness: `kani_vbat_correction_total`
  - property row: `prop_vbat_correction_bounds`

## US-FM0201A.3 Barometric compensation

- units:
  - input `baro_kpa10`: `Kpa10`
  - output `baro_corr_x1000`: `RatioX1000`
- fixed-point and shape:
  - 1-D correction curve, ratio scale x1000
- validation invariants:
  - axis strictly increasing, length in `2..=16`
  - values in `0..=4000`
  - nominal point `1000 kPa10` maps to `1000` in the default calibration
- formula:
  - `baro_corr_x1000 = lerp_floor(baro_corr_curve, baro_kpa10)`
- integration order into corrected pulse:
  - first multiplicative correction after `pw_air_us`
  - `pw_baro = floor(pw_air_us * baro_corr_x1000 / 1000)`
- state update:
  - none
- diagnostics:
  - invalid curve shape/range produces `CalibrationInvalid` during validation
- proof obligations:
  - Verus lemma slot: `lemma_baro_correction_*`
  - Kani harness: `kani_baro_correction_total`
  - property row: `prop_baro_correction_bounds`

## US-FM0201A.4 Cranking enrichment

- units:
  - input `clt_c10`: `TempC10`
  - output `cranking_corr_x1000`: `RatioX1000`
- fixed-point and shape:
  - 1-D curve indexed by CLT, ratio scale x1000
  - active only when `mode == Cranking`
- validation invariants:
  - axis strictly increasing, length in `2..=16`
  - values in `0..=6000`
- formula:
  - if `mode == Cranking`: `cranking_corr_x1000 = lerp_floor(cranking_curve, clt_c10)`
  - else: `cranking_corr_x1000 = 1000`
- integration order into corrected pulse:
  - applied after Vbat compensation and before after-start enrichment
  - `pw_cranking = floor(pw_vbat * cranking_corr_x1000 / 1000)`
- state update:
  - none
- diagnostics:
  - none beyond validation
- proof obligations:
  - Verus lemma slot: `lemma_cranking_enrichment_*`
  - Kani harness: `kani_cranking_total`
  - property row: `prop_enrichment_ordering`

## US-FM0201A.5 After-start enrichment

- units:
  - input `cycles_since_start`: `u16`
  - input `clt_c10`: `TempC10`
  - output `afterstart_corr_x1000`: `RatioX1000`
- fixed-point and shape:
  - 2-D table over cycle count and CLT, ratio scale x1000, dimensions `16 x 16`
  - decay-by-cycle behavior is table-defined and deterministic
- validation invariants:
  - both axes strictly increasing, lengths in `2..=16`
  - values in `0..=4000`
- formula:
  - if `mode == Running` and `cycles_since_start <= afterstart_window_cycles`:
    - `afterstart_corr_x1000 = bilerp_floor(afterstart_table, cycles_since_start, clt_c10)`
  - else:
    - `afterstart_corr_x1000 = 1000`
- integration order into corrected pulse:
  - applied after cranking correction and before warm-up correction
  - `pw_afterstart = floor(pw_cranking * afterstart_corr_x1000 / 1000)`
- state update:
  - increment `cycles_since_start` once per successful `step` while engine-enabled
- diagnostics:
  - invalid table shape/range produces `CalibrationInvalid` during validation
- proof obligations:
  - Verus lemma slot: `lemma_afterstart_correction_*`
  - Kani harness: `kani_afterstart_total`
  - property row: `prop_enrichment_ordering`

## US-FM0201A.6 Warm-up enrichment

- units:
  - input `clt_c10`: `TempC10`
  - output `warmup_corr_x1000`: `RatioX1000`
- fixed-point and shape:
  - 1-D curve indexed by CLT, ratio scale x1000
- validation invariants:
  - axis strictly increasing, length in `2..=16`
  - values in `0..=4000`
- formula:
  - `warmup_corr_x1000 = lerp_floor(warmup_curve, clt_c10)`
- integration order into corrected pulse:
  - applied after after-start correction and before CLT/IAT/AFR/trim corrections
  - `pw_warmup = floor(pw_afterstart * warmup_corr_x1000 / 1000)`
- state update:
  - none
- diagnostics:
  - invalid curve shape/range produces `CalibrationInvalid` during validation
- proof obligations:
  - Verus lemma slot: `lemma_warmup_correction_*`
  - Kani harness: `kani_warmup_total`
  - property row: `prop_enrichment_ordering`

## US-FM0201A.7 Acceleration enrichment (AE)

- units:
  - inputs: `tps_delta_x100` (`i16`), `map_delta_kpa10` (`i16`)
  - state: `AeState { active: bool, pulse_us: u32, decay_steps_remaining: u16 }`
  - output: `ae_pulse_us` additive `PulseWidthUs`
- fixed-point and shape:
  - trigger threshold tables are 1-D `16`-point curves over RPM or load bins
  - decay table is 1-D `16`-point curve indexed by elapsed AE steps
  - AE is additive in microseconds, not multiplicative ratio
- validation invariants:
  - all thresholds and decay-curve entries are in `0..=20000`
  - AE trigger axis lengths in `2..=16`, strictly increasing
- formula:
  - trigger condition: `abs(tps_delta_x100) >= tps_ae_threshold || abs(map_delta_kpa10) >= map_ae_threshold`
  - if triggered:
    - `ae_pulse_us = ae_shot_lookup(...)`
    - `decay_steps_remaining = ae_decay_steps_lookup(...)`
  - else if `decay_steps_remaining > 0`:
    - `ae_pulse_us = floor(prev_ae_pulse_us * decay_ratio_x1000 / 1000)`
    - `decay_steps_remaining = decay_steps_remaining - 1`
  - else:
    - `ae_pulse_us = 0`
- integration order into corrected pulse:
  - additive step after all multiplicative corrections and before deadtime
  - `pw_after_ae = pw_trimmed + ae_pulse_us`
- state update:
  - `ae_state` is updated deterministically each step from current deltas and prior `ae_state`
- diagnostics:
  - AE itself does not emit cut or fault diagnostics
  - invalid AE tables/curves produce `CalibrationInvalid` during validation
- proof obligations:
  - Verus lemma slot: `lemma_ae_step_*`
  - Kani harness: `kani_ae_total`
  - property row: `prop_enrichment_ordering`

## US-FM0201A.8 Frozen corrected pulse-width composition order

Phase-2 corrected fuel uses this exact order:

1. `pw_air = floor(pw_base * map_kpa10 / pref_kpa10)`
2. `pw_baro = floor(pw_air * baro_corr_x1000 / 1000)`
3. `pw_vbat = floor(pw_baro * vbat_corr_x1000 / 1000)`
4. `pw_cranking = floor(pw_vbat * cranking_corr_x1000 / 1000)`
5. `pw_afterstart = floor(pw_cranking * afterstart_corr_x1000 / 1000)`
6. `pw_warmup = floor(pw_afterstart * warmup_corr_x1000 / 1000)`
7. `pw_clt = floor(pw_warmup * clt_corr_x1000 / 1000)`
8. `pw_iat = floor(pw_clt * iat_corr_x1000 / 1000)`
9. `pw_afr = floor(pw_iat * afr_corr_x1000 / 1000)`
10. `pw_trim = floor(pw_afr * trim_corr_x1000 / 1000)`
11. `pw_ae = pw_trim + ae_pulse_us`
12. `pw_deadtime = pw_ae + deadtime_us`
13. `pw_corr_us = clamp(pw_deadtime, 0, pw_max_us)`

Default v3 calibration for new US-FM0201A factors is identity for FM0016 compatibility:

- `vbat_corr_x1000 = 1000`
- `baro_corr_x1000 = 1000`
- `cranking_corr_x1000 = 1000` outside cranking
- `afterstart_corr_x1000 = 1000`
- `warmup_corr_x1000 = 1000`
- `ae_pulse_us = 0`

# Phase-2 Domain Freeze: Cuts, Limiters, and Arbitration (US-FM0201B)

This section freezes full behavior for DFCO, rev limiter (soft/hard), launch control, flat shift, knock suppression, unified arbiter priority, safety latching, cut reason encoding, and StepResult field expansion.

All US-FM0201B behavior is integer-only and deterministic.

## US-FM0201B.1 DFCO

- units:
  - input `rpm`: `Rpm`
  - input `tps_x100`: percent x100 (`u16`)
  - input `map_kpa10`: `Kpa10`
  - output `dfco_fuel_cut`: `bool`
- validation invariants:
  - `dfco_entry_rpm > dfco_exit_rpm`
  - `dfco_entry_tps_x100 <= dfco_exit_tps_x100`
  - `dfco_delay_cycles in 0..=65535`
- formula and hysteresis:
  - enter DFCO when `rpm >= dfco_entry_rpm && tps_x100 <= dfco_entry_tps_x100 && map_kpa10 <= dfco_entry_map_kpa10` for `dfco_delay_cycles` consecutive enabled cycles
  - remain in DFCO while `rpm > dfco_exit_rpm && tps_x100 <= dfco_exit_tps_x100`
  - exit DFCO otherwise
- state update:
  - update `dfco_active` latch and `dfco_qualify_counter`
- diagnostics:
  - no standalone diagnostic code; effect is represented by arbiter `cut_reason_code = 5` when selected
- proof obligations:
  - Verus lemma slot: `lemma_dfco_state_step_*`
  - Kani harness: `kani_cuts_arbiter_total`
  - property row: `prop_arbiter_priority_total`

## US-FM0201B.2 Rev limiter (soft/hard)

- units:
  - input `rpm`: `Rpm`
  - output `soft_rev_spark_cut`: `bool`
  - output `hard_rev_fuel_cut`: `bool`
  - output `soft_retard_deg10`: `i16`
- validation invariants:
  - `hard_rev_rpm > soft_rev_rpm`
  - `rev_hysteresis_rpm > 0`
  - `soft_retard_max_deg10 in 0..=720`
- formula and hysteresis:
  - hard limit active when `rpm >= hard_rev_rpm`, released when `rpm <= hard_rev_rpm - rev_hysteresis_rpm`
  - soft limit active when `rpm >= soft_rev_rpm`, released when `rpm <= soft_rev_rpm - rev_hysteresis_rpm`
  - when hard active: force fuel cut; when soft active without hard: spark cut/retard only
- state update:
  - update soft/hard limiter latches
- diagnostics:
  - represented through arbiter code `2` (`HardRevLimit`) or `6` (`SoftRevSparkCut`)
- proof obligations:
  - Verus lemma slot: `lemma_rev_limit_step_*`
  - Kani harness: `kani_cuts_arbiter_total`
  - property row: `prop_arbiter_priority_total`

## US-FM0201B.3 Launch control

- units:
  - input `launch_armed`: `bool`
  - input `rpm`: `Rpm`
  - output `launch_cut`: `bool`
- validation invariants:
  - `launch_rpm_limit in 0..=20000`
  - `launch_cut_cycles in 0..=65535`
- formula:
  - launch subsystem is active only when `launch_armed == true`
  - while active, assert cut when `rpm >= launch_rpm_limit` using cycle-bounded cut pattern
- state update:
  - update `launch_state { active, cut_cycle_count }`
- diagnostics:
  - represented through arbiter code `3` (`LaunchCut`) when selected
- proof obligations:
  - Verus lemma slot: `lemma_launch_step_*`
  - Kani harness: `kani_cuts_arbiter_total`
  - property row: `prop_arbiter_priority_total`

## US-FM0201B.4 Flat shift

- units:
  - input `flat_shift_armed`: `bool`
  - input `rpm`: `Rpm`
  - output `flat_shift_cut`: `bool`
- validation invariants:
  - `flat_shift_rpm_min in 0..=20000`
  - `flat_shift_cut_cycles in 0..=65535`
- formula:
  - active only when `flat_shift_armed == true && rpm >= flat_shift_rpm_min`
  - apply cycle-bounded spark/fuel cut pattern while active
- state update:
  - update `flat_shift_state { active, cut_cycle_count }`
- diagnostics:
  - represented through arbiter code `4` (`FlatShiftCut`) when selected
- proof obligations:
  - Verus lemma slot: `lemma_flat_shift_step_*`
  - Kani harness: `kani_cuts_arbiter_total`
  - property row: `prop_arbiter_priority_total`

## US-FM0201B.5 Knock suppression

- units:
  - input `knock_intensity_x100`: `u16`
  - output `knock_retard_deg10`: `i16`
  - output `knock_active`: `bool`
- validation invariants:
  - `knock_threshold_x100 in 0..=10000`
  - `knock_retard_step_deg10 in 0..=200`
  - `knock_retard_max_deg10 in 0..=720`
- formula:
  - detect when `knock_intensity_x100 >= knock_threshold_x100`
  - on detect: increment retard by `knock_retard_step_deg10` up to `knock_retard_max_deg10`
  - on clear: recover toward zero by configured recovery step/counter
- state update:
  - update `knock_state { retard_deg10, recovery_counter, detected }`
- diagnostics:
  - represented through arbiter code `7` (`KnockSparkRetardOnly`) when no higher-priority cut is active and knock response remains asserted
- proof obligations:
  - Verus lemma slot: `lemma_knock_step_*`
  - Kani harness: `kani_cuts_arbiter_total`
  - property row: `prop_arbiter_priority_total`

## US-FM0201B.6 Safety latching

- units:
  - inputs are boolean fault predicates from safety subsystem
  - output `safety_latched`: `bool`
- validation invariants:
  - clear condition is explicit and deterministic
- formula:
  - if any latch-worthy safety fault is asserted, set `safety_latched = true`
  - once latched, remain latched until clear condition is true
- state update:
  - update `safety_latched` field in `LogicalState`
- diagnostics:
  - when selected by arbiter, map to `cut_reason_code = 1` (`SafetyLatched`)
- proof obligations:
  - covered by `lemma_arbiter_step_*` and `kani_cuts_arbiter_total`

## US-FM0201B.7 Unified arbiter priority and `cut_reason_code`

The arbiter input rows are evaluated in this exact frozen priority order (higher row wins):

1. `SafetyLatched` -> code `1`
2. `HardRevLimit` -> code `2`
3. `LaunchCut` -> code `3`
4. `FlatShiftCut` -> code `4`
5. `DfcoCut` -> code `5`
6. `SoftRevSparkCut` -> code `6`
7. `KnockSparkRetardOnly` -> code `7`
8. otherwise `None` -> code `0`

Frozen arbiter output mapping:

- `SafetyLatched`: `fuel_cut = true`, `spark_cut = true`
- `HardRevLimit`: `fuel_cut = true`, `spark_cut = true`
- `LaunchCut`: `fuel_cut = true`, `spark_cut = true`
- `FlatShiftCut`: `fuel_cut = true`, `spark_cut = true`
- `DfcoCut`: `fuel_cut = true`, `spark_cut = false`
- `SoftRevSparkCut`: `fuel_cut = false`, `spark_cut = true`
- `KnockSparkRetardOnly`: `fuel_cut = false`, `spark_cut = false`, knock retard applied
- `None`: `fuel_cut = false`, `spark_cut = false`

Proof obligations:

- Verus lemma slot: `lemma_arbiter_step_*`
- Kani harness: `kani_cuts_arbiter_total`
- property row: `prop_arbiter_priority_total`

## US-FM0201B.8 StepResult additive fields

Phase-2 `StepResult.output` adds these cut-domain fields:

- `cut_reason_code: u8`
- `fuel_cut: bool`
- `spark_cut: bool`
- `advance_deg10_trim: i16`
- `knock_intensity_x100: u16`

Frozen semantics:

- `cut_reason_code` is always one of `{0,1,2,3,4,5,6,7}` using the exact mapping above.
- `fuel_cut` and `spark_cut` are the post-arbiter final booleans (not pre-arbiter intermediate flags).
- `advance_deg10_trim` is zero unless a trim-producing subsystem is active (for US-FM0201B this is knock and/or soft rev behavior).
- default compatibility output for inactive Phase-2 cut subsystems remains identity versus v2 (`cut_reason_code=0`, no additional cuts, no extra trim).

# Phase-2 Domain Freeze: Closed-Loop Controllers (US-FM0201C)

This section freezes full behavior for idle PI control and lambda closed-loop PI control.

All US-FM0201C behavior is integer-only and deterministic.

## US-FM0201C.1 Idle PI controller

- units:
  - input `rpm_error`: signed RPM (`i16`) computed as `target_rpm - measured_rpm`
  - output `idle_duty_x1000`: duty ratio x1000 (`u16`)
  - state `idle_integrator_state`: `PiIntegratorState`
- integer gain formats:
  - `idle_kp_x1000`: proportional gain in x1000
  - `idle_ki_x1000`: integral gain in x1000 per step
  - proportional term: `p_term = floor(rpm_error * idle_kp_x1000 / 1000)`
  - integral increment term: `i_step = floor(rpm_error * idle_ki_x1000 / 1000)`
- frozen integrator clamp limits:
  - `idle_integrator_state.min_acc = -2000`
  - `idle_integrator_state.max_acc = 2000`
  - clamp is inclusive and idempotent
- dead-band:
  - if `abs(rpm_error) <= idle_deadband_rpm`, treat `rpm_error = 0` for both `p_term` and `i_step`
  - frozen `idle_deadband_rpm = 20`
- anti-windup rule:
  - compute provisional output `u_pre = idle_base_duty_x1000 + p_term + idle_integrator_state.acc`
  - if `u_pre` is outside actuator bounds and `sign(i_step)` would drive farther into saturation, do not integrate (`acc_next = acc`)
  - otherwise integrate with clamp (`acc_next = clamp(acc + i_step, min_acc, max_acc)`)
- freeze gates:
  - freeze integrator update when any of:
  - `clt_c10 < 700` (coolant below 70.0 C)
  - `ae_active == true`
  - `fuel_cut == true || spark_cut == true`
  - frozen update in these gates: `acc_next = acc`
- output unit and clamp:
  - `idle_duty_x1000 = clamp(idle_base_duty_x1000 + p_term + acc_next, idle_duty_min_x1000, idle_duty_max_x1000)`
  - `idle_duty_min_x1000 = 0`
  - `idle_duty_max_x1000 = 1000`
- diagnostics:
  - no standalone diagnostic code; out-of-range gains or limits are calibration-invalid
- proof obligations:
  - Verus lemma slot: `lemma_idle_pi_step_*`
  - Kani harness: `kani_idle_pi_total`
  - property row: `prop_pi_clamp_idempotent`

## US-FM0201C.2 Lambda closed-loop PI controller

- units:
  - input `lambda_error_x1000`: signed lambda ratio error x1000 (`i16`), positive when measured lambda is lean versus target
  - output `lambda_correction_x1000`: fuel correction ratio x1000 (`u16`)
  - state `lambda_integrator_state`: `PiIntegratorState`
- integer gain formats:
  - `lambda_kp_x1000`: proportional gain in x1000
  - `lambda_ki_x1000`: integral gain in x1000 per step
  - proportional term: `p_term = floor(lambda_error_x1000 * lambda_kp_x1000 / 1000)`
  - integral increment term: `i_step = floor(lambda_error_x1000 * lambda_ki_x1000 / 1000)`
- frozen integrator clamp limits:
  - `lambda_integrator_state.min_acc = -2000`
  - `lambda_integrator_state.max_acc = 2000`
  - clamp is inclusive and idempotent
- dead-band:
  - if `abs(lambda_error_x1000) <= lambda_deadband_x1000`, treat `lambda_error_x1000 = 0` for both `p_term` and `i_step`
  - frozen `lambda_deadband_x1000 = 10`
- anti-windup rule:
  - compute provisional correction `corr_pre = 1000 + p_term + lambda_integrator_state.acc`
  - if `corr_pre` is outside correction bounds and `sign(i_step)` would drive farther into saturation, do not integrate (`acc_next = acc`)
  - otherwise integrate with clamp (`acc_next = clamp(acc + i_step, min_acc, max_acc)`)
- freeze gates:
  - freeze integrator update when any of:
  - `clt_c10 < 700` (coolant below 70.0 C)
  - `ae_active == true`
  - `fuel_cut == true || spark_cut == true`
  - frozen update in these gates: `acc_next = acc`
- output unit and clamp:
  - `lambda_correction_x1000 = clamp(1000 + p_term + acc_next, 750, 1250)`
  - correction is applied multiplicatively to fueling as ratio x1000
- diagnostics:
  - no standalone diagnostic code; out-of-range gains or bounds are calibration-invalid
- proof obligations:
  - Verus lemma slot: `lemma_lambda_pi_step_*`
  - Kani harness: `kani_lambda_pi_total`
  - property row: `prop_pi_clamp_idempotent`

## US-FM0201C.3 Frozen integration ordering for closed-loop outputs

- idle PI executes after cut arbitration inputs are known, so freeze gates observe final cut booleans.
- lambda PI executes after AE state update and cut arbitration, so freeze gates observe `ae_active`, `fuel_cut`, and `spark_cut` from the same step.
- default Phase-2 compatibility values preserve v2 outputs when these controllers are not explicitly enabled:
  - `idle_kp_x1000 = 0`, `idle_ki_x1000 = 0`
  - `lambda_kp_x1000 = 0`, `lambda_ki_x1000 = 0`
  - controller outputs hold identity values (`idle_duty_x1000` unchanged by PI; `lambda_correction_x1000 = 1000`)

## US-FM0201D Sensor path scope freeze

This section freezes full behavior for CLT, IAT, MAP, TPS, MAF, O2/lambda, knock, baro, and Vbat sensor paths, including plausibility and slew-limit rules.

All US-FM0201D sensor conversions are integer-only, deterministic, and use floor semantics where division is required.

## US-FM0201D.1 Sensor curves and unit mapping

Frozen conversion-table dimensions, inputs, outputs, and monotonicity claims:

| Sensor | Frozen table shape | Input unit | Output unit | Monotonicity claim |
| --- | --- | --- | --- | --- |
| CLT thermistor | 1-D curve, 16 points | `adc_counts` (`u16`) | `TempC10` (`i16`) | non-increasing temperature with increasing resistance |
| IAT thermistor | 1-D curve, 16 points | `adc_counts` (`u16`) | `TempC10` (`i16`) | non-increasing temperature with increasing resistance |
| MAP linear | 1-D linear, 2 points | `adc_counts` (`u16`) | `Kpa10` (`u16`) | non-decreasing |
| TPS linear | 1-D linear, 2 points | `adc_counts` (`u16`) | percent x100 (`u16`) | non-decreasing |
| MAF piecewise linear | 1-D curve, 16 points | `adc_counts` (`u16`) | flow x100 (`u16`) | non-decreasing |
| O2 wideband/lambda | 1-D linear, 2 points | `adc_counts` (`u16`) | `AfrX100` (`u16`) | non-decreasing |
| O2 narrowband | threshold + hysteresis scalars | `adc_counts` (`u16`) | `AfrX100` equivalent (`u16`) | two-state switch with hysteresis |
| Knock window | scalar gain + threshold | window energy (`u16`) | intensity x100 (`u16`) | non-decreasing |
| Baro linear | 1-D linear, 2 points | `adc_counts` (`u16`) | `Kpa10` (`u16`) | non-decreasing |
| Vbat linear | 1-D linear, 2 points | `adc_counts` (`u16`) | `Millivolts` (`u16`) | non-decreasing |

Frozen sensor-path clipping rule:

- `sensor_output = clamp(interpolate(sensor_table, adc_counts), sensor_min, sensor_max)`
- linear 2-point sensors use the same floor linear interpolation helper as other 1-D lookups.
- thermistor conversions map counts -> millivolts -> resistance -> curve temperature and then clamp to plausible sensor range.

## US-FM0201D.2 Per-sensor plausibility thresholds

The plausibility gate is enabled only when `rpm >= 1000` and applies a `500_000 us` debounce for assert and clear.

Frozen per-sensor plausibility ranges:

| Sensor | Plausible min | Plausible max | Unit |
| --- | --- | --- | --- |
| CLT | `-400` | `1500` | `TempC10` |
| IAT | `-400` | `1200` | `TempC10` |
| MAP | `100` | `3000` | `Kpa10` |
| TPS | `0` | `10000` | percent x100 |
| MAF | `0` | `60000` | flow x100 |
| O2/lambda | `500` | `3000` | `AfrX100` |
| Knock intensity | `0` | `10000` | x100 |
| Baro | `500` | `1200` | `Kpa10` |
| Vbat | `6000` | `18000` | `Millivolts` |

Frozen cross-sensor plausibility rules:

- high-TPS/low-MAP fault when `tps_percent >= 80` and `map_kpa10 <= 300`
- low-TPS/high-MAP fault when `tps_percent <= 10` and `map_kpa10 >= 950`
- out-of-range single-sensor values assert the same plausibility subsystem fault channel with the sensor-specific code frozen by US-FM0227.

## US-FM0201D.3 Per-sensor slew limits

Slew limits are applied per step using elapsed time `dt_us`:

- `max_delta = floor(max_rate_per_s * dt_us / 1_000_000)`
- accepted value is clamped to `last_value ± max_delta`
- first sample initializes last-value state and is not clamped.

Frozen maximum per-sensor rates:

| Sensor | Max rate per second | Unit |
| --- | --- | --- |
| CLT | `200` | `TempC10/s` |
| IAT | `300` | `TempC10/s` |
| MAP | `2000` | `Kpa10/s` |
| TPS | `50000` | percent x100/s |
| MAF | `100000` | flow x100/s |
| O2/lambda | `2000` | `AfrX100/s` |
| Knock intensity | `50000` | x100/s |
| Baro | `50` | `Kpa10/s` |
| Vbat | `5000` | `Millivolts/s` |

Frozen shared slew constraints:

- minimum sample interval for slew checking: `1000 us`
- when `dt_us < 1000`, keep last accepted reading and do not advance reject counters.
- repeated rejections keep the last accepted value until the first in-rate sample arrives.

## US-FM0201D.4 Proof and test obligation binding

US-FM0201D obligations are pinned to previously frozen matrix rows:

- Verus lemma slots: `lemma_clt_from_counts_*`, `lemma_iat_from_counts_*`, `lemma_map_from_counts_*`, `lemma_tps_from_counts_*`, `lemma_maf_from_counts_*`, `lemma_o2_from_counts_*`, `lemma_knock_from_window_*`, `lemma_baro_from_counts_*`, `lemma_vbat_from_counts_*`, `lemma_sensor_plausibility_step_*`, `lemma_sensor_slew_step_*`
- Kani harness rows: `kani_sensor_curves_total`, `kani_sensor_plausibility_total`, `kani_sensor_slew_total`
- property rows: `prop_sensor_curve_clamp`, `prop_sensor_slew_limit`

# Phase-2 Domain Freeze: Trigger, Scheduler-Cancel, Torque (US-FM0201E)

This section freezes full behavior for 60-2 trigger decoding, sync-loss/resync policy, cam-phase disambiguation with typed `Option<CamTooth>`, stall detection, scheduler sync-loss cancellation, and torque request/arbiter/actuate reducer order.

All US-FM0201E formulas are integer-only, deterministic, and side-effect free.

## US-FM0201E.1 60-2 trigger decoder

- units:
  - input `tooth_timestamp_us`: `Micros`
  - output `sync_state`: `TriggerSyncState`
  - output `angle_deg10`: `Degrees10` in `[0, 7200)`
  - output `rpm_estimate`: `Rpm` in `0..=20000`
- frozen shape:
  - wheel model is exactly 60 slots with 2 missing teeth
  - one full wheel cycle is 58 observed teeth
  - crank angle per observed tooth is `120` deg10
- frozen formula:
  - tooth interval: `dt_us = tooth_timestamp_us - last_tooth_timestamp_us` with monotone timestamp precondition
  - missing-tooth candidate when `dt_us >= floor(prev_dt_us * 3 / 2)` and `prev_dt_us > 0`
  - when synced, tooth index increments modulo 58 and `angle_deg10 = (tooth_index * 120) % 7200`
  - `rpm_estimate = clamp(floor(60_000_000 / (dt_us * 58)), 0, 20000)` when `dt_us > 0`, else hold prior estimate
- state update:
  - update `trigger_state`, `sync_state`, `rpm_estimate`, `last_tooth_timestamp_us`, `prev_tooth_interval_us`, and current tooth index
- diagnostics:
  - no direct diagnostic code; unsynced/sync-loss visibility is through sync state and downstream arbitration
- proof obligations:
  - Verus lemma slot: `lemma_trigger_60_2_step_*`
  - Kani harness: `kani_trigger_decoder_total`
  - property row: `prop_trigger_sync_totality`
  - fuzz obligation (US-FM0285): tooth-interval streams encoded as little-endian `u32` intervals, bounded to `0..=128` intervals (`0..=512` bytes), with each interval mapped to `50..=200000` us before stepping.

## US-FM0201E.2 Sync-loss and resync policy

- sync-loss detection:
  - while `sync_state == Synced`, assert sync loss when either:
  - no tooth arrives for `stall_timeout_us` (see stall section), or
  - observed gap ratio is outside frozen tolerance for two consecutive candidate windows
  - frozen gap tolerance window is `dt / prev_dt ∈ [3/4, 5/4]` for non-missing candidate teeth
  - missing-tooth candidate threshold remains `dt / prev_dt >= 3/2`
- resync policy:
  - from `NoSync` or `SyncLoss`, transition to `PreSync` on first missing-tooth candidate
  - transition `PreSync -> Synced` only after the next valid tooth sequence confirms expected spacing/order
  - on failed confirmation, return to `NoSync`
- frozen determinism:
  - identical tooth timestamp streams produce identical sync-state streams
  - no ambient clock is read; only provided timestamps and carried state are used
- proof obligations:
  - Verus lemma slot: `lemma_trigger_sync_loss_step_*`
  - Kani harness: `kani_trigger_sync_loss_total`
  - property row: `prop_trigger_sync_totality`

## US-FM0201E.3 Cam-phase disambiguation (`Option<CamTooth>`)

- typed input:
  - cam signal input is exactly `Option<CamTooth>`
- frozen `None` behavior:
  - when cam input is `None`, phase state is deterministic passthrough:
  - no phase toggle occurs
  - no synthetic edge is generated
  - all non-cam outputs are unchanged except by crank-only logic
- frozen `Some(CamTooth)` behavior:
  - phase update is allowed only on valid crank-synchronized windows
  - out-of-window cam edges are ignored and do not desync crank state
- proof obligations:
  - Verus lemma slot: `lemma_cam_phase_step_*`
  - Kani harness: `kani_cam_phase_option_total`
  - property row: `prop_cam_none_passthrough`

## US-FM0201E.4 Stall detection

- units:
  - input elapsed time from last tooth: microseconds
  - output `stalled`: boolean derived from timeout
- frozen threshold:
  - `stall_timeout_us = 400000`
- formula:
  - if `now_us - last_tooth_timestamp_us >= stall_timeout_us`, set `stalled = true`
  - if stalled, force `rpm_estimate = 0` and `sync_state = SyncLoss`
  - clear stalled only on a new valid tooth sequence that completes resync criteria
- state update:
  - `stall_counter` increments once per step while stalled up to `u16::MAX` saturation
- proof obligations:
  - Verus lemma slot: `lemma_stall_detection_step_*`
  - Kani harness: `kani_stall_total`
  - property row: `prop_trigger_sync_totality`

## US-FM0201E.5 Scheduler sync-loss cancellation

- frozen cancellation rule:
  - on transition into `SyncLoss`, request scheduler cancellation of all pending angle-based events in the same semantic step
  - cancellation is idempotent if already cancelled
- frozen output consequence:
  - after cancellation, `pending` event batch for the step is empty until sync is re-established
  - no injection or spark semantic event may be emitted while unsynced
- proof obligations:
  - Verus lemma slot: `lemma_scheduler_cancel_on_sync_loss_*`
  - Kani harness coverage through trigger/sync-loss rows
  - property row: `prop_trigger_sync_totality`

## US-FM0201E.6 Torque request/arbiter/actuate reducer order

Frozen reducer order is exactly:

1. derive `torque_request_x1000` from driver/request sources
2. apply limiter stack to compute `torque_allowed_x1000`
3. apply safety and cut-domain clamps
4. emit final `torque_actuated_x1000`
5. project fuel/spark trims from actuated torque output

Frozen constraints:

- all three torque fields are clamped in `0..=1000`
- reducer order is strict; no stage may read a later-stage value
- each stage is pure and deterministic over current inputs and state

proof obligations:

- Verus lemma slot: `lemma_torque_pipeline_step_*`
- Kani harness: `kani_torque_pipeline_total`
- property row: `prop_arbiter_priority_total`

# Phase-2 Domain Freeze: Persistence and TunerStudio Protocol (US-FM0201F)

This section freezes full behavior for persistence layouts, CRC/version handling, migration, factory reset bytes, TS page metadata, OUTPC framing, command dispatch, burn/save sequencing, and diagnostic ring-buffer semantics.

All US-FM0201F formulas and state transitions are integer-only, deterministic, and side-effect free at the spec layer.

## US-FM0201F.1 KV page layouts

Frozen persisted page payload sizes:

- fuel page payload bytes: `512`
- ignition page payload bytes: `512`
- angles page payload bytes: `68`

Frozen persisted record format for each page:

- byte `0..=1`: `schema_version_le_u16`
- byte `2..=3`: `page_id_le_u16`
- byte `4..=5`: `payload_len_le_u16`
- byte `6..(6 + payload_len - 1)`: payload bytes
- final 4 bytes: `crc32c_le_u32` over bytes `0..(end_crc_exclusive - 1)`

Frozen page identifiers:

- `page_id = 1`: fuel payload (`512` bytes)
- `page_id = 2`: ignition payload (`512` bytes)
- `page_id = 3`: angles payload (`68` bytes)

## US-FM0201F.2 CRC and version invariants

Frozen invariants:

- decode succeeds only when `payload_len` matches the frozen page size for the declared `page_id`
- decode succeeds only when `schema_version` is present in the migration matrix (US-FM0201F.3)
- decode succeeds only when computed `crc32c` exactly matches stored `crc32c`
- CRC mismatch is a typed decode error and never a panic path
- no partial decode is exposed; decode is all-or-error

Proof obligations:

- Verus lemma slots: `lemma_persist_encode_*`, `lemma_persist_decode_*`
- Kani harness: `kani_persist_total`
- property row: `prop_persist_roundtrip`

## US-FM0201F.3 Version migration matrix

Frozen current schema version: `3`.

Migration matrix:

| From | To | Policy |
| --- | --- | --- |
| `1` | `2` | deterministic field-copy + zero-init newly introduced bytes |
| `2` | `3` | deterministic field-copy + canonical default fill for new offsets |
| `1` | `3` | apply `1 -> 2 -> 3` sequentially, no shortcut path |
| `3` | `3` | identity |

Frozen failure behavior:

- any `from_version > 3` is rejected as unsupported
- any missing required source bytes for the declared source version is rejected as malformed
- migration result payload length is always the frozen destination length

Proof obligations:

- Verus lemma slot: `lemma_persist_migrate_*`
- Kani harness: `kani_persist_total`
- property row: `prop_persist_roundtrip`

## US-FM0201F.4 Factory-reset canonical bytes

Factory reset writes canonical payload bytes per page:

- fuel page payload: all `0x00` except fixed identity constants at frozen offsets
- ignition page payload: all `0x00` except fixed identity constants at frozen offsets
- angles page payload: all `0x00` except fixed geometry defaults at frozen offsets

Frozen requirement:

- canonical payload bytes are a single deterministic constant array per page id and schema version
- encoded factory-reset records always include valid header (`schema_version=3`, `payload_len` exact) and valid CRC32C

Proof obligations:

- Verus lemma slot: `lemma_persist_factory_reset_*`
- Kani harness: `kani_persist_total`
- property row: `prop_persist_roundtrip`

## US-FM0201F.5 TS page numbers, signatures, and sizes

Frozen TS page metadata:

| Page number | Symbol | Signature | Payload size |
| --- | --- | --- | --- |
| `1` | `fuel` | `0x46554C31` (`"FUL1"`) | `512` |
| `2` | `ignition` | `0x49474E31` (`"IGN1"`) | `512` |
| `3` | `angles` | `0x414E4731` (`"ANG1"`) | `68` |
| `4` | `outpc` | `0x4F555431` (`"OUT1"`) | `64` |

Frozen metadata rule:

- `ts_page_meta(page_number)` is total for all `u8` page numbers and returns either exact metadata or typed `UnknownPage`

Proof obligations:

- Verus lemma slot: `lemma_ts_page_meta_*`
- Kani harness: `kani_ts_proto_total`
- property row: `prop_ts_dispatch_totality`

## US-FM0201F.6 OUTPC layout

Frozen OUTPC payload length: `64` bytes.

Frozen byte layout (little-endian integer fields):

| Byte offset | Field | Unit/encoding |
| --- | --- | --- |
| `0..=1` | `rpm` | `u16` |
| `2..=3` | `map_kpa10` | `u16` |
| `4..=5` | `tps_x100` | `u16` |
| `6..=7` | `clt_c10` | `i16` |
| `8..=9` | `iat_c10` | `i16` |
| `10..=11` | `pw_corr_us` | `u16` saturated from semantic `u32` |
| `12..=13` | `advance_deg10` | `i16` |
| `14` | `sync_state_code` | `u8` enum code |
| `15` | `cut_reason_code` | `u8` |
| `16..=19` | `status_flags` | bitfield `u32` |
| `20..=63` | reserved | all zeros |

Frozen encoding rule:

- every reserved byte is encoded as zero
- encode never panics; out-of-range narrowing uses explicit saturating cast where required (`pw_corr_us`)

Proof obligations:

- Verus lemma slot: `lemma_ts_outpc_encode_*`
- Kani harness: `kani_ts_proto_total`
- property row: `prop_ts_dispatch_totality`

## US-FM0201F.7 TS command dispatch state machine

Frozen dispatch states:

1. `Idle`
2. `RxFrame`
3. `Decode`
4. `Execute`
5. `EncodeReply`
6. `ErrorReply`
7. `Idle` (next command)

Frozen command classes:

- `ReadPage(page_number, offset, len)`
- `WritePage(page_number, offset, bytes)`
- `Burn(page_number)`
- `GetOutpc`
- `GetSignature(page_number)`
- `Unknown(command_id)`

Frozen dispatch behavior:

- every frame is processed in one deterministic transition sequence
- unknown commands produce typed error replies and return to `Idle`
- malformed frames produce typed decode errors and return to `Idle`
- no command may bypass `Decode`

Proof obligations:

- Verus lemma slot: `lemma_ts_dispatch_step_*`
- Kani harness: `kani_ts_proto_total`
- property row: `prop_ts_dispatch_totality`

## US-FM0201F.8 Burn/save protocol

Frozen write/burn/save sequencing:

1. `WritePage` mutates only staged RAM buffer for the addressed page
2. `Burn(page)` validates staged bytes and computes CRC/header
3. successful burn atomically commits staged page record to persisted store
4. failed burn leaves previously committed persisted record unchanged
5. `SaveAll` applies per-page burn in page-number order `1,2,3`

Frozen constraints:

- burn is rejected when engine-running guard is active; persisted bytes remain unchanged
- burn acknowledges success only after CRC-valid record exists for the target page
- no cross-page partial writes: each page commit is atomic

Proof obligations:

- Verus lemma slot: `lemma_ts_burn_save_step_*`
- Kani harness: `kani_ts_proto_total`
- property row: `prop_ts_dispatch_totality`

## US-FM0201F.9 Diagnostic ring-buffer invariant

Frozen diagnostic log ring shape:

- capacity: `64` entries
- per-entry fields match the current shared TS `diag_log` row encoding:
  `code (u8)`, `severity (u8)`, `action (u8)`, `source (u8, high bit = context_present)`,
  `start_us (u32)`, `end_us (u32)`, `context (u32)`

Frozen ring-buffer invariants:

- `0 <= len <= 64`
- `head` and `tail` indexes are always in `0..64`
- push on full buffer overwrites oldest entry and advances `tail` by one
- pop on empty buffer is typed `None` and does not modify indexes
- iteration order is oldest to newest, deterministic for identical push/pop traces

Proof obligations:

- Verus lemma slot: `lemma_ts_diag_ring_step_*`
- Kani harness: `kani_ts_proto_total`
- property row: `prop_ts_dispatch_totality`
