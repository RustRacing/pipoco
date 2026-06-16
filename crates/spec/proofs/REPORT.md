# ECU Formal Methods Proof Bundle — Phase 2 + Phase 3 (v5)

**Generated:** 2026-04-24
**Stories:** US-FM0201..US-FM0297 (Phase 2) + US-FM0300..US-FM0315 (Phase 3 v4) + US-FM0400..US-FM0438 (Phase 5 v5)
**Branch:** main

---

## 1. Verus Lemmas

**Total verified:** 50 lemmas across 5 story groups

| Story | Count | Key Lemmas |
|-------|-------|------------|
| US-FM0014 (v1 baseline) | 15 | norm7200_in_range, cyc7200_distance_bound, lerp/bilerp bounds, duration_nonnegative, fuel/spark_cut_no_events, step_determinism |
| US-FM0105 | +2 | (cumulative 17) |
| US-FM0247 | +12 | clamp bounds, arbiter priority totality, enrichment order |
| US-FM0248 | +4 | idle/lambda PI freeze idempotent, saturation clamp idempotent |
| US-FM0249 | +9 | CLT/IAT/MAP/TPS/MAF/O2/knock/baro/vbat monotonicity+clamp |
| US-FM0250 | +3 | trigger sync state totality, angle wrap, RPM estimate bound |

**Verification command:**
```
RUSTUP_TOOLCHAIN=1.95.0-x86_64-unknown-linux-gnu /tmp/verus-install/verus-x86-linux/verus ecu-spec/proofs/verus.rs
```
**Result:** `verification results:: 50 verified, 0 errors`

---

## 2. Kani Harnesses

**Total:** 55 harnesses

| Harness Group | Count | Description |
|--------------|-------|--------------|
| Numeric helpers | 8 | norm7200, cyc7200, lerp/bilerp overflow, duration bounds |
| Fuel pipeline | 5 | deadtime, Vbat, baro, cranking, afterstart |
| Closed-loop | 2 | idle PI, lambda PI |
| Enrichment/AE | 2 | warmup, AE |
| Cuts/Arbiter | 2 | cuts arbiter totality, knock |
| Sensor curves | 10+ | CLT, IAT, MAP, TPS, MAF, O2, knock, baro, vbat |
| Trigger decoder | 1 | trigger decoder totality |
| Persistence | 4 | persist round-trip (fuel, ignition, angles variants) |
| TS proto | 1 | page_meta totality, OUTPC round-trip |
| Validation | 10+ | axis/table/curve dimension and range variants |
| Runtime | 2 | runtime_input_total, frozen_diagnostic_priority |

**Verification command:**
```
cargo kani -p ecu-spec
```

---

## 3. TLA+ Modules

**Total:** 6 modules (all safety + conditional liveness checked)

| Module | States | Distinct | Depth | Duration | Key Invariants |
|--------|--------|----------|-------|----------|----------------|
| scheduler | 4,822,273 | 64,512 | 17 | 53s | Inv (safety), ConditionalLiveness with event emission |
| trigger | 86 | 25 | 11 | 0s | SyncAcquire, SyncLoss, Resync |
| persist | 1,134 | 132 | 9 | 0s | Burn/save atomicity, rollback |
| safety | 161 | 16 | 4 | 0s | LatchedImpliesCutAsserted |
| arbiter | 16,385 | 128 | 2 | 0s | NoLowerPriorityCutWins |
| ts_proto | 62 | 25 | 5 | 0s | Command → Response liveness |

**TLC command pattern:**
```
java -cp /home/user/.local/lib/tla/tla2tools.jar tlc2.TLC -config ecu-spec/tla/{module}.cfg ecu-spec/tla/{module}.tla
```
**All modules:** `Model checking completed. No error has been found.`

---

## 4. Property Tests

**Total:** 43 property tests across 3 test files

| File | Count | Coverage |
|------|-------|----------|
| `prop_semantics.rs` | 32 | Phase-2 oracle properties (deadtime, Vbat, baro, enrichment, arbiter, PI, sensor curves, trigger) |
| `ts_proto_properties.rs` | 5 | TS proto totality, OUTPC round-trip, dispatch, burn/save, diag log |
| `persist_roundtrip.rs` | 6 | Persistence round-trip, migration (v1→v2→v3), factory reset idempotence |

**Run:** `cargo test -p ecu-spec --test prop_semantics --test ts_proto_properties --test persist_roundtrip`

---

## 5. Oracle Example Vectors

**File:** `ecu-spec/tests/oracle_examples.rs`
**Count:** 35 test functions

Covers: deadtime midpoint, Vbat edges, baro edges, cranking, AE decay, DFCO hysteresis, rev-limit soft/hard, idle PI step, lambda CL PI step, knock retard, launch cut cycle, flat-shift cut cycle, arbiter priority ordering.

---

## 6. Coverage Summary

**Gate threshold:** 90.0% line coverage (all crates)

| Crate | Line | Functions | Branches |
|-------|------|-----------|----------|
| ecu-spec | 97.56% | 98.56% | 97.19% |
| ecu-scheduler | 88.34% | (not recorded) | (not recorded) |
| ecu-runtime | measured separately | — | — |

**Command:** `cargo llvm-cov --package <crate> --summary-only`

---

## 7. Benchmark Summary

**Gate threshold:** 5.0% p95 regression

| Benchmark | Baseline | Tool |
|-----------|----------|------|
| spec_step | 678.22 ns p95 | criterion 0.4, release, Intel Skylake |
| runtime_step | (see criterion output) | criterion 0.4, release |

**Command:** `cargo bench --all-features -- --save-baseline=fm0289-baseline`

---

## 8. Formal Tool Versions (Pinned)

| Tool | Version | Path |
|------|---------|------|
| Verus | 0.2026.04.19.6f7d4de | `/tmp/verus-install/verus-x86-linux/verus` |
| Rust toolchain | 1.95.0-x86_64-unknown-linux-gnu | `~/.rustup/toolchains/` |
| Kani | 0.67.0 | `cargo kani` |
| TLA+ toolbox (TLC) | 2.19 | `/home/user/.local/lib/tla/tla2tools.jar` |
| cargo-public-api | 0.51.0 | `~/.cargo/bin/cargo-public-api` |
| cargo-llvm-cov | 0.6.14 | `~/.cargo/bin/cargo-llvm-cov` |

---

## 9. Workspace Verification Commands

### Fast gates (host CI)
```bash
cargo fmt --all --check
cargo test -p ecu-spec
cargo clippy -p ecu-spec --all-targets -- -D warnings
cargo test -p ecu-runtime
cargo test -p ecu-scheduler
cargo test --workspace --exclude ecu-rp2040-pico --exclude stm32f4-ecu --exclude ecu-rp2350b
```

### Formal gates (tools/verify_formal.sh)
```bash
bash tools/verify_formal.sh
# Chains: prd.json, fmt, tests, Verus, TLC (6 modules), Kani, rg audits, panic-freedom, no_alloc, fuzz corpora
```

### Embedded builds (tools/verify.sh)
```bash
bash tools/verify.sh
# rp2040-pico release builds (ts-ecu, flash-kv, ts-gauges, pio_capture)
# stm32f4-ecu check (release, features)
# stm32f4 clippy
# ecu-rp2350b check
```

---

## 10. Phase 3 (v4) Proof-to-Product Conformance Results

**Stories:** US-FM0300..US-FM0315
**Date:** 2026-04-24

### Non-Formal Gates

| Gate | Command | Result |
|------|---------|--------|
| prd.json | `python3 -m json.tool prd.json` | PASS |
| evidence.schema.json | `python3 -m json.tool tools/evidence.schema.json` | PASS |
| fmt | `cargo fmt --all --check` | PASS |
| workspace clippy | `cargo clippy --workspace --exclude ecu-rp2040-pico --exclude stm32f4-ecu --exclude ecu-rp2350b --all-targets -- -D warnings` | PASS |
| workspace tests | `cargo test --workspace --exclude ecu-rp2040-pico --exclude stm32f4-ecu --exclude ecu-rp2350b` | PASS (577 tests) |
| embedded RP2040 | `cargo check -p ecu-rp2040-pico --release --target thumbv6m-none-eabi` | PASS |
| embedded STM32F4 | `cargo check -p stm32f4-ecu --release --target thumbv7em-none-eabihf` | PASS |
| embedded RP2350B | `cargo check -p ecu-rp2350b --release --target thumbv8m.main-none-eabihf` | PASS |
| Verus | `RUSTUP_TOOLCHAIN=1.95.0... /tmp/verus-install/.../verus ecu-spec/proofs/verus.rs` | PASS (50 verified, 0 errors) |
| production panic/unwrap/expect scan | `rg` audit in `tools/verify_formal.sh` | 0 production matches |
| production no std/alloc scan | `rg` audit in `tools/verify_formal.sh` | 0 matches |
| no_alloc audit | `tools/verify_formal.sh` phase 6 | PASS |
| panic-freedom audit | `tools/verify_formal.sh` phase 5 | PASS |

### Conformance Tests

| Test File | Result |
|-----------|--------|
| `ecu-runtime/tests/fm0016_runtime_conformance.rs` | 5/5 PASS |
| `ecu-scheduler/tests/fm0016_scheduler_conformance.rs` | 5/5 PASS |
| `tests/fm0016_core_reducer.rs` | 5/5 PASS |

### Kani Harnesses (v4 New — Exec Coupling)

| Harness | Status | Time |
|---------|--------|------|
| `kani_exec_interp_matches_contract` | PASS | 40s |
| `kani_exec_fuel_pipeline_matches_contract` | PASS | 21s |
| `kani_exec_schedule_matches_contract` | PASS | 5s |
| `kani_exec_persist_matches_contract` | PASS | 80s |
| `kani_exec_ts_proto_matches_contract` | PASS | 3s |

### Coverage Summary

| Crate | Line Coverage | Threshold |
|-------|-------------|-----------|
| ecu-spec | 97.42% | 90.0% |
| ecu-runtime | 90.32% | 90.0% |
| ecu-scheduler | 96.86% | 90.0% |

### Embedded Gate Completion (US-FM0306)

`tools/verify.sh` updated to include all pinned embedded targets:
- RP2040 Pico (thumbv6m-none-eabi)
- STM32F4 (thumbv7em-none-eabihf)
- RP2350B (thumbv8m.main-none-eabihf)

### Blockers
None. All v4 gates passed 2026-04-24.

---

## 11. Phase 5 (v5) Proof-to-Product Conformance Results

**Stories:** US-FM0400..US-FM0438
**Date:** 2026-04-24

### Non-Formal Gates

| Gate | Command | Result |
|------|---------|--------|
| prd.json | `python3 -m json.tool prd.json` | PASS |
| evidence.schema.json | `python3 -m json.tool tools/evidence.schema.json` | PASS |
| fmt | `cargo fmt --all --check` | PASS |
| workspace clippy | `cargo clippy --workspace --exclude ecu-rp2040-pico --exclude stm32f4-ecu --exclude ecu-rp2350b --all-targets -- -D warnings` | PASS |
| workspace tests | `cargo test --workspace --exclude ecu-rp2040-pico --exclude stm32f4-ecu --exclude ecu-rp2350b` | PASS (all suites) |

### Formal Gates

| Gate | Command | Result |
|------|---------|--------|
| Verus | `RUSTUP_TOOLCHAIN=1.95.0... /tmp/verus-install/.../verus ecu-spec/proofs/verus.rs` | PASS (50 lemmas, 0 errors) |
| Kani | `cargo kani -p ecu-spec --output-format terse` | PASS (55 harnesses, 0 failures) |
| TLC scheduler | `java -cp tla2tools.jar tlc2.TLC -config scheduler.cfg scheduler.tla` | PASS (4,822,273 states, 64,512 distinct, 0 errors, 53s) |
| Embedded builds | `bash tools/verify.sh` | PASS (rp2040-pico, stm32f4-ecu, rp2350b all build) |

### v5 Adapter Contract Conformance Tests

| Test File | Result |
|-----------|--------|
| `ecu-runtime/tests/fm0016_runtime_conformance.rs` | 8/8 PASS |
| `ecu-scheduler/tests/fm0016_scheduler_conformance.rs` | 9/9 PASS |
| `tests/fm0016_core_reducer.rs` | 10/10 PASS |

### v5 Key Changes

- **No blocked rows**: All conformance map rows are `covered` or `adapter-contract`. No row uses `status=blocked`.
- **Adapter contracts**: Typed Rust enums owned by their product crates (`RuntimeAdapterContract`, `SchedulerAdapterContract`, `CoreAdapterContract`) document every model gap between frozen spec and product implementation.
- **Product-observed paths**: All conformance reducer tests use product code for observed data. `oracle_result(case)` called once per fixture for expected data only.
- **No oracle-copy surfaces**: Scheduler no longer exposes differential DTOs as
  production API. Runtime/root conformance tests use product APIs only for
  observed values.
- **ECU-spec dependency hygiene**: No production code in `ecu-runtime`, `ecu-scheduler`, `ecu-compat`, or board crates depends on `ecu-spec`.

### Blockers
None. All v5 gates passed.
