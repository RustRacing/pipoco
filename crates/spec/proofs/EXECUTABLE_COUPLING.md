# Executable Coupling Table

Schema: [contextFile §Executable Proof Coupling](proof-to-product-v5-execution-plan.md##-executable-proof-coupling)

Columns: `frozen contract`, `executable function(s)`, `proof artifact`, `coupling method`, `symbolic domain`, `status`, `command`

---

## Interpolation

| Frozen Contract | Executable Function(s) | Proof Artifact | Coupling Method | Symbolic Domain | Status | Command |
|---|---|---|---|---|---|---|
| `find_segment` | `ecu_spec::interp::find_segment(&Axis16, u16) -> usize` | `verus.rs lemma_find_segment_in_range` | Verus lemma + Kani harness | `axis.len ∈ [2, 16]`, `x ∈ [0, 65535]` | covered | `RUSTUP_TOOLCHAIN=1.95.0 cargo test -p ecu-spec -- interp` |
| `lerp_u16` | `ecu_spec::interp::lerp_u16(u16, u16, u16, u16, u16) -> u16` | `verus.rs lemma_lerp_breakpoint_exactness`, `verus.rs lemma_lerp_boundedness` | Verus lemma + Kani harness `kani_lerp_u16_no_overflow` | `x0 ≤ x ≤ x1`, `y0, y1 ∈ [0, 65535]` | covered | `RUSTUP_TOOLCHAIN=1.95.0 cargo test -p ecu-spec -- lerp` |
| `lerp_i16` | `ecu_spec::interp::lerp_i16(u16, u16, i16, i16, u16) -> i16` | `verus.rs lemma_lerp_boundedness` | Verus lemma + Kani harness `kani_lerp_i16_no_overflow` | `x0 ≤ x ≤ x1`, `y0, y1 ∈ [-32768, 32767]` | covered | `RUSTUP_TOOLCHAIN=1.95.0 cargo test -p ecu-spec -- lerp` |
| `bilerp_u16` | `ecu_spec::interp::bilerp_u16(&Table2D16<u16>, Rpm, Kpa10) -> u16` | `verus.rs lemma_bilerp_grid_point_exactness`, `verus.rs lemma_bilerp_convex_hull_boundedness` | Verus lemma + Kani harness `kani_bilerp_u16_no_overflow` | `rpm ∈ [0, 65535]`, `load ∈ [0, 65535]`, table axes sorted | covered | `RUSTUP_TOOLCHAIN=1.95.0 cargo test -p ecu-spec -- bilerp` |
| `bilerp_i16` | `ecu_spec::interp::bilerp_i16(&Table2D16<i16>, Rpm, Kpa10) -> i16` | `verus.rs lemma_bilerp_convex_hull_boundedness` | Verus lemma + Kani harness `kani_bilerp_i16_no_overflow` | `rpm ∈ [0, 65535]`, `load ∈ [0, 65535]`, table axes sorted | covered | `RUSTUP_TOOLCHAIN=1.95.0 cargo test -p ecu-spec -- bilerp` |

---

## Fuel Correction

| Frozen Contract | Executable Function(s) | Proof Artifact | Coupling Method | Symbolic Domain | Status | Command |
|---|---|---|---|---|---|---|
| VE lookup | `ecu_spec::fuel::lookup_ve(&ValidatedCalibration, Rpm, Kpa10) -> VePctX100` | `verus.rs lemma_constant_table_reproduction`, `kani.rs kani_step_runtime_input_total` | Verus lemma + Kani harness | `rpm ∈ [0, 30000]`, `load ∈ [0, 6553]`, validated cal | covered | `RUSTUP_TOOLCHAIN=1.95.0 cargo test -p ecu-spec -- fuel` |
| Target AFR | `ecu_spec::fuel::lookup_target_afr(&ValidatedCalibration, Rpm, Kpa10) -> AfrX100` | `verus.rs lemma_constant_table_reproduction`, `kani.rs kani_step_runtime_input_total` | Verus lemma + Kani harness | same as VE | covered | `RUSTUP_TOOLCHAIN=1.95.0 cargo test -p ecu-spec -- fuel` |
| Base PW | `ecu_spec::fuel::compute_pw_base_us(&FuelParts) -> PulseWidthUs` | `kani.rs kani_compute_pw_corr_clamped` | Kani harness | `ve ∈ [0, 20000]`, `target_afr ∈ [500, 2500]` | covered | `cargo kani -p ecu-spec --harness kani_compute_pw_corr_clamped` |
| Air PW | `ecu_spec::fuel::compute_pw_air_us(Rpm, Kpa10, VePctX100, AfrX100) -> PulseWidthUs` | `kani.rs kani_compute_pw_corr_clamped` | Kani harness | same as base PW | covered | `cargo kani -p ecu-spec --harness kani_compute_pw_corr_clamped` |
| Corrected PW | `ecu_spec::fuel::compute_pw_corr_us(&FuelParts) -> PulseWidthUs` | `kani.rs kani_compute_pw_corr_clamped` | Kani harness | corrected PW bounded by `u32::MAX` | covered | `cargo kani -p ecu-spec --harness kani_compute_pw_corr_clamped` |

---

## Scheduling

| Frozen Contract | Executable Function(s) | Proof Artifact | Coupling Method | Symbolic Domain | Status | Command |
|---|---|---|---|---|---|---|
| Duration conversion | `ecu_spec::numeric::duration_us_to_deg10(u32, Rpm) -> Degrees10` | `verus.rs lemma_duration_nonnegative`, `kani.rs kani_duration_us_to_deg10_no_overflow` | Verus lemma + Kani harness | `us ∈ [0, 2^32-1]`, `rpm ∈ [1, 30000]` | covered | `cargo kani -p ecu-spec --harness kani_duration_us_to_deg10_in_cycle_bound` |
| SOI/EOI scheduling | `ecu_spec::schedule::schedule_cylinder(&ValidatedCalibration, &InputSnapshot, CylinderId) -> CylinderSchedule` | `verus.rs lemma_spark_after_dwell_ordering`, `verus.rs lemma_fuel_cut_no_injection_events`, `verus.rs lemma_spark_cut_no_spark_events` | Verus lemma | valid cal, `angle ∈ [0, 7200)` | covered | `RUSTUP_TOOLCHAIN=1.95.0 cargo test -p ecu-spec -- schedule` |
| Spark advance | `ecu_spec::schedule::compute_spark_advance_deg10(&ValidatedCalibration, Rpm, Kpa10) -> SignedDegrees10` | `verus.rs lemma_constant_table_reproduction` | Verus lemma + Kani harness | same as VE | covered | `RUSTUP_TOOLCHAIN=1.95.0 cargo test -p ecu-spec -- spark` |
| Dwell computation | `ecu_spec::schedule::compute_dwell_us(&ValidatedCalibration, Rpm, Kpa10) -> PulseWidthUs` | `verus.rs lemma_constant_table_reproduction` | Verus lemma | same as VE | covered | `RUSTUP_TOOLCHAIN=1.95.0 cargo test -p ecu-spec -- dwell` |
| Cut suppression | `ecu_spec::schedule::schedule_all_cylinders(...)` | `verus.rs lemma_fuel_cut_no_injection_events`, `verus.rs lemma_spark_cut_no_spark_events` | Verus lemma | `fuel_cut ∨ spark_cut ⇒ no events emitted` | covered | `RUSTUP_TOOLCHAIN=1.95.0 cargo test -p ecu-spec -- cut` |

---

## Persistence

| Frozen Contract | Executable Function(s) | Proof Artifact | Coupling Method | Symbolic Domain | Status | Command |
|---|---|---|---|---|---|---|
| `persist_encode` | `ecu_spec::persist_spec::persist_encode(&PersistPage) -> Result<EncodedPersistRecord, PersistEncodeError>` | `verus.rs lemma_persist_decode_encode_roundtrip`, `kani.rs kani_exec_persist_matches_contract` | Verus lemma + executable Kani bridge | valid `PersistPage`, record ≤ `PERSIST_RECORD_MAX_BYTES` | covered | `cargo kani -p ecu-spec --harness kani_exec_persist_matches_contract` |
| `persist_decode` | `ecu_spec::persist_spec::persist_decode(&[u8]) -> Result<PersistPage, PersistDecodeError>` | `verus.rs lemma_persist_decode_encode_roundtrip`, `kani.rs kani_exec_persist_matches_contract` | Verus lemma + executable Kani bridge | any byte slice, error on invalid CRC/header | covered | `cargo kani -p ecu-spec --harness kani_exec_persist_matches_contract` |
| `persist_migrate` | `ecu_spec::persist_spec::persist_migrate(PersistPageId, u16, u16, &[u8]) -> Result<PersistPage, PersistMigrationError>` | `verus.rs lemma_persist_migrate_current_idempotent`, `ecu-spec/tests/persist_roundtrip.rs` | Verus lemma + property test bridge | supported schema migration paths only | covered | `cargo test -p ecu-spec -- persist_migration` |
| `factory_reset` | `ecu_spec::persist_spec::factory_reset(&[u8]) -> Result<EncodedPersistRecord, PersistDecodeError>` | `ecu-spec/tests/persist_roundtrip.rs prop_factory_reset_roundtrip_equals_reset_twice` | property test bridge | valid encoded input pages | covered | `cargo test -p ecu-spec -- factory_reset` |

---

## TunerStudio Protocol

| Frozen Contract | Executable Function(s) | Proof Artifact | Coupling Method | Symbolic Domain | Status | Command |
|---|---|---|---|---|---|---|
| Page metadata | `ecu_spec::ts_spec::page_meta(TsPageId) -> Result<TsPageMeta, TsPageMetaError>` | `verus.rs lemma_ts_page_meta_totality` | Verus lemma | `page_id ∈ [1, 3]` | covered | `RUSTUP_TOOLCHAIN=1.95.0 cargo test -p ecu-spec -- page_meta` |
| OUTPC encode | `ecu_spec::ts_spec::encode_outpc(&OutpcFrame) -> [u8; 44]` | `verus.rs lemma_ts_outpc_roundtrip` | Verus lemma + property test | valid `OutpcFrame` | covered | `cargo test -p ecu-spec -- outpc_roundtrip` |
| OUTPC decode | `ecu_spec::ts_spec::decode_outpc(&[u8]) -> Result<OutpcFrame, OutpcCodecError>` | `verus.rs lemma_ts_outpc_roundtrip` | Verus lemma + property test | any 44-byte slice | covered | `cargo test -p ecu-spec -- outpc_roundtrip` |
| TS dispatch | `ecu_spec::ts_spec::ts_dispatch_step(&[u8]) -> TsDispatchResult<'_>` | `verus.rs lemma_ts_dispatch_totality`, `kani.rs kani_exec_ts_proto_matches_contract` | Verus lemma + executable Kani bridge | any frame ≤ 255 bytes | covered | `cargo kani -p ecu-spec --harness kani_exec_ts_proto_matches_contract` |
| Burn/save sequencing | `ecu_spec::ts_spec::burn_page`, `save_all` | `ecu-spec/tests/ts_proto_properties.rs prop_ts_burn_save_commit_atomicity`, `ecu-spec/tests/persist_roundtrip.rs prop_ts_burn_interleaved_commits_are_atomic` | property test bridge | `engine_running ∈ {true, false}`, valid page | covered | `cargo test -p ecu-spec -- burn` |
| Diagnostic ring | `ecu_spec::ts_spec::ts_diag_log_push`, `ts_diag_log_pop_oldest`, `encode_ts_diag_log_oldest_first` | `ecu-spec/tests/ts_proto_properties.rs prop_ts_diag_log_wrap_oldest_first`, `ecu-spec/src/ts_spec.rs diag_log_* tests` | property test bridge | `ring.len ∈ [0, 64]` | covered | `cargo test -p ecu-spec -- diag_log` |

---

## FM0016 Runtime Conformance (reducer)

| Frozen Contract | Executable Function(s) | Proof Artifact | Coupling Method | Symbolic Domain | Status | Command |
|---|---|---|---|---|---|---|
| Full step (runtime) | `ecu_runtime::EngineRuntime::step(&EngineRuntime, StepInputs) -> ActionBatch` | `ecu-runtime/tests/fm0016_runtime_conformance.rs` | Direct execution test against 88 FM0016 fixtures; fuel/timing fields via `RuntimeAdapterContract` | all fixture cases | adapter-contract | `cargo test -p ecu-runtime -- fm0016` |
| Trigger decoder | `ecu_runtime::DecoderObservation::Trigger(...)` | `ecu-runtime/tests/fm0016_runtime_reducer.rs` | Property test against trigger decoder fixtures | tooth stream inputs | covered | `cargo test -p ecu-runtime -- trigger_decoder` |
| Full step (scheduler) | real `ecu-scheduler` public scheduling API | `ecu-scheduler/tests/fm0016_scheduler_conformance.rs` | Direct execution test against 88 FM0016 fixtures; angle-law fields via `SchedulerAdapterContract` | all fixture cases | adapter-contract | `cargo test -p ecu-scheduler -- fm0016` |

---

## FM0016 Root/Core Conformance (reducer)

| Frozen Contract | Executable Function(s) | Proof Artifact | Coupling Method | Symbolic Domain | Status | Command |
|---|---|---|---|---|---|---|
| Root IPW fuel | `ecu_compat::EcuState::injection_pulse_width(rpm, load)` | `tests/fm0016_core_reducer.rs` | Direct execution test against FM0016 fixtures; IPW vs VE gap via `CoreAdapterContract::IpwVsVeFuelModel` | all fixture cases | adapter-contract | `cargo test --test fm0016_core_reducer` |
| Root ignition timing | `ecu_compat::EcuState::ignition_advance_deg(rpm, load)` | `tests/fm0016_core_reducer.rs` | Direct execution; timing table vs VE via `CoreAdapterContract::TimingTableVsFrozenSpec` | all fixture cases | adapter-contract | `cargo test --test fm0016_core_reducer` |
| Root RPM/sync/faults | `ecu_compat::EcuState::current_rpm()`, `current_synced()`, `current_fault_flags()` | `tests/fm0016_core_reducer.rs` | Direct comparison against fixture inputs | all fixture cases | covered | `cargo test --test fm0016_core_reducer` |

---

## Notes
- **Coupling method**: `Verus lemma` = verus proof inside `ecu-spec/proofs/verus.rs`; `Kani harness` = symbolic test in `ecu-spec/proofs/kani.rs`; `property test` = proptest in test file; `reducer` = differential conformance test.
- **Symbolic domain**: the preconditions under which the coupling proof is valid. Bounds must match the pinned Kani harness bounds in `kani.rs`.
- **Adapter-contract rows**: runtime, scheduler, and root fuel/timing rows are marked adapter-contract because the product uses IPW tables and the spec uses VE models. The adapter contracts document the model gap explicitly via typed Rust enums.
- **Covered rows**: all other rows have a real proof artifact (Verus lemma, Kani harness, or property test) that validates the contract through the product code path.
- **Verus mirrors without Kani bridge**: interpolation, fuel, scheduling, persistence, and TS protocol rows above identify an executable Kani or property-test bridge where one exists. All adapter-contract rows are documented with typed Rust contracts; no blocked rows remain.
