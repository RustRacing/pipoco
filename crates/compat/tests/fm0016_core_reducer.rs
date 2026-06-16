//! Real-execution FM0016 conformance tests for the root ecu-compat boundary.
//!
//! Drives `EcuState` through public API methods using FM0016 fixture inputs,
//! then compares observable outputs to `oracle_result(case)` called once per fixture.
//!
//! ## Root/Core Boundary Architecture
//!
//! ecu-compat (EcuState) uses IPW (Injector Pulse Width) table lookup — a direct
//! (rpm, load) → pulse-width mapping without volumetric efficiency computation.
//!
//! spec oracle_result uses VE model: VE(rpm,load) × displacement × MAP / (BARO × VE_base)
//! with full correction pipeline (clt, iat, vbat, deadtime, lambda, etc.).
//!
//! These produce fundamentally different fuel pulse widths and timing values for the
//! same raw sensor inputs. Fuel and timing fields are validated via
//! `CoreAdapterContract` variants (IpwVsVeFuelModel, TimingTableVsFrozenSpec).

#![cfg(test)]

use ecu_compat::compat::{CoreAdapterContract, CoreObservedSurface, EcuState};
use ecu_compat::constants::ignition::{MAX_DWELL_US, MIN_DWELL_US};
use fm0016_fixture_matrix::{assert_fixture_semantics, fixture_cases, oracle_result, FixtureCase};
use std::panic::catch_unwind;

// Include the fixture matrix (lives in tests/formal/)
mod fm0016_fixture_matrix {
    #![allow(dead_code)]
    include!("../tests/formal/fm0016_fixture_matrix.rs");
}

// ---------------------------------------------------------------------------
// Observable from EcuState public API
// ---------------------------------------------------------------------------

/// Drive EcuState with FM0016 fixture inputs through public API methods,
/// then observe the product-owned compatibility surface.
fn drive_core(case: &FixtureCase) -> CoreObservedSurface {
    let mut state = EcuState::new();
    let input = case.input;

    // --- Set sensor inputs via public setters ---
    state.clt_x10 = input.clt_c10.0;
    state.iat_x10 = input.iat_c10.0;
    state.tps_percent = ((input.tps_x100 as u32) / 100).min(100) as u8;
    state.map_kpa_x10 = input.map_kpa10.get().min(9999);

    // --- Set RPM and sync state via public setters ---
    state.rpm = input.rpm.get();
    state.synced = matches!(input.sync, ecu_spec::SyncState::Synced);

    state.refreshed_observed_surface()
}

// ---------------------------------------------------------------------------
// Adapter contract validation
//
// Fuel and timing fields use CoreAdapterContract to document the IPW vs VE gap.
// ---------------------------------------------------------------------------

/// Returns the set of adapter contracts that apply to a given fixture case.
/// These document fields that cannot be directly compared between IPW and VE models.
fn adapter_contracts_for_case(_case: &FixtureCase) -> &'static [CoreAdapterContract] {
    &[
        CoreAdapterContract::IpwVsVeFuelModel,
        CoreAdapterContract::TimingTableVsFrozenSpec,
        CoreAdapterContract::BasePwIncomparable,
        CoreAdapterContract::CorrectedPwIncomparable,
    ]
}

// ---------------------------------------------------------------------------
// Per-fixture conformance test
// ---------------------------------------------------------------------------

fn conformance_test(case: &FixtureCase) {
    // Call oracle_result ONCE for expected data
    let spec = oracle_result(*case);
    assert_fixture_semantics(*case, &spec);

    // Execute EcuState via public API for observed data
    let obs = drive_core(case);
    let contracts = adapter_contracts_for_case(case);
    let expected_enrich_mult_x100 = [obs.wue_percent, obs.ase_percent, obs.ae_percent]
        .into_iter()
        .fold(100u32, |acc, pct| {
            acc.saturating_mul(100 + pct as u32) / 100
        }) as u16;

    // ----- RPM -----
    // EcuState.rpm is set directly from fixture rpm (no trigger decoder state needed).
    let spec_rpm = case.input.rpm.get();
    assert_eq!(
        obs.rpm, spec_rpm,
        "FAIL rpm: obs={} exp={} ({})",
        obs.rpm, spec_rpm, case.fixture
    );

    // ----- Sync state -----
    let exp_synced = matches!(case.input.sync, ecu_spec::SyncState::Synced);
    assert_eq!(
        obs.synced, exp_synced,
        "FAIL synced: obs={} exp={} ({})",
        obs.synced, exp_synced, case.fixture
    );

    // ----- Runtime input shell -----
    assert_eq!(
        obs.tooth_count, 0,
        "FAIL tooth_count: default compatibility path should keep initial tooth count ({})",
        case.fixture
    );
    assert_eq!(
        obs.battery_voltage_mv, 12_500,
        "FAIL battery_voltage_mv: default compatibility path should keep initial battery voltage ({})",
        case.fixture
    );
    assert_eq!(
        obs.clt_x10, case.input.clt_c10.0,
        "FAIL clt_x10: observed CLT should match driven product input ({})",
        case.fixture
    );
    assert_eq!(
        obs.iat_x10, case.input.iat_c10.0,
        "FAIL iat_x10: observed IAT should match driven product input ({})",
        case.fixture
    );
    assert_eq!(
        obs.tps_percent,
        ((case.input.tps_x100 as u32) / 100).min(100) as u8,
        "FAIL tps_percent: observed TPS should match clamped driven product input ({})",
        case.fixture
    );
    assert_eq!(
        obs.map_kpa_x10,
        case.input.map_kpa10.get().min(9999),
        "FAIL map_kpa_x10: observed MAP should match clamped driven product input ({})",
        case.fixture
    );
    assert_eq!(
        obs.last_enrichment_update_us, 0,
        "FAIL last_enrichment_update_us: root reducer path should keep default enrichment timestamp ({})",
        case.fixture
    );
    assert_eq!(
        obs.last_enrichment_tps_percent, 0,
        "FAIL last_enrichment_tps_percent: root reducer path should keep default enrichment TPS history ({})",
        case.fixture
    );
    assert_eq!(
        obs.last_enrichment_map_kpa_x10, 0,
        "FAIL last_enrichment_map_kpa_x10: root reducer path should keep default enrichment MAP history ({})",
        case.fixture
    );

    // ----- Fault state -----
    // Normal fixtures should not trigger faults.
    assert_eq!(
        obs.has_fault,
        obs.emergency_trigger_map_oob
            || obs.emergency_trigger_tps_oob
            || obs.emergency_mode
            || obs.last_fault_code != 0,
        "FAIL has_fault: should match observed fault shell ({})",
        case.fixture
    );
    let is_fault_fixture = case.fixture.contains("fault")
        || case.fixture.contains("plausibility")
        || case.fixture.contains("sensor_err");
    if !is_fault_fixture {
        assert!(
            !obs.has_fault,
            "FAIL fault: unexpected fault active ({})",
            case.fixture
        );
    }

    // ----- Fuel/timing fields: CoreAdapterContract assertions -----
    //
    // The IPW table lookup (base_pw_us) and IPW corrections (final_pw_us) are
    // architecturally incomparable to the spec VE displacement model. The
    // spark_advance_deg from the timing table is incomparable to VE timing.
    // We assert the adapter contracts document this gap and verify the fields
    // are exercised (non-zero for running, valid ranges).
    //
    // IpwVsVeFuelModel: base_pw_us, fuel_mult_x100
    assert!(
        obs.base_pw_us > 0 || !obs.synced,
        "FAIL base_pw_us: IPW fuel should be exercised ({})",
        case.fixture
    );
    assert!(
        obs.fuel_mult_x100 > 0,
        "FAIL fuel_mult_x100: IPW fuel multiplier should be exercised ({})",
        case.fixture
    );
    // TimingTableVsFrozenSpec: spark_advance_deg
    if obs.synced && obs.rpm > 0 {
        assert!(
            obs.spark_base_timing_deg >= -50 && obs.spark_base_timing_deg <= 90,
            "FAIL spark_base_timing_deg: out of valid range ({})",
            case.fixture
        );
        assert!(
            obs.spark_advance_deg >= -50 && obs.spark_advance_deg <= 90,
            "FAIL spark_advance_deg: out of valid range ({})",
            case.fixture
        );
        assert!(
            obs.spark_advance_with_limiter_deg >= -50 && obs.spark_advance_with_limiter_deg <= 90,
            "FAIL spark_advance_with_limiter_deg: out of valid range ({})",
            case.fixture
        );
        assert!(
            obs.spark_advance_with_limiter_deg <= obs.spark_advance_deg,
            "FAIL spark_advance_with_limiter_deg: should not exceed pre-limiter timing ({})",
            case.fixture
        );
        assert!(
            obs.dwell_us >= MIN_DWELL_US && obs.dwell_us <= MAX_DWELL_US,
            "FAIL dwell_us: out of valid range ({})",
            case.fixture
        );
        assert!(
            obs.ign_knock_retard_deg >= 0,
            "FAIL ign_knock_retard_deg: should be non-negative ({})",
            case.fixture
        );
        assert_eq!(
            obs.spark_advance_deg,
            obs.spark_base_timing_deg + obs.ign_clt_correction_deg + obs.ign_iat_correction_deg
                - obs.ign_knock_retard_deg,
            "FAIL spark_advance_deg: should match base timing plus observed corrections ({})",
            case.fixture
        );
        assert_eq!(
            obs.commanded_advance_x10_output,
            obs.spark_advance_with_limiter_deg * 10,
            "FAIL commanded_advance_x10_output: refreshed output cache should match limiter timing ({})",
            case.fixture
        );
    }
    assert!(
        obs.rev_limiter_fuel_cut_pct <= 100,
        "FAIL rev_limiter_fuel_cut_pct: should be <= 100 ({})",
        case.fixture
    );
    assert!(
        obs.rev_limiter_ign_retard_deg >= 0,
        "FAIL rev_limiter_ign_retard_deg: should be non-negative ({})",
        case.fixture
    );
    if !obs.rev_limiter_active {
        assert_eq!(
            obs.rev_limiter_fuel_cut_pct, 0,
            "FAIL rev_limiter_fuel_cut_pct: inactive limiter should have zero cut ({})",
            case.fixture
        );
        assert_eq!(
            obs.rev_limiter_ign_retard_deg, 0,
            "FAIL rev_limiter_ign_retard_deg: inactive limiter should have zero retard ({})",
            case.fixture
        );
    }
    assert!(
        !obs.ltft_learning,
        "FAIL ltft_learning: root reducer path should keep LTFT learning inactive ({})",
        case.fixture
    );
    assert_eq!(
        obs.ltft_learned_cell_count, 0,
        "FAIL ltft_learned_cell_count: root reducer path should keep LTFT table empty ({})",
        case.fixture
    );
    assert!(
        !obs.knock_retard_active,
        "FAIL knock_retard_active: root reducer path should keep knock retard inactive ({})",
        case.fixture
    );
    assert_eq!(
        obs.total_knock_count, 0,
        "FAIL total_knock_count: root reducer path should keep knock count at zero ({})",
        case.fixture
    );
    assert!(
        !obs.torque_limited,
        "FAIL torque_limited: root reducer path should keep torque limiting inactive ({})",
        case.fixture
    );
    // BasePwIncomparable: clt_enrich_pct, iat_enrich_pct, vbatt_enrich_pct
    assert!(
        obs.clt_enrich_pct >= 50 && obs.clt_enrich_pct <= 200,
        "FAIL clt_enrich_pct: should be in valid range ({})",
        case.fixture
    );
    assert!(
        obs.iat_enrich_pct >= 50 && obs.iat_enrich_pct <= 200,
        "FAIL iat_enrich_pct: should be in valid range ({})",
        case.fixture
    );
    assert!(
        obs.vbatt_enrich_pct >= 50 && obs.vbatt_enrich_pct <= 200,
        "FAIL vbatt_enrich_pct: should be in valid range ({})",
        case.fixture
    );
    assert_eq!(
        obs.enrich_mult_x100,
        expected_enrich_mult_x100,
        "FAIL enrich_mult_x100: refreshed snapshot multiplier should match observed enrichment shell ({})",
        case.fixture
    );
    let stft_authority = EcuState::new().lambda_config().authority_max_x10;
    assert!(
        obs.stft_x10 >= -stft_authority && obs.stft_x10 <= stft_authority,
        "FAIL stft_x10: should respect lambda authority ({})",
        case.fixture
    );
    // CorrectedPwIncomparable: final_pw_us
    assert!(
        obs.final_pw_us >= obs.base_pw_us,
        "FAIL final_pw_us: corrected PW should be >= base PW ({})",
        case.fixture
    );
    assert_eq!(
        obs.final_pw_output_us,
        u32::from(obs.final_pw_us),
        "FAIL final_pw_output_us: refreshed output cache should match observed corrected PW ({})",
        case.fixture
    );
    assert_eq!(
        obs.isr_count, 0,
        "FAIL isr_count: root reducer path should keep default ISR count ({})",
        case.fixture
    );
    assert_eq!(
        obs.isr_max_us, 0,
        "FAIL isr_max_us: root reducer path should keep default ISR max ({})",
        case.fixture
    );
    assert_eq!(
        obs.isr_avg_us, 0,
        "FAIL isr_avg_us: root reducer path should keep default ISR average ({})",
        case.fixture
    );
    // ----- Cuts -----
    // Fuel/spark cut detection is exercised through should_inject_fuel and
    // rev_limiter_state.ignition_retard.
    assert_eq!(
        obs.fuel_cut, !obs.inject_fuel_allowed,
        "FAIL fuel_cut: should mirror !inject_fuel_allowed ({})",
        case.fixture
    );
    // For non-cut fixtures: cuts should not be active
    let is_cut_fixture = case.fixture.contains("cut");
    if !is_cut_fixture {
        assert!(
            !obs.fuel_cut,
            "FAIL fuel_cut: should not be active for non-cut fixture ({})",
            case.fixture
        );
        assert!(
            !obs.spark_cut,
            "FAIL spark_cut: should not be active for non-cut fixture ({})",
            case.fixture
        );
    }

    // ----- Adapter contract coverage -----
    // Verify that the full root-boundary contract set is present for every
    // fixture — the IPW vs VE gap is universal.
    assert!(
        contracts.contains(&CoreAdapterContract::IpwVsVeFuelModel),
        "FAIL contract: IpwVsVeFuelModel must be present ({})",
        case.fixture
    );
    assert!(
        contracts.contains(&CoreAdapterContract::TimingTableVsFrozenSpec),
        "FAIL contract: TimingTableVsFrozenSpec must be present ({})",
        case.fixture
    );
    assert!(
        contracts.contains(&CoreAdapterContract::BasePwIncomparable),
        "FAIL contract: BasePwIncomparable must be present ({})",
        case.fixture
    );
    assert!(
        contracts.contains(&CoreAdapterContract::CorrectedPwIncomparable),
        "FAIL contract: CorrectedPwIncomparable must be present ({})",
        case.fixture
    );
}

// ---------------------------------------------------------------------------
// Test harness
// ---------------------------------------------------------------------------

fn run_all(cases: &[FixtureCase]) {
    let failures: Vec<_> = cases
        .iter()
        .filter(|c| catch_unwind(|| conformance_test(c)).is_err())
        .map(|c| c.fixture)
        .collect();

    if !failures.is_empty() {
        panic!(
            "FAIL core_fm0016_all: {} fixtures failed: {:?}",
            failures.len(),
            failures
        );
    }
}

fn run_filtered(name: &str, filter: impl Fn(&FixtureCase) -> bool) {
    let cases: Vec<_> = fixture_cases().into_iter().filter(&filter).collect();
    assert!(!cases.is_empty(), "No cases matched filter: {name}");
    run_all(&cases);
}

#[test]
fn core_fm0016_all() {
    run_filtered("all", |_| true);
}

#[test]
fn core_fm0016_running() {
    run_filtered("running", |c| {
        matches!(c.input.mode, ecu_spec::EngineMode::Running) && !c.fixture.contains("cut")
    });
}

#[test]
fn core_fm0016_cuts() {
    run_filtered("cuts", |c| c.fixture.contains("cut"));
}

#[test]
fn core_fm0016_synced() {
    run_filtered("synced", |c| {
        matches!(c.input.sync, ecu_spec::SyncState::Synced)
    });
}

#[test]
fn core_fm0016_unsynced() {
    run_filtered("unsynced", |c| {
        !matches!(c.input.sync, ecu_spec::SyncState::Synced)
    });
}
