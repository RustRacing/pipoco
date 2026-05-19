use std::collections::BTreeSet;
use std::fmt::Write as _;

use ecu_board_api::{
    AuxOutput, AuxValue, EcuOutput, EdgeKind, IgnitionProfileMode, OutputLevel, TriggerEdge,
};
use ecu_domain::{
    AbsoluteTimeAuthority, CancelReason, CrankSyncState, EnginePhase, EngineTimeAuthority,
    FaultCode, FaultSeverity, Kpa10, Lambda100, Micros, Percent, PhaseSyncState, Rpm, Ticks,
};
use ecu_runtime::Action;
use ecu_sim_core as core_plant;
use ecu_sim_driver::x86_runtime_board::{
    bridge_output_transitions_to_core_frame, run_x86_runtime_tick, X86RuntimeBoard,
    X86RuntimeTickResult,
};

fn synced_sensor_snapshot() -> ecu_board_api::SensorSnapshot {
    let authority = EngineTimeAuthority::new(
        CrankSyncState::PrimaryLocked,
        PhaseSyncState::CamValidated720,
        AbsoluteTimeAuthority::ExpertManual,
        EngineTimeAuthority::MAX_CONFIDENCE_X1000,
        0,
    );

    ecu_board_api::SensorSnapshot::new_with_engine_time_authority(
        Micros::new(1_000),
        Rpm::new(3_000),
        Kpa10::new(450),
        Percent::new(12),
        840,
        550,
        12_500,
        Lambda100::new(100),
        authority,
        EnginePhase::Running,
    )
}

fn unsynced_sensor_snapshot() -> ecu_board_api::SensorSnapshot {
    ecu_board_api::SensorSnapshot::new_with_engine_time_authority(
        Micros::new(1_000),
        Rpm::new(0),
        Kpa10::new(0),
        Percent::new(0),
        840,
        550,
        12_500,
        Lambda100::new(100),
        EngineTimeAuthority::none(),
        EnginePhase::Off,
    )
}

fn synced_board() -> X86RuntimeBoard {
    let mut board = X86RuntimeBoard::new();
    board.set_clock(Micros::new(1_000));
    board.set_sensor_snapshot(synced_sensor_snapshot());
    board
        .set_trigger_edges(&[
            TriggerEdge::new(EdgeKind::Rising, Ticks::new(0)),
            TriggerEdge::new(EdgeKind::Falling, Ticks::new(1)),
        ])
        .unwrap();
    board
}

fn bridge_test_plant_config() -> core_plant::PlantConfig<6> {
    core_plant::PlantConfig {
        cylinder_count: 6,
        cylinder_phase_deg10: [
            core_plant::CrankDeg10(0),
            core_plant::CrankDeg10(1200),
            core_plant::CrankDeg10(2400),
            core_plant::CrankDeg10(3600),
            core_plant::CrankDeg10(4800),
            core_plant::CrankDeg10(6000),
        ],
        displacement_cc: 3000,
        compression_ratio_x100: 1000,
        crank_inertia_x1000: 1000,
        friction_torque_nm_x100: core_plant::TorqueNmX100(1200),
        starter_torque_nm_x100: core_plant::TorqueNmX100(5000),
        physics_mode: core_plant::PlantPhysicsMode::SyntheticTorque,
        engine: core_plant::EngineGeometryConfig {
            bore_um: 86_000,
            stroke_um: 86_000,
            rod_length_um: 143_000,
            displacement_cc: 3000,
            compression_ratio_x100: 1000,
            displacement_tolerance_pct: 5,
        },
        crank: core_plant::CrankConfig {
            inertia_kg_m2_x1e6: 1_000_000,
            starter_torque_nm_x100: core_plant::TorqueNmX100(5000),
            min_substep_us: core_plant::Micros(10),
            max_substep_us: core_plant::Micros(1000),
            max_substep_deg10: 5,
        },
        trigger: core_plant::TriggerConfig {
            crank_teeth: 36,
            missing_teeth: 1,
            cam_pulses: 1,
        },
        air: core_plant::AirConfig {
            load_mode: core_plant::LoadMode::SpeedDensity,
            idle_map_kpa10: core_plant::Kpa10(350),
            wide_open_map_kpa10: core_plant::Kpa10(1000),
            manifold_volume_cc: 2000,
            manifold_filling_enabled: false,
            throttle_area_mm2: 1800,
            throttle_discharge_coeff_x1000: 700,
            ve_table: core_plant::VeTable::constant(1000),
            reference_air_temp_k10: core_plant::Kelvin10(2930),
            reference_pressure_kpa10: core_plant::Kpa10(1013),
        },
        fuel: core_plant::FuelConfig {
            enabled: true,
            stoich_afr_x100: 1470,
            fuel_lhv_j_per_kg: 43_000_000,
            wall_film_enabled: false,
            wall_film_deposit_x1000: 0,
            wall_film_tau_ms: 100,
        },
        spark: core_plant::SparkConfig {
            min_dwell_us: core_plant::Micros(1000),
            mbt_deg10: core_plant::Degrees10(180),
            max_advance_deg10: core_plant::Degrees10(450),
            ignition_delay_deg10: 50,
        },
        combustion: core_plant::CombustionConfig {
            min_air_mass_ug: core_plant::MassUg(1000),
            min_lambda_x1000: 700,
            max_lambda_x1000: 1300,
            torque_scale_x100: 100,
            burn_duration_deg10: 450,
            ca50_target_at_mbt_deg10: 100,
            pmax_target_at_mbt_deg10: 160,
            ca50_sensitivity_x1000: 3,
            pmax_sensitivity_x1000: 1,
            burn_curve: core_plant::BurnCurve::default_wiebe_like(),
        },
        valve_events: core_plant::ValveEvents {
            ivo_deg_btdc_x10: 0,
            ivc_deg_abdc_x10: 500,
            evo_deg_bbdc_x10: 500,
            evc_deg_atdc_x10: 0,
            intake_lift_mm_x100: 900,
            exhaust_lift_mm_x100: 850,
            intake_duration_deg_x10: 2400,
            exhaust_duration_deg_x10: 2350,
        },
        residual: core_plant::ResidualGasConfig {
            enabled: false,
            base_fraction_x1000: 50,
            overlap_gain_x1000: 2,
            low_map_gain_x1000: 1,
            exhaust_backpressure_gain_x1000: 1,
            scavenging_gain_x1000: 5,
        },
        thermo: core_plant::ThermoConfig {
            p_ref_pa: core_plant::PressurePa(101_325),
            initial_cylinder_pressure_pa: core_plant::PressurePa(101_325),
            initial_cylinder_temp_k10: core_plant::Kelvin10(2930),
            gamma_x1000: 1350,
            r_air_j_per_kg_k: 287,
            cv_air_j_per_kg_k: 718,
            wall_heat_loss_x1000: 0,
        },
        losses: core_plant::LossConfig {
            fmep_base_pa: core_plant::PressurePa(20_000),
            fmep_rpm_pa_per_krpm: 8_000,
            fmep_rpm2_pa_per_krpm2: 1_000,
            fmep_load_pa_per_kpa: 0,
            pumping_base_pa: core_plant::PressurePa(5_000),
            pumping_throttle_pa_per_x1000: 0,
            accessory_torque_nm_x100: core_plant::TorqueNmX100(0),
        },
        knock: core_plant::KnockConfig {
            fuel_octane_x10: 950,
            risk_threshold_x1000: 650,
            pmax_weight_x1000: 1,
            temp_weight_x1000: 1,
            advance_weight_x1000: 1,
            compression_weight_x1000: 1,
            octane_credit_x1000: 1,
            rich_margin_credit_x1000: 1,
        },
        dyno: core_plant::DynoConfig {
            mode: core_plant::DynoMode::Disabled,
            fixed_load_torque_nm_x100: core_plant::TorqueNmX100(0),
            target_rpm: core_plant::Rpm(0),
            sweep_start_rpm: core_plant::Rpm(0),
            sweep_end_rpm: core_plant::Rpm(0),
            sweep_step_rpm: core_plant::Rpm(0),
            hold_cycles_before_sample: 20,
            sample_cycles: 4,
            rpm_error_limit: core_plant::Rpm(10),
            pid_kp_x1000: 1000,
            pid_ki_x1000: 0,
            pid_kd_x1000: 0,
        },
        sensors: core_plant::SensorConfig {
            sensor_saturation_enabled: false,
            lambda_transport_enabled: false,
            lambda_delay_crank_deg: 720,
            lambda_sensor_tau_ms: 0,
            lambda_exhaust_mixing_x1000: 0,
        },
    }
}

fn bridge_test_plant() -> core_plant::Plant<6, 128, 12> {
    core_plant::Plant::new(bridge_test_plant_config())
}

fn expand_wasted_spark_pairs(ecu_outputs: &mut core_plant::EcuOutputFrame<6, 12>) {
    // The M50 mega-compatible runtime uses a 3-coil wasted-spark topology.
    // Pair the three ignition channels back onto their companion cylinders so
    // the 6-cylinder plant sees both cylinders in each wasted-spark pair.
    const PAIRED_CYLINDER_BY_IGNITION_CHANNEL: [u8; 3] = [5, 4, 3];
    let spark_events = ecu_outputs.spark_events;

    for spark in spark_events.as_slice().iter().copied() {
        let paired_cylinder = PAIRED_CYLINDER_BY_IGNITION_CHANNEL[spark.cylinder.0 as usize];
        let mut paired_spark = spark;
        paired_spark.cylinder = core_plant::CylinderIndex(paired_cylinder);
        ecu_outputs.spark_events.push(paired_spark).unwrap();
    }
}

fn step_bridge_outputs_into_core_plant(
    mut ecu_outputs: core_plant::EcuOutputFrame<6, 12>,
) -> core_plant::PlantStepOutput<6, 128, 12> {
    expand_wasted_spark_pairs(&mut ecu_outputs);

    let mut plant = bridge_test_plant();
    // Start the plant at a steady crank reference so the bridged 0-degree
    // injection/spark markers are evaluated inside one combustion window.
    // The test keeps the command stream intact and only chooses the plant's
    // initial phase/rpm reference.
    plant.reset(core_plant::InitialPlantState {
        timestamp_us: core_plant::Micros(0),
        rpm: core_plant::Rpm(1200),
        crank_angle_deg10: core_plant::CrankDeg10(0),
        cylinder_fuel_mass_ug: [core_plant::MassUg(0); 6],
    });
    let mut input = core_plant::PlantStepInput::<6, 12>::idle(core_plant::Micros(100_000));
    input.driver.throttle_x1000 = 1000;
    input.ecu_outputs = ecu_outputs;
    let mut output = core_plant::PlantStepOutput::<6, 128, 12>::empty();

    plant.step(&input, &mut output).unwrap();
    output
}

fn result_digest(board: &X86RuntimeBoard, result: &X86RuntimeTickResult) -> String {
    let mut digest = String::new();
    let telemetry = board.telemetry_frame();

    write!(
        digest,
        "now={} step={:?} edges={} outputs={:?} aux={:?} telemetry={:?} diag={:?}",
        result.now_us.get(),
        result.step_result,
        result.trigger_edges.len(),
        board.scheduled_outputs(),
        board.aux_commands(),
        telemetry,
        board.diagnostics(),
    )
    .unwrap();

    digest
}

fn assert_m50_channels(result: &X86RuntimeTickResult) {
    let mut injector_channels = BTreeSet::new();
    let mut ignition_channels = BTreeSet::new();

    for transition in result.scheduled_outputs.iter() {
        match transition.output {
            EcuOutput::Injector(channel) => {
                injector_channels.insert(channel.get());
            }
            EcuOutput::Ignition(channel) => {
                ignition_channels.insert(channel.get());
            }
        }

        assert!(matches!(
            transition.level,
            OutputLevel::High | OutputLevel::Low
        ));
    }

    let expected_injectors: BTreeSet<u8> = [0, 1, 2, 3, 4, 5].into_iter().collect();
    let expected_ignitions: BTreeSet<u8> = [0, 1, 2].into_iter().collect();

    assert_eq!(injector_channels.len(), 6);
    assert_eq!(injector_channels, expected_injectors);
    assert_eq!(ignition_channels, expected_ignitions);
}

#[test]
fn synced_m50_mega_profile_emits_expected_channels_and_telemetry_identity() {
    let mut board = synced_board();
    let result = run_x86_runtime_tick(&mut board).unwrap();

    assert_m50_channels(&result);

    let telemetry = board.telemetry_frame().expect("telemetry frame");
    assert_eq!(telemetry.profile_id.get(), 0x50b2);
    assert_eq!(
        telemetry.ignition_profile_mode,
        IgnitionProfileMode::WastedSpark
    );
    assert_eq!(telemetry.pin_map_id.get(), 23);
    assert_eq!(telemetry.runtime_build_id.get(), 1);
    assert_eq!(telemetry.control_mode, ecu_domain::ControlMode::ClosedLoop);
}

#[test]
fn synced_m50_runtime_outputs_bridge_into_clean_core_plant_commands() {
    let mut board = synced_board();
    let result = run_x86_runtime_tick(&mut board).unwrap();

    let bridge = bridge_output_transitions_to_core_frame::<6, 12>(&result.scheduled_outputs);
    assert!(bridge.diagnostics.is_clean(), "{:?}", bridge.diagnostics);
    assert_eq!(bridge.ecu_outputs.injection_events.len(), 6);
    assert_eq!(bridge.ecu_outputs.spark_events.len(), 6);
    assert!(!bridge.ecu_outputs.injection_events.is_empty());
    assert!(!bridge.ecu_outputs.spark_events.is_empty());

    let output = step_bridge_outputs_into_core_plant(bridge.ecu_outputs);
    assert_eq!(output.consumed_events.injection_count, 6);
    assert_eq!(output.consumed_events.spark_count, 12);
    assert_eq!(output.consumed_events.ignored_injection_count, 0);
    assert_eq!(output.consumed_events.ignored_spark_count, 0);
    assert!(
        output
            .combustion
            .cylinders
            .iter()
            .any(|c| c.misfire.is_none()),
        "{:?}",
        output.combustion
    );
}

#[test]
fn removing_injection_events_suppresses_combustion() {
    let mut board = synced_board();
    let result = run_x86_runtime_tick(&mut board).unwrap();

    let bridge = bridge_output_transitions_to_core_frame::<6, 12>(&result.scheduled_outputs);
    let mut ecu_outputs = bridge.ecu_outputs;
    ecu_outputs.injection_events.clear();

    let output = step_bridge_outputs_into_core_plant(ecu_outputs);
    assert_eq!(output.consumed_events.injection_count, 0);
    assert_eq!(output.consumed_events.spark_count, 12);
    assert_eq!(output.consumed_events.ignored_injection_count, 0);
    assert_eq!(output.consumed_events.ignored_spark_count, 0);
    assert_eq!(output.combustion.total_torque_nm_x100.0, 0);
    assert!(output
        .combustion
        .cylinders
        .iter()
        .all(|c| c.misfire == Some(core_plant::MisfireReason::NoFuel)));
}

#[test]
fn removing_spark_events_suppresses_combustion() {
    let mut board = synced_board();
    let result = run_x86_runtime_tick(&mut board).unwrap();

    let bridge = bridge_output_transitions_to_core_frame::<6, 12>(&result.scheduled_outputs);
    let mut ecu_outputs = bridge.ecu_outputs;
    ecu_outputs.spark_events.clear();

    let output = step_bridge_outputs_into_core_plant(ecu_outputs);
    assert_eq!(output.consumed_events.injection_count, 6);
    assert_eq!(output.consumed_events.spark_count, 0);
    assert_eq!(output.consumed_events.ignored_injection_count, 0);
    assert_eq!(output.consumed_events.ignored_spark_count, 0);
    assert_eq!(output.combustion.total_torque_nm_x100.0, 0);
    assert!(output
        .combustion
        .cylinders
        .iter()
        .all(|c| c.misfire == Some(core_plant::MisfireReason::NoSpark)));
}

#[test]
fn repeated_runs_are_byte_identical() {
    let mut left = synced_board();
    let mut right = synced_board();

    let left_result = run_x86_runtime_tick(&mut left).unwrap();
    let right_result = run_x86_runtime_tick(&mut right).unwrap();

    let left_digest = result_digest(&left, &left_result);
    let right_digest = result_digest(&right, &right_result);

    assert_eq!(left_digest.as_bytes(), right_digest.as_bytes());
    assert_eq!(left_result, right_result);
}

#[test]
fn independent_instances_do_not_share_state() {
    let mut synced = synced_board();
    let mut unsynced = X86RuntimeBoard::new();
    unsynced.set_clock(Micros::new(1_000));
    unsynced.set_sensor_snapshot(unsynced_sensor_snapshot());

    let synced_result = run_x86_runtime_tick(&mut synced).unwrap();
    let unsynced_result = run_x86_runtime_tick(&mut unsynced).unwrap();

    assert_ne!(synced_result, unsynced_result);
    assert!(synced.diagnostics().scheduled_transition_count > 0);
    assert_eq!(unsynced.diagnostics().scheduled_transition_count, 0);
    assert!(unsynced.diagnostics().force_safe_state_count > 0);
}

#[test]
fn fault_shutdown_triggers_cancel_all_and_safe_state() {
    let mut board = synced_board();
    board.runtime_mut().set_fault_state(
        FaultCode::SafetyCut,
        FaultSeverity::Critical,
        CancelReason::SafetyShutdown,
    );

    let result = run_x86_runtime_tick(&mut board).unwrap();

    assert!(result.scheduled_outputs.is_empty());
    assert_eq!(board.diagnostics().cancel_all_count, 1);
    assert_eq!(board.diagnostics().force_safe_state_count, 1);
    assert_eq!(
        board.diagnostics().last_cancel_reason,
        Some(CancelReason::SafetyShutdown)
    );
}

#[test]
fn unsynced_state_reaches_safe_state_diagnostics() {
    let mut board = X86RuntimeBoard::new();
    board.set_clock(Micros::new(1_000));
    board.set_sensor_snapshot(unsynced_sensor_snapshot());

    let result = run_x86_runtime_tick(&mut board).unwrap();

    assert!(result.scheduled_outputs.is_empty());
    assert_eq!(board.diagnostics().cancel_all_count, 0);
    assert!(board.diagnostics().force_safe_state_count > 0);
}

#[test]
fn limp_home_routes_aux_fan_command() {
    let mut board = synced_board();
    board.runtime_mut().set_fault_state(
        FaultCode::SensorOutOfRange,
        FaultSeverity::Warning,
        CancelReason::Manual,
    );

    let result = run_x86_runtime_tick(&mut board).unwrap();

    assert!(matches!(
        result.step_result.operating_mode,
        ecu_domain::ControlMode::LimpHome
    ));
    let aux = board.aux_commands();
    assert!(aux.iter().any(|command| {
        command.output == AuxOutput::Fan && command.value == AuxValue::Level(OutputLevel::High)
    }));
    assert!(board.diagnostics().aux_command_count > 0);
}

#[test]
fn authority_telemetry_gates_outputs_and_sync_loss_cancels() {
    let mut board = X86RuntimeBoard::new();
    board.configure_full_cop();

    let blocked_authority = EngineTimeAuthority::new(
        CrankSyncState::PrimaryLocked,
        PhaseSyncState::CamObserved720,
        AbsoluteTimeAuthority::GeometryOnly,
        EngineTimeAuthority::MAX_CONFIDENCE_X1000,
        0,
    );
    board.set_clock(Micros::new(1_000));
    board.set_sensor_snapshot(
        ecu_board_api::SensorSnapshot::new_with_engine_time_authority(
            Micros::new(1_000),
            Rpm::new(3_000),
            Kpa10::new(450),
            Percent::new(12),
            840,
            550,
            12_500,
            Lambda100::new(100),
            blocked_authority,
            EnginePhase::Running,
        ),
    );
    board
        .set_trigger_edges(&[TriggerEdge::new(EdgeKind::Rising, Ticks::new(0))])
        .unwrap();

    let blocked = run_x86_runtime_tick(&mut board).unwrap();
    assert!(blocked.scheduled_outputs.is_empty());
    assert_eq!(
        board.diagnostics().engine_time.source(),
        AbsoluteTimeAuthority::GeometryOnly
    );
    assert!(!board.diagnostics().engine_time.full_sequential_authorized);
    assert!(blocked
        .step_result
        .actions
        .iter()
        .any(|action| matches!(action, Action::Idle)));

    board.set_clock(Micros::new(2_000));
    board.set_sensor_snapshot(synced_sensor_snapshot());
    board
        .set_trigger_edges(&[TriggerEdge::new(EdgeKind::Rising, Ticks::new(1))])
        .unwrap();

    let validated = run_x86_runtime_tick(&mut board).unwrap();
    let bridge = bridge_output_transitions_to_core_frame::<6, 12>(&validated.scheduled_outputs);
    assert!(board.diagnostics().engine_time.full_sequential_authorized);
    assert_eq!(bridge.ecu_outputs.injection_events.len(), 6);
    assert_eq!(bridge.ecu_outputs.spark_events.len(), 6);

    board.set_clock(Micros::new(3_000));
    board.set_sensor_snapshot(unsynced_sensor_snapshot());
    board.set_trigger_edges(&[]).unwrap();

    let sync_loss = run_x86_runtime_tick(&mut board).unwrap();
    assert!(sync_loss
        .step_result
        .actions
        .iter()
        .any(|action| matches!(action, Action::CancelScheduler(CancelReason::SyncLoss))));
    assert_eq!(board.diagnostics().cancel_all_count, 1);
    assert_eq!(
        board.diagnostics().scheduled_transition_count,
        validated.scheduled_outputs.len()
    );
}
