use std::collections::BTreeSet;
use std::fmt::Write as _;

use ecu_board_api::{
    AuxOutput, AuxValue, BoardCapabilities, EcuOutput, EdgeKind, IgnitionProfileId,
    IgnitionProfileMode, LoadSourceCapabilities, OutputLevel, OutputTransition,
    OutputTransitionBatch, TriggerEdge,
};
use ecu_board_profiles::{
    check_profile_board_compatibility, conservative_first_start_preset,
    profiles::m50b25tu::{m50_runtime_output_profile, M50B25TU_FULL_COP, M50B25TU_MEGA_COMPAT},
    AuxOutputRole, BoardFirstStartCapabilities, BoardFirstStartSafetyCapabilities,
    FirstStartLoadSource,
};
use ecu_domain::{
    AbsoluteTimeAuthority, CancelReason, ChannelId, CrankSyncState, EnginePhase,
    EngineTimeAuthority, FaultCode, FaultSeverity, Kpa10, Lambda100, Micros, Percent,
    PhaseSyncState, PulseWidthUs, Rpm, Ticks,
};
use ecu_runtime::{
    Action, BaseFuelModel, RuntimeFuelStrategy, RuntimeSemanticAxis16, RuntimeSemanticCalibration,
    RuntimeSemanticCurve16U16, RuntimeSemanticState, RuntimeSemanticTable2dU16,
    RUNTIME_SEMANTIC_TABLE_LEN,
};
use ecu_sim_core as core_plant;
use ecu_sim_core::config::{
    BurnCurve, CombustionConfig, KnockConfig, LossConfig, ResidualGasConfig, ThermoConfig,
    ValveEvents, VeTable,
};
use ecu_sim_core::types::{Kelvin10, PressurePa};
use ecu_sim_driver::{
    bridge_output_transitions_to_core_frame, drive_hifi_runtime_board_tick, run_x86_runtime_tick,
    SimulatorReadinessEvidence, SoftwareReadinessReport, X86HifiAdapterStepInput, X86RuntimeBoard,
    X86RuntimeTickResult,
};
use ecu_sim_hifi::PlantConfig as HifiPlantConfig;

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
    let mut board = X86RuntimeBoard::default();
    board.configure_full_ecu(m50_runtime_output_profile(M50B25TU_MEGA_COMPAT));
    board.configure_fuel_model(test_base_fuel_model());
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

fn test_base_fuel_model() -> BaseFuelModel {
    BaseFuelModel::new(
        [Rpm::new(0); 16],
        [Kpa10::new(0); 16],
        [[PulseWidthUs::new(1_000); 16]; 16],
    )
}

fn semantic_axis2() -> RuntimeSemanticAxis16 {
    RuntimeSemanticAxis16 {
        len: 2,
        values: [0, 2_000, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    }
}

fn semantic_curve_u16(value: u16) -> RuntimeSemanticCurve16U16 {
    RuntimeSemanticCurve16U16 {
        axis: semantic_axis2(),
        values: [value; RUNTIME_SEMANTIC_TABLE_LEN],
    }
}

fn semantic_table_u16(value: u16) -> RuntimeSemanticTable2dU16 {
    RuntimeSemanticTable2dU16 {
        rpm_axis: semantic_axis2(),
        load_axis: semantic_axis2(),
        values: [[value; RUNTIME_SEMANTIC_TABLE_LEN]; RUNTIME_SEMANTIC_TABLE_LEN],
    }
}

fn semantic_shift_strategy(launch_rpm_limit: u16, flat_shift_rpm_min: u16) -> RuntimeFuelStrategy {
    RuntimeFuelStrategy::SpeedDensityVe {
        calibration: RuntimeSemanticCalibration {
            ve_table: semantic_table_u16(7_000),
            afr_target_table: semantic_table_u16(1_470),
            deadtime_table_us: semantic_table_u16(0),
            clt_corr_curve: semantic_curve_u16(1_000),
            iat_corr_curve: semantic_curve_u16(1_000),
            baro_corr_curve: semantic_curve_u16(1_000),
            vbat_corr_curve: semantic_curve_u16(1_000),
            cranking_curve: semantic_curve_u16(1_000),
            afterstart_table: semantic_table_u16(1_000),
            warmup_curve: semantic_curve_u16(1_000),
            ae_tps_threshold_curve: semantic_curve_u16(1_000),
            ae_map_threshold_curve: semantic_curve_u16(1_000),
            ae_shot_curve_us: semantic_curve_u16(0),
            ae_decay_steps_curve: semantic_curve_u16(1),
            ae_decay_ratio_curve_x1000: semantic_curve_u16(1_000),
            required_fuel_us: 1_000,
            pref_kpa10: 1_000,
            stoich_afr_x100: 1_470,
            pw_max_us: 20_000,
            afterstart_window_cycles: 0,
            dfco_entry_rpm: 9_000,
            dfco_exit_rpm: 8_900,
            dfco_entry_tps_x100: 1,
            dfco_exit_tps_x100: 2,
            dfco_entry_map_kpa10: 20,
            dfco_delay_cycles: 1,
            soft_rev_rpm: 9_000,
            hard_rev_rpm: 10_000,
            rev_hysteresis_rpm: 100,
            soft_retard_max_deg10: 0,
            launch_rpm_limit,
            launch_cut_cycles: 0,
            flat_shift_rpm_min,
            flat_shift_cut_cycles: 0,
            knock_threshold_x100: 10_000,
            knock_retard_step_deg10: 0,
            knock_retard_max_deg10: 0,
            knock_recovery_step_deg10: 0,
            knock_recovery_delay_cycles: 0,
            lambda_kp_x1000: 0,
            lambda_ki_x1000: 0,
        },
        state: RuntimeSemanticState::default(),
    }
}

fn semantic_shift_board(launch_rpm_limit: u16, flat_shift_rpm_min: u16) -> X86RuntimeBoard {
    let mut board = synced_board();
    board.configure_runtime_fuel_strategy(semantic_shift_strategy(
        launch_rpm_limit,
        flat_shift_rpm_min,
    ));
    board
}

fn transition(output: EcuOutput, level: OutputLevel, at_us: u32) -> OutputTransition {
    OutputTransition::new(output, level, Ticks::new(at_us))
}

fn transition_batch<const N: usize>(
    transitions: [OutputTransition; N],
) -> OutputTransitionBatch<128> {
    let mut batch = OutputTransitionBatch::<128>::new();
    for transition in transitions {
        batch
            .push(transition)
            .expect("test transition batch capacity");
    }
    batch
}

fn hifi_runtime_test_config() -> HifiPlantConfig {
    let mut cfg = ecu_sim_hifi::default_plant_config();
    cfg.combustion.spark_angle_rad = 15.0_f64.to_radians();
    cfg
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
            ve_table: VeTable::constant(1000),
            reference_air_temp_k10: Kelvin10(2930),
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
        combustion: CombustionConfig {
            min_air_mass_ug: core_plant::MassUg(1000),
            min_lambda_x1000: 700,
            max_lambda_x1000: 1300,
            torque_scale_x100: 100,
            burn_duration_deg10: 450,
            ca50_target_at_mbt_deg10: 100,
            pmax_target_at_mbt_deg10: 160,
            ca50_sensitivity_x1000: 3,
            pmax_sensitivity_x1000: 1,
            burn_curve: BurnCurve::default_hifi_generated(),
        },
        valve_events: ValveEvents {
            ivo_deg_btdc_x10: 0,
            ivc_deg_abdc_x10: 500,
            evo_deg_bbdc_x10: 500,
            evc_deg_atdc_x10: 0,
            intake_lift_mm_x100: 900,
            exhaust_lift_mm_x100: 850,
            intake_duration_deg_x10: 2400,
            exhaust_duration_deg_x10: 2350,
        },
        residual: ResidualGasConfig {
            enabled: false,
            base_fraction_x1000: 50,
            overlap_gain_x1000: 2,
            low_map_gain_x1000: 1,
            exhaust_backpressure_gain_x1000: 1,
            scavenging_gain_x1000: 5,
        },
        thermo: ThermoConfig {
            p_ref_pa: PressurePa(101_325),
            initial_cylinder_pressure_pa: PressurePa(101_325),
            initial_cylinder_temp_k10: Kelvin10(2930),
            gamma_x1000: 1350,
            r_air_j_per_kg_k: 287,
            cv_air_j_per_kg_k: 718,
            wall_heat_loss_x1000: 0,
        },
        losses: LossConfig {
            fmep_base_pa: PressurePa(20_000),
            fmep_rpm_pa_per_krpm: 8_000,
            fmep_rpm2_pa_per_krpm2: 1_000,
            fmep_load_pa_per_kpa: 0,
            pumping_base_pa: PressurePa(5_000),
            pumping_throttle_pa_per_x1000: 0,
            accessory_torque_nm_x100: core_plant::TorqueNmX100(0),
        },
        knock: KnockConfig {
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
        ..core_plant::InitialPlantState::default()
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

fn complete_generic_board_capabilities() -> BoardFirstStartCapabilities {
    let mut capabilities = BoardFirstStartCapabilities::from_board_capabilities(
        BoardCapabilities::new(true, true, true, 3, 6, 7, true, true, true)
            .with_load_sources(LoadSourceCapabilities::new(true, true, true, true)),
    );

    capabilities.sensors.clt = true;
    capabilities.sensors.iat = true;
    capabilities.sensors.vbatt = true;
    capabilities.sensors.vss = true;
    capabilities.sensors.knock_front = true;
    capabilities.sensors.knock_rear = true;
    capabilities.sensors.baro = true;
    capabilities.outputs.aux_outputs = [
        Some(AuxOutputRole::VvtIntake),
        Some(AuxOutputRole::IdleOpen),
        Some(AuxOutputRole::IdleClose),
        Some(AuxOutputRole::FuelPump),
        Some(AuxOutputRole::Fan),
        Some(AuxOutputRole::TachOut),
        Some(AuxOutputRole::Cel),
        None,
        None,
        None,
        None,
    ];
    capabilities.safety = BoardFirstStartSafetyCapabilities {
        sync_loss_cuts_fuel: true,
        sync_loss_cuts_ignition: true,
        trigger_angle_has_authority: true,
    };

    capabilities
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
fn injection_only_profile_schedules_injectors_without_spark_outputs() {
    let mut board = synced_board();
    board.configure_batch_injection(2);

    let result = run_x86_runtime_tick(&mut board).unwrap();

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
    }

    assert_eq!(injector_channels, [0, 1].into_iter().collect());
    assert!(ignition_channels.is_empty());

    let telemetry = board.telemetry_frame().expect("telemetry frame");
    assert_eq!(telemetry.ignition_profile_id, IgnitionProfileId::new(0));
    assert_eq!(
        telemetry.ignition_profile_mode,
        IgnitionProfileMode::Disabled
    );
}

#[test]
fn ignition_only_profile_schedules_spark_without_injector_outputs() {
    let mut board = synced_board();
    board.configure_crank_only_wasted_spark(4);

    let result = run_x86_runtime_tick(&mut board).unwrap();

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
    }

    assert!(injector_channels.is_empty());
    assert_eq!(ignition_channels, [0, 1].into_iter().collect());

    let telemetry = board.telemetry_frame().expect("telemetry frame");
    assert_eq!(telemetry.ignition_profile_id, IgnitionProfileId::new(2));
    assert_eq!(
        telemetry.ignition_profile_mode,
        IgnitionProfileMode::WastedSpark
    );
}

#[test]
fn wasted_spark_profile_identity_uses_normalized_event_count() {
    let mut board = synced_board();
    board.configure_crank_only_wasted_spark(40);

    let result = run_x86_runtime_tick(&mut board).unwrap();

    let mut ignition_channels = BTreeSet::new();
    for transition in result.scheduled_outputs.iter() {
        if let EcuOutput::Ignition(channel) = transition.output {
            ignition_channels.insert(channel.get());
        }
    }

    assert_eq!(
        ignition_channels,
        [0, 2, 3, 4, 5, 6, 7].into_iter().collect()
    );
    let telemetry = board.telemetry_frame().expect("telemetry frame");
    assert_eq!(telemetry.ignition_profile_id, IgnitionProfileId::new(8));
    assert_eq!(
        telemetry.ignition_profile_mode,
        IgnitionProfileMode::WastedSpark
    );
}

#[test]
fn synced_m50_runtime_outputs_bridge_into_clean_core_plant_commands() {
    let mut board = synced_board();
    let result = run_x86_runtime_tick(&mut board).unwrap();

    let bridge = bridge_output_transitions_to_core_frame::<6, 12>(&result.scheduled_outputs);
    assert!(bridge.diagnostics.is_clean(), "{:?}", bridge.diagnostics);
    assert_eq!(bridge.ecu_outputs.injection_events.len(), 6);
    assert_eq!(bridge.ecu_outputs.spark_events.len(), 3);
    assert!(!bridge.ecu_outputs.injection_events.is_empty());
    assert!(!bridge.ecu_outputs.spark_events.is_empty());

    let output = step_bridge_outputs_into_core_plant(bridge.ecu_outputs);
    assert_eq!(output.consumed_events.injection_count, 6);
    assert_eq!(output.consumed_events.spark_count, 6);
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
fn default_shift_arming_leaves_semantic_cuts_inactive() {
    let mut board = semantic_shift_board(2_500, 9_000);

    let result = run_x86_runtime_tick(&mut board).unwrap();

    assert!(!result.step_result.control.fuel_intent.fuel_cut);
    assert!(!result.step_result.control.fuel_intent.spark_cut);
}

#[test]
fn launch_shift_arming_can_trigger_semantic_cut() {
    let mut board = semantic_shift_board(2_500, 9_000);
    board.set_shift_arming(true, false);

    let result = run_x86_runtime_tick(&mut board).unwrap();

    assert!(result.step_result.control.fuel_intent.fuel_cut);
    assert!(result.step_result.control.fuel_intent.spark_cut);
}

#[test]
fn flat_shift_arming_can_trigger_semantic_cut() {
    let mut board = semantic_shift_board(9_000, 2_500);
    board.set_shift_arming(false, true);

    let result = run_x86_runtime_tick(&mut board).unwrap();

    assert!(result.step_result.control.fuel_intent.fuel_cut);
    assert!(result.step_result.control.fuel_intent.spark_cut);
}

#[test]
fn authority_aware_ingress_preserves_sensor_authority() {
    let mut board = X86RuntimeBoard::default();
    board
        .runtime_mut()
        .configure_full_ecu(m50_runtime_output_profile(M50B25TU_FULL_COP));

    let authority = EngineTimeAuthority::new(
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
            authority,
            EnginePhase::Running,
        ),
    );
    board
        .set_trigger_edges(&[TriggerEdge::new(EdgeKind::Rising, Ticks::new(0))])
        .unwrap();

    let result = run_x86_runtime_tick(&mut board).unwrap();

    assert_eq!(result.sensor_snapshot.engine_time.authority, authority);
    assert_eq!(board.diagnostics().engine_time.authority, authority);
    assert_eq!(
        board.diagnostics().engine_time.source(),
        AbsoluteTimeAuthority::GeometryOnly
    );
}

#[test]
fn runtime_bridge_reports_short_injector_pulse_width() {
    let batch = transition_batch([
        transition(
            EcuOutput::Injector(ChannelId::new(0)),
            OutputLevel::High,
            100,
        ),
        transition(
            EcuOutput::Injector(ChannelId::new(0)),
            OutputLevel::Low,
            100,
        ),
    ]);

    let bridge = bridge_output_transitions_to_core_frame::<4, 1>(&batch);

    assert_eq!(bridge.diagnostics.short_pulse_width_count, 1);
    assert_eq!(bridge.diagnostics.short_dwell_count, 0);
    assert!(!bridge.diagnostics.is_clean());
    assert_eq!(bridge.ecu_outputs.injection_events.len(), 1);
    assert_eq!(
        bridge.ecu_outputs.injection_events.as_slice()[0]
            .pulse_width_us
            .0,
        1
    );
}

#[test]
fn runtime_bridge_reports_short_spark_dwell() {
    let batch = transition_batch([
        transition(
            EcuOutput::Ignition(ChannelId::new(2)),
            OutputLevel::High,
            250,
        ),
        transition(
            EcuOutput::Ignition(ChannelId::new(2)),
            OutputLevel::Low,
            250,
        ),
    ]);

    let bridge = bridge_output_transitions_to_core_frame::<4, 1>(&batch);

    assert_eq!(bridge.diagnostics.short_pulse_width_count, 0);
    assert_eq!(bridge.diagnostics.short_dwell_count, 1);
    assert!(!bridge.diagnostics.is_clean());
    assert_eq!(bridge.ecu_outputs.spark_events.len(), 1);
    assert_eq!(bridge.ecu_outputs.spark_events.as_slice()[0].dwell_us.0, 1);
}

#[test]
fn runtime_bridge_reports_open_high_and_out_of_order_transitions() {
    let open_high = transition_batch([transition(
        EcuOutput::Injector(ChannelId::new(0)),
        OutputLevel::High,
        100,
    )]);
    let open_high_bridge = bridge_output_transitions_to_core_frame::<4, 2>(&open_high);
    assert_eq!(open_high_bridge.diagnostics.open_high_count, 1);
    assert_eq!(open_high_bridge.ecu_outputs.injection_events.len(), 0);
    assert!(!open_high_bridge.diagnostics.is_clean());

    let out_of_order = transition_batch([
        transition(
            EcuOutput::Injector(ChannelId::new(0)),
            OutputLevel::High,
            200,
        ),
        transition(
            EcuOutput::Injector(ChannelId::new(0)),
            OutputLevel::Low,
            100,
        ),
        transition(
            EcuOutput::Ignition(ChannelId::new(2)),
            OutputLevel::High,
            400,
        ),
        transition(
            EcuOutput::Ignition(ChannelId::new(2)),
            OutputLevel::Low,
            300,
        ),
    ]);
    let out_of_order_bridge = bridge_output_transitions_to_core_frame::<4, 2>(&out_of_order);
    assert_eq!(
        out_of_order_bridge
            .diagnostics
            .out_of_order_transition_count,
        2
    );
    assert_eq!(out_of_order_bridge.diagnostics.short_pulse_width_count, 0);
    assert_eq!(out_of_order_bridge.diagnostics.short_dwell_count, 0);
    assert_eq!(out_of_order_bridge.ecu_outputs.injection_events.len(), 0);
    assert_eq!(out_of_order_bridge.ecu_outputs.spark_events.len(), 0);
}

#[test]
fn runtime_bridge_reports_duplicate_orphan_range_and_capacity_errors() {
    let malformed = transition_batch([
        transition(
            EcuOutput::Injector(ChannelId::new(0)),
            OutputLevel::High,
            100,
        ),
        transition(
            EcuOutput::Injector(ChannelId::new(0)),
            OutputLevel::High,
            110,
        ),
        transition(
            EcuOutput::Injector(ChannelId::new(0)),
            OutputLevel::Low,
            200,
        ),
        transition(
            EcuOutput::Ignition(ChannelId::new(0)),
            OutputLevel::Low,
            300,
        ),
        transition(
            EcuOutput::Ignition(ChannelId::new(3)),
            OutputLevel::High,
            400,
        ),
    ]);
    let malformed_bridge = bridge_output_transitions_to_core_frame::<1, 2>(&malformed);
    assert_eq!(malformed_bridge.diagnostics.duplicate_high_count, 1);
    assert_eq!(malformed_bridge.diagnostics.orphan_low_count, 1);
    assert_eq!(malformed_bridge.diagnostics.out_of_range_channel_count, 1);
    assert_eq!(malformed_bridge.diagnostics.unmapped_channel_count, 0);
    assert!(!malformed_bridge.diagnostics.is_clean());

    let overflow = transition_batch([
        transition(
            EcuOutput::Injector(ChannelId::new(0)),
            OutputLevel::High,
            100,
        ),
        transition(
            EcuOutput::Injector(ChannelId::new(0)),
            OutputLevel::Low,
            200,
        ),
        transition(
            EcuOutput::Injector(ChannelId::new(1)),
            OutputLevel::High,
            300,
        ),
        transition(
            EcuOutput::Injector(ChannelId::new(1)),
            OutputLevel::Low,
            400,
        ),
    ]);
    let overflow_bridge = bridge_output_transitions_to_core_frame::<2, 1>(&overflow);
    assert_eq!(overflow_bridge.ecu_outputs.injection_events.len(), 1);
    assert_eq!(overflow_bridge.diagnostics.capacity_overflow_count, 1);
    assert!(!overflow_bridge.diagnostics.is_clean());
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
    assert_eq!(output.consumed_events.spark_count, 6);
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
    let mut unsynced = X86RuntimeBoard::default();
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
    board.set_fault_state(
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
    let mut board = X86RuntimeBoard::default();
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
    board.set_fault_state(
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
        command.output == AuxOutput::SafetyRelay(1)
            && command.value == AuxValue::Level(OutputLevel::High)
    }));
    assert!(board.diagnostics().aux_command_count > 0);
}

#[test]
fn authority_telemetry_gates_outputs_and_sync_loss_cancels() {
    let mut board = X86RuntimeBoard::default();
    board
        .runtime_mut()
        .configure_full_ecu(m50_runtime_output_profile(M50B25TU_FULL_COP));

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

#[test]
fn force_safe_state_records_low_transition_for_already_high_output() {
    let mut board = X86RuntimeBoard::default();
    board
        .runtime_mut()
        .configure_full_ecu(m50_runtime_output_profile(M50B25TU_FULL_COP));
    board.set_clock(Micros::new(1_000));
    board.set_sensor_snapshot(synced_sensor_snapshot());
    board
        .set_trigger_edges(&[TriggerEdge::new(EdgeKind::Rising, Ticks::new(1))])
        .unwrap();

    let validated = run_x86_runtime_tick(&mut board).unwrap();
    assert!(validated
        .step_result
        .actions
        .iter()
        .any(|action| matches!(action, Action::ArmIgnition(_))));

    let output = EcuOutput::Ignition(ChannelId::new(2));
    board.mark_output_high_for_test(output, Ticks::new(2_500));
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

    let safe_state_outputs = board.safe_state_outputs();
    let forced_low = safe_state_outputs
        .iter()
        .find(|transition| transition.output == output && transition.level == OutputLevel::Low)
        .copied()
        .expect("safe state should emit a low transition for the energized output");

    assert_eq!(forced_low.at, Ticks::new(3_000));
    assert_eq!(
        forced_low.at.get().saturating_sub(2_500),
        500,
        "safe-state low transition preserves observable pulse-width evidence"
    );
}

#[test]
fn hifi_runtime_tick_feeds_runtime_board_from_adapter_step() {
    let mut board = X86RuntimeBoard::default();
    board.configure_full_ecu(m50_runtime_output_profile(M50B25TU_MEGA_COMPAT));
    board.configure_fuel_model(test_base_fuel_model());

    let outputs = transition_batch([
        transition(
            EcuOutput::Injector(ChannelId::new(0)),
            OutputLevel::High,
            100,
        ),
        transition(
            EcuOutput::Injector(ChannelId::new(0)),
            OutputLevel::Low,
            4_100,
        ),
        transition(
            EcuOutput::Ignition(ChannelId::new(1)),
            OutputLevel::High,
            200,
        ),
        transition(
            EcuOutput::Ignition(ChannelId::new(1)),
            OutputLevel::Low,
            1_700,
        ),
    ]);

    let (step, runtime) = drive_hifi_runtime_board_tick::<4, 128>(
        &mut board,
        &hifi_runtime_test_config(),
        &outputs,
        X86HifiAdapterStepInput {
            now_us: 10_000,
            window_us: 2_000,
            throttle_x1000: 500,
            battery_mv: 12_800,
            load_torque_nm_x100: 300,
            injector_flow_kg_per_s: 0.02,
            injector_deadtime_us: 700,
            crank_ref: None,
        },
    )
    .unwrap();

    assert!(!step.trigger_edges.is_empty());
    assert!(runtime.diagnostics.synced);
    assert!(!runtime.trigger_edges.is_empty());
    assert!(runtime.diagnostics.drained_trigger_edges > 0);
    assert!(runtime.sensor_snapshot.rpm.get() > 0);
    assert_eq!(
        runtime.sensor_snapshot.rpm,
        ecu_domain::Rpm::new(step.sensor_frame.rpm.get())
    );
    assert_eq!(
        runtime.sensor_snapshot.map,
        ecu_domain::Kpa10::new(step.sensor_frame.map_kpa10)
    );
    assert_eq!(
        runtime.sensor_snapshot.throttle,
        ecu_domain::Percent::new((step.sensor_frame.tps_x1000 / 10).min(100) as u8)
    );
}

#[test]
fn software_readiness_report_aggregates_profile_simulator_ts_and_metadata_gates() {
    let profile = M50B25TU_MEGA_COMPAT;
    let preset = conservative_first_start_preset(&profile, FirstStartLoadSource::MapSpeedDensity);
    let compatibility = check_profile_board_compatibility(
        &profile,
        &preset,
        &complete_generic_board_capabilities(),
    );

    let mut first = synced_board();
    let first_result = run_x86_runtime_tick(&mut first).unwrap();
    let mut second = synced_board();
    let second_result = run_x86_runtime_tick(&mut second).unwrap();
    let mut faulted = synced_board();
    faulted.set_fault_state(
        FaultCode::SafetyCut,
        FaultSeverity::Critical,
        CancelReason::SafetyShutdown,
    );
    let faulted_result = run_x86_runtime_tick(&mut faulted).unwrap();

    let report = SoftwareReadinessReport {
        compatibility,
        simulator: SimulatorReadinessEvidence {
            sync_acquired: first.diagnostics().synced,
            output_schedule_seen: !first_result.scheduled_outputs.is_empty(),
            fault_cut_seen: faulted_result.scheduled_outputs.is_empty()
                && faulted.diagnostics().force_safe_state_count > 0,
            deterministic_replay: first_result == second_result,
        },
        ts_pages_valid: true,
        evidence_metadata_valid: true,
    };

    assert!(report.ready(), "{:?}", report);
}
