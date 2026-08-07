use super::*;
use crate::compat::StepInputs;
use ecu_board_api::AuxOutput;
#[cfg(test)]
use ecu_calibration::{
    ExpertIgnitionMode, ExpertInjectionLayout, ExpertTriggerCalibration, ExpertUnlock,
    SecondaryTriggerMode, TriggerAuthority,
};

pub(super) const INLINE_SIX_FIRING_ORDER: [CylinderId; 6] = [
    CylinderId::new(1),
    CylinderId::new(5),
    CylinderId::new(3),
    CylinderId::new(6),
    CylinderId::new(2),
    CylinderId::new(4),
];

pub(super) const INLINE_FULL_ECU_AUX_SAFETY: AuxSafetyProfile = AuxSafetyProfile::off_on_limp([
    AuxOutput::Pwm(ChannelId::new(0)),
    AuxOutput::Digital(ChannelId::new(0)),
    AuxOutput::Digital(ChannelId::new(1)),
]);

pub(super) const fn inline_sequential_wasted_spark_profile() -> FullEcuOutputProfile {
    FullEcuOutputProfile::sequential_wasted_spark(
        INLINE_SIX_FIRING_ORDER,
        6,
        3,
        INLINE_FULL_ECU_AUX_SAFETY,
        OutputAuthorityRequirement::FullSequential720,
    )
}
pub(super) const fn inline_sequential_cop_profile() -> FullEcuOutputProfile {
    FullEcuOutputProfile::sequential_coil_on_plug(
        INLINE_SIX_FIRING_ORDER,
        6,
        6,
        INLINE_FULL_ECU_AUX_SAFETY,
        OutputAuthorityRequirement::FullSequential720,
    )
}
pub(super) fn test_fuel_model() -> BaseFuelModel {
    test_fuel_model_with_base_pw(2500)
}
pub(super) fn test_fuel_model_with_base_pw(pw_us: u16) -> BaseFuelModel {
    let rpm_bins = [
        Rpm::new(500),
        Rpm::new(1000),
        Rpm::new(1500),
        Rpm::new(2000),
        Rpm::new(2500),
        Rpm::new(3000),
        Rpm::new(3500),
        Rpm::new(4000),
        Rpm::new(4500),
        Rpm::new(5000),
        Rpm::new(5500),
        Rpm::new(6000),
        Rpm::new(6500),
        Rpm::new(7000),
        Rpm::new(7500),
        Rpm::new(8000),
    ];
    let load_bins = [
        Kpa10::new(200),
        Kpa10::new(300),
        Kpa10::new(400),
        Kpa10::new(500),
        Kpa10::new(600),
        Kpa10::new(700),
        Kpa10::new(800),
        Kpa10::new(900),
        Kpa10::new(1000),
        Kpa10::new(1100),
        Kpa10::new(1200),
        Kpa10::new(1300),
        Kpa10::new(1400),
        Kpa10::new(1500),
        Kpa10::new(1600),
        Kpa10::new(1700),
    ];
    let mut pulse_widths = [[ecu_domain::PulseWidthUs::new(0); 16]; 16];
    pulse_widths[5][5] = ecu_domain::PulseWidthUs::new(pw_us as u32);
    BaseFuelModel::new(rpm_bins, load_bins, pulse_widths)
}
pub(super) fn authority(
    crank: CrankSyncState,
    phase: PhaseSyncState,
    absolute: AbsoluteTimeAuthority,
) -> EngineTimeAuthority {
    EngineTimeAuthority::new(
        crank,
        phase,
        absolute,
        EngineTimeAuthority::MAX_CONFIDENCE_X1000,
        0,
    )
}
pub(super) fn validated_expert_authority() -> EngineTimeAuthority {
    let calibration = ExpertTriggerCalibration {
        expert_unlock: ExpertUnlock::Unlocked,
        authority: TriggerAuthority::ExpertManual,
        profile_identity: 0x4D353054,
        profile_hash: 0xA5A5_1234,
        secondary_trigger_mode: SecondaryTriggerMode::SingleToothCam,
        ignition_mode: ExpertIgnitionMode::SequentialCop,
        injection_layout: ExpertInjectionLayout::Sequential,
        ..ExpertTriggerCalibration::default()
    };
    let startup_authority = authority(
        CrankSyncState::PrimaryLocked,
        PhaseSyncState::CamValidated720,
        AbsoluteTimeAuthority::GeometryOnly,
    );

    calibration
        .to_runtime_engine_time_authority(startup_authority)
        .expect("validated expert authority")
}
pub(super) fn test_timed_injection(channel: u8, start_at: u32, end_at: u32) -> TimedInjectionPlan {
    TimedInjectionPlan {
        plan: InjectionPlan {
            output: ExclusiveChannel::new(OutputGroup::Injector, ChannelId::new(channel)),
            pulse_width: PulseWidthUs::new(end_at.saturating_sub(start_at)),
        },
        start_at: Micros::new(start_at),
        end_at: Micros::new(end_at),
    }
}
pub(super) fn test_timed_ignition(channel: u8, start_at: u32, end_at: u32) -> TimedIgnitionPlan {
    TimedIgnitionPlan {
        plan: ecu_scheduler::IgnitionPlan {
            output: ExclusiveChannel::new(OutputGroup::Ignition, ChannelId::new(channel)),
            dwell: DwellUs::new(end_at.saturating_sub(start_at) as u16),
            advance: Degrees10::new(120),
        },
        start_at: Micros::new(start_at),
        end_at: Micros::new(end_at),
    }
}
pub(super) fn running_step_inputs(
    now_us: u32,
    rpm: u32,
    trigger_synced: bool,
    cam_seen: bool,
) -> StepInputs {
    StepInputs {
        now_us: Micros::new(now_us),
        rpm,
        load_kpa10: 700,
        angle_x10: 2_000,
        trigger_synced,
        cam_seen,
        launch_armed: false,
        flat_shift_armed: false,
        safety_latch_request: false,
    }
}
pub(super) fn running_control_inputs(now_us: u32, rpm: u16) -> ControlInputs {
    ControlInputs {
        enrichment: EnrichmentInputs {
            now_us: Micros::new(now_us),
            clt_c: 20,
            cranking: false,
            just_started: false,
            tpsdot_pct_s: 0,
            mapdot_kpa_s: 0,
        },
        lambda: LambdaTrimInputs {
            now_us: Micros::new(now_us),
            clt_c: 80,
            just_started: false,
            lambda_valid: true,
            measured_lambda100: ecu_domain::Lambda100::new(100),
            requested_open_loop: false,
        },
        torque: TorqueInputs::new(90, 90, 90, 90, 90),
        ignition: IgnitionInputs::new(
            ecu_domain::Degrees10::new(100),
            0,
            0,
            0,
            false,
            Rpm::new(rpm),
        ),
        fuel_sensors: FuelSensorInputs::default(),
        knock_intensity_x100: 0,
    }
}
pub(super) fn spark_only_control_inputs(now_us: u32, rpm: u16) -> ControlInputs {
    ControlInputs::spark_only(
        Micros::new(now_us),
        IgnitionInputs::new(Degrees10::new(100), 0, 0, 0, false, Rpm::new(rpm)),
    )
}
pub(super) fn differential_running_input(mode: RuntimeEngineMode) -> DifferentialInputSnapshot {
    DifferentialInputSnapshot {
        now_us: Micros::new(1_000),
        rpm: Rpm::new(3_000),
        map_kpa10: Kpa10::new(700),
        load_kpa10: Kpa10::new(700),
        angle_x10: Degrees10::new(2_000),
        clt_c10: 200,
        iat_c10: 250,
        baro_kpa10: Kpa10::new(1_010),
        vbatt_mv: 12_000,
        sync: SyncState::Locked { cam_ref: false },
        fuel_cut: false,
        spark_cut: false,
        mode,
        target_afr_override_x100: RuntimeAfrOverride::None,
        launch_armed: false,
        flat_shift_armed: false,
        safety_latch_request: false,
    }
}
pub(super) fn arm_scheduler_count<const N: usize>(actions: ActionBatch<N>) -> usize {
    actions
        .iter()
        .filter(|action| matches!(*action, Action::ArmScheduler { .. }))
        .count()
}
pub(super) fn arm_ignition_count<const N: usize>(actions: ActionBatch<N>) -> usize {
    actions
        .iter()
        .filter(|action| matches!(*action, Action::ArmIgnition(_)))
        .count()
}
pub(super) fn arm_injection_count<const N: usize>(actions: ActionBatch<N>) -> usize {
    actions
        .iter()
        .filter(|action| matches!(*action, Action::ArmInjection(_)))
        .count()
}
pub(super) fn canonical_runtime_input() -> SpecInputSnapshot {
    SpecInputSnapshot {
        t_us: ecu_spec::Micros::new(10_000),
        rpm: ecu_spec::Rpm::new(1000),
        map_kpa10: ecu_spec::Kpa10::new(1000),
        load_kpa10: ecu_spec::Kpa10::new(1000),
        tps_x100: 0,
        clt_c10: ecu_spec::TempC10::new(4),
        iat_c10: ecu_spec::TempC10::new(20),
        baro_kpa10: ecu_spec::Kpa10::new(1000),
        vbatt_mv: ecu_spec::Millivolts::new(12_000),
        knock_intensity_x100: 0,
        launch_armed: false,
        flat_shift_armed: false,
        sync: ecu_spec::SyncState::Synced,
        fuel_cut: false,
        spark_cut: false,
        mode: ecu_spec::EngineMode::Running,
        target_afr_override_x100: ecu_spec::AfrOverride::None,
    }
}
pub(super) fn assert_within(
    field: &str,
    runtime: u32,
    oracle: u32,
    tolerance: u32,
    input: &SpecInputSnapshot,
    fixture: &str,
) {
    let difference = runtime.abs_diff(oracle);
    assert!(
            difference <= tolerance,
            "input_snapshot={input:?}\ncalibration_fixture={fixture}\nruntime_output={runtime}\noracle_output={oracle}\nfield={field}\ndifference={difference}\ntolerance={tolerance}"
        );
}
pub(super) fn semantic_axis2() -> RuntimeSemanticAxis16 {
    let mut values = [0u16; RUNTIME_SEMANTIC_TABLE_LEN];
    values[0] = 100;
    values[1] = 200;
    RuntimeSemanticAxis16 { len: 2, values }
}
pub(super) fn semantic_table_u16(value: u16) -> RuntimeSemanticTable2dU16 {
    RuntimeSemanticTable2dU16 {
        rpm_axis: semantic_axis2(),
        load_axis: semantic_axis2(),
        values: [[value; RUNTIME_SEMANTIC_TABLE_LEN]; RUNTIME_SEMANTIC_TABLE_LEN],
    }
}
pub(super) fn semantic_deadtime_table_u16(value: u16) -> RuntimeSemanticDeadtimeTableU16 {
    RuntimeSemanticDeadtimeTableU16 {
        vbat_mv_axis: semantic_axis2(),
        pressure_kpa10_axis: semantic_axis2(),
        values: [[value; RUNTIME_SEMANTIC_TABLE_LEN]; RUNTIME_SEMANTIC_TABLE_LEN],
    }
}
pub(super) fn semantic_table_i16(value: i16) -> RuntimeSemanticTable2dI16 {
    RuntimeSemanticTable2dI16 {
        rpm_axis: semantic_axis2(),
        load_axis: semantic_axis2(),
        values: [[value; RUNTIME_SEMANTIC_TABLE_LEN]; RUNTIME_SEMANTIC_TABLE_LEN],
    }
}
pub(super) fn semantic_table_u32(value: u32) -> RuntimeSemanticTable2dU32 {
    RuntimeSemanticTable2dU32 {
        rpm_axis: semantic_axis2(),
        load_axis: semantic_axis2(),
        values: [[value; RUNTIME_SEMANTIC_TABLE_LEN]; RUNTIME_SEMANTIC_TABLE_LEN],
    }
}
pub(super) fn semantic_schedule_calibration(dwell_us: u32) -> RuntimeSemanticScheduleCalibration {
    let mut phases = [0u16; RUNTIME_SEMANTIC_TABLE_LEN];
    phases[0] = 0;
    RuntimeSemanticScheduleCalibration {
        spark_advance_table_deg10: semantic_table_i16(150),
        dwell_table_us: semantic_table_u32(dwell_us),
        injection_target_table_deg10: semantic_table_u16(360),
        injection_angle_mode: RuntimeSemanticInjectionAngleMode::EndOfInjection,
        cylinder_phase_deg10: RuntimeSemanticCylinderArrayU16 {
            count: 1,
            values: phases,
        },
    }
}
pub(super) fn semantic_schedule_input(rpm: u16, sync: SyncState) -> RuntimeSemanticInputSnapshot {
    RuntimeSemanticInputSnapshot {
        t_us: Micros::new(0),
        rpm: Rpm::new(rpm),
        map_kpa10: Kpa10::new(100),
        load_kpa10: Kpa10::new(100),
        tps_x100: 0,
        clt_c10: 800,
        iat_c10: 250,
        baro_kpa10: Kpa10::new(1000),
        vbatt_mv: 12_000,
        lambda_valid: true,
        lambda_measured: Lambda100::new(100),
        requested_open_loop: false,
        knock_intensity_x100: 0,
        launch_armed: false,
        flat_shift_armed: false,
        sync,
        fuel_cut: false,
        spark_cut: false,
        direct_fuel_cut_request: false,
        direct_spark_cut_request: false,
        safety_latch_request: false,
        mode: RuntimeSemanticEngineMode::Running,
        target_afr_override_x100: RuntimeSemanticAfrOverride::None,
    }
}
pub(super) fn semantic_curve_u16(value: u16) -> RuntimeSemanticCurve16U16 {
    RuntimeSemanticCurve16U16 {
        axis: semantic_axis2(),
        values: [value; RUNTIME_SEMANTIC_TABLE_LEN],
    }
}
pub(super) fn semantic_fuel_calibration_with_ve_cells(
    map_cell: u16,
    tps_cell: u16,
) -> RuntimeSemanticCalibration {
    let mut ve_values = [[map_cell; RUNTIME_SEMANTIC_TABLE_LEN]; RUNTIME_SEMANTIC_TABLE_LEN];
    ve_values[0][1] = tps_cell;

    RuntimeSemanticCalibration {
        ve_table: RuntimeSemanticTable2dU16 {
            rpm_axis: semantic_axis2(),
            load_axis: semantic_axis2(),
            values: ve_values,
        },
        afr_target_table: semantic_table_u16(1470),
        deadtime_table_us: semantic_deadtime_table_u16(0),
        clt_corr_curve: semantic_curve_u16(1000),
        iat_corr_curve: semantic_curve_u16(1000),
        baro_corr_curve: semantic_curve_u16(1000),
        vbat_corr_curve: semantic_curve_u16(1000),
        cranking_curve: semantic_curve_u16(1000),
        afterstart_table: semantic_table_u16(1000),
        warmup_curve: semantic_curve_u16(1000),
        ae_tps_threshold_curve: semantic_curve_u16(1000),
        ae_map_threshold_curve: semantic_curve_u16(1000),
        ae_shot_curve_us: semantic_curve_u16(0),
        ae_decay_steps_curve: semantic_curve_u16(1),
        ae_decay_ratio_curve_x1000: semantic_curve_u16(1000),
        required_fuel_us: 1000,
        pref_kpa10: 1000,
        stoich_afr_x100: 1470,
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
        idle_target_rpm: 0,
        idle_base_duty_x1000: 0,
        idle_kp_x1000: 0,
        idle_ki_x1000: 0,
        launch_rpm_limit: 9_000,
        launch_cut_cycles: 0,
        flat_shift_rpm_min: 9_000,
        flat_shift_cut_cycles: 0,
        knock_threshold_x100: 10_000,
        knock_retard_step_deg10: 0,
        knock_retard_max_deg10: 0,
        knock_recovery_step_deg10: 0,
        knock_recovery_delay_cycles: 0,
        lambda_kp_x1000: 0,
        lambda_ki_x1000: 0,
    }
}
pub(super) fn semantic_fuel_observations(
    pw_corr_us: u32,
    fuel_cut: bool,
    spark_cut: bool,
) -> RuntimeSemanticFuelObservations {
    RuntimeSemanticFuelObservations {
        ve_pct_x100: 8000,
        target_afr_x100: 1470,
        pw_base_us: 1000,
        pw_air_us: 1000,
        pw_corr_us,
        warmup_corr_x1000: 1000,
        fuel_cut,
        spark_cut,
        lambda_correction_x1000: 1000,
        lambda_integrator_state: RuntimeSemanticPiIntegratorState::default(),
        idle_duty_x1000: 0,
        idle_integrator_state: RuntimeSemanticPiIntegratorState {
            acc: 0,
            min_acc: -2000,
            max_acc: 2000,
            frozen: false,
        },
        advance_deg10_trim: 0,
    }
}
pub(super) fn semantic_fuel_state_calibration() -> RuntimeSemanticCalibration {
    let mut calibration = semantic_fuel_calibration_with_ve_cells(100, 100);
    calibration.warmup_curve = semantic_curve_u16(1200);
    calibration.afterstart_table = semantic_table_u16(1200);
    calibration.afterstart_window_cycles = 3;
    calibration.ae_tps_threshold_curve = semantic_curve_u16(1);
    calibration.ae_map_threshold_curve = semantic_curve_u16(1);
    calibration.ae_shot_curve_us = semantic_curve_u16(250);
    calibration.ae_decay_steps_curve = semantic_curve_u16(2);
    calibration.ae_decay_ratio_curve_x1000 = semantic_curve_u16(1000);
    calibration
}
pub(super) fn semantic_launch_and_flat_shift_calibration() -> RuntimeSemanticCalibration {
    let mut calibration = semantic_fuel_calibration_with_ve_cells(7000, 7000);
    calibration.launch_rpm_limit = 2_500;
    calibration.launch_cut_cycles = 0;
    calibration.flat_shift_rpm_min = 2_500;
    calibration.flat_shift_cut_cycles = 0;
    calibration
}
pub(super) fn assert_invalid_full_ecu_profile_is_inert(profile: FullEcuOutputProfile) {
    assert!(!profile.is_valid());
    assert_eq!(profile.event_count(), 0);

    let mut runtime = EngineRuntime::new();
    runtime.configure_fuel_model(test_fuel_model());
    runtime.configure_full_ecu(profile);
    runtime.set_engine_time_authority(validated_expert_authority());
    runtime.apply_decoder_observation(DecoderObservation::Trigger(TriggerObservation {
        at_us: Micros::new(1_000),
        rpm: Rpm::new(3_000),
        angle_x10: Degrees10::new(120),
        synced: true,
    }));

    let result = runtime.step(
        running_step_inputs(5_000, 3_000, true, true),
        running_control_inputs(5_000, 3_000),
    );

    assert_eq!(arm_scheduler_count(result.actions), 0);
    assert_eq!(arm_injection_count(result.actions), 0);
    assert_eq!(arm_ignition_count(result.actions), 0);
    assert_eq!(runtime.scheduler.mode(), ecu_scheduler::SchedulerMode::Idle);
}
