use super::*;
use ecu_calibration::{
    ActiveCalibration, Calibration, CalibrationRevision, ExpertIgnitionMode, ExpertInjectionLayout,
    ExpertTriggerCalibration, ExpertUnlock, PrimaryTriggerSpeed, SecondaryTriggerMode,
    StagedCalibration, TriggerAuthority, TriggerEdge, TriggerFilter, TriggerPattern,
};
use ecu_domain::{Degrees10, Lambda100, Micros, Rpm};
use ecu_runtime::compat::StepInputs;
use ecu_runtime::{
    Action, ControlInputs, EngineRuntime, EnrichmentInputs, IgnitionInputs, LambdaTrimInputs,
    TorqueInputs,
};

#[test]
fn maps_runtime_snapshot_without_reading_internal_state() {
    let mut runtime = EngineRuntime::new();
    runtime.configure_fuel_model({
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
            ecu_domain::Kpa10::new(200),
            ecu_domain::Kpa10::new(300),
            ecu_domain::Kpa10::new(400),
            ecu_domain::Kpa10::new(500),
            ecu_domain::Kpa10::new(600),
            ecu_domain::Kpa10::new(700),
            ecu_domain::Kpa10::new(800),
            ecu_domain::Kpa10::new(900),
            ecu_domain::Kpa10::new(1000),
            ecu_domain::Kpa10::new(1100),
            ecu_domain::Kpa10::new(1200),
            ecu_domain::Kpa10::new(1300),
            ecu_domain::Kpa10::new(1400),
            ecu_domain::Kpa10::new(1500),
            ecu_domain::Kpa10::new(1600),
            ecu_domain::Kpa10::new(1700),
        ];
        let mut pulse_widths = [[ecu_domain::PulseWidthUs::new(0); 16]; 16];
        pulse_widths[5][5] = ecu_domain::PulseWidthUs::new(2500);
        ecu_runtime::BaseFuelModel::new(rpm_bins, load_bins, pulse_widths)
    });

    let step = runtime.step(
        StepInputs {
            now_us: Micros::new(1_000),
            rpm: 3_000,
            load_kpa10: 700,
            angle_x10: 2_000,
            trigger_synced: true,
            cam_seen: true,
            flat_shift_armed: false,
            launch_armed: false,
        },
        ControlInputs {
            enrichment: EnrichmentInputs {
                now_us: Micros::new(1_000),
                clt_c: 20,
                cranking: false,
                just_started: false,
                tpsdot_pct_s: 0,
                mapdot_kpa_s: 0,
            },
            lambda: LambdaTrimInputs {
                clt_c: 80,
                lambda_valid: true,
                measured_lambda100: Lambda100::new(100),
                requested_open_loop: false,
            },
            torque: TorqueInputs::new(90, 90, 90, 90, 90),
            ignition: IgnitionInputs::new(Degrees10::new(100), 0, 0, 0, false, Rpm::new(3000)),
        },
    );

    let adapter = RuntimeSnapshotAdapter::new();
    let snapshot = runtime.snapshot();
    let view = adapter.map(&snapshot);

    assert_eq!(view.sync_state, snapshot.engine.sync);
    assert_eq!(view.engine_phase, snapshot.engine.phase);
    assert_eq!(view.control_mode, snapshot.engine.mode);
    assert_eq!(view.rpm, snapshot.engine.rpm.get());
    assert_eq!(view.load_kpa10, snapshot.engine.load_kpa10.get());
    assert_eq!(view.angle_x10, snapshot.engine.angle_x10.get());
    assert_eq!(
        view.fuel_pulse_width_us,
        snapshot.control.fuel_pulse_width.get()
    );
    assert_eq!(
        view.ignition_advance_deg10,
        snapshot.control.ignition_advance.get()
    );
    assert_eq!(view.dwell_us, snapshot.control.dwell.get());
    assert_eq!(
        view.lambda_target_x100,
        snapshot.control.lambda_target.get()
    );
    assert_eq!(view.torque_limit_x100, snapshot.control.torque_limit_x100);
    assert_eq!(view.fault_code, snapshot.faults.fault);
    assert_eq!(view.fault_severity, snapshot.faults.severity);
    assert_eq!(view.cancel_reason, snapshot.faults.cancel_reason);
    assert!(matches!(
        step.actions.iter().next(),
        Some(Action::ArmScheduler { .. })
    ));
}

#[test]
fn runtime_view_fills_outpc_runtime_fields_only() {
    let view = TunerStudioRuntimeView {
        sync_state: SyncState::Locked { cam_ref: false },
        rpm: 3_200,
        load_kpa10: 850,
        fuel_pulse_width_us: 2_450,
        ignition_advance_deg10: 175,
        dwell_us: 3_100,
        ..TunerStudioRuntimeView::default()
    };
    let mut out = crate::outpc::Outpc {
        tps_percent: 42,
        clt_c: 80,
        iat_c: 35,
        vbatt_mv: 12_600,
        lambda_x100: 98,
        ..crate::outpc::Outpc::default()
    };

    view.fill_outpc_runtime_fields(&mut out);

    let rpm = out.rpm;
    let map_kpa_x10 = out.map_kpa_x10;
    let pw_us = out.pw_us;
    let dwell_us = out.dwell_us;
    let advance_x10 = out.advance_x10;
    let clt_c = out.clt_c;
    let iat_c = out.iat_c;
    let vbatt_mv = out.vbatt_mv;
    let lambda_x100 = out.lambda_x100;

    assert_eq!(rpm, 3_200);
    assert_eq!(map_kpa_x10, 850);
    assert_eq!(pw_us, 2_450);
    assert_eq!(dwell_us, 3_100);
    assert_eq!(advance_x10, 175);
    assert_eq!(out.synced, 1);
    assert_eq!(out.tps_percent, 42);
    assert_eq!(clt_c, 80);
    assert_eq!(iat_c, 35);
    assert_eq!(vbatt_mv, 12_600);
    assert_eq!(lambda_x100, 98);
}

#[test]
fn runtime_snapshot_adapter_fills_outpc_projection() {
    let mut runtime = EngineRuntime::new();
    let step = runtime.step(
        StepInputs {
            now_us: Micros::new(1_000),
            rpm: 2_750,
            load_kpa10: 700,
            angle_x10: 900,
            trigger_synced: true,
            cam_seen: true,
            flat_shift_armed: false,
            launch_armed: false,
        },
        ControlInputs {
            enrichment: EnrichmentInputs {
                now_us: Micros::new(1_000),
                clt_c: 75,
                cranking: false,
                just_started: false,
                tpsdot_pct_s: 0,
                mapdot_kpa_s: 0,
            },
            lambda: LambdaTrimInputs {
                clt_c: 75,
                lambda_valid: true,
                measured_lambda100: Lambda100::new(100),
                requested_open_loop: false,
            },
            torque: TorqueInputs::new(90, 90, 90, 90, 90),
            ignition: IgnitionInputs::new(Degrees10::new(120), 0, 0, 0, false, Rpm::new(2750)),
        },
    );
    let snapshot = runtime.snapshot();
    let adapter = RuntimeSnapshotAdapter::new();
    let mut out = crate::outpc::Outpc::default();

    adapter.fill_outpc(&snapshot, &mut out);

    let rpm = out.rpm;
    let map_kpa_x10 = out.map_kpa_x10;
    let pw_us = out.pw_us;
    let dwell_us = out.dwell_us;
    let advance_x10 = out.advance_x10;

    assert_eq!(rpm, snapshot.engine.rpm.get());
    assert_eq!(map_kpa_x10, snapshot.engine.load_kpa10.get());
    assert_eq!(pw_us, snapshot.control.fuel_pulse_width.get());
    assert_eq!(dwell_us, snapshot.control.dwell.get());
    assert_eq!(advance_x10, snapshot.control.ignition_advance.get());
    assert_eq!(out.synced, 1);
    assert!(matches!(
        step.actions.iter().next(),
        Some(Action::ArmScheduler { .. })
    ));
}

#[test]
fn calibration_edits_only_touch_staged_state() {
    let active = ActiveCalibration::new(CalibrationRevision::new(7), Calibration::default());
    let staged = StagedCalibration::new(CalibrationRevision::new(7), Calibration::default());
    let mut surface = CalibrationEditSurface::new(CalibrationSnapshot { active, staged });

    let diff = surface.apply_write(CalibrationWrite::RuntimeSensitive);

    assert_eq!(diff.class, CalibrationClass::RuntimeSensitive);
    assert_eq!(diff.base_revision.get(), 7);
    assert_eq!(diff.staged_revision.get(), 8);
    assert_eq!(surface.snapshot().active.revision().get(), 7);
    assert_eq!(surface.snapshot().staged.revision().get(), 8);
    assert!(surface.snapshot().staged.is_dirty());
}

#[test]
fn no_change_write_does_not_dirty_staged_state() {
    let active = ActiveCalibration::new(CalibrationRevision::new(3), Calibration::default());
    let staged = StagedCalibration::new(CalibrationRevision::new(3), Calibration::default());
    let mut surface = CalibrationEditSurface::new(CalibrationSnapshot { active, staged });

    let diff = surface.apply_write(CalibrationWrite::NoChange);

    assert_eq!(diff.class, CalibrationClass::NoChange);
    assert_eq!(surface.snapshot().active.revision().get(), 3);
    assert_eq!(surface.snapshot().staged.revision().get(), 3);
    assert!(!surface.snapshot().staged.is_dirty());
}

#[test]
fn commit_activates_staged_state_when_policy_allows_now() {
    let active = ActiveCalibration::new(CalibrationRevision::new(10), Calibration::default());
    let mut staged = StagedCalibration::new(CalibrationRevision::new(10), Calibration::default());
    staged.mark_dirty();
    let mut surface = CalibrationEditSurface::new(CalibrationSnapshot { active, staged });

    let pre_commit = surface.diff();
    let result = surface.commit(CommitPolicy::Immediate, true);

    assert_eq!(result, CalibrationCommandResult::Committed(pre_commit));
    assert_eq!(surface.snapshot().active.revision().get(), 11);
    assert_eq!(surface.snapshot().staged.base_revision().get(), 11);
    assert_eq!(surface.snapshot().staged.revision().get(), 11);
    assert!(!surface.snapshot().staged.is_dirty());
    assert_eq!(surface.diff().class, CalibrationClass::NoChange);
}

#[test]
fn commit_defers_when_runtime_is_not_safe() {
    let active = ActiveCalibration::new(CalibrationRevision::new(4), Calibration::default());
    let mut staged = StagedCalibration::new(CalibrationRevision::new(4), Calibration::default());
    staged.mark_dirty();
    let mut surface = CalibrationEditSurface::new(CalibrationSnapshot { active, staged });

    let pre_commit = surface.diff();
    let result = surface.commit(CommitPolicy::SafeOnly, false);

    assert_eq!(result, CalibrationCommandResult::Deferred(pre_commit));
    assert_eq!(surface.snapshot().active.revision().get(), 4);
    assert_eq!(surface.snapshot().staged.revision().get(), 5);
    assert!(surface.snapshot().staged.is_dirty());
}

#[test]
fn persist_captures_current_snapshot_metadata() {
    let active = ActiveCalibration::new(CalibrationRevision::new(8), Calibration::default());
    let staged = StagedCalibration::new(CalibrationRevision::new(8), Calibration::default());
    let surface = CalibrationEditSurface::new(CalibrationSnapshot { active, staged });

    let blob = surface.persist();

    assert_eq!(
        blob.schema_version(),
        ecu_calibration::CalibrationSchemaVersion::CURRENT
    );
    assert_eq!(blob.revision().get(), 8);
    assert_eq!(blob.snapshot(), surface.snapshot());
}

#[test]
fn reset_staged_rebases_edit_surface_to_active_snapshot() {
    let active = ActiveCalibration::new(CalibrationRevision::new(6), Calibration::default());
    let mut staged = StagedCalibration::new(CalibrationRevision::new(6), Calibration::default());
    staged.mark_dirty();
    let mut surface = CalibrationEditSurface::new(CalibrationSnapshot { active, staged });

    let result = surface.reset_staged();

    assert_eq!(result, CalibrationCommandResult::Reset(surface.diff()));
    assert_eq!(surface.snapshot().staged.base_revision().get(), 6);
    assert_eq!(surface.snapshot().staged.revision().get(), 6);
    assert!(!surface.snapshot().staged.is_dirty());
    assert_eq!(surface.diff().class, CalibrationClass::NoChange);
}

#[test]
fn runtime_view_isolated_from_staged_calibration_edits() {
    let runtime = EngineRuntime::new();
    let snapshot = runtime.snapshot();
    let adapter = RuntimeSnapshotAdapter::new();

    let before = adapter.map(&snapshot);

    let active = ActiveCalibration::new(CalibrationRevision::new(12), Calibration::default());
    let staged = StagedCalibration::new(CalibrationRevision::new(12), Calibration::default());
    let mut surface = CalibrationEditSurface::new(CalibrationSnapshot { active, staged });
    surface.apply_write(CalibrationWrite::RuntimeSensitive);
    let _ = surface.commit(CommitPolicy::Deferred, true);

    let after = adapter.map(&snapshot);

    assert_eq!(before, after);
    assert_eq!(after.sync_state, snapshot.engine.sync);
    assert_eq!(after.engine_phase, snapshot.engine.phase);
    assert_eq!(after.control_mode, snapshot.engine.mode);
}

#[test]
fn commit_and_persist_follow_snapshot_ownership_model() {
    let active = ActiveCalibration::new(CalibrationRevision::new(21), Calibration::default());
    let mut staged = StagedCalibration::new(CalibrationRevision::new(21), Calibration::default());
    staged.mark_dirty();
    let mut surface = CalibrationEditSurface::new(CalibrationSnapshot { active, staged });
    let staged_blob = surface.persist();

    assert_eq!(staged_blob.revision().get(), 21);
    assert_eq!(staged_blob.snapshot().active.revision().get(), 21);
    assert!(staged_blob.snapshot().staged.is_dirty());

    let result = surface.commit(CommitPolicy::Immediate, true);

    assert!(matches!(result, CalibrationCommandResult::Committed(_)));
    assert_eq!(surface.snapshot().active.revision().get(), 22);
    assert_eq!(surface.snapshot().staged.revision().get(), 22);
    assert!(!surface.snapshot().staged.is_dirty());

    let committed_blob = surface.persist();

    assert_eq!(committed_blob.revision().get(), 22);
    assert_eq!(committed_blob.snapshot(), surface.snapshot());
}

fn sample_expert_trigger() -> ExpertTriggerCalibration {
    ExpertTriggerCalibration {
        expert_unlock: ExpertUnlock::Unlocked,
        authority: TriggerAuthority::ExpertManual,
        profile_identity: 0x4D353054,
        profile_hash: 0xA11C_E550,
        trigger_pattern: TriggerPattern::MissingTooth,
        primary_base_teeth: 60,
        missing_teeth: 2,
        primary_trigger_speed: PrimaryTriggerSpeed::Crank,
        trigger_angle_atdc_deg10: 720,
        trigger_angle_multiplier: 1,
        primary_trigger_edge: TriggerEdge::Falling,
        secondary_trigger_edge: TriggerEdge::Rising,
        secondary_trigger_mode: SecondaryTriggerMode::SingleToothCam,
        trigger_filter: TriggerFilter::Aggressive,
        resync_every_cycle: true,
        skip_cycles: 3,
        ignition_mode: ExpertIgnitionMode::SequentialCop,
        injection_layout: ExpertInjectionLayout::Sequential,
        ..ExpertTriggerCalibration::default()
    }
}

#[test]
fn expert_trigger_ts_page_roundtrips_every_field() {
    let expert = sample_expert_trigger();
    let mut page = [0u8; EXPERT_TRIGGER_PAGE_LEN];

    assert_eq!(
        encode_expert_trigger_page(&expert, &mut page),
        Ok(EXPERT_TRIGGER_PAGE_LEN)
    );
    assert_eq!(decode_expert_trigger_page(&page), Ok(expert));
}

#[test]
fn expert_trigger_ts_page_rejects_invalid_manual_config() {
    let expert = sample_expert_trigger();
    let mut page = [0u8; EXPERT_TRIGGER_PAGE_LEN];
    encode_expert_trigger_page(&expert, &mut page).expect("encode valid expert page");
    page[2] = ExpertUnlock::Locked.code();

    assert!(matches!(
        decode_expert_trigger_page(&page),
        Err(ExpertTriggerPageError::Record(
            ecu_calibration::ExpertTriggerRecordError::InvalidCalibration(
                ecu_calibration::ExpertTriggerValidationError::ExpertUnlockRequired
            )
        ))
    ));
}

#[test]
fn expert_trigger_page_updates_only_staged_calibration() {
    let active = ActiveCalibration::new(CalibrationRevision::new(30), Calibration::default());
    let staged = StagedCalibration::new(CalibrationRevision::new(30), Calibration::default());
    let mut surface = CalibrationEditSurface::new(CalibrationSnapshot { active, staged });
    let expert = sample_expert_trigger();
    let mut page = [0u8; EXPERT_TRIGGER_PAGE_LEN];
    encode_expert_trigger_page(&expert, &mut page).expect("encode valid expert page");

    let diff = surface
        .apply_expert_trigger_page(&page)
        .expect("apply expert page");

    assert_eq!(diff.class, CalibrationClass::SafetyCritical);
    assert_eq!(surface.snapshot().active.revision().get(), 30);
    assert_eq!(surface.snapshot().staged.revision().get(), 31);
    assert_eq!(
        surface
            .snapshot()
            .staged
            .calibration()
            .geometry
            .expert_trigger,
        expert
    );
    assert_ne!(
        surface
            .snapshot()
            .active
            .calibration()
            .geometry
            .expert_trigger,
        expert
    );
}

#[test]
fn certified_profile_trigger_page_rejects_stale_manual_overwrite() {
    let certified = ExpertTriggerCalibration {
        authority: TriggerAuthority::CertifiedProfile,
        profile_identity: 0x4D353054,
        profile_hash: 0xCAFE_BABE,
        ..ExpertTriggerCalibration::default()
    };
    let mut active_calibration = Calibration::default();
    active_calibration.set_expert_trigger(certified);
    let active = ActiveCalibration::new(CalibrationRevision::new(40), active_calibration);
    let staged = StagedCalibration::new(CalibrationRevision::new(40), active_calibration);
    let mut surface = CalibrationEditSurface::new(CalibrationSnapshot { active, staged });
    let stale = ExpertTriggerCalibration {
        trigger_angle_atdc_deg10: 450,
        ..certified
    };
    let mut page = [0u8; EXPERT_TRIGGER_PAGE_LEN];
    encode_expert_trigger_page(&stale, &mut page).expect("encode stale certified page");

    assert_eq!(
        surface.apply_expert_trigger_page(&page),
        Err(ExpertTriggerPageError::Transition(
            ecu_calibration::ExpertTriggerValidationError::CertifiedProfileOverwrite
        ))
    );
    assert_eq!(surface.snapshot().staged.revision().get(), 40);
}

#[test]
fn certified_profile_page_is_rejected_from_non_certified_ts_surface() {
    let active = ActiveCalibration::new(CalibrationRevision::new(8), Calibration::default());
    let staged = StagedCalibration::new(CalibrationRevision::new(8), Calibration::default());
    let mut surface = CalibrationEditSurface::new(CalibrationSnapshot { active, staged });
    let certified = ExpertTriggerCalibration {
        expert_unlock: ecu_calibration::ExpertUnlock::Locked,
        authority: TriggerAuthority::CertifiedProfile,
        profile_identity: 0x4D353054,
        profile_hash: 0xCAFE_BABE,
        ..ExpertTriggerCalibration::default()
    };
    let mut page = [0u8; EXPERT_TRIGGER_PAGE_LEN];
    encode_expert_trigger_page(&certified, &mut page).expect("encode certified page");

    assert_eq!(
        surface.apply_expert_trigger_page(&page),
        Err(ExpertTriggerPageError::Transition(
            ecu_calibration::ExpertTriggerValidationError::CertifiedProfileRequiresTrustedPath
        ))
    );
    assert_eq!(surface.snapshot().staged.revision().get(), 8);
}
