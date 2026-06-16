use std::sync::{Arc, Barrier, Mutex, OnceLock};

use ecu_domain::{CancelReason, ControlMode, FaultCode, FaultSeverity};
use ecu_runtime::RuntimeSnapshot;

use crate::encoding::{
    encode_cancel_reason, encode_control_mode, encode_engine_phase, encode_fault_code,
    encode_fault_severity, snapshot_from_runtime,
};
use crate::state::{
    event_less, fabricate_wrong_nonce_handle_for_test, force_next_handle_generation_for_test,
    handle_registry_stats, max_handle_generation_for_test, with_handle, EcuSimHandle,
};
use crate::*;

static TEST_LOCK: OnceLock<Mutex<()>> = OnceLock::new();

fn lock_tests() -> std::sync::MutexGuard<'static, ()> {
    TEST_LOCK
        .get_or_init(|| Mutex::new(()))
        .lock()
        .unwrap_or_else(|poisoned| poisoned.into_inner())
}

fn cfg() -> EcuSimInitCfg {
    EcuSimInitCfg {
        cylinders: 4,
        has_cam: 1,
        inj_mode: EcuSimInjMode::Batch as i32,
        ign_mode: EcuSimIgnMode::Wasted as i32,
        firing_len: 4,
        firing_order: [1, 3, 4, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        inj_count: 1,
        inj_channels: [7, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        ign_count: 1,
        ign_channels: [9, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    }
}

fn warm_synced_runtime() {
    ecu_sim_reset();
    let cfg = cfg();
    assert_eq!(
        unsafe { ecu_sim_init((&cfg) as *const EcuSimInitCfg) },
        EcuSimStatus::Ok
    );
    let sensors = EcuSimSensorFrame {
        now_us: 1_000,
        map_kpa10: 900,
        tps_x100: 2_000,
        clt_c10: 850,
        iat_c10: 300,
        vbatt_mv: 13_800,
        baro_kpa10: 1_013,
        lambda_x100: 100,
        lambda_valid: 1,
        knock_x100: 0,
        vehicle_speed_kph10: 0,
        maf_x100: 0,
        cam_phase_deg10: 0,
        cam_phase_valid: 0,
        validity_flags: 0,
    };
    assert_eq!(
        unsafe { ecu_sim_set_sensors((&sensors) as *const EcuSimSensorFrame) },
        EcuSimStatus::Ok
    );
    assert_eq!(ecu_sim_on_crank_edge(1_000), EcuSimStatus::Ok);
    assert_eq!(ecu_sim_on_crank_edge(1_500), EcuSimStatus::Ok);
    assert_eq!(ecu_sim_on_cam_edge(1_500), EcuSimStatus::Ok);
}

#[test]
fn ffi_layout_stays_stable() {
    assert_eq!(core::mem::size_of::<EcuSimInitCfg>(), 64);
    assert_eq!(core::mem::offset_of!(EcuSimInitCfg, cylinders), 0);
    assert_eq!(core::mem::offset_of!(EcuSimInitCfg, has_cam), 1);
    assert_eq!(core::mem::offset_of!(EcuSimInitCfg, inj_mode), 4);
    assert_eq!(core::mem::offset_of!(EcuSimInitCfg, ign_mode), 8);
    assert_eq!(core::mem::offset_of!(EcuSimInitCfg, firing_len), 12);
    assert_eq!(core::mem::offset_of!(EcuSimInitCfg, firing_order), 13);
    assert_eq!(core::mem::offset_of!(EcuSimInitCfg, inj_count), 29);
    assert_eq!(core::mem::offset_of!(EcuSimInitCfg, inj_channels), 30);
    assert_eq!(core::mem::offset_of!(EcuSimInitCfg, ign_count), 46);
    assert_eq!(core::mem::offset_of!(EcuSimInitCfg, ign_channels), 47);
    assert_eq!(core::mem::size_of::<EcuSimSensorFrame>(), 32);
    assert_eq!(core::mem::offset_of!(EcuSimSensorFrame, now_us), 0);
    assert_eq!(core::mem::offset_of!(EcuSimSensorFrame, map_kpa10), 4);
    assert_eq!(core::mem::offset_of!(EcuSimSensorFrame, tps_x100), 6);
    assert_eq!(core::mem::offset_of!(EcuSimSensorFrame, clt_c10), 8);
    assert_eq!(core::mem::offset_of!(EcuSimSensorFrame, iat_c10), 10);
    assert_eq!(core::mem::offset_of!(EcuSimSensorFrame, vbatt_mv), 12);
    assert_eq!(core::mem::offset_of!(EcuSimSensorFrame, baro_kpa10), 14);
    assert_eq!(core::mem::offset_of!(EcuSimSensorFrame, lambda_x100), 16);
    assert_eq!(core::mem::offset_of!(EcuSimSensorFrame, lambda_valid), 18);
    assert_eq!(core::mem::offset_of!(EcuSimSensorFrame, knock_x100), 20);
    assert_eq!(
        core::mem::offset_of!(EcuSimSensorFrame, vehicle_speed_kph10),
        22
    );
    assert_eq!(core::mem::offset_of!(EcuSimSensorFrame, maf_x100), 24);
    assert_eq!(
        core::mem::offset_of!(EcuSimSensorFrame, cam_phase_deg10),
        26
    );
    assert_eq!(
        core::mem::offset_of!(EcuSimSensorFrame, cam_phase_valid),
        28
    );
    assert_eq!(core::mem::offset_of!(EcuSimSensorFrame, validity_flags), 29);
    assert_eq!(core::mem::size_of::<EcuSimOutputEvent>(), 16);
    assert_eq!(core::mem::offset_of!(EcuSimOutputEvent, time_us), 0);
    assert_eq!(core::mem::offset_of!(EcuSimOutputEvent, channel), 4);
    assert_eq!(core::mem::offset_of!(EcuSimOutputEvent, kind), 8);
    assert_eq!(core::mem::offset_of!(EcuSimOutputEvent, high), 12);
    assert_eq!(core::mem::size_of::<EcuSimSnapshot>(), 40);
    assert_eq!(core::mem::offset_of!(EcuSimSnapshot, now_us), 0);
    assert_eq!(core::mem::offset_of!(EcuSimSnapshot, rpm), 4);
    assert_eq!(core::mem::offset_of!(EcuSimSnapshot, synced), 6);
    assert_eq!(core::mem::offset_of!(EcuSimSnapshot, tooth), 7);
    assert_eq!(core::mem::offset_of!(EcuSimSnapshot, angle_x10), 8);
    assert_eq!(core::mem::offset_of!(EcuSimSnapshot, load_kpa10), 10);
    assert_eq!(
        core::mem::offset_of!(EcuSimSnapshot, output_overflow_count),
        12
    );
    assert_eq!(core::mem::offset_of!(EcuSimSnapshot, engine_phase), 16);
    assert_eq!(core::mem::offset_of!(EcuSimSnapshot, fault_code), 17);
    assert_eq!(core::mem::offset_of!(EcuSimSnapshot, fault_severity), 18);
    assert_eq!(core::mem::offset_of!(EcuSimSnapshot, cancel_reason), 19);
    assert_eq!(core::mem::offset_of!(EcuSimSnapshot, control_mode), 20);
    assert_eq!(
        core::mem::offset_of!(EcuSimSnapshot, fuel_pulse_width_us),
        22
    );
    assert_eq!(
        core::mem::offset_of!(EcuSimSnapshot, ignition_advance_x10),
        24
    );
    assert_eq!(core::mem::offset_of!(EcuSimSnapshot, dwell_us), 26);
    assert_eq!(
        core::mem::offset_of!(EcuSimSnapshot, lambda_target_x100),
        28
    );
    assert_eq!(core::mem::offset_of!(EcuSimSnapshot, torque_limit_x100), 30);
    assert_eq!(core::mem::offset_of!(EcuSimSnapshot, rev_soft_active), 32);
    assert_eq!(core::mem::offset_of!(EcuSimSnapshot, rev_hard_active), 33);
    assert_eq!(core::mem::offset_of!(EcuSimSnapshot, launch_active), 34);
    assert_eq!(core::mem::offset_of!(EcuSimSnapshot, flat_shift_active), 35);
    assert_eq!(core::mem::offset_of!(EcuSimSnapshot, fuel_cut), 36);
    assert_eq!(core::mem::offset_of!(EcuSimSnapshot, spark_cut), 37);
    assert_eq!(core::mem::align_of::<EcuSimInitCfg>(), 4);
    assert_eq!(core::mem::align_of::<EcuSimOutputEvent>(), 4);
    assert_eq!(core::mem::align_of::<EcuSimSnapshot>(), 4);
}

#[test]
fn default_snapshot_is_diagnostically_clean() {
    let _guard = lock_tests();
    warm_synced_runtime();
    ecu_sim_step(2_000);
    let mut snap = EcuSimSnapshot::default();
    assert_eq!(unsafe { ecu_sim_snapshot(&mut snap) }, EcuSimStatus::Ok);
    assert_eq!(snap.fault_code, 0, "default fault_code must be 0");
    assert_eq!(snap.fault_severity, 0, "default fault_severity must be 0");
    assert_eq!(snap.cancel_reason, 0, "default cancel_reason must be 0");
    assert_eq!(snap.control_mode, 1, "default control_mode must be 1");
}

#[test]
fn fault_code_encoding_is_explicit_and_stable() {
    assert_eq!(encode_fault_code(FaultCode::None), 0);
    assert_eq!(encode_fault_code(FaultCode::SyncLoss), 1);
    assert_eq!(encode_fault_code(FaultCode::SensorOutOfRange), 2);
    assert_eq!(encode_fault_code(FaultCode::CalibrationInvalid), 3);
    assert_eq!(encode_fault_code(FaultCode::SafetyCut), 4);
    assert_eq!(encode_fault_code(FaultCode::ActuatorFault), 5);
}

#[test]
fn fault_severity_encoding_is_explicit_and_stable() {
    assert_eq!(encode_fault_severity(FaultSeverity::Info), 0);
    assert_eq!(encode_fault_severity(FaultSeverity::Warning), 1);
    assert_eq!(encode_fault_severity(FaultSeverity::Critical), 2);
}

#[test]
fn cancel_reason_encoding_is_explicit_and_stable() {
    assert_eq!(encode_cancel_reason(CancelReason::Manual), 0);
    assert_eq!(encode_cancel_reason(CancelReason::SyncLoss), 1);
    assert_eq!(encode_cancel_reason(CancelReason::SafetyShutdown), 2);
    assert_eq!(encode_cancel_reason(CancelReason::Commit), 3);
    assert_eq!(encode_cancel_reason(CancelReason::Timeout), 4);
}

#[test]
fn control_mode_encoding_is_explicit_and_stable() {
    assert_eq!(encode_control_mode(ControlMode::OpenLoop), 0);
    assert_eq!(encode_control_mode(ControlMode::ClosedLoop), 1);
    assert_eq!(encode_control_mode(ControlMode::LimpHome), 2);
    assert_eq!(encode_control_mode(ControlMode::Shutdown), 3);
}

#[test]
fn engine_phase_encoding_is_explicit_and_stable() {
    assert_eq!(encode_engine_phase(ecu_domain::EnginePhase::Off), 0);
    assert_eq!(encode_engine_phase(ecu_domain::EnginePhase::Cranking), 1);
    assert_eq!(encode_engine_phase(ecu_domain::EnginePhase::Running), 2);
    assert_eq!(encode_engine_phase(ecu_domain::EnginePhase::Stopping), 3);
}

#[test]
fn snapshot_conversion_covers_non_default_diagnostics() {
    let runtime_snapshot = RuntimeSnapshot {
        engine: ecu_runtime::EngineState {
            sync: ecu_domain::SyncState::Locked { cam_ref: false },
            phase: ecu_domain::EnginePhase::Stopping,
            mode: ecu_domain::ControlMode::LimpHome,
            rpm: ecu_domain::Rpm::new(900),
            load_kpa10: ecu_domain::Kpa10::new(950),
            angle_x10: ecu_domain::Degrees10::new(123),
            engine_time_authority: ecu_domain::EngineTimeAuthority::none(),
        },
        control: ecu_runtime::ControlState {
            fuel_pulse_width: ecu_domain::PulseWidthUs::new(2_250),
            ignition_advance: ecu_domain::Degrees10::new(87),
            dwell: ecu_domain::DwellUs::new(2_500),
            lambda_target: ecu_domain::Lambda100::new(142),
            torque_limit_x100: 77,
        },
        faults: ecu_runtime::FaultState {
            fault: ecu_domain::FaultCode::SensorOutOfRange,
            severity: ecu_domain::FaultSeverity::Warning,
            cancel_reason: ecu_domain::CancelReason::Manual,
        },
        rev_soft_active: true,
        rev_hard_active: false,
        launch_active: true,
        flat_shift_active: false,
        safety_latched: false,
        fuel_cut: true,
        spark_cut: false,
        legacy_cut_reason_code: 3,
        knock_intensity_x100: 0,
        knock_retard_deg10: 0,
    };
    let snap = snapshot_from_runtime(runtime_snapshot, 42_000, 17, 3);
    assert_eq!(snap.fault_code, 2);
    assert_eq!(snap.fault_severity, 1);
    assert_eq!(snap.cancel_reason, 0);
    assert_eq!(snap.control_mode, 2);
    assert_eq!(snap.engine_phase, 3);
    assert_eq!(snap.now_us, 42_000);
    assert_eq!(snap.tooth, 17);
    assert_eq!(snap.load_kpa10, 950);
    assert_eq!(snap.output_overflow_count, 3);
    assert_eq!(snap.fuel_pulse_width_us, 2_250);
    assert_eq!(snap.ignition_advance_x10, 87);
    assert_eq!(snap.dwell_us, 2_500);
    assert_eq!(snap.lambda_target_x100, 142);
    assert_eq!(snap.torque_limit_x100, 77);
    assert_eq!(snap.rev_soft_active, 1);
    assert_eq!(snap.rev_hard_active, 0);
    assert_eq!(snap.launch_active, 1);
    assert_eq!(snap.flat_shift_active, 0);
    assert_eq!(snap.fuel_cut, 1);
    assert_eq!(snap.spark_cut, 0);
}

#[test]
fn init_rejects_null_and_invalid_config() {
    let _guard = lock_tests();
    ecu_sim_reset();
    assert_eq!(
        unsafe { ecu_sim_init(core::ptr::null()) },
        EcuSimStatus::ErrInvalid
    );

    let mut invalid = cfg();
    invalid.inj_count = 0;
    assert_eq!(
        unsafe { ecu_sim_init((&invalid) as *const EcuSimInitCfg) },
        EcuSimStatus::ErrInvalid
    );
}

#[test]
fn init_rejects_structurally_invalid_config() {
    let _guard = lock_tests();
    ecu_sim_reset();

    let mut invalid = cfg();
    invalid.has_cam = 2;
    assert_eq!(
        unsafe { ecu_sim_init((&invalid) as *const EcuSimInitCfg) },
        EcuSimStatus::ErrInvalid
    );

    invalid = cfg();
    invalid.firing_len = 0;
    assert_eq!(
        unsafe { ecu_sim_init((&invalid) as *const EcuSimInitCfg) },
        EcuSimStatus::ErrInvalid
    );

    invalid = cfg();
    invalid.firing_len = 3;
    assert_eq!(
        unsafe { ecu_sim_init((&invalid) as *const EcuSimInitCfg) },
        EcuSimStatus::ErrInvalid
    );

    invalid = cfg();
    invalid.firing_order[1] = 1;
    assert_eq!(
        unsafe { ecu_sim_init((&invalid) as *const EcuSimInitCfg) },
        EcuSimStatus::ErrInvalid
    );

    invalid = cfg();
    invalid.inj_count = 2;
    invalid.inj_channels[1] = 0;
    assert_eq!(
        unsafe { ecu_sim_init((&invalid) as *const EcuSimInitCfg) },
        EcuSimStatus::ErrInvalid
    );

    invalid = cfg();
    invalid.ign_count = 2;
    invalid.ign_channels[1] = invalid.ign_channels[0];
    assert_eq!(
        unsafe { ecu_sim_init((&invalid) as *const EcuSimInitCfg) },
        EcuSimStatus::ErrInvalid
    );
}

#[test]
fn snapshot_before_init_returns_not_init() {
    let _guard = lock_tests();
    ecu_sim_reset();
    let mut snapshot = EcuSimSnapshot::default();
    assert_eq!(
        unsafe { ecu_sim_snapshot((&mut snapshot) as *mut EcuSimSnapshot) },
        EcuSimStatus::ErrNotInit
    );
}

#[test]
fn snapshot_null_pointer_returns_invalid() {
    let _guard = lock_tests();
    ecu_sim_reset();
    assert_eq!(
        unsafe { ecu_sim_snapshot(core::ptr::null_mut()) },
        EcuSimStatus::ErrInvalid
    );

    let handle = ecu_sim_handle_create();
    assert!(!handle.is_null());
    assert_eq!(
        unsafe { ecu_sim_handle_snapshot(handle, core::ptr::null_mut()) },
        EcuSimStatus::ErrInvalid
    );
    unsafe { ecu_sim_handle_destroy(handle) };
}

#[test]
fn step_exports_timestamp_sorted_channel_mapped_events() {
    let _guard = lock_tests();
    warm_synced_runtime();

    assert_eq!(ecu_sim_step(2_000), EcuSimStatus::Ok);
    let mut events = [EcuSimOutputEvent::ZERO; 8];
    let copied = unsafe { ecu_sim_dequeue_events(events.as_mut_ptr(), events.len()) };

    let live = &events[..copied];
    assert!(copied >= 4, "copied {copied}: {live:?}");
    assert!(live.windows(2).all(|pair| !event_less(pair[1], pair[0])));
    assert!(live
        .iter()
        .any(|event| { event.kind == EcuSimOutputKind::Injector as i32 && event.channel == 7 }));
    assert!(live
        .iter()
        .any(|event| { event.kind == EcuSimOutputKind::Ignition as i32 && event.channel == 9 }));
}

#[test]
fn snapshot_reports_runtime_progress() {
    let _guard = lock_tests();
    warm_synced_runtime();

    assert_eq!(ecu_sim_step(2_000), EcuSimStatus::Ok);
    let mut snapshot = EcuSimSnapshot::default();
    assert_eq!(
        unsafe { ecu_sim_snapshot((&mut snapshot) as *mut EcuSimSnapshot) },
        EcuSimStatus::Ok
    );

    assert_eq!(snapshot.now_us, 2_000);
    assert_eq!(snapshot.rpm, 2_000);
    assert_eq!(snapshot.synced, 1);
    assert_eq!(snapshot.angle_x10, 120);
}

#[test]
fn missing_crank_timeout_reports_sync_loss_and_allows_resync() {
    let _guard = lock_tests();
    warm_synced_runtime();

    assert_eq!(ecu_sim_step(7_000), EcuSimStatus::Ok);
    let mut lost = EcuSimSnapshot::default();
    assert_eq!(
        unsafe { ecu_sim_snapshot((&mut lost) as *mut EcuSimSnapshot) },
        EcuSimStatus::Ok
    );
    assert_eq!(lost.synced, 0, "missing crank timeout must clear sync");

    assert_eq!(ecu_sim_on_crank_edge(7_500), EcuSimStatus::Ok);
    assert_eq!(ecu_sim_on_crank_edge(8_000), EcuSimStatus::Ok);
    assert_eq!(ecu_sim_on_cam_edge(8_000), EcuSimStatus::Ok);
    assert_eq!(ecu_sim_step(8_500), EcuSimStatus::Ok);

    let mut recovered = EcuSimSnapshot::default();
    assert_eq!(
        unsafe { ecu_sim_snapshot((&mut recovered) as *mut EcuSimSnapshot) },
        EcuSimStatus::Ok
    );
    assert_eq!(recovered.synced, 1, "fresh crank/cam edges must resync");
}

#[test]
fn injected_fault_survives_ffi_snapshot() {
    let _guard = lock_tests();
    warm_synced_runtime();

    assert_eq!(ecu_sim_step(2_000), EcuSimStatus::Ok);
    assert_eq!(
        ecu_sim_inject_fault_for_test(
            FaultCode::SensorOutOfRange,
            FaultSeverity::Warning,
            CancelReason::SyncLoss,
        ),
        EcuSimStatus::Ok
    );
    let mut snapshot = EcuSimSnapshot::default();
    assert_eq!(
        unsafe { ecu_sim_snapshot((&mut snapshot) as *mut EcuSimSnapshot) },
        EcuSimStatus::Ok
    );

    assert_eq!(snapshot.fault_code, 2);
    assert_eq!(snapshot.fault_severity, 1);
    assert_eq!(snapshot.cancel_reason, 1);
    assert_eq!(snapshot.synced, 1);
    assert_eq!(snapshot.tooth, 2);
}

#[test]
fn dequeue_null_pointer_returns_zero() {
    let _guard = lock_tests();
    warm_synced_runtime();
    assert_eq!(ecu_sim_step(2_000), EcuSimStatus::Ok);
    assert_eq!(
        unsafe { ecu_sim_dequeue_events(core::ptr::null_mut(), 4) },
        0
    );
}

#[test]
fn dequeue_status_reports_invalid_and_not_init() {
    let _guard = lock_tests();
    ecu_sim_reset();

    let mut copied = usize::MAX;
    let mut events = [EcuSimOutputEvent::ZERO; 4];
    assert_eq!(
        unsafe { ecu_sim_dequeue_events_status(events.as_mut_ptr(), events.len(), &mut copied) },
        EcuSimStatus::ErrNotInit
    );
    assert_eq!(copied, 0);

    warm_synced_runtime();
    assert_eq!(ecu_sim_step(2_000), EcuSimStatus::Ok);
    copied = usize::MAX;
    assert_eq!(
        unsafe { ecu_sim_dequeue_events_status(core::ptr::null_mut(), 4, &mut copied) },
        EcuSimStatus::ErrInvalid
    );
    assert_eq!(copied, 0);

    assert_eq!(
        unsafe {
            ecu_sim_dequeue_events_status(events.as_mut_ptr(), events.len(), core::ptr::null_mut())
        },
        EcuSimStatus::ErrInvalid
    );
}

#[test]
fn handle_dequeue_status_reports_invalid_and_not_init() {
    let _guard = lock_tests();

    let handle = ecu_sim_handle_create();
    assert!(!handle.is_null());
    let mut copied = usize::MAX;
    let mut events = [EcuSimOutputEvent::ZERO; 4];
    assert_eq!(
        unsafe {
            ecu_sim_handle_dequeue_events_status(
                handle,
                events.as_mut_ptr(),
                events.len(),
                &mut copied,
            )
        },
        EcuSimStatus::ErrNotInit
    );
    assert_eq!(copied, 0);

    assert_eq!(
        unsafe {
            ecu_sim_handle_dequeue_events_status(
                core::ptr::null_mut(),
                events.as_mut_ptr(),
                events.len(),
                &mut copied,
            )
        },
        EcuSimStatus::ErrInvalid
    );

    unsafe { ecu_sim_handle_destroy(handle) };
}

#[test]
fn null_handle_operations_return_expected_statuses() {
    let _guard = lock_tests();
    ecu_sim_reset();

    let mut snapshot = EcuSimSnapshot::default();
    let mut events = [EcuSimOutputEvent::ZERO; 4];
    let cfg = cfg();
    let sensors = EcuSimSensorFrame::default();

    assert_eq!(
        ecu_sim_handle_reset(core::ptr::null_mut()),
        EcuSimStatus::ErrInvalid
    );
    assert_eq!(
        unsafe { ecu_sim_handle_init(core::ptr::null_mut(), (&cfg) as *const EcuSimInitCfg) },
        EcuSimStatus::ErrInvalid
    );
    assert_eq!(
        unsafe {
            ecu_sim_handle_set_sensors(
                core::ptr::null_mut(),
                (&sensors) as *const EcuSimSensorFrame,
            )
        },
        EcuSimStatus::ErrInvalid
    );
    assert_eq!(
        ecu_sim_handle_set_time(core::ptr::null_mut(), 123),
        EcuSimStatus::ErrInvalid
    );
    assert_eq!(
        ecu_sim_handle_on_crank_edge(core::ptr::null_mut(), 123),
        EcuSimStatus::ErrInvalid
    );
    assert_eq!(
        ecu_sim_handle_on_cam_edge(core::ptr::null_mut(), 123),
        EcuSimStatus::ErrInvalid
    );
    assert_eq!(
        ecu_sim_handle_step(core::ptr::null_mut(), 123),
        EcuSimStatus::ErrInvalid
    );
    assert_eq!(
        unsafe {
            ecu_sim_handle_snapshot(
                core::ptr::null_mut(),
                (&mut snapshot) as *mut EcuSimSnapshot,
            )
        },
        EcuSimStatus::ErrInvalid
    );
    assert_eq!(
        unsafe {
            ecu_sim_handle_dequeue_events(core::ptr::null_mut(), events.as_mut_ptr(), events.len())
        },
        0
    );
    assert_eq!(
        unsafe { ecu_sim_handle_dequeue_events(core::ptr::null_mut(), core::ptr::null_mut(), 0) },
        0
    );

    unsafe { ecu_sim_handle_destroy(core::ptr::null_mut()) };
}

#[test]
fn fabricated_low_numeric_handle_tokens_are_rejected() {
    let _guard = lock_tests();
    ecu_sim_reset();

    let cfg = cfg();
    let sensors = EcuSimSensorFrame::default();
    let fabricated_handles = [1usize, 2, 7, 0x10, 0x1234, 0xdead_beef];

    for raw_handle in fabricated_handles {
        let handle = raw_handle as *mut super::EcuSimHandleOpaque;
        let mut snapshot = EcuSimSnapshot::default();
        let mut events = [EcuSimOutputEvent::ZERO; 4];

        assert_eq!(
            ecu_sim_handle_reset(handle),
            EcuSimStatus::ErrInvalid,
            "raw handle {raw_handle:#x}"
        );
        assert_eq!(
            unsafe { ecu_sim_handle_init(handle, (&cfg) as *const EcuSimInitCfg) },
            EcuSimStatus::ErrInvalid,
            "raw handle {raw_handle:#x}"
        );
        assert_eq!(
            unsafe { ecu_sim_handle_set_sensors(handle, (&sensors) as *const EcuSimSensorFrame) },
            EcuSimStatus::ErrInvalid,
            "raw handle {raw_handle:#x}"
        );
        assert_eq!(
            ecu_sim_handle_set_time(handle, 123),
            EcuSimStatus::ErrInvalid,
            "raw handle {raw_handle:#x}"
        );
        assert_eq!(
            ecu_sim_handle_on_crank_edge(handle, 123),
            EcuSimStatus::ErrInvalid,
            "raw handle {raw_handle:#x}"
        );
        assert_eq!(
            ecu_sim_handle_on_cam_edge(handle, 123),
            EcuSimStatus::ErrInvalid,
            "raw handle {raw_handle:#x}"
        );
        assert_eq!(
            ecu_sim_handle_step(handle, 123),
            EcuSimStatus::ErrInvalid,
            "raw handle {raw_handle:#x}"
        );
        assert_eq!(
            unsafe { ecu_sim_handle_snapshot(handle, (&mut snapshot) as *mut EcuSimSnapshot) },
            EcuSimStatus::ErrInvalid,
            "raw handle {raw_handle:#x}"
        );
        assert_eq!(
            unsafe { ecu_sim_handle_dequeue_events(handle, events.as_mut_ptr(), events.len()) },
            0,
            "raw handle {raw_handle:#x}"
        );

        unsafe { ecu_sim_handle_destroy(handle) };
    }
}

#[test]
fn fabricated_structured_handle_tokens_are_rejected() {
    let _guard = lock_tests();

    let handle = ecu_sim_handle_create();
    assert!(!handle.is_null(), "handle should be created");
    let forged = fabricate_wrong_nonce_handle_for_test(handle)
        .expect("test should be able to perturb a live handle nonce");
    assert_ne!(forged, handle);

    assert_eq!(ecu_sim_handle_reset(forged), EcuSimStatus::ErrInvalid);
    assert_eq!(
        ecu_sim_handle_set_time(forged, 123),
        EcuSimStatus::ErrInvalid
    );

    unsafe {
        ecu_sim_handle_destroy(forged);
    }
    assert_eq!(ecu_sim_handle_reset(handle), EcuSimStatus::Ok);

    unsafe {
        ecu_sim_handle_destroy(handle);
    }
}

#[test]
fn destroyed_handles_reject_post_destroy_calls_and_double_destroy_is_safe() {
    let _guard = lock_tests();
    let handle = ecu_sim_handle_create();
    assert!(!handle.is_null(), "handle should be created");

    let cfg = cfg();
    let sensors = EcuSimSensorFrame {
        now_us: 1_000,
        ..Default::default()
    };

    assert_eq!(
        unsafe { ecu_sim_handle_init(handle, (&cfg) as *const EcuSimInitCfg) },
        EcuSimStatus::Ok
    );
    assert_eq!(ecu_sim_handle_set_time(handle, 1_000), EcuSimStatus::Ok);
    assert_eq!(
        unsafe { ecu_sim_handle_set_sensors(handle, (&sensors) as *const EcuSimSensorFrame) },
        EcuSimStatus::Ok
    );

    unsafe {
        ecu_sim_handle_destroy(handle);
        ecu_sim_handle_destroy(handle);
    }

    let mut snapshot = EcuSimSnapshot::default();
    let mut events = [EcuSimOutputEvent::ZERO; 4];
    assert_eq!(ecu_sim_handle_reset(handle), EcuSimStatus::ErrInvalid);
    assert_eq!(
        unsafe { ecu_sim_handle_init(handle, (&cfg) as *const EcuSimInitCfg) },
        EcuSimStatus::ErrInvalid
    );
    assert_eq!(
        ecu_sim_handle_set_time(handle, 2_000),
        EcuSimStatus::ErrInvalid
    );
    assert_eq!(
        unsafe { ecu_sim_handle_set_sensors(handle, (&sensors) as *const EcuSimSensorFrame) },
        EcuSimStatus::ErrInvalid
    );
    assert_eq!(
        ecu_sim_handle_on_crank_edge(handle, 2_000),
        EcuSimStatus::ErrInvalid
    );
    assert_eq!(
        ecu_sim_handle_on_cam_edge(handle, 2_000),
        EcuSimStatus::ErrInvalid
    );
    assert_eq!(ecu_sim_handle_step(handle, 2_000), EcuSimStatus::ErrInvalid);
    assert_eq!(
        unsafe { ecu_sim_handle_snapshot(handle, (&mut snapshot) as *mut EcuSimSnapshot) },
        EcuSimStatus::ErrInvalid
    );
    assert_eq!(
        unsafe { ecu_sim_handle_dequeue_events(handle, events.as_mut_ptr(), events.len()) },
        0
    );
}

#[test]
fn handle_churn_reclaims_registry_slots_and_keeps_stale_tokens_invalid() {
    let _guard = lock_tests();
    let cfg = cfg();
    let stale = ecu_sim_handle_create();
    assert!(!stale.is_null(), "stale handle should be created");
    assert_eq!(
        unsafe { ecu_sim_handle_init(stale, (&cfg) as *const EcuSimInitCfg) },
        EcuSimStatus::Ok
    );
    unsafe {
        ecu_sim_handle_destroy(stale);
    }
    assert_eq!(ecu_sim_handle_reset(stale), EcuSimStatus::ErrInvalid);

    let (_, slots_before, free_before, _) = handle_registry_stats();
    let mut snapshot = EcuSimSnapshot::default();

    for idx in 0..2_048u32 {
        let handle = ecu_sim_handle_create();
        assert!(!handle.is_null(), "handle should be created");
        assert_ne!(handle, stale, "token reuse at iteration {idx}");
        assert_eq!(
            unsafe { ecu_sim_handle_init(handle, (&cfg) as *const EcuSimInitCfg) },
            EcuSimStatus::Ok
        );
        assert_eq!(ecu_sim_handle_set_time(handle, idx + 1), EcuSimStatus::Ok);
        unsafe {
            ecu_sim_handle_destroy(handle);
        }
        assert_eq!(ecu_sim_handle_reset(handle), EcuSimStatus::ErrInvalid);
        if idx % 256 == 0 {
            assert_eq!(
                unsafe { ecu_sim_handle_snapshot(stale, (&mut snapshot) as *mut EcuSimSnapshot) },
                EcuSimStatus::ErrInvalid
            );
        }
    }

    let (live_after, slots_after, free_after, max_generation_after) = handle_registry_stats();
    assert_eq!(live_after, 0);
    assert_eq!(
        slots_after, slots_before,
        "ordinary churn should reuse existing free slots"
    );
    assert_eq!(
        free_after, free_before,
        "ordinary churn should return the reused slot to the free list"
    );
    assert!(
        max_generation_after > 1,
        "slot generation must advance during churn"
    );
    assert_eq!(ecu_sim_handle_reset(stale), EcuSimStatus::ErrInvalid);
}

#[test]
fn exhausted_generation_slots_retire_instead_of_wrapping() {
    let _guard = lock_tests();

    let seed = ecu_sim_handle_create();
    assert!(!seed.is_null(), "seed handle should be created");
    unsafe {
        ecu_sim_handle_destroy(seed);
    }
    assert!(
        force_next_handle_generation_for_test(max_handle_generation_for_test()),
        "test should be able to force the next free slot to max generation"
    );

    let exhausted = ecu_sim_handle_create();
    assert!(
        !exhausted.is_null(),
        "max-generation handle should be created"
    );
    let (_, _, free_before_retire, _) = handle_registry_stats();
    unsafe {
        ecu_sim_handle_destroy(exhausted);
    }

    let (_, slots_after_exhaust, free_after_exhaust, _) = handle_registry_stats();
    assert_eq!(
        free_after_exhaust, free_before_retire,
        "destroying a max-generation handle must retire, not recycle, its slot"
    );

    let replacement = ecu_sim_handle_create();
    assert!(
        !replacement.is_null(),
        "replacement handle should be created"
    );
    assert_ne!(
        replacement, exhausted,
        "replacement must not reuse a token after generation exhaustion"
    );

    let (_, slots_after_replacement, _, _) = handle_registry_stats();
    assert!(
        slots_after_replacement >= slots_after_exhaust,
        "replacement must not shrink the registry after exhaustion retires a slot"
    );

    unsafe {
        ecu_sim_handle_destroy(replacement);
    }
}

#[test]
fn destroy_waits_for_active_handle_calls_to_finish() {
    let _guard = lock_tests();
    let handle = ecu_sim_handle_create();
    assert!(!handle.is_null(), "handle should be created");

    let cfg = cfg();
    assert_eq!(
        unsafe { ecu_sim_handle_init(handle, (&cfg) as *const EcuSimInitCfg) },
        EcuSimStatus::Ok
    );

    let entered = Arc::new(Barrier::new(2));
    let release = Arc::new(Barrier::new(2));
    let handle_addr = handle as usize;
    let worker_entered = Arc::clone(&entered);
    let worker_release = Arc::clone(&release);
    let worker = std::thread::spawn(move || {
        let handle = handle_addr as *mut super::EcuSimHandleOpaque;
        let result = with_handle(handle, |state| {
            worker_entered.wait();
            worker_release.wait();
            state.set_time(42_000)
        });
        assert_eq!(result, Ok(EcuSimStatus::Ok));
    });

    entered.wait();

    let (started_tx, started_rx) = std::sync::mpsc::channel();
    let (done_tx, done_rx) = std::sync::mpsc::channel();
    let destroy_handle_addr = handle as usize;
    let destroyer = std::thread::spawn(move || {
        started_tx.send(()).expect("destroy start signal");
        let handle = destroy_handle_addr as *mut super::EcuSimHandleOpaque;
        unsafe { ecu_sim_handle_destroy(handle) };
        done_tx.send(()).expect("destroy completion signal");
    });

    started_rx.recv().expect("destroy thread should start");
    assert!(
        done_rx.try_recv().is_err(),
        "destroy must wait for the active call to finish"
    );

    release.wait();
    done_rx
        .recv()
        .expect("destroy should complete after release");

    worker.join().expect("worker thread");
    destroyer.join().expect("destroy thread");
    assert_eq!(ecu_sim_handle_reset(handle), EcuSimStatus::ErrInvalid);
}

#[test]
fn handle_instances_run_independently() {
    let _guard = lock_tests();
    let handle_a = ecu_sim_handle_create();
    let handle_b = ecu_sim_handle_create();
    assert!(!handle_a.is_null(), "first handle should be created");
    assert!(!handle_b.is_null(), "second handle should be created");

    let cfg = cfg();
    let sensors_a = EcuSimSensorFrame {
        now_us: 1_000,
        map_kpa10: 900,
        tps_x100: 2_000,
        clt_c10: 850,
        iat_c10: 300,
        vbatt_mv: 13_800,
        baro_kpa10: 1_013,
        lambda_x100: 100,
        lambda_valid: 1,
        knock_x100: 0,
        vehicle_speed_kph10: 0,
        maf_x100: 0,
        cam_phase_deg10: 0,
        cam_phase_valid: 0,
        validity_flags: 0,
    };
    let sensors_b = EcuSimSensorFrame {
        now_us: 2_000,
        map_kpa10: 650,
        tps_x100: 500,
        clt_c10: 720,
        iat_c10: 280,
        vbatt_mv: 12_400,
        baro_kpa10: 990,
        lambda_x100: 105,
        lambda_valid: 1,
        knock_x100: 0,
        vehicle_speed_kph10: 0,
        maf_x100: 0,
        cam_phase_deg10: 0,
        cam_phase_valid: 0,
        validity_flags: 0,
    };

    assert_eq!(
        unsafe { ecu_sim_handle_init(handle_a, (&cfg) as *const EcuSimInitCfg) },
        EcuSimStatus::Ok
    );
    assert_eq!(
        unsafe { ecu_sim_handle_init(handle_b, (&cfg) as *const EcuSimInitCfg) },
        EcuSimStatus::Ok
    );

    assert_eq!(
        unsafe { ecu_sim_handle_set_sensors(handle_a, (&sensors_a) as *const EcuSimSensorFrame) },
        EcuSimStatus::Ok
    );
    assert_eq!(ecu_sim_handle_set_time(handle_a, 1_000), EcuSimStatus::Ok);
    assert_eq!(
        ecu_sim_handle_on_crank_edge(handle_a, 1_000),
        EcuSimStatus::Ok
    );
    assert_eq!(
        ecu_sim_handle_on_crank_edge(handle_a, 1_500),
        EcuSimStatus::Ok
    );
    assert_eq!(
        ecu_sim_handle_on_cam_edge(handle_a, 1_500),
        EcuSimStatus::Ok
    );
    assert_eq!(ecu_sim_handle_step(handle_a, 2_000), EcuSimStatus::Ok);

    assert_eq!(
        unsafe { ecu_sim_handle_set_sensors(handle_b, (&sensors_b) as *const EcuSimSensorFrame) },
        EcuSimStatus::Ok
    );
    assert_eq!(ecu_sim_handle_set_time(handle_b, 2_000), EcuSimStatus::Ok);

    let mut snap_a = EcuSimSnapshot::default();
    let mut snap_b = EcuSimSnapshot::default();
    assert_eq!(
        unsafe { ecu_sim_handle_snapshot(handle_a, (&mut snap_a) as *mut EcuSimSnapshot) },
        EcuSimStatus::Ok
    );
    assert_eq!(
        unsafe { ecu_sim_handle_snapshot(handle_b, (&mut snap_b) as *mut EcuSimSnapshot) },
        EcuSimStatus::Ok
    );

    let mut events_a = [EcuSimOutputEvent::ZERO; 8];
    let mut events_b = [EcuSimOutputEvent::ZERO; 8];
    let copied_a =
        unsafe { ecu_sim_handle_dequeue_events(handle_a, events_a.as_mut_ptr(), events_a.len()) };
    let copied_b =
        unsafe { ecu_sim_handle_dequeue_events(handle_b, events_b.as_mut_ptr(), events_b.len()) };

    assert_eq!(snap_a.now_us, 2_000);
    assert_eq!(snap_a.synced, 1);
    assert!(
        snap_a.rpm > 0,
        "active handle should report runtime progress"
    );
    assert_eq!(snap_b.now_us, 2_000);
    assert_eq!(snap_b.synced, 0, "second handle must stay unsynced");
    assert_eq!(
        snap_b.rpm, 0,
        "second handle must not inherit runtime state"
    );
    assert!(copied_a > 0, "first handle should have queued output");
    assert_eq!(copied_b, 0, "second handle should have no queued output");
    assert_ne!(
        snap_a.tooth, snap_b.tooth,
        "handles should diverge independently"
    );

    unsafe {
        ecu_sim_handle_destroy(handle_a);
        ecu_sim_handle_destroy(handle_b);
    }
}

#[test]
fn handle_snapshots_and_event_queues_stay_independent() {
    let _guard = lock_tests();
    let handle_a = ecu_sim_handle_create();
    let handle_b = ecu_sim_handle_create();
    assert!(!handle_a.is_null(), "first handle should be created");
    assert!(!handle_b.is_null(), "second handle should be created");

    let cfg = cfg();
    let sensors_a = EcuSimSensorFrame {
        now_us: 1_000,
        map_kpa10: 900,
        tps_x100: 2_000,
        lambda_valid: 1,
        ..Default::default()
    };

    assert_eq!(
        unsafe { ecu_sim_handle_init(handle_a, (&cfg) as *const EcuSimInitCfg) },
        EcuSimStatus::Ok
    );
    assert_eq!(
        unsafe { ecu_sim_handle_init(handle_b, (&cfg) as *const EcuSimInitCfg) },
        EcuSimStatus::Ok
    );

    assert_eq!(
        unsafe { ecu_sim_handle_set_sensors(handle_a, (&sensors_a) as *const EcuSimSensorFrame) },
        EcuSimStatus::Ok
    );
    assert_eq!(ecu_sim_handle_set_time(handle_a, 1_000), EcuSimStatus::Ok);
    assert_eq!(
        ecu_sim_handle_on_crank_edge(handle_a, 1_000),
        EcuSimStatus::Ok
    );
    assert_eq!(
        ecu_sim_handle_on_crank_edge(handle_a, 1_500),
        EcuSimStatus::Ok
    );
    assert_eq!(
        ecu_sim_handle_on_cam_edge(handle_a, 1_500),
        EcuSimStatus::Ok
    );
    assert_eq!(ecu_sim_handle_step(handle_a, 2_000), EcuSimStatus::Ok);

    let mut snap_a = EcuSimSnapshot::default();
    let mut snap_b = EcuSimSnapshot::default();
    assert_eq!(
        unsafe { ecu_sim_handle_snapshot(handle_a, (&mut snap_a) as *mut EcuSimSnapshot) },
        EcuSimStatus::Ok
    );
    assert_eq!(
        unsafe { ecu_sim_handle_snapshot(handle_b, (&mut snap_b) as *mut EcuSimSnapshot) },
        EcuSimStatus::Ok
    );

    let mut events_a = [EcuSimOutputEvent::ZERO; ECU_SIM_MAX_EVENTS];
    let mut events_b = [EcuSimOutputEvent::ZERO; ECU_SIM_MAX_EVENTS];
    let copied_a =
        unsafe { ecu_sim_handle_dequeue_events(handle_a, events_a.as_mut_ptr(), events_a.len()) };
    let copied_b =
        unsafe { ecu_sim_handle_dequeue_events(handle_b, events_b.as_mut_ptr(), events_b.len()) };

    assert_eq!(snap_a.now_us, 2_000);
    assert_eq!(snap_a.synced, 1);
    assert_eq!(snap_a.rpm, 2_000);
    assert_eq!(snap_b.now_us, 0);
    assert_eq!(snap_b.synced, 0);
    assert_eq!(snap_b.rpm, 0);
    assert!(copied_a > 0);
    assert_eq!(copied_b, 0);
    assert!(events_a[..copied_a]
        .iter()
        .any(|event| { event.kind == EcuSimOutputKind::Injector as i32 && event.channel == 7 }));
    assert!(events_a[..copied_a]
        .iter()
        .any(|event| { event.kind == EcuSimOutputKind::Ignition as i32 && event.channel == 9 }));

    unsafe {
        ecu_sim_handle_destroy(handle_a);
        ecu_sim_handle_destroy(handle_b);
    }
}

#[test]
fn singleton_wrapper_compatibility_matches_handle_state() {
    let _guard = lock_tests();

    let handle = ecu_sim_handle_create();
    assert!(!handle.is_null(), "handle should be created");

    let cfg = cfg();
    let sensors = EcuSimSensorFrame {
        now_us: 1_000,
        map_kpa10: 900,
        tps_x100: 2_000,
        clt_c10: 850,
        iat_c10: 300,
        vbatt_mv: 13_800,
        baro_kpa10: 1_013,
        lambda_x100: 100,
        lambda_valid: 1,
        knock_x100: 0,
        vehicle_speed_kph10: 0,
        maf_x100: 0,
        cam_phase_deg10: 0,
        cam_phase_valid: 0,
        validity_flags: 0,
    };

    ecu_sim_reset();
    assert_eq!(
        unsafe { ecu_sim_init((&cfg) as *const EcuSimInitCfg) },
        EcuSimStatus::Ok
    );
    assert_eq!(
        unsafe { ecu_sim_set_sensors((&sensors) as *const EcuSimSensorFrame) },
        EcuSimStatus::Ok
    );
    assert_eq!(ecu_sim_set_time(1_000), EcuSimStatus::Ok);
    assert_eq!(ecu_sim_on_crank_edge(1_000), EcuSimStatus::Ok);
    assert_eq!(ecu_sim_on_crank_edge(1_500), EcuSimStatus::Ok);
    assert_eq!(ecu_sim_on_cam_edge(1_500), EcuSimStatus::Ok);
    assert_eq!(ecu_sim_step(2_000), EcuSimStatus::Ok);

    assert_eq!(
        unsafe { ecu_sim_handle_init(handle, (&cfg) as *const EcuSimInitCfg) },
        EcuSimStatus::Ok
    );
    assert_eq!(
        unsafe { ecu_sim_handle_set_sensors(handle, (&sensors) as *const EcuSimSensorFrame) },
        EcuSimStatus::Ok
    );
    assert_eq!(ecu_sim_handle_set_time(handle, 1_000), EcuSimStatus::Ok);
    assert_eq!(
        ecu_sim_handle_on_crank_edge(handle, 1_000),
        EcuSimStatus::Ok
    );
    assert_eq!(
        ecu_sim_handle_on_crank_edge(handle, 1_500),
        EcuSimStatus::Ok
    );
    assert_eq!(ecu_sim_handle_on_cam_edge(handle, 1_500), EcuSimStatus::Ok);
    assert_eq!(ecu_sim_handle_step(handle, 2_000), EcuSimStatus::Ok);

    let mut singleton_snapshot = EcuSimSnapshot::default();
    let mut handle_snapshot = EcuSimSnapshot::default();
    assert_eq!(
        unsafe { ecu_sim_snapshot((&mut singleton_snapshot) as *mut EcuSimSnapshot) },
        EcuSimStatus::Ok
    );
    assert_eq!(
        unsafe { ecu_sim_handle_snapshot(handle, (&mut handle_snapshot) as *mut EcuSimSnapshot) },
        EcuSimStatus::Ok
    );

    let mut singleton_events = [EcuSimOutputEvent::ZERO; ECU_SIM_MAX_EVENTS];
    let mut handle_events = [EcuSimOutputEvent::ZERO; ECU_SIM_MAX_EVENTS];
    let singleton_copied =
        unsafe { ecu_sim_dequeue_events(singleton_events.as_mut_ptr(), singleton_events.len()) };
    let handle_copied = unsafe {
        ecu_sim_handle_dequeue_events(handle, handle_events.as_mut_ptr(), handle_events.len())
    };

    assert_eq!(singleton_snapshot, handle_snapshot);
    assert_eq!(singleton_copied, handle_copied);
    assert_eq!(
        &singleton_events[..singleton_copied],
        &handle_events[..handle_copied]
    );

    ecu_sim_reset();
    unsafe {
        ecu_sim_handle_destroy(handle);
    }
}

#[test]
fn handle_calls_are_serialized_for_same_handle() {
    let _guard = lock_tests();
    let handle = ecu_sim_handle_create();
    assert!(!handle.is_null(), "handle should be created");
    let cfg = cfg();
    assert_eq!(
        unsafe { ecu_sim_handle_init(handle, (&cfg) as *const EcuSimInitCfg) },
        EcuSimStatus::Ok
    );

    let handle_addr = handle as usize;
    let mut threads = Vec::new();
    for worker in 0..4u32 {
        threads.push(std::thread::spawn(move || {
            let handle = handle_addr as *mut super::EcuSimHandleOpaque;
            for step in 0..32u32 {
                let now_us = 1_000 + worker * 1_000 + step;
                assert_eq!(ecu_sim_handle_set_time(handle, now_us), EcuSimStatus::Ok);
            }
        }));
    }
    for thread in threads {
        thread.join().expect("handle worker thread");
    }

    let mut snapshot = EcuSimSnapshot::default();
    assert_eq!(
        unsafe { ecu_sim_handle_snapshot(handle, (&mut snapshot) as *mut EcuSimSnapshot) },
        EcuSimStatus::Ok
    );
    assert!(snapshot.now_us >= 1_000);

    unsafe {
        ecu_sim_handle_destroy(handle);
    }
}

#[test]
fn event_overflow_is_latched_until_queue_drained() {
    let _guard = lock_tests();
    warm_synced_runtime();

    assert_eq!(ecu_sim_step(11_000), EcuSimStatus::Ok);
}

#[test]
fn handle_skeleton_starts_uninitialized() {
    let mut handle = EcuSimHandle::new();
    assert_eq!(handle.require_init(), Err(EcuSimStatus::ErrNotInit));
    assert!(!handle.initialized());
    handle.reset();
    assert_eq!(handle.require_init(), Err(EcuSimStatus::ErrNotInit));
}
