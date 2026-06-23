use ecu_compat::compat::EcuState;
use ecu_compat::ts::pages::PAGE_DIAG_LOG;
use ecu_compat::{Kpa10, Micros};
use ecu_ts::pages::{
    DIAG_LOG_ENTRY_BYTES, DIAG_LOG_ENTRY_COUNT, TS_DIAG_SOURCE_CONTEXT_PRESENT,
    TS_DIAG_SOURCE_SENSOR, TS_DIAG_SOURCE_USER, TS_FAULT_ACTION_OBSERVE_ONLY,
    TS_FAULT_SEVERITY_WARNING,
};
use ecu_ts::server::PageStore;

#[test]
fn diag_log_contains_events_after_fault() {
    let mut state = EcuState::new();
    state.set_emergency_trigger_map_oob(false); // don't care for this test
    state.config.sensors_limits.map_min_kpa_x10 = 500;
    state.config.sensors_limits.map_max_kpa_x10 = 3000;
    state.config.sensors_limits.clear_time_s = 1;

    // Simulate an out-of-range then sustained in-range to push an event into log
    let _ = state.process_sensor_update(Micros::new(0), Kpa10::new(100), 10);
    let _ = state.process_sensor_update(Micros::new(1_500_000), Kpa10::new(1000), 10); // back in range
    let _ = state.process_sensor_update(Micros::new(3_000_000), Kpa10::new(1000), 10); // sustain past clear time

    let pages = state.page_store();

    let mut out = [0u8; DIAG_LOG_ENTRY_BYTES * DIAG_LOG_ENTRY_COUNT];
    let n = pages
        .read_page(PAGE_DIAG_LOG, &mut out)
        .expect("read diag log");
    assert_eq!(n, DIAG_LOG_ENTRY_BYTES * DIAG_LOG_ENTRY_COUNT);
    // First entry should be non-zero code
    assert!(out[0] != 0, "expected at least one diag event present");
    assert_eq!(out[1], TS_FAULT_SEVERITY_WARNING);
    assert_eq!(out[2], TS_FAULT_ACTION_OBSERVE_ONLY);
    assert_eq!(
        out[3],
        TS_DIAG_SOURCE_SENSOR | TS_DIAG_SOURCE_CONTEXT_PRESENT
    );
    let context = u32::from_le_bytes([out[12], out[13], out[14], out[15]]);
    assert_eq!(
        context, 1000,
        "MAP-range clear event should preserve the in-range sample that closed the fault"
    );
}

#[test]
fn diag_log_encodes_cam_missing_event() {
    let mut state = EcuState::new();
    // Manually push a cam-missing event into the log
    state.diag_log_mut().push(ecu_compat::diag::DiagEvent {
        code: ecu_compat::diag::DiagCode::CamMissing,
        timestamp: ecu_compat::Micros::new(100),
        source: ecu_compat::diag::DiagSource::User,
        context: Some(42),
        start_us: 100,
        end_us: 200,
    });

    let pages = state.page_store();

    let mut out = [0u8; DIAG_LOG_ENTRY_BYTES * DIAG_LOG_ENTRY_COUNT];
    let n = pages
        .read_page(PAGE_DIAG_LOG, &mut out)
        .expect("read diag log");
    assert_eq!(n, DIAG_LOG_ENTRY_BYTES * DIAG_LOG_ENTRY_COUNT);
    assert_eq!(out[0], 3, "cam missing code should encode as 3");
    assert_eq!(out[1], TS_FAULT_SEVERITY_WARNING);
    assert_eq!(out[2], TS_FAULT_ACTION_OBSERVE_ONLY);
    assert_eq!(out[3], TS_DIAG_SOURCE_USER | TS_DIAG_SOURCE_CONTEXT_PRESENT);
    let start = u32::from_le_bytes([out[4], out[5], out[6], out[7]]);
    let end = u32::from_le_bytes([out[8], out[9], out[10], out[11]]);
    let context = u32::from_le_bytes([out[12], out[13], out[14], out[15]]);
    assert_eq!(start, 100);
    assert_eq!(end, 200);
    assert_eq!(context, 42);
}
