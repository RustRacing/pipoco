use ecu_compat::compat::EcuState;
use ecu_compat::trigger::SyncState;
use ecu_compat::ts::pages::{PAGE_DIAG, PAGE_SNAPSHOT, TS_DIAG_BYTES};
use ecu_compat::units::{Kpa10, Micros};
use ecu_ts::pages::{
    TS_CANCEL_REASON_NONE, TS_CURRENT_FAULT_MAP_RANGE, TS_FAULT_ACTION_LIMP_HOME,
    TS_FAULT_FLAG_ACTIVE, TS_FAULT_FLAG_EMERGENCY_MODE, TS_FAULT_FLAG_SNAPSHOT_PRESENT,
    TS_FAULT_SEVERITY_WARNING,
};
use ecu_ts::server::PageStore;

#[test]
fn diag_page_sync_fields_reflect_state() {
    let mut state = EcuState::new();

    let mut out = [0u8; TS_DIAG_BYTES];
    {
        let pages = state.page_store();
        let n = pages.read_page(PAGE_DIAG, &mut out).expect("read diag");
        assert_eq!(n, TS_DIAG_BYTES);
        assert_eq!(out[0], 0, "tooth count should default to zero");
        assert_eq!(out[1], 0, "cam reference flag should default clear");
        assert_eq!(out[2], 0, "sync state should default unsynced");
        assert_eq!(out[3], 0, "sync detail should default unsynced");
        assert_eq!(out[24], 0, "fault code should default clear");
        assert_eq!(out[25], 0, "fault severity should default clear");
        assert_eq!(out[26], 0, "fault action should default clear");
        assert_eq!(out[27], 0, "cancel reason should default clear");
        assert_eq!(out[28], 0, "fault flags should default clear");
        assert_eq!(out[29], 0, "latest diag code should default clear");
        assert_eq!(&out[30..], &[0; 2], "reserved tail should stay zeroed");
    }

    state.set_trigger_inputs(1_500, true, 17);
    state.refresh_snapshot();

    {
        let pages = state.page_store();
        let n = pages.read_page(PAGE_DIAG, &mut out).expect("read diag");
        assert_eq!(n, TS_DIAG_BYTES);
        assert_eq!(out[0], 17, "tooth count should be live");
        assert_eq!(out[1], 0, "boolean sync alone is not cam-referenced");
        assert_eq!(out[2], 2, "sync state should be locked");
        assert_eq!(out[3], 2, "sync detail should be locked without cam ref");
        assert_eq!(u16::from_le_bytes([out[8], out[9]]), 1_500);
        assert_eq!(out[24], 0, "fault code should stay clear");
        assert_eq!(out[25], 0, "fault severity should stay clear");
        assert_eq!(out[26], 0, "fault action should stay clear");
        assert_eq!(out[27], 0, "cancel reason should stay clear");
        assert_eq!(out[28], 0, "fault flags should stay clear");
        assert_eq!(out[29], 0, "latest diag code should stay clear");
        assert_eq!(&out[30..], &[0; 2], "reserved tail should stay zeroed");
    }
}

#[test]
fn diag_page_cam_reference_flag_sets_when_locked_with_cam_ref() {
    let mut state = EcuState::new();
    state.set_trigger_inputs(2_000, true, 23);
    state.refresh_snapshot();
    state.snapshot.sync = SyncState::Locked { cam_ref: true };

    let pages = state.page_store();
    let mut out = [0u8; TS_DIAG_BYTES];
    let n = pages.read_page(PAGE_DIAG, &mut out).expect("read diag");
    assert_eq!(n, TS_DIAG_BYTES);
    assert_eq!(out[0], 23, "tooth count should be live");
    assert_eq!(out[1], 1, "cam reference flag should be set");
    assert_eq!(out[2], 2, "sync state should be locked");
    assert_eq!(out[3], 3, "sync detail should be locked with cam ref");
    assert_eq!(u16::from_le_bytes([out[8], out[9]]), 2_000);
    assert_eq!(out[24], 0, "fault code should stay clear");
    assert_eq!(out[25], 0, "fault severity should stay clear");
    assert_eq!(out[26], 0, "fault action should stay clear");
    assert_eq!(out[27], 0, "cancel reason should stay clear");
    assert_eq!(out[28], 0, "fault flags should stay clear");
    assert_eq!(out[29], 0, "latest diag code should stay clear");
    assert_eq!(&out[30..], &[0; 2], "reserved tail should stay zeroed");
}

#[test]
fn diag_and_snapshot_pages_expose_active_map_fault_surface() {
    let mut state = EcuState::new();
    state.set_emergency_trigger_map_oob(true);
    state.process_sensor_update(Micros::new(1_000), Kpa10::new(9_999), 20);
    state.refresh_snapshot();

    let pages = state.page_store();

    let mut diag = [0u8; TS_DIAG_BYTES];
    let n = pages.read_page(PAGE_DIAG, &mut diag).expect("read diag");
    assert_eq!(n, TS_DIAG_BYTES);
    assert_eq!(diag[24], TS_CURRENT_FAULT_MAP_RANGE);
    assert_eq!(diag[25], TS_FAULT_SEVERITY_WARNING);
    assert_eq!(diag[26], TS_FAULT_ACTION_LIMP_HOME);
    assert_eq!(diag[27], TS_CANCEL_REASON_NONE);
    assert_eq!(
        diag[28],
        TS_FAULT_FLAG_ACTIVE | TS_FAULT_FLAG_EMERGENCY_MODE | TS_FAULT_FLAG_SNAPSHOT_PRESENT
    );
    assert_eq!(
        diag[29], 0,
        "active diagnostic has not yet rolled into the log"
    );

    let mut snapshot = [0u8; 32];
    let n = pages
        .read_page(PAGE_SNAPSHOT, &mut snapshot)
        .expect("read snapshot");
    assert_eq!(n, 32);
    assert_eq!(snapshot[3], TS_CANCEL_REASON_NONE);
    assert_eq!(snapshot[18], TS_CURRENT_FAULT_MAP_RANGE);
    assert_eq!(snapshot[19], TS_FAULT_SEVERITY_WARNING);
}
