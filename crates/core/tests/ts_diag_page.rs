use ecu_core::compat::EcuState;
use ecu_core::trigger::SyncState;
use ecu_core::ts::pages::{PAGE_DIAG, TS_DIAG_BYTES};
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
        assert_eq!(&out[24..], &[0; 8], "reserved tail should stay zeroed");
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
        assert_eq!(&out[24..], &[0; 8], "reserved tail should stay zeroed");
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
    assert_eq!(&out[24..], &[0; 8], "reserved tail should stay zeroed");
}
