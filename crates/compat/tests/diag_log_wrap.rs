use ecu_compat::compat::EcuState;
use ecu_compat::ts::pages::PAGE_DIAG_LOG;
use ecu_compat::{Kpa10, Micros};
use ecu_ts::pages::{DIAG_LOG_ENTRY_BYTES, DIAG_LOG_ENTRY_COUNT};
use ecu_ts::server::PageStore;

#[test]
fn diag_log_wraps_and_retains_recent() {
    let mut state = EcuState::new();
    // Configure limits and clear time so we can generate events
    state.config.sensors_limits.map_min_kpa_x10 = 500;
    state.config.sensors_limits.map_max_kpa_x10 = 3000;
    state.config.sensors_limits.clear_time_s = 1;

    // Generate >16 events by toggling OOB -> in-range sustained
    // Each cycle: push OOB at t, then two in-range updates to exceed clear_time and emit event
    let mut now = 0u32;
    for _ in 0..20 {
        let _ = state.process_sensor_update(Micros::new(now), Kpa10::new(100), 10); // OOB
        now = now.wrapping_add(1_500_000); // 1.5s later back in-range
        let _ = state.process_sensor_update(Micros::new(now), Kpa10::new(1000), 10);
        now = now.wrapping_add(1_500_000); // sustain
        let _ = state.process_sensor_update(Micros::new(now), Kpa10::new(1000), 10);
        now = now.wrapping_add(10_000);
    }

    let pages = state.page_store();

    let mut out = [0u8; DIAG_LOG_ENTRY_BYTES * DIAG_LOG_ENTRY_COUNT];
    let _ = pages.read_page(PAGE_DIAG_LOG, &mut out).expect("read log");
    // Count non-empty entries (code != 0)
    let count = out
        .chunks_exact(DIAG_LOG_ENTRY_BYTES)
        .filter(|e| e[0] != 0)
        .count();
    assert_eq!(count, 16, "ring buffer should contain 16 recent events");
}
