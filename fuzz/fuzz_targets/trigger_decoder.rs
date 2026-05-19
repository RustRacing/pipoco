#![no_main]

use ecu_spec::{trigger_60_2_step, Micros, TriggerState, TriggerSyncState};
use libfuzzer_sys::fuzz_target;

const TRIGGER_FUZZ_MAX_INTERVALS: usize = 128;
const TRIGGER_FUZZ_MAX_BYTES: usize = TRIGGER_FUZZ_MAX_INTERVALS * 4;
const MIN_INTERVAL_US: u32 = 50;
const MAX_INTERVAL_US: u32 = 200_000;
const INTERVAL_SPAN_US: u32 = MAX_INTERVAL_US - MIN_INTERVAL_US + 1;

fn map_interval(raw: u32) -> u32 {
    MIN_INTERVAL_US + (raw % INTERVAL_SPAN_US)
}

fuzz_target!(|data: &[u8]| {
    if data.len() > TRIGGER_FUZZ_MAX_BYTES || data.len() % 4 != 0 {
        return;
    }

    let mut state = TriggerState::default();
    let mut timestamp_us: u64 = state.last_tooth_timestamp_us.get() as u64;

    for chunk in data.chunks_exact(4) {
        let raw = u32::from_le_bytes([chunk[0], chunk[1], chunk[2], chunk[3]]);
        let interval_us = map_interval(raw);
        timestamp_us = timestamp_us.saturating_add(interval_us as u64);

        let step = std::panic::catch_unwind(|| {
            trigger_60_2_step(state, Micros::new(timestamp_us.min(u32::MAX as u64) as u32))
        });
        assert!(step.is_ok(), "trigger_60_2_step panicked");

        let step = match step {
            Ok(step) => step,
            Err(_) => return,
        };

        assert!(step.angle_deg10.get() < 7200);
        assert!(step.rpm_estimate.get() <= 20_000);
        assert_eq!(step.sync_state, step.state.sync_state);
        assert!(matches!(
            step.sync_state,
            TriggerSyncState::NoSync
                | TriggerSyncState::PreSync
                | TriggerSyncState::Synced
                | TriggerSyncState::SyncLoss
        ));

        state = step.state;
    }
});
