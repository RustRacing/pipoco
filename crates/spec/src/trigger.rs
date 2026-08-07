use crate::{Degrees10, EngineMode, Micros, Rpm};

const TEETH_PER_REV_OBSERVED: u16 = 58;
const TOOTH_ANGLE_DEG10: u16 = 120;
const MAX_RPM_ESTIMATE: u16 = 20_000;
const RPM_NUMERATOR_US_PER_MIN: u64 = 60_000_000;
const STALL_TIMEOUT_US: u32 = 400_000;
const GAP_TOLERANCE_MIN_NUM: u64 = 3;
const GAP_TOLERANCE_MIN_DEN: u64 = 4;
const GAP_TOLERANCE_MAX_NUM: u64 = 5;
const GAP_TOLERANCE_MAX_DEN: u64 = 4;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct SyncTransitionInput {
    sync_state: TriggerSyncState,
    tooth_index: u8,
    angle_deg10: Degrees10,
    missing_tooth_candidate: bool,
    gap_outside_tolerance: bool,
    prior_gap_fault_windows: u8,
    prev_dt_us: u32,
    dt_us: u32,
    stall: bool,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub enum TriggerSyncState {
    #[default]
    NoSync,
    PreSync,
    Synced,
    SyncLoss,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub struct TriggerState {
    pub trigger_state: TriggerSyncState,
    pub sync_state: TriggerSyncState,
    pub last_tooth_timestamp_us: Micros,
    pub prev_tooth_interval_us: Micros,
    pub gap_fault_windows: u8,
    pub stall_counter: u16,
    pub tooth_index: u8,
    pub angle_deg10: Degrees10,
    pub rpm_estimate: Rpm,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub struct TriggerStepResult {
    pub state: TriggerState,
    pub sync_state: TriggerSyncState,
    pub angle_deg10: Degrees10,
    pub rpm_estimate: Rpm,
    pub engine_mode: EngineMode,
    pub cancel_pending_events: bool,
}

#[must_use]
pub fn trigger_60_2_step(state: TriggerState, tooth_timestamp_us: Micros) -> TriggerStepResult {
    let dt = tooth_timestamp_us
        .get()
        .saturating_sub(state.last_tooth_timestamp_us.get());
    let prev_dt = state.prev_tooth_interval_us.get();

    let missing_tooth_candidate = if prev_dt == 0 {
        false
    } else {
        u64::from(dt) >= (u64::from(prev_dt) * 3) / 2
    };

    let rpm_estimate = rpm_estimate_from_dt(dt, state.rpm_estimate);
    let stall = dt >= STALL_TIMEOUT_US;
    let gap_outside_tolerance = gap_outside_tolerance(prev_dt, dt, missing_tooth_candidate);
    let (next_sync_state, next_tooth_index, next_angle_deg10, next_gap_fault_windows) =
        step_sync_and_angle(SyncTransitionInput {
            sync_state: state.sync_state,
            tooth_index: state.tooth_index,
            angle_deg10: state.angle_deg10,
            missing_tooth_candidate,
            gap_outside_tolerance,
            prior_gap_fault_windows: state.gap_fault_windows,
            prev_dt_us: prev_dt,
            dt_us: dt,
            stall,
        });
    let cancel_pending_events = state.sync_state != TriggerSyncState::SyncLoss
        && next_sync_state == TriggerSyncState::SyncLoss;
    let stall_counter = if stall {
        state.stall_counter.saturating_add(1)
    } else {
        0
    };
    let rpm_estimate = if next_sync_state == TriggerSyncState::SyncLoss {
        Rpm::new(0)
    } else {
        rpm_estimate
    };
    let engine_mode = if stall || rpm_estimate.get() == 0 {
        EngineMode::Off
    } else {
        EngineMode::Running
    };

    let next_state = TriggerState {
        trigger_state: next_sync_state,
        sync_state: next_sync_state,
        last_tooth_timestamp_us: tooth_timestamp_us,
        prev_tooth_interval_us: Micros::new(dt),
        gap_fault_windows: next_gap_fault_windows,
        stall_counter,
        tooth_index: next_tooth_index,
        angle_deg10: next_angle_deg10,
        rpm_estimate,
    };

    TriggerStepResult {
        state: next_state,
        sync_state: next_sync_state,
        angle_deg10: next_angle_deg10,
        rpm_estimate,
        engine_mode,
        cancel_pending_events,
    }
}

fn gap_outside_tolerance(prev_dt_us: u32, dt_us: u32, missing_tooth_candidate: bool) -> bool {
    if prev_dt_us == 0 || dt_us == 0 || missing_tooth_candidate {
        return false;
    }

    let lhs = u64::from(dt_us);
    let rhs = u64::from(prev_dt_us);
    lhs * GAP_TOLERANCE_MIN_DEN < rhs * GAP_TOLERANCE_MIN_NUM
        || lhs * GAP_TOLERANCE_MAX_DEN > rhs * GAP_TOLERANCE_MAX_NUM
}

fn rpm_estimate_from_dt(dt_us: u32, prior: Rpm) -> Rpm {
    if dt_us == 0 {
        return prior;
    }

    let den = u64::from(dt_us) * u64::from(TEETH_PER_REV_OBSERVED);
    if den == 0 {
        return prior;
    }

    let rpm = RPM_NUMERATOR_US_PER_MIN / den;
    let clamped = core::cmp::min(rpm, u64::from(MAX_RPM_ESTIMATE)) as u16;
    Rpm::new(clamped)
}

fn step_sync_and_angle(input: SyncTransitionInput) -> (TriggerSyncState, u8, Degrees10, u8) {
    let SyncTransitionInput {
        sync_state,
        tooth_index,
        angle_deg10,
        missing_tooth_candidate,
        gap_outside_tolerance,
        prior_gap_fault_windows,
        prev_dt_us,
        dt_us,
        stall,
    } = input;
    if dt_us == 0 {
        return (
            sync_state,
            tooth_index,
            angle_deg10,
            prior_gap_fault_windows,
        );
    }
    if stall {
        return (TriggerSyncState::SyncLoss, 0, Degrees10::new(0), 0);
    }

    match sync_state {
        TriggerSyncState::NoSync | TriggerSyncState::SyncLoss => {
            if missing_tooth_candidate {
                (TriggerSyncState::PreSync, 0, Degrees10::new(0), 0)
            } else {
                (TriggerSyncState::NoSync, 0, Degrees10::new(0), 0)
            }
        }
        TriggerSyncState::PreSync => {
            if missing_tooth_candidate {
                (TriggerSyncState::PreSync, 0, Degrees10::new(0), 0)
            } else if prev_dt_us > 0 && dt_us >= prev_dt_us {
                (TriggerSyncState::NoSync, 0, Degrees10::new(0), 0)
            } else {
                (TriggerSyncState::Synced, 0, Degrees10::new(0), 0)
            }
        }
        TriggerSyncState::Synced => {
            let gap_fault_windows = if gap_outside_tolerance {
                prior_gap_fault_windows.saturating_add(1)
            } else {
                0
            };
            if gap_fault_windows >= 2 {
                return (TriggerSyncState::SyncLoss, 0, Degrees10::new(0), 0);
            }
            if missing_tooth_candidate {
                (TriggerSyncState::Synced, 0, Degrees10::new(0), 0)
            } else {
                let next_index = (u16::from(tooth_index) + 1) % TEETH_PER_REV_OBSERVED;
                let angle = (next_index * TOOTH_ANGLE_DEG10) % 7200;
                (
                    TriggerSyncState::Synced,
                    next_index as u8,
                    Degrees10::new(angle as i16),
                    gap_fault_windows,
                )
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn run_stream(times: &[u32]) -> [TriggerStepResult; 8] {
        let mut out = [TriggerStepResult::default(); 8];
        let mut state = TriggerState::default();
        let mut i = 0usize;
        while i < times.len() {
            let step = trigger_60_2_step(state, Micros::new(times[i]));
            state = step.state;
            out[i] = step;
            i += 1;
        }
        out
    }

    #[test]
    fn acquires_sync_from_missing_tooth_then_confirm_tooth() {
        let trace = run_stream(&[1000, 2000, 3500, 4500]);

        assert_eq!(trace[1].sync_state, TriggerSyncState::NoSync);
        assert_eq!(trace[2].sync_state, TriggerSyncState::PreSync);
        assert_eq!(trace[3].sync_state, TriggerSyncState::Synced);
        assert_eq!(trace[3].angle_deg10, Degrees10::new(0));
    }

    #[test]
    fn synced_state_advances_tooth_index_and_angle() {
        let mut state = TriggerState::default();
        for t in [1000u32, 2000, 3500, 4500] {
            state = trigger_60_2_step(state, Micros::new(t)).state;
        }
        assert_eq!(state.sync_state, TriggerSyncState::Synced);

        let s1 = trigger_60_2_step(state, Micros::new(5500));
        assert_eq!(s1.angle_deg10, Degrees10::new(120));
        let s2 = trigger_60_2_step(s1.state, Micros::new(6500));
        assert_eq!(s2.angle_deg10, Degrees10::new(240));
    }

    #[test]
    fn synced_state_enters_sync_loss_after_two_consecutive_gap_fault_windows() {
        let mut state = TriggerState::default();
        for t in [1000u32, 2000, 3500, 4500] {
            state = trigger_60_2_step(state, Micros::new(t)).state;
        }
        assert_eq!(state.sync_state, TriggerSyncState::Synced);

        // Two short gaps, each outside tolerance but not missing-tooth candidates.
        let s1 = trigger_60_2_step(state, Micros::new(5200));
        assert_eq!(s1.sync_state, TriggerSyncState::Synced);
        assert_eq!(s1.state.gap_fault_windows, 1);
        let s2 = trigger_60_2_step(s1.state, Micros::new(5700));
        assert_eq!(s2.sync_state, TriggerSyncState::SyncLoss);
        assert!(s2.cancel_pending_events);
    }

    #[test]
    fn sync_loss_from_stall_timeout_forces_zero_rpm() {
        let mut state = TriggerState::default();
        for t in [1000u32, 2000, 3500, 4500] {
            state = trigger_60_2_step(state, Micros::new(t)).state;
        }

        let stalled = trigger_60_2_step(state, Micros::new(500_000));
        assert_eq!(stalled.sync_state, TriggerSyncState::SyncLoss);
        assert_eq!(stalled.rpm_estimate, Rpm::new(0));
        assert_eq!(stalled.engine_mode, EngineMode::Off);
        assert_eq!(stalled.state.stall_counter, 1);
        assert!(stalled.cancel_pending_events);
    }

    #[test]
    fn no_tooth_gap_below_timeout_decays_rpm_without_forcing_off_mode() {
        let mut state = TriggerState::default();
        for t in [1000u32, 2000, 3500, 4500] {
            state = trigger_60_2_step(state, Micros::new(t)).state;
        }

        let decayed = trigger_60_2_step(state, Micros::new(204_500));
        assert_eq!(decayed.sync_state, TriggerSyncState::Synced);
        assert_eq!(decayed.rpm_estimate, Rpm::new(5));
        assert_eq!(decayed.engine_mode, EngineMode::Running);
    }

    #[test]
    fn presync_failed_confirmation_returns_no_sync() {
        let mut state = TriggerState::default();
        state = trigger_60_2_step(state, Micros::new(1000)).state;
        state = trigger_60_2_step(state, Micros::new(2000)).state;
        let presync = trigger_60_2_step(state, Micros::new(3500));
        assert_eq!(presync.sync_state, TriggerSyncState::PreSync);

        // Confirmation must contract after candidate gap; this one expands.
        let failed = trigger_60_2_step(presync.state, Micros::new(5200));
        assert_eq!(failed.sync_state, TriggerSyncState::NoSync);
    }

    #[test]
    fn cancel_signal_is_idempotent_inside_sync_loss() {
        let mut state = TriggerState::default();
        for t in [1000u32, 2000, 3500, 4500, 5200, 5700] {
            state = trigger_60_2_step(state, Micros::new(t)).state;
        }
        assert_eq!(state.sync_state, TriggerSyncState::SyncLoss);

        let next = trigger_60_2_step(state, Micros::new(7800));
        assert_eq!(next.sync_state, TriggerSyncState::PreSync);
        assert!(!next.cancel_pending_events);
    }

    #[test]
    fn rpm_estimate_clamps_at_bound() {
        let state = TriggerState {
            last_tooth_timestamp_us: Micros::new(0),
            prev_tooth_interval_us: Micros::new(1),
            ..TriggerState::default()
        };

        let step = trigger_60_2_step(state, Micros::new(1));
        assert_eq!(step.rpm_estimate, Rpm::new(20_000));
    }

    #[test]
    fn same_timestamp_stream_is_deterministic() {
        let times = [1000, 2000, 3500, 4500, 5500, 6500, 7500, 8500];
        let left = run_stream(&times);
        let right = run_stream(&times);
        assert_eq!(left, right);
    }
}
