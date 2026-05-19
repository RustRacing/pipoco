use crate::{TriggerState, TriggerSyncState};

#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub enum CamTooth {
    #[default]
    Edge,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub enum CamPhase {
    #[default]
    Unknown,
    PhaseA,
    PhaseB,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub struct CamPhaseState {
    pub phase: CamPhase,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub struct CamPhaseStepResult {
    pub state: CamPhaseState,
    pub phase: CamPhase,
    pub edge_applied: bool,
}

#[must_use]
pub fn cam_phase_step(
    state: CamPhaseState,
    trigger: TriggerState,
    cam_tooth: Option<CamTooth>,
) -> CamPhaseStepResult {
    match cam_tooth {
        None => CamPhaseStepResult {
            state,
            phase: state.phase,
            edge_applied: false,
        },
        Some(_) => {
            if is_valid_cam_window(trigger) {
                let next_phase = next_phase(state.phase);
                CamPhaseStepResult {
                    state: CamPhaseState { phase: next_phase },
                    phase: next_phase,
                    edge_applied: true,
                }
            } else {
                CamPhaseStepResult {
                    state,
                    phase: state.phase,
                    edge_applied: false,
                }
            }
        }
    }
}

fn is_valid_cam_window(trigger: TriggerState) -> bool {
    trigger.sync_state == TriggerSyncState::Synced && trigger.tooth_index == 0
}

fn next_phase(phase: CamPhase) -> CamPhase {
    match phase {
        CamPhase::Unknown => CamPhase::PhaseA,
        CamPhase::PhaseA => CamPhase::PhaseB,
        CamPhase::PhaseB => CamPhase::PhaseA,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{Degrees10, Micros, Rpm};

    fn synced_window_trigger() -> TriggerState {
        TriggerState {
            trigger_state: TriggerSyncState::Synced,
            sync_state: TriggerSyncState::Synced,
            last_tooth_timestamp_us: Micros::new(2_000),
            prev_tooth_interval_us: Micros::new(1_000),
            gap_fault_windows: 0,
            stall_counter: 0,
            tooth_index: 0,
            angle_deg10: Degrees10::new(0),
            rpm_estimate: Rpm::new(1_000),
        }
    }

    #[test]
    fn none_input_is_deterministic_passthrough() {
        let state = CamPhaseState {
            phase: CamPhase::PhaseB,
        };
        let trigger = synced_window_trigger();

        let step = cam_phase_step(state, trigger, None);

        assert_eq!(step.state, state);
        assert_eq!(step.phase, CamPhase::PhaseB);
        assert!(!step.edge_applied);
    }

    #[test]
    fn some_input_in_valid_window_updates_phase() {
        let state = CamPhaseState {
            phase: CamPhase::Unknown,
        };
        let trigger = synced_window_trigger();

        let step = cam_phase_step(state, trigger, Some(CamTooth::Edge));

        assert_eq!(step.state.phase, CamPhase::PhaseA);
        assert_eq!(step.phase, CamPhase::PhaseA);
        assert!(step.edge_applied);
    }

    #[test]
    fn out_of_window_cam_edge_is_ignored() {
        let state = CamPhaseState {
            phase: CamPhase::PhaseA,
        };
        let mut trigger = synced_window_trigger();
        trigger.tooth_index = 12;

        let step = cam_phase_step(state, trigger, Some(CamTooth::Edge));

        assert_eq!(step.state, state);
        assert_eq!(step.phase, CamPhase::PhaseA);
        assert!(!step.edge_applied);
    }
}
