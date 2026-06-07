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
pub enum CamEdgeAction {
    #[default]
    SetPhaseA,
    SetPhaseB,
    Toggle,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub struct CamPhaseState {
    pub phase: CamPhase,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct CamPhaseConfig {
    pub reference_tooth: u8,
    pub window_before: u8,
    pub window_after: u8,
    pub tooth_count: u8,
    pub edge_action: CamEdgeAction,
}

impl Default for CamPhaseConfig {
    fn default() -> Self {
        Self {
            reference_tooth: 0,
            window_before: 0,
            window_after: 0,
            tooth_count: 58,
            edge_action: CamEdgeAction::SetPhaseA,
        }
    }
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
    cam_phase_step_with_config(CamPhaseConfig::default(), state, trigger, cam_tooth)
}

#[must_use]
pub fn cam_phase_step_with_config(
    config: CamPhaseConfig,
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
            if is_valid_cam_window(config, trigger) {
                let next_phase = apply_edge_action(config.edge_action, state.phase);
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

fn is_valid_cam_window(config: CamPhaseConfig, trigger: TriggerState) -> bool {
    if trigger.sync_state != TriggerSyncState::Synced || config.tooth_count == 0 {
        return false;
    }

    let tooth = trigger.tooth_index % config.tooth_count;
    let reference = config.reference_tooth % config.tooth_count;
    let after = forward_distance(reference, tooth, config.tooth_count);
    let before = forward_distance(tooth, reference, config.tooth_count);

    after <= config.window_after || before <= config.window_before
}

fn forward_distance(from: u8, to: u8, modulo: u8) -> u8 {
    if to >= from {
        to - from
    } else {
        modulo - from + to
    }
}

fn apply_edge_action(action: CamEdgeAction, phase: CamPhase) -> CamPhase {
    match action {
        CamEdgeAction::SetPhaseA => CamPhase::PhaseA,
        CamEdgeAction::SetPhaseB => CamPhase::PhaseB,
        CamEdgeAction::Toggle => match phase {
            CamPhase::Unknown => CamPhase::PhaseA,
            CamPhase::PhaseA => CamPhase::PhaseB,
            CamPhase::PhaseB => CamPhase::PhaseA,
        },
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
    fn single_reference_cam_pulse_reasserts_same_phase_each_cycle() {
        let trigger = synced_window_trigger();
        let state = CamPhaseState {
            phase: CamPhase::PhaseA,
        };

        let step = cam_phase_step(state, trigger, Some(CamTooth::Edge));

        assert_eq!(step.state.phase, CamPhase::PhaseA);
        assert!(step.edge_applied);
    }

    #[test]
    fn toggle_edge_action_supports_alternating_cam_patterns() {
        let trigger = synced_window_trigger();
        let config = CamPhaseConfig {
            edge_action: CamEdgeAction::Toggle,
            ..CamPhaseConfig::default()
        };
        let state = CamPhaseState {
            phase: CamPhase::PhaseA,
        };

        let step = cam_phase_step_with_config(config, state, trigger, Some(CamTooth::Edge));

        assert_eq!(step.state.phase, CamPhase::PhaseB);
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

    #[test]
    fn configured_cam_window_accepts_edges_around_reference_tooth() {
        let state = CamPhaseState::default();
        let mut trigger = synced_window_trigger();
        trigger.tooth_index = 57;
        let config = CamPhaseConfig {
            reference_tooth: 0,
            window_before: 1,
            window_after: 1,
            tooth_count: 58,
            ..CamPhaseConfig::default()
        };

        let before = cam_phase_step_with_config(config, state, trigger, Some(CamTooth::Edge));
        trigger.tooth_index = 1;
        let after = cam_phase_step_with_config(config, state, trigger, Some(CamTooth::Edge));
        trigger.tooth_index = 2;
        let outside = cam_phase_step_with_config(config, state, trigger, Some(CamTooth::Edge));

        assert!(before.edge_applied);
        assert!(after.edge_applied);
        assert!(!outside.edge_applied);
    }

    #[test]
    fn invalid_cam_window_config_does_not_apply_edges() {
        let state = CamPhaseState::default();
        let trigger = synced_window_trigger();
        let config = CamPhaseConfig {
            tooth_count: 0,
            ..CamPhaseConfig::default()
        };

        let step = cam_phase_step_with_config(config, state, trigger, Some(CamTooth::Edge));

        assert_eq!(step.state, state);
        assert!(!step.edge_applied);
    }
}
