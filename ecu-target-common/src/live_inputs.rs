use crate::adapter::BoardEvent;
use ecu_domain::{Degrees10, Rpm};

/// Last live engine inputs shared by board sensor and control-input plumbing.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SplitLiveInputs {
    rpm: Rpm,
    angle_x10: Degrees10,
    trigger_synced: bool,
}

impl SplitLiveInputs {
    pub const fn new() -> Self {
        Self {
            rpm: Rpm::new(0),
            angle_x10: Degrees10::new(0),
            trigger_synced: false,
        }
    }

    pub fn apply_event(&mut self, event: BoardEvent) {
        if let BoardEvent::TriggerEdge {
            rpm,
            angle_x10,
            synced,
            ..
        } = event
        {
            self.rpm = rpm;
            self.angle_x10 = angle_x10;
            self.trigger_synced = synced;
        }
    }

    pub const fn rpm(self) -> Rpm {
        self.rpm
    }

    pub const fn angle_x10(self) -> Degrees10 {
        self.angle_x10
    }

    pub const fn trigger_synced(self) -> bool {
        self.trigger_synced
    }
}

impl Default for SplitLiveInputs {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use ecu_domain::Micros;

    #[test]
    fn live_inputs_start_unsynced_and_zeroed() {
        let inputs = SplitLiveInputs::new();

        assert_eq!(inputs.rpm(), Rpm::new(0));
        assert_eq!(inputs.angle_x10(), Degrees10::new(0));
        assert!(!inputs.trigger_synced());
    }

    #[test]
    fn live_inputs_update_from_trigger_event_only() {
        let mut inputs = SplitLiveInputs::new();

        inputs.apply_event(BoardEvent::CamEdge {
            at_us: Micros::new(10),
            cam_seen: true,
        });
        assert_eq!(inputs, SplitLiveInputs::new());

        inputs.apply_event(BoardEvent::TriggerEdge {
            at_us: Micros::new(20),
            rpm: Rpm::new(1_500),
            angle_x10: Degrees10::new(120),
            authority: ecu_domain::EngineTimeAuthority::none(),
            synced: true,
        });

        assert_eq!(inputs.rpm(), Rpm::new(1_500));
        assert_eq!(inputs.angle_x10(), Degrees10::new(120));
        assert!(inputs.trigger_synced());
    }
}
