use ecu_domain::{Degrees10, Rpm};

/// Minimal live trigger update consumed by board sensor live sinks.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SplitLiveTriggerEvent {
    rpm: Rpm,
    angle_x10: Degrees10,
    synced: bool,
}

impl SplitLiveTriggerEvent {
    pub const fn new(rpm: Rpm, angle_x10: Degrees10, synced: bool) -> Self {
        Self {
            rpm,
            angle_x10,
            synced,
        }
    }

    pub const fn rpm(self) -> Rpm {
        self.rpm
    }

    pub const fn angle_x10(self) -> Degrees10 {
        self.angle_x10
    }

    pub const fn synced(self) -> bool {
        self.synced
    }
}

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

    pub fn apply_event(&mut self, event: SplitLiveTriggerEvent) {
        self.rpm = event.rpm();
        self.angle_x10 = event.angle_x10();
        self.trigger_synced = event.synced();
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

    #[test]
    fn live_inputs_start_unsynced_and_zeroed() {
        let inputs = SplitLiveInputs::new();

        assert_eq!(inputs.rpm(), Rpm::new(0));
        assert_eq!(inputs.angle_x10(), Degrees10::new(0));
        assert!(!inputs.trigger_synced());
    }

    #[test]
    fn live_inputs_update_from_trigger_event_contract() {
        let mut inputs = SplitLiveInputs::new();

        inputs.apply_event(SplitLiveTriggerEvent::new(
            Rpm::new(1_500),
            Degrees10::new(120),
            true,
        ));

        assert_eq!(inputs.rpm(), Rpm::new(1_500));
        assert_eq!(inputs.angle_x10(), Degrees10::new(120));
        assert!(inputs.trigger_synced());
    }
}
