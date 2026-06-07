/// RPM below this is treated as cranking (entry threshold).
pub const CRANKING_RPM_THRESHOLD: u16 = 500;
/// RPM at or above this exits cranking (hysteresis exit threshold).
pub const CRANKING_EXIT_RPM: u16 = 600;

/// Cranking gate with simple hysteresis to avoid flapping around the threshold.
#[derive(Debug, Clone, Copy)]
pub struct CrankingGate {
    cranking: bool,
}

impl CrankingGate {
    pub const fn new() -> Self {
        Self { cranking: false }
    }

    /// Update internal state based on current RPM and return `true` if cranking.
    pub fn update(&mut self, rpm: u16) -> bool {
        if self.cranking {
            if rpm >= CRANKING_EXIT_RPM {
                self.cranking = false;
            }
        } else if rpm < CRANKING_RPM_THRESHOLD {
            self.cranking = true;
        }
        self.cranking
    }

    pub fn is_cranking(&self) -> bool {
        self.cranking
    }
}

impl Default for CrankingGate {
    fn default() -> Self {
        Self::new()
    }
}
