use ecu_domain::{Kpa10, PulseWidthUs, Rpm};

/// First-pass base fuel model backed by an IPW table.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct BaseFuelModel {
    rpm_bins: [Rpm; 16],
    load_bins: [Kpa10; 16],
    pulse_widths: [[PulseWidthUs; 16]; 16],
}

impl BaseFuelModel {
    pub const fn new(
        rpm_bins: [Rpm; 16],
        load_bins: [Kpa10; 16],
        pulse_widths: [[PulseWidthUs; 16]; 16],
    ) -> Self {
        Self {
            rpm_bins,
            load_bins,
            pulse_widths,
        }
    }

    pub const fn rpm_bins(self) -> [Rpm; 16] {
        self.rpm_bins
    }

    pub const fn load_bins(self) -> [Kpa10; 16] {
        self.load_bins
    }

    pub const fn pulse_widths(self) -> [[PulseWidthUs; 16]; 16] {
        self.pulse_widths
    }

    pub fn calculate_base_fuel(&self, rpm: Rpm, load: Kpa10) -> PulseWidthUs {
        let rpm_idx = self.nearest_index_rpm(rpm);
        let load_idx = self.nearest_index_load(load);
        self.pulse_widths[load_idx][rpm_idx]
    }

    fn nearest_index_rpm(&self, value: Rpm) -> usize {
        nearest_index_rpm(&self.rpm_bins, value)
    }

    fn nearest_index_load(&self, value: Kpa10) -> usize {
        nearest_index_kpa10(&self.load_bins, value)
    }
}

impl Default for BaseFuelModel {
    fn default() -> Self {
        Self {
            rpm_bins: [Rpm::new(0); 16],
            load_bins: [Kpa10::new(0); 16],
            pulse_widths: [[PulseWidthUs::new(0); 16]; 16],
        }
    }
}

fn nearest_index_rpm(bins: &[Rpm; 16], value: Rpm) -> usize {
    let mut idx = 0usize;
    while idx < 15 {
        if value.get() < bins[idx + 1].get() {
            return idx;
        }
        idx += 1;
    }
    15
}

fn nearest_index_kpa10(bins: &[Kpa10; 16], value: Kpa10) -> usize {
    let mut idx = 0usize;
    while idx < 15 {
        if value.get() < bins[idx + 1].get() {
            return idx;
        }
        idx += 1;
    }
    15
}
