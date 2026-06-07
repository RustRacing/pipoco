//! Legacy-free plant-bridge types shared by x86 board implementations.
//!
//! These types describe translated ECU output frames and bridge diagnostics
//! without depending on the legacy application path.

/// Diagnostics emitted by the x86 plant bridge.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct X86PlantBridgeDiagnostics {
    pub duplicate_high_count: u32,
    pub orphan_low_count: u32,
    pub open_high_count: u32,
    pub unmapped_channel_count: u32,
    pub out_of_range_channel_count: u32,
    pub out_of_order_transition_count: u32,
    pub short_pulse_width_count: u32,
    pub short_dwell_count: u32,
    pub capacity_overflow_count: u32,
}

impl X86PlantBridgeDiagnostics {
    pub const fn empty() -> Self {
        Self {
            duplicate_high_count: 0,
            orphan_low_count: 0,
            open_high_count: 0,
            unmapped_channel_count: 0,
            out_of_range_channel_count: 0,
            out_of_order_transition_count: 0,
            short_pulse_width_count: 0,
            short_dwell_count: 0,
            capacity_overflow_count: 0,
        }
    }

    pub const fn is_clean(self) -> bool {
        self.duplicate_high_count == 0
            && self.orphan_low_count == 0
            && self.open_high_count == 0
            && self.unmapped_channel_count == 0
            && self.out_of_range_channel_count == 0
            && self.out_of_order_transition_count == 0
            && self.short_pulse_width_count == 0
            && self.short_dwell_count == 0
            && self.capacity_overflow_count == 0
    }
}
