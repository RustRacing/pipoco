//! Minimal diagnostics (DTC-like) tracking for sensor range faults and cam status.

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum DiagCode {
    MapRange,
    TpsRange,
    CamMissing,
    LowVoltage,
    Overvoltage,
    MapFailureHighLoad,
    TpsMapPlausibility,
    KnockDetected,
}

#[derive(Copy, Clone, Debug)]
pub struct DiagState {
    pub active: bool,
    pub start_us: u32,
    pub in_range_since_us: u32,
    pub total_us: u32,
}

impl DiagState {
    pub const fn new() -> Self { Self { active: false, start_us: 0, in_range_since_us: 0, total_us: 0 } }
}

impl Default for DiagState {
    fn default() -> Self { Self::new() }
}

#[derive(Copy, Clone, Debug)]
pub struct DiagEvent { pub code: DiagCode, pub start_us: u32, pub end_us: u32 }

pub struct DiagLog<const N: usize> {
    pub events: [Option<DiagEvent>; N],
    pub head: u8,
}

impl<const N: usize> DiagLog<N> {
    pub const fn new() -> Self { Self { events: [None; N], head: 0 } }
    pub fn push(&mut self, ev: DiagEvent) { let idx = (self.head as usize) % N; self.events[idx] = Some(ev); self.head = self.head.wrapping_add(1); }
}

impl<const N: usize> Default for DiagLog<N> {
    fn default() -> Self { Self::new() }
}
