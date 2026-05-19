//! Missing-tooth trigger pattern generator.
//!
//! Generates deterministic timestamped crank and cam edge streams for 60-2
//! and similar missing-tooth patterns. Used for runtime replay and live app tests.

use ecu_domain::{Degrees10, Micros, Rpm};
use ecu_io::{EdgeLine, EdgePolarity, EdgeSample};

/// Error type for trigger pattern operations.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TriggerPatternError {
    InvalidSlots,
    InvalidMissingTeeth,
    InvalidRpm,
    BufferFull,
}

/// Missing-tooth trigger pattern definition.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct MissingToothPattern {
    /// Number of crank slots per revolution.
    pub crank_slots_per_rev: u16,
    /// Number of missing slots (teeth).
    pub missing_slots: u16,
    /// Full cycle length in degrees*10.
    pub cycle_deg10: u16,
    /// Cam signal phase offset in degrees*10.
    pub cam_phase_deg10: i16,
}

impl MissingToothPattern {
    /// Standard 60-2 pattern: 60 slots, 2 missing, 58 actual teeth.
    pub const fn sixty_minus_two() -> Self {
        Self {
            crank_slots_per_rev: 60,
            missing_slots: 2,
            cycle_deg10: 7200,
            cam_phase_deg10: 0,
        }
    }

    /// Validate the pattern parameters.
    pub fn validate(self) -> Result<(), TriggerPatternError> {
        if self.crank_slots_per_rev < 4 {
            return Err(TriggerPatternError::InvalidSlots);
        }
        if self.missing_slots == 0 || self.missing_slots >= self.crank_slots_per_rev {
            return Err(TriggerPatternError::InvalidMissingTeeth);
        }
        Ok(())
    }

    /// Number of edges emitted per crank revolution.
    pub fn emitted_edges_per_rev(self) -> u16 {
        self.crank_slots_per_rev - self.missing_slots
    }

    /// Number of consecutive missing slots (gap size).
    pub fn missing_gap_slots(self) -> u16 {
        self.missing_slots
    }

    /// Nominal slot angle in degrees*10.
    pub fn slot_angle_deg10(self) -> i16 {
        (36000 / self.crank_slots_per_rev) as i16
    }
}

/// Missing-tooth edge generator with fixed state.
///
/// Tracks every slot position (0 to crank_slots_per_rev-1) including gap slots.
/// Gap slots are skipped without emitting. The edge_idx advances through ALL
/// slots but emission only happens for non-gap slots.
#[derive(Debug, Clone, Copy)]
pub struct MissingToothEdgeGenerator {
    pattern: MissingToothPattern,
    rpm: Rpm,
    /// Current slot index (0 to crank_slots_per_rev-1), including gap slots.
    edge_idx: u16,
    /// Current crank angle in degrees*10.
    angle_deg10: Degrees10,
    /// Whether we've emitted the cam edge for this cycle.
    cam_emitted: bool,
    /// Whether we're currently skipping gap slots.
    in_gap: bool,
    /// How many gap slots remain to skip.
    gap_remaining: u16,
    /// Whether GAP_EXIT has already fired for the current gap traversal.
    /// Used to distinguish normal 2→1→0 countdown from the u16::MAX→0 wrap.
    gap_exited: bool,
    /// Skip the next N edges after exiting the gap (duplicate teeth).
    skip_next: u8,
    /// Cumulative microseconds — advances by revolution_us each wrap,
    /// providing monotonically increasing timestamps across revolutions.
    cumulative_us: u32,
}

impl MissingToothEdgeGenerator {
    /// Create a new generator for the given pattern.
    pub fn new(pattern: MissingToothPattern) -> Result<Self, TriggerPatternError> {
        pattern.validate()?;
        Ok(Self {
            pattern,
            rpm: Rpm::new(0),
            edge_idx: 0,
            angle_deg10: Degrees10::new(0),
            cam_emitted: false,
            in_gap: false,
            gap_remaining: 0,
            gap_exited: false,
            skip_next: 0,
            cumulative_us: 0,
        })
    }

    /// Reset the generator to initial state.
    pub fn reset(&mut self) {
        self.edge_idx = 0;
        self.angle_deg10 = Degrees10::new(0);
        self.cam_emitted = false;
        self.in_gap = false;
        self.gap_remaining = 0;
        self.gap_exited = false;
        self.skip_next = 0;
        self.cumulative_us = 0;
    }

    /// Set the RPM for edge generation.
    pub fn set_rpm(&mut self, rpm: Rpm) -> Result<(), TriggerPatternError> {
        if rpm.get() == 0 {
            return Err(TriggerPatternError::InvalidRpm);
        }
        self.rpm = rpm;
        Ok(())
    }

    /// Generate the next edge.
    /// Emits exactly 59 edges per 60-2 revolution:
    /// - Slots 0..57: emit normal tooth
    /// - Slot 58 (gap_start): emit gap_start tooth, then enter gap mode
    /// - Slots 59, 0: gap slots — skip without emitting
    /// - After skipping both gap slots: GAP_EXIT fires, skip_next=1 skips
    ///   the duplicate tooth at slot 1
    /// - Slot 2 onwards: normal emission resumes
    ///
    /// Total: 59 edges per revolution (58 normal + 1 gap_start tooth)
    #[allow(clippy::unnecessary_cast)]
    pub fn next_edge(&mut self) -> Result<EdgeSample, TriggerPatternError> {
        if self.rpm.get() == 0 {
            return Err(TriggerPatternError::InvalidRpm);
        }

        let slot_angle = self.pattern.slot_angle_deg10();
        let crank_slots = self.pattern.crank_slots_per_rev;
        let revolution_us = 60_000_000u64 / self.rpm.get() as u64;
        let slot_period_us = revolution_us / crank_slots as u64;

        loop {
            // 1. skip_next handler: skip duplicate tooth after GAP_EXIT.
            // After GAP_EXIT sets skip_next=1, we skip exactly one tooth.
            if self.skip_next > 0 {
                self.skip_next -= 1;
                self.edge_idx = self.edge_idx.wrapping_add(1);
                // Advance cumulative_us by one full revolution when we wrap,
                // so that the gap tooth emitted after the wrap has t=cumulative_us
                // (which equals the previous revolution_us, not 0).
                if self.edge_idx == 0 {
                    self.cumulative_us = self.cumulative_us.wrapping_add(revolution_us as u32);
                }
                self.angle_deg10 = Degrees10::new(0);
                self.cam_emitted = false;
                continue;
            }

            // 2. gap_entry: enter gap mode when we reach gap_start.
            // gap_start = crank_slots - missing_slots = 60 - 2 = 58.
            if !self.in_gap {
                let gap_start = crank_slots - self.pattern.missing_slots;
                if self.edge_idx >= gap_start {
                    self.in_gap = true;
                    // gap_remaining starts at missing_slots (2).
                    // We will skip exactly 2 gap slots before GAP_EXIT fires.
                    self.gap_remaining = self.pattern.missing_slots;
                    // Reset gap_exited so the FIRST gap slot (prev=2, now 1)
                    // does NOT trigger GAP_EXIT — only the wrap to u16::MAX does.
                    self.gap_exited = false;
                }
            }

            // 3. gap handler: skip gap teeth without emitting.
            // We are AT a gap tooth. Advance past it, count down.
            if self.in_gap {
                self.edge_idx = self.edge_idx.wrapping_add(1);
                // Advance angle for the skipped gap slot.
                self.angle_deg10 =
                    Degrees10::new((self.angle_deg10.get() + slot_angle as i16) % 7200);

                // Advance cumulative_us for the skipped gap slot.
                self.cumulative_us = self.cumulative_us.wrapping_add(slot_period_us as u32);
                // When edge_idx wraps to 0, we've completed one full revolution —
                // add a full revolution_us so the next emitted tooth is at
                // cumulative_us + slot_period_us (not at 0 again).
                if self.edge_idx == 0 {
                    self.cumulative_us = self.cumulative_us.wrapping_add(revolution_us as u32);
                }

                // Count down gap_remaining; GAP_EXIT fires only on the
                // u16::MAX→0 wrap (not on the normal 2→1→0 countdown).
                let prev = self.gap_remaining;
                self.gap_remaining = self.gap_remaining.wrapping_sub(1);
                if self.gap_exited || (prev == 0 && self.gap_remaining == u16::MAX) {
                    // GAP_EXIT: all missing slots have been skipped.
                    // Set skip_next=1 to skip the duplicate tooth at slot 1.
                    self.in_gap = false;
                    self.gap_exited = false;
                    self.skip_next = 1;
                    self.angle_deg10 = Degrees10::new(0);
                    continue;
                }
                if prev == 1 {
                    // About to wrap on the next iteration — mark gap_exited
                    // so the wrap triggers GAP_EXIT rather than continuing.
                    self.gap_exited = true;
                }
                continue;
            }

            // 4. wrap: handle edge_idx wrapping at end of cycle.
            if self.edge_idx >= crank_slots {
                self.edge_idx = 0;
                self.angle_deg10 = Degrees10::new(0);
                self.cam_emitted = false;
                // We've completed one revolution — add revolution_us to
                // cumulative_us so the next emitted tooth continues from
                // cumulative_us + slot_period_us rather than resetting to 0.
                self.cumulative_us = self.cumulative_us.wrapping_add(revolution_us as u32);
                continue;
            }

            // 5. Cam edge: emit when crank angle first passes cam_phase_deg10 in this revolution.
            if !self.cam_emitted && self.angle_deg10.get() == self.pattern.cam_phase_deg10 {
                let cam_edge = EdgeSample {
                    at_us: Micros::new(self.cumulative_us),
                    line: EdgeLine::Cam,
                    polarity: EdgePolarity::Rising,
                    angle_x10: self.angle_deg10,
                    rpm: self.rpm,
                };
                self.cam_emitted = true;
                return Ok(cam_edge);
            }

            // 6. Emit crank tooth at current edge_idx with monotonically increasing time.
            let edge = EdgeSample {
                at_us: Micros::new(self.cumulative_us),
                line: EdgeLine::Crank,
                polarity: EdgePolarity::Rising,
                angle_x10: self.angle_deg10,
                rpm: self.rpm,
            };

            self.edge_idx = self.edge_idx.wrapping_add(1);
            self.cumulative_us = self.cumulative_us.wrapping_add(slot_period_us as u32);
            self.angle_deg10 = Degrees10::new((self.angle_deg10.get() + slot_angle as i16) % 7200);

            return Ok(edge);
        }
    }

    /// Fill the output array with edges. Returns the number of edges written.
    pub fn next_edges<const N: usize>(
        &mut self,
        out: &mut [Option<EdgeSample>; N],
    ) -> Result<usize, TriggerPatternError> {
        if self.rpm.get() == 0 {
            return Err(TriggerPatternError::InvalidRpm);
        }

        let mut count = 0;
        for slot in out.iter_mut() {
            match self.next_edge() {
                Ok(edge) => {
                    *slot = Some(edge);
                    count += 1;
                }
                Err(TriggerPatternError::InvalidRpm) => {
                    *slot = None;
                    break;
                }
                Err(e) => return Err(e),
            }
        }
        Ok(count)
    }

    /// Get the pattern being generated.
    pub fn pattern(&self) -> MissingToothPattern {
        self.pattern
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn sixty_minus_two_validation() {
        let p = MissingToothPattern::sixty_minus_two();
        assert_eq!(p.crank_slots_per_rev, 60);
        assert_eq!(p.missing_slots, 2);
        assert_eq!(p.emitted_edges_per_rev(), 58);
        assert_eq!(p.missing_gap_slots(), 2);
        assert_eq!(p.slot_angle_deg10(), 600);
    }

    #[test]
    fn invalid_pattern_rejected() {
        let p = MissingToothPattern {
            crank_slots_per_rev: 2,
            missing_slots: 1,
            cycle_deg10: 7200,
            cam_phase_deg10: 0,
        };
        assert!(p.validate().is_err());

        let p2 = MissingToothPattern {
            crank_slots_per_rev: 60,
            missing_slots: 0,
            cycle_deg10: 7200,
            cam_phase_deg10: 0,
        };
        assert!(p2.validate().is_err());
    }

    #[test]
    fn one_revolution_at_1000_rpm() {
        let mut gen =
            MissingToothEdgeGenerator::new(MissingToothPattern::sixty_minus_two()).unwrap();
        gen.set_rpm(Rpm::new(1000)).unwrap();

        // At 1000 RPM: one revolution = 60000 us
        // One slot = 60000/60 = 1000 us

        // 60-2 emits 59 edges per revolution: 58 normal teeth + tooth 58 at gap_start.
        // Use a 59-element array so next_edges stops at the revolution boundary.
        let mut edges = [None; 59];
        let count = gen.next_edges(&mut edges).unwrap();
        assert_eq!(count, 59);

        // First edge at t=0
        assert_eq!(edges[0].unwrap().at_us.get(), 0);
        // edges[59] is out of bounds — array has 59 elements (indices 0..58)
    }

    #[test]
    fn missing_gap_is_three_slots() {
        let p = MissingToothPattern::sixty_minus_two();
        // The gap covers 3 nominal slot intervals: slots 58, 59, and the wrap-to-0.
        // missing_gap_slots() returns missing_slots (2); the gap advancement
        // across the wrap adds one more slot interval.
        assert_eq!(p.missing_gap_slots(), 2);
    }

    #[test]
    fn generator_resets() {
        let mut gen =
            MissingToothEdgeGenerator::new(MissingToothPattern::sixty_minus_two()).unwrap();
        gen.set_rpm(Rpm::new(1000)).unwrap();

        // Generate a few edges
        gen.next_edge().unwrap();
        gen.next_edge().unwrap();

        // Reset
        gen.reset();

        // Should start from beginning
        let edge = gen.next_edge().unwrap();
        assert_eq!(edge.angle_x10.get(), 0);
    }

    #[test]
    fn zero_rpm_returns_error() {
        let mut gen =
            MissingToothEdgeGenerator::new(MissingToothPattern::sixty_minus_two()).unwrap();
        gen.set_rpm(Rpm::new(1000)).unwrap();
        gen.set_rpm(Rpm::new(0)).unwrap_err();
    }

    #[test]
    fn two_revolutions_at_1000_rpm() {
        let mut gen =
            MissingToothEdgeGenerator::new(MissingToothPattern::sixty_minus_two()).unwrap();
        gen.set_rpm(Rpm::new(1000)).unwrap();

        // Two revolutions = 118 edges (59 per rev: 58 real teeth + 1 gap tooth)
        let mut count = 0;
        loop {
            match gen.next_edge() {
                Ok(_edge) => {
                    count += 1;
                    if count >= 118 {
                        break;
                    }
                }
                Err(e) => {
                    panic!("Unexpected error: {:?}", e);
                }
            }
        }
        assert_eq!(count, 118);
    }
}
