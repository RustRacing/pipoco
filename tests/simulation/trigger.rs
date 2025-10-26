//! Trigger wheel signal generator
//!
//! Generates realistic trigger wheel edges based on engine RPM and position.

use super::{EngineSimulator, TriggerPattern};

/// Trigger wheel signal generator
pub struct TriggerGenerator {
    pattern: TriggerPattern,
    last_tooth: u8,
    last_edge_time: u32,
    next_tooth_time: u32,
}

impl TriggerGenerator {
    /// Create new trigger generator
    pub fn new(pattern: TriggerPattern) -> Self {
        Self {
            pattern,
            last_tooth: 0,
            last_edge_time: 0,
            next_tooth_time: 0,
        }
    }

    /// Get the number of teeth for this pattern
    fn teeth_count(&self) -> u8 {
        match self.pattern {
            TriggerPattern::SixtyMinusTwo => 58,
            TriggerPattern::ThirtySixMinusOne => 35,
            TriggerPattern::TwentyFourMinusOne => 23,
        }
    }

    /// Check if we're at the missing tooth gap
    fn is_gap(&self, tooth: u8) -> bool {
        match self.pattern {
            TriggerPattern::SixtyMinusTwo => tooth == 0,  // After tooth 58
            TriggerPattern::ThirtySixMinusOne => tooth == 0,
            TriggerPattern::TwentyFourMinusOne => tooth == 0,
        }
    }

    /// Calculate next trigger edge time based on engine state
    ///
    /// Returns Some(time) if an edge should occur, None if not yet
    pub fn next_edge(&mut self, engine: &EngineSimulator, current_time: u32) -> Option<u32> {
        let rpm = engine.state().rpm;

        if rpm == 0 {
            // Engine stopped, no edges
            return None;
        }

        // Calculate tooth period
        let tooth_period = engine.tooth_period_us();
        if tooth_period == 0 {
            return None;
        }

        // Initialize next tooth time on first call
        if self.next_tooth_time == 0 {
            self.next_tooth_time = current_time + tooth_period;
            self.last_edge_time = current_time;
        }

        // Check if it's time for the next tooth
        if current_time >= self.next_tooth_time {
            // Advance to next tooth
            self.last_tooth = (self.last_tooth + 1) % (self.teeth_count() + 1);

            // Calculate time until next tooth
            let period = if self.is_gap(self.last_tooth) {
                // Missing tooth gap - 2x normal period
                tooth_period * 2
            } else {
                tooth_period
            };

            self.last_edge_time = self.next_tooth_time;
            self.next_tooth_time = self.next_tooth_time.wrapping_add(period);

            return Some(self.last_edge_time);
        }

        None
    }

    /// Reset trigger generator (for engine stop/start)
    pub fn reset(&mut self) {
        self.last_tooth = 0;
        self.last_edge_time = 0;
        self.next_tooth_time = 0;
    }

    /// Get current tooth position
    pub fn current_tooth(&self) -> u8 {
        self.last_tooth
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use super::super::EngineConfig;

    #[test]
    fn test_trigger_generator_creation() {
        let gen = TriggerGenerator::new(TriggerPattern::SixtyMinusTwo);
        assert_eq!(gen.current_tooth(), 0);
    }

    #[test]
    fn test_teeth_count() {
        let gen = TriggerGenerator::new(TriggerPattern::SixtyMinusTwo);
        assert_eq!(gen.teeth_count(), 58);
    }

    #[test]
    fn test_gap_detection() {
        let gen = TriggerGenerator::new(TriggerPattern::SixtyMinusTwo);
        assert!(gen.is_gap(0));  // After tooth 58 comes gap
        assert!(!gen.is_gap(1));
        assert!(!gen.is_gap(30));
        assert!(!gen.is_gap(58));
    }

    #[test]
    fn test_edge_generation_at_1000_rpm() {
        let config = EngineConfig::default();
        let mut engine = EngineSimulator::new(config);
        engine.state_mut().rpm = 1000;

        let mut gen = TriggerGenerator::new(TriggerPattern::SixtyMinusTwo);

        let mut current_time = 0u32;
        let mut edge_count = 0;

        // Simulate for 120ms (should get 2 full revolutions)
        while current_time < 120_000 {
            if let Some(_edge_time) = gen.next_edge(&engine, current_time) {
                edge_count += 1;
            }
            current_time += 10;  // Advance by 10us
        }

        // At 1000 RPM, one revolution = 60ms
        // 120ms = 2 revolutions = 116 teeth (58 * 2)
        assert!(edge_count >= 110 && edge_count <= 120,
                "Expected ~116 edges, got {}", edge_count);
    }

    #[test]
    fn test_missing_tooth_gap() {
        let config = EngineConfig::default();
        let mut engine = EngineSimulator::new(config);
        engine.state_mut().rpm = 1000;

        let mut gen = TriggerGenerator::new(TriggerPattern::SixtyMinusTwo);

        let mut current_time = 0u32;
        let mut tooth_times = Vec::new();

        // Collect first 60 edge times
        while tooth_times.len() < 60 {
            if let Some(edge_time) = gen.next_edge(&engine, current_time) {
                tooth_times.push(edge_time);
            }
            current_time += 10;
        }

        // Calculate periods between edges
        let mut periods = Vec::new();
        for i in 1..tooth_times.len() {
            periods.push(tooth_times[i] - tooth_times[i-1]);
        }

        // Find gaps (should be roughly 2x normal period)
        let avg_period = periods.iter().filter(|&&p| p < 1500).sum::<u32>() / 57;
        let gaps: Vec<_> = periods.iter()
            .enumerate()
            .filter(|(_, &p)| p > avg_period * 3 / 2)
            .collect();

        // Should have exactly one gap per revolution (58 teeth)
        assert_eq!(gaps.len(), 1, "Should have 1 gap, found {}", gaps.len());

        // Gap should be roughly 2x normal period
        let gap_period = *gaps[0].1;
        assert!(gap_period > avg_period * 3 / 2,
                "Gap period {} should be > 1.5x normal period {}", gap_period, avg_period);
    }

    #[test]
    fn test_reset() {
        let config = EngineConfig::default();
        let mut engine = EngineSimulator::new(config);
        engine.state_mut().rpm = 1000;

        let mut gen = TriggerGenerator::new(TriggerPattern::SixtyMinusTwo);

        // Generate some edges
        let mut current_time = 0u32;
        for _ in 0..10 {
            let _ = gen.next_edge(&engine, current_time);
            current_time += 1000;
        }

        assert!(gen.current_tooth() > 0);

        // Reset
        gen.reset();
        assert_eq!(gen.current_tooth(), 0);
    }
}
