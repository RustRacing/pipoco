#![allow(dead_code)]
//! Output capture for analyzing ECU behavior
//!
//! Records injector and ignition events for post-simulation analysis.

/// Event type recorded by output capture
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum OutputEvent {
    InjectionStart,
    InjectionEnd,
    IgnitionChargeStart,
    IgnitionFire,
}

/// Captured output event with timing
#[derive(Debug, Clone)]
pub struct CapturedEvent {
    pub time_us: u32,
    pub channel: u8,
    pub event: OutputEvent,
    pub duration_us: Option<u16>, // For injection pulse width or dwell time
}

/// Output capture for recording ECU outputs
pub struct OutputCapture {
    events: Vec<CapturedEvent>,
    injection_pulse_widths: Vec<u16>,
}

impl OutputCapture {
    /// Create new output capture
    pub fn new() -> Self {
        Self {
            events: Vec::new(),
            injection_pulse_widths: Vec::new(),
        }
    }

    /// Record an injection event
    pub fn record_injection(&mut self, time_us: u32, pulse_width_us: u16) {
        self.events.push(CapturedEvent {
            time_us,
            channel: 0, // Channel doesn't matter for MVP batch injection
            event: OutputEvent::InjectionStart,
            duration_us: Some(pulse_width_us),
        });

        self.injection_pulse_widths.push(pulse_width_us);
    }

    /// Record an ignition event
    pub fn record_ignition(&mut self, time_us: u32, dwell_us: u16) {
        self.events.push(CapturedEvent {
            time_us,
            channel: 0,
            event: OutputEvent::IgnitionFire,
            duration_us: Some(dwell_us),
        });
    }

    /// Get all captured events
    pub fn events(&self) -> &[CapturedEvent] {
        &self.events
    }

    /// Get injection count
    pub fn injection_count(&self) -> usize {
        self.injection_pulse_widths.len()
    }

    /// Get average injection pulse width
    pub fn average_injection_pw(&self) -> u16 {
        if self.injection_pulse_widths.is_empty() {
            return 0;
        }

        let sum: u32 = self.injection_pulse_widths.iter().map(|&x| x as u32).sum();
        (sum / self.injection_pulse_widths.len() as u32) as u16
    }

    /// Get minimum injection pulse width
    pub fn min_injection_pw(&self) -> Option<u16> {
        self.injection_pulse_widths.iter().copied().min()
    }

    /// Get maximum injection pulse width
    pub fn max_injection_pw(&self) -> Option<u16> {
        self.injection_pulse_widths.iter().copied().max()
    }

    /// Get injection pulse width at specific index
    pub fn injection_pw_at(&self, index: usize) -> Option<u16> {
        self.injection_pulse_widths.get(index).copied()
    }

    /// Clear all captured events
    pub fn clear(&mut self) {
        self.events.clear();
        self.injection_pulse_widths.clear();
    }

    /// Get event count by type
    pub fn event_count(&self, event_type: OutputEvent) -> usize {
        self.events.iter().filter(|e| e.event == event_type).count()
    }
}

impl Default for OutputCapture {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_output_capture_creation() {
        let capture = OutputCapture::new();
        assert_eq!(capture.injection_count(), 0);
        assert_eq!(capture.events().len(), 0);
    }

    #[test]
    fn test_record_injection() {
        let mut capture = OutputCapture::new();

        capture.record_injection(1000, 1500);
        capture.record_injection(2000, 1600);

        assert_eq!(capture.injection_count(), 2);
        assert_eq!(capture.average_injection_pw(), 1550);
    }

    #[test]
    fn test_injection_statistics() {
        let mut capture = OutputCapture::new();

        capture.record_injection(1000, 1000);
        capture.record_injection(2000, 1500);
        capture.record_injection(3000, 2000);

        assert_eq!(capture.injection_count(), 3);
        assert_eq!(capture.average_injection_pw(), 1500);
        assert_eq!(capture.min_injection_pw(), Some(1000));
        assert_eq!(capture.max_injection_pw(), Some(2000));
    }

    #[test]
    fn test_clear() {
        let mut capture = OutputCapture::new();

        capture.record_injection(1000, 1500);
        assert_eq!(capture.injection_count(), 1);

        capture.clear();
        assert_eq!(capture.injection_count(), 0);
        assert_eq!(capture.events().len(), 0);
    }

    #[test]
    fn test_event_count_by_type() {
        let mut capture = OutputCapture::new();

        capture.record_injection(1000, 1500);
        capture.record_injection(2000, 1500);
        capture.record_ignition(1500, 3000);

        assert_eq!(capture.event_count(OutputEvent::InjectionStart), 2);
        assert_eq!(capture.event_count(OutputEvent::IgnitionFire), 1);
    }

    #[test]
    fn test_injection_pw_at() {
        let mut capture = OutputCapture::new();

        capture.record_injection(1000, 1500);
        capture.record_injection(2000, 1600);

        assert_eq!(capture.injection_pw_at(0), Some(1500));
        assert_eq!(capture.injection_pw_at(1), Some(1600));
        assert_eq!(capture.injection_pw_at(2), None);
    }
}
