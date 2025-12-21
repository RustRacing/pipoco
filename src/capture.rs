//! Lightweight capture buffers for ECU diagnostics
//!
//! Provides:
//! - Simple timestamp ring buffer for ISR-to-main communication
//! - Event-triggered capture with pre/post-trigger data

// ============================================================================
// Basic Timestamp Capture (existing)
// ============================================================================

#[derive(Debug)]
pub struct CaptureBuffer<const N: usize> {
    buf: [u32; N],
    head: u8,
    tail: u8,
}

impl<const N: usize> CaptureBuffer<N> {
    pub const fn new() -> Self {
        Self {
            buf: [0; N],
            head: 0,
            tail: 0,
        }
    }

    /// Push a timestamp if space is available (drops on overflow)
    pub fn push(&mut self, ts: u32) {
        let next = self.head.wrapping_add(1);
        if next != self.tail {
            self.buf[self.head as usize] = ts;
            self.head = next;
        }
    }

    /// Pop a timestamp if available
    pub fn try_pop(&mut self) -> Option<u32> {
        if self.tail == self.head {
            None
        } else {
            let v = self.buf[self.tail as usize];
            self.tail = self.tail.wrapping_add(1);
            Some(v)
        }
    }
}

impl<const N: usize> Default for CaptureBuffer<N> {
    fn default() -> Self {
        Self::new()
    }
}

// ============================================================================
// Event-Triggered Capture System
// ============================================================================

/// Trigger sources for event capture
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum CaptureTrigger {
    /// Manual trigger via TunerStudio or command
    Manual = 0,
    /// Trigger sync was lost
    SyncLoss = 1,
    /// Knock event detected
    KnockDetected = 2,
    /// Sudden RPM change (stall or spike)
    RpmSpike = 3,
    /// Any sensor fault (out of range, implausible)
    SensorFault = 4,
    /// Entered limp mode
    LimpEntry = 5,
    /// Lambda sensor fault or oscillation
    LambdaFault = 6,
    /// Low voltage / brown-out
    LowVoltage = 7,
}

impl CaptureTrigger {
    /// Get bitmask for this trigger
    pub const fn mask(self) -> u8 {
        1 << (self as u8)
    }
}

/// Bitmask of enabled triggers
pub mod trigger_mask {
    use super::CaptureTrigger;

    pub const MANUAL: u8 = CaptureTrigger::Manual.mask();
    pub const SYNC_LOSS: u8 = CaptureTrigger::SyncLoss.mask();
    pub const KNOCK: u8 = CaptureTrigger::KnockDetected.mask();
    pub const RPM_SPIKE: u8 = CaptureTrigger::RpmSpike.mask();
    pub const SENSOR_FAULT: u8 = CaptureTrigger::SensorFault.mask();
    pub const LIMP_ENTRY: u8 = CaptureTrigger::LimpEntry.mask();
    pub const LAMBDA_FAULT: u8 = CaptureTrigger::LambdaFault.mask();
    pub const LOW_VOLTAGE: u8 = CaptureTrigger::LowVoltage.mask();
    pub const ALL: u8 = 0xFF;
    pub const SAFETY_ONLY: u8 = SYNC_LOSS | SENSOR_FAULT | LIMP_ENTRY | LOW_VOLTAGE;
}

/// Configuration for triggered capture
#[derive(Debug, Clone, Copy)]
pub struct CaptureConfig {
    /// Enable triggered capture
    pub enable: bool,
    /// Number of samples to keep before trigger (ring buffer size)
    pub pre_trigger_samples: u16,
    /// Number of samples to capture after trigger
    pub post_trigger_samples: u16,
    /// Sample interval in microseconds
    pub sample_interval_us: u32,
    /// Bitmask of enabled trigger sources
    pub enabled_triggers: u8,
    /// RPM change threshold for RpmSpike trigger (RPM/second)
    pub rpm_spike_threshold: u16,
}

impl CaptureConfig {
    pub const DEFAULT: Self = Self {
        enable: true,
        pre_trigger_samples: 50,
        post_trigger_samples: 100,
        sample_interval_us: 10_000, // 10ms = 100Hz
        enabled_triggers: trigger_mask::SAFETY_ONLY,
        rpm_spike_threshold: 2000, // 2000 RPM/second
    };

    /// Check if a trigger source is enabled
    pub fn is_trigger_enabled(&self, trigger: CaptureTrigger) -> bool {
        self.enable && (self.enabled_triggers & trigger.mask()) != 0
    }
}

impl Default for CaptureConfig {
    fn default() -> Self {
        Self::DEFAULT
    }
}

/// A single captured frame of engine data
#[derive(Debug, Clone, Copy, Default)]
pub struct CaptureFrame {
    /// Timestamp in microseconds
    pub timestamp_us: u32,
    /// Engine RPM
    pub rpm: u16,
    /// MAP reading (kPa x10)
    pub map_kpa_x10: u16,
    /// TPS reading (0-100%)
    pub tps_percent: u8,
    /// Battery voltage (millivolts / 100)
    pub voltage_mv_div100: u8,
    /// Short-term fuel trim (percent + 128, so 128 = 0%)
    pub stft_offset: u8,
    /// Status flags (sync, limp, knock, etc.)
    pub status: u8,
}

impl CaptureFrame {
    pub const fn new() -> Self {
        Self {
            timestamp_us: 0,
            rpm: 0,
            map_kpa_x10: 0,
            tps_percent: 0,
            voltage_mv_div100: 0,
            stft_offset: 128, // 0%
            status: 0,
        }
    }

    /// Create a frame from current engine state
    pub fn from_state(
        timestamp_us: u32,
        rpm: u16,
        map_kpa_x10: u16,
        tps_percent: u8,
        voltage_mv: u16,
        stft_x10: i16,
        synced: bool,
        limp: bool,
        knock: bool,
    ) -> Self {
        let voltage_scaled = (voltage_mv / 100).min(255) as u8;
        let stft_scaled = ((stft_x10 / 10) + 128).clamp(0, 255) as u8;

        let mut status = 0u8;
        if synced { status |= 0x01; }
        if limp { status |= 0x02; }
        if knock { status |= 0x04; }

        Self {
            timestamp_us,
            rpm,
            map_kpa_x10,
            tps_percent,
            voltage_mv_div100: voltage_scaled,
            stft_offset: stft_scaled,
            status,
        }
    }

    /// Size in bytes (for serialization)
    pub const SIZE: usize = 12;

    /// Serialize to bytes
    pub fn to_bytes(&self) -> [u8; Self::SIZE] {
        let mut out = [0u8; Self::SIZE];
        out[0..4].copy_from_slice(&self.timestamp_us.to_le_bytes());
        out[4..6].copy_from_slice(&self.rpm.to_le_bytes());
        out[6..8].copy_from_slice(&self.map_kpa_x10.to_le_bytes());
        out[8] = self.tps_percent;
        out[9] = self.voltage_mv_div100;
        out[10] = self.stft_offset;
        out[11] = self.status;
        out
    }
}

/// State of the triggered capture system
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CaptureState {
    /// Idle, waiting for trigger (ring buffer filling)
    Armed,
    /// Trigger fired, capturing post-trigger samples
    Capturing,
    /// Capture complete, data ready to download
    Complete,
    /// Disabled
    Disabled,
}

/// Event-triggered capture with pre/post-trigger data
#[derive(Debug)]
pub struct TriggeredCapture<const N: usize> {
    /// Ring buffer for samples
    buffer: [CaptureFrame; N],
    /// Current write position (wraps around)
    head: usize,
    /// Number of valid samples in buffer
    count: usize,
    /// Current state
    pub state: CaptureState,
    /// Trigger reason (if triggered)
    pub trigger_reason: Option<CaptureTrigger>,
    /// Index where trigger occurred
    trigger_index: usize,
    /// Samples remaining after trigger
    post_remaining: u16,
    /// Last sample timestamp
    last_sample_us: u32,
    /// Last RPM (for spike detection)
    last_rpm: u16,
    /// Configuration
    pub config: CaptureConfig,
}

impl<const N: usize> TriggeredCapture<N> {
    pub const fn new() -> Self {
        Self {
            buffer: [CaptureFrame::new(); N],
            head: 0,
            count: 0,
            state: CaptureState::Armed,
            trigger_reason: None,
            trigger_index: 0,
            post_remaining: 0,
            last_sample_us: 0,
            last_rpm: 0,
            config: CaptureConfig::DEFAULT,
        }
    }

    /// Add a sample to the capture buffer
    ///
    /// In Armed state: samples are added to ring buffer (old samples overwritten)
    /// In Capturing state: samples are added until post_remaining reaches 0
    /// In Complete/Disabled state: samples are ignored
    ///
    /// # Arguments
    /// * `frame` - The sample to add
    /// * `now_us` - Current timestamp
    pub fn sample(&mut self, frame: CaptureFrame, now_us: u32) {
        // Check sample interval
        let elapsed = now_us.wrapping_sub(self.last_sample_us);
        if self.last_sample_us != 0 && elapsed < self.config.sample_interval_us {
            return;
        }
        self.last_sample_us = now_us;

        match self.state {
            CaptureState::Armed => {
                // Add to ring buffer
                self.buffer[self.head] = frame;
                self.head = (self.head + 1) % N;
                if self.count < N {
                    self.count += 1;
                }

                // Check for RPM spike auto-trigger
                if self.config.is_trigger_enabled(CaptureTrigger::RpmSpike) && self.last_rpm > 0 {
                    let rpm_change = (frame.rpm as i32 - self.last_rpm as i32).unsigned_abs();
                    // Convert to RPM/second
                    if elapsed > 0 {
                        let rate = (rpm_change as u64 * 1_000_000) / elapsed as u64;
                        if rate > self.config.rpm_spike_threshold as u64 {
                            self.trigger(CaptureTrigger::RpmSpike);
                        }
                    }
                }
                self.last_rpm = frame.rpm;
            }
            CaptureState::Capturing => {
                // Add post-trigger sample
                self.buffer[self.head] = frame;
                self.head = (self.head + 1) % N;
                if self.count < N {
                    self.count += 1;
                }

                self.post_remaining = self.post_remaining.saturating_sub(1);
                if self.post_remaining == 0 {
                    self.state = CaptureState::Complete;
                }
            }
            CaptureState::Complete | CaptureState::Disabled => {
                // Ignore samples
            }
        }
    }

    /// Trigger capture
    ///
    /// Transitions from Armed to Capturing state.
    /// The ring buffer contents become pre-trigger data.
    pub fn trigger(&mut self, reason: CaptureTrigger) {
        if self.state != CaptureState::Armed {
            return;
        }

        if !self.config.is_trigger_enabled(reason) {
            return;
        }

        self.state = CaptureState::Capturing;
        self.trigger_reason = Some(reason);
        self.trigger_index = if self.head == 0 { N - 1 } else { self.head - 1 };
        self.post_remaining = self.config.post_trigger_samples.min(N as u16);
    }

    /// Check if capture is complete
    pub fn is_complete(&self) -> bool {
        self.state == CaptureState::Complete
    }

    /// Check if armed and waiting for trigger
    pub fn is_armed(&self) -> bool {
        self.state == CaptureState::Armed
    }

    /// Get the number of captured samples
    pub fn sample_count(&self) -> usize {
        self.count
    }

    /// Get a sample at the given index (0 = oldest)
    pub fn get_sample(&self, index: usize) -> Option<&CaptureFrame> {
        if index >= self.count {
            return None;
        }

        // Calculate actual buffer index
        let start = if self.count == N {
            self.head
        } else {
            0
        };
        let actual = (start + index) % N;
        Some(&self.buffer[actual])
    }

    /// Get the sample where trigger occurred
    pub fn get_trigger_sample(&self) -> Option<&CaptureFrame> {
        if self.trigger_reason.is_some() {
            Some(&self.buffer[self.trigger_index])
        } else {
            None
        }
    }

    /// Reset capture system
    pub fn reset(&mut self) {
        self.head = 0;
        self.count = 0;
        self.state = if self.config.enable {
            CaptureState::Armed
        } else {
            CaptureState::Disabled
        };
        self.trigger_reason = None;
        self.trigger_index = 0;
        self.post_remaining = 0;
    }

    /// Arm the capture system
    pub fn arm(&mut self) {
        if self.config.enable {
            self.reset();
            self.state = CaptureState::Armed;
        }
    }

    /// Disable capture
    pub fn disable(&mut self) {
        self.state = CaptureState::Disabled;
    }

    /// Enable capture
    pub fn enable(&mut self) {
        if self.state == CaptureState::Disabled {
            self.state = CaptureState::Armed;
        }
    }
}

impl<const N: usize> Default for TriggeredCapture<N> {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    // Basic CaptureBuffer tests
    #[test]
    fn test_capture_buffer_basic() {
        let mut buf = CaptureBuffer::<4>::new();
        assert!(buf.try_pop().is_none());

        buf.push(100);
        buf.push(200);
        assert_eq!(buf.try_pop(), Some(100));
        assert_eq!(buf.try_pop(), Some(200));
        assert!(buf.try_pop().is_none());
    }

    // CaptureTrigger tests
    #[test]
    fn test_capture_trigger_mask() {
        assert_eq!(CaptureTrigger::Manual.mask(), 0x01);
        assert_eq!(CaptureTrigger::SyncLoss.mask(), 0x02);
        assert_eq!(CaptureTrigger::KnockDetected.mask(), 0x04);
    }

    #[test]
    fn test_capture_config_trigger_enabled() {
        let config = CaptureConfig {
            enabled_triggers: trigger_mask::SYNC_LOSS | trigger_mask::KNOCK,
            ..CaptureConfig::DEFAULT
        };

        assert!(!config.is_trigger_enabled(CaptureTrigger::Manual));
        assert!(config.is_trigger_enabled(CaptureTrigger::SyncLoss));
        assert!(config.is_trigger_enabled(CaptureTrigger::KnockDetected));
        assert!(!config.is_trigger_enabled(CaptureTrigger::RpmSpike));
    }

    #[test]
    fn test_capture_frame_from_state() {
        let frame = CaptureFrame::from_state(
            1000, 3000, 800, 50, 12500, 25, true, false, true,
        );

        assert_eq!(frame.timestamp_us, 1000);
        assert_eq!(frame.rpm, 3000);
        assert_eq!(frame.map_kpa_x10, 800);
        assert_eq!(frame.tps_percent, 50);
        assert_eq!(frame.voltage_mv_div100, 125);
        assert_eq!(frame.stft_offset, 130); // 2.5% + 128
        assert_eq!(frame.status, 0x05); // synced + knock
    }

    #[test]
    fn test_triggered_capture_new() {
        let capture = TriggeredCapture::<32>::new();
        assert_eq!(capture.state, CaptureState::Armed);
        assert_eq!(capture.sample_count(), 0);
        assert!(capture.trigger_reason.is_none());
    }

    #[test]
    fn test_triggered_capture_sample() {
        let mut capture = TriggeredCapture::<32>::new();
        capture.config.sample_interval_us = 0; // Disable rate limiting

        for i in 0..10 {
            let frame = CaptureFrame {
                timestamp_us: i * 1000,
                rpm: 3000,
                ..CaptureFrame::new()
            };
            capture.sample(frame, i * 1000);
        }

        assert_eq!(capture.sample_count(), 10);
        assert!(capture.is_armed());
    }

    #[test]
    fn test_triggered_capture_trigger() {
        let mut capture = TriggeredCapture::<32>::new();
        capture.config.sample_interval_us = 0;
        capture.config.enabled_triggers = trigger_mask::ALL;
        capture.config.post_trigger_samples = 5;

        // Add pre-trigger samples
        for i in 0..10 {
            let frame = CaptureFrame {
                timestamp_us: i * 1000,
                rpm: 3000,
                ..CaptureFrame::new()
            };
            capture.sample(frame, i * 1000);
        }

        assert!(capture.is_armed());

        // Trigger
        capture.trigger(CaptureTrigger::SyncLoss);
        assert_eq!(capture.state, CaptureState::Capturing);
        assert_eq!(capture.trigger_reason, Some(CaptureTrigger::SyncLoss));

        // Add post-trigger samples
        for i in 10..20 {
            let frame = CaptureFrame {
                timestamp_us: i * 1000,
                rpm: 3000,
                ..CaptureFrame::new()
            };
            capture.sample(frame, i * 1000);
        }

        assert!(capture.is_complete());
    }

    #[test]
    fn test_triggered_capture_disabled_trigger() {
        let mut capture = TriggeredCapture::<32>::new();
        capture.config.enabled_triggers = trigger_mask::SYNC_LOSS; // Only sync loss

        // Try to trigger with knock (not enabled)
        capture.trigger(CaptureTrigger::KnockDetected);

        // Should still be armed
        assert!(capture.is_armed());
        assert!(capture.trigger_reason.is_none());

        // Trigger with sync loss (enabled)
        capture.trigger(CaptureTrigger::SyncLoss);
        assert_eq!(capture.state, CaptureState::Capturing);
    }

    #[test]
    fn test_triggered_capture_ring_buffer_wrap() {
        let mut capture = TriggeredCapture::<8>::new();
        capture.config.sample_interval_us = 0;

        // Add more samples than buffer size
        for i in 0..15 {
            let frame = CaptureFrame {
                timestamp_us: i * 1000,
                rpm: (i + 1) as u16 * 100,
                ..CaptureFrame::new()
            };
            capture.sample(frame, i * 1000);
        }

        // Should have wrapped, keeping last 8 samples
        assert_eq!(capture.sample_count(), 8);

        // Oldest sample should be #7 (0-indexed: sample at i=7)
        let oldest = capture.get_sample(0).unwrap();
        assert_eq!(oldest.rpm, 800); // (7+1) * 100
    }

    #[test]
    fn test_triggered_capture_reset() {
        let mut capture = TriggeredCapture::<32>::new();
        capture.config.sample_interval_us = 0;
        capture.config.enabled_triggers = trigger_mask::ALL;

        // Add samples and trigger
        for i in 0..5 {
            capture.sample(CaptureFrame::new(), i * 1000);
        }
        capture.trigger(CaptureTrigger::Manual);

        assert_eq!(capture.state, CaptureState::Capturing);

        // Reset
        capture.reset();

        assert_eq!(capture.state, CaptureState::Armed);
        assert_eq!(capture.sample_count(), 0);
        assert!(capture.trigger_reason.is_none());
    }

    #[test]
    fn test_triggered_capture_get_sample() {
        let mut capture = TriggeredCapture::<8>::new();
        capture.config.sample_interval_us = 0;

        for i in 0..5 {
            let frame = CaptureFrame {
                timestamp_us: i * 1000,
                rpm: (i + 1) as u16 * 100,
                ..CaptureFrame::new()
            };
            capture.sample(frame, i * 1000);
        }

        assert_eq!(capture.get_sample(0).unwrap().rpm, 100);
        assert_eq!(capture.get_sample(4).unwrap().rpm, 500);
        assert!(capture.get_sample(5).is_none());
    }

    #[test]
    fn test_capture_frame_serialization() {
        let frame = CaptureFrame {
            timestamp_us: 12345678,
            rpm: 3500,
            map_kpa_x10: 850,
            tps_percent: 75,
            voltage_mv_div100: 125,
            stft_offset: 135,
            status: 0x07,
        };

        let bytes = frame.to_bytes();
        assert_eq!(bytes.len(), CaptureFrame::SIZE);

        // Verify timestamp
        let ts = u32::from_le_bytes([bytes[0], bytes[1], bytes[2], bytes[3]]);
        assert_eq!(ts, 12345678);

        // Verify RPM
        let rpm = u16::from_le_bytes([bytes[4], bytes[5]]);
        assert_eq!(rpm, 3500);
    }

    #[test]
    fn test_triggered_capture_sample_rate_limiting() {
        let mut capture = TriggeredCapture::<32>::new();
        capture.config.sample_interval_us = 10_000; // 10ms

        // Try to add samples too fast
        capture.sample(CaptureFrame::new(), 0);
        capture.sample(CaptureFrame::new(), 5_000);  // Too soon
        capture.sample(CaptureFrame::new(), 10_000); // OK
        capture.sample(CaptureFrame::new(), 15_000); // Too soon
        capture.sample(CaptureFrame::new(), 20_000); // OK

        assert_eq!(capture.sample_count(), 3);
    }

    #[test]
    fn test_triggered_capture_disable_enable() {
        let mut capture = TriggeredCapture::<32>::new();

        capture.disable();
        assert_eq!(capture.state, CaptureState::Disabled);

        capture.enable();
        assert_eq!(capture.state, CaptureState::Armed);
    }

    #[test]
    fn test_triggered_capture_post_trigger_exact_count() {
        let mut capture = TriggeredCapture::<32>::new();
        capture.config.sample_interval_us = 0;
        // Only enable Manual trigger (not ALL to avoid RPM spike auto-trigger)
        capture.config.enabled_triggers = trigger_mask::MANUAL;
        capture.config.post_trigger_samples = 3; // Exactly 3 post-trigger

        // Add pre-trigger samples with stable RPM
        for i in 0..5u32 {
            capture.sample(CaptureFrame { rpm: 3000, ..CaptureFrame::new() }, i * 1000);
        }

        // Trigger
        capture.trigger(CaptureTrigger::Manual);
        assert_eq!(capture.state, CaptureState::Capturing);

        // Add exactly 3 post-trigger samples
        capture.sample(CaptureFrame { rpm: 3000, ..CaptureFrame::new() }, 10_000);
        assert_eq!(capture.state, CaptureState::Capturing);

        capture.sample(CaptureFrame { rpm: 3000, ..CaptureFrame::new() }, 11_000);
        assert_eq!(capture.state, CaptureState::Capturing);

        capture.sample(CaptureFrame { rpm: 3000, ..CaptureFrame::new() }, 12_000);
        assert!(capture.is_complete(), "Should be complete after 3 post-trigger samples");

        // Verify sample count (5 pre + 3 post = 8)
        assert_eq!(capture.sample_count(), 8);
    }

    #[test]
    fn test_triggered_capture_trigger_sample_access() {
        let mut capture = TriggeredCapture::<32>::new();
        capture.config.sample_interval_us = 0;
        // Only enable SyncLoss (avoid RPM spike auto-trigger)
        capture.config.enabled_triggers = trigger_mask::SYNC_LOSS;
        capture.config.post_trigger_samples = 2;

        // Add pre-trigger samples with stable RPM, using timestamp to mark which sample
        for i in 0..5u32 {
            capture.sample(CaptureFrame {
                rpm: 3000,
                timestamp_us: (i + 1) * 1000, // Use timestamp to identify samples
                ..CaptureFrame::new()
            }, i * 1000);
        }

        // Last sample before trigger has timestamp 5000
        capture.trigger(CaptureTrigger::SyncLoss);

        // Get the trigger sample
        let trigger_sample = capture.get_trigger_sample().unwrap();
        assert_eq!(trigger_sample.timestamp_us, 5000, "Trigger sample should be the last pre-trigger sample");

        // Complete the capture
        capture.sample(CaptureFrame { rpm: 3000, timestamp_us: 6000, ..CaptureFrame::new() }, 10_000);
        capture.sample(CaptureFrame { rpm: 3000, timestamp_us: 7000, ..CaptureFrame::new() }, 11_000);

        assert!(capture.is_complete());

        // Trigger sample should still be accessible
        let trigger_sample = capture.get_trigger_sample().unwrap();
        assert_eq!(trigger_sample.timestamp_us, 5000);
    }

    #[test]
    fn test_triggered_capture_complete_ignores_samples() {
        let mut capture = TriggeredCapture::<32>::new();
        capture.config.sample_interval_us = 0;
        capture.config.enabled_triggers = trigger_mask::ALL;
        capture.config.post_trigger_samples = 1;

        // Trigger and complete
        capture.sample(CaptureFrame { rpm: 100, ..CaptureFrame::new() }, 0);
        capture.trigger(CaptureTrigger::Manual);
        capture.sample(CaptureFrame { rpm: 200, ..CaptureFrame::new() }, 1000);

        assert!(capture.is_complete());
        let count_before = capture.sample_count();

        // Try to add more samples after complete
        capture.sample(CaptureFrame { rpm: 300, ..CaptureFrame::new() }, 2000);
        capture.sample(CaptureFrame { rpm: 400, ..CaptureFrame::new() }, 3000);

        // Count should not change
        assert_eq!(capture.sample_count(), count_before);
    }

    #[test]
    fn test_triggered_capture_retrigger_after_reset() {
        let mut capture = TriggeredCapture::<32>::new();
        capture.config.sample_interval_us = 0;
        capture.config.enabled_triggers = trigger_mask::ALL;
        capture.config.post_trigger_samples = 1;

        // First capture cycle
        capture.sample(CaptureFrame::new(), 0);
        capture.trigger(CaptureTrigger::SyncLoss);
        capture.sample(CaptureFrame::new(), 1000);
        assert!(capture.is_complete());
        assert_eq!(capture.trigger_reason, Some(CaptureTrigger::SyncLoss));

        // Reset and trigger again with different reason
        capture.reset();
        assert!(capture.is_armed());
        assert!(capture.trigger_reason.is_none());

        capture.sample(CaptureFrame::new(), 10_000);
        capture.trigger(CaptureTrigger::KnockDetected);
        assert_eq!(capture.trigger_reason, Some(CaptureTrigger::KnockDetected));
    }

    #[test]
    fn test_triggered_capture_rpm_spike_detection() {
        let mut capture = TriggeredCapture::<32>::new();
        capture.config.sample_interval_us = 0;
        capture.config.enabled_triggers = trigger_mask::RPM_SPIKE;
        capture.config.rpm_spike_threshold = 5000; // 5000 RPM/sec
        capture.config.post_trigger_samples = 2;

        // Normal samples (1000 RPM, 1ms apart = stable)
        capture.sample(CaptureFrame { rpm: 3000, ..CaptureFrame::new() }, 0);
        capture.sample(CaptureFrame { rpm: 3000, ..CaptureFrame::new() }, 1000);
        assert!(capture.is_armed());

        // Sudden spike (3000 -> 3100 in 1ms = 100,000 RPM/sec)
        capture.sample(CaptureFrame { rpm: 3100, ..CaptureFrame::new() }, 2000);

        // Should trigger
        assert_eq!(capture.state, CaptureState::Capturing);
        assert_eq!(capture.trigger_reason, Some(CaptureTrigger::RpmSpike));
    }

    #[test]
    fn test_triggered_capture_all_trigger_types() {
        // Test each trigger type can be enabled/disabled independently
        for trigger_type in [
            CaptureTrigger::Manual,
            CaptureTrigger::SyncLoss,
            CaptureTrigger::KnockDetected,
            CaptureTrigger::SensorFault,
            CaptureTrigger::LimpEntry,
            CaptureTrigger::LambdaFault,
            CaptureTrigger::LowVoltage,
        ] {
            let mut capture = TriggeredCapture::<8>::new();
            capture.config.enabled_triggers = trigger_type.mask();
            capture.config.sample_interval_us = 0;

            // Should trigger with enabled type
            capture.sample(CaptureFrame::new(), 0);
            capture.trigger(trigger_type);
            assert_eq!(capture.trigger_reason, Some(trigger_type),
                "Trigger {:?} should be enabled", trigger_type);

            // Reset and try different type
            capture.reset();
            let other_type = if trigger_type == CaptureTrigger::Manual {
                CaptureTrigger::SyncLoss
            } else {
                CaptureTrigger::Manual
            };
            capture.trigger(other_type);
            assert!(capture.trigger_reason.is_none(),
                "Trigger {:?} should be disabled when only {:?} is enabled", other_type, trigger_type);
        }
    }

    #[test]
    fn test_triggered_capture_pre_trigger_count() {
        let mut capture = TriggeredCapture::<8>::new();
        capture.config.sample_interval_us = 0;
        // Only enable Manual (avoid RPM spike auto-trigger)
        capture.config.enabled_triggers = trigger_mask::MANUAL;
        capture.config.post_trigger_samples = 2;

        // Add fewer samples than buffer size, stable RPM with distinct timestamps
        for i in 0..3u32 {
            capture.sample(CaptureFrame {
                rpm: 3000,
                timestamp_us: (i + 1) * 1000,
                ..CaptureFrame::new()
            }, i * 1000);
        }

        assert_eq!(capture.sample_count(), 3);

        // Trigger - should preserve pre-trigger samples
        capture.trigger(CaptureTrigger::Manual);

        // Add post-trigger samples
        capture.sample(CaptureFrame { rpm: 3000, timestamp_us: 4000, ..CaptureFrame::new() }, 10_000);
        capture.sample(CaptureFrame { rpm: 3000, timestamp_us: 5000, ..CaptureFrame::new() }, 11_000);

        assert!(capture.is_complete());
        assert_eq!(capture.sample_count(), 5); // 3 pre + 2 post

        // Verify ordering by timestamp
        assert_eq!(capture.get_sample(0).unwrap().timestamp_us, 1000);
        assert_eq!(capture.get_sample(1).unwrap().timestamp_us, 2000);
        assert_eq!(capture.get_sample(2).unwrap().timestamp_us, 3000);
        assert_eq!(capture.get_sample(3).unwrap().timestamp_us, 4000);
        assert_eq!(capture.get_sample(4).unwrap().timestamp_us, 5000);
    }
}
