//! Capture model and type definitions.

/// Trigger sources for event capture.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum CaptureTrigger {
    /// Manual trigger via TunerStudio or command.
    Manual = 0,
    /// Trigger sync was lost.
    SyncLoss = 1,
    /// Knock event detected.
    KnockDetected = 2,
    /// Sudden RPM change (stall or spike).
    RpmSpike = 3,
    /// Any sensor fault (out of range, implausible).
    SensorFault = 4,
    /// Entered limp mode.
    LimpEntry = 5,
    /// Lambda sensor fault or oscillation.
    LambdaFault = 6,
    /// Low voltage / brown-out.
    LowVoltage = 7,
}

impl CaptureTrigger {
    /// Bitmask for this trigger.
    pub const fn mask(self) -> u8 {
        1 << (self as u8)
    }
}

/// Bitmask of enabled triggers.
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

/// Configuration for triggered capture.
#[derive(Debug, Clone, Copy)]
pub struct CaptureConfig {
    /// Enable triggered capture.
    pub enable: bool,
    /// Number of samples to keep before trigger (ring buffer size).
    pub pre_trigger_samples: u16,
    /// Number of samples to capture after trigger.
    pub post_trigger_samples: u16,
    /// Sample interval in microseconds.
    pub sample_interval_us: u32,
    /// Bitmask of enabled trigger sources.
    pub enabled_triggers: u8,
    /// RPM change threshold for RpmSpike trigger (RPM/second).
    pub rpm_spike_threshold: u16,
}

impl CaptureConfig {
    pub const DEFAULT: Self = Self {
        enable: true,
        pre_trigger_samples: 50,
        post_trigger_samples: 100,
        sample_interval_us: 10_000,
        enabled_triggers: trigger_mask::SAFETY_ONLY,
        rpm_spike_threshold: 2000,
    };

    /// Check if a trigger source is enabled.
    pub fn is_trigger_enabled(&self, trigger: CaptureTrigger) -> bool {
        self.enable && (self.enabled_triggers & trigger.mask()) != 0
    }
}

impl Default for CaptureConfig {
    fn default() -> Self {
        Self::DEFAULT
    }
}

/// A single captured frame of engine data.
#[derive(Debug, Clone, Copy, Default)]
pub struct CaptureFrame {
    /// Timestamp in microseconds.
    pub timestamp_us: u32,
    /// Engine RPM.
    pub rpm: u16,
    /// MAP reading (kPa x10).
    pub map_kpa_x10: u16,
    /// TPS reading (0-100%).
    pub tps_percent: u8,
    /// Battery voltage (millivolts / 100).
    pub voltage_mv_div100: u8,
    /// Short-term fuel trim (percent + 128, so 128 = 0%).
    pub stft_offset: u8,
    /// Status flags (sync, limp, knock, etc.).
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
            stft_offset: 128,
            status: 0,
        }
    }

    /// Create a frame from current engine state.
    #[allow(clippy::too_many_arguments)]
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
        if synced {
            status |= 0x01;
        }
        if limp {
            status |= 0x02;
        }
        if knock {
            status |= 0x04;
        }

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

    /// Serialized frame size (bytes).
    pub const SIZE: usize = 12;

    /// Serialize to bytes.
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

/// State of the triggered capture system.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CaptureState {
    /// Idle, waiting for trigger (ring buffer filling).
    Armed,
    /// Trigger fired, capturing post-trigger samples.
    Capturing,
    /// Capture complete, data ready to download.
    Complete,
    /// Disabled.
    Disabled,
}

/// Event-triggered capture with pre/post-trigger data.
#[derive(Debug)]
pub struct TriggeredCapture<const N: usize> {
    pub(super) buffer: [CaptureFrame; N],
    pub(super) head: usize,
    pub(super) count: usize,
    /// Current state.
    pub state: CaptureState,
    /// Trigger reason (if triggered).
    pub trigger_reason: Option<CaptureTrigger>,
    pub(super) trigger_index: usize,
    pub(super) post_remaining: u16,
    pub(super) last_sample_us: u32,
    pub(super) last_rpm: u16,
    /// Configuration.
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

    /// Check if capture is complete.
    pub fn is_complete(&self) -> bool {
        self.state == CaptureState::Complete
    }

    /// Check if armed and waiting for trigger.
    pub fn is_armed(&self) -> bool {
        self.state == CaptureState::Armed
    }

    /// Get the number of captured samples.
    pub fn sample_count(&self) -> usize {
        self.count
    }

    /// Get a sample at the given index (0 = oldest).
    pub fn get_sample(&self, index: usize) -> Option<&CaptureFrame> {
        if index >= self.count {
            return None;
        }

        let start = if self.count == N { self.head } else { 0 };
        let actual = (start + index) % N;
        Some(&self.buffer[actual])
    }

    /// Get the sample where trigger occurred.
    pub fn get_trigger_sample(&self) -> Option<&CaptureFrame> {
        if self.trigger_reason.is_some() {
            Some(&self.buffer[self.trigger_index])
        } else {
            None
        }
    }
}

impl<const N: usize> Default for TriggeredCapture<N> {
    fn default() -> Self {
        Self::new()
    }
}
