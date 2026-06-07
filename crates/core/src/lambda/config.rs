/// LTFT constants
pub mod ltft_constants {
    /// Default RPM bins for 4x4 LTFT table.
    pub const RPM_BINS: [u16; 4] = [1000, 2000, 3500, 5500];
    /// Default load bins (kPa x10) for 4x4 LTFT table.
    pub const LOAD_BINS: [u16; 4] = [300, 600, 900, 1200];
    /// Maximum LTFT trim (±10%).
    pub const MAX_TRIM_X10: i16 = 100;
    /// Minimum samples before cell is considered learned.
    pub const MIN_SAMPLES: u16 = 50;
    /// Default learning rate (0-255, where 255 = instant).
    pub const DEFAULT_LEARN_RATE: u8 = 4;
    /// STFT threshold for learning (only learn if STFT is stable).
    pub const STFT_THRESHOLD_X10: i16 = 30; // 3%
    /// Minimum time at steady-state before learning (microseconds).
    pub const STEADY_STATE_TIME_US: u32 = 2_000_000; // 2 seconds
    /// Maximum RPM deviation for steady-state detection.
    pub const STEADY_STATE_RPM_DEV: u16 = 200;
    /// Maximum load deviation for steady-state detection (kPa x10).
    pub const STEADY_STATE_LOAD_DEV: u16 = 50;
    /// Key for persisting LTFT table.
    pub const PERSIST_KEY: &[u8] = b"ltft";
}

pub use ecu_calibration::configs::{LambdaConfig, LtftConfig};

/// O2 sensor type.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum O2SensorType {
    /// Narrowband (0-1V, rich/lean only).
    #[default]
    Narrowband,
    /// Wideband (0-5V = 10-20 AFR typical).
    Wideband,
}

/// Reason closed-loop is disabled.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DisableReason {
    /// Feature disabled in config.
    ConfigDisabled,
    /// Engine too cold.
    CoolantTooLow,
    /// At wide-open throttle.
    WideOpenThrottle,
    /// RPM too low.
    RpmTooLow,
    /// Engine not running / no O2 signal.
    NoSignal,
    /// Manual override.
    ManualDisable,
}
