//! System-wide constants for ECU configuration
//!
//! All tunable parameters and hardware-specific constants are defined here
//! to make them easy to find and modify.

/// 60-2 trigger wheel configuration
pub mod trigger {
    /// Total number of teeth on the trigger wheel (excluding missing teeth)
    pub const TEETH_PER_REV: u8 = 58;

    /// Number of missing teeth in the gap
    pub const MISSING_TEETH: u8 = 2;

    /// Minimum valid tooth period in microseconds
    /// Periods shorter than this are considered noise
    pub const MIN_VALID_PERIOD_US: u32 = 100;

    /// Maximum time between teeth before losing sync (microseconds)
    /// If no tooth seen for this long, assume signal lost
    pub const SYNC_TIMEOUT_US: u32 = 200_000; // 200ms

    /// Threshold multiplier for missing tooth detection
    /// Missing tooth period must be > last_period * MISSING_TOOTH_THRESHOLD_NUM / MISSING_TOOTH_THRESHOLD_DEN
    pub const MISSING_TOOTH_THRESHOLD_NUM: u32 = 3; // Numerator: 3/2 = 1.5x
    pub const MISSING_TOOTH_THRESHOLD_DEN: u32 = 2; // Denominator
}

/// Engine timing configuration
pub mod timing {
    /// Tooth number for injection scheduling (approximately TDC)
    pub const INJECTION_TOOTH: u8 = 30;

    /// Tooth number for ignition scheduling (approximately 10° BTDC)
    pub const IGNITION_TOOTH: u8 = 58;

    /// Delay from trigger event to injection start (microseconds)
    pub const INJECTION_DELAY_US: u32 = 100;

    /// Ignition coil dwell time (microseconds)
    /// Time the coil is charged before spark
    pub const DWELL_TIME_US: u32 = 3000; // 3ms
}

/// Fuel table configuration
pub mod fuel {
    /// RPM axis bins for IPW table (16 points)
    pub const RPM_BINS: [u16; 16] = [
        500, 1000, 1500, 2000, 2500, 3000, 3500, 4000, 4500, 5000, 5500, 6000, 6500, 7000, 7500,
        8000,
    ];

    /// Load axis bins for IPW table (16 points, in kPa)
    pub const LOAD_BINS: [u16; 16] = [
        20, 30, 40, 50, 60, 70, 80, 90, 100, 110, 120, 130, 140, 150, 160, 170,
    ];

    /// Default pulse width (microseconds)
    pub const DEFAULT_PULSE_WIDTH_US: u16 = 1000; // 1ms

    /// Minimum pulse width (microseconds)
    pub const MIN_PULSE_WIDTH_US: u16 = 500; // 0.5ms

    /// Maximum pulse width (microseconds)
    pub const MAX_PULSE_WIDTH_US: u16 = 20000; // 20ms

    /// Default load value for MVP testing (kPa)
    pub const DEFAULT_LOAD_KPA: u16 = 80;
}

/// Correction factors
pub mod corrections {
    /// Correction factor representing 1.0x (no correction)
    /// All correction factors are scaled by this value
    /// Example: 150 = 1.5x, 80 = 0.8x
    pub const UNITY_CORRECTION: u8 = 100;
}

/// Scheduler configuration
pub mod scheduler {
    /// Maximum number of scheduled events
    /// Scaled to support up to ~16 outputs (start/stop per event)
    pub const MAX_EVENTS: usize = 64;

    /// Channel assignments
    pub const CHANNEL_INJ1: u8 = 0;
    pub const CHANNEL_INJ2: u8 = 1;
    pub const CHANNEL_IGN1: u8 = 2;
    pub const CHANNEL_IGN2: u8 = 3;
    /// Maximum number of logical output channels supported by the scheduler.
    /// Boards may provide fewer physical pins; unused channels are ignored at drive time.
    pub const MAX_CHANNELS: u8 = 16;
}

/// RPM calculation constants
pub mod rpm {
    /// Numerator for RPM calculation from missing tooth period
    /// RPM = RPM_CALC_NUMERATOR / period_us
    ///
    /// Derivation:
    /// - 60-2 wheel has 58 teeth per revolution
    /// - Missing gap = 2 teeth worth of time
    /// - Full revolution = 29 * gap_period (58 teeth / 2)
    /// - RPM = 60,000,000 us/min / (gap_period * 29)
    /// - RPM = 2,068,966 / gap_period
    /// - Rounded to 2,000,000 for fast integer division (~3% error)
    pub const RPM_CALC_NUMERATOR: u32 = 2_000_000;

    /// Minimum period for valid RPM calculation (microseconds)
    /// Periods longer than this result in RPM = 0
    pub const MAX_PERIOD_FOR_CALC_US: u32 = 60_000; // < 1000 RPM
}

/// Ignition timing and dwell constants
pub mod ignition {
    /// Default ignition timing (degrees BTDC)
    /// Conservative value safe for most engines
    pub const DEFAULT_TIMING_BTDC: i16 = 15;

    /// Minimum ignition timing (degrees BTDC)
    /// Negative values = ATDC (after TDC)
    /// -10° ATDC is very retarded, used for extreme knock or limiting
    pub const MIN_TIMING_BTDC: i16 = -10;

    /// Maximum ignition timing (degrees BTDC)
    /// 45° is very advanced, typical max is 35-40° for most engines
    pub const MAX_TIMING_BTDC: i16 = 45;

    /// Minimum coil dwell time (microseconds)
    /// Below this, spark energy is insufficient
    pub const MIN_DWELL_US: u32 = 1500; // 1.5ms

    /// Maximum coil dwell time (microseconds)
    /// Above this, coil may overheat
    pub const MAX_DWELL_US: u32 = 6000; // 6ms

    /// Default dwell time (microseconds)
    /// At nominal voltage (13.5V)
    pub const DEFAULT_DWELL_US: u32 = 3000; // 3ms

    /// Cranking timing (degrees BTDC)
    /// Fixed timing during cranking for reliable starting
    pub const CRANKING_TIMING_BTDC: i16 = 10;
}

/// Rev limiter (RPM limiting) constants
pub mod rev_limiter {
    /// Default maximum RPM (conservative for street use)
    /// Typical 4-cylinder redline: 6500-7500 RPM
    pub const DEFAULT_MAX_RPM: u16 = 7000;

    /// RPM below max where soft limiting begins
    /// Gives 500 RPM window for gradual reduction
    pub const DEFAULT_SOFT_LIMIT_START_RPM: u16 = 6500;

    /// Hysteresis: RPM must drop this much below limit before re-enabling
    /// Prevents oscillation at the limiter
    pub const HYSTERESIS_RPM: u16 = 200;

    /// Minimum safe RPM for engine operation
    /// Below this is considered a stall
    pub const MIN_RUNNING_RPM: u16 = 400;
}

/// Safety features constants
pub mod safety {
    /// Cranking RPM threshold
    /// Above this, engine is considered running (not cranking)
    pub const CRANKING_RPM_THRESHOLD: u16 = 500;
    /// Exit hysteresis for cranking detection (RPM)
    pub const CRANKING_EXIT_RPM: u16 = 600;

    /// TPS threshold for wide-open throttle (WOT)
    /// 90% or higher is considered WOT for flood clear
    pub const WOT_TPS_THRESHOLD: u8 = 90;

    /// Sync loss timeout (microseconds)
    /// If no trigger edges for this long, assume sync lost
    /// Must be longer than slowest expected tooth period
    pub const SYNC_LOSS_TIMEOUT_US: u32 = 200_000; // 200ms = ~300 RPM minimum

    /// Sync recovery attempts before shutdown
    /// Allows recovery from brief ESD-induced glitches
    pub const SYNC_RECOVERY_ATTEMPTS: u8 = 3;

    /// Time window for sync recovery attempts (microseconds)
    /// If we lose sync multiple times within this window, shut down
    /// But if losses are spread out (ESD events), keep trying
    pub const SYNC_RECOVERY_WINDOW_US: u32 = 5_000_000; // 5 seconds
}
