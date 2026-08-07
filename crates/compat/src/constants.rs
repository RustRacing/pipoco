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
    /// RPM axis bins for IPW table (16 points).
    ///
    /// Single source of truth: `ecu_calibration::FUEL_RUNTIME_RPM_BINS`
    /// (review 014).
    pub use ecu_calibration::FUEL_RUNTIME_RPM_BINS as RPM_BINS;

    /// Load axis bins for IPW table (16 points, in kPa).
    ///
    /// Single source of truth: `ecu_calibration::FUEL_RUNTIME_LOAD_BINS`
    /// (review 014).
    pub use ecu_calibration::FUEL_RUNTIME_LOAD_BINS as LOAD_BINS;

    /// Default pulse width (microseconds)
    pub const DEFAULT_PULSE_WIDTH_US: u16 = 1000; // 1ms

    /// Minimum pulse width (microseconds)
    pub const MIN_PULSE_WIDTH_US: u16 = 500; // 0.5ms

    /// Maximum pulse width (microseconds).
    ///
    /// Single source of truth: `ecu_domain::MAX_PULSE_WIDTH_US` (review 008).
    pub use ecu_domain::MAX_PULSE_WIDTH_US;

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

/// RPM calculation constants
pub mod rpm {
    /// Numerator for RPM calculation from missing tooth period
    /// RPM = RPM_CALC_NUMERATOR / period_us.
    ///
    /// Single source of truth: `ecu_domain::RPM_CALC_NUMERATOR_FAST`
    /// (review 008); the exact numerator is `ecu_domain::RPM_CALC_NUMERATOR_EXACT`.
    pub use ecu_domain::RPM_CALC_NUMERATOR_FAST as RPM_CALC_NUMERATOR;

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

/// Load failure detection thresholds
pub mod load_failure {
    /// RPM threshold above which MAP failure is critical
    /// At high RPM, running without load information risks engine damage
    pub const LOAD_FAILURE_RPM_THRESHOLD: u16 = 4000;

    /// RPM limit when in load-failure limp mode
    pub const LOAD_FAILURE_LIMP_RPM: u16 = 3000;

    /// Time required with good signal before exiting limp (microseconds)
    pub const LOAD_FAILURE_RECOVERY_US: u32 = 2_000_000; // 2 seconds

    /// Debounce time for sensor failure detection (microseconds)
    /// Prevents brief glitches from triggering limp mode
    pub const LOAD_FAILURE_DEBOUNCE_US: u32 = 100_000; // 100ms
}

/// Voltage monitoring thresholds
pub mod voltage {
    /// Critical low voltage (millivolts) - cut fuel to prevent damage
    /// Below 8V, injectors and coils behave erratically
    pub use ecu_domain::voltage::BROWNOUT_CRITICAL_MV;

    /// Warning low voltage (millivolts) - enter limp mode
    /// Below 10V, reduce load on electrical system
    pub const BROWNOUT_WARNING_MV: u16 = 10000;

    /// Overvoltage threshold (millivolts) - load dump detection
    /// Above 16.5V indicates alternator load dump or jump start
    pub use ecu_domain::voltage::OVERVOLTAGE_MV;

    /// Recovery voltage (millivolts) - exit limp mode
    /// Must be above this for sustained period to exit limp
    pub const RECOVERY_MV: u16 = 11500;

    /// Time required at recovery voltage before exiting limp (microseconds)
    pub const RECOVERY_TIME_US: u32 = 2_000_000; // 2 seconds

    /// RPM limit during voltage warning (limp mode)
    pub const LIMP_RPM_LIMIT: u16 = 3000;

    /// Number of consecutive critical readings before fuel cut
    /// Prevents single-sample glitches from cutting fuel
    pub const CRITICAL_DEBOUNCE_COUNT: u8 = 3;
}
