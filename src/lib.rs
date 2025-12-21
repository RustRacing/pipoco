//! ECU Core Library
//!
//! Minimal viable ECU (Engine Control Unit) implementation in Rust.
//! Designed for no_std embedded environments with zero dependencies.
//!
//! # Architecture
//!
//! This library uses an IPW (Injector Pulse Width) table approach instead of
//! traditional VE (Volumetric Efficiency) calculations. This eliminates complex
//! math from the embedded module - all calculations are pre-computed and stored
//! in lookup tables.
//!
//! ## Modules
//!
//! - `trigger`: 60-2 trigger wheel decoder for position and RPM
//! - `tables`: IPW table lookup (no interpolation)
//! - `scheduler`: Event scheduling for injection and ignition
//! - `hal`: Hardware abstraction traits
//! - `constants`: System-wide configuration constants
//! - `transport`: Transport-agnostic inter-component communication
//!
//! ## Design Principles
//!
//! - **Integer-only arithmetic**: No floating-point operations
//! - **Static memory**: No heap allocation, all state in static variables
//! - **Wrapping arithmetic**: Correctly handles timer overflow
//! - **Minimal dependencies**: Zero external dependencies in core library

#![cfg_attr(not(test), no_std)]

pub mod app;
pub mod actuators;
pub mod capture;
pub mod config;
pub mod constants;
pub mod dfco;
pub mod diag;
pub mod enrichment;
pub mod hal;
pub mod ignition;
pub mod knock;
pub mod lambda;
pub mod management;
pub mod persist;
pub mod rev_limiter;
pub mod safety;
pub mod scheduler;
pub mod sensors;
pub mod tables;
pub mod telemetry;
pub mod torque;
pub mod transport;
pub mod trigger;
pub mod ts;
pub mod ve_engine;

pub use app::EcuApp;
pub use capture::CaptureBuffer;
pub use ignition::{calculate_dwell, calculate_timing, IgnitionCorrections, IgnitionTable};
pub use rev_limiter::{
    apply_limiter_retard, should_inject, update_limiter, LimiterStrategy, RevLimiterConfig,
    RevLimiterState,
};
pub use safety::{
    should_allow_injection, update_flood_clear, FloodClearState, LoadFailureConfig,
    LoadFailureReason, LoadFailureTracker, PowerState, SyncLossTracker, VoltageMonitor,
};
pub use scheduler::{Channel, Event, Scheduler};
pub use tables::IpwTable;
pub use telemetry::IsrStats;
pub use transport::{Message, Transport, TransportError, TransportStats};
pub use trigger::{TriggerDecoder, TriggerTiming};

#[cfg(feature = "transport-bbqueue")]
pub use transport::BbqTransport;

use constants::corrections::*;
use constants::fuel::*;

/// Fixed-point math helper (no floats!)
///
/// Multiplies value by (multiplier / 100) using integer arithmetic only.
/// Uses saturating multiplication to prevent overflow.
///
/// # Arguments
/// * `value` - Base value (e.g., pulse width in microseconds)
/// * `multiplier` - Correction factor scaled by 100 (e.g., 150 = 1.5x, 80 = 0.8x)
///
/// # Returns
/// Corrected value, saturated at u16::MAX if overflow would occur
///
/// # Example
/// ```
/// use ecu_core::scale_u16;
///
/// assert_eq!(scale_u16(1000, 150), 1500);  // 1.5x
/// assert_eq!(scale_u16(1000, 80), 800);    // 0.8x
/// assert_eq!(scale_u16(1000, 100), 1000);  // 1.0x (no change)
/// ```
pub fn scale_u16(value: u16, multiplier: u8) -> u16 {
    // Use saturating multiply to prevent overflow
    let intermediate = (value as u32).saturating_mul(multiplier as u32);
    let result = intermediate / 100;

    // Clamp to u16::MAX
    if result > u16::MAX as u32 {
        u16::MAX
    } else {
        result as u16
    }
}

/// Apply a signed closed-loop delta in percent to a pulse width.
/// Positive increases fuel, negative decreases.
pub fn apply_cl_delta(pw: u16, cl_delta_percent: i16) -> u16 {
    if cl_delta_percent == 0 { return pw; }
    if cl_delta_percent > 0 {
        let m = (100i16 + cl_delta_percent).clamp(0, 200) as u8;
        scale_u16(pw, m)
    } else {
        // Decrease: scale by (100 - |delta|)
        let m = (100i16 - (-cl_delta_percent)).clamp(0, 200) as u8;
        scale_u16(pw, m)
    }
}

/// Correction multipliers (100 = 1.0x)
///
/// All corrections are represented as integers scaled by 100 to avoid
/// floating-point operations. A value of 100 means no correction (1.0x).
///
/// # Examples
/// - 150 = 1.5x (add 50% fuel)
/// - 80 = 0.8x (reduce fuel by 20%)
/// - 100 = 1.0x (no change)
#[derive(Debug, Clone, Copy)]
pub struct Corrections {
    /// Coolant temperature correction
    pub clt: u8,
    /// Intake air temperature correction
    pub iat: u8,
    /// Battery voltage correction (compensates for injector opening time)
    pub vbatt: u8,
}

impl Corrections {
    /// Default corrections (1.0x all - no corrections applied)
    pub const DEFAULT: Self = Self {
        clt: UNITY_CORRECTION,
        iat: UNITY_CORRECTION,
        vbatt: UNITY_CORRECTION,
    };

    /// Create new corrections with specified values
    pub const fn new(clt: u8, iat: u8, vbatt: u8) -> Self {
        Self { clt, iat, vbatt }
    }
}

/// Global ECU state
///
/// Contains all state needed for ECU operation. Designed to be stored
/// in a static variable for access from ISR context.
pub struct EcuState {
    pub rpm: u16,
    pub synced: bool,
    pub tooth_count: u8,
    pub ipw_table: [[u16; 16]; 16],
    pub ignition_table: [[i16; 16]; 16],
    pub corrections: Corrections,
    pub ignition_corrections: ignition::IgnitionCorrections,
    pub battery_voltage_mv: u16,
    pub rev_limiter_config: rev_limiter::RevLimiterConfig,
    pub rev_limiter_state: rev_limiter::RevLimiterState,
    pub tps_percent: u8, // Throttle position (0-100%) (clamped)
    pub map_kpa_x10: u16, // MAP (kPa*10) (clamped)
    pub flood_clear_state: safety::FloodClearState,
    pub sync_loss_tracker: safety::SyncLossTracker,
    pub sensors_cal: sensors::SensorsCal,
    pub sensors_limits: sensors::SensorsLimits,
    pub emergency_trigger_map_oob: bool,
    pub emergency_trigger_tps_oob: bool,
    pub emergency_mode: bool,
    pub diag_map: diag::DiagState,
    pub diag_tps: diag::DiagState,
    pub diag_cam: diag::DiagState,
    pub diag_log: diag::DiagLog<16>,
    pub ae_config: enrichment::AeConfig,
    pub wue_config: enrichment::WueConfig,
    pub ase_config: enrichment::AseConfig,
    pub dfco_config: dfco::DfcoConfig,
    pub idle_config: actuators::IdleConfig,
    pub fan_config: actuators::FanConfig,
    pub cl_config: actuators::ClConfig,
    pub voltage_monitor: safety::VoltageMonitor,
    pub load_failure_config: safety::LoadFailureConfig,
    pub load_failure_tracker: safety::LoadFailureTracker,
    pub plausibility_config: sensors::plausibility::PlausibilityConfig,
    pub plausibility_state: sensors::plausibility::PlausibilityState,
    pub rate_config: sensors::plausibility::RateConfig,
    pub rate_state: sensors::plausibility::RateValidationState,
    pub lambda_config: lambda::LambdaConfig,
    pub lambda_state: lambda::LambdaState,
    pub ltft_manager: lambda::LtftManager,
    pub knock_controller: knock::KnockController,
    pub torque_controller: torque::TorqueController,
    pub inj_angle_btdc_x10: [u16; 16],
    pub tdc_per_cyl_x10: [u16; 16],
    pub tooth0_angle_x10: u16,
    pub cam_missing_timeout_ms: u16,
}

impl EcuState {
    /// Create new ECU state with defaults
    pub const fn new() -> Self {
        Self {
            rpm: 0,
            synced: false,
            tooth_count: 0,
            ipw_table: [[DEFAULT_PULSE_WIDTH_US; 16]; 16],
            ignition_table: [[constants::ignition::DEFAULT_TIMING_BTDC; 16]; 16],
            corrections: Corrections::DEFAULT,
            ignition_corrections: ignition::IgnitionCorrections::DEFAULT,
            battery_voltage_mv: 12500, // 12.5V nominal
            rev_limiter_config: rev_limiter::RevLimiterConfig::DEFAULT,
            rev_limiter_state: rev_limiter::RevLimiterState::new(),
            tps_percent: 0, // Throttle closed (clamped)
            map_kpa_x10: 1000,
            flood_clear_state: safety::FloodClearState::new(),
            sync_loss_tracker: safety::SyncLossTracker::new(),
            sensors_cal: sensors::SensorsCal::default(),
            sensors_limits: sensors::SensorsLimits::default(),
            emergency_trigger_map_oob: false,
            emergency_trigger_tps_oob: false,
            emergency_mode: false,
            diag_map: diag::DiagState::new(),
            diag_tps: diag::DiagState::new(),
            diag_cam: diag::DiagState::new(),
            diag_log: diag::DiagLog::new(),
        ae_config: enrichment::AeConfig::DEFAULT,
            wue_config: enrichment::WueConfig::DEFAULT,
            ase_config: enrichment::AseConfig::DEFAULT,
            dfco_config: dfco::DfcoConfig::DEFAULT,
            idle_config: actuators::IdleConfig::DEFAULT,
            fan_config: actuators::FanConfig::DEFAULT,
            cl_config: actuators::ClConfig::DEFAULT,
            voltage_monitor: safety::VoltageMonitor::new(),
            load_failure_config: safety::LoadFailureConfig::DEFAULT,
            load_failure_tracker: safety::LoadFailureTracker::new(),
            plausibility_config: sensors::plausibility::PlausibilityConfig::DEFAULT,
            plausibility_state: sensors::plausibility::PlausibilityState::new(),
            rate_config: sensors::plausibility::RateConfig::DEFAULT,
            rate_state: sensors::plausibility::RateValidationState::new(),
            lambda_config: lambda::LambdaConfig::DEFAULT,
            lambda_state: lambda::LambdaState::new(),
            ltft_manager: lambda::LtftManager::new(),
            knock_controller: knock::KnockController::new(),
            torque_controller: torque::TorqueController::new(),
            inj_angle_btdc_x10: [0; 16],
            tdc_per_cyl_x10: [0; 16],
            tooth0_angle_x10: 0,
            cam_missing_timeout_ms: 500,
        }
    }

    /// Calculate fuel pulse width with corrections
    ///
    /// Performs the complete fuel calculation:
    /// 1. Table lookup for base pulse width
    /// 2. Apply temperature and voltage corrections
    /// 3. Clamp to valid range
    ///
    /// Uses integer-only arithmetic with saturating operations to prevent overflow.
    ///
    /// # Arguments
    /// * `rpm` - Engine speed in RPM
    /// * `load` - Engine load in kPa (or TPS %)
    ///
    /// # Returns
    /// Final pulse width in microseconds, clamped to MIN/MAX limits
    pub fn calculate_fuel(&self, rpm: u16, load: u16) -> u16 {
        let table = IpwTable {
            rpm_bins: RPM_BINS,
            load_bins: LOAD_BINS,
            values: self.ipw_table,
        };

        // 1. Base lookup
        let mut pw = table.lookup(rpm, load);

        // 2. Apply corrections sequentially with saturation
        pw = scale_u16(pw, self.corrections.clt);
        pw = scale_u16(pw, self.corrections.iat);
        pw = scale_u16(pw, self.corrections.vbatt);

        // 3. Clamp to reasonable range
        pw = pw.clamp(MIN_PULSE_WIDTH_US, MAX_PULSE_WIDTH_US);

        pw
    }

    /// Calculate fuel and apply additional enrichment percentages (WUE/ASE/AE).
    /// Percentages are 0..=100 where 0 means no extra fuel, 20 means +20%.
    pub fn calculate_fuel_with_enrichments(
        &self,
        rpm: u16,
        load: u16,
        wue_percent: u8,
        ase_percent: u8,
        ae_percent: u8,
        cl_delta_percent: i16,
    ) -> u16 {
        let mut pw = self.calculate_fuel(rpm, load);
        // Apply enrichments multiplicatively: pw *= (100 + pct) / 100
        let enrich = |val: u16, pct: u8| -> u16 {
            let mult = (100u16 + pct as u16) as u8; // safe up to 200
            scale_u16(val, mult)
        };
        pw = enrich(pw, wue_percent);
        pw = enrich(pw, ase_percent);
        pw = enrich(pw, ae_percent);
        // Apply closed-loop delta (may increase or decrease)
        pw = apply_cl_delta(pw, cl_delta_percent);
        pw.clamp(MIN_PULSE_WIDTH_US, MAX_PULSE_WIDTH_US)
    }

    /// Deprecated: prefer calculate_fuel_with_enrichments with cl_delta_percent.
    pub fn calculate_fuel_with_enrichments_no_cl(
        &self,
        rpm: u16,
        load: u16,
        wue_percent: u8,
        ase_percent: u8,
        ae_percent: u8,
    ) -> u16 {
        self.calculate_fuel_with_enrichments(rpm, load, wue_percent, ase_percent, ae_percent, 0)
    }

    /// Calculate ignition timing with corrections
    ///
    /// Performs the complete ignition timing calculation:
    /// 1. Table lookup for base timing
    /// 2. Apply temperature and knock corrections
    /// 3. Clamp to safe limits
    ///
    /// # Arguments
    /// * `rpm` - Engine speed in RPM
    /// * `load` - Engine load in kPa
    ///
    /// # Returns
    /// Final timing in degrees BTDC (positive = advance, negative = retard)
    pub fn calculate_ignition_timing(&self, rpm: u16, load: u16) -> i16 {
        let table = ignition::IgnitionTable {
            rpm_bins: constants::fuel::RPM_BINS,
            load_bins: constants::fuel::LOAD_BINS,
            values: self.ignition_table,
        };

        // 1. Base lookup
        let base_timing = table.lookup(rpm, load);

        // 2. Apply corrections and clamp
        ignition::calculate_timing(base_timing, &self.ignition_corrections)
    }

    /// Calculate coil dwell time based on battery voltage
    ///
    /// # Returns
    /// Dwell time in microseconds
    pub fn calculate_dwell(&self) -> u32 {
        ignition::calculate_dwell(self.battery_voltage_mv)
    }

    /// Initialize IPW table with linear test values
    ///
    /// Creates a simple linear fuel map for initial testing.
    /// More fuel at higher load, slightly less at higher RPM.
    ///
    /// This is a helper method for hardware testing. Real tuning data
    /// should be loaded from external storage or CAN.
    pub fn init_linear_table(&mut self) {
        for row in 0..16 {
            for col in 0..16 {
                let base = DEFAULT_PULSE_WIDTH_US;
                let load_factor = (row as u16).saturating_mul(50); // 0-750us
                let rpm_factor = (col as u16).saturating_mul(10); // 0-150us

                // More fuel at higher load, slightly less at higher RPM
                self.ipw_table[row][col] =
                    base.saturating_add(load_factor).saturating_sub(rpm_factor);
            }
        }
    }

    /// Initialize ignition table with conservative values
    ///
    /// Creates a conservative ignition map safe for initial testing.
    /// Should be replaced with properly tuned values for production.
    pub fn init_ignition_table(&mut self) {
        let mut table = ignition::IgnitionTable {
            rpm_bins: constants::fuel::RPM_BINS,
            load_bins: constants::fuel::LOAD_BINS,
            values: self.ignition_table,
        };

        ignition::init_conservative_table(&mut table);
        self.ignition_table = table.values;
    }

    /// Update rev limiter state based on current RPM
    ///
    /// Should be called every engine cycle or in main loop.
    /// Updates internal limiter state which affects fuel and ignition.
    pub fn update_rev_limiter(&mut self) {
        rev_limiter::update_limiter(
            self.rpm,
            &self.rev_limiter_config,
            &mut self.rev_limiter_state,
        );
    }

    /// Check if fuel injection should proceed (considers rev limiter)
    ///
    /// # Arguments
    /// * `cylinder` - Cylinder number (0-3)
    ///
    /// # Returns
    /// `true` if injection should occur, `false` if limiter is cutting fuel
    pub fn should_inject_fuel(&self, cylinder: u8) -> bool {
        rev_limiter::should_inject(&self.rev_limiter_state, cylinder)
    }

    /// Calculate ignition timing with all corrections (including rev limiter)
    ///
    /// This is the main method to use - applies ignition corrections AND rev limiter retard.
    ///
    /// # Arguments
    /// * `rpm` - Engine speed in RPM
    /// * `load` - Engine load in kPa
    ///
    /// # Returns
    /// Final timing in degrees BTDC with all corrections applied
    pub fn calculate_ignition_timing_with_limiter(&self, rpm: u16, load: u16) -> i16 {
        self.calculate_ignition_timing_with_limiter_cyl(rpm, load, 0)
    }

    /// Calculate ignition timing with all corrections (including rev limiter and knock)
    ///
    /// This is the main method to use - applies ignition corrections, rev limiter retard,
    /// and per-cylinder knock retard.
    ///
    /// # Arguments
    /// * `rpm` - Engine speed in RPM
    /// * `load` - Engine load in kPa
    /// * `cylinder` - Cylinder number (0-7) for per-cylinder knock retard
    ///
    /// # Returns
    /// Final timing in degrees BTDC with all corrections applied
    pub fn calculate_ignition_timing_with_limiter_cyl(
        &self,
        rpm: u16,
        load: u16,
        cylinder: u8,
    ) -> i16 {
        // Get base timing with normal corrections
        let base_timing = self.calculate_ignition_timing(rpm, load);

        // Apply rev limiter retard
        let with_limiter = rev_limiter::apply_limiter_retard(base_timing, &self.rev_limiter_state);

        // Apply knock retard (returns negative value)
        let knock_retard = self.knock_controller.get_retard_degrees(cylinder);
        (with_limiter + knock_retard).max(constants::ignition::MIN_TIMING_BTDC)
    }

    /// Update flood clear state based on current conditions
    ///
    /// Should be called every engine cycle or main loop iteration.
    ///
    /// # Returns
    /// `true` if flood clear is active (fuel should be cut)
    pub fn update_flood_clear(&mut self) -> bool {
        safety::update_flood_clear(self.rpm, self.tps_percent, &mut self.flood_clear_state)
    }

    /// Record a sync loss event
    ///
    /// Call this when trigger sync is lost. The tracker will determine
    /// if this is an ESD glitch (recoverable) or real failure (shutdown).
    ///
    /// # Arguments
    /// * `current_time_us` - Current timestamp in microseconds
    ///
    /// # Returns
    /// `true` if engine should shut down, `false` if should attempt recovery
    pub fn record_sync_loss(&mut self, current_time_us: u32) -> bool {
        self.synced = false;
        self.sync_loss_tracker.record_sync_loss(current_time_us)
    }

    /// Record successful sync recovery
    ///
    /// Call this when sync is successfully re-established after a loss.
    pub fn record_sync_recovery(&mut self) {
        self.synced = true;
        self.sync_loss_tracker.record_recovery();
    }

    /// Reset sync loss window after sustained good operation
    ///
    /// Call this periodically (e.g., every 10 seconds) when sync is stable.
    /// This allows the system to recover from old ESD events.
    pub fn reset_sync_loss_window(&mut self) {
        self.sync_loss_tracker.reset_window();
    }

    /// Check if fuel injection should proceed considering ALL safety features
    ///
    /// This is the master safety check. Returns `true` only if:
    /// - Not in flood clear mode
    /// - Not shut down due to sync loss
    /// - Rev limiter allows injection
    /// - Engine is synced
    /// - Voltage is not critically low
    ///
    /// # Arguments
    /// * `cylinder` - Cylinder number (0-3)
    ///
    /// # Returns
    /// `true` if injection should proceed, `false` otherwise
    pub fn should_inject_with_all_safety(&self, cylinder: u8) -> bool {
        // Must be synced
        if !self.synced {
            return false;
        }
        // Emergency mode blocks fuel
        if self.emergency_mode {
            return false;
        }

        // Check voltage - critical low voltage blocks fuel
        if self.voltage_monitor.should_block_fuel() {
            return false;
        }

        // Check flood clear and shutdown
        if !safety::should_allow_injection(
            self.flood_clear_state.active,
            self.sync_loss_tracker.is_shutdown(),
        ) {
            return false;
        }

        // Check rev limiter
        if !self.should_inject_fuel(cylinder) {
            return false;
        }

        true
    }

    /// Update voltage monitor with current battery reading
    ///
    /// Should be called periodically (e.g., every 10-100ms) with ADC reading.
    ///
    /// # Arguments
    /// * `voltage_mv` - Battery voltage in millivolts
    /// * `now_us` - Current timestamp in microseconds
    ///
    /// # Returns
    /// Current power state
    pub fn update_voltage(&mut self, voltage_mv: u16, now_us: u32) -> safety::PowerState {
        let state = self.voltage_monitor.update(voltage_mv, now_us);

        // Update battery_voltage_mv for other calculations (injector dead time, dwell)
        self.battery_voltage_mv = voltage_mv;

        // Log diagnostic events on state transitions
        if state == safety::PowerState::Critical && !self.diag_map.active {
            // Log low voltage event (reusing diag infrastructure)
            self.diag_log.push(diag::DiagEvent {
                code: diag::DiagCode::LowVoltage,
                start_us: now_us,
                end_us: 0, // Will be updated when recovered
            });
        }

        state
    }

    /// Get effective RPM limit considering all sources
    ///
    /// Returns the most restrictive RPM limit from:
    /// - Rev limiter config
    /// - Voltage limp mode
    /// - Load failure limp mode
    pub fn get_effective_rpm_limit(&self) -> u16 {
        let mut limit = self.rev_limiter_config.max_rpm;

        // Apply voltage limp limit if active
        if let Some(voltage_limit) = self.voltage_monitor.get_rpm_limit() {
            limit = limit.min(voltage_limit);
        }

        // Apply load failure limp limit if active
        if let Some(load_limit) = self.load_failure_tracker.get_rpm_limit(&self.load_failure_config)
        {
            limit = limit.min(load_limit);
        }

        limit
    }

    /// Check for load failure condition (MAP fault at high RPM)
    ///
    /// Should be called after process_sensor_update to check if MAP fault
    /// combined with high RPM requires limp mode activation.
    ///
    /// # Arguments
    /// * `now_us` - Current timestamp in microseconds
    ///
    /// # Returns
    /// `true` if in load-failure limp mode
    pub fn check_load_failure(&mut self, now_us: u32) -> bool {
        let map_fault = self.diag_map.active;
        let in_limp = self.load_failure_tracker.check(
            map_fault,
            self.rpm,
            &self.load_failure_config,
            now_us,
        );

        // Log event when entering limp mode
        if in_limp && self.load_failure_tracker.entered_us == now_us {
            self.diag_log.push(diag::DiagEvent {
                code: diag::DiagCode::MapFailureHighLoad,
                start_us: now_us,
                end_us: 0,
            });
        }

        in_limp
    }

    /// Check TPS vs MAP sensor plausibility
    ///
    /// Detects implausible sensor combinations that indicate sensor failure.
    /// Should be called after process_sensor_update.
    ///
    /// # Arguments
    /// * `now_us` - Current timestamp in microseconds
    ///
    /// # Returns
    /// The confirmed plausibility fault (if any)
    pub fn check_plausibility(
        &mut self,
        now_us: u32,
    ) -> sensors::plausibility::PlausibilityFault {
        let old_has_fault = self.plausibility_state.has_fault();

        let fault = self.plausibility_state.check(
            self.tps_percent,
            self.map_kpa_x10,
            self.rpm,
            &self.plausibility_config,
            now_us,
        );

        // Log event when fault is first confirmed
        if self.plausibility_state.has_fault() && !old_has_fault {
            self.diag_log.push(diag::DiagEvent {
                code: diag::DiagCode::TpsMapPlausibility,
                start_us: now_us,
                end_us: 0,
            });
        }

        fault
    }

    /// Check if there's a plausibility fault active
    pub fn has_plausibility_fault(&self) -> bool {
        self.plausibility_state.has_fault()
    }

    /// Validate sensor rate-of-change
    ///
    /// Filters out impossible sensor spikes that indicate noise or failure.
    /// Should be called before process_sensor_update for best filtering.
    ///
    /// # Arguments
    /// * `tps_percent` - Raw TPS reading
    /// * `map_kpa_x10` - Raw MAP reading
    /// * `now_us` - Current timestamp in microseconds
    ///
    /// # Returns
    /// (validated_tps, validated_map) - Filtered values
    pub fn validate_sensor_rates(
        &mut self,
        tps_percent: u8,
        map_kpa_x10: u16,
        now_us: u32,
    ) -> (u8, u16) {
        let (validated_tps, validated_map, _, _) = self.rate_state.validate(
            tps_percent,
            map_kpa_x10,
            &self.rate_config,
            now_us,
        );

        (validated_tps, validated_map)
    }

    /// Check if any sensor rate was rejected in the last update
    pub fn any_rate_rejected(&self) -> bool {
        self.rate_state.any_rejected()
    }

    /// Update LTFT learning
    ///
    /// Call this periodically (e.g., 10Hz) during normal operation.
    /// LTFT will only learn when conditions are stable.
    ///
    /// # Arguments
    /// * `clt_c` - Coolant temperature in Celsius
    /// * `now_us` - Current timestamp in microseconds
    ///
    /// # Returns
    /// Current LTFT value for the operating point (percent x10)
    pub fn update_ltft(&mut self, clt_c: i16, now_us: u32) -> i16 {
        self.ltft_manager.update(
            self.rpm,
            self.map_kpa_x10,
            clt_c,
            self.lambda_state.stft_x10,
            self.lambda_state.active,
            now_us,
        )
    }

    /// Get combined fuel trim (STFT + LTFT)
    ///
    /// # Returns
    /// Combined fuel trim (percent x10), clamped to ±20%
    pub fn get_total_fuel_trim(&self) -> i16 {
        self.ltft_manager.get_total_trim(
            self.lambda_state.stft_x10,
            self.rpm,
            self.map_kpa_x10,
        )
    }

    /// Reset LTFT learning
    ///
    /// Clears all learned values. Use via TunerStudio command or after
    /// major engine changes that invalidate learned data.
    pub fn reset_ltft(&mut self) {
        self.ltft_manager.reset();
    }

    /// Check if LTFT learning is currently active
    pub fn is_ltft_learning(&self) -> bool {
        self.ltft_manager.state.learning_active
    }

    /// Get number of LTFT cells that have been learned
    pub fn ltft_learned_cell_count(&self) -> u8 {
        self.ltft_manager.table.learned_cell_count()
    }

    /// Process a knock sensor sample
    ///
    /// Call this during the knock window with the current sensor reading.
    ///
    /// # Arguments
    /// * `cylinder` - Cylinder index (0-7)
    /// * `level` - Knock sensor reading
    /// * `clt_c` - Coolant temperature in Celsius
    /// * `now_us` - Current timestamp
    ///
    /// # Returns
    /// `true` if knock was detected
    pub fn process_knock_sample(
        &mut self,
        cylinder: u8,
        level: u16,
        clt_c: i16,
        now_us: u32,
    ) -> bool {
        let detected = self.knock_controller.process(cylinder, level, self.rpm, clt_c, now_us);

        // Log knock event to diagnostics
        if detected {
            self.diag_log.push(diag::DiagEvent {
                code: diag::DiagCode::KnockDetected,
                start_us: now_us,
                end_us: 0,
            });
        }

        detected
    }

    /// Update knock timing recovery
    ///
    /// Call this periodically (e.g., every 100ms) to allow timing recovery.
    pub fn update_knock_recovery(&mut self, now_us: u32) {
        self.knock_controller.update_recovery(now_us);
    }

    /// Reset knock controller state
    pub fn reset_knock(&mut self) {
        self.knock_controller.reset();
    }

    /// Check if any knock retard is active
    pub fn has_knock_retard(&self) -> bool {
        self.knock_controller.state.has_retard(&self.knock_controller.config)
    }

    /// Get total knock count across all cylinders
    pub fn total_knock_count(&self) -> u32 {
        self.knock_controller.state.total_knock_count
    }

    /// Update torque controller with current conditions
    ///
    /// Call this periodically (e.g., in main loop) to update torque arbitration.
    ///
    /// # Arguments
    /// * `iat_c` - Intake air temperature in Celsius
    ///
    /// # Returns
    /// Arbitrated torque target (Nm x10)
    pub fn update_torque(&mut self, iat_c: i16) -> i16 {
        self.torque_controller.update(self.rpm, self.map_kpa_x10, iat_c)
    }

    /// Submit a driver torque request based on pedal position
    ///
    /// # Arguments
    /// * `pedal_percent` - Accelerator pedal position (0-100%)
    /// * `now_us` - Current timestamp
    pub fn request_driver_torque(&mut self, pedal_percent: u8, now_us: u32) {
        self.torque_controller.request_driver(pedal_percent, self.rpm, now_us);
    }

    /// Submit an idle controller torque request
    ///
    /// # Arguments
    /// * `target_rpm` - Target idle RPM
    /// * `now_us` - Current timestamp
    pub fn request_idle_torque(&mut self, target_rpm: u16, now_us: u32) {
        self.torque_controller.request_idle(target_rpm, self.rpm, now_us);
    }

    /// Submit torque limits based on current safety states
    ///
    /// Call this after updating safety monitors to apply torque limits.
    pub fn apply_safety_torque_limits(&mut self, now_us: u32) {
        // Rev limiter
        let rev_limited = self.rev_limiter_state.active;
        self.torque_controller.request_rev_limit(rev_limited, now_us);

        // Limp mode from voltage or load failure
        let limp_active = self.voltage_monitor.limp_active
            || self.load_failure_tracker.in_limp;
        self.torque_controller.request_limp(limp_active, now_us);
    }

    /// Get actuator targets from torque controller
    ///
    /// Returns fuel/timing modifications to achieve torque target.
    pub fn get_torque_actuators(&self) -> torque::ActuatorTargets {
        self.torque_controller.get_actuator_targets(self.rpm)
    }

    /// Check if torque is being limited
    pub fn is_torque_limited(&self) -> bool {
        self.torque_controller.is_limited()
    }

    /// Reset torque controller
    pub fn reset_torque(&mut self) {
        self.torque_controller.reset();
    }
}

impl Default for EcuState {
    fn default() -> Self {
        Self::new()
    }
}

impl EcuState {
    /// Clamp sensor values, update diag states, and set/clear emergency mode.
    /// Returns (clamped_map_kpa_x10, clamped_tps_percent).
    pub fn process_sensor_update(
        &mut self,
        now_us: u32,
        raw_map_kpa_x10: u16,
        raw_tps_percent: u8,
    ) -> (u16, u8) {
        let lim = self.sensors_limits;
        let map = raw_map_kpa_x10.clamp(lim.map_min_kpa_x10, lim.map_max_kpa_x10);
        let tps = raw_tps_percent.clamp(lim.tps_min_percent, lim.tps_max_percent);

        // MAP diag
        let map_oob = raw_map_kpa_x10 < lim.map_min_kpa_x10 || raw_map_kpa_x10 > lim.map_max_kpa_x10;
        if map_oob {
            if !self.diag_map.active {
                self.diag_map.active = true;
                self.diag_map.start_us = now_us;
                self.diag_map.in_range_since_us = 0;
                if self.emergency_trigger_map_oob {
                    self.emergency_mode = true;
                }
            }
        } else if self.diag_map.active {
            if self.diag_map.in_range_since_us == 0 {
                self.diag_map.in_range_since_us = now_us;
            }
            let clear_time_us = (lim.clear_time_s as u32) * 1_000_000;
            if now_us.wrapping_sub(self.diag_map.in_range_since_us) >= clear_time_us {
                let dur = now_us.wrapping_sub(self.diag_map.start_us);
                self.diag_map.total_us = self.diag_map.total_us.saturating_add(dur);
                self.diag_log.push(diag::DiagEvent {
                    code: diag::DiagCode::MapRange,
                    start_us: self.diag_map.start_us,
                    end_us: now_us,
                });
                self.diag_map = diag::DiagState::new();
            }
        }

        // TPS diag
        let tps_oob = raw_tps_percent < lim.tps_min_percent || raw_tps_percent > lim.tps_max_percent;
        if tps_oob {
            if !self.diag_tps.active {
                self.diag_tps.active = true;
                self.diag_tps.start_us = now_us;
                self.diag_tps.in_range_since_us = 0;
                if self.emergency_trigger_tps_oob {
                    self.emergency_mode = true;
                }
            }
        } else if self.diag_tps.active {
            if self.diag_tps.in_range_since_us == 0 {
                self.diag_tps.in_range_since_us = now_us;
            }
            let clear_time_us = (lim.clear_time_s as u32) * 1_000_000;
            if now_us.wrapping_sub(self.diag_tps.in_range_since_us) >= clear_time_us {
                let dur = now_us.wrapping_sub(self.diag_tps.start_us);
                self.diag_tps.total_us = self.diag_tps.total_us.saturating_add(dur);
                self.diag_log.push(diag::DiagEvent {
                    code: diag::DiagCode::TpsRange,
                    start_us: self.diag_tps.start_us,
                    end_us: now_us,
                });
                self.diag_tps = diag::DiagState::new();
            }
        }

        // Clear emergency mode if triggers inactive
        if self.emergency_mode {
            let map_emerg_active = self.emergency_trigger_map_oob && self.diag_map.active;
            let tps_emerg_active = self.emergency_trigger_tps_oob && self.diag_tps.active;
            if !(map_emerg_active || tps_emerg_active) {
                self.emergency_mode = false;
            }
        }

        self.map_kpa_x10 = map;
        self.tps_percent = tps;
        (map, tps)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_scale_u16_normal() {
        assert_eq!(scale_u16(1000, 150), 1500); // 1.5x
        assert_eq!(scale_u16(1000, 80), 800); // 0.8x
        assert_eq!(scale_u16(1000, 100), 1000); // 1.0x
        assert_eq!(scale_u16(500, 200), 1000); // 2.0x
    }

    #[test]
    fn test_scale_u16_saturation() {
        // Test overflow protection
        assert_eq!(scale_u16(u16::MAX, 200), u16::MAX); // Would overflow
        assert_eq!(scale_u16(50000, 200), u16::MAX); // Would overflow
    }

    #[test]
    fn test_fuel_calculation_clamping() {
        let mut state = EcuState::new();

        // Test minimum clamping (with very low correction)
        state.corrections.clt = 10; // 0.1x (very low)
        let pw = state.calculate_fuel(3000, 60);
        assert_eq!(pw, MIN_PULSE_WIDTH_US);

        // Test maximum clamping (with very high base value and correction)
        // First set a high base value in the table
        // 3000 RPM maps to RPM bin index 5, 60 kPa maps to load bin index 4
        // Table is [load_idx][rpm_idx]
        state.ipw_table[4][5] = 15000; // 15ms base
        state.corrections.clt = 255; // 2.55x (very high)
        state.corrections.iat = 255;
        state.corrections.vbatt = 255;
        // This should result in: 15000 * 2.55 * 2.55 * 2.55 = 249,146 which exceeds MAX
        let pw = state.calculate_fuel(3000, 60); // Maps to bin [4][5]
        assert_eq!(pw, MAX_PULSE_WIDTH_US);
    }

    #[test]
    fn test_fuel_calculation_normal() {
        let state = EcuState::new();

        // With default corrections (1.0x), should return table value
        let pw = state.calculate_fuel(3000, 60);
        assert_eq!(pw, DEFAULT_PULSE_WIDTH_US);
    }

    #[test]
    fn test_linear_table_initialization() {
        let mut state = EcuState::new();
        state.init_linear_table();

        // Verify table has been populated
        // First cell should be base + 0 - 0
        assert_eq!(state.ipw_table[0][0], DEFAULT_PULSE_WIDTH_US);

        // Last cell should be base + 750 - 150
        let expected = DEFAULT_PULSE_WIDTH_US + 750 - 150;
        assert_eq!(state.ipw_table[15][15], expected);

        // Verify middle cell has reasonable value
        assert!(state.ipw_table[8][8] > DEFAULT_PULSE_WIDTH_US);
    }

    // --- Knock Integration Tests ---

    #[test]
    fn test_ecustate_knock_process_sample() {
        let mut state = EcuState::new();
        state.rpm = 3000;
        state.knock_controller.config.enable = true;
        state.knock_controller.config.threshold = 100;
        state.knock_controller.config.debounce_count = 1; // Immediate detection

        // Below threshold - no knock
        let detected = state.process_knock_sample(0, 50, 80, 1000);
        assert!(!detected);
        assert!(!state.has_knock_retard());

        // Above threshold - knock detected
        let detected = state.process_knock_sample(0, 150, 80, 2000);
        assert!(detected);
        assert!(state.has_knock_retard());

        // Check that diag log contains knock event
        assert!(state.diag_log.events.iter().filter_map(|e| e.as_ref()).any(|e| e.code == diag::DiagCode::KnockDetected));
    }

    #[test]
    fn test_ecustate_knock_affects_timing() {
        let mut state = EcuState::new();
        state.rpm = 3000;
        state.knock_controller.config.enable = true;
        state.knock_controller.config.threshold = 100;
        state.knock_controller.config.debounce_count = 1;
        state.knock_controller.config.retard_step_x10 = 30; // 3 degrees per knock

        // Get base timing
        let base = state.calculate_ignition_timing_with_limiter_cyl(3000, 80, 0);

        // Trigger knock
        state.process_knock_sample(0, 200, 80, 1000);

        // Check timing is retarded
        let after_knock = state.calculate_ignition_timing_with_limiter_cyl(3000, 80, 0);
        assert!(after_knock < base, "Timing should be retarded after knock");
        assert_eq!(base - after_knock, 3, "Should retard by 3 degrees");
    }

    #[test]
    fn test_ecustate_knock_recovery() {
        let mut state = EcuState::new();
        state.rpm = 3000;
        state.knock_controller.config.enable = true;
        state.knock_controller.config.threshold = 100;
        state.knock_controller.config.debounce_count = 1;
        state.knock_controller.config.retard_step_x10 = 50; // 5 degrees
        state.knock_controller.config.recovery_rate_x10 = 100; // 10 degrees/sec for faster test

        // Trigger knock
        state.process_knock_sample(0, 200, 80, 0);
        assert!(state.has_knock_retard());

        // Recover over time (need enough time for 50 x10 units at 100 x10/sec = 0.5 sec)
        // With 100ms intervals, need 5 calls
        for i in 0..6 {
            state.update_knock_recovery(100_000 + i * 100_000);
        }

        // Should have recovered
        assert!(!state.has_knock_retard());
    }

    #[test]
    fn test_ecustate_knock_disabled_conditions() {
        let mut state = EcuState::new();
        state.knock_controller.config.enable = true;
        state.knock_controller.config.threshold = 100;
        state.knock_controller.config.debounce_count = 1; // Immediate detection
        state.knock_controller.config.min_rpm = 2000;
        state.knock_controller.config.min_clt_c = 60;

        // Low RPM - disabled
        state.rpm = 1500;
        let detected = state.process_knock_sample(0, 200, 80, 1000);
        assert!(!detected);

        // Cold engine - disabled
        state.rpm = 3000;
        let detected = state.process_knock_sample(0, 200, 50, 2000);
        assert!(!detected);

        // Warm engine, good RPM - enabled
        let detected = state.process_knock_sample(0, 200, 80, 3000);
        assert!(detected);
    }

    // --- LTFT Integration Tests ---

    #[test]
    fn test_ecustate_ltft_learning() {
        let mut state = EcuState::new();
        state.rpm = 2500;
        state.map_kpa_x10 = 600;
        state.lambda_state.active = true;
        state.lambda_state.stft_x10 = 30; // 3% rich
        state.ltft_manager.config.enable = true;

        // Initial trim should be 0
        let trim = state.get_total_fuel_trim();
        assert_eq!(trim, 30); // Just STFT

        // Update LTFT several times with steady conditions
        for i in 0..20 {
            state.update_ltft(80, i * 1_000_000);
        }

        // Should be learning
        assert!(state.is_ltft_learning() || state.ltft_learned_cell_count() > 0);
    }

    #[test]
    fn test_ecustate_ltft_disabled_cold() {
        let mut state = EcuState::new();
        state.rpm = 2500;
        state.map_kpa_x10 = 600;
        state.lambda_state.active = true;
        state.lambda_state.stft_x10 = 30;
        state.ltft_manager.config.enable = true;
        state.ltft_manager.config.min_clt_c = 70;

        // Cold engine - LTFT should not learn
        for i in 0..20 {
            state.update_ltft(50, i * 1_000_000);
        }

        assert_eq!(state.ltft_learned_cell_count(), 0);
    }

    #[test]
    fn test_ecustate_ltft_reset() {
        let mut state = EcuState::new();
        state.rpm = 2500;
        state.map_kpa_x10 = 600;
        state.lambda_state.active = true;
        state.lambda_state.stft_x10 = 30;
        state.ltft_manager.config.enable = true;

        // Learn for a while
        for i in 0..20 {
            state.update_ltft(80, i * 1_000_000);
        }

        // Reset
        state.reset_ltft();

        // All cells should be cleared
        assert_eq!(state.ltft_learned_cell_count(), 0);
    }

    // --- Torque Integration Tests ---

    #[test]
    fn test_ecustate_torque_driver_request() {
        let mut state = EcuState::new();
        state.rpm = 3000;
        state.map_kpa_x10 = 800;

        // First update to get max available
        state.update_torque(25);

        // Driver pedal at 50%
        state.request_driver_torque(50, 1000);
        let torque = state.update_torque(25);

        // Should have ~50% of max available
        assert!(torque > 0);
        assert!(torque <= state.torque_controller.max_available_x10);
    }

    #[test]
    fn test_ecustate_torque_safety_limits() {
        let mut state = EcuState::new();
        state.rpm = 3000;
        state.map_kpa_x10 = 800;

        // Update and request full power
        state.update_torque(25);
        state.request_driver_torque(100, 1000);
        let full_power = state.update_torque(25);

        // Activate rev limiter
        state.rev_limiter_state.active = true;
        state.apply_safety_torque_limits(2000);
        let limited = state.update_torque(25);

        // Should be severely limited
        assert!(limited < full_power, "Rev limiter should limit torque");
        assert!(state.is_torque_limited());
    }

    #[test]
    fn test_ecustate_torque_actuators() {
        let mut state = EcuState::new();
        state.rpm = 3000;
        state.map_kpa_x10 = 800;

        // Update and request 50%
        state.update_torque(25);
        state.request_driver_torque(50, 1000);
        state.update_torque(25);

        let targets = state.get_torque_actuators();

        // Should have some fuel reduction or timing retard if limited
        // At 50% pedal with full MAP, driver usually gets what they want
        assert!(targets.fuel_mult_x100 <= 100);
    }

    #[test]
    fn test_ecustate_torque_zero_rpm() {
        let mut state = EcuState::new();
        state.rpm = 0;
        state.map_kpa_x10 = 800;

        // Should handle 0 RPM gracefully
        let torque = state.update_torque(25);
        assert_eq!(torque, 0); // No torque at 0 RPM
    }

    // --- Cross-Module Integration Tests ---

    #[test]
    fn test_integration_knock_reduces_torque() {
        let mut state = EcuState::new();
        state.rpm = 3000;
        state.map_kpa_x10 = 800;
        state.knock_controller.config.enable = true;
        state.knock_controller.config.threshold = 100;
        state.knock_controller.config.debounce_count = 1;
        state.knock_controller.config.retard_step_x10 = 50; // 5 degrees

        // Update torque to get max available
        state.update_torque(25);
        state.request_driver_torque(100, 1000);
        let base_torque = state.update_torque(25);

        // Trigger knock
        state.process_knock_sample(0, 200, 80, 2000);
        assert!(state.has_knock_retard());

        // Submit knock-based torque request
        let knock_retard = state.knock_controller.state.get_retard(0);
        state.torque_controller.arbiter.request(
            torque::request::knock_torque_request(knock_retard, state.torque_controller.max_available_x10, 3000)
        );
        let reduced_torque = state.torque_controller.arbiter.arbitrate(state.torque_controller.max_available_x10);

        // Knock should reduce available torque
        assert!(reduced_torque < base_torque, "Knock should reduce arbitrated torque");
    }

    #[test]
    fn test_integration_torque_affects_actuators() {
        let mut state = EcuState::new();
        state.rpm = 3000;
        state.map_kpa_x10 = 800;

        // Full power request
        state.update_torque(25);
        state.request_driver_torque(100, 1000);
        state.update_torque(25);

        let full_power_targets = state.get_torque_actuators();

        // Activate rev limiter
        state.rev_limiter_state.active = true;
        state.apply_safety_torque_limits(2000);
        state.update_torque(25);

        let limited_targets = state.get_torque_actuators();

        // Rev limiter should cause actuator changes
        assert!(
            limited_targets.fuel_cut || limited_targets.timing_reduced ||
            limited_targets.fuel_mult_x100 < full_power_targets.fuel_mult_x100,
            "Rev limiter should cause actuator intervention"
        );
    }

    #[test]
    fn test_integration_limp_mode_propagation() {
        let mut state = EcuState::new();
        state.rpm = 3000;
        state.map_kpa_x10 = 800;

        // Full power request
        state.update_torque(25);
        state.request_driver_torque(100, 1000);
        let full_power = state.update_torque(25);

        // Trigger load failure -> limp mode
        state.load_failure_tracker.in_limp = true;
        state.apply_safety_torque_limits(2000);
        let limp_torque = state.update_torque(25);

        // Limp mode should limit torque to ~30%
        assert!(limp_torque < full_power / 2, "Limp mode should severely limit torque");
        assert!(state.is_torque_limited());
    }

    #[test]
    fn test_integration_multiple_safety_systems() {
        let mut state = EcuState::new();
        state.rpm = 6500;
        state.map_kpa_x10 = 900;
        state.knock_controller.config.enable = true;
        state.knock_controller.config.threshold = 100;
        state.knock_controller.config.debounce_count = 1;
        state.rev_limiter_config.max_rpm = 6500;

        // Driver wants full power
        state.update_torque(25);
        state.request_driver_torque(100, 1000);

        // Trigger knock
        state.process_knock_sample(0, 200, 80, 2000);

        // Update rev limiter (at hard limit)
        rev_limiter::update_limiter(state.rpm, &state.rev_limiter_config, &mut state.rev_limiter_state);

        // Apply safety limits
        state.apply_safety_torque_limits(3000);

        // Get final timing with all corrections
        let final_timing = state.calculate_ignition_timing_with_limiter_cyl(state.rpm, 80, 0);
        let base_timing = state.calculate_ignition_timing(state.rpm, 80);

        // Timing should be reduced by both knock and rev limiter
        assert!(final_timing < base_timing, "Safety systems should reduce timing");
    }

    #[test]
    fn test_integration_lambda_ltft_combined_trim() {
        let mut state = EcuState::new();
        state.rpm = 2500;
        state.map_kpa_x10 = 600;
        state.lambda_state.active = true;
        state.lambda_state.stft_x10 = 30; // 3% STFT
        state.ltft_manager.config.enable = true;

        // Pre-learn some LTFT (stft=20, rate=50, max_trim=200)
        state.ltft_manager.table.learn(state.rpm, state.map_kpa_x10, 20, 50, 200);

        // Get combined trim
        let total_trim = state.get_total_fuel_trim();

        // Should combine STFT + LTFT
        assert!(total_trim > 30, "Combined trim should include LTFT");
        assert!(total_trim <= 200, "Combined trim should be clamped");
    }

    #[test]
    fn test_integration_sync_loss_disables_injection() {
        let mut state = EcuState::new();
        state.rpm = 3000;
        state.synced = true;

        // Should allow injection when synced
        assert!(state.should_inject_with_all_safety(0));

        // Record sync loss
        let should_shutdown = state.record_sync_loss(1000);

        if !should_shutdown {
            // First loss doesn't shutdown, but should still not inject
            assert!(!state.synced);
            assert!(!state.should_inject_with_all_safety(0),
                "Should not inject without sync");
        }
    }

    #[test]
    fn test_integration_voltage_affects_safety() {
        let mut state = EcuState::new();
        state.rpm = 3000;
        state.synced = true;

        // Normal voltage - should inject
        assert!(state.should_inject_with_all_safety(0));

        // Critical low voltage
        state.voltage_monitor.limp_active = true;

        // Apply safety torque limits
        state.update_torque(25);
        state.request_driver_torque(100, 1000);
        state.apply_safety_torque_limits(2000);
        let limited_torque = state.update_torque(25);

        // Limp mode should be active
        assert!(state.is_torque_limited());
    }
}
