use super::EcuState;
use crate::compat_state::{DiagnosticFlags, RuntimeSignals, SafetyStatus};

impl EcuState {
    /// Current injector pulse width for a given rpm/load lookup.
    ///
    /// Returns the table-lookup PW before corrections are applied.
    pub fn injection_pulse_width(&self, rpm: u16, load: u16) -> u16 {
        self.calculate_fuel(rpm, load)
    }

    /// Current ignition dwell time in microseconds.
    pub fn ignition_dwell_us(&self) -> u32 {
        self.calculate_dwell()
    }

    /// Current ignition advance for a given rpm/load in degrees BTDC.
    ///
    /// Returns advance before per-cylinder limiting.
    pub fn ignition_advance_deg(&self, rpm: u16, load: u16) -> i16 {
        self.calculate_ignition_timing(rpm, load)
    }

    /// Runtime mirror of the current scalar inputs exposed on `EcuState`.
    pub fn runtime_signals(&self) -> RuntimeSignals {
        RuntimeSignals {
            rpm: self.rpm,
            synced: self.synced,
            tooth_count: self.tooth_count,
            battery_voltage_mv: self.inputs.battery_voltage_mv,
            clt_x10: self.clt_x10,
            iat_x10: self.iat_x10,
            tps_percent: self.tps_percent,
            map_kpa_x10: self.map_kpa_x10,
        }
    }

    /// Whether the legacy public scalar fields still match the split runtime
    /// input mirror.
    ///
    /// The public scalar fields remain for compatibility. New code should use
    /// the setter/accessor methods so both representations stay synchronized.
    pub fn runtime_signal_mirrors_consistent(&self) -> bool {
        self.runtime_signals()
            == RuntimeSignals {
                rpm: self.inputs.rpm,
                synced: self.inputs.synced,
                tooth_count: self.inputs.tooth_count,
                battery_voltage_mv: self.inputs.battery_voltage_mv,
                clt_x10: self.inputs.clt_x10,
                iat_x10: self.inputs.iat_x10,
                tps_percent: self.inputs.tps_percent,
                map_kpa_x10: self.inputs.map_kpa_x10,
            }
    }

    /// Current RPM.
    pub fn current_rpm(&self) -> u16 {
        self.runtime_signals().rpm
    }

    /// Current sync state.
    pub fn current_synced(&self) -> bool {
        self.runtime_signals().synced
    }

    /// Diagnostic fault flags that back the legacy tuple accessor.
    pub fn diagnostic_flags(&self) -> DiagnosticFlags {
        DiagnosticFlags {
            emergency_trigger_map_oob: self.faults.emergency_trigger_map_oob,
            emergency_trigger_tps_oob: self.faults.emergency_trigger_tps_oob,
            emergency_mode: self.faults.emergency_mode,
        }
    }

    /// Active fault flags: (emap_oob, etps_oob, emode).
    pub fn current_fault_flags(&self) -> (bool, bool, bool) {
        let flags = self.diagnostic_flags();
        (
            flags.emergency_trigger_map_oob,
            flags.emergency_trigger_tps_oob,
            flags.emergency_mode,
        )
    }

    /// Safety cut state derived from limiter and cut logic.
    pub fn safety_status(&self) -> SafetyStatus {
        SafetyStatus {
            fuel_cut_active: !self.should_inject_fuel(0),
            spark_cut_active: self.rev_limiter_state.ignition_retard != 0,
        }
    }

    /// Fuel cut active — true if rev limiter or other safety is cutting fuel.
    pub fn fuel_cut_active(&self) -> bool {
        self.safety_status().fuel_cut_active
    }

    /// Spark cut active — true if rev limiter or DFCO is cutting ignition.
    pub fn spark_cut_active(&self) -> bool {
        self.safety_status().spark_cut_active
    }
}
