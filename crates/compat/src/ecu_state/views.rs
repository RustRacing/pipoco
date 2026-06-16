use super::EcuState;
use crate::compat_state::{CoreObservedSurface, DiagnosticFlags, RuntimeSignals, SafetyStatus};
use crate::constants;
use crate::ignition::IgnitionTable;
use crate::scale_u16;

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

    /// FM0016-facing observed surface with the cached output shell refreshed
    /// through the product-owned snapshot path first.
    pub fn refreshed_observed_surface(&mut self) -> CoreObservedSurface {
        self.refresh_snapshot();
        self.observed_surface()
    }

    /// FM0016-facing observed surface for the root compatibility boundary.
    pub fn observed_surface(&self) -> CoreObservedSurface {
        let runtime = self.runtime_signals();
        let load = runtime.map_kpa_x10;
        let base_pw_us = self.injection_pulse_width(runtime.rpm, load);
        let corrections = self.corrections();
        let ignition_table = IgnitionTable {
            rpm_bins: constants::fuel::RPM_BINS,
            load_bins: constants::fuel::LOAD_BINS,
            values: self.config.ignition_table,
        };
        let spark_base_timing_deg = ignition_table.lookup(runtime.rpm, load);
        let final_pw_us = scale_u16(
            scale_u16(scale_u16(base_pw_us, corrections.clt), corrections.iat),
            corrections.vbatt,
        );
        let ignition_corrections = self.ignition_corrections();
        let diagnostic = self.diagnostic_flags();
        let safety = self.safety_status();
        let inject_fuel_allowed = self.should_inject_fuel(0);

        CoreObservedSurface {
            rpm: runtime.rpm,
            synced: runtime.synced,
            tooth_count: runtime.tooth_count,
            battery_voltage_mv: runtime.battery_voltage_mv,
            clt_x10: runtime.clt_x10,
            iat_x10: runtime.iat_x10,
            tps_percent: runtime.tps_percent,
            map_kpa_x10: runtime.map_kpa_x10,
            last_enrichment_update_us: self.last_enrichment_update_us(),
            last_enrichment_tps_percent: self.last_enrichment_tps_percent(),
            last_enrichment_map_kpa_x10: self.last_enrichment_map_kpa_x10(),
            base_pw_us,
            final_pw_us,
            final_pw_output_us: self.final_pw_output().raw(),
            fuel_mult_x100: self.fuel_mult_x100(),
            wue_percent: self.wue_percent(),
            ase_percent: self.ase_percent(),
            ae_percent: self.ae_percent(),
            enrich_mult_x100: self.snapshot.enrich_mult_x100,
            stft_x10: self.stft_x10(),
            clt_enrich_pct: corrections.clt as u16,
            iat_enrich_pct: corrections.iat as u16,
            vbatt_enrich_pct: corrections.vbatt as u16,
            ign_clt_correction_deg: ignition_corrections.clt_correction,
            ign_iat_correction_deg: ignition_corrections.iat_correction,
            ign_knock_retard_deg: ignition_corrections.knock_retard,
            spark_base_timing_deg,
            spark_advance_deg: self.ignition_advance_deg(runtime.rpm, load),
            spark_advance_with_limiter_deg: self
                .calculate_ignition_timing_with_limiter(runtime.rpm, load),
            commanded_advance_x10_output: self.commanded_advance_x10_output(),
            dwell_us: self.ignition_dwell_us(),
            rev_limiter_active: self.rev_limiter_state.active,
            rev_limiter_fuel_cut_pct: self.rev_limiter_state.fuel_cut_percent,
            rev_limiter_ign_retard_deg: self.rev_limiter_state.ignition_retard,
            ltft_learning: self.is_ltft_learning(),
            ltft_learned_cell_count: self.ltft_learned_cell_count(),
            knock_retard_active: self.has_knock_retard(),
            total_knock_count: self.total_knock_count(),
            torque_limited: self.is_torque_limited(),
            emergency_trigger_map_oob: diagnostic.emergency_trigger_map_oob,
            emergency_trigger_tps_oob: diagnostic.emergency_trigger_tps_oob,
            emergency_mode: diagnostic.emergency_mode,
            last_fault_code: self.snapshot.last_fault_code,
            isr_count: self.snapshot.isr_count,
            isr_max_us: self.snapshot.isr_max_us,
            isr_avg_us: self.snapshot.isr_avg_us,
            has_fault: diagnostic.emergency_trigger_map_oob
                || diagnostic.emergency_trigger_tps_oob
                || diagnostic.emergency_mode
                || self.snapshot.last_fault_code != 0,
            inject_fuel_allowed,
            fuel_cut: safety.fuel_cut_active,
            spark_cut: safety.spark_cut_active,
        }
    }
}
