use ecu_calibration::sensors::SensorsCal;

use super::EcuState;
use crate::compat_state::EcuInputs;
use crate::fuel_state::Corrections;
use crate::units::Micros;
use crate::{actuators, dfco, diag, enrichment, ignition, lambda, rev_limiter, safety, sensors};

impl EcuState {
    pub fn corrections(&self) -> &Corrections {
        &self.corrections
    }

    pub fn corrections_mut(&mut self) -> &mut Corrections {
        &mut self.corrections
    }

    pub fn ipw_table(&self) -> &[[u16; 16]; 16] {
        &self.config.ipw_table
    }

    pub fn ipw_table_mut(&mut self) -> &mut [[u16; 16]; 16] {
        &mut self.config.ipw_table
    }

    pub fn ignition_table(&self) -> &[[i16; 16]; 16] {
        &self.config.ignition_table
    }

    pub fn ignition_table_mut(&mut self) -> &mut [[i16; 16]; 16] {
        &mut self.config.ignition_table
    }

    pub fn sensors_cal(&self) -> &SensorsCal {
        &self.config.sensors_cal
    }

    pub fn sensors_cal_mut(&mut self) -> &mut SensorsCal {
        &mut self.config.sensors_cal
    }

    pub fn sensors_limits(&self) -> &sensors::SensorsLimits {
        &self.config.sensors_limits
    }

    pub fn sensors_limits_mut(&mut self) -> &mut sensors::SensorsLimits {
        &mut self.config.sensors_limits
    }

    pub fn ae_config(&self) -> &enrichment::AeConfig {
        &self.config.ae_config
    }

    pub fn ae_config_mut(&mut self) -> &mut enrichment::AeConfig {
        &mut self.config.ae_config
    }

    pub fn wue_config(&self) -> &enrichment::WueConfig {
        &self.config.wue_config
    }

    pub fn wue_config_mut(&mut self) -> &mut enrichment::WueConfig {
        &mut self.config.wue_config
    }

    pub fn ase_config(&self) -> &enrichment::AseConfig {
        &self.config.ase_config
    }

    pub fn ase_config_mut(&mut self) -> &mut enrichment::AseConfig {
        &mut self.config.ase_config
    }

    pub fn dfco_config(&self) -> &dfco::DfcoConfig {
        &self.config.dfco_config
    }

    pub fn dfco_config_mut(&mut self) -> &mut dfco::DfcoConfig {
        &mut self.config.dfco_config
    }

    pub fn idle_config(&self) -> &actuators::IdleConfig {
        &self.config.idle_config
    }

    pub fn idle_config_mut(&mut self) -> &mut actuators::IdleConfig {
        &mut self.config.idle_config
    }

    pub fn fan_config(&self) -> &actuators::FanConfig {
        &self.config.fan_config
    }

    pub fn fan_config_mut(&mut self) -> &mut actuators::FanConfig {
        &mut self.config.fan_config
    }

    pub fn cl_config(&self) -> &actuators::ClConfig {
        &self.config.cl_config
    }

    pub fn cl_config_mut(&mut self) -> &mut actuators::ClConfig {
        &mut self.config.cl_config
    }

    pub fn load_failure_config(&self) -> &safety::LoadFailureConfig {
        &self.config.load_failure_config
    }

    pub fn load_failure_config_mut(&mut self) -> &mut safety::LoadFailureConfig {
        &mut self.config.load_failure_config
    }

    pub fn plausibility_config(&self) -> &sensors::plausibility::PlausibilityConfig {
        &self.config.plausibility_config
    }

    pub fn plausibility_config_mut(&mut self) -> &mut sensors::plausibility::PlausibilityConfig {
        &mut self.config.plausibility_config
    }

    pub fn rate_config(&self) -> &sensors::plausibility::RateConfig {
        &self.config.rate_config
    }

    pub fn rate_config_mut(&mut self) -> &mut sensors::plausibility::RateConfig {
        &mut self.config.rate_config
    }

    pub fn lambda_config(&self) -> &lambda::LambdaConfig {
        &self.config.lambda_config
    }

    pub fn lambda_config_mut(&mut self) -> &mut lambda::LambdaConfig {
        &mut self.config.lambda_config
    }

    pub fn ignition_corrections(&self) -> &ignition::IgnitionCorrections {
        &self.ignition_corrections
    }

    pub fn ignition_corrections_mut(&mut self) -> &mut ignition::IgnitionCorrections {
        &mut self.ignition_corrections
    }

    pub fn rev_limiter_config(&self) -> &rev_limiter::RevLimiterConfig {
        &self.config.rev_limiter_config
    }

    pub fn rev_limiter_config_mut(&mut self) -> &mut rev_limiter::RevLimiterConfig {
        &mut self.config.rev_limiter_config
    }

    pub fn inj_angle_btdc_x10(&self) -> &[u16; 16] {
        &self.config.inj_angle_btdc_x10
    }

    pub fn inj_angle_btdc_x10_mut(&mut self) -> &mut [u16; 16] {
        &mut self.config.inj_angle_btdc_x10
    }

    pub fn tdc_per_cyl_x10(&self) -> &[u16; 16] {
        &self.config.tdc_per_cyl_x10
    }

    pub fn tdc_per_cyl_x10_mut(&mut self) -> &mut [u16; 16] {
        &mut self.config.tdc_per_cyl_x10
    }

    pub fn tooth0_angle_x10(&self) -> u16 {
        self.config.tooth0_angle_x10
    }

    pub fn tooth0_angle_x10_mut(&mut self) -> &mut u16 {
        &mut self.config.tooth0_angle_x10
    }

    pub fn cam_missing_timeout_ms(&self) -> u16 {
        self.config.cam_missing_timeout_ms
    }

    pub fn cam_missing_timeout_ms_mut(&mut self) -> &mut u16 {
        &mut self.config.cam_missing_timeout_ms
    }

    pub fn rpm(&self) -> u16 {
        self.runtime_signals().rpm
    }

    pub fn synced(&self) -> bool {
        self.runtime_signals().synced
    }

    pub fn tooth_count(&self) -> u8 {
        self.runtime_signals().tooth_count
    }

    pub(crate) fn trigger_inputs(&self) -> EcuInputs {
        self.inputs
    }

    pub fn set_rpm(&mut self, rpm: u16) {
        self.rpm = rpm;
        self.inputs.rpm = rpm;
    }

    pub fn set_synced(&mut self, synced: bool) {
        self.synced = synced;
        self.inputs.synced = synced;
    }

    pub fn set_tooth_count(&mut self, tooth_count: u8) {
        self.tooth_count = tooth_count;
        self.inputs.tooth_count = tooth_count;
    }

    pub fn battery_voltage_mv(&self) -> u16 {
        self.runtime_signals().battery_voltage_mv
    }

    pub fn set_battery_voltage_mv(&mut self, battery_voltage_mv: u16) {
        self.inputs.battery_voltage_mv = battery_voltage_mv;
    }

    pub fn last_enrichment_update_us(&self) -> u32 {
        self.inputs.last_enrichment_update_us
    }

    pub fn last_enrichment_tps_percent(&self) -> u8 {
        self.inputs.last_enrichment_tps_percent
    }

    pub fn last_enrichment_map_kpa_x10(&self) -> u16 {
        self.inputs.last_enrichment_map_kpa_x10
    }

    pub fn wue_percent(&self) -> u8 {
        self.derived.wue_percent
    }

    pub fn ase_percent(&self) -> u8 {
        self.derived.ase_percent
    }

    pub fn ae_percent(&self) -> u8 {
        self.derived.ae_percent
    }

    pub fn stft_x10(&self) -> i16 {
        self.lambda_state.stft_x10
    }

    pub fn set_stft_x10(&mut self, stft_x10: i16) {
        self.derived.stft_x10 = stft_x10;
        self.lambda_state.stft_x10 = stft_x10;
    }

    pub fn ltft_manager(&self) -> &lambda::LtftManager {
        &self.derived.ltft_manager
    }

    pub fn ltft_manager_mut(&mut self) -> &mut lambda::LtftManager {
        &mut self.derived.ltft_manager
    }

    pub fn fuel_mult_x100(&self) -> u16 {
        self.derived.fuel_mult_x100
    }

    pub fn set_fuel_mult_x100(&mut self, fuel_mult_x100: u16) {
        self.derived.fuel_mult_x100 = fuel_mult_x100;
    }

    pub fn final_pw_output(&self) -> Micros {
        self.outputs.final_pw
    }

    pub fn commanded_advance_x10_output(&self) -> i16 {
        self.outputs.commanded_advance_x10
    }

    pub fn set_commanded_advance_x10_output(&mut self, value: i16) {
        self.outputs.commanded_advance_x10 = value;
    }

    pub fn emergency_trigger_map_oob(&self) -> bool {
        self.faults.emergency_trigger_map_oob
    }

    pub fn emergency_trigger_map_oob_mut(&mut self) -> &mut bool {
        &mut self.faults.emergency_trigger_map_oob
    }

    pub fn set_emergency_trigger_map_oob(&mut self, value: bool) {
        self.faults.emergency_trigger_map_oob = value;
    }

    pub fn emergency_trigger_tps_oob(&self) -> bool {
        self.faults.emergency_trigger_tps_oob
    }

    pub fn emergency_trigger_tps_oob_mut(&mut self) -> &mut bool {
        &mut self.faults.emergency_trigger_tps_oob
    }

    pub fn set_emergency_trigger_tps_oob(&mut self, value: bool) {
        self.faults.emergency_trigger_tps_oob = value;
    }

    pub fn emergency_mode(&self) -> bool {
        self.faults.emergency_mode
    }

    pub fn emergency_mode_ref(&self) -> &bool {
        &self.faults.emergency_mode
    }

    pub fn emergency_mode_mut(&mut self) -> &mut bool {
        &mut self.faults.emergency_mode
    }

    pub fn set_emergency_mode(&mut self, value: bool) {
        self.faults.emergency_mode = value;
    }

    pub fn diag_log(&self) -> &diag::DiagLog<16> {
        &self.faults.diag_log
    }

    pub fn diag_log_mut(&mut self) -> &mut diag::DiagLog<16> {
        &mut self.faults.diag_log
    }

    pub fn set_trigger_inputs(&mut self, rpm: u16, synced: bool, tooth_count: u8) {
        self.rpm = rpm;
        self.synced = synced;
        self.tooth_count = tooth_count;
        self.inputs.rpm = rpm;
        self.inputs.synced = synced;
        self.inputs.tooth_count = tooth_count;
    }

    pub fn clt_x10(&self) -> i16 {
        self.runtime_signals().clt_x10
    }

    pub fn iat_x10(&self) -> i16 {
        self.runtime_signals().iat_x10
    }

    pub fn set_clt_x10(&mut self, clt_x10: i16) {
        self.clt_x10 = clt_x10;
        self.inputs.clt_x10 = clt_x10;
    }

    pub fn set_iat_x10(&mut self, iat_x10: i16) {
        self.iat_x10 = iat_x10;
        self.inputs.iat_x10 = iat_x10;
    }

    pub fn tps_percent(&self) -> u8 {
        self.runtime_signals().tps_percent
    }

    pub fn map_kpa_x10(&self) -> u16 {
        self.runtime_signals().map_kpa_x10
    }

    pub fn set_tps_percent(&mut self, tps_percent: u8) {
        self.tps_percent = tps_percent;
        self.inputs.tps_percent = tps_percent;
    }

    pub fn set_map_kpa_x10(&mut self, map_kpa_x10: u16) {
        self.map_kpa_x10 = map_kpa_x10;
        self.inputs.map_kpa_x10 = map_kpa_x10;
    }
}
