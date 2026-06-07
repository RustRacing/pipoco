use super::EcuState;
use crate::constants::fuel::{LOAD_BINS, MAX_PULSE_WIDTH_US, MIN_PULSE_WIDTH_US, RPM_BINS};
use crate::fuel_state::{apply_cl_delta, scale_u16};
use crate::tables::IpwTable;
use crate::units::{Kpa10, Micros, Rpm};

impl EcuState {
    pub fn refresh_enrichments(
        &mut self,
        now_us: u32,
        clt_x10: i16,
        iat_x10: i16,
        tps_percent: u8,
        map_kpa_x10: u16,
    ) {
        let clt_c = clt_x10 / 10;
        let _iat_c = iat_x10 / 10;

        self.derived.wue_percent = self.config.wue_config.compute_percent(clt_c);

        let first_tick = self.inputs.last_enrichment_update_us == 0;
        let tpsdot_pct_s = if first_tick {
            0
        } else {
            let dt_us = now_us
                .wrapping_sub(self.inputs.last_enrichment_update_us)
                .max(1);
            let dt_s = dt_us as i64;
            let delta = tps_percent as i64 - self.inputs.last_enrichment_tps_percent as i64;
            ((delta * 1_000_000) / dt_s) as i16
        };
        let mapdot_kpa_s = if first_tick {
            0
        } else {
            let dt_us = now_us
                .wrapping_sub(self.inputs.last_enrichment_update_us)
                .max(1);
            let dt_s = dt_us as i64;
            let delta = map_kpa_x10 as i64 - self.inputs.last_enrichment_map_kpa_x10 as i64;
            ((delta * 1_000_000) / dt_s) as i16
        };

        self.derived.ae_percent =
            self.ae_state
                .update(now_us, tpsdot_pct_s, mapdot_kpa_s, &self.config.ae_config);

        let trigger_inputs = self.trigger_inputs();
        let just_started = first_tick && trigger_inputs.synced && trigger_inputs.rpm > 0;
        self.derived.ase_percent =
            self.ase_state
                .update(now_us, just_started, &self.config.ase_config);

        self.inputs.last_enrichment_update_us = now_us;
        self.inputs.last_enrichment_tps_percent = tps_percent;
        self.inputs.last_enrichment_map_kpa_x10 = map_kpa_x10;
    }

    /// Apply the current torque arbitration result to cached outputs.
    pub fn apply_torque_result(&mut self, result: &crate::torque::arbiter::TorqueResult) {
        self.set_fuel_mult_x100(result.fuel_mult_x100);
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
            values: self.config.ipw_table,
        };

        // 1. Base lookup
        let mut pw = table.lookup(rpm, load);

        // 2. Apply corrections sequentially with saturation
        pw = scale_u16(pw, self.corrections().clt);
        pw = scale_u16(pw, self.corrections().iat);
        pw = scale_u16(pw, self.corrections().vbatt);

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

    /// Calculate the final injector pulse width after enrichment and torque trims.
    pub fn final_pw(&self, rpm: Rpm, load: Kpa10) -> Micros {
        let base = self.calculate_fuel(rpm.raw(), load.raw()) as u32;

        let enrich_mult_x100 = [
            self.derived.wue_percent,
            self.derived.ase_percent,
            self.derived.ae_percent,
        ]
        .into_iter()
        .fold(100u32, |acc, pct| {
            acc.saturating_mul(100 + pct as u32) / 100
        });

        let e = base.saturating_mul(enrich_mult_x100) / 100;
        let stft = self.stft_x10() as i32;
        let l = if stft >= 0 {
            e.saturating_add(e.saturating_mul(stft as u32) / 1000)
        } else {
            e.saturating_sub(e.saturating_mul((-stft) as u32) / 1000)
        };
        let t = l.saturating_mul(self.fuel_mult_x100() as u32) / 100;

        Micros::new(t.clamp(MIN_PULSE_WIDTH_US as u32, MAX_PULSE_WIDTH_US as u32))
    }
}
