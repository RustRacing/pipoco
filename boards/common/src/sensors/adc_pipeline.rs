use ecu_calibration::sensors::convert::{counts_to_mv, DividerConfig};
use ecu_calibration::sensors::curve::Piecewise;
use ecu_calibration::sensors::model::{CalibratedSensor, ResistiveSensor, VoltageSensor};
use ecu_calibration::sensors::SensorsCal;

#[derive(Copy, Clone)]
pub struct ThermistorBias {
    pub known_ohms: u32,
    pub divider: DividerConfig,
}

impl ThermistorBias {
    pub const PULLUP_2490: Self = Self {
        known_ohms: 2_490,
        divider: DividerConfig::PullupTop,
    };
}

#[derive(Copy, Clone)]
pub struct LambdaAdcCalibration {
    pub mv_min: u16,
    pub lambda_min_x100: u16,
    pub mv_max: u16,
    pub lambda_max_x100: u16,
}

impl LambdaAdcCalibration {
    /// Common 0-5V wideband-controller output: 0.5V = 0.68 lambda, 4.5V = 1.36 lambda.
    pub const WIDEBAND_0V5_TO_4V5: Self = Self {
        mv_min: 500,
        lambda_min_x100: 68,
        mv_max: 4500,
        lambda_max_x100: 136,
    };
}

#[derive(Copy, Clone)]
pub struct BaroAdcCalibration {
    pub mv_min: u16,
    pub kpa_min_x10: u16,
    pub mv_max: u16,
    pub kpa_max_x10: u16,
}

impl BaroAdcCalibration {
    /// Generic linear 0.5-4.5V baro input spanning 50.0-120.0 kPa.
    pub const LINEAR_0V5_TO_4V5: Self = Self {
        mv_min: 500,
        kpa_min_x10: 500,
        mv_max: 4500,
        kpa_max_x10: 1200,
    };
}

#[derive(Copy, Clone)]
pub struct AdcConfig {
    pub vref_mv: u16,
    pub adc_bits: u8,
    /// Scale for VBATT: vbatt_mv = mv_counts * num / den
    pub vbatt_scale_num: u16,
    pub vbatt_scale_den: u16,
    pub clt_bias: ThermistorBias,
    pub iat_bias: ThermistorBias,
    pub lambda_cal: LambdaAdcCalibration,
    pub baro_cal: BaroAdcCalibration,
}

#[derive(Copy, Clone, Default)]
pub struct RawCounts {
    pub map: u16,
    /// Optional HFM/MAF analog channel.
    pub maf: Option<u16>,
    pub tps: u16,
    pub clt: u16,
    pub iat: u16,
    /// If VBATT counts is not a dedicated channel, reuse one (e.g., IAT) and set scale accordingly.
    pub vbatt: u16,
    /// Optional wideband analog channel.
    pub lambda: Option<u16>,
    /// Optional dedicated barometric pressure analog channel.
    pub baro: Option<u16>,
}

#[derive(Copy, Clone, Default)]
pub struct Outputs {
    pub map_kpa_x10: u16,
    pub maf_valid: bool,
    pub maf_x100: u16,
    pub tps_percent: u8,
    pub clt_c: i16,
    pub iat_c: i16,
    pub vbatt_mv: u16,
    pub lambda_valid: bool,
    pub lambda_x100: u16,
    pub baro_valid: bool,
    pub baro_kpa_x10: u16,
}

/// Convert raw ADC counts to calibrated engineering units using SensorsCal.
pub fn convert_all(cfg: AdcConfig, cal: &SensorsCal, raw: RawCounts) -> Outputs {
    // MAP calibration (mV → kPa×10) via 2‑point linear
    let map_curve = Piecewise::new(
        [cal.map_v0_mv as u32, cal.map_v1_mv as u32],
        [cal.map_kpa0_x10 as i32, cal.map_kpa1_x10 as i32],
    );
    let map = VoltageSensor {
        vref_mv: cfg.vref_mv,
        adc_bits: cfg.adc_bits,
        last_counts: 0,
        curve: map_curve,
    };
    let map_kpa_x10 = map.value_from_counts(raw.map).clamp(0, 65535) as u16;

    let maf_curve = Piecewise::new(
        [
            cal.maf_mv[0] as u32,
            cal.maf_mv[1] as u32,
            cal.maf_mv[2] as u32,
            cal.maf_mv[3] as u32,
            cal.maf_mv[4] as u32,
            cal.maf_mv[5] as u32,
            cal.maf_mv[6] as u32,
            cal.maf_mv[7] as u32,
        ],
        [
            cal.maf_flow_x100[0] as i32,
            cal.maf_flow_x100[1] as i32,
            cal.maf_flow_x100[2] as i32,
            cal.maf_flow_x100[3] as i32,
            cal.maf_flow_x100[4] as i32,
            cal.maf_flow_x100[5] as i32,
            cal.maf_flow_x100[6] as i32,
            cal.maf_flow_x100[7] as i32,
        ],
    );
    let maf = VoltageSensor {
        vref_mv: cfg.vref_mv,
        adc_bits: cfg.adc_bits,
        last_counts: 0,
        curve: maf_curve,
    };
    let maf_x100 = raw
        .maf
        .map(|counts| maf.value_from_counts(counts).clamp(0, 65535) as u16)
        .unwrap_or(0);

    // TPS calibration (counts → %)
    let tps_min = cal.tps_min_counts;
    let tps_max = cal.tps_max_counts;
    let tps = if tps_max <= tps_min {
        0
    } else {
        let tps_clamped = raw.tps.clamp(tps_min, tps_max);
        ((tps_clamped - tps_min) as u32) * 100 / ((tps_max - tps_min) as u32)
    };
    let tps_percent = tps.min(100) as u8;

    // CLT/IAT thermistor calibration using ohms→°C piecewise
    let mut clt_oh = cal.clt_ohms;
    let mut clt_deg = cal.clt_deg_c;
    if clt_oh[0] > clt_oh[7] {
        clt_oh.reverse();
        clt_deg.reverse();
    }
    let mut iat_oh = cal.iat_ohms;
    let mut iat_deg = cal.iat_deg_c;
    if iat_oh[0] > iat_oh[7] {
        iat_oh.reverse();
        iat_deg.reverse();
    }
    let clt_curve = Piecewise::new(
        clt_oh,
        [
            clt_deg[0] as i32,
            clt_deg[1] as i32,
            clt_deg[2] as i32,
            clt_deg[3] as i32,
            clt_deg[4] as i32,
            clt_deg[5] as i32,
            clt_deg[6] as i32,
            clt_deg[7] as i32,
        ],
    );
    let iat_curve = Piecewise::new(
        iat_oh,
        [
            iat_deg[0] as i32,
            iat_deg[1] as i32,
            iat_deg[2] as i32,
            iat_deg[3] as i32,
            iat_deg[4] as i32,
            iat_deg[5] as i32,
            iat_deg[6] as i32,
            iat_deg[7] as i32,
        ],
    );
    let clt_model = ResistiveSensor {
        vref_mv: cfg.vref_mv,
        adc_bits: cfg.adc_bits,
        r_known_ohms: cfg.clt_bias.known_ohms,
        divider: cfg.clt_bias.divider,
        last_counts: 0,
        curve: clt_curve,
    };
    let iat_model = ResistiveSensor {
        vref_mv: cfg.vref_mv,
        adc_bits: cfg.adc_bits,
        r_known_ohms: cfg.iat_bias.known_ohms,
        divider: cfg.iat_bias.divider,
        last_counts: 0,
        curve: iat_curve,
    };
    let clt_c = clt_model.value_from_counts(raw.clt) as i16;
    let iat_c = iat_model.value_from_counts(raw.iat) as i16;

    // VBATT conversion using configured scale
    let mv: u32 = counts_to_mv(raw.vbatt, cfg.vref_mv, cfg.adc_bits);
    let vbatt_mv = if cfg.vbatt_scale_den == 0 {
        0
    } else {
        (mv * (cfg.vbatt_scale_num as u32) / (cfg.vbatt_scale_den as u32)) as u16
    };

    // Wideband lambda (simple linear volts -> lambda) if provided
    let lambda_x100 = if let Some(counts) = raw.lambda {
        let mv = counts_to_mv(counts, cfg.vref_mv, cfg.adc_bits);
        if cfg.lambda_cal.mv_max <= cfg.lambda_cal.mv_min {
            100
        } else {
            let mv_min = u32::from(cfg.lambda_cal.mv_min);
            let mv_max = u32::from(cfg.lambda_cal.mv_max);
            let lam_min = i32::from(cfg.lambda_cal.lambda_min_x100);
            let lam_max = i32::from(cfg.lambda_cal.lambda_max_x100);
            let mv_clamped = mv.clamp(mv_min, mv_max);
            let lam = lam_min
                + (lam_max - lam_min) * (mv_clamped as i32 - mv_min as i32)
                    / (mv_max - mv_min) as i32;
            lam.clamp(50, 200) as u16
        }
    } else {
        100
    };

    // Dedicated barometric pressure ADC path if the board policy uses one.
    let baro_kpa_x10 = if let Some(counts) = raw.baro {
        let mv = counts_to_mv(counts, cfg.vref_mv, cfg.adc_bits);
        if cfg.baro_cal.mv_max <= cfg.baro_cal.mv_min {
            0
        } else {
            let mv_min = u32::from(cfg.baro_cal.mv_min);
            let mv_max = u32::from(cfg.baro_cal.mv_max);
            let kpa_min = i32::from(cfg.baro_cal.kpa_min_x10);
            let kpa_max = i32::from(cfg.baro_cal.kpa_max_x10);
            let mv_clamped = mv.clamp(mv_min, mv_max);
            let kpa = kpa_min
                + (kpa_max - kpa_min) * (mv_clamped as i32 - mv_min as i32)
                    / (mv_max - mv_min) as i32;
            kpa.clamp(0, 65535) as u16
        }
    } else {
        0
    };

    Outputs {
        map_kpa_x10,
        maf_valid: raw.maf.is_some(),
        maf_x100,
        tps_percent,
        clt_c,
        iat_c,
        vbatt_mv,
        lambda_valid: raw.lambda.is_some(),
        lambda_x100,
        baro_valid: raw.baro.is_some(),
        baro_kpa_x10,
    }
}

/// Clamp slew rate for u16 signals (e.g., MAP in kPa×10) based on elapsed time.
/// max_rate_per_s uses the same unit per second (e.g., kPa×10 per second).
pub fn clamp_slew_u16(prev: u16, new: u16, max_rate_per_s: u16, dt_us: u32) -> u16 {
    if dt_us == 0 || prev == new {
        return new;
    }
    let max_delta = ((max_rate_per_s as u32).saturating_mul(dt_us) / 1_000_000) as i32;
    let delta = (new as i32) - (prev as i32);
    if delta > max_delta {
        (prev as i32 + max_delta) as u16
    } else if delta < -max_delta {
        (prev as i32 - max_delta) as u16
    } else {
        new
    }
}

/// Clamp slew rate for u8 signals (e.g., TPS percent) based on elapsed time.
/// max_rate_per_s in %/s.
pub fn clamp_slew_u8(prev: u8, new: u8, max_rate_per_s: u16, dt_us: u32) -> u8 {
    if dt_us == 0 || prev == new {
        return new;
    }
    let max_delta = ((max_rate_per_s as u32).saturating_mul(dt_us) / 1_000_000) as i32;
    let delta = (new as i32) - (prev as i32);
    if delta > max_delta {
        (prev as i32 + max_delta).clamp(0, 255) as u8
    } else if delta < -max_delta {
        (prev as i32 - max_delta).clamp(0, 255) as u8
    } else {
        new
    }
}

#[cfg(test)]
mod tests;
