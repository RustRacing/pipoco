use ecu_core::sensors::convert::{counts_to_mv, DividerConfig};
use ecu_core::sensors::curve::Piecewise;
use ecu_core::sensors::model::{CalibratedSensor, ResistiveSensor, VoltageSensor};
use ecu_core::sensors::SensorsCal;

#[derive(Copy, Clone)]
pub struct AdcConfig {
    pub vref_mv: u16,
    pub adc_bits: u8,
    /// Scale for VBATT: vbatt_mv = mv_counts * num / den
    pub vbatt_scale_num: u16,
    pub vbatt_scale_den: u16,
}

#[derive(Copy, Clone, Default)]
pub struct RawCounts {
    pub map: u16,
    pub tps: u16,
    pub clt: u16,
    pub iat: u16,
    /// If VBATT counts is not a dedicated channel, reuse one (e.g., IAT) and set scale accordingly.
    pub vbatt: u16,
    /// Optional wideband analog channel (counts). If not used, set to 0.
    pub lambda: u16,
}

#[derive(Copy, Clone, Default)]
pub struct Outputs {
    pub map_kpa_x10: u16,
    pub tps_percent: u8,
    pub clt_c: i16,
    pub iat_c: i16,
    pub vbatt_mv: u16,
    pub lambda_x100: u16,
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
        curve: map_curve,
    };
    let map_kpa_x10 = map.value_from_counts(raw.map).clamp(0, 65535) as u16;

    // TPS calibration (counts → %)
    let tps_min = cal.tps_min_counts;
    let tps_max = cal.tps_max_counts;
    let tps_clamped = raw.tps.clamp(tps_min, tps_max);
    let tps = ((tps_clamped - tps_min) as u32) * 100 / ((tps_max - tps_min) as u32);
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
    const R_KNOWN: u32 = 2490; // 2.49k bias typical
    let clt_model = ResistiveSensor {
        vref_mv: cfg.vref_mv,
        adc_bits: cfg.adc_bits,
        r_known_ohms: R_KNOWN,
        divider: DividerConfig::PullupTop,
        curve: clt_curve,
    };
    let iat_model = ResistiveSensor {
        vref_mv: cfg.vref_mv,
        adc_bits: cfg.adc_bits,
        r_known_ohms: R_KNOWN,
        divider: DividerConfig::PullupTop,
        curve: iat_curve,
    };
    let clt_c = clt_model.value_from_counts(raw.clt) as i16;
    let iat_c = iat_model.value_from_counts(raw.iat) as i16;

    // VBATT conversion using configured scale
    let mv: u32 = counts_to_mv(raw.vbatt, cfg.vref_mv, cfg.adc_bits);
    let vbatt_mv = (mv * (cfg.vbatt_scale_num as u32) / (cfg.vbatt_scale_den as u32)) as u16;

    // Wideband lambda (simple linear volts -> lambda) if provided
    let lambda_x100 = if raw.lambda == 0 {
        100
    } else {
        let mv = counts_to_mv(raw.lambda, cfg.vref_mv, cfg.adc_bits);
        // Assume 0.5V -> 0.68 lambda, 4.5V -> 1.36 lambda (typical 0-5V wideband slope)
        let mv_min = 500u32;
        let mv_max = 4500u32;
        let lam_min = 68i32;
        let lam_max = 136i32;
        let mv_clamped = mv.clamp(mv_min, mv_max);
        let lam = lam_min
            + ((lam_max - lam_min) as i32)
                * ((mv_clamped as i32 - mv_min as i32))
                / ((mv_max - mv_min) as i32);
        lam.clamp(50, 200) as u16
    };

    Outputs {
        map_kpa_x10,
        tps_percent,
        clt_c,
        iat_c,
        vbatt_mv,
        lambda_x100,
    }
}

/// Clamp slew rate for u16 signals (e.g., MAP in kPa×10) based on elapsed time.
/// max_rate_per_s uses the same unit per second (e.g., kPa×10 per second).
pub fn clamp_slew_u16(prev: u16, new: u16, max_rate_per_s: u16, dt_us: u32) -> u16 {
    if dt_us == 0 || prev == new { return new; }
    let max_delta = ((max_rate_per_s as u32).saturating_mul(dt_us) / 1_000_000) as i32;
    let delta = (new as i32) - (prev as i32);
    if delta > max_delta { (prev as i32 + max_delta) as u16 }
    else if delta < -max_delta { (prev as i32 - max_delta) as u16 }
    else { new }
}

/// Clamp slew rate for u8 signals (e.g., TPS percent) based on elapsed time.
/// max_rate_per_s in %/s.
pub fn clamp_slew_u8(prev: u8, new: u8, max_rate_per_s: u16, dt_us: u32) -> u8 {
    if dt_us == 0 || prev == new { return new; }
    let max_delta = ((max_rate_per_s as u32).saturating_mul(dt_us) / 1_000_000) as i32;
    let delta = (new as i32) - (prev as i32);
    if delta > max_delta { (prev as i32 + max_delta).clamp(0, 255) as u8 }
    else if delta < -max_delta { (prev as i32 - max_delta).clamp(0, 255) as u8 }
    else { new }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_clamp_slew_u16_limits_delta() {
        // prev=1000, new=2000, dt=100ms, max_rate=1000 units/s => max_delta=100
        let out = clamp_slew_u16(1000, 2000, 1000, 100_000);
        assert_eq!(out, 1100);
        // Negative direction
        let out2 = clamp_slew_u16(1000, 0, 1000, 100_000);
        assert_eq!(out2, 900);
        // Within limits
        let out3 = clamp_slew_u16(1000, 1050, 1000, 100_000);
        assert_eq!(out3, 1050);
    }

    #[test]
    fn test_clamp_slew_u8_limits_delta() {
        // prev=10%, new=90%, dt=50ms, max_rate=200%/s => max_delta=10%
        let out = clamp_slew_u8(10, 90, 200, 50_000);
        assert_eq!(out, 20);
        // Negative direction
        let out2 = clamp_slew_u8(50, 0, 200, 50_000);
        assert_eq!(out2, 40);
        // Within limits
        let out3 = clamp_slew_u8(10, 15, 200, 50_000);
        assert_eq!(out3, 15);
    }
}
