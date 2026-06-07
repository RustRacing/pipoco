//! Generic sensor calibration and conversion math (no_std).
//!
//! Platform-agnostic building blocks: ADC/divider conversion, piecewise curves,
//! calibrated sensor models, and the TS-exposed sensor calibration values.

pub mod convert;
pub mod curve;
pub mod model;

pub use model::{ResistiveSensor as ThermistorSensor, VoltageSensor as CurveSensor};

/// Quality of a sensor reading.
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum Quality {
    Good,
    Degraded,
    Fault,
}

/// Sensor read failures.
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum SensorError {
    OutOfRange,
    Stale,
    NotReady,
}

/// Generic sensor contract.
pub trait Sensor {
    type Reading;

    fn read(&mut self) -> Result<Self::Reading, SensorError>;

    fn quality(&self) -> Quality;
}

/// Sensor calibration values exposed to TS (page 3)
#[derive(Copy, Clone)]
pub struct SensorsCal {
    pub tps_min_counts: u16,
    pub tps_max_counts: u16,
    pub map_v0_mv: u16,
    pub map_kpa0_x10: u16,
    pub map_v1_mv: u16,
    pub map_kpa1_x10: u16,
    /// Optional voltage-to-airflow curve for HFM/MAF sensors, in millivolts.
    pub maf_mv: [u16; 8],
    /// Optional HFM/MAF airflow curve values, in source-native units x100.
    pub maf_flow_x100: [u16; 8],
    pub clt_deg_c: [i16; 8],
    pub iat_deg_c: [i16; 8],
    pub clt_ohms: [u32; 8],
    pub iat_ohms: [u32; 8],
}

impl SensorsCal {
    pub const fn default() -> Self {
        Self {
            tps_min_counts: 200,
            tps_max_counts: 3800,
            map_v0_mv: 500, // 0.5V → 10kPa
            map_kpa0_x10: 100,
            map_v1_mv: 4500, // 4.5V → 250kPa
            map_kpa1_x10: 2500,
            // Generic monotonic HFM/MAF placeholder curve. Use board/profile data for exact sensors.
            maf_mv: [0, 730, 1250, 1750, 2250, 2750, 3250, 4500],
            maf_flow_x100: [0, 280, 780, 1520, 2520, 3880, 5680, 9300],
            // Simple 8-point tables from -40..+100C
            clt_deg_c: [-40, -20, 0, 20, 40, 60, 80, 100],
            iat_deg_c: [-20, 0, 10, 20, 30, 40, 50, 60],
            // Example ohms (approx NTC 2.49k bias); these are placeholders
            clt_ohms: [100_000, 60_000, 35_000, 20_000, 12_000, 7_000, 4_500, 3_000],
            iat_ohms: [40_000, 25_000, 18_000, 12_000, 8_000, 6_000, 4_500, 3_500],
        }
    }

    pub const fn with_map_calibration(
        mut self,
        map_v0_mv: u16,
        map_kpa0_x10: u16,
        map_v1_mv: u16,
        map_kpa1_x10: u16,
    ) -> Self {
        self.map_v0_mv = map_v0_mv;
        self.map_kpa0_x10 = map_kpa0_x10;
        self.map_v1_mv = map_v1_mv;
        self.map_kpa1_x10 = map_kpa1_x10;
        self
    }

    pub const fn with_maf_calibration(mut self, maf_mv: [u16; 8], maf_flow_x100: [u16; 8]) -> Self {
        self.maf_mv = maf_mv;
        self.maf_flow_x100 = maf_flow_x100;
        self
    }

    pub const fn with_clt_calibration(mut self, deg_c: [i16; 8], ohms: [u32; 8]) -> Self {
        self.clt_deg_c = deg_c;
        self.clt_ohms = ohms;
        self
    }

    pub const fn with_iat_calibration(mut self, deg_c: [i16; 8], ohms: [u32; 8]) -> Self {
        self.iat_deg_c = deg_c;
        self.iat_ohms = ohms;
        self
    }

    pub const fn mpxh6400a_5v_scaled(output_scale_num: u16, output_scale_den: u16) -> Self {
        // Datasheet nominal endpoints: 20 kPa ~= 0.2 V, 400 kPa ~= 4.8 V.
        Self::default().with_map_calibration(
            scale_mv(200, output_scale_num, output_scale_den),
            200,
            scale_mv(4800, output_scale_num, output_scale_den),
            4000,
        )
    }

    pub const fn mpx5700ap_5v_scaled(output_scale_num: u16, output_scale_den: u16) -> Self {
        // Datasheet transfer function gives ~0.296 V at 15 kPa and ~4.7 V at 700 kPa.
        Self::default().with_map_calibration(
            scale_mv(296, output_scale_num, output_scale_den),
            150,
            scale_mv(4700, output_scale_num, output_scale_den),
            7000,
        )
    }
}

const fn scale_mv(mv: u16, numerator: u16, denominator: u16) -> u16 {
    if denominator == 0 {
        0
    } else {
        ((mv as u32 * numerator as u32) / denominator as u32) as u16
    }
}

#[cfg(test)]
mod tests {
    use super::SensorsCal;

    #[test]
    fn mpxh6400a_preset_sets_400_kpa_range() {
        let cal = SensorsCal::mpxh6400a_5v_scaled(1, 1);

        assert_eq!(cal.map_v0_mv, 200);
        assert_eq!(cal.map_kpa0_x10, 200);
        assert_eq!(cal.map_v1_mv, 4800);
        assert_eq!(cal.map_kpa1_x10, 4000);
    }

    #[test]
    fn mpx5700ap_preset_sets_700_kpa_range_and_scales_voltage() {
        let cal = SensorsCal::mpx5700ap_5v_scaled(33, 50);

        assert_eq!(cal.map_v0_mv, 195);
        assert_eq!(cal.map_kpa0_x10, 150);
        assert_eq!(cal.map_v1_mv, 3102);
        assert_eq!(cal.map_kpa1_x10, 7000);
    }

    #[test]
    fn maf_calibration_can_be_overridden_without_changing_map() {
        let cal = SensorsCal::mpxh6400a_5v_scaled(1, 1).with_maf_calibration(
            [0, 500, 1000, 1500, 2000, 2500, 3000, 3500],
            [0, 100, 300, 700, 1200, 2000, 3200, 5000],
        );

        assert_eq!(cal.map_kpa1_x10, 4000);
        assert_eq!(cal.maf_mv[3], 1500);
        assert_eq!(cal.maf_flow_x100[7], 5000);
    }

    #[test]
    fn default_maf_curve_is_monotonic() {
        let cal = SensorsCal::default();

        for idx in 1..cal.maf_mv.len() {
            assert!(cal.maf_mv[idx] > cal.maf_mv[idx - 1]);
            assert!(cal.maf_flow_x100[idx] >= cal.maf_flow_x100[idx - 1]);
        }
    }

    #[test]
    fn thermistor_calibrations_can_be_overridden_independently() {
        let cal = SensorsCal::default()
            .with_clt_calibration(
                [-40, -20, 0, 20, 40, 60, 80, 100],
                [90_000, 50_000, 27_000, 12_000, 5_500, 2_800, 1_500, 900],
            )
            .with_iat_calibration(
                [-20, 0, 10, 20, 30, 40, 50, 60],
                [38_000, 24_000, 17_000, 11_500, 7_800, 5_600, 4_100, 3_100],
            );

        assert_eq!(cal.clt_ohms[0], 90_000);
        assert_eq!(cal.clt_deg_c[7], 100);
        assert_eq!(cal.iat_ohms[0], 38_000);
        assert_eq!(cal.iat_deg_c[7], 60);
        assert_eq!(cal.map_kpa1_x10, SensorsCal::default().map_kpa1_x10);
    }
}
