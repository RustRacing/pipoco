pub mod convert;
pub mod curve;
pub mod model;
pub mod plausibility;
pub mod slew;
pub mod thermistor;

/// Runtime sensor limits and clear timing
#[derive(Copy, Clone)]
pub struct SensorsLimits {
    pub map_min_kpa_x10: u16,
    pub map_max_kpa_x10: u16,
    pub tps_min_percent: u8,
    pub tps_max_percent: u8,
    /// Seconds in-range before clearing an active diag
    pub clear_time_s: u16,
}

impl SensorsLimits {
    pub const fn default() -> Self {
        Self {
            map_min_kpa_x10: 100,  // 10.0 kPa
            map_max_kpa_x10: 3000, // 300.0 kPa
            tps_min_percent: 0,
            tps_max_percent: 100,
            clear_time_s: 3,
        }
    }
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
            // Simple 8-point tables from -40..+100C
            clt_deg_c: [-40, -20, 0, 20, 40, 60, 80, 100],
            iat_deg_c: [-20, 0, 10, 20, 30, 40, 50, 60],
            // Example ohms (approx NTC 2.49k bias); these are placeholders
            clt_ohms: [100_000, 60_000, 35_000, 20_000, 12_000, 7_000, 4_500, 3_000],
            iat_ohms: [40_000, 25_000, 18_000, 12_000, 8_000, 6_000, 4_500, 3_500],
        }
    }
}
