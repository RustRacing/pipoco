use crate::types::*;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct SensorSnapshot {
    pub timestamp_us: Micros,
    pub rpm: Rpm,
    pub crank_angle_deg10: CrankDeg10,
    pub map_kpa10: Kpa10,
    pub tps_x1000: u16,
    pub clt_c10: Celsius10,
    pub iat_c10: Celsius10,
    pub lambda_x1000: u16,
    pub battery_mv: Millivolts,
    pub knock_intensity_x100: u16,
}

impl SensorSnapshot {
    pub const fn empty() -> Self {
        Self {
            timestamp_us: Micros(0),
            rpm: Rpm(0),
            crank_angle_deg10: CrankDeg10(0),
            map_kpa10: Kpa10(0),
            tps_x1000: 0,
            clt_c10: Celsius10(0),
            iat_c10: Celsius10(0),
            lambda_x1000: 1000,
            battery_mv: Millivolts(0),
            knock_intensity_x100: 0,
        }
    }
}
