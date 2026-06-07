use crate::interp::{find_segment, lerp_u16};
use crate::{Curve16, Kpa10, RatioX1000};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum BaroSource {
    FixedKpa,
    StartupMapSample,
    DedicatedSensor,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct BaroSourceConfig {
    pub source: BaroSource,
    pub fixed_kpa10: Kpa10,
}

impl Default for BaroSourceConfig {
    fn default() -> Self {
        Self {
            source: BaroSource::StartupMapSample,
            fixed_kpa10: Kpa10(1013),
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct BaroSourceInput {
    pub startup_map_kpa10: Kpa10,
    pub dedicated_baro_kpa10: Option<Kpa10>,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct BaroSourceReading {
    pub source: BaroSource,
    pub baro_kpa10: Kpa10,
    pub valid: bool,
}

pub fn select_baro_source(config: &BaroSourceConfig, input: BaroSourceInput) -> BaroSourceReading {
    match config.source {
        BaroSource::FixedKpa => BaroSourceReading {
            source: BaroSource::FixedKpa,
            baro_kpa10: config.fixed_kpa10,
            valid: valid_baro(config.fixed_kpa10),
        },
        BaroSource::StartupMapSample => BaroSourceReading {
            source: BaroSource::StartupMapSample,
            baro_kpa10: input.startup_map_kpa10,
            valid: valid_baro(input.startup_map_kpa10),
        },
        BaroSource::DedicatedSensor => {
            if let Some(baro_kpa10) = input.dedicated_baro_kpa10 {
                BaroSourceReading {
                    source: BaroSource::DedicatedSensor,
                    baro_kpa10,
                    valid: valid_baro(baro_kpa10),
                }
            } else {
                BaroSourceReading {
                    source: BaroSource::DedicatedSensor,
                    baro_kpa10: Kpa10(0),
                    valid: false,
                }
            }
        }
    }
}

const fn valid_baro(baro_kpa10: Kpa10) -> bool {
    baro_kpa10.0 >= 500 && baro_kpa10.0 <= 1200
}

pub fn baro_correction(baro_corr_curve: &Curve16, baro_kpa10: Kpa10) -> RatioX1000 {
    RatioX1000(lookup_curve_u16(baro_corr_curve, baro_kpa10.0))
}

fn lookup_curve_u16(curve: &Curve16, x: u16) -> u16 {
    let len = curve.axis.len as usize;
    let clipped = x.clamp(curve.axis.values[0], curve.axis.values[len - 1]);
    let seg = find_segment(&curve.axis, clipped);
    lerp_u16(
        curve.axis.values[seg],
        curve.axis.values[seg + 1],
        curve.values[seg],
        curve.values[seg + 1],
        clipped,
    )
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::Axis16;

    fn axis(values: &[u16]) -> Axis16 {
        let mut axis = Axis16 {
            len: values.len() as u8,
            ..Axis16::default()
        };
        let mut idx = 0usize;
        while idx < values.len() {
            axis.values[idx] = values[idx];
            idx += 1;
        }
        axis
    }

    fn baro_curve() -> Curve16 {
        let mut values = [0u16; 16];
        values[0] = 700;
        values[1] = 850;
        values[2] = 1000;
        Curve16 {
            axis: axis(&[700, 850, 1000]),
            values,
        }
    }

    #[test]
    fn baro_high_altitude_matches_curve_point() {
        let corr = baro_correction(&baro_curve(), Kpa10(700));
        assert_eq!(corr, RatioX1000(700));
    }

    #[test]
    fn baro_sea_level_matches_curve_point() {
        let corr = baro_correction(&baro_curve(), Kpa10(1000));
        assert_eq!(corr, RatioX1000(1000));
    }

    #[test]
    fn fixed_baro_source_uses_configured_value() {
        let config = BaroSourceConfig {
            source: BaroSource::FixedKpa,
            fixed_kpa10: Kpa10(990),
        };

        let reading = select_baro_source(
            &config,
            BaroSourceInput {
                startup_map_kpa10: Kpa10(940),
                dedicated_baro_kpa10: Some(Kpa10(980)),
            },
        );

        assert_eq!(reading.source, BaroSource::FixedKpa);
        assert_eq!(reading.baro_kpa10, Kpa10(990));
        assert!(reading.valid);
    }

    #[test]
    fn startup_map_baro_source_uses_startup_map_sample() {
        let reading = select_baro_source(
            &BaroSourceConfig::default(),
            BaroSourceInput {
                startup_map_kpa10: Kpa10(950),
                dedicated_baro_kpa10: None,
            },
        );

        assert_eq!(reading.source, BaroSource::StartupMapSample);
        assert_eq!(reading.baro_kpa10, Kpa10(950));
        assert!(reading.valid);
    }

    #[test]
    fn dedicated_baro_source_requires_sensor_value() {
        let config = BaroSourceConfig {
            source: BaroSource::DedicatedSensor,
            ..BaroSourceConfig::default()
        };

        let missing = select_baro_source(
            &config,
            BaroSourceInput {
                startup_map_kpa10: Kpa10(950),
                dedicated_baro_kpa10: None,
            },
        );
        let present = select_baro_source(
            &config,
            BaroSourceInput {
                startup_map_kpa10: Kpa10(950),
                dedicated_baro_kpa10: Some(Kpa10(1000)),
            },
        );

        assert!(!missing.valid);
        assert_eq!(missing.baro_kpa10, Kpa10(0));
        assert!(present.valid);
        assert_eq!(present.baro_kpa10, Kpa10(1000));
    }

    #[test]
    fn baro_source_rejects_out_of_plausible_range_pressure() {
        let config = BaroSourceConfig {
            source: BaroSource::FixedKpa,
            fixed_kpa10: Kpa10(1300),
        };

        let reading = select_baro_source(
            &config,
            BaroSourceInput {
                startup_map_kpa10: Kpa10(950),
                dedicated_baro_kpa10: None,
            },
        );

        assert_eq!(reading.baro_kpa10, Kpa10(1300));
        assert!(!reading.valid);
    }
}
