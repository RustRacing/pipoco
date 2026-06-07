use crate::Kpa10;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum AirLoadSource {
    MapSpeedDensity,
    MafFlow,
    TpsAlphaN,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct AirLoadConfig {
    pub source: AirLoadSource,
    pub maf_min_x100: u16,
    pub tps_min_x100: u16,
}

impl Default for AirLoadConfig {
    fn default() -> Self {
        Self {
            source: AirLoadSource::MapSpeedDensity,
            maf_min_x100: 1,
            tps_min_x100: 1,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct AirLoadInput {
    pub map_kpa10: Kpa10,
    pub maf_x100: u16,
    pub tps_x100: u16,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct AirLoad {
    pub source: AirLoadSource,
    /// Source-native scaled value. MAP is kPa x10, MAF is flow x100, TPS is percent x100.
    pub value_x100: u16,
    pub valid: bool,
}

pub fn air_load_select(config: &AirLoadConfig, input: AirLoadInput) -> AirLoad {
    match config.source {
        AirLoadSource::MapSpeedDensity => AirLoad {
            source: AirLoadSource::MapSpeedDensity,
            value_x100: input.map_kpa10.0,
            valid: input.map_kpa10.0 > 0,
        },
        AirLoadSource::MafFlow => AirLoad {
            source: AirLoadSource::MafFlow,
            value_x100: input.maf_x100,
            valid: input.maf_x100 >= config.maf_min_x100,
        },
        AirLoadSource::TpsAlphaN => AirLoad {
            source: AirLoadSource::TpsAlphaN,
            value_x100: input.tps_x100,
            valid: input.tps_x100 >= config.tps_min_x100,
        },
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn input() -> AirLoadInput {
        AirLoadInput {
            map_kpa10: Kpa10(950),
            maf_x100: 3200,
            tps_x100: 2500,
        }
    }

    #[test]
    fn map_source_preserves_map_units() {
        let load = air_load_select(&AirLoadConfig::default(), input());

        assert_eq!(load.source, AirLoadSource::MapSpeedDensity);
        assert_eq!(load.value_x100, 950);
        assert!(load.valid);
    }

    #[test]
    fn maf_source_preserves_flow_units_without_pretending_kpa() {
        let config = AirLoadConfig {
            source: AirLoadSource::MafFlow,
            maf_min_x100: 10,
            ..AirLoadConfig::default()
        };

        let load = air_load_select(&config, input());

        assert_eq!(load.source, AirLoadSource::MafFlow);
        assert_eq!(load.value_x100, 3200);
        assert!(load.valid);
    }

    #[test]
    fn alpha_n_source_preserves_tps_units() {
        let config = AirLoadConfig {
            source: AirLoadSource::TpsAlphaN,
            tps_min_x100: 100,
            ..AirLoadConfig::default()
        };

        let load = air_load_select(&config, input());

        assert_eq!(load.source, AirLoadSource::TpsAlphaN);
        assert_eq!(load.value_x100, 2500);
        assert!(load.valid);
    }

    #[test]
    fn maf_source_can_report_invalid_low_flow() {
        let config = AirLoadConfig {
            source: AirLoadSource::MafFlow,
            maf_min_x100: 10,
            ..AirLoadConfig::default()
        };
        let load = air_load_select(
            &config,
            AirLoadInput {
                maf_x100: 0,
                ..input()
            },
        );

        assert!(!load.valid);
    }
}
