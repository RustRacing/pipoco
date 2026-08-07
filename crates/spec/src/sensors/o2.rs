use crate::{numeric::clamp_u16, AfrX100, Calibration, O2SensorMode};

const ADC_MIN: u16 = 0;
const ADC_MAX: u16 = 4095;
const AFR_MIN_X100: u16 = 500;
const AFR_MAX_X100: u16 = 3000;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct O2SensorReading {
    pub afr_x100: AfrX100,
    pub rich: bool,
}

pub fn o2_from_counts(
    calibration: &Calibration,
    adc_counts: u16,
    was_rich: bool,
) -> O2SensorReading {
    let counts = clamp_u16(adc_counts, ADC_MIN, ADC_MAX);
    match calibration.o2_sensor_mode {
        O2SensorMode::WidebandLinear => wideband_reading(calibration, counts),
        O2SensorMode::NarrowbandSwitch => narrowband_reading(calibration, counts, was_rich),
    }
}

fn wideband_reading(calibration: &Calibration, counts: u16) -> O2SensorReading {
    let min_afr = calibration.o2_wideband_afr_min_x100 as u32;
    let max_afr = calibration.o2_wideband_afr_max_x100 as u32;
    let span = max_afr - min_afr;
    let scaled = (counts as u32 * span) / ADC_MAX as u32;
    let afr = clamp_u16((min_afr + scaled) as u16, AFR_MIN_X100, AFR_MAX_X100);
    O2SensorReading {
        afr_x100: AfrX100::new(afr),
        rich: afr <= calibration.stoich_afr_x100,
    }
}

fn narrowband_reading(calibration: &Calibration, counts: u16, was_rich: bool) -> O2SensorReading {
    let threshold = calibration.o2_narrowband_threshold_counts;
    let hysteresis = calibration.o2_narrowband_hysteresis_counts;
    let lower = threshold.saturating_sub(hysteresis);
    let upper = threshold.saturating_add(hysteresis);
    let rich = if was_rich {
        counts <= upper
    } else {
        counts < lower
    };
    let afr = if rich {
        calibration.o2_narrowband_rich_afr_x100
    } else {
        calibration.o2_narrowband_lean_afr_x100
    };
    O2SensorReading {
        afr_x100: AfrX100::new(clamp_u16(afr, AFR_MIN_X100, AFR_MAX_X100)),
        rich,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn calibration_wideband() -> Calibration {
        Calibration {
            o2_sensor_mode: O2SensorMode::WidebandLinear,
            o2_wideband_afr_min_x100: 1000,
            o2_wideband_afr_max_x100: 2000,
            stoich_afr_x100: 1470,
            ..Calibration::default()
        }
    }

    fn calibration_narrowband() -> Calibration {
        Calibration {
            o2_sensor_mode: O2SensorMode::NarrowbandSwitch,
            o2_narrowband_threshold_counts: 2000,
            o2_narrowband_hysteresis_counts: 100,
            o2_narrowband_rich_afr_x100: 1350,
            o2_narrowband_lean_afr_x100: 1550,
            ..Calibration::default()
        }
    }

    #[test]
    fn wideband_clamps_at_adc_endpoints() {
        let cal = calibration_wideband();
        assert_eq!(o2_from_counts(&cal, 0, false).afr_x100, AfrX100::new(1000));
        assert_eq!(
            o2_from_counts(&cal, 4095, false).afr_x100,
            AfrX100::new(2000)
        );
    }

    #[test]
    fn wideband_midpoint_uses_floor_linear_interpolation() {
        let cal = calibration_wideband();
        assert_eq!(
            o2_from_counts(&cal, 2048, false).afr_x100,
            AfrX100::new(1500)
        );
    }

    #[test]
    fn narrowband_applies_hysteresis_window() {
        let cal = calibration_narrowband();
        let toggled_lean = o2_from_counts(&cal, 2200, true);
        assert!(!toggled_lean.rich);
        assert_eq!(toggled_lean.afr_x100, AfrX100::new(1550));

        let held_lean = o2_from_counts(&cal, 1950, false);
        assert!(!held_lean.rich);
        assert_eq!(held_lean.afr_x100, AfrX100::new(1550));

        let toggled_rich = o2_from_counts(&cal, 1899, false);
        assert!(toggled_rich.rich);
        assert_eq!(toggled_rich.afr_x100, AfrX100::new(1350));
    }
}
