use ecu_calibration::sensors::SensorsCal;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MapSensorKind {
    Mpxh6400a,
    Mpxh6400ac6u,
    Mpx5700ap,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct VoltageScale {
    pub numerator: u16,
    pub denominator: u16,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MapCalibrationError {
    ZeroDenominator,
    VoltageAmplification,
}

impl VoltageScale {
    pub const DIRECT_5V: Self = Self {
        numerator: 1,
        denominator: 1,
    };

    pub const fn new(numerator: u16, denominator: u16) -> Self {
        Self {
            numerator,
            denominator,
        }
    }

    pub const fn validate(self) -> Result<Self, MapCalibrationError> {
        if self.denominator == 0 {
            Err(MapCalibrationError::ZeroDenominator)
        } else if self.numerator > self.denominator {
            Err(MapCalibrationError::VoltageAmplification)
        } else {
            Ok(self)
        }
    }
}

pub fn map_sensor_calibration(kind: MapSensorKind, scale: VoltageScale) -> SensorsCal {
    let scale = match scale.validate() {
        Ok(scale) => scale,
        Err(_) => VoltageScale::DIRECT_5V,
    };
    match kind {
        MapSensorKind::Mpxh6400a | MapSensorKind::Mpxh6400ac6u => {
            SensorsCal::mpxh6400a_5v_scaled(scale.numerator, scale.denominator)
        }
        MapSensorKind::Mpx5700ap => {
            SensorsCal::mpx5700ap_5v_scaled(scale.numerator, scale.denominator)
        }
    }
}

pub fn try_map_sensor_calibration(
    kind: MapSensorKind,
    scale: VoltageScale,
) -> Result<SensorsCal, MapCalibrationError> {
    let scale = scale.validate()?;
    Ok(map_sensor_calibration(kind, scale))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn selects_mpxh6400a_calibration_without_board_pin_assumptions() {
        let cal = map_sensor_calibration(MapSensorKind::Mpxh6400a, VoltageScale::DIRECT_5V);

        assert_eq!(cal.map_v0_mv, 200);
        assert_eq!(cal.map_kpa0_x10, 200);
        assert_eq!(cal.map_v1_mv, 4800);
        assert_eq!(cal.map_kpa1_x10, 4000);
    }

    #[test]
    fn mpxh6400ac6u_uses_mpxh6400a_transfer_function() {
        let family = map_sensor_calibration(MapSensorKind::Mpxh6400a, VoltageScale::DIRECT_5V);
        let order_code =
            map_sensor_calibration(MapSensorKind::Mpxh6400ac6u, VoltageScale::DIRECT_5V);

        assert_eq!(order_code.map_v0_mv, family.map_v0_mv);
        assert_eq!(order_code.map_kpa0_x10, family.map_kpa0_x10);
        assert_eq!(order_code.map_v1_mv, family.map_v1_mv);
        assert_eq!(order_code.map_kpa1_x10, family.map_kpa1_x10);
    }

    #[test]
    fn selects_mpx5700ap_calibration_without_board_pin_assumptions() {
        let cal = map_sensor_calibration(MapSensorKind::Mpx5700ap, VoltageScale::DIRECT_5V);

        assert_eq!(cal.map_v0_mv, 296);
        assert_eq!(cal.map_kpa0_x10, 150);
        assert_eq!(cal.map_v1_mv, 4700);
        assert_eq!(cal.map_kpa1_x10, 7000);
    }

    #[test]
    fn voltage_scale_applies_to_sensor_voltage_endpoints_only() {
        let cal = map_sensor_calibration(MapSensorKind::Mpx5700ap, VoltageScale::new(33, 50));

        assert_eq!(cal.map_v0_mv, 195);
        assert_eq!(cal.map_kpa0_x10, 150);
        assert_eq!(cal.map_v1_mv, 3102);
        assert_eq!(cal.map_kpa1_x10, 7000);
    }

    #[test]
    fn checked_map_calibration_rejects_invalid_voltage_scale() {
        assert!(matches!(
            try_map_sensor_calibration(MapSensorKind::Mpx5700ap, VoltageScale::new(1, 0)),
            Err(MapCalibrationError::ZeroDenominator)
        ));
        assert!(matches!(
            try_map_sensor_calibration(MapSensorKind::Mpx5700ap, VoltageScale::new(5, 3)),
            Err(MapCalibrationError::VoltageAmplification)
        ));
    }

    #[test]
    fn infallible_map_calibration_falls_back_to_direct_scale_for_invalid_scale() {
        let direct = map_sensor_calibration(MapSensorKind::Mpx5700ap, VoltageScale::DIRECT_5V);
        let invalid = map_sensor_calibration(MapSensorKind::Mpx5700ap, VoltageScale::new(1, 0));

        assert_eq!(invalid.map_v0_mv, direct.map_v0_mv);
        assert_eq!(invalid.map_kpa0_x10, direct.map_kpa0_x10);
        assert_eq!(invalid.map_v1_mv, direct.map_v1_mv);
        assert_eq!(invalid.map_kpa1_x10, direct.map_kpa1_x10);
    }
}
