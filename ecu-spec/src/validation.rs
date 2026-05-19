use crate::{
    Axis16, Calibration, Curve16, SignedCurve16, Table2D16, ValidatedCalibration, ValidationError,
};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ValueRange {
    pub min: u32,
    pub max: u32,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct SignedValueRange {
    pub min: i32,
    pub max: i32,
}

pub fn validate_axis(axis: &Axis16) -> Result<(), ValidationError> {
    if axis.len < 2 {
        return Err(ValidationError::AxisTooShort);
    }
    if axis.len > 16 {
        return Err(ValidationError::TableDimensionMismatch);
    }

    let len = axis.len as usize;
    let mut idx = 0usize;
    while idx + 1 < len {
        if axis.values[idx] >= axis.values[idx + 1] {
            return Err(ValidationError::AxisNotStrictlyIncreasing);
        }
        idx += 1;
    }

    Ok(())
}

pub fn validate_curve(
    curve: &Curve16,
    range: ValueRange,
    err: ValidationError,
) -> Result<(), ValidationError> {
    if curve.axis.len > 16 {
        return Err(ValidationError::CurveDimensionMismatch);
    }
    validate_axis(&curve.axis)?;

    let len = curve.axis.len as usize;
    let mut idx = 0usize;
    while idx < len {
        let value = curve.values[idx] as u32;
        if value < range.min || value > range.max {
            return Err(err);
        }
        idx += 1;
    }

    Ok(())
}

pub fn validate_signed_curve(
    curve: &SignedCurve16,
    range: SignedValueRange,
    err: ValidationError,
) -> Result<(), ValidationError> {
    if curve.axis.len > 16 {
        return Err(ValidationError::CurveDimensionMismatch);
    }
    validate_axis(&curve.axis)?;

    let len = curve.axis.len as usize;
    let mut idx = 0usize;
    while idx < len {
        let value = curve.values[idx] as i32;
        if value < range.min || value > range.max {
            return Err(err);
        }
        idx += 1;
    }

    Ok(())
}

pub fn validate_table_u16(
    table: &Table2D16<u16>,
    range: ValueRange,
    err: ValidationError,
) -> Result<(), ValidationError> {
    validate_axis(&table.rpm_axis)?;
    validate_axis(&table.load_axis)?;

    let load_len = table.load_axis.len as usize;
    let rpm_len = table.rpm_axis.len as usize;

    let mut load_idx = 0usize;
    while load_idx < load_len {
        let mut rpm_idx = 0usize;
        while rpm_idx < rpm_len {
            let value = table.values[load_idx][rpm_idx] as u32;
            if value < range.min || value > range.max {
                return Err(err);
            }
            rpm_idx += 1;
        }
        load_idx += 1;
    }

    Ok(())
}

fn validate_table_u32(
    table: &Table2D16<u32>,
    range: ValueRange,
    err: ValidationError,
) -> Result<(), ValidationError> {
    validate_axis(&table.rpm_axis)?;
    validate_axis(&table.load_axis)?;

    let load_len = table.load_axis.len as usize;
    let rpm_len = table.rpm_axis.len as usize;

    let mut load_idx = 0usize;
    while load_idx < load_len {
        let mut rpm_idx = 0usize;
        while rpm_idx < rpm_len {
            let value = table.values[load_idx][rpm_idx];
            if value < range.min || value > range.max {
                return Err(err);
            }
            rpm_idx += 1;
        }
        load_idx += 1;
    }

    Ok(())
}

pub fn validate_table_i16(
    table: &Table2D16<i16>,
    range: SignedValueRange,
    err: ValidationError,
) -> Result<(), ValidationError> {
    validate_axis(&table.rpm_axis)?;
    validate_axis(&table.load_axis)?;

    let load_len = table.load_axis.len as usize;
    let rpm_len = table.rpm_axis.len as usize;

    let mut load_idx = 0usize;
    while load_idx < load_len {
        let mut rpm_idx = 0usize;
        while rpm_idx < rpm_len {
            let value = table.values[load_idx][rpm_idx] as i32;
            if value < range.min || value > range.max {
                return Err(err);
            }
            rpm_idx += 1;
        }
        load_idx += 1;
    }

    Ok(())
}

pub fn validate_calibration(raw: Calibration) -> Result<ValidatedCalibration, ValidationError> {
    validate_table_u16(
        &raw.ve_table,
        ValueRange { min: 0, max: 30000 },
        ValidationError::VeOutOfRange,
    )?;
    validate_table_u16(
        &raw.afr_target_table,
        ValueRange {
            min: 500,
            max: 3000,
        },
        ValidationError::AfrOutOfRange,
    )?;
    validate_table_i16(
        &raw.spark_advance_table_deg10,
        SignedValueRange {
            min: -7200,
            max: 7200,
        },
        ValidationError::SparkAdvanceOutOfRange,
    )?;
    validate_table_u32(
        &raw.dwell_table_us,
        ValueRange { min: 1, max: 20000 },
        ValidationError::DwellOutOfRange,
    )?;
    validate_table_u16(
        &raw.injection_target_table_deg10,
        ValueRange { min: 0, max: 7199 },
        ValidationError::InjectionTargetOutOfRange,
    )?;
    validate_table_u16(
        &raw.deadtime_table_us,
        ValueRange { min: 0, max: 20000 },
        ValidationError::DwellOutOfRange,
    )?;
    validate_curve(
        &raw.cranking_curve,
        ValueRange { min: 0, max: 6000 },
        ValidationError::CorrectionAboveLimit,
    )?;
    validate_table_u16(
        &raw.afterstart_table,
        ValueRange { min: 0, max: 4000 },
        ValidationError::CorrectionAboveLimit,
    )?;
    validate_curve(
        &raw.warmup_curve,
        ValueRange { min: 0, max: 4000 },
        ValidationError::CorrectionAboveLimit,
    )?;
    validate_curve(
        &raw.ae_tps_threshold_curve,
        ValueRange { min: 0, max: 20000 },
        ValidationError::CorrectionAboveLimit,
    )?;
    validate_curve(
        &raw.ae_map_threshold_curve,
        ValueRange { min: 0, max: 20000 },
        ValidationError::CorrectionAboveLimit,
    )?;
    validate_curve(
        &raw.ae_shot_curve_us,
        ValueRange { min: 0, max: 20000 },
        ValidationError::CorrectionAboveLimit,
    )?;
    validate_curve(
        &raw.ae_decay_steps_curve,
        ValueRange { min: 0, max: 20000 },
        ValidationError::CorrectionAboveLimit,
    )?;
    validate_curve(
        &raw.ae_decay_ratio_curve_x1000,
        ValueRange { min: 0, max: 20000 },
        ValidationError::CorrectionAboveLimit,
    )?;
    validate_curve(
        &raw.clt_corr_curve,
        ValueRange { min: 0, max: 4000 },
        ValidationError::CorrectionAboveLimit,
    )?;
    validate_curve(
        &raw.iat_corr_curve,
        ValueRange { min: 0, max: 4000 },
        ValidationError::CorrectionAboveLimit,
    )?;
    validate_curve(
        &raw.baro_corr_curve,
        ValueRange { min: 0, max: 4000 },
        ValidationError::CorrectionAboveLimit,
    )?;
    validate_curve(
        &raw.vbat_corr_curve,
        ValueRange { min: 0, max: 4000 },
        ValidationError::CorrectionAboveLimit,
    )?;

    if raw.required_fuel_us == 0 {
        return Err(ValidationError::RequiredFuelZero);
    }
    if raw.pref_kpa10 == 0 {
        return Err(ValidationError::ReferencePressureZero);
    }
    if raw.stoich_afr_x100 < 500 || raw.stoich_afr_x100 > 3000 {
        return Err(ValidationError::AfrOutOfRange);
    }
    if raw.dfco_entry_rpm.0 <= raw.dfco_exit_rpm.0
        || raw.dfco_entry_tps_x100 > raw.dfco_exit_tps_x100
    {
        return Err(ValidationError::DfcoConfigInvalid);
    }
    if raw.hard_rev_rpm.0 <= raw.soft_rev_rpm.0
        || raw.rev_hysteresis_rpm.0 == 0
        || raw.soft_retard_max_deg10 > 720
    {
        return Err(ValidationError::RevLimitConfigInvalid);
    }
    if raw.launch_rpm_limit.0 > 20_000 {
        return Err(ValidationError::LaunchConfigInvalid);
    }
    if raw.flat_shift_rpm_min.0 > 20_000 {
        return Err(ValidationError::FlatShiftConfigInvalid);
    }
    if raw.knock_threshold_x100 > 10000
        || raw.knock_retard_step_deg10 > 200
        || raw.knock_retard_max_deg10 > 720
        || raw.knock_recovery_step_deg10 > 200
    {
        return Err(ValidationError::KnockConfigInvalid);
    }
    if raw.idle_base_duty_x1000 > 1000 {
        return Err(ValidationError::IdleConfigInvalid);
    }
    validate_signed_curve(
        &raw.idle_advance_curve_deg10,
        SignedValueRange {
            min: -7200,
            max: 7200,
        },
        ValidationError::SparkAdvanceOutOfRange,
    )?;
    validate_signed_curve(
        &raw.clt_timing_corr_curve_deg10,
        SignedValueRange {
            min: -7200,
            max: 7200,
        },
        ValidationError::SparkAdvanceOutOfRange,
    )?;
    validate_signed_curve(
        &raw.iat_timing_corr_curve_deg10,
        SignedValueRange {
            min: -7200,
            max: 7200,
        },
        ValidationError::SparkAdvanceOutOfRange,
    )?;
    if raw.idle_timing_rpm_max.0 > 20_000
        || raw.idle_timing_tps_max_x100 > 10_000
        || raw.idle_timing_min_trim_deg10 > raw.idle_timing_max_trim_deg10
        || raw.idle_timing_min_trim_deg10 < -7200
        || raw.idle_timing_max_trim_deg10 > 7200
    {
        return Err(ValidationError::IdleConfigInvalid);
    }
    if raw.o2_wideband_afr_min_x100 < 500
        || raw.o2_wideband_afr_min_x100 > 3000
        || raw.o2_wideband_afr_max_x100 < 500
        || raw.o2_wideband_afr_max_x100 > 3000
        || raw.o2_wideband_afr_min_x100 >= raw.o2_wideband_afr_max_x100
        || raw.o2_narrowband_threshold_counts > 4095
        || raw.o2_narrowband_hysteresis_counts > 4095
        || raw.o2_narrowband_rich_afr_x100 < 500
        || raw.o2_narrowband_rich_afr_x100 > 3000
        || raw.o2_narrowband_lean_afr_x100 < 500
        || raw.o2_narrowband_lean_afr_x100 > 3000
        || raw.o2_narrowband_rich_afr_x100 > raw.o2_narrowband_lean_afr_x100
    {
        return Err(ValidationError::O2ConfigInvalid);
    }
    if raw.tps_adc_min_counts >= raw.tps_adc_max_counts || raw.tps_adc_max_counts > 4095 {
        return Err(ValidationError::TpsConfigInvalid);
    }
    if raw.pw_max_us == 0 {
        return Err(ValidationError::PwMaxZero);
    }
    if raw.cylinder_phase_deg10.count == 0 {
        return Err(ValidationError::CylinderCountZero);
    }
    if raw.cylinder_phase_deg10.count > 8 {
        return Err(ValidationError::CylinderCountTooLarge);
    }

    let count = raw.cylinder_phase_deg10.count as usize;
    let mut idx = 0usize;
    while idx < count {
        let phase = raw.cylinder_phase_deg10.values[idx];
        if phase > 7199 {
            return Err(ValidationError::AngleOutOfRange);
        }
        idx += 1;
    }

    Ok(ValidatedCalibration(raw))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{
        Axis16, Curve16, CylinderArrayU16, FuelModel, InjectionAngleMode, PwMaxPolicy,
        SignedCurve16, TrimPolicy,
    };

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

    fn curve(values: &[u16]) -> Curve16 {
        let mut curve = Curve16 {
            axis: axis(values),
            ..Curve16::default()
        };
        let mut idx = 0usize;
        while idx < values.len() {
            curve.values[idx] = values[idx];
            idx += 1;
        }
        curve
    }

    fn signed_curve(value: i16) -> SignedCurve16 {
        let mut curve = SignedCurve16 {
            axis: axis(&[500, 1000]),
            ..SignedCurve16::default()
        };
        curve.values[0] = value;
        curve.values[1] = value;
        curve
    }

    fn calibration() -> Calibration {
        Calibration {
            fuel_model: FuelModel::SpeedDensityRequiredFuel,
            ve_table: Table2D16 {
                rpm_axis: axis(&[100, 200]),
                load_axis: axis(&[10, 20]),
                values: {
                    let mut values = [[0u16; 16]; 16];
                    values[0][0] = 1000;
                    values[0][1] = 1000;
                    values[1][0] = 1000;
                    values[1][1] = 1000;
                    values
                },
            },
            afr_target_table: Table2D16 {
                rpm_axis: axis(&[100, 200]),
                load_axis: axis(&[10, 20]),
                values: {
                    let mut values = [[0u16; 16]; 16];
                    values[0][0] = 1470;
                    values[0][1] = 1470;
                    values[1][0] = 1470;
                    values[1][1] = 1470;
                    values
                },
            },
            spark_advance_table_deg10: Table2D16 {
                rpm_axis: axis(&[100, 200]),
                load_axis: axis(&[10, 20]),
                values: [[0i16; 16]; 16],
            },
            dwell_table_us: Table2D16 {
                rpm_axis: axis(&[100, 200]),
                load_axis: axis(&[10, 20]),
                values: {
                    let mut values = [[0u32; 16]; 16];
                    values[0][0] = 1000;
                    values[0][1] = 1000;
                    values[1][0] = 1000;
                    values[1][1] = 1000;
                    values
                },
            },
            injection_target_table_deg10: Table2D16 {
                rpm_axis: axis(&[100, 200]),
                load_axis: axis(&[10, 20]),
                values: [[0u16; 16]; 16],
            },
            deadtime_table_us: Table2D16 {
                rpm_axis: axis(&[1000, 13000]),
                load_axis: axis(&[1000, 3000]),
                values: {
                    let mut values = [[0u16; 16]; 16];
                    values[0][0] = 1000;
                    values[0][1] = 1000;
                    values[1][0] = 1000;
                    values[1][1] = 1000;
                    values
                },
            },
            cranking_curve: curve(&[1000, 2000]),
            afterstart_table: Table2D16 {
                rpm_axis: axis(&[0, 10]),
                load_axis: axis(&[0, 1000]),
                values: {
                    let mut values = [[0u16; 16]; 16];
                    values[0][0] = 1000;
                    values[0][1] = 1000;
                    values[1][0] = 1000;
                    values[1][1] = 1000;
                    values
                },
            },
            afterstart_window_cycles: 0,
            warmup_curve: curve(&[1000, 2000]),
            ae_tps_threshold_curve: curve(&[1000, 2000]),
            ae_map_threshold_curve: curve(&[1000, 2000]),
            ae_shot_curve_us: curve(&[0, 100]),
            ae_decay_steps_curve: curve(&[0, 16]),
            ae_decay_ratio_curve_x1000: curve(&[1000, 2000]),
            dfco_entry_rpm: crate::Rpm(2500),
            dfco_exit_rpm: crate::Rpm(2000),
            dfco_entry_tps_x100: 200,
            dfco_exit_tps_x100: 300,
            dfco_entry_map_kpa10: crate::Kpa10(500),
            dfco_delay_cycles: 2,
            soft_rev_rpm: crate::Rpm(6000),
            hard_rev_rpm: crate::Rpm(6500),
            rev_hysteresis_rpm: crate::Rpm(100),
            soft_retard_max_deg10: 100,
            launch_rpm_limit: crate::Rpm(5000),
            launch_cut_cycles: 4,
            flat_shift_rpm_min: crate::Rpm(5000),
            flat_shift_cut_cycles: 4,
            knock_threshold_x100: 500,
            knock_retard_step_deg10: 20,
            knock_retard_max_deg10: 200,
            knock_recovery_step_deg10: 10,
            knock_recovery_delay_cycles: 2,
            tps_adc_min_counts: 0,
            tps_adc_max_counts: 4095,
            idle_target_rpm: crate::Rpm(900),
            idle_base_duty_x1000: 0,
            idle_kp_x1000: 0,
            idle_ki_x1000: 0,
            idle_timing_enabled: false,
            idle_timing_pid_enabled: false,
            idle_timing_rpm_max: crate::Rpm(1200),
            idle_timing_tps_max_x100: 200,
            idle_advance_curve_deg10: signed_curve(0),
            idle_timing_kp_x1000: 0,
            idle_timing_ki_x1000: 0,
            idle_timing_min_trim_deg10: -300,
            idle_timing_max_trim_deg10: 300,
            clt_timing_corr_curve_deg10: signed_curve(0),
            iat_timing_corr_curve_deg10: signed_curve(0),
            lambda_kp_x1000: 0,
            lambda_ki_x1000: 0,
            o2_sensor_mode: crate::O2SensorMode::WidebandLinear,
            o2_wideband_afr_min_x100: 500,
            o2_wideband_afr_max_x100: 3000,
            o2_narrowband_threshold_counts: 2048,
            o2_narrowband_hysteresis_counts: 64,
            o2_narrowband_rich_afr_x100: 1400,
            o2_narrowband_lean_afr_x100: 1550,
            clt_corr_curve: curve(&[1000, 2000]),
            iat_corr_curve: curve(&[1000, 2000]),
            baro_corr_curve: curve(&[1000, 2000]),
            vbat_corr_curve: curve(&[1000, 2000]),
            required_fuel_us: 100,
            pref_kpa10: 100,
            stoich_afr_x100: 1470,
            trim_policy: TrimPolicy::Identity,
            pw_max_policy: PwMaxPolicy::Fixed,
            pw_max_us: 1000,
            injection_angle_mode: InjectionAngleMode::EndOfInjection,
            cylinder_phase_deg10: CylinderArrayU16 {
                count: 1,
                values: {
                    let mut values = [0u16; 16];
                    values[0] = 0;
                    values
                },
            },
        }
    }

    #[test]
    fn validate_axis_reports_length_and_order_errors() {
        let short = Axis16 {
            len: 1,
            ..Axis16::default()
        };
        assert_eq!(validate_axis(&short), Err(ValidationError::AxisTooShort));

        let mut unsorted = axis(&[10, 20]);
        assert_eq!(validate_axis(&unsorted), Ok(()));

        unsorted.len = 17;
        assert_eq!(
            validate_axis(&unsorted),
            Err(ValidationError::TableDimensionMismatch)
        );
    }

    #[test]
    fn validation_curve_and_table_ranges_report_expected_errors() {
        let mut bad_curve = curve(&[1, 2]);
        bad_curve.values[1] = 5001;
        assert_eq!(
            validate_curve(
                &bad_curve,
                ValueRange { min: 0, max: 4000 },
                ValidationError::CorrectionAboveLimit
            ),
            Err(ValidationError::CorrectionAboveLimit)
        );

        let mut bad_table = Calibration::default().ve_table;
        bad_table.rpm_axis = axis(&[100, 200]);
        bad_table.load_axis = axis(&[10, 20]);
        bad_table.values[0][0] = 30001;
        assert_eq!(
            validate_table_u16(
                &bad_table,
                ValueRange { min: 0, max: 30000 },
                ValidationError::VeOutOfRange
            ),
            Err(ValidationError::VeOutOfRange)
        );
    }

    #[test]
    fn validation_curve_detects_axis_order_error_before_range_error() {
        let mut bad_curve = curve(&[1, 2]);
        bad_curve.axis.values[1] = bad_curve.axis.values[0];
        bad_curve.values[0] = 5001;
        assert_eq!(
            validate_curve(
                &bad_curve,
                ValueRange { min: 0, max: 4000 },
                ValidationError::CorrectionAboveLimit
            ),
            Err(ValidationError::AxisNotStrictlyIncreasing)
        );
    }

    #[test]
    fn validation_table_detects_rpm_axis_error_before_load_or_cell_error() {
        let mut bad_table = Calibration::default().ve_table;
        bad_table.rpm_axis = axis(&[10, 20]);
        bad_table.load_axis = axis(&[1, 2]);
        bad_table.values[0][0] = 30001;
        bad_table.rpm_axis.values[1] = bad_table.rpm_axis.values[0];
        assert_eq!(
            validate_table_u16(
                &bad_table,
                ValueRange { min: 0, max: 30000 },
                ValidationError::VeOutOfRange
            ),
            Err(ValidationError::AxisNotStrictlyIncreasing)
        );
    }

    #[test]
    fn validation_table_detects_load_axis_error_before_cell_error() {
        let mut bad_table = Calibration::default().ve_table;
        bad_table.rpm_axis = axis(&[10, 20]);
        bad_table.load_axis = axis(&[1, 2]);
        bad_table.values[0][0] = 30001;
        bad_table.load_axis.values[1] = bad_table.load_axis.values[0];
        assert_eq!(
            validate_table_u16(
                &bad_table,
                ValueRange { min: 0, max: 30000 },
                ValidationError::VeOutOfRange
            ),
            Err(ValidationError::AxisNotStrictlyIncreasing)
        );
    }

    #[test]
    fn validation_calibration_reports_each_priority_error() {
        let mut cal = calibration();

        cal.ve_table.rpm_axis = axis(&[10]);
        cal.ve_table.load_axis = axis(&[10, 20]);
        assert_eq!(
            validate_calibration(cal).unwrap_err(),
            ValidationError::AxisTooShort
        );

        let mut cal = calibration();
        cal.required_fuel_us = 0;
        assert_eq!(
            validate_calibration(cal).unwrap_err(),
            ValidationError::RequiredFuelZero
        );

        let mut cal = calibration();
        cal.pref_kpa10 = 0;
        assert_eq!(
            validate_calibration(cal).unwrap_err(),
            ValidationError::ReferencePressureZero
        );

        let mut cal = calibration();
        cal.stoich_afr_x100 = 100;
        assert_eq!(
            validate_calibration(cal).unwrap_err(),
            ValidationError::AfrOutOfRange
        );

        let mut cal = calibration();
        cal.pw_max_us = 0;
        assert_eq!(
            validate_calibration(cal).unwrap_err(),
            ValidationError::PwMaxZero
        );

        let mut cal = calibration();
        cal.cylinder_phase_deg10.count = 0;
        assert_eq!(
            validate_calibration(cal).unwrap_err(),
            ValidationError::CylinderCountZero
        );

        let mut cal = calibration();
        cal.cylinder_phase_deg10.count = 9;
        assert_eq!(
            validate_calibration(cal).unwrap_err(),
            ValidationError::CylinderCountTooLarge
        );

        let mut cal = calibration();
        cal.cylinder_phase_deg10.values[0] = 7200;
        assert_eq!(
            validate_calibration(cal).unwrap_err(),
            ValidationError::AngleOutOfRange
        );
    }

    #[test]
    fn validation_calibration_succeeds_for_valid_input() {
        assert!(validate_calibration(calibration()).is_ok());
    }

    #[test]
    fn validation_unused_tails_are_ignored() {
        let mut cal = calibration();
        cal.ve_table.values[15][15] = 30001;
        cal.ve_table.rpm_axis.len = 2;
        cal.ve_table.load_axis.len = 2;
        assert!(validate_calibration(cal).is_ok());
    }

    #[test]
    fn validation_priority_is_first_error_wins() {
        let mut cal = calibration();
        cal.ve_table.rpm_axis = axis(&[100]);
        cal.required_fuel_us = 0;
        assert_eq!(
            validate_calibration(cal).unwrap_err(),
            ValidationError::AxisTooShort
        );

        let mut cal = calibration();
        cal.required_fuel_us = 0;
        cal.pref_kpa10 = 0;
        assert_eq!(
            validate_calibration(cal).unwrap_err(),
            ValidationError::RequiredFuelZero
        );

        let mut cal = calibration();
        cal.pref_kpa10 = 0;
        cal.stoich_afr_x100 = 100;
        assert_eq!(
            validate_calibration(cal).unwrap_err(),
            ValidationError::ReferencePressureZero
        );

        let mut cal = calibration();
        cal.dfco_entry_rpm = crate::Rpm(2000);
        cal.dfco_exit_rpm = crate::Rpm(2000);
        assert_eq!(
            validate_calibration(cal).unwrap_err(),
            ValidationError::DfcoConfigInvalid
        );

        let mut cal = calibration();
        cal.hard_rev_rpm = cal.soft_rev_rpm;
        assert_eq!(
            validate_calibration(cal).unwrap_err(),
            ValidationError::RevLimitConfigInvalid
        );

        let mut cal = calibration();
        cal.launch_rpm_limit = crate::Rpm(20_001);
        assert_eq!(
            validate_calibration(cal).unwrap_err(),
            ValidationError::LaunchConfigInvalid
        );

        let mut cal = calibration();
        cal.tps_adc_min_counts = 2000;
        cal.tps_adc_max_counts = 2000;
        assert_eq!(
            validate_calibration(cal).unwrap_err(),
            ValidationError::TpsConfigInvalid
        );
    }

    #[test]
    fn validation_covers_all_error_variants() {
        let mut seen = [false; 26];
        let variants = [
            ValidationError::AxisTooShort,
            ValidationError::AxisNotStrictlyIncreasing,
            ValidationError::TableDimensionMismatch,
            ValidationError::CurveDimensionMismatch,
            ValidationError::CylinderCountZero,
            ValidationError::CylinderCountTooLarge,
            ValidationError::AngleOutOfRange,
            ValidationError::RequiredFuelZero,
            ValidationError::ReferencePressureZero,
            ValidationError::PwMaxZero,
            ValidationError::CorrectionBelowZero,
            ValidationError::CorrectionAboveLimit,
            ValidationError::VeOutOfRange,
            ValidationError::AfrOutOfRange,
            ValidationError::DwellOutOfRange,
            ValidationError::SparkAdvanceOutOfRange,
            ValidationError::InjectionTargetOutOfRange,
            ValidationError::TargetAfrOverrideOutOfRange,
            ValidationError::DfcoConfigInvalid,
            ValidationError::RevLimitConfigInvalid,
            ValidationError::LaunchConfigInvalid,
            ValidationError::FlatShiftConfigInvalid,
            ValidationError::KnockConfigInvalid,
            ValidationError::IdleConfigInvalid,
            ValidationError::TpsConfigInvalid,
            ValidationError::O2ConfigInvalid,
        ];

        for (idx, variant) in variants.iter().enumerate() {
            match variant {
                ValidationError::AxisTooShort => seen[idx] = true,
                ValidationError::AxisNotStrictlyIncreasing => seen[idx] = true,
                ValidationError::TableDimensionMismatch => seen[idx] = true,
                ValidationError::CurveDimensionMismatch => seen[idx] = true,
                ValidationError::CylinderCountZero => seen[idx] = true,
                ValidationError::CylinderCountTooLarge => seen[idx] = true,
                ValidationError::AngleOutOfRange => seen[idx] = true,
                ValidationError::RequiredFuelZero => seen[idx] = true,
                ValidationError::ReferencePressureZero => seen[idx] = true,
                ValidationError::PwMaxZero => seen[idx] = true,
                ValidationError::CorrectionBelowZero => seen[idx] = true,
                ValidationError::CorrectionAboveLimit => seen[idx] = true,
                ValidationError::VeOutOfRange => seen[idx] = true,
                ValidationError::AfrOutOfRange => seen[idx] = true,
                ValidationError::DwellOutOfRange => seen[idx] = true,
                ValidationError::SparkAdvanceOutOfRange => seen[idx] = true,
                ValidationError::InjectionTargetOutOfRange => seen[idx] = true,
                ValidationError::TargetAfrOverrideOutOfRange => seen[idx] = true,
                ValidationError::DfcoConfigInvalid => seen[idx] = true,
                ValidationError::RevLimitConfigInvalid => seen[idx] = true,
                ValidationError::LaunchConfigInvalid => seen[idx] = true,
                ValidationError::FlatShiftConfigInvalid => seen[idx] = true,
                ValidationError::KnockConfigInvalid => seen[idx] = true,
                ValidationError::IdleConfigInvalid => seen[idx] = true,
                ValidationError::TpsConfigInvalid => seen[idx] = true,
                ValidationError::O2ConfigInvalid => seen[idx] = true,
            }
        }

        assert!(seen.iter().all(|v| *v));
    }
}
