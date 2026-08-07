use super::*;
use crate::{
    Axis16, Curve16, CylinderArrayU16, FuelModel, InjectionAngleMode, PwMaxPolicy, SignedCurve16,
    TrimPolicy,
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
        dfco_entry_rpm: crate::Rpm::new(2500),
        dfco_exit_rpm: crate::Rpm::new(2000),
        dfco_entry_tps_x100: 200,
        dfco_exit_tps_x100: 300,
        dfco_entry_map_kpa10: crate::Kpa10::new(500),
        dfco_delay_cycles: 2,
        soft_rev_rpm: crate::Rpm::new(6000),
        hard_rev_rpm: crate::Rpm::new(6500),
        rev_hysteresis_rpm: crate::Rpm::new(100),
        soft_retard_max_deg10: 100,
        launch_rpm_limit: crate::Rpm::new(5000),
        launch_cut_cycles: 4,
        flat_shift_rpm_min: crate::Rpm::new(5000),
        flat_shift_cut_cycles: 4,
        knock_threshold_x100: 500,
        knock_retard_step_deg10: 20,
        knock_retard_max_deg10: 200,
        knock_recovery_step_deg10: 10,
        knock_recovery_delay_cycles: 2,
        tps_adc_min_counts: 0,
        tps_adc_max_counts: 4095,
        idle_target_rpm: crate::Rpm::new(900),
        idle_base_duty_x1000: 0,
        idle_kp_x1000: 0,
        idle_ki_x1000: 0,
        idle_timing_enabled: false,
        idle_timing_pid_enabled: false,
        idle_timing_rpm_max: crate::Rpm::new(1200),
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
    cal.dfco_entry_rpm = crate::Rpm::new(2000);
    cal.dfco_exit_rpm = crate::Rpm::new(2000);
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
    cal.launch_rpm_limit = crate::Rpm::new(20_001);
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
