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
#[path = "validation_tests.rs"]
mod tests;
