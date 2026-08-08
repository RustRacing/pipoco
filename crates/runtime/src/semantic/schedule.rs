use crate::runtime_full_sequential_authorized;
use ecu_domain::{EngineTimeAuthority, SyncState};

use super::fuel::semantic_bilerp_u16;
use super::fuel::{clamp_u16_s, semantic_find_segment};
use super::types::*;

// --------------------------------------------------------------------------
// v10 Runtime Semantic Schedule Evaluator — pure, deterministic, no_std
// --------------------------------------------------------------------------

fn semantic_lerp_i32(x0: u16, x1: u16, y0: i32, y1: i32, x: u16) -> i32 {
    if x1 <= x0 {
        return y0;
    }
    let x_clip = clamp_u16_s(x, x0, x1);
    let num = (x_clip - x0) as i64;
    let den = (x1 - x0) as i64;
    let delta = y1 as i64 - y0 as i64;
    let product = delta * num;
    let quotient = product / den;
    let remainder = product % den;
    let correction = if product < 0 && remainder != 0 { -1 } else { 0 };
    (y0 as i64 + quotient + correction) as i32
}

fn semantic_lerp_u32(x0: u16, x1: u16, y0: u32, y1: u32, x: u16) -> u32 {
    if x1 <= x0 {
        return y0;
    }
    let x_clip = clamp_u16_s(x, x0, x1);
    let num = (x_clip - x0) as i128;
    let den = (x1 - x0) as i128;
    let delta = y1 as i128 - y0 as i128;
    let product = delta * num;
    let quotient = product / den;
    let remainder = product % den;
    let correction = if product < 0 && remainder != 0 { -1 } else { 0 };
    (y0 as i128 + quotient + correction) as u32
}

/// Bilinear interpolation on a 2D table of i16 (floor semantics).
/// Handles negative values with widened intermediates.
fn semantic_bilerp_i16(table: &RuntimeSemanticTable2dI16, rpm: u16, load: u16) -> i32 {
    let len_rpm = table.rpm_axis.len as usize;
    let len_load = table.load_axis.len as usize;
    if len_rpm < 2 || len_load < 2 {
        return table.values[0][0] as i32;
    }
    let rpm_idx = semantic_find_segment(&table.rpm_axis, rpm);
    let load_idx = semantic_find_segment(&table.load_axis, load);

    let rpm_lo = table.rpm_axis.values[rpm_idx];
    let rpm_hi = table.rpm_axis.values[(rpm_idx + 1).min(len_rpm - 1)];
    let load_lo = table.load_axis.values[load_idx];
    let load_hi = table.load_axis.values[(load_idx + 1).min(len_load - 1)];

    let v00 = table.values[load_idx][rpm_idx] as i32;
    let v01 = table.values[(load_idx + 1).min(len_load - 1)][rpm_idx] as i32;
    let v10 = table.values[load_idx][(rpm_idx + 1).min(len_rpm - 1)] as i32;
    let v11 = table.values[(load_idx + 1).min(len_load - 1)][(rpm_idx + 1).min(len_rpm - 1)] as i32;

    let interp_lo = semantic_lerp_i32(rpm_lo, rpm_hi, v00, v10, rpm);
    let interp_hi = semantic_lerp_i32(rpm_lo, rpm_hi, v01, v11, rpm);
    semantic_lerp_i32(load_lo, load_hi, interp_lo, interp_hi, load)
}

/// Bilinear interpolation on a 2D table of u32 (floor semantics).
/// Matches the structure of semantic_bilerp_u16 but returns u32.
fn semantic_bilerp_u32(table: &RuntimeSemanticTable2dU32, rpm: u16, load: u16) -> u32 {
    let len_rpm = table.rpm_axis.len as usize;
    let len_load = table.load_axis.len as usize;
    if len_rpm < 2 || len_load < 2 {
        return table.values[0][0];
    }
    let rpm_idx = semantic_find_segment(&table.rpm_axis, rpm);
    let load_idx = semantic_find_segment(&table.load_axis, load);

    let rpm_lo = table.rpm_axis.values[rpm_idx];
    let rpm_hi = table.rpm_axis.values[(rpm_idx + 1).min(len_rpm - 1)];
    let load_lo = table.load_axis.values[load_idx];
    let load_hi = table.load_axis.values[(load_idx + 1).min(len_load - 1)];

    let v00 = table.values[load_idx][rpm_idx];
    let v01 = table.values[(load_idx + 1).min(len_load - 1)][rpm_idx];
    let v10 = table.values[load_idx][(rpm_idx + 1).min(len_rpm - 1)];
    let v11 = table.values[(load_idx + 1).min(len_load - 1)][(rpm_idx + 1).min(len_rpm - 1)];

    let interp_lo = semantic_lerp_u32(rpm_lo, rpm_hi, v00, v10, rpm);
    let interp_hi = semantic_lerp_u32(rpm_lo, rpm_hi, v01, v11, rpm);
    semantic_lerp_u32(load_lo, load_hi, interp_lo, interp_hi, load)
}

/// Normalize an i32 angle into [0, 7200) for signed values.
#[inline]
fn norm7200_i32(x: i32) -> u16 {
    // Normalize using mod 7200, then handle negative by adding 7200
    let mut v = x % 7200;
    if v < 0 {
        v += 7200;
    }
    v as u16
}

/// Validate the schedule calibration tables and cylinder phases.
fn validate_schedule_calibration(
    cal: &RuntimeSemanticScheduleCalibration,
) -> Result<(), RuntimeSemanticScheduleError> {
    // Check spark_advance_table_deg10 axes
    {
        let len_rpm = cal.spark_advance_table_deg10.rpm_axis.len as usize;
        let len_load = cal.spark_advance_table_deg10.load_axis.len as usize;
        if !(2..=RUNTIME_SEMANTIC_TABLE_LEN).contains(&len_rpm)
            || !(2..=RUNTIME_SEMANTIC_TABLE_LEN).contains(&len_load)
        {
            return Err(RuntimeSemanticScheduleError::AxisLenInvalid);
        }
    }
    // Check dwell_table_us axes
    {
        let len_rpm = cal.dwell_table_us.rpm_axis.len as usize;
        let len_load = cal.dwell_table_us.load_axis.len as usize;
        if !(2..=RUNTIME_SEMANTIC_TABLE_LEN).contains(&len_rpm)
            || !(2..=RUNTIME_SEMANTIC_TABLE_LEN).contains(&len_load)
        {
            return Err(RuntimeSemanticScheduleError::AxisLenInvalid);
        }
    }
    // Check injection_target_table_deg10 axes
    {
        let len_rpm = cal.injection_target_table_deg10.rpm_axis.len as usize;
        let len_load = cal.injection_target_table_deg10.load_axis.len as usize;
        if !(2..=RUNTIME_SEMANTIC_TABLE_LEN).contains(&len_rpm)
            || !(2..=RUNTIME_SEMANTIC_TABLE_LEN).contains(&len_load)
        {
            return Err(RuntimeSemanticScheduleError::AxisLenInvalid);
        }
    }

    // Validate each axis is strictly increasing
    let mut idx = 0usize;
    let len_rpm = cal.spark_advance_table_deg10.rpm_axis.len as usize;
    while idx + 1 < len_rpm {
        if cal.spark_advance_table_deg10.rpm_axis.values[idx]
            >= cal.spark_advance_table_deg10.rpm_axis.values[idx + 1]
        {
            return Err(RuntimeSemanticScheduleError::AxisNotStrictlyIncreasing);
        }
        idx += 1;
    }
    let mut idx = 0usize;
    let len_load = cal.spark_advance_table_deg10.load_axis.len as usize;
    while idx + 1 < len_load {
        if cal.spark_advance_table_deg10.load_axis.values[idx]
            >= cal.spark_advance_table_deg10.load_axis.values[idx + 1]
        {
            return Err(RuntimeSemanticScheduleError::AxisNotStrictlyIncreasing);
        }
        idx += 1;
    }
    let mut idx = 0usize;
    let len_rpm = cal.dwell_table_us.rpm_axis.len as usize;
    while idx + 1 < len_rpm {
        if cal.dwell_table_us.rpm_axis.values[idx] >= cal.dwell_table_us.rpm_axis.values[idx + 1] {
            return Err(RuntimeSemanticScheduleError::AxisNotStrictlyIncreasing);
        }
        idx += 1;
    }
    let mut idx = 0usize;
    let len_load = cal.dwell_table_us.load_axis.len as usize;
    while idx + 1 < len_load {
        if cal.dwell_table_us.load_axis.values[idx] >= cal.dwell_table_us.load_axis.values[idx + 1]
        {
            return Err(RuntimeSemanticScheduleError::AxisNotStrictlyIncreasing);
        }
        idx += 1;
    }
    let mut idx = 0usize;
    let len_rpm = cal.injection_target_table_deg10.rpm_axis.len as usize;
    while idx + 1 < len_rpm {
        if cal.injection_target_table_deg10.rpm_axis.values[idx]
            >= cal.injection_target_table_deg10.rpm_axis.values[idx + 1]
        {
            return Err(RuntimeSemanticScheduleError::AxisNotStrictlyIncreasing);
        }
        idx += 1;
    }
    let mut idx = 0usize;
    let len_load = cal.injection_target_table_deg10.load_axis.len as usize;
    while idx + 1 < len_load {
        if cal.injection_target_table_deg10.load_axis.values[idx]
            >= cal.injection_target_table_deg10.load_axis.values[idx + 1]
        {
            return Err(RuntimeSemanticScheduleError::AxisNotStrictlyIncreasing);
        }
        idx += 1;
    }

    // Cylinder count
    if !(1..=8).contains(&cal.cylinder_phase_deg10.count) {
        return Err(RuntimeSemanticScheduleError::CylinderCountInvalid);
    }

    // Cylinder phase values must be < 7200
    let mut idx = 0usize;
    let count = cal.cylinder_phase_deg10.count as usize;
    while idx < count {
        if cal.cylinder_phase_deg10.values[idx] >= 7200 {
            return Err(RuntimeSemanticScheduleError::CylinderPhaseInvalid);
        }
        idx += 1;
    }

    Ok(())
}

/// The main v10 semantic schedule evaluator.
///
/// Pure, deterministic, no_std/no_alloc, Verus-friendly.
pub fn runtime_semantic_evaluate_schedule(
    cal: &RuntimeSemanticScheduleCalibration,
    input: RuntimeSemanticInputSnapshot,
    fuel: RuntimeSemanticFuelObservations,
) -> Result<RuntimeSemanticScheduleObservations, RuntimeSemanticScheduleError> {
    runtime_semantic_evaluate_schedule_with_authority(cal, input, fuel, EngineTimeAuthority::none())
}

/// Semantic schedule evaluator variant for full sequential authority checks.
pub fn runtime_semantic_evaluate_schedule_with_authority(
    cal: &RuntimeSemanticScheduleCalibration,
    input: RuntimeSemanticInputSnapshot,
    fuel: RuntimeSemanticFuelObservations,
    authority: EngineTimeAuthority,
) -> Result<RuntimeSemanticScheduleObservations, RuntimeSemanticScheduleError> {
    // Validate calibration
    validate_schedule_calibration(cal)?;

    let rpm = input.rpm.get();
    let load_kpa10 = input.load_kpa10.get();

    // Duration conversions
    let injection_duration_deg10 = {
        let raw = (fuel.pw_corr_us as u64 * rpm as u64 * 6u64) / 100_000u64;
        if raw > u16::MAX as u64 {
            return Err(RuntimeSemanticScheduleError::DurationOverflow);
        }
        raw as u16
    };

    let dwell_us = semantic_bilerp_u32(&cal.dwell_table_us, rpm, load_kpa10);
    let dwell_duration_deg10 = {
        let raw = (dwell_us as u64 * rpm as u64 * 6u64) / 100_000u64;
        if raw > u16::MAX as u64 {
            return Err(RuntimeSemanticScheduleError::DurationOverflow);
        }
        raw as u16
    };

    // Table lookups
    let base_spark_advance_deg10 =
        semantic_bilerp_i16(&cal.spark_advance_table_deg10, rpm, load_kpa10);
    let spark_advance_deg10 = (base_spark_advance_deg10 + fuel.advance_deg10_trim as i32)
        .clamp(i16::MIN as i32, i16::MAX as i32);
    let injection_target_deg10 =
        semantic_bilerp_u16(&cal.injection_target_table_deg10, rpm, load_kpa10);

    // Per-cylinder angle law
    let mut soi_deg10_values = [0u16; RUNTIME_SEMANTIC_TABLE_LEN];
    let mut eoi_deg10_values = [0u16; RUNTIME_SEMANTIC_TABLE_LEN];
    let mut spark_deg10_values = [0u16; RUNTIME_SEMANTIC_TABLE_LEN];
    let mut dwell_start_deg10_values = [0u16; RUNTIME_SEMANTIC_TABLE_LEN];

    let count = cal.cylinder_phase_deg10.count as usize;
    let mut cyl = 0usize;
    while cyl < count {
        let phase = cal.cylinder_phase_deg10.values[cyl];

        let (soi, eoi) = match cal.injection_angle_mode {
            RuntimeSemanticInjectionAngleMode::EndOfInjection => {
                // Use i32 arithmetic then norm7200 to match spec behavior.
                // phase and injection_target are u16 but subtraction may underflow
                // (e.g., phase=0, inj_target=360: 0-360 = -360 on i32, norm7200 => 6840).
                let phase_i = phase as i32;
                let inj_tgt_i = injection_target_deg10 as i32;
                let inj_dur_i = injection_duration_deg10 as i32;
                let eoi_i = phase_i - inj_tgt_i;
                let eoi = norm7200_i32(eoi_i);
                let soi_i = eoi_i - inj_dur_i;
                let soi = norm7200_i32(soi_i);
                (soi, eoi)
            }
            RuntimeSemanticInjectionAngleMode::StartOfInjection => {
                let phase_i = phase as i32;
                let inj_tgt_i = injection_target_deg10 as i32;
                let inj_dur_i = injection_duration_deg10 as i32;
                let soi_i = phase_i - inj_tgt_i;
                let soi = norm7200_i32(soi_i);
                let eoi_i = soi_i + inj_dur_i;
                let eoi = norm7200_i32(eoi_i);
                (soi, eoi)
            }
        };

        let phase_i: i32 = phase as i32;
        let spark = norm7200_i32(phase_i - spark_advance_deg10);
        let dwell_start = norm7200_i32(phase_i - spark_advance_deg10 - dwell_duration_deg10 as i32);

        soi_deg10_values[cyl] = soi;
        eoi_deg10_values[cyl] = eoi;
        spark_deg10_values[cyl] = spark;
        dwell_start_deg10_values[cyl] = dwell_start;

        cyl += 1;
    }

    // Event emission
    let engine_enabled = matches!(
        input.mode,
        RuntimeSemanticEngineMode::Cranking | RuntimeSemanticEngineMode::Running
    );
    let sync_enabled = matches!(input.sync, SyncState::Locked { .. })
        && runtime_full_sequential_authorized(authority);

    let mut events = [RuntimeSemanticScheduleEvent {
        kind: RuntimeSemanticScheduleEventKind::InjectionOpen,
        cylinder: 0,
        angle_deg10: 0,
    }; 64];
    let mut event_count: usize = 0;

    let enabled = engine_enabled && sync_enabled;
    let fuel_events_enabled = enabled && !fuel.fuel_cut && fuel.pw_corr_us > 0;
    let spark_events_enabled = enabled && !fuel.spark_cut;

    let mut cyl = 0usize;
    while cyl < count {
        if event_count + 4 > 64 {
            return Err(RuntimeSemanticScheduleError::EventBatchFull);
        }
        if fuel_events_enabled {
            // InjectionOpen
            events[event_count] = RuntimeSemanticScheduleEvent {
                kind: RuntimeSemanticScheduleEventKind::InjectionOpen,
                cylinder: cyl as u8,
                angle_deg10: soi_deg10_values[cyl],
            };
            event_count += 1;
            // InjectionClose
            events[event_count] = RuntimeSemanticScheduleEvent {
                kind: RuntimeSemanticScheduleEventKind::InjectionClose,
                cylinder: cyl as u8,
                angle_deg10: eoi_deg10_values[cyl],
            };
            event_count += 1;
        }
        if spark_events_enabled {
            // CoilChargeStart (dwell begin)
            events[event_count] = RuntimeSemanticScheduleEvent {
                kind: RuntimeSemanticScheduleEventKind::CoilChargeStart,
                cylinder: cyl as u8,
                angle_deg10: dwell_start_deg10_values[cyl],
            };
            event_count += 1;
            // CoilFire (spark)
            events[event_count] = RuntimeSemanticScheduleEvent {
                kind: RuntimeSemanticScheduleEventKind::CoilFire,
                cylinder: cyl as u8,
                angle_deg10: spark_deg10_values[cyl],
            };
            event_count += 1;
        }
        cyl += 1;
    }

    let diagnostic = if fuel.fuel_cut {
        RuntimeSemanticScheduleDiagnostic::FuelCutActive
    } else if fuel.spark_cut {
        RuntimeSemanticScheduleDiagnostic::SparkCutActive
    } else if engine_enabled && !sync_enabled {
        RuntimeSemanticScheduleDiagnostic::Unsynced
    } else {
        RuntimeSemanticScheduleDiagnostic::None
    };

    Ok(RuntimeSemanticScheduleObservations {
        injection_target_deg10: injection_target_deg10 as u16,
        spark_advance_deg10: spark_advance_deg10 as i16,
        dwell_us,
        injection_duration_deg10,
        dwell_duration_deg10,
        soi_deg10: RuntimeSemanticCylinderArrayU16 {
            count: cal.cylinder_phase_deg10.count,
            values: soi_deg10_values,
        },
        eoi_deg10: RuntimeSemanticCylinderArrayU16 {
            count: cal.cylinder_phase_deg10.count,
            values: eoi_deg10_values,
        },
        spark_deg10: RuntimeSemanticCylinderArrayU16 {
            count: cal.cylinder_phase_deg10.count,
            values: spark_deg10_values,
        },
        dwell_start_deg10: RuntimeSemanticCylinderArrayU16 {
            count: cal.cylinder_phase_deg10.count,
            values: dwell_start_deg10_values,
        },
        events: RuntimeSemanticScheduleEventBatch {
            len: event_count as u8,
            events,
        },
        diagnostic,
    })
}

#[cfg(test)]
mod tests {
    use super::norm7200_i32;

    /// Injection and spark angles are computed by subtraction, so they go
    /// negative routinely (the documented case is phase 0 with a 360 deg10
    /// injection target). No FM0016 fixture drives that path, so cover the
    /// normalization directly.
    #[test]
    fn norm7200_wraps_negative_angles_into_the_cycle() {
        assert_eq!(norm7200_i32(0), 0);
        assert_eq!(norm7200_i32(-1), 7199);
        assert_eq!(norm7200_i32(-360), 6840, "phase 0 minus a 360 deg10 target");
        assert_eq!(norm7200_i32(-7199), 1);
        assert_eq!(norm7200_i32(-7200), 0, "a whole cycle back is the origin");
        assert_eq!(norm7200_i32(-7201), 7199);
        assert_eq!(norm7200_i32(-14_400), 0);
        assert_eq!(norm7200_i32(-14_401), 7199);
    }

    #[test]
    fn norm7200_wraps_angles_at_or_beyond_a_full_cycle() {
        assert_eq!(norm7200_i32(7199), 7199);
        assert_eq!(norm7200_i32(7200), 0);
        assert_eq!(norm7200_i32(7201), 1);
        assert_eq!(norm7200_i32(14_400), 0);
        assert_eq!(norm7200_i32(14_401), 1);
    }

    /// Whatever the input, the result must be a valid crank-cycle angle.
    #[test]
    fn norm7200_output_is_always_within_the_cycle() {
        for x in [
            i32::MIN + 1,
            -1_000_000,
            -7201,
            -1,
            0,
            1,
            7199,
            7200,
            1_000_000,
            i32::MAX,
        ] {
            assert!(
                norm7200_i32(x) < 7200,
                "norm7200_i32({x}) escaped the cycle"
            );
        }
    }
}
