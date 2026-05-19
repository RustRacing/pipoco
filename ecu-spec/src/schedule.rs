use crate::numeric::{duration_us_to_deg10, norm7200};
use crate::{
    spark_selected_for_mode, Degrees10, DiagnosticCode, EventBatch, EventBatchFull, EventKind,
    InputSnapshot, Kpa10, PulseWidthUs, Rpm, SignedDegrees10, Table2D16, ValidatedCalibration,
};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct FuelOutput {
    pub pw_corr_us: PulseWidthUs,
}

impl Default for FuelOutput {
    fn default() -> Self {
        Self {
            pw_corr_us: PulseWidthUs(0),
        }
    }
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct CylinderSchedule {
    pub soi_deg10: Degrees10,
    pub eoi_deg10: Degrees10,
    pub spark_deg10: Degrees10,
    pub dwell_start_deg10: Degrees10,
    pub events: EventBatch,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ScheduleOutput {
    pub injection_target_deg10: Degrees10,
    pub spark_advance_deg10: SignedDegrees10,
    pub dwell_us: PulseWidthUs,
    pub injection_duration_deg10: Degrees10,
    pub dwell_duration_deg10: Degrees10,
    pub soi_deg10: crate::CylinderArrayU16,
    pub eoi_deg10: crate::CylinderArrayU16,
    pub spark_deg10: crate::CylinderArrayU16,
    pub dwell_start_deg10: crate::CylinderArrayU16,
    pub events: EventBatch,
    pub diagnostic: DiagnosticCode,
}

impl Default for ScheduleOutput {
    fn default() -> Self {
        Self {
            injection_target_deg10: Degrees10::default(),
            spark_advance_deg10: SignedDegrees10::default(),
            dwell_us: PulseWidthUs::default(),
            injection_duration_deg10: Degrees10::default(),
            dwell_duration_deg10: Degrees10::default(),
            soi_deg10: crate::CylinderArrayU16::default(),
            eoi_deg10: crate::CylinderArrayU16::default(),
            spark_deg10: crate::CylinderArrayU16::default(),
            dwell_start_deg10: crate::CylinderArrayU16::default(),
            events: EventBatch::default(),
            diagnostic: DiagnosticCode::None,
        }
    }
}

fn lookup_table_u16(table: &Table2D16<u16>, rpm: u16, load: u16) -> u16 {
    crate::interp::bilerp_u16(table, Rpm(rpm), Kpa10(load))
}

fn lookup_table_i16(table: &Table2D16<i16>, rpm: u16, load: u16) -> i16 {
    crate::interp::bilerp_i16(table, Rpm(rpm), Kpa10(load))
}

fn lerp_u32(x0: u16, x1: u16, y0: u32, y1: u32, x: u16) -> u32 {
    if x1 <= x0 {
        return y0;
    }

    let num = (x - x0) as i128;
    let den = (x1 - x0) as i128;
    let delta = y1 as i128 - y0 as i128;
    let product = delta * num;
    let quotient = product / den;
    let remainder = product % den;
    let correction = if product < 0 && remainder != 0 { -1 } else { 0 };
    (y0 as i128 + quotient + correction) as u32
}

#[cfg(kani)]
pub(crate) fn lerp_u32_for_proof(x0: u16, x1: u16, y0: u32, y1: u32, x: u16) -> u32 {
    lerp_u32(x0, x1, y0, y1, x)
}

fn bilerp_u32(table: &Table2D16<u32>, rpm: Rpm, load: Kpa10) -> u32 {
    let rpm_len = table.rpm_axis.len as usize;
    let load_len = table.load_axis.len as usize;
    let rpm_clip = crate::numeric::clamp_u16(
        rpm.0,
        table.rpm_axis.values[0],
        table.rpm_axis.values[rpm_len - 1],
    );
    let load_clip = crate::numeric::clamp_u16(
        load.0,
        table.load_axis.values[0],
        table.load_axis.values[load_len - 1],
    );
    let rpm_seg = crate::interp::find_segment(&table.rpm_axis, rpm_clip);
    let load_seg = crate::interp::find_segment(&table.load_axis, load_clip);

    let rpm_x0 = table.rpm_axis.values[rpm_seg];
    let rpm_x1 = table.rpm_axis.values[rpm_seg + 1];
    let load_y0 = table.load_axis.values[load_seg];
    let load_y1 = table.load_axis.values[load_seg + 1];

    let lower = lerp_u32(
        rpm_x0,
        rpm_x1,
        table.values[load_seg][rpm_seg],
        table.values[load_seg][rpm_seg + 1],
        rpm_clip,
    );
    let upper = lerp_u32(
        rpm_x0,
        rpm_x1,
        table.values[load_seg + 1][rpm_seg],
        table.values[load_seg + 1][rpm_seg + 1],
        rpm_clip,
    );
    lerp_u32(load_y0, load_y1, lower, upper, load_clip)
}

fn engine_enabled(input: InputSnapshot) -> bool {
    matches!(
        input.mode,
        crate::EngineMode::Cranking | crate::EngineMode::Running
    )
}

fn sync_enabled(input: InputSnapshot) -> bool {
    input.sync == crate::SyncState::Synced
}

fn diagnostic_for_input(input: InputSnapshot) -> DiagnosticCode {
    if input.fuel_cut {
        DiagnosticCode::FuelCutActive
    } else if input.spark_cut {
        DiagnosticCode::SparkCutActive
    } else if engine_enabled(input) && !sync_enabled(input) {
        DiagnosticCode::Unsynced
    } else {
        DiagnosticCode::None
    }
}

pub fn compute_spark_advance_deg10(
    cal: &ValidatedCalibration,
    input: InputSnapshot,
) -> SignedDegrees10 {
    SignedDegrees10(lookup_table_i16(
        &cal.0.spark_advance_table_deg10,
        input.rpm.0,
        input.load_kpa10.0,
    ))
}

pub fn compute_dwell_us(cal: &ValidatedCalibration, input: InputSnapshot) -> PulseWidthUs {
    PulseWidthUs(bilerp_u32(
        &cal.0.dwell_table_us,
        input.rpm,
        input.load_kpa10,
    ))
}

pub fn compute_injection_target_deg10(
    cal: &ValidatedCalibration,
    input: InputSnapshot,
) -> Degrees10 {
    Degrees10(lookup_table_u16(
        &cal.0.injection_target_table_deg10,
        input.rpm.0,
        input.load_kpa10.0,
    ))
}

#[allow(clippy::too_many_arguments)]
fn schedule_events(
    event_batch: &mut EventBatch,
    cyl_index: u8,
    soi: Degrees10,
    eoi: Degrees10,
    spark: Degrees10,
    dwell_start: Degrees10,
    fuel_events_enabled: bool,
    spark_events_enabled: bool,
) -> Result<(), EventBatchFull> {
    let cylinder = crate::CylinderId(cyl_index);
    if fuel_events_enabled {
        event_batch.push(crate::SemanticEvent {
            kind: EventKind::InjectionOpen,
            cylinder,
            angle_deg10: soi,
        })?;
        event_batch.push(crate::SemanticEvent {
            kind: EventKind::InjectionClose,
            cylinder,
            angle_deg10: eoi,
        })?;
    }
    if spark_events_enabled {
        event_batch.push(crate::SemanticEvent {
            kind: EventKind::CoilChargeStart,
            cylinder,
            angle_deg10: dwell_start,
        })?;
        event_batch.push(crate::SemanticEvent {
            kind: EventKind::CoilFire,
            cylinder,
            angle_deg10: spark,
        })?;
    }
    Ok(())
}

pub fn schedule_cylinder(
    cal: &ValidatedCalibration,
    input: InputSnapshot,
    fuel: FuelOutput,
    cyl_index: u8,
) -> CylinderSchedule {
    schedule_cylinder_with_advance_trim(cal, input, fuel, cyl_index, SignedDegrees10(0))
}

pub fn schedule_cylinder_with_advance_trim(
    cal: &ValidatedCalibration,
    input: InputSnapshot,
    fuel: FuelOutput,
    cyl_index: u8,
    advance_trim_deg10: SignedDegrees10,
) -> CylinderSchedule {
    let phase = cal.0.cylinder_phase_deg10.values[cyl_index as usize];
    let injection_target = compute_injection_target_deg10(cal, input).0;
    let spark_advance =
        compute_spark_advance_deg10(cal, input).0 as i32 + advance_trim_deg10.0 as i32;
    let dwell_us = compute_dwell_us(cal, input).0;
    let injection_duration = duration_us_to_deg10(fuel.pw_corr_us, input.rpm).0;
    let dwell_duration = duration_us_to_deg10(PulseWidthUs(dwell_us), input.rpm).0;

    let (soi, eoi) = match cal.0.injection_angle_mode {
        crate::InjectionAngleMode::EndOfInjection => {
            let eoi = norm7200(phase as i32 - injection_target as i32);
            let soi = norm7200(eoi.0 as i32 - injection_duration as i32);
            (soi, eoi)
        }
        crate::InjectionAngleMode::StartOfInjection => {
            let soi = norm7200(phase as i32 - injection_target as i32);
            let eoi = norm7200(soi.0 as i32 + injection_duration as i32);
            (soi, eoi)
        }
    };
    let spark = norm7200(phase as i32 - spark_advance);
    let dwell_start = norm7200(spark.0 as i32 - dwell_duration as i32);

    let mut events = EventBatch::default();
    let enabled = engine_enabled(input) && sync_enabled(input);
    let fuel_events_enabled = enabled && !input.fuel_cut && fuel.pw_corr_us.0 > 0;
    let spark_events_enabled = enabled && !input.spark_cut && spark_selected_for_mode(input.mode);
    let _ = schedule_events(
        &mut events,
        cyl_index,
        soi,
        eoi,
        spark,
        dwell_start,
        fuel_events_enabled,
        spark_events_enabled,
    );

    CylinderSchedule {
        soi_deg10: soi,
        eoi_deg10: eoi,
        spark_deg10: spark,
        dwell_start_deg10: dwell_start,
        events,
    }
}

fn cylinder_count(cal: &ValidatedCalibration) -> usize {
    cal.0.cylinder_phase_deg10.count as usize
}

#[cfg(test)]
pub(crate) fn tiny_event_batch() -> EventBatch {
    EventBatch {
        len: 0,
        events: [crate::SemanticEvent::default(); crate::types::MAX_EVENTS_PER_STEP],
    }
}

pub fn schedule_all_cylinders(
    cal: &ValidatedCalibration,
    input: InputSnapshot,
    fuel: FuelOutput,
) -> ScheduleOutput {
    schedule_all_cylinders_with_advance_trim(cal, input, fuel, SignedDegrees10(0))
}

pub fn schedule_all_cylinders_with_advance_trim(
    cal: &ValidatedCalibration,
    input: InputSnapshot,
    fuel: FuelOutput,
    advance_trim_deg10: SignedDegrees10,
) -> ScheduleOutput {
    schedule_all_cylinders_with_seeded_events(
        cal,
        input,
        fuel,
        EventBatch::default(),
        advance_trim_deg10,
    )
}

fn schedule_all_cylinders_with_seeded_events(
    cal: &ValidatedCalibration,
    input: InputSnapshot,
    fuel: FuelOutput,
    seeded_events: EventBatch,
    advance_trim_deg10: SignedDegrees10,
) -> ScheduleOutput {
    let injection_target_deg10 = compute_injection_target_deg10(cal, input);
    let base_spark_advance_deg10 = compute_spark_advance_deg10(cal, input).0 as i32;
    let spark_advance_deg10 = SignedDegrees10(crate::numeric::clamp_i32(
        base_spark_advance_deg10 + advance_trim_deg10.0 as i32,
        i16::MIN as i32,
        i16::MAX as i32,
    ) as i16);
    let dwell_us = compute_dwell_us(cal, input);
    let injection_duration_deg10 = duration_us_to_deg10(fuel.pw_corr_us, input.rpm);
    let dwell_duration_deg10 = duration_us_to_deg10(dwell_us, input.rpm);
    let mut out = ScheduleOutput {
        injection_target_deg10,
        spark_advance_deg10,
        dwell_us,
        injection_duration_deg10,
        dwell_duration_deg10,
        ..ScheduleOutput::default()
    };

    out.diagnostic = diagnostic_for_input(input);
    out.events = seeded_events;

    let enabled = engine_enabled(input) && sync_enabled(input);
    let fuel_events_enabled = enabled && !input.fuel_cut && fuel.pw_corr_us.0 > 0;
    let spark_events_enabled = enabled && !input.spark_cut;
    let mut cyl = 0usize;
    while cyl < cylinder_count(cal) {
        let schedule =
            schedule_cylinder_with_advance_trim(cal, input, fuel, cyl as u8, advance_trim_deg10);
        out.soi_deg10.values[cyl] = schedule.soi_deg10.0;
        out.eoi_deg10.values[cyl] = schedule.eoi_deg10.0;
        out.spark_deg10.values[cyl] = schedule.spark_deg10.0;
        out.dwell_start_deg10.values[cyl] = schedule.dwell_start_deg10.0;

        let cylinder = crate::CylinderId(cyl as u8);
        if fuel_events_enabled {
            if out
                .events
                .push(crate::SemanticEvent {
                    kind: EventKind::InjectionOpen,
                    cylinder,
                    angle_deg10: schedule.soi_deg10,
                })
                .is_err()
            {
                out.diagnostic = DiagnosticCode::CalibrationInvalid;
                return out;
            }
            if out
                .events
                .push(crate::SemanticEvent {
                    kind: EventKind::InjectionClose,
                    cylinder,
                    angle_deg10: schedule.eoi_deg10,
                })
                .is_err()
            {
                out.diagnostic = DiagnosticCode::CalibrationInvalid;
                return out;
            }
        }
        if spark_events_enabled {
            if out
                .events
                .push(crate::SemanticEvent {
                    kind: EventKind::CoilChargeStart,
                    cylinder,
                    angle_deg10: schedule.dwell_start_deg10,
                })
                .is_err()
            {
                out.diagnostic = DiagnosticCode::CalibrationInvalid;
                return out;
            }
            if out
                .events
                .push(crate::SemanticEvent {
                    kind: EventKind::CoilFire,
                    cylinder,
                    angle_deg10: schedule.spark_deg10,
                })
                .is_err()
            {
                out.diagnostic = DiagnosticCode::CalibrationInvalid;
                return out;
            }
        }
        cyl += 1;
    }

    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{
        Axis16, Calibration, CylinderArrayU16, FuelModel, InjectionAngleMode, PwMaxPolicy,
        SyncState, TrimPolicy,
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

    fn table_u16(value: u16) -> Table2D16<u16> {
        Table2D16 {
            rpm_axis: axis(&[100, 200]),
            load_axis: axis(&[10, 20]),
            values: {
                let mut values = [[0u16; 16]; 16];
                values[0][0] = value;
                values[0][1] = value;
                values[1][0] = value;
                values[1][1] = value;
                values
            },
        }
    }

    fn table_u32(value: u32) -> Table2D16<u32> {
        Table2D16 {
            rpm_axis: axis(&[100, 200]),
            load_axis: axis(&[10, 20]),
            values: {
                let mut values = [[0u32; 16]; 16];
                values[0][0] = value;
                values[0][1] = value;
                values[1][0] = value;
                values[1][1] = value;
                values
            },
        }
    }

    fn table_i16(value: i16) -> Table2D16<i16> {
        Table2D16 {
            rpm_axis: axis(&[100, 200]),
            load_axis: axis(&[10, 20]),
            values: {
                let mut values = [[0i16; 16]; 16];
                values[0][0] = value;
                values[0][1] = value;
                values[1][0] = value;
                values[1][1] = value;
                values
            },
        }
    }

    fn calibration() -> ValidatedCalibration {
        ValidatedCalibration(Calibration {
            fuel_model: FuelModel::SpeedDensityRequiredFuel,
            ve_table: table_u16(8000),
            afr_target_table: table_u16(1470),
            spark_advance_table_deg10: table_i16(150),
            dwell_table_us: table_u32(2500),
            injection_target_table_deg10: table_u16(360),
            required_fuel_us: 3000,
            pref_kpa10: 1000,
            stoich_afr_x100: 1470,
            trim_policy: TrimPolicy::Identity,
            pw_max_policy: PwMaxPolicy::Fixed,
            pw_max_us: 25000,
            injection_angle_mode: InjectionAngleMode::EndOfInjection,
            cylinder_phase_deg10: CylinderArrayU16 {
                count: 4,
                values: [0, 1800, 3600, 5400, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            },
            ..Calibration::default()
        })
    }

    fn input() -> InputSnapshot {
        InputSnapshot {
            rpm: crate::Rpm(1000),
            load_kpa10: crate::Kpa10(100),
            tps_x100: 0,
            sync: SyncState::Synced,
            mode: crate::EngineMode::Running,
            ..InputSnapshot::default()
        }
    }

    #[test]
    fn computes_cylinder_angles_and_events_in_order() {
        let cal = calibration();
        let input = input();
        let fuel = FuelOutput {
            pw_corr_us: PulseWidthUs(3200),
        };
        let schedule = schedule_all_cylinders(&cal, input, fuel);
        assert_eq!(schedule.injection_target_deg10, Degrees10(360));
        assert_eq!(schedule.spark_advance_deg10, SignedDegrees10(150));
        assert_eq!(schedule.dwell_us, PulseWidthUs(2500));
        assert_eq!(schedule.injection_duration_deg10, Degrees10(192));
        assert_eq!(schedule.dwell_duration_deg10, Degrees10(150));
        assert_eq!(schedule.events.len, 16);
        assert_eq!(schedule.soi_deg10.values[0], 6648);
        assert_eq!(schedule.eoi_deg10.values[0], 6840);
        assert_eq!(schedule.spark_deg10.values[0], 7050);
        assert_eq!(schedule.dwell_start_deg10.values[0], 6900);
        assert_eq!(schedule.events.events[0].kind, EventKind::InjectionOpen);
        assert_eq!(schedule.events.events[1].kind, EventKind::InjectionClose);
        assert_eq!(schedule.events.events[2].kind, EventKind::CoilChargeStart);
        assert_eq!(schedule.events.events[3].kind, EventKind::CoilFire);
    }

    #[test]
    fn suppresses_cut_events_and_updates_diagnostics() {
        let cal = calibration();
        let mut cut_input = input();
        cut_input.fuel_cut = true;
        let schedule = schedule_all_cylinders(
            &cal,
            cut_input,
            FuelOutput {
                pw_corr_us: PulseWidthUs(0),
            },
        );
        assert_eq!(schedule.diagnostic, DiagnosticCode::FuelCutActive);
        assert_eq!(schedule.events.len, 8);
        assert_eq!(schedule.events.events[0].kind, EventKind::CoilChargeStart);

        let mut spark_input = input();
        spark_input.spark_cut = true;
        let schedule = schedule_all_cylinders(
            &cal,
            spark_input,
            FuelOutput {
                pw_corr_us: PulseWidthUs(3200),
            },
        );
        assert_eq!(schedule.diagnostic, DiagnosticCode::SparkCutActive);
        assert_eq!(schedule.events.len, 8);
        assert_eq!(schedule.events.events[0].kind, EventKind::InjectionOpen);
    }

    #[test]
    fn unsynced_suppresses_all_events() {
        let cal = calibration();
        let mut input = input();
        input.sync = SyncState::Unsynced;
        let schedule = schedule_all_cylinders(
            &cal,
            input,
            FuelOutput {
                pw_corr_us: PulseWidthUs(3200),
            },
        );
        assert_eq!(schedule.diagnostic, DiagnosticCode::Unsynced);
        assert_eq!(schedule.events.len, 0);
        assert_eq!(schedule.soi_deg10.values[0], 6648);
        assert_eq!(schedule.spark_deg10.values[0], 7050);
    }

    #[test]
    fn off_and_shutdown_suppress_events_without_unsynced_diagnostic() {
        let cal = calibration();
        let mut input = input();
        input.sync = SyncState::Unsynced;

        input.mode = crate::EngineMode::Off;
        let schedule = schedule_all_cylinders(
            &cal,
            input,
            FuelOutput {
                pw_corr_us: PulseWidthUs(3200),
            },
        );
        assert_eq!(schedule.diagnostic, DiagnosticCode::None);
        assert_eq!(schedule.events.len, 0);
        assert_eq!(schedule.soi_deg10.values[0], 6648);

        input.mode = crate::EngineMode::Shutdown;
        let schedule = schedule_all_cylinders(
            &cal,
            input,
            FuelOutput {
                pw_corr_us: PulseWidthUs(3200),
            },
        );
        assert_eq!(schedule.diagnostic, DiagnosticCode::None);
        assert_eq!(schedule.events.len, 0);
    }

    #[test]
    fn decreasing_dwell_table_interpolates_without_unsigned_underflow() {
        let mut cal = calibration();
        cal.0.dwell_table_us = Table2D16 {
            rpm_axis: axis(&[100, 200]),
            load_axis: axis(&[10, 20]),
            values: {
                let mut values = [[0u32; 16]; 16];
                values[0][0] = 3000;
                values[0][1] = 2000;
                values[1][0] = 1000;
                values[1][1] = 500;
                values
            },
        };

        let mut input = input();
        input.rpm = Rpm(150);
        input.load_kpa10 = Kpa10(15);

        let dwell = compute_dwell_us(&cal, input);
        assert_eq!(dwell, PulseWidthUs(1625));
    }

    #[test]
    fn tiny_batch_defensive_path_sets_calibration_invalid() {
        let cal = calibration();
        let input = input();
        let fuel = FuelOutput {
            pw_corr_us: PulseWidthUs(3200),
        };
        let mut tiny = tiny_event_batch();
        tiny.len = crate::types::MAX_EVENTS_PER_STEP as u8;

        let schedule =
            schedule_all_cylinders_with_seeded_events(&cal, input, fuel, tiny, SignedDegrees10(0));
        assert_eq!(schedule.diagnostic, DiagnosticCode::CalibrationInvalid);
        assert_eq!(schedule.events.len, crate::types::MAX_EVENTS_PER_STEP as u8);
    }
}
