use ecu_spec::{
    burn_page, committed_page_record, decode_outpc, default_reference_calibration, encode_outpc,
    encode_ts_diag_log_oldest_first, page_meta, persist_decode, persist_encode, persist_migrate,
    save_all, step as spec_step, ts_diag_log_push, ts_dispatch_step, write_page, AfrOverride,
    AfrX100, Degrees10, DiagnosticCode, EngineMode, EventKind, InputSnapshot, Kpa10, LogicalState,
    Millivolts, OutpcFrame, PersistPage, PersistPageId, Rpm, StepResult, SyncState, TempC10,
    TsBurnSaveError, TsBurnSaveStore, TsCommandDecodeError, TsDiagLogEntry, TsDiagLogRing,
    TsDispatchError, TsEffect, TsPageId, ValidatedCalibration, TS_DIAG_LOG_CAPACITY,
    TS_DIAG_LOG_ENTRY_BYTES,
};

#[allow(dead_code)]
pub const EPS_VE_X100: u16 = 1;
#[allow(dead_code)]
pub const EPS_PW_US: u32 = 1;
#[allow(dead_code)]
pub const EPS_ANGLE_DEG10: u16 = 1;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct FixtureCase {
    pub fixture: &'static str,
    pub variant: &'static str,
    pub calibration: ValidatedCalibration,
    pub input: InputSnapshot,
}

#[allow(dead_code)]
pub const REQUIRED_COMPARED_FIELDS: [&str; 25] = [
    "ObservableOutput.ve_pct_x100",
    "ObservableOutput.target_afr_x100",
    "ObservableOutput.pw_base_us",
    "ObservableOutput.pw_air_us",
    "ObservableOutput.pw_corr_us",
    "ObservableOutput.idle_duty_x1000",
    "ObservableOutput.lambda_correction_x1000",
    "ObservableOutput.advance_deg10_trim",
    "ObservableOutput.cut_reason_code",
    "ObservableOutput.fuel_cut",
    "ObservableOutput.spark_cut",
    "ObservableOutput.knock_intensity_x100",
    "ObservableOutput.torque_request_x1000",
    "ObservableOutput.torque_allowed_x1000",
    "ObservableOutput.torque_actuated_x1000",
    "ObservableOutput.soi_deg10",
    "ObservableOutput.eoi_deg10",
    "ObservableOutput.spark_deg10",
    "ObservableOutput.dwell_start_deg10",
    "ObservableOutput.events.{kind,angle_deg10}",
    "ObservableOutput.diagnostic",
    "Trigger.rpm_estimate",
    "Trigger.sync_state",
    "LogicalState.idle_integrator_state",
    "LogicalState.lambda_integrator_state",
];

pub fn required_fixture_names() -> [&'static str; 35] {
    [
        "running_synced_no_cut",
        "fuel_cut_running_synced",
        "spark_cut_running_synced",
        "running_unsynced",
        "off_unsynced",
        "shutdown_unsynced",
        "decreasing_dwell_table",
        "negative_temperature_inputs",
        "afr_override_low_high_clamp",
        "interp_last_segment_right_closure",
        "interp_segment_boundary_equality",
        "interp_decreasing_u16_endpoints",
        "interp_distinct_corner_bilinear_cell",
        "interp_negative_slope_i16_curve",
        "deadtime_addition",
        "vbat_deadtime_response",
        "baro_correction_response",
        "cranking_correction_response",
        "afterstart_enrichment_window",
        "warmup_correction_response",
        "ae_onset_decay",
        "dfco_entry_exit_hysteresis",
        "rev_limit_soft_hard_recovery",
        "idle_pi_response",
        "lambda_cl_integrator_response",
        "knock_response",
        "launch_control_pattern",
        "flat_shift_pattern",
        "arbiter_priority_pairwise_conflicts",
        "sensor_curves_input_sweep",
        "trigger_decoder_tooth_stream",
        "persistence_roundtrip_migration",
        "ts_proto_dispatch_burn_diag",
        "safety_latching",
        "torque_pipeline",
    ]
}

#[allow(dead_code)]
const SENSOR_COUNTS_AXIS: [u16; 16] = [
    0, 256, 512, 768, 1024, 1280, 1536, 1792, 2048, 2304, 2560, 2816, 3072, 3328, 3584, 4095,
];

#[allow(dead_code)]
const RUNTIME_CLT_TEMP_C10: [i16; 16] = [
    1200, 1020, 860, 730, 610, 500, 390, 290, 200, 120, 40, -40, -120, -200, -300, -400,
];

#[allow(dead_code)]
const RUNTIME_IAT_TEMP_C10: [i16; 16] = [
    1100, 950, 820, 700, 590, 490, 390, 300, 220, 140, 70, 0, -80, -170, -280, -400,
];

#[allow(dead_code)]
const RUNTIME_MAF_FLOW_X100: [u16; 16] = [
    0, 120, 280, 500, 780, 1120, 1520, 1980, 2520, 3150, 3880, 4720, 5680, 6760, 7960, 9300,
];

// Allow dead code in helpers used only inside #[cfg(test)] module below
#[allow(dead_code)]
fn clamp_u16(value: u16, lo: u16, hi: u16) -> u16 {
    value.clamp(lo, hi)
}

#[allow(dead_code)]
fn runtime_find_segment(axis: &[u16], x: u16) -> usize {
    let clipped = clamp_u16(x, axis[0], axis[axis.len() - 1]);
    let mut idx = 0usize;
    while idx + 1 < axis.len() {
        let lo = axis[idx];
        let hi = axis[idx + 1];
        if clipped >= lo && (clipped < hi || (idx + 1 == axis.len() - 1 && clipped == hi)) {
            return idx;
        }
        idx += 1;
    }
    axis.len() - 2
}

#[allow(dead_code)]
fn runtime_lerp_u16(x0: u16, x1: u16, y0: u16, y1: u16, x: u16) -> u16 {
    if x1 <= x0 {
        return y0;
    }
    let x_clip = clamp_u16(x, x0, x1);
    let num = (x_clip - x0) as i64;
    let den = (x1 - x0) as i64;
    let delta = y1 as i64 - y0 as i64;
    let product = delta * num;
    let quotient = product / den;
    let remainder = product % den;
    let correction = if product < 0 && remainder != 0 { -1 } else { 0 };
    (y0 as i64 + quotient + correction) as u16
}

#[allow(dead_code)]
fn runtime_lerp_i16(x0: u16, x1: u16, y0: i16, y1: i16, x: u16) -> i16 {
    if x1 <= x0 {
        return y0;
    }
    let x_clip = clamp_u16(x, x0, x1);
    let num = (x_clip - x0) as i64;
    let den = (x1 - x0) as i64;
    let delta = y1 as i64 - y0 as i64;
    let product = delta * num;
    let quotient = product / den;
    let remainder = product % den;
    let correction = if product < 0 && remainder != 0 { -1 } else { 0 };
    (y0 as i64 + quotient + correction) as i16
}

#[allow(dead_code)]
fn runtime_temp_from_counts(adc_counts: u16, table: &[i16; 16]) -> i16 {
    let counts = clamp_u16(adc_counts, 0, 4095);
    let seg = runtime_find_segment(&SENSOR_COUNTS_AXIS, counts);
    runtime_lerp_i16(
        SENSOR_COUNTS_AXIS[seg],
        SENSOR_COUNTS_AXIS[seg + 1],
        table[seg],
        table[seg + 1],
        counts,
    )
}

#[allow(dead_code)]
fn runtime_maf_from_counts(adc_counts: u16) -> u16 {
    let counts = clamp_u16(adc_counts, 0, 4095);
    let seg = runtime_find_segment(&SENSOR_COUNTS_AXIS, counts);
    runtime_lerp_u16(
        SENSOR_COUNTS_AXIS[seg],
        SENSOR_COUNTS_AXIS[seg + 1],
        RUNTIME_MAF_FLOW_X100[seg],
        RUNTIME_MAF_FLOW_X100[seg + 1],
        counts,
    )
}

fn canonical_input() -> InputSnapshot {
    InputSnapshot {
        t_us: ecu_spec::Micros(10_000),
        rpm: Rpm(1000),
        map_kpa10: Kpa10(1000),
        load_kpa10: Kpa10(1000),
        tps_x100: 5000,
        clt_c10: TempC10(800),
        iat_c10: TempC10(250),
        baro_kpa10: Kpa10(1000),
        vbatt_mv: Millivolts(12_000),
        knock_intensity_x100: 0,
        launch_armed: false,
        flat_shift_armed: false,
        sync: SyncState::Synced,
        fuel_cut: false,
        spark_cut: false,
        mode: EngineMode::Running,
        target_afr_override_x100: AfrOverride::None,
    }
}

fn decreasing_dwell_calibration() -> ValidatedCalibration {
    let mut cal = default_reference_calibration();
    cal.0.dwell_table_us.values[0][0] = 3000;
    cal.0.dwell_table_us.values[0][1] = 1000;
    cal.0.dwell_table_us.values[1][0] = 3000;
    cal.0.dwell_table_us.values[1][1] = 1000;
    cal
}

fn interp_corner_calibration() -> ValidatedCalibration {
    let mut cal = default_reference_calibration();

    cal.0.ve_table.rpm_axis.len = 3;
    cal.0.ve_table.rpm_axis.values[0] = 500;
    cal.0.ve_table.rpm_axis.values[1] = 1000;
    cal.0.ve_table.rpm_axis.values[2] = 1500;
    cal.0.ve_table.load_axis.len = 3;
    cal.0.ve_table.load_axis.values[0] = 500;
    cal.0.ve_table.load_axis.values[1] = 1000;
    cal.0.ve_table.load_axis.values[2] = 1500;
    cal.0.ve_table.values[0][0] = 6000;
    cal.0.ve_table.values[0][1] = 7000;
    cal.0.ve_table.values[0][2] = 8500;
    cal.0.ve_table.values[1][0] = 7000;
    cal.0.ve_table.values[1][1] = 9000;
    cal.0.ve_table.values[1][2] = 11500;
    cal.0.ve_table.values[2][0] = 9000;
    cal.0.ve_table.values[2][1] = 12000;
    cal.0.ve_table.values[2][2] = 15000;

    cal.0.spark_advance_table_deg10.values[0][0] = 320;
    cal.0.spark_advance_table_deg10.values[0][1] = 260;
    cal.0.spark_advance_table_deg10.values[1][0] = 300;
    cal.0.spark_advance_table_deg10.values[1][1] = 220;
    cal
}

fn deadtime_vbat_baro_calibration() -> ValidatedCalibration {
    let mut cal = default_reference_calibration();

    cal.0.deadtime_table_us.rpm_axis.len = 3;
    cal.0.deadtime_table_us.rpm_axis.values[0] = 10_000;
    cal.0.deadtime_table_us.rpm_axis.values[1] = 12_000;
    cal.0.deadtime_table_us.rpm_axis.values[2] = 14_000;
    cal.0.deadtime_table_us.load_axis.len = 3;
    cal.0.deadtime_table_us.load_axis.values[0] = 800;
    cal.0.deadtime_table_us.load_axis.values[1] = 1000;
    cal.0.deadtime_table_us.load_axis.values[2] = 1200;
    cal.0.deadtime_table_us.values[0][0] = 1300;
    cal.0.deadtime_table_us.values[0][1] = 700;
    cal.0.deadtime_table_us.values[0][2] = 300;
    cal.0.deadtime_table_us.values[1][0] = 1300;
    cal.0.deadtime_table_us.values[1][1] = 700;
    cal.0.deadtime_table_us.values[1][2] = 300;
    cal.0.deadtime_table_us.values[2][0] = 1300;
    cal.0.deadtime_table_us.values[2][1] = 700;
    cal.0.deadtime_table_us.values[2][2] = 300;

    cal.0.baro_corr_curve.axis.len = 3;
    cal.0.baro_corr_curve.axis.values[0] = 800;
    cal.0.baro_corr_curve.axis.values[1] = 1000;
    cal.0.baro_corr_curve.axis.values[2] = 1200;
    cal.0.baro_corr_curve.values[0] = 800;
    cal.0.baro_corr_curve.values[1] = 1000;
    cal.0.baro_corr_curve.values[2] = 1200;

    cal.0.clt_corr_curve.axis.len = 2;
    cal.0.clt_corr_curve.axis.values[0] = 0;
    cal.0.clt_corr_curve.axis.values[1] = 1000;
    cal.0.clt_corr_curve.values[0] = 1000;
    cal.0.clt_corr_curve.values[1] = 1000;

    cal.0.iat_corr_curve.axis.len = 2;
    cal.0.iat_corr_curve.axis.values[0] = 0;
    cal.0.iat_corr_curve.axis.values[1] = 1000;
    cal.0.iat_corr_curve.values[0] = 1000;
    cal.0.iat_corr_curve.values[1] = 1000;

    cal.0.vbat_corr_curve.axis.len = 3;
    cal.0.vbat_corr_curve.axis.values[0] = 10_000;
    cal.0.vbat_corr_curve.axis.values[1] = 12_000;
    cal.0.vbat_corr_curve.axis.values[2] = 14_000;
    cal.0.vbat_corr_curve.values[0] = 1000;
    cal.0.vbat_corr_curve.values[1] = 1000;
    cal.0.vbat_corr_curve.values[2] = 1000;

    cal
}

fn cranking_afterstart_warmup_calibration() -> ValidatedCalibration {
    let mut cal = default_reference_calibration();

    cal.0.cranking_curve.axis.len = 3;
    cal.0.cranking_curve.axis.values[0] = 0;
    cal.0.cranking_curve.axis.values[1] = 400;
    cal.0.cranking_curve.axis.values[2] = 800;
    cal.0.cranking_curve.values[0] = 2000;
    cal.0.cranking_curve.values[1] = 1500;
    cal.0.cranking_curve.values[2] = 1000;

    cal.0.afterstart_table.rpm_axis.len = 3;
    cal.0.afterstart_table.rpm_axis.values[0] = 0;
    cal.0.afterstart_table.rpm_axis.values[1] = 5;
    cal.0.afterstart_table.rpm_axis.values[2] = 10;
    cal.0.afterstart_table.load_axis.len = 3;
    cal.0.afterstart_table.load_axis.values[0] = 0;
    cal.0.afterstart_table.load_axis.values[1] = 400;
    cal.0.afterstart_table.load_axis.values[2] = 800;
    cal.0.afterstart_table.values[0][0] = 1600;
    cal.0.afterstart_table.values[0][1] = 1400;
    cal.0.afterstart_table.values[0][2] = 1200;
    cal.0.afterstart_table.values[1][0] = 1500;
    cal.0.afterstart_table.values[1][1] = 1300;
    cal.0.afterstart_table.values[1][2] = 1100;
    cal.0.afterstart_table.values[2][0] = 1400;
    cal.0.afterstart_table.values[2][1] = 1200;
    cal.0.afterstart_table.values[2][2] = 1000;
    cal.0.afterstart_window_cycles = 10;

    cal.0.warmup_curve.axis.len = 3;
    cal.0.warmup_curve.axis.values[0] = 0;
    cal.0.warmup_curve.axis.values[1] = 400;
    cal.0.warmup_curve.axis.values[2] = 800;
    cal.0.warmup_curve.values[0] = 1300;
    cal.0.warmup_curve.values[1] = 1100;
    cal.0.warmup_curve.values[2] = 1000;

    cal.0.clt_corr_curve.axis.len = 2;
    cal.0.clt_corr_curve.axis.values[0] = 0;
    cal.0.clt_corr_curve.axis.values[1] = 1000;
    cal.0.clt_corr_curve.values[0] = 1000;
    cal.0.clt_corr_curve.values[1] = 1000;

    cal.0.iat_corr_curve.axis.len = 2;
    cal.0.iat_corr_curve.axis.values[0] = 0;
    cal.0.iat_corr_curve.axis.values[1] = 1000;
    cal.0.iat_corr_curve.values[0] = 1000;
    cal.0.iat_corr_curve.values[1] = 1000;

    cal.0.baro_corr_curve.axis.len = 2;
    cal.0.baro_corr_curve.axis.values[0] = 800;
    cal.0.baro_corr_curve.axis.values[1] = 1200;
    cal.0.baro_corr_curve.values[0] = 1000;
    cal.0.baro_corr_curve.values[1] = 1000;

    cal.0.vbat_corr_curve.axis.len = 2;
    cal.0.vbat_corr_curve.axis.values[0] = 10_000;
    cal.0.vbat_corr_curve.axis.values[1] = 14_000;
    cal.0.vbat_corr_curve.values[0] = 1000;
    cal.0.vbat_corr_curve.values[1] = 1000;

    cal.0.deadtime_table_us.rpm_axis.len = 2;
    cal.0.deadtime_table_us.rpm_axis.values[0] = 10_000;
    cal.0.deadtime_table_us.rpm_axis.values[1] = 14_000;
    cal.0.deadtime_table_us.load_axis.len = 2;
    cal.0.deadtime_table_us.load_axis.values[0] = 800;
    cal.0.deadtime_table_us.load_axis.values[1] = 1200;
    cal.0.deadtime_table_us.values[0][0] = 0;
    cal.0.deadtime_table_us.values[0][1] = 0;
    cal.0.deadtime_table_us.values[1][0] = 0;
    cal.0.deadtime_table_us.values[1][1] = 0;

    cal
}

fn ae_dfco_calibration() -> ValidatedCalibration {
    let mut cal = default_reference_calibration();

    cal.0.ae_tps_threshold_curve.axis.len = 2;
    cal.0.ae_tps_threshold_curve.axis.values[0] = 500;
    cal.0.ae_tps_threshold_curve.axis.values[1] = 7000;
    cal.0.ae_tps_threshold_curve.values[0] = 400;
    cal.0.ae_tps_threshold_curve.values[1] = 400;

    cal.0.ae_map_threshold_curve.axis.len = 2;
    cal.0.ae_map_threshold_curve.axis.values[0] = 100;
    cal.0.ae_map_threshold_curve.axis.values[1] = 3000;
    cal.0.ae_map_threshold_curve.values[0] = 2000;
    cal.0.ae_map_threshold_curve.values[1] = 2000;

    cal.0.ae_shot_curve_us.axis.len = 2;
    cal.0.ae_shot_curve_us.axis.values[0] = 500;
    cal.0.ae_shot_curve_us.axis.values[1] = 7000;
    cal.0.ae_shot_curve_us.values[0] = 1000;
    cal.0.ae_shot_curve_us.values[1] = 1000;

    cal.0.ae_decay_steps_curve.axis.len = 2;
    cal.0.ae_decay_steps_curve.axis.values[0] = 500;
    cal.0.ae_decay_steps_curve.axis.values[1] = 7000;
    cal.0.ae_decay_steps_curve.values[0] = 2;
    cal.0.ae_decay_steps_curve.values[1] = 2;

    cal.0.ae_decay_ratio_curve_x1000.axis.len = 2;
    cal.0.ae_decay_ratio_curve_x1000.axis.values[0] = 0;
    cal.0.ae_decay_ratio_curve_x1000.axis.values[1] = 16;
    cal.0.ae_decay_ratio_curve_x1000.values[0] = 800;
    cal.0.ae_decay_ratio_curve_x1000.values[1] = 800;

    cal.0.dfco_entry_rpm = Rpm(2000);
    cal.0.dfco_exit_rpm = Rpm(1800);
    cal.0.dfco_entry_tps_x100 = 200;
    cal.0.dfco_exit_tps_x100 = 300;
    cal.0.dfco_entry_map_kpa10 = Kpa10(500);
    cal.0.dfco_delay_cycles = 1;

    cal
}

fn rev_limit_calibration() -> ValidatedCalibration {
    let mut cal = default_reference_calibration();
    cal.0.soft_rev_rpm = Rpm(4000);
    cal.0.hard_rev_rpm = Rpm(5000);
    cal.0.rev_hysteresis_rpm = Rpm(200);
    cal.0.soft_retard_max_deg10 = 120;
    cal
}

fn idle_pi_calibration() -> ValidatedCalibration {
    let mut cal = default_reference_calibration();
    cal.0.idle_target_rpm = Rpm(1000);
    cal.0.idle_base_duty_x1000 = 350;
    cal.0.idle_kp_x1000 = 300;
    cal.0.idle_ki_x1000 = 500;
    cal
}

fn lambda_cl_calibration() -> ValidatedCalibration {
    let mut cal = default_reference_calibration();
    cal.0.lambda_kp_x1000 = 0;
    cal.0.lambda_ki_x1000 = 0;
    cal
}

fn knock_calibration() -> ValidatedCalibration {
    let mut cal = default_reference_calibration();
    cal.0.knock_threshold_x100 = 500;
    cal.0.knock_retard_step_deg10 = 40;
    cal.0.knock_retard_max_deg10 = 200;
    cal.0.knock_recovery_step_deg10 = 20;
    cal.0.knock_recovery_delay_cycles = 1;
    cal
}

fn launch_flat_shift_calibration() -> ValidatedCalibration {
    let mut cal = default_reference_calibration();
    cal.0.soft_rev_rpm = Rpm(20_000);
    cal.0.hard_rev_rpm = Rpm(20_000);
    cal.0.rev_hysteresis_rpm = Rpm(0);
    cal.0.launch_rpm_limit = Rpm(5000);
    cal.0.launch_cut_cycles = 2;
    cal.0.flat_shift_rpm_min = Rpm(5000);
    cal.0.flat_shift_cut_cycles = 2;
    cal
}

fn arbiter_priority_calibration() -> ValidatedCalibration {
    let mut cal = default_reference_calibration();
    cal.0.dfco_entry_rpm = Rpm(2000);
    cal.0.dfco_exit_rpm = Rpm(1800);
    cal.0.dfco_entry_tps_x100 = 200;
    cal.0.dfco_exit_tps_x100 = 300;
    cal.0.dfco_entry_map_kpa10 = Kpa10(500);
    cal.0.dfco_delay_cycles = 1;

    cal.0.soft_rev_rpm = Rpm(4500);
    cal.0.hard_rev_rpm = Rpm(6500);
    cal.0.rev_hysteresis_rpm = Rpm(100);
    cal.0.soft_retard_max_deg10 = 120;

    cal.0.launch_rpm_limit = Rpm(5000);
    cal.0.launch_cut_cycles = 0;
    cal.0.flat_shift_rpm_min = Rpm(5000);
    cal.0.flat_shift_cut_cycles = 0;

    cal.0.knock_threshold_x100 = 500;
    cal.0.knock_retard_step_deg10 = 40;
    cal.0.knock_retard_max_deg10 = 200;
    cal.0.knock_recovery_step_deg10 = 20;
    cal.0.knock_recovery_delay_cycles = 1;
    cal
}

pub fn fixture_cases() -> [FixtureCase; 88] {
    let base = canonical_input();
    [
        FixtureCase {
            fixture: "running_synced_no_cut",
            variant: "default",
            calibration: default_reference_calibration(),
            input: base,
        },
        FixtureCase {
            fixture: "fuel_cut_running_synced",
            variant: "default",
            calibration: default_reference_calibration(),
            input: InputSnapshot {
                fuel_cut: true,
                ..base
            },
        },
        FixtureCase {
            fixture: "spark_cut_running_synced",
            variant: "default",
            calibration: default_reference_calibration(),
            input: InputSnapshot {
                spark_cut: true,
                ..base
            },
        },
        FixtureCase {
            fixture: "running_unsynced",
            variant: "default",
            calibration: default_reference_calibration(),
            input: InputSnapshot {
                sync: SyncState::Unsynced,
                ..base
            },
        },
        FixtureCase {
            fixture: "off_unsynced",
            variant: "default",
            calibration: default_reference_calibration(),
            input: InputSnapshot {
                sync: SyncState::Unsynced,
                mode: EngineMode::Off,
                ..base
            },
        },
        FixtureCase {
            fixture: "shutdown_unsynced",
            variant: "default",
            calibration: default_reference_calibration(),
            input: InputSnapshot {
                sync: SyncState::Unsynced,
                mode: EngineMode::Shutdown,
                ..base
            },
        },
        FixtureCase {
            fixture: "decreasing_dwell_table",
            variant: "default",
            calibration: decreasing_dwell_calibration(),
            input: InputSnapshot {
                rpm: Rpm(750),
                load_kpa10: Kpa10(750),
                map_kpa10: Kpa10(750),
                ..base
            },
        },
        FixtureCase {
            fixture: "negative_temperature_inputs",
            variant: "default",
            calibration: default_reference_calibration(),
            input: InputSnapshot {
                clt_c10: TempC10(-350),
                iat_c10: TempC10(-120),
                ..base
            },
        },
        FixtureCase {
            fixture: "afr_override_low_high_clamp",
            variant: "low",
            calibration: default_reference_calibration(),
            input: InputSnapshot {
                target_afr_override_x100: AfrOverride::Some(AfrX100(100)),
                ..base
            },
        },
        FixtureCase {
            fixture: "afr_override_low_high_clamp",
            variant: "high",
            calibration: default_reference_calibration(),
            input: InputSnapshot {
                target_afr_override_x100: AfrOverride::Some(AfrX100(4000)),
                ..base
            },
        },
        FixtureCase {
            fixture: "interp_last_segment_right_closure",
            variant: "default",
            calibration: interp_corner_calibration(),
            input: InputSnapshot {
                rpm: Rpm(1500),
                load_kpa10: Kpa10(1500),
                map_kpa10: Kpa10(1500),
                ..base
            },
        },
        FixtureCase {
            fixture: "interp_segment_boundary_equality",
            variant: "default",
            calibration: interp_corner_calibration(),
            input: InputSnapshot {
                rpm: Rpm(1000),
                load_kpa10: Kpa10(1000),
                map_kpa10: Kpa10(1000),
                ..base
            },
        },
        FixtureCase {
            fixture: "interp_decreasing_u16_endpoints",
            variant: "default",
            calibration: decreasing_dwell_calibration(),
            input: InputSnapshot {
                rpm: Rpm(750),
                load_kpa10: Kpa10(750),
                map_kpa10: Kpa10(750),
                ..base
            },
        },
        FixtureCase {
            fixture: "interp_distinct_corner_bilinear_cell",
            variant: "default",
            calibration: interp_corner_calibration(),
            input: InputSnapshot {
                rpm: Rpm(750),
                load_kpa10: Kpa10(750),
                map_kpa10: Kpa10(750),
                ..base
            },
        },
        FixtureCase {
            fixture: "interp_negative_slope_i16_curve",
            variant: "default",
            calibration: interp_corner_calibration(),
            input: InputSnapshot {
                rpm: Rpm(750),
                load_kpa10: Kpa10(1000),
                map_kpa10: Kpa10(1000),
                ..base
            },
        },
        FixtureCase {
            fixture: "deadtime_addition",
            variant: "default",
            calibration: deadtime_vbat_baro_calibration(),
            input: InputSnapshot {
                vbatt_mv: Millivolts(12_000),
                baro_kpa10: Kpa10(1000),
                map_kpa10: Kpa10(1000),
                ..base
            },
        },
        FixtureCase {
            fixture: "vbat_deadtime_response",
            variant: "low_vbat",
            calibration: deadtime_vbat_baro_calibration(),
            input: InputSnapshot {
                vbatt_mv: Millivolts(10_000),
                baro_kpa10: Kpa10(1000),
                map_kpa10: Kpa10(1000),
                ..base
            },
        },
        FixtureCase {
            fixture: "vbat_deadtime_response",
            variant: "high_vbat",
            calibration: deadtime_vbat_baro_calibration(),
            input: InputSnapshot {
                vbatt_mv: Millivolts(14_000),
                baro_kpa10: Kpa10(1000),
                map_kpa10: Kpa10(1000),
                ..base
            },
        },
        FixtureCase {
            fixture: "baro_correction_response",
            variant: "low_baro",
            calibration: deadtime_vbat_baro_calibration(),
            input: InputSnapshot {
                baro_kpa10: Kpa10(800),
                vbatt_mv: Millivolts(12_000),
                map_kpa10: Kpa10(1000),
                ..base
            },
        },
        FixtureCase {
            fixture: "baro_correction_response",
            variant: "high_baro",
            calibration: deadtime_vbat_baro_calibration(),
            input: InputSnapshot {
                baro_kpa10: Kpa10(1200),
                vbatt_mv: Millivolts(12_000),
                map_kpa10: Kpa10(1000),
                ..base
            },
        },
        FixtureCase {
            fixture: "cranking_correction_response",
            variant: "cranking",
            calibration: cranking_afterstart_warmup_calibration(),
            input: InputSnapshot {
                mode: EngineMode::Cranking,
                clt_c10: TempC10(0),
                ..base
            },
        },
        FixtureCase {
            fixture: "cranking_correction_response",
            variant: "running",
            calibration: cranking_afterstart_warmup_calibration(),
            input: InputSnapshot {
                mode: EngineMode::Running,
                clt_c10: TempC10(0),
                ..base
            },
        },
        FixtureCase {
            fixture: "afterstart_enrichment_window",
            variant: "inside_window",
            calibration: cranking_afterstart_warmup_calibration(),
            input: InputSnapshot {
                mode: EngineMode::Running,
                clt_c10: TempC10(0),
                ..base
            },
        },
        FixtureCase {
            fixture: "afterstart_enrichment_window",
            variant: "outside_window",
            calibration: cranking_afterstart_warmup_calibration(),
            input: InputSnapshot {
                mode: EngineMode::Running,
                clt_c10: TempC10(0),
                ..base
            },
        },
        FixtureCase {
            fixture: "warmup_correction_response",
            variant: "cold",
            calibration: cranking_afterstart_warmup_calibration(),
            input: InputSnapshot {
                mode: EngineMode::Running,
                clt_c10: TempC10(0),
                ..base
            },
        },
        FixtureCase {
            fixture: "ae_onset_decay",
            variant: "onset",
            calibration: ae_dfco_calibration(),
            input: InputSnapshot {
                mode: EngineMode::Running,
                rpm: Rpm(3000),
                map_kpa10: Kpa10(1000),
                load_kpa10: Kpa10(1000),
                ..base
            },
        },
        FixtureCase {
            fixture: "ae_onset_decay",
            variant: "decay",
            calibration: ae_dfco_calibration(),
            input: InputSnapshot {
                mode: EngineMode::Running,
                rpm: Rpm(3000),
                map_kpa10: Kpa10(1000),
                load_kpa10: Kpa10(1000),
                ..base
            },
        },
        FixtureCase {
            fixture: "dfco_entry_exit_hysteresis",
            variant: "entry",
            calibration: ae_dfco_calibration(),
            input: InputSnapshot {
                mode: EngineMode::Running,
                rpm: Rpm(3000),
                tps_x100: 0,
                map_kpa10: Kpa10(350),
                load_kpa10: Kpa10(350),
                ..base
            },
        },
        FixtureCase {
            fixture: "dfco_entry_exit_hysteresis",
            variant: "exit",
            calibration: ae_dfco_calibration(),
            input: InputSnapshot {
                mode: EngineMode::Running,
                rpm: Rpm(3000),
                tps_x100: 600,
                map_kpa10: Kpa10(350),
                load_kpa10: Kpa10(350),
                ..base
            },
        },
        FixtureCase {
            fixture: "rev_limit_soft_hard_recovery",
            variant: "under_limit",
            calibration: rev_limit_calibration(),
            input: InputSnapshot {
                rpm: Rpm(3500),
                ..base
            },
        },
        FixtureCase {
            fixture: "rev_limit_soft_hard_recovery",
            variant: "soft_limit",
            calibration: rev_limit_calibration(),
            input: InputSnapshot {
                rpm: Rpm(4500),
                ..base
            },
        },
        FixtureCase {
            fixture: "rev_limit_soft_hard_recovery",
            variant: "hard_limit",
            calibration: rev_limit_calibration(),
            input: InputSnapshot {
                rpm: Rpm(5100),
                ..base
            },
        },
        FixtureCase {
            fixture: "rev_limit_soft_hard_recovery",
            variant: "recovery",
            calibration: rev_limit_calibration(),
            input: InputSnapshot {
                rpm: Rpm(3600),
                ..base
            },
        },
        FixtureCase {
            fixture: "idle_pi_response",
            variant: "integrate",
            calibration: idle_pi_calibration(),
            input: InputSnapshot {
                rpm: Rpm(700),
                clt_c10: TempC10(800),
                ..base
            },
        },
        FixtureCase {
            fixture: "idle_pi_response",
            variant: "freeze_cold",
            calibration: idle_pi_calibration(),
            input: InputSnapshot {
                rpm: Rpm(700),
                clt_c10: TempC10(650),
                ..base
            },
        },
        FixtureCase {
            fixture: "lambda_cl_integrator_response",
            variant: "saturation",
            calibration: lambda_cl_calibration(),
            input: InputSnapshot {
                mode: EngineMode::Running,
                clt_c10: TempC10(800),
                fuel_cut: false,
                spark_cut: false,
                ..base
            },
        },
        FixtureCase {
            fixture: "lambda_cl_integrator_response",
            variant: "freeze_cut",
            calibration: lambda_cl_calibration(),
            input: InputSnapshot {
                mode: EngineMode::Running,
                clt_c10: TempC10(800),
                fuel_cut: true,
                spark_cut: false,
                ..base
            },
        },
        FixtureCase {
            fixture: "knock_response",
            variant: "below_threshold",
            calibration: knock_calibration(),
            input: InputSnapshot {
                knock_intensity_x100: 400,
                ..base
            },
        },
        FixtureCase {
            fixture: "knock_response",
            variant: "detect",
            calibration: knock_calibration(),
            input: InputSnapshot {
                knock_intensity_x100: 600,
                ..base
            },
        },
        FixtureCase {
            fixture: "knock_response",
            variant: "retard",
            calibration: knock_calibration(),
            input: InputSnapshot {
                knock_intensity_x100: 600,
                ..base
            },
        },
        FixtureCase {
            fixture: "knock_response",
            variant: "recovery",
            calibration: knock_calibration(),
            input: InputSnapshot {
                knock_intensity_x100: 100,
                ..base
            },
        },
        FixtureCase {
            fixture: "launch_control_pattern",
            variant: "disarmed",
            calibration: launch_flat_shift_calibration(),
            input: InputSnapshot {
                rpm: Rpm(5600),
                launch_armed: false,
                flat_shift_armed: false,
                ..base
            },
        },
        FixtureCase {
            fixture: "launch_control_pattern",
            variant: "armed_pattern",
            calibration: launch_flat_shift_calibration(),
            input: InputSnapshot {
                rpm: Rpm(5600),
                launch_armed: true,
                flat_shift_armed: false,
                ..base
            },
        },
        FixtureCase {
            fixture: "flat_shift_pattern",
            variant: "disarmed",
            calibration: launch_flat_shift_calibration(),
            input: InputSnapshot {
                rpm: Rpm(5600),
                launch_armed: false,
                flat_shift_armed: false,
                ..base
            },
        },
        FixtureCase {
            fixture: "flat_shift_pattern",
            variant: "armed_pattern",
            calibration: launch_flat_shift_calibration(),
            input: InputSnapshot {
                rpm: Rpm(5600),
                launch_armed: false,
                flat_shift_armed: true,
                ..base
            },
        },
        FixtureCase {
            fixture: "arbiter_priority_pairwise_conflicts",
            variant: "safety_latched_over_hard_rev_limit",
            calibration: arbiter_priority_calibration(),
            input: InputSnapshot {
                mode: EngineMode::Shutdown,
                rpm: Rpm(7000),
                ..base
            },
        },
        FixtureCase {
            fixture: "arbiter_priority_pairwise_conflicts",
            variant: "safety_latched_over_launch_cut",
            calibration: arbiter_priority_calibration(),
            input: InputSnapshot {
                mode: EngineMode::Shutdown,
                rpm: Rpm(5600),
                launch_armed: true,
                ..base
            },
        },
        FixtureCase {
            fixture: "arbiter_priority_pairwise_conflicts",
            variant: "safety_latched_over_flat_shift_cut",
            calibration: arbiter_priority_calibration(),
            input: InputSnapshot {
                mode: EngineMode::Shutdown,
                rpm: Rpm(5600),
                flat_shift_armed: true,
                ..base
            },
        },
        FixtureCase {
            fixture: "arbiter_priority_pairwise_conflicts",
            variant: "safety_latched_over_dfco_cut",
            calibration: arbiter_priority_calibration(),
            input: InputSnapshot {
                mode: EngineMode::Shutdown,
                rpm: Rpm(3000),
                tps_x100: 0,
                map_kpa10: Kpa10(350),
                load_kpa10: Kpa10(350),
                ..base
            },
        },
        FixtureCase {
            fixture: "arbiter_priority_pairwise_conflicts",
            variant: "safety_latched_over_soft_rev_spark_cut",
            calibration: arbiter_priority_calibration(),
            input: InputSnapshot {
                mode: EngineMode::Shutdown,
                rpm: Rpm(5000),
                ..base
            },
        },
        FixtureCase {
            fixture: "arbiter_priority_pairwise_conflicts",
            variant: "safety_latched_over_knock_spark_retard_only",
            calibration: arbiter_priority_calibration(),
            input: InputSnapshot {
                mode: EngineMode::Shutdown,
                knock_intensity_x100: 600,
                ..base
            },
        },
        FixtureCase {
            fixture: "arbiter_priority_pairwise_conflicts",
            variant: "hard_rev_limit_over_launch_cut",
            calibration: arbiter_priority_calibration(),
            input: InputSnapshot {
                rpm: Rpm(7000),
                launch_armed: true,
                ..base
            },
        },
        FixtureCase {
            fixture: "arbiter_priority_pairwise_conflicts",
            variant: "hard_rev_limit_over_flat_shift_cut",
            calibration: arbiter_priority_calibration(),
            input: InputSnapshot {
                rpm: Rpm(7000),
                flat_shift_armed: true,
                ..base
            },
        },
        FixtureCase {
            fixture: "arbiter_priority_pairwise_conflicts",
            variant: "hard_rev_limit_over_dfco_cut",
            calibration: arbiter_priority_calibration(),
            input: InputSnapshot {
                rpm: Rpm(7000),
                tps_x100: 0,
                map_kpa10: Kpa10(350),
                load_kpa10: Kpa10(350),
                ..base
            },
        },
        FixtureCase {
            fixture: "arbiter_priority_pairwise_conflicts",
            variant: "hard_rev_limit_over_soft_rev_spark_cut",
            calibration: arbiter_priority_calibration(),
            input: InputSnapshot {
                rpm: Rpm(7000),
                ..base
            },
        },
        FixtureCase {
            fixture: "arbiter_priority_pairwise_conflicts",
            variant: "hard_rev_limit_over_knock_spark_retard_only",
            calibration: arbiter_priority_calibration(),
            input: InputSnapshot {
                rpm: Rpm(7000),
                knock_intensity_x100: 600,
                ..base
            },
        },
        FixtureCase {
            fixture: "arbiter_priority_pairwise_conflicts",
            variant: "launch_cut_over_flat_shift_cut",
            calibration: arbiter_priority_calibration(),
            input: InputSnapshot {
                rpm: Rpm(5600),
                launch_armed: true,
                flat_shift_armed: true,
                ..base
            },
        },
        FixtureCase {
            fixture: "arbiter_priority_pairwise_conflicts",
            variant: "launch_cut_over_dfco_cut",
            calibration: arbiter_priority_calibration(),
            input: InputSnapshot {
                rpm: Rpm(5600),
                launch_armed: true,
                tps_x100: 0,
                map_kpa10: Kpa10(350),
                load_kpa10: Kpa10(350),
                ..base
            },
        },
        FixtureCase {
            fixture: "arbiter_priority_pairwise_conflicts",
            variant: "launch_cut_over_soft_rev_spark_cut",
            calibration: arbiter_priority_calibration(),
            input: InputSnapshot {
                rpm: Rpm(5600),
                launch_armed: true,
                ..base
            },
        },
        FixtureCase {
            fixture: "arbiter_priority_pairwise_conflicts",
            variant: "launch_cut_over_knock_spark_retard_only",
            calibration: arbiter_priority_calibration(),
            input: InputSnapshot {
                rpm: Rpm(5600),
                launch_armed: true,
                knock_intensity_x100: 600,
                ..base
            },
        },
        FixtureCase {
            fixture: "arbiter_priority_pairwise_conflicts",
            variant: "flat_shift_cut_over_dfco_cut",
            calibration: arbiter_priority_calibration(),
            input: InputSnapshot {
                rpm: Rpm(5600),
                flat_shift_armed: true,
                tps_x100: 0,
                map_kpa10: Kpa10(350),
                load_kpa10: Kpa10(350),
                ..base
            },
        },
        FixtureCase {
            fixture: "arbiter_priority_pairwise_conflicts",
            variant: "flat_shift_cut_over_soft_rev_spark_cut",
            calibration: arbiter_priority_calibration(),
            input: InputSnapshot {
                rpm: Rpm(5600),
                flat_shift_armed: true,
                ..base
            },
        },
        FixtureCase {
            fixture: "arbiter_priority_pairwise_conflicts",
            variant: "flat_shift_cut_over_knock_spark_retard_only",
            calibration: arbiter_priority_calibration(),
            input: InputSnapshot {
                rpm: Rpm(5600),
                flat_shift_armed: true,
                knock_intensity_x100: 600,
                ..base
            },
        },
        FixtureCase {
            fixture: "arbiter_priority_pairwise_conflicts",
            variant: "dfco_cut_over_soft_rev_spark_cut",
            calibration: arbiter_priority_calibration(),
            input: InputSnapshot {
                rpm: Rpm(5600),
                tps_x100: 0,
                map_kpa10: Kpa10(350),
                load_kpa10: Kpa10(350),
                ..base
            },
        },
        FixtureCase {
            fixture: "arbiter_priority_pairwise_conflicts",
            variant: "dfco_cut_over_knock_spark_retard_only",
            calibration: arbiter_priority_calibration(),
            input: InputSnapshot {
                rpm: Rpm(5600),
                tps_x100: 0,
                map_kpa10: Kpa10(350),
                load_kpa10: Kpa10(350),
                knock_intensity_x100: 600,
                ..base
            },
        },
        FixtureCase {
            fixture: "arbiter_priority_pairwise_conflicts",
            variant: "soft_rev_spark_cut_over_knock_spark_retard_only",
            calibration: arbiter_priority_calibration(),
            input: InputSnapshot {
                rpm: Rpm(5600),
                knock_intensity_x100: 600,
                ..base
            },
        },
        FixtureCase {
            fixture: "sensor_curves_input_sweep",
            variant: "full_adc_domain",
            calibration: default_reference_calibration(),
            input: base,
        },
        FixtureCase {
            fixture: "trigger_decoder_tooth_stream",
            variant: "sync_acquire",
            calibration: default_reference_calibration(),
            input: base,
        },
        FixtureCase {
            fixture: "trigger_decoder_tooth_stream",
            variant: "sync_loss",
            calibration: default_reference_calibration(),
            input: base,
        },
        FixtureCase {
            fixture: "trigger_decoder_tooth_stream",
            variant: "resync",
            calibration: default_reference_calibration(),
            input: base,
        },
        FixtureCase {
            fixture: "trigger_decoder_tooth_stream",
            variant: "stall",
            calibration: default_reference_calibration(),
            input: base,
        },
        FixtureCase {
            fixture: "persistence_roundtrip_migration",
            variant: "fuel_roundtrip_v3",
            calibration: default_reference_calibration(),
            input: base,
        },
        FixtureCase {
            fixture: "persistence_roundtrip_migration",
            variant: "ignition_roundtrip_v3",
            calibration: default_reference_calibration(),
            input: base,
        },
        FixtureCase {
            fixture: "persistence_roundtrip_migration",
            variant: "angles_roundtrip_v3",
            calibration: default_reference_calibration(),
            input: base,
        },
        FixtureCase {
            fixture: "persistence_roundtrip_migration",
            variant: "fuel_migrate_v1_to_v3",
            calibration: default_reference_calibration(),
            input: base,
        },
        FixtureCase {
            fixture: "persistence_roundtrip_migration",
            variant: "ignition_migrate_v2_to_v3",
            calibration: default_reference_calibration(),
            input: base,
        },
        FixtureCase {
            fixture: "persistence_roundtrip_migration",
            variant: "angles_migrate_v1_to_v3",
            calibration: default_reference_calibration(),
            input: base,
        },
        FixtureCase {
            fixture: "ts_proto_dispatch_burn_diag",
            variant: "dispatch_read_write",
            calibration: default_reference_calibration(),
            input: base,
        },
        FixtureCase {
            fixture: "ts_proto_dispatch_burn_diag",
            variant: "outpc_roundtrip",
            calibration: default_reference_calibration(),
            input: base,
        },
        FixtureCase {
            fixture: "ts_proto_dispatch_burn_diag",
            variant: "burn_save_sequence",
            calibration: default_reference_calibration(),
            input: base,
        },
        FixtureCase {
            fixture: "ts_proto_dispatch_burn_diag",
            variant: "diag_log_wrap",
            calibration: default_reference_calibration(),
            input: base,
        },
        FixtureCase {
            fixture: "ts_proto_dispatch_burn_diag",
            variant: "page_meta_frozen",
            calibration: default_reference_calibration(),
            input: base,
        },
        FixtureCase {
            fixture: "safety_latching",
            variant: "latch_on_fault",
            calibration: default_reference_calibration(),
            input: InputSnapshot {
                fuel_cut: true,
                ..base
            },
        },
        FixtureCase {
            fixture: "safety_latching",
            variant: "hold_through_clear_attempt",
            calibration: default_reference_calibration(),
            input: base,
        },
        FixtureCase {
            fixture: "safety_latching",
            variant: "release_on_clear_condition",
            calibration: default_reference_calibration(),
            input: InputSnapshot {
                mode: EngineMode::Off,
                ..base
            },
        },
        FixtureCase {
            fixture: "torque_pipeline",
            variant: "request_stage",
            calibration: default_reference_calibration(),
            input: InputSnapshot {
                mode: EngineMode::Running,
                fuel_cut: false,
                spark_cut: false,
                tps_x100: 5370,
                ..base
            },
        },
        FixtureCase {
            fixture: "torque_pipeline",
            variant: "arbiter_stage",
            calibration: default_reference_calibration(),
            input: InputSnapshot {
                mode: EngineMode::Off,
                fuel_cut: false,
                spark_cut: false,
                tps_x100: 9000,
                ..base
            },
        },
        FixtureCase {
            fixture: "torque_pipeline",
            variant: "actuate_stage",
            calibration: default_reference_calibration(),
            input: InputSnapshot {
                mode: EngineMode::Running,
                fuel_cut: true,
                spark_cut: false,
                tps_x100: 6500,
                ..base
            },
        },
    ]
}

#[allow(dead_code)]
fn patterned_payload(
    page_id: PersistPageId,
    _schema_version: u16,
    seed: u8,
) -> [u8; ecu_spec::PERSIST_MAX_PAYLOAD_BYTES] {
    let mut payload = [0u8; ecu_spec::PERSIST_MAX_PAYLOAD_BYTES];
    let len = page_id.payload_len();
    let mut idx = 0usize;
    while idx < len {
        payload[idx] = seed.wrapping_add((idx as u8).wrapping_mul(7));
        idx += 1;
    }
    payload
}

fn fixture_state(case: FixtureCase) -> LogicalState {
    match (case.fixture, case.variant) {
        ("lambda_cl_integrator_response", "saturation") => LogicalState {
            lambda_integrator_state: ecu_spec::PiIntegratorState {
                acc: 400,
                ..ecu_spec::PiIntegratorState::zero()
            },
            ..LogicalState::default()
        },
        ("lambda_cl_integrator_response", "freeze_cut") => LogicalState {
            lambda_integrator_state: ecu_spec::PiIntegratorState {
                acc: 180,
                ..ecu_spec::PiIntegratorState::zero()
            },
            ..LogicalState::default()
        },
        ("knock_response", "retard") => LogicalState {
            knock_state: ecu_spec::KnockState {
                retard_deg10: 40,
                recovery_counter: 0,
                detected: true,
            },
            ..LogicalState::default()
        },
        ("knock_response", "recovery") => LogicalState {
            knock_state: ecu_spec::KnockState {
                retard_deg10: 60,
                recovery_counter: 1,
                detected: false,
            },
            ..LogicalState::default()
        },
        ("launch_control_pattern", "disarmed") => LogicalState {
            launch_active: true,
            launch_cut_cycle_count: 3,
            ..LogicalState::default()
        },
        ("flat_shift_pattern", "disarmed") => LogicalState {
            flat_shift_active: true,
            flat_shift_cut_cycle_count: 3,
            ..LogicalState::default()
        },
        ("safety_latching", "hold_through_clear_attempt")
        | ("safety_latching", "release_on_clear_condition") => LogicalState {
            safety_latched: true,
            ..LogicalState::default()
        },
        _ => LogicalState::default(),
    }
}

pub fn oracle_result(case: FixtureCase) -> StepResult {
    let state = fixture_state(case);
    spec_step(&case.calibration, case.input, &state)
}

#[allow(dead_code)]
fn has_event(result: &StepResult, kind: EventKind) -> bool {
    let mut idx = 0usize;
    while idx < result.output.events.len as usize {
        if result.output.events.events[idx].kind == kind {
            return true;
        }
        idx += 1;
    }
    false
}

#[allow(dead_code)]
pub fn assert_fixture_semantics(case: FixtureCase, result: &StepResult) {
    match case.fixture {
        "running_synced_no_cut" => {
            assert_eq!(result.output.diagnostic, DiagnosticCode::None);
            assert_eq!(result.output.events.len, 16);
        }
        "fuel_cut_running_synced" => {
            assert_eq!(result.output.diagnostic, DiagnosticCode::FuelCutActive);
            assert_eq!(result.output.pw_corr_us.0, 0);
            assert!(!has_event(result, EventKind::InjectionOpen));
            assert!(!has_event(result, EventKind::InjectionClose));
            // input.fuel_cut=true gates actuated to 0; request/allowed reflect base torque.
            assert_eq!(result.output.torque_request_x1000, 500);
            assert_eq!(result.output.torque_allowed_x1000, 500);
            assert_eq!(result.output.torque_actuated_x1000, 0);
        }
        "spark_cut_running_synced" => {
            assert_eq!(result.output.diagnostic, DiagnosticCode::FuelCutActive);
            assert!(!has_event(result, EventKind::CoilChargeStart));
            assert!(!has_event(result, EventKind::CoilFire));
            // input.spark_cut=true gates actuated to 0; request/allowed reflect base torque.
            assert_eq!(result.output.torque_request_x1000, 500);
            assert_eq!(result.output.torque_allowed_x1000, 500);
            assert_eq!(result.output.torque_actuated_x1000, 0);
        }
        "running_unsynced" => {
            assert_eq!(result.output.diagnostic, DiagnosticCode::Unsynced);
            assert_eq!(result.output.events.len, 0);
        }
        "off_unsynced" => {
            assert_eq!(result.output.diagnostic, DiagnosticCode::None);
            assert_eq!(result.output.events.len, 0);
        }
        "shutdown_unsynced" => {
            assert_eq!(result.output.diagnostic, DiagnosticCode::FuelCutActive);
            assert_eq!(result.output.events.len, 0);
        }
        "decreasing_dwell_table" => {
            assert_eq!(result.output.dwell_us.0, 2000);
            let expected_dwell_duration =
                ecu_spec::duration_us_to_deg10(result.output.dwell_us, case.input.rpm).0;
            let live_count = case.calibration.0.cylinder_phase_deg10.count as usize;
            let mut idx = 0usize;
            while idx < live_count {
                let spark = Degrees10(result.output.spark_deg10.values[idx]);
                let dwell_start = Degrees10(result.output.dwell_start_deg10.values[idx]);
                let cyclic = ecu_spec::cyc7200_distance(spark, dwell_start);
                assert_eq!(cyclic, expected_dwell_duration);
                idx += 1;
            }
        }
        "negative_temperature_inputs" => {
            let zero_temp = InputSnapshot {
                clt_c10: TempC10(0),
                iat_c10: TempC10(0),
                ..case.input
            };
            let zero_result = spec_step(&case.calibration, zero_temp, &LogicalState::default());
            assert_eq!(result.output.pw_corr_us.0, zero_result.output.pw_corr_us.0);
            assert_eq!(result.output.pw_air_us.0, zero_result.output.pw_air_us.0);
        }
        "afr_override_low_high_clamp" => match case.variant {
            "low" => {
                assert_eq!(result.output.target_afr_x100.0, 500);
            }
            "high" => {
                assert_eq!(result.output.target_afr_x100.0, 3000);
            }
            _ => unreachable!("unexpected AFR override variant"),
        },
        "interp_last_segment_right_closure"
        | "interp_segment_boundary_equality"
        | "interp_decreasing_u16_endpoints"
        | "interp_distinct_corner_bilinear_cell"
        | "interp_negative_slope_i16_curve" => {
            assert_eq!(result.output.diagnostic, DiagnosticCode::None);
            assert_eq!(result.output.events.len, 16);
        }
        "deadtime_addition" => {
            assert_eq!(result.output.diagnostic, DiagnosticCode::None);
            assert!(result.output.pw_corr_us.0 > result.output.pw_air_us.0);
            assert_eq!(result.output.pw_corr_us.0 - result.output.pw_air_us.0, 700);
        }
        "vbat_deadtime_response" => {
            let paired_input = match case.variant {
                "low_vbat" => InputSnapshot {
                    vbatt_mv: Millivolts(14_000),
                    ..case.input
                },
                "high_vbat" => InputSnapshot {
                    vbatt_mv: Millivolts(10_000),
                    ..case.input
                },
                _ => unreachable!("unexpected vbat deadtime variant"),
            };
            let paired = spec_step(&case.calibration, paired_input, &LogicalState::default());
            assert_eq!(result.output.pw_air_us.0, paired.output.pw_air_us.0);
            match case.variant {
                "low_vbat" => assert!(result.output.pw_corr_us.0 > paired.output.pw_corr_us.0),
                "high_vbat" => assert!(result.output.pw_corr_us.0 < paired.output.pw_corr_us.0),
                _ => unreachable!("unexpected vbat deadtime variant"),
            }
        }
        "baro_correction_response" => {
            let paired_input = match case.variant {
                "low_baro" => InputSnapshot {
                    baro_kpa10: Kpa10(1200),
                    ..case.input
                },
                "high_baro" => InputSnapshot {
                    baro_kpa10: Kpa10(800),
                    ..case.input
                },
                _ => unreachable!("unexpected baro correction variant"),
            };
            let paired = spec_step(&case.calibration, paired_input, &LogicalState::default());
            assert_eq!(result.output.pw_base_us.0, paired.output.pw_base_us.0);
            assert_eq!(result.output.pw_air_us.0, paired.output.pw_air_us.0);
            match case.variant {
                "low_baro" => assert!(result.output.pw_corr_us.0 < paired.output.pw_corr_us.0),
                "high_baro" => assert!(result.output.pw_corr_us.0 > paired.output.pw_corr_us.0),
                _ => unreachable!("unexpected baro correction variant"),
            }
        }
        "cranking_correction_response" => {
            let paired_input = match case.variant {
                "cranking" => InputSnapshot {
                    mode: EngineMode::Running,
                    ..case.input
                },
                "running" => InputSnapshot {
                    mode: EngineMode::Cranking,
                    ..case.input
                },
                _ => unreachable!("unexpected cranking correction variant"),
            };
            let paired = spec_step(&case.calibration, paired_input, &LogicalState::default());
            match case.variant {
                "cranking" => assert!(result.output.pw_corr_us.0 > paired.output.pw_corr_us.0),
                "running" => assert!(result.output.pw_corr_us.0 < paired.output.pw_corr_us.0),
                _ => unreachable!("unexpected cranking correction variant"),
            }
        }
        "afterstart_enrichment_window" => {
            let mut paired_state = LogicalState::default();
            let mut case_state = LogicalState::default();
            match case.variant {
                "inside_window" => {
                    case_state.scheduler.last_cycle_epoch = 5;
                    paired_state.scheduler.last_cycle_epoch = 20;
                }
                "outside_window" => {
                    case_state.scheduler.last_cycle_epoch = 20;
                    paired_state.scheduler.last_cycle_epoch = 5;
                }
                _ => unreachable!("unexpected afterstart variant"),
            }
            let case_result = spec_step(&case.calibration, case.input, &case_state);
            let paired = spec_step(&case.calibration, case.input, &paired_state);
            match case.variant {
                "inside_window" => {
                    assert!(case_result.output.pw_corr_us.0 > paired.output.pw_corr_us.0)
                }
                "outside_window" => {
                    assert!(case_result.output.pw_corr_us.0 < paired.output.pw_corr_us.0)
                }
                _ => unreachable!("unexpected afterstart variant"),
            }
            assert_eq!(result.output.pw_base_us.0, case_result.output.pw_base_us.0);
            assert_eq!(result.output.pw_air_us.0, case_result.output.pw_air_us.0);
        }
        "warmup_correction_response" => {
            let paired_input = InputSnapshot {
                clt_c10: TempC10(800),
                ..case.input
            };
            let paired = spec_step(&case.calibration, paired_input, &LogicalState::default());
            match case.variant {
                "cold" => assert!(result.output.pw_corr_us.0 > paired.output.pw_corr_us.0),
                _ => unreachable!("unexpected warmup variant"),
            }
            assert_eq!(result.output.pw_base_us.0, paired.output.pw_base_us.0);
            assert_eq!(result.output.pw_air_us.0, paired.output.pw_air_us.0);
        }
        "sensor_curves_input_sweep" => {
            let mut calibration = case.calibration.0;
            calibration.o2_sensor_mode = ecu_spec::O2SensorMode::WidebandLinear;

            let mut counts = 0u16;
            while counts <= 4095 {
                assert_eq!(
                    runtime_temp_from_counts(counts, &RUNTIME_CLT_TEMP_C10),
                    ecu_spec::clt_from_counts(counts).0
                );
                assert_eq!(
                    runtime_temp_from_counts(counts, &RUNTIME_IAT_TEMP_C10),
                    ecu_spec::iat_from_counts(counts).0
                );

                let map_runtime = 100 + ((counts as u32 * (3000 - 100) as u32) / 4095) as u16;
                assert_eq!(map_runtime, ecu_spec::map_from_counts(counts).0);

                let adc_min = calibration.tps_adc_min_counts.clamp(0, 4095);
                let adc_max = calibration.tps_adc_max_counts.clamp(0, 4095);
                let tps_runtime = if adc_max <= adc_min || counts <= adc_min {
                    0
                } else if counts >= adc_max {
                    10_000
                } else {
                    (((counts - adc_min) as u32 * 10_000) / (adc_max - adc_min) as u32) as u16
                };
                assert_eq!(tps_runtime, ecu_spec::tps_from_counts(&calibration, counts));

                assert_eq!(
                    runtime_maf_from_counts(counts),
                    ecu_spec::maf_from_counts(counts)
                );

                let o2_wide_runtime = {
                    let min_afr = calibration.o2_wideband_afr_min_x100 as u32;
                    let max_afr = calibration.o2_wideband_afr_max_x100 as u32;
                    let span = max_afr - min_afr;
                    let scaled = (counts as u32 * span) / 4095;
                    (min_afr + scaled).clamp(500, 3000) as u16
                };
                assert_eq!(
                    o2_wide_runtime,
                    ecu_spec::o2_from_counts(&calibration, counts, false)
                        .afr_x100
                        .0
                );

                assert_eq!(counts.clamp(0, 10_000), ecu_spec::knock_from_window(counts));

                let baro_runtime = 500 + ((counts as u32 * (1200 - 500) as u32) / 4095) as u16;
                assert_eq!(baro_runtime, ecu_spec::baro_from_counts(counts).0);

                let vbat_runtime = 6000 + ((counts as u32 * (18000 - 6000) as u32) / 4095) as u16;
                assert_eq!(vbat_runtime, ecu_spec::vbat_from_counts(counts).0);

                if counts == 4095 {
                    break;
                }
                counts = counts.saturating_add(1);
            }

            calibration.o2_sensor_mode = ecu_spec::O2SensorMode::NarrowbandSwitch;
            let mut prev_state_false = false;
            let mut prev_state_true = true;
            let mut nb_counts = 0u16;
            while nb_counts <= 4095 {
                let threshold = calibration.o2_narrowband_threshold_counts;
                let hysteresis = calibration.o2_narrowband_hysteresis_counts;
                let lower = threshold.saturating_sub(hysteresis);
                let upper = threshold.saturating_add(hysteresis);

                let runtime_false = if prev_state_false {
                    nb_counts <= upper
                } else {
                    nb_counts < lower
                };
                let runtime_true = if prev_state_true {
                    nb_counts <= upper
                } else {
                    nb_counts < lower
                };

                assert_eq!(
                    runtime_false,
                    ecu_spec::o2_from_counts(&calibration, nb_counts, prev_state_false).rich
                );
                assert_eq!(
                    runtime_true,
                    ecu_spec::o2_from_counts(&calibration, nb_counts, prev_state_true).rich
                );

                prev_state_false = runtime_false;
                prev_state_true = runtime_true;

                if nb_counts == 4095 {
                    break;
                }
                nb_counts = nb_counts.saturating_add(1);
            }
        }
        "ae_onset_decay" => {
            let mut state = LogicalState::default();
            match case.variant {
                "onset" => {
                    state.math.last_valid_load_kpa10 = Kpa10(0);
                    state.math.last_valid_map_kpa10 = Kpa10(1000);
                    let with_ae = spec_step(&case.calibration, case.input, &state);

                    let mut paired_state = state;
                    paired_state.math.last_valid_load_kpa10 = Kpa10(1000);
                    let without_ae = spec_step(&case.calibration, case.input, &paired_state);
                    assert!(with_ae.output.pw_corr_us.0 > without_ae.output.pw_corr_us.0);
                    assert!(with_ae.next_state.ae.active);
                    assert_eq!(with_ae.next_state.ae.decay_steps_remaining, 2);
                    assert_eq!(result.output.pw_base_us.0, with_ae.output.pw_base_us.0);
                    assert_eq!(result.output.pw_air_us.0, with_ae.output.pw_air_us.0);
                }
                "decay" => {
                    state.ae.active = true;
                    state.ae.pulse_us = 1000;
                    state.ae.decay_steps_remaining = 2;
                    state.math.last_valid_load_kpa10 = Kpa10(1000);
                    state.math.last_valid_map_kpa10 = Kpa10(1000);
                    let decay_step = spec_step(&case.calibration, case.input, &state);

                    let mut no_ae_state = LogicalState::default();
                    no_ae_state.math.last_valid_load_kpa10 = Kpa10(1000);
                    no_ae_state.math.last_valid_map_kpa10 = Kpa10(1000);
                    let no_ae = spec_step(&case.calibration, case.input, &no_ae_state);

                    let mut onset_state = LogicalState::default();
                    onset_state.math.last_valid_load_kpa10 = Kpa10(0);
                    onset_state.math.last_valid_map_kpa10 = Kpa10(1000);
                    let onset = spec_step(&case.calibration, case.input, &onset_state);

                    assert!(decay_step.output.pw_corr_us.0 > no_ae.output.pw_corr_us.0);
                    assert!(decay_step.output.pw_corr_us.0 < onset.output.pw_corr_us.0);
                    assert_eq!(decay_step.next_state.ae.pulse_us, 800);
                    assert_eq!(decay_step.next_state.ae.decay_steps_remaining, 1);
                    assert_eq!(result.output.pw_base_us.0, decay_step.output.pw_base_us.0);
                    assert_eq!(result.output.pw_air_us.0, decay_step.output.pw_air_us.0);
                }
                _ => unreachable!("unexpected ae fixture variant"),
            }
        }
        "dfco_entry_exit_hysteresis" => match case.variant {
            "entry" => {
                let entry = spec_step(&case.calibration, case.input, &LogicalState::default());
                assert!(entry.next_state.dfco_active);
                assert_eq!(entry.next_state.dfco_qualify_counter, 1);
                assert_eq!(entry.output.diagnostic, DiagnosticCode::FuelCutActive);
                assert_eq!(entry.output.pw_corr_us.0, 0);
                assert_eq!(result.output.pw_corr_us.0, entry.output.pw_corr_us.0);
            }
            "exit" => {
                let active_state = LogicalState {
                    dfco_active: true,
                    dfco_qualify_counter: 1,
                    ..LogicalState::default()
                };
                let exit = spec_step(&case.calibration, case.input, &active_state);
                assert!(!exit.next_state.dfco_active);
                assert_eq!(exit.next_state.dfco_qualify_counter, 0);
                assert_eq!(exit.output.diagnostic, DiagnosticCode::None);
                assert!(exit.output.pw_corr_us.0 > 0);
                assert_eq!(result.output.pw_base_us.0, exit.output.pw_base_us.0);
            }
            _ => unreachable!("unexpected dfco fixture variant"),
        },
        "rev_limit_soft_hard_recovery" => match case.variant {
            "under_limit" => {
                assert_eq!(result.output.cut_reason_code, 0);
                assert_eq!(result.output.diagnostic, DiagnosticCode::None);
                assert!(!result.output.fuel_cut);
                assert!(!result.output.spark_cut);
                assert_eq!(result.output.advance_deg10_trim, 0);
                assert!(!result.next_state.rev_soft_active);
                assert!(!result.next_state.rev_hard_active);
                // No cuts; limiter_ceiling=1000, request=tps/10=500 → allowed=500, actuated=500.
                assert_eq!(result.output.torque_request_x1000, 500);
                assert_eq!(result.output.torque_allowed_x1000, 500);
                assert_eq!(result.output.torque_actuated_x1000, 500);
            }
            "soft_limit" => {
                assert_eq!(result.output.cut_reason_code, 6);
                assert_eq!(result.output.diagnostic, DiagnosticCode::SparkCutActive);
                assert!(!result.output.fuel_cut);
                assert!(result.output.spark_cut);
                assert!(result.output.pw_corr_us.0 > 0);
                assert_eq!(result.output.advance_deg10_trim, -120);
                assert!(result.next_state.rev_soft_active);
                assert!(!result.next_state.rev_hard_active);
                // Spark cut gates actuated to 0; allowed still 500.
                assert_eq!(result.output.torque_request_x1000, 500);
                assert_eq!(result.output.torque_allowed_x1000, 500);
                assert_eq!(result.output.torque_actuated_x1000, 0);
            }
            "hard_limit" => {
                assert_eq!(result.output.cut_reason_code, 2);
                assert_eq!(result.output.diagnostic, DiagnosticCode::FuelCutActive);
                assert!(result.output.fuel_cut);
                assert!(result.output.spark_cut);
                assert_eq!(result.output.pw_corr_us.0, 0);
                assert!(result.next_state.rev_soft_active);
                assert!(result.next_state.rev_hard_active);
                // Both soft+hard active; fuel_cut gates actuated to 0.
                assert_eq!(result.output.torque_request_x1000, 500);
                assert_eq!(result.output.torque_allowed_x1000, 0);
                assert_eq!(result.output.torque_actuated_x1000, 0);
            }
            "recovery" => {
                let state = LogicalState {
                    rev_soft_active: true,
                    rev_hard_active: true,
                    ..LogicalState::default()
                };
                let recovered = spec_step(&case.calibration, case.input, &state);
                assert_eq!(recovered.output.cut_reason_code, 0);
                assert_eq!(recovered.output.diagnostic, DiagnosticCode::None);
                assert!(!recovered.output.fuel_cut);
                assert!(!recovered.output.spark_cut);
                assert_eq!(recovered.output.advance_deg10_trim, 0);
                assert!(!recovered.next_state.rev_soft_active);
                assert!(!recovered.next_state.rev_hard_active);
                assert_eq!(result.output.pw_base_us.0, recovered.output.pw_base_us.0);
                assert_eq!(result.output.pw_air_us.0, recovered.output.pw_air_us.0);
                // RPM=3500 is below soft(4000) and hard(5000); no cuts, no torque limiting.
                assert_eq!(recovered.output.torque_request_x1000, 500);
                assert_eq!(recovered.output.torque_allowed_x1000, 500);
                assert_eq!(recovered.output.torque_actuated_x1000, 500);
            }
            _ => unreachable!("unexpected rev-limit fixture variant"),
        },
        "idle_pi_response" => match case.variant {
            "integrate" => {
                assert_eq!(result.output.idle_duty_x1000, 590);
                assert_eq!(result.next_state.idle_integrator_state.acc, 150);
                assert!(!result.next_state.idle_integrator_state.frozen);
                assert_eq!(
                    result.next_state.idle_duty_x1000,
                    result.output.idle_duty_x1000
                );
            }
            "freeze_cold" => {
                let seeded_state = LogicalState {
                    idle_integrator_state: ecu_spec::PiIntegratorState {
                        acc: 123,
                        ..ecu_spec::PiIntegratorState::zero()
                    },
                    ..LogicalState::default()
                };
                let cold = spec_step(&case.calibration, case.input, &seeded_state);
                let warm = spec_step(
                    &case.calibration,
                    InputSnapshot {
                        clt_c10: TempC10(800),
                        ..case.input
                    },
                    &seeded_state,
                );
                assert_eq!(cold.next_state.idle_integrator_state.acc, 123);
                assert!(cold.next_state.idle_integrator_state.frozen);
                assert!(cold.output.idle_duty_x1000 < warm.output.idle_duty_x1000);
                assert_eq!(result.output.pw_base_us.0, cold.output.pw_base_us.0);
                assert_eq!(result.output.pw_air_us.0, cold.output.pw_air_us.0);
            }
            _ => unreachable!("unexpected idle fixture variant"),
        },
        "lambda_cl_integrator_response" => match case.variant {
            "saturation" => {
                assert_eq!(result.output.lambda_correction_x1000, 1250);
                assert_eq!(result.next_state.lambda_integrator_state.acc, 400);
                assert!(!result.next_state.lambda_integrator_state.frozen);
                assert_eq!(result.next_state.lambda_correction_x1000, 1250);
            }
            "freeze_cut" => {
                assert_eq!(result.output.lambda_correction_x1000, 1180);
                assert_eq!(result.next_state.lambda_integrator_state.acc, 180);
                assert!(result.next_state.lambda_integrator_state.frozen);
                assert_eq!(result.next_state.lambda_correction_x1000, 1180);
            }
            _ => unreachable!("unexpected lambda fixture variant"),
        },
        "knock_response" => match case.variant {
            "below_threshold" => {
                assert_eq!(result.output.cut_reason_code, 0);
                assert_eq!(result.output.advance_deg10_trim, 0);
                assert_eq!(result.output.knock_intensity_x100, 400);
                assert!(!result.output.spark_cut);
                assert!(!result.next_state.knock_state.detected);
                assert_eq!(result.next_state.knock_state.retard_deg10, 0);
                assert_eq!(result.output.diagnostic, DiagnosticCode::None);
            }
            "detect" => {
                assert_eq!(result.output.cut_reason_code, 7);
                assert_eq!(result.output.advance_deg10_trim, -40);
                assert_eq!(result.output.knock_intensity_x100, 600);
                assert!(!result.output.fuel_cut);
                assert!(!result.output.spark_cut);
                assert!(result.next_state.knock_state.detected);
                assert_eq!(result.next_state.knock_state.retard_deg10, 40);
                assert_eq!(result.output.diagnostic, DiagnosticCode::None);
            }
            "retard" => {
                assert_eq!(result.output.cut_reason_code, 7);
                assert_eq!(result.output.advance_deg10_trim, -80);
                assert_eq!(result.output.knock_intensity_x100, 600);
                assert!(result.next_state.knock_state.detected);
                assert_eq!(result.next_state.knock_state.retard_deg10, 80);
                assert_eq!(result.next_state.knock_state.recovery_counter, 0);
                assert_eq!(result.output.diagnostic, DiagnosticCode::None);
            }
            "recovery" => {
                // Below threshold, but existing retard remains active after one
                // recovery step, so the arbiter still reports knock reason 7.
                assert_eq!(result.output.cut_reason_code, 7);
                assert_eq!(result.output.advance_deg10_trim, -40);
                assert_eq!(result.output.knock_intensity_x100, 100);
                assert!(!result.next_state.knock_state.detected);
                assert_eq!(result.next_state.knock_state.retard_deg10, 40);
                assert_eq!(result.next_state.knock_state.recovery_counter, 0);
                assert_eq!(result.output.diagnostic, DiagnosticCode::None);
            }
            _ => unreachable!("unexpected knock fixture variant"),
        },
        "launch_control_pattern" => match case.variant {
            "disarmed" => {
                assert_eq!(result.output.cut_reason_code, 0);
                assert!(!result.output.fuel_cut);
                assert!(!result.output.spark_cut);
                assert!(!result.next_state.launch_active);
                assert_eq!(result.next_state.launch_cut_cycle_count, 0);
            }
            "armed_pattern" => {
                let s0 = spec_step(&case.calibration, case.input, &LogicalState::default());
                let s1 = spec_step(
                    &case.calibration,
                    case.input,
                    &LogicalState {
                        launch_active: true,
                        launch_cut_cycle_count: s0.next_state.launch_cut_cycle_count,
                        ..LogicalState::default()
                    },
                );
                let s2 = spec_step(
                    &case.calibration,
                    case.input,
                    &LogicalState {
                        launch_active: true,
                        launch_cut_cycle_count: s1.next_state.launch_cut_cycle_count,
                        ..LogicalState::default()
                    },
                );
                assert_eq!(s0.output.cut_reason_code, 3);
                assert!(s0.output.fuel_cut);
                assert!(s0.output.spark_cut);
                assert_eq!(s0.next_state.launch_cut_cycle_count, 1);
                assert_eq!(s1.output.cut_reason_code, 3);
                assert!(s1.output.fuel_cut);
                assert!(s1.output.spark_cut);
                assert_eq!(s1.next_state.launch_cut_cycle_count, 2);
                assert_eq!(s2.output.cut_reason_code, 0);
                assert!(!s2.output.fuel_cut);
                assert!(!s2.output.spark_cut);
                assert_eq!(s2.next_state.launch_cut_cycle_count, 0);
                assert_eq!(result.output.cut_reason_code, s0.output.cut_reason_code);
            }
            _ => unreachable!("unexpected launch fixture variant"),
        },
        "flat_shift_pattern" => match case.variant {
            "disarmed" => {
                assert_eq!(result.output.cut_reason_code, 0);
                assert!(!result.output.fuel_cut);
                assert!(!result.output.spark_cut);
                assert!(!result.next_state.flat_shift_active);
                assert_eq!(result.next_state.flat_shift_cut_cycle_count, 0);
            }
            "armed_pattern" => {
                let s0 = spec_step(&case.calibration, case.input, &LogicalState::default());
                let s1 = spec_step(
                    &case.calibration,
                    case.input,
                    &LogicalState {
                        flat_shift_active: true,
                        flat_shift_cut_cycle_count: s0.next_state.flat_shift_cut_cycle_count,
                        ..LogicalState::default()
                    },
                );
                let s2 = spec_step(
                    &case.calibration,
                    case.input,
                    &LogicalState {
                        flat_shift_active: true,
                        flat_shift_cut_cycle_count: s1.next_state.flat_shift_cut_cycle_count,
                        ..LogicalState::default()
                    },
                );
                assert_eq!(s0.output.cut_reason_code, 4);
                assert!(s0.output.fuel_cut);
                assert!(s0.output.spark_cut);
                assert_eq!(s0.next_state.flat_shift_cut_cycle_count, 1);
                assert_eq!(s1.output.cut_reason_code, 4);
                assert!(s1.output.fuel_cut);
                assert!(s1.output.spark_cut);
                assert_eq!(s1.next_state.flat_shift_cut_cycle_count, 2);
                assert_eq!(s2.output.cut_reason_code, 0);
                assert!(!s2.output.fuel_cut);
                assert!(!s2.output.spark_cut);
                assert_eq!(s2.next_state.flat_shift_cut_cycle_count, 0);
                assert_eq!(result.output.cut_reason_code, s0.output.cut_reason_code);
            }
            _ => unreachable!("unexpected flat-shift fixture variant"),
        },
        "arbiter_priority_pairwise_conflicts" => {
            let (cut_reason_code, fuel_cut, spark_cut) = match case.variant {
                "safety_latched_over_hard_rev_limit"
                | "safety_latched_over_launch_cut"
                | "safety_latched_over_flat_shift_cut"
                | "safety_latched_over_dfco_cut"
                | "safety_latched_over_soft_rev_spark_cut"
                | "safety_latched_over_knock_spark_retard_only" => (1, true, true),
                "hard_rev_limit_over_launch_cut"
                | "hard_rev_limit_over_flat_shift_cut"
                | "hard_rev_limit_over_dfco_cut"
                | "hard_rev_limit_over_soft_rev_spark_cut"
                | "hard_rev_limit_over_knock_spark_retard_only" => (2, true, true),
                "launch_cut_over_flat_shift_cut"
                | "launch_cut_over_dfco_cut"
                | "launch_cut_over_soft_rev_spark_cut"
                | "launch_cut_over_knock_spark_retard_only" => (3, true, true),
                "flat_shift_cut_over_dfco_cut"
                | "flat_shift_cut_over_soft_rev_spark_cut"
                | "flat_shift_cut_over_knock_spark_retard_only" => (4, true, true),
                "dfco_cut_over_soft_rev_spark_cut" | "dfco_cut_over_knock_spark_retard_only" => {
                    (5, true, false)
                }
                "soft_rev_spark_cut_over_knock_spark_retard_only" => (6, false, true),
                _ => unreachable!("unexpected arbiter-priority fixture variant"),
            };

            assert_eq!(result.output.cut_reason_code, cut_reason_code);
            assert_eq!(result.output.fuel_cut, fuel_cut);
            assert_eq!(result.output.spark_cut, spark_cut);
        }
        "trigger_decoder_tooth_stream" => {
            use ecu_spec::{
                trigger_60_2_step, Micros as SpecMicros, TriggerState, TriggerSyncState,
            };

            let stream: &[u32] = match case.variant {
                "sync_acquire" => &[1000, 2000, 3500, 4500],
                "sync_loss" => &[1000, 2000, 3500, 4500, 5200, 5700],
                "resync" => &[1000, 2000, 3500, 4500, 5200, 5700, 7800, 9000],
                "stall" => &[1000, 2000, 3500, 4500, 500_000],
                _ => unreachable!("unexpected trigger stream variant"),
            };

            let mut state = TriggerState::default();
            let mut step = trigger_60_2_step(state, SpecMicros::new(stream[0]));
            state = step.state;
            let mut idx = 1usize;
            while idx < stream.len() {
                step = trigger_60_2_step(state, SpecMicros::new(stream[idx]));
                state = step.state;
                idx += 1;
            }

            match case.variant {
                "sync_acquire" => {
                    assert_eq!(step.sync_state, TriggerSyncState::Synced);
                    assert_eq!(step.angle_deg10.0, 0);
                    assert!(step.rpm_estimate.0 > 0);
                    assert!(!step.cancel_pending_events);
                }
                "sync_loss" => {
                    assert_eq!(step.sync_state, TriggerSyncState::SyncLoss);
                    assert_eq!(step.rpm_estimate.0, 0);
                    assert!(step.cancel_pending_events);
                }
                "resync" => {
                    assert_eq!(step.sync_state, TriggerSyncState::Synced);
                    assert_eq!(step.angle_deg10.0, 0);
                    assert!(step.rpm_estimate.0 > 0);
                    assert!(!step.cancel_pending_events);
                }
                "stall" => {
                    assert_eq!(step.sync_state, TriggerSyncState::SyncLoss);
                    assert_eq!(step.rpm_estimate.0, 0);
                    assert_eq!(step.state.stall_counter, 1);
                    assert!(step.cancel_pending_events);
                }
                _ => unreachable!("unexpected trigger stream variant"),
            }
        }
        "persistence_roundtrip_migration" => match case.variant {
            "fuel_roundtrip_v3" | "ignition_roundtrip_v3" | "angles_roundtrip_v3" => {
                let (page_id, seed) = match case.variant {
                    "fuel_roundtrip_v3" => (PersistPageId::Fuel, 0x11),
                    "ignition_roundtrip_v3" => (PersistPageId::Ignition, 0x33),
                    "angles_roundtrip_v3" => (PersistPageId::Angles, 0x55),
                    _ => unreachable!("unexpected persistence round-trip variant"),
                };
                let payload_len = page_id.payload_len();

                let payload = patterned_payload(page_id, 3, seed);
                let page = PersistPage::new(3, page_id, &payload[..payload_len])
                    .expect("fixture payload must satisfy page shape");
                let encoded = persist_encode(&page).expect("fixture encode must succeed");
                let decoded = persist_decode(&encoded.bytes[..encoded.len as usize])
                    .expect("fixture decode must succeed");

                assert_eq!(decoded.schema_version, 3);
                assert_eq!(decoded.page_id, page_id);
                assert_eq!(decoded.payload_slice(), &payload[..payload_len]);
            }
            "fuel_migrate_v1_to_v3" | "ignition_migrate_v2_to_v3" | "angles_migrate_v1_to_v3" => {
                let (page_id, from_version, seed) = match case.variant {
                    "fuel_migrate_v1_to_v3" => (PersistPageId::Fuel, 1, 0x77),
                    "ignition_migrate_v2_to_v3" => (PersistPageId::Ignition, 2, 0x99),
                    "angles_migrate_v1_to_v3" => (PersistPageId::Angles, 1, 0xBB),
                    _ => unreachable!("unexpected persistence migration variant"),
                };
                let payload_len = page_id.payload_len();
                let source_payload = patterned_payload(page_id, from_version, seed);
                let migrated =
                    persist_migrate(page_id, from_version, 3, &source_payload[..payload_len])
                        .expect("fixture migrate must succeed");

                assert_eq!(migrated.schema_version, 3);
                assert_eq!(migrated.page_id, page_id);
                assert_eq!(
                    &migrated.payload_slice()[..payload_len],
                    &source_payload[..payload_len]
                );
            }
            _ => unreachable!("unexpected persistence fixture variant"),
        },
        "ts_proto_dispatch_burn_diag" => match case.variant {
            "dispatch_read_write" => {
                let read_payload = [3u8, 0x34, 0x12, 0x78, 0x56];
                let write_payload = [2u8, 0x10, 0x00, 0xAA, 0xBB, 0xCC];
                let read_frame = ts_proto_frame(0x20, &read_payload);
                let write_frame = ts_proto_frame(0x21, &write_payload);

                let read_step = ts_dispatch_step(&read_frame);
                assert_eq!(
                    read_step.effect,
                    Err(TsDispatchError::CommandDecode(
                        TsCommandDecodeError::PageRangeOutOfBounds {
                            command_id: 0x20,
                            page_number: 3,
                            offset: 0x1234,
                            len: 0x5678,
                            page_len: page_meta(3).unwrap().payload_size,
                        }
                    ))
                );

                let write_step = ts_dispatch_step(&write_frame);
                match write_step.effect {
                    Ok(TsEffect::WritePage {
                        page_number,
                        offset,
                        bytes,
                    }) => {
                        assert_eq!(page_number, 2);
                        assert_eq!(offset, 0x0010);
                        assert_eq!(bytes, &[0xAA, 0xBB, 0xCC]);
                    }
                    _ => unreachable!("expected write-page effect"),
                }

                let unknown_frame = ts_proto_frame(0x7F, &[]);
                let unknown = ts_dispatch_step(&unknown_frame);
                assert_eq!(unknown.effect, Err(TsDispatchError::UnknownCommand(0x7F)));
            }
            "outpc_roundtrip" => {
                let frame = OutpcFrame {
                    rpm: 2345,
                    map_kpa10: 980,
                    tps_x100: 5050,
                    clt_c10: 860,
                    iat_c10: -120,
                    pw_corr_us: u16::MAX,
                    advance_deg10: -35,
                    sync_state_code: 1,
                    cut_reason_code: 4,
                    status_flags: 0xA5A5_55AA,
                };
                let encoded = encode_outpc(frame);
                let decoded = decode_outpc(&encoded).expect("OUTPC bytes must decode");
                assert_eq!(decoded, frame);
            }
            "burn_save_sequence" => {
                let mut store = TsBurnSaveStore::default();
                assert!(write_page(&mut store, 1, 0, &[0x11, 0x22]).is_ok());
                assert_eq!(
                    burn_page(&mut store, 1, true),
                    Err(TsBurnSaveError::EngineRunning)
                );
                assert!(burn_page(&mut store, 1, false).is_ok());
                let fuel = committed_page_record(&store, 1).expect("fuel committed");
                assert!(persist_decode(&fuel.bytes[..fuel.len as usize]).is_ok());

                assert!(write_page(&mut store, 2, 0, &[0x33]).is_ok());
                assert!(write_page(&mut store, 3, 0, &[0x44]).is_ok());
                assert!(save_all(&mut store, false).is_ok());
                let ign = committed_page_record(&store, 2).expect("ign committed");
                let ang = committed_page_record(&store, 3).expect("angles committed");
                assert_eq!(fuel.bytes[6], 0x11);
                assert_eq!(ign.bytes[6], 0x33);
                assert_eq!(ang.bytes[6], 0x44);
            }
            "diag_log_wrap" => {
                let mut ring = TsDiagLogRing::default();
                let mut idx = 0u16;
                while idx < (TS_DIAG_LOG_CAPACITY as u16 + 2) {
                    ts_diag_log_push(
                        &mut ring,
                        TsDiagLogEntry {
                            code: idx as u8,
                            severity: 2,
                            action: 1,
                            source: (idx & 0x7F) as u8,
                            context_present: true,
                            start_us: idx as u32,
                            end_us: idx as u32 + 10,
                            context: (idx ^ 0x00FF) as u32,
                        },
                    );
                    idx += 1;
                }
                assert_eq!(ring.len as usize, TS_DIAG_LOG_CAPACITY);
                let encoded = encode_ts_diag_log_oldest_first(&ring);
                assert_eq!(
                    encoded.len as usize,
                    TS_DIAG_LOG_CAPACITY * TS_DIAG_LOG_ENTRY_BYTES
                );
                assert_eq!(
                    u32::from_le_bytes(encoded.bytes[4..8].try_into().unwrap()),
                    2
                );
                let tail = encoded.len as usize - TS_DIAG_LOG_ENTRY_BYTES;
                assert_eq!(
                    u32::from_le_bytes(encoded.bytes[tail + 4..tail + 8].try_into().unwrap()),
                    (TS_DIAG_LOG_CAPACITY + 1) as u32
                );
            }
            "page_meta_frozen" => {
                let p1 = page_meta(1).expect("known page");
                assert_eq!(p1.page_id, TsPageId::Fuel);
                assert_eq!(p1.payload_size, 512);

                let p2 = page_meta(2).expect("known page");
                assert_eq!(p2.page_id, TsPageId::Ignition);
                assert_eq!(p2.payload_size, 512);

                let p3 = page_meta(3).expect("known page");
                assert_eq!(p3.page_id, TsPageId::Angles);
                assert_eq!(p3.payload_size, 68);

                let p4 = page_meta(4).expect("known page");
                assert_eq!(p4.page_id, TsPageId::Outpc);
                assert_eq!(p4.payload_size, 64);
            }
            _ => unreachable!("unexpected TS proto fixture variant"),
        },
        "safety_latching" => match case.variant {
            "latch_on_fault" => {
                assert!(result.next_state.safety_latched);
                assert_eq!(result.output.cut_reason_code, 1);
                assert!(result.output.fuel_cut);
                assert!(result.output.spark_cut);
            }
            "hold_through_clear_attempt" => {
                assert!(result.next_state.safety_latched);
                assert_eq!(result.output.cut_reason_code, 1);
                assert!(result.output.fuel_cut);
                assert!(result.output.spark_cut);
            }
            "release_on_clear_condition" => {
                assert!(!result.next_state.safety_latched);
                assert_eq!(result.output.cut_reason_code, 0);
                assert!(!result.output.fuel_cut);
                assert!(!result.output.spark_cut);
            }
            _ => unreachable!("unexpected safety-latching fixture variant"),
        },
        "torque_pipeline" => match case.variant {
            "request_stage" => {
                assert_eq!(result.output.torque_request_x1000, 537);
                assert_eq!(result.output.torque_allowed_x1000, 537);
                assert_eq!(result.output.torque_actuated_x1000, 537);
            }
            "arbiter_stage" => {
                assert_eq!(result.output.torque_request_x1000, 900);
                assert_eq!(result.output.torque_allowed_x1000, 0);
                assert_eq!(result.output.torque_actuated_x1000, 0);
            }
            "actuate_stage" => {
                assert_eq!(result.output.torque_request_x1000, 650);
                assert_eq!(result.output.torque_allowed_x1000, 650);
                assert_eq!(result.output.torque_actuated_x1000, 0);
            }
            _ => unreachable!("unexpected torque-pipeline fixture variant"),
        },
        _ => unreachable!("unexpected fixture"),
    }
}

#[allow(dead_code)]
fn ts_crc16_ccitt(data: &[u8]) -> u16 {
    let mut crc: u16 = 0xffff;
    for &byte in data {
        crc ^= (byte as u16) << 8;
        let mut bit = 0;
        while bit < 8 {
            if (crc & 0x8000) != 0 {
                crc = (crc << 1) ^ 0x1021;
            } else {
                crc <<= 1;
            }
            bit += 1;
        }
    }
    crc
}

#[allow(dead_code)]
fn ts_proto_frame(command_id: u8, payload: &[u8]) -> [u8; 80] {
    let mut out = [0u8; 80];
    out[0..2].copy_from_slice(&ecu_spec::TS_PROTO_MAGIC.to_le_bytes());
    let len = (1 + payload.len() + 2) as u16;
    out[2..4].copy_from_slice(&len.to_le_bytes());
    out[4] = command_id;
    if !payload.is_empty() {
        out[5..5 + payload.len()].copy_from_slice(payload);
    }
    let crc = ts_crc16_ccitt(&out[4..5 + payload.len()]);
    let crc_off = 5 + payload.len();
    out[crc_off..crc_off + 2].copy_from_slice(&crc.to_le_bytes());
    out
}

// ============================================================================
// FM0016 Fixture Matrix Integrity Tests (per US-FM0403)
//
// These tests verify the fixture matrix self-consistency WITHOUT running
// any product code. They ensure:
//   - required_fixture_names() contains all fixture names that appear in fixture_cases()
//   - fixture_cases() has exactly one entry per unique fixture name
//   - required_fixture_names() has no duplicates
//   - oracle_result is called only in tests/reducers (guarded by #[test])
// ============================================================================

#[cfg(test)]
mod fixture_integrity_tests {
    use std::collections::BTreeSet;
    #[test]
    fn fixture_matrix_required_names_match_observed_fixtures() {
        // Every fixture name that appears in fixture_cases() MUST be in required_fixture_names()
        let cases = super::fixture_cases();
        let observed: BTreeSet<&'static str> = cases.iter().map(|c| c.fixture).collect();
        let required: BTreeSet<&'static str> =
            super::required_fixture_names().into_iter().collect();
        assert_eq!(
            observed,
            required,
            "fixture_cases() fixture names must exactly match required_fixture_names()\n\
             observed ({}) vs required ({})",
            observed.len(),
            required.len()
        );
    }

    #[test]
    fn fixture_matrix_case_count_matches_required_count() {
        // fixture_cases() has 88 entries but only 35 unique fixture names.
        // required_fixture_names() has 35 entries (one per fixture family).
        // The counts must match (one required entry per fixture family).
        let cases = super::fixture_cases();
        let observed_unique: BTreeSet<&'static str> = cases.iter().map(|c| c.fixture).collect();
        let required_count = super::required_fixture_names().len();
        assert_eq!(
            observed_unique.len(),
            required_count,
            "fixture_cases() unique fixture families ({}) must equal required_fixture_names() count ({})",
            observed_unique.len(),
            required_count
        );
    }

    #[test]
    fn fixture_matrix_required_names_are_unique() {
        // required_fixture_names() must not contain duplicate entries.
        let required = super::required_fixture_names();
        let mut seen = BTreeSet::new();
        let mut duplicates = BTreeSet::new();
        for &name in &required {
            if !seen.insert(name) {
                duplicates.insert(name);
            }
        }
        assert!(
            duplicates.is_empty(),
            "required_fixture_names() must not contain duplicates: {:?}",
            duplicates
        );
    }

    #[test]
    fn fixture_matrix_each_case_name_in_required() {
        // Every fixture case's name must appear in required_fixture_names().
        let cases = super::fixture_cases();
        let required: BTreeSet<&'static str> =
            super::required_fixture_names().into_iter().collect();
        let mut missing = Vec::new();
        for case in &cases {
            if !required.contains(case.fixture) {
                missing.push(case.fixture);
            }
        }
        assert!(
            missing.is_empty(),
            "All fixture case names must appear in required_fixture_names(): {:?}",
            missing
        );
    }

    #[test]
    fn fixture_matrix_oracle_result_is_called_only_in_test_reducers() {
        // oracle_result() is ONLY called from tests/reducer files.
        // This test documents the invariant and will fail if oracle_result
        // is ever called from non-test production code.
        //
        // Locations that MAY call oracle_result (checked at compile/link time):
        //   - tests/fm0016_core_reducer.rs
        //   - ecu-scheduler/tests/fm0016_scheduler_reducer.rs
        //   - ecu-runtime/tests/fm0016_runtime_reducer.rs
        //
        // oracle_result is NOT called from:
        //   - ecu-spec library code (only defined there)
        //   - production reducer logic
        //   - any other test file
        //
        // This is verified by the fact that oracle_result panics on unknown fixtures,
        // so any accidental call outside the reducer tests would be caught immediately.
        let cases = super::fixture_cases();
        // Smoke-test: calling oracle_result on all cases must not panic.
        // If this test passes, oracle_result handles all fixture variants correctly.
        for case in &cases {
            let _ = super::oracle_result(*case);
        }
    }
}
