use ecu_spec::{
    arbiter_step, baro_correction, baro_from_counts, bilerp_u16, cam_phase_step, clt_from_counts,
    cyc7200_distance, deadtime_lookup, duration_us_to_deg10, fault_event_for_clear,
    fault_event_from_state, find_segment, iat_from_counts, idle_step, idle_timing_step,
    idle_timing_step_with_base, knock_from_window, lambda_step_with_error, lerp_i16, lerp_u16,
    lookup_idle_advance_deg10, lookup_target_afr, maf_from_counts, map_from_counts, norm7200,
    o2_from_counts, schedule_all_cylinders, schedule_all_cylinders_with_advance_trim,
    select_idle_or_running_advance_deg10, sensor_plausibility_step, sensor_slew_step, step,
    tps_from_counts, trigger_60_2_step, validate_calibration, vbat_correction, vbat_from_counts,
    AeState, AfrOverride, AfrX100, ArbiterInputs, Axis16, Calibration, CamPhase, CamPhaseState,
    CamTooth, DiagState, DiagnosticCode, EngineMode, EventBatch, FlatShiftResult, FuelModel,
    FuelOutput, InjectionAngleMode, InputSnapshot, KnockResult, KnockState, Kpa10, LogicalState,
    MathState, Micros, Millivolts, O2SensorMode, PiIntegratorState, PulseWidthUs, RatioX1000,
    RevLimitResult, Rpm, SchedulerState, SensorPlausibilityInput, SensorPlausibilityState,
    SensorSlewInput, SensorSlewState, SignedCurve16, SignedDegrees10, SpecCancelReason,
    SpecFaultAction, SpecFaultCode, SpecFaultEvent, SpecFaultPersistence, SpecFaultSeverity,
    SpecFaultState, SyncState, Table2D16, TempC10, TriggerState, TriggerSyncState, TrimPolicy,
    ValidatedCalibration, ValidationError,
};
use proptest::prelude::*;

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

fn table_u16_2x2(
    rpm_axis: Axis16,
    load_axis: Axis16,
    c00: u16,
    c01: u16,
    c10: u16,
    c11: u16,
) -> Table2D16<u16> {
    Table2D16 {
        rpm_axis,
        load_axis,
        values: {
            let mut values = [[0u16; 16]; 16];
            values[0][0] = c00;
            values[0][1] = c01;
            values[1][0] = c10;
            values[1][1] = c11;
            values
        },
    }
}

fn table_i16_2x2(
    rpm_axis: Axis16,
    load_axis: Axis16,
    c00: i16,
    c01: i16,
    c10: i16,
    c11: i16,
) -> Table2D16<i16> {
    Table2D16 {
        rpm_axis,
        load_axis,
        values: {
            let mut values = [[0i16; 16]; 16];
            values[0][0] = c00;
            values[0][1] = c01;
            values[1][0] = c10;
            values[1][1] = c11;
            values
        },
    }
}

fn uniform_u16_table(value: u16) -> Table2D16<u16> {
    table_u16_2x2(
        axis(&[500, 1000]),
        axis(&[500, 1000]),
        value,
        value,
        value,
        value,
    )
}

fn uniform_i16_table(value: i16) -> Table2D16<i16> {
    table_i16_2x2(
        axis(&[500, 1000]),
        axis(&[500, 1000]),
        value,
        value,
        value,
        value,
    )
}

fn uniform_u32_table(value: u32) -> Table2D16<u32> {
    Table2D16 {
        rpm_axis: axis(&[500, 1000]),
        load_axis: axis(&[500, 1000]),
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

fn uniform_curve(axis_values: &[u16], value: u16) -> ecu_spec::Curve16 {
    ecu_spec::Curve16 {
        axis: axis(axis_values),
        values: {
            let mut values = [0u16; 16];
            let mut idx = 0usize;
            while idx < axis_values.len() {
                values[idx] = value;
                idx += 1;
            }
            values
        },
    }
}

fn signed_curve_2x2(axis_values: &[u16], low: i16, high: i16) -> SignedCurve16 {
    let mut curve = SignedCurve16 {
        axis: axis(axis_values),
        ..SignedCurve16::default()
    };
    curve.values[0] = low;
    curve.values[1] = high;
    curve
}

fn bilinear_surface_value(a: u32, b: u32, c: u32, d: u32, rpm: u16, load: u16) -> u32 {
    let rpm = rpm as u32;
    let load = load as u32;
    a + (b * rpm) + (c * load) + (d * rpm * load)
}

fn table_u16_from_surface(
    rpm_axis: Axis16,
    load_axis: Axis16,
    a: u32,
    b: u32,
    c: u32,
    d: u32,
) -> Table2D16<u16> {
    table_u16_2x2(
        rpm_axis,
        load_axis,
        bilinear_surface_value(a, b, c, d, rpm_axis.values[0], load_axis.values[0]) as u16,
        bilinear_surface_value(a, b, c, d, rpm_axis.values[1], load_axis.values[0]) as u16,
        bilinear_surface_value(a, b, c, d, rpm_axis.values[0], load_axis.values[1]) as u16,
        bilinear_surface_value(a, b, c, d, rpm_axis.values[1], load_axis.values[1]) as u16,
    )
}

fn scale_table(table: &Table2D16<u16>, scale: u16) -> Table2D16<u16> {
    let mut scaled = *table;
    let mut load_idx = 0usize;
    while load_idx < 2 {
        let mut rpm_idx = 0usize;
        while rpm_idx < 2 {
            scaled.values[load_idx][rpm_idx] *= scale;
            rpm_idx += 1;
        }
        load_idx += 1;
    }
    scaled
}

#[test]
fn fault_event_projection_keeps_no_fault_inactive() {
    assert_eq!(
        fault_event_from_state(SpecFaultState::default()),
        SpecFaultEvent {
            active: false,
            action: SpecFaultAction::None,
            persistence: SpecFaultPersistence::Inactive,
        }
    );
}

#[test]
fn fault_event_projection_maps_warning_sensor_fault_to_limp() {
    assert_eq!(
        fault_event_from_state(SpecFaultState {
            code: SpecFaultCode::SensorOutOfRange,
            severity: SpecFaultSeverity::Warning,
            cancel_reason: SpecCancelReason::Manual,
        }),
        SpecFaultEvent {
            active: true,
            action: SpecFaultAction::LimpHome,
            persistence: SpecFaultPersistence::LatchedUntilClear,
        }
    );
}

#[test]
fn fault_event_projection_maps_critical_safety_fault_to_shutdown() {
    assert_eq!(
        fault_event_from_state(SpecFaultState {
            code: SpecFaultCode::SafetyCut,
            severity: SpecFaultSeverity::Critical,
            cancel_reason: SpecCancelReason::SafetyShutdown,
        }),
        SpecFaultEvent {
            active: true,
            action: SpecFaultAction::Shutdown,
            persistence: SpecFaultPersistence::LatchedUntilClear,
        }
    );
}

#[test]
fn fault_event_projection_maps_safety_cut_to_shutdown_regardless_of_severity() {
    assert_eq!(
        fault_event_from_state(SpecFaultState {
            code: SpecFaultCode::SafetyCut,
            severity: SpecFaultSeverity::Info,
            cancel_reason: SpecCancelReason::Manual,
        }),
        SpecFaultEvent {
            active: true,
            action: SpecFaultAction::Shutdown,
            persistence: SpecFaultPersistence::LatchedUntilClear,
        }
    );
}

#[test]
fn fault_event_projection_maps_clear_from_active_fault_to_cleared_event() {
    assert_eq!(
        fault_event_for_clear(SpecFaultState {
            code: SpecFaultCode::SensorOutOfRange,
            severity: SpecFaultSeverity::Warning,
            cancel_reason: SpecCancelReason::Manual,
        }),
        SpecFaultEvent {
            active: false,
            action: SpecFaultAction::Cleared,
            persistence: SpecFaultPersistence::Inactive,
        }
    );
}

fn valid_state() -> impl Strategy<Value = LogicalState> {
    (
        -500i16..500,
        -500i16..500,
        -200i16..200,
        -200i16..200,
        -500i16..500,
        -200i16..200,
        0u32..100_000,
        prop_oneof![
            Just(DiagnosticCode::None),
            Just(DiagnosticCode::FuelCutActive),
            Just(DiagnosticCode::SparkCutActive),
            Just(DiagnosticCode::Unsynced),
            Just(DiagnosticCode::SensorPlausibilityFault),
        ],
        any::<bool>(),
        any::<bool>(),
        any::<bool>(),
    )
        .prop_map(
            |(
                map_delta,
                load_delta,
                clt_delta,
                iat_delta,
                baro_delta,
                trim_delta,
                last_cycle_epoch,
                current,
                unsynced,
                fuel_cut_active,
                spark_cut_active,
            )| {
                LogicalState {
                    math: MathState {
                        last_valid_map_kpa10: Kpa10((1000i32 + map_delta as i32) as u16),
                        last_valid_load_kpa10: Kpa10((1000i32 + load_delta as i32) as u16),
                        last_valid_clt_c10: TempC10(clt_delta),
                        last_valid_iat_c10: TempC10(iat_delta),
                        last_valid_baro_kpa10: Kpa10((1000i32 + baro_delta as i32) as u16),
                        trim_ratio_x1000: RatioX1000((1000i32 + trim_delta as i32) as u16),
                    },
                    scheduler: SchedulerState {
                        pending: EventBatch::default(),
                        last_cycle_epoch,
                    },
                    ae: AeState::default(),
                    idle_integrator_state: PiIntegratorState::zero(),
                    idle_timing_integrator_state: PiIntegratorState::zero(),
                    idle_duty_x1000: 0,
                    lambda_integrator_state: PiIntegratorState::zero(),
                    lambda_correction_x1000: 1000,
                    dfco_active: false,
                    dfco_qualify_counter: 0,
                    rev_soft_active: false,
                    rev_hard_active: false,
                    launch_active: false,
                    launch_cut_cycle_count: 0,
                    flat_shift_active: false,
                    flat_shift_cut_cycle_count: 0,
                    knock_state: KnockState::default(),
                    safety_latched: false,
                    knock_intensity_x100: 0,
                    sensor_plausibility_state: SensorPlausibilityState::default(),
                    sensor_slew_state: SensorSlewState::default(),
                    diag: DiagState {
                        current,
                        unsynced,
                        fuel_cut_active,
                        spark_cut_active,
                    },
                }
            },
        )
}

fn canonical_calibration(mode: InjectionAngleMode) -> ValidatedCalibration {
    ValidatedCalibration(Calibration {
        fuel_model: FuelModel::SpeedDensityRequiredFuel,
        ve_table: uniform_u16_table(8000),
        afr_target_table: uniform_u16_table(1470),
        spark_advance_table_deg10: uniform_i16_table(150),
        dwell_table_us: uniform_u32_table(2500),
        injection_target_table_deg10: uniform_u16_table(360),
        deadtime_table_us: table_u16_2x2(
            axis(&[1000, 13000]),
            axis(&[1000, 3000]),
            800,
            800,
            800,
            800,
        ),
        clt_corr_curve: uniform_curve(&[0, 100], 1000),
        iat_corr_curve: uniform_curve(&[0, 100], 1000),
        baro_corr_curve: uniform_curve(&[0, 100], 1000),
        vbat_corr_curve: uniform_curve(&[8000, 16000], 1000),
        cranking_curve: uniform_curve(&[0, 100], 1000),
        afterstart_table: table_u16_2x2(axis(&[0, 10]), axis(&[0, 100]), 1000, 1000, 1000, 1000),
        afterstart_window_cycles: 0,
        warmup_curve: uniform_curve(&[0, 100], 1000),
        ae_tps_threshold_curve: uniform_curve(&[500, 7000], 20000),
        ae_map_threshold_curve: uniform_curve(&[100, 3000], 20000),
        ae_shot_curve_us: uniform_curve(&[100, 3000], 0),
        ae_decay_steps_curve: uniform_curve(&[100, 3000], 0),
        ae_decay_ratio_curve_x1000: uniform_curve(&[0, 16], 1000),
        dfco_entry_rpm: Rpm(20_000),
        dfco_exit_rpm: Rpm(19_000),
        dfco_entry_tps_x100: 0,
        dfco_exit_tps_x100: 100,
        dfco_entry_map_kpa10: Kpa10(0),
        dfco_delay_cycles: 0,
        soft_rev_rpm: Rpm(19_500),
        hard_rev_rpm: Rpm(20_000),
        rev_hysteresis_rpm: Rpm(100),
        soft_retard_max_deg10: 0,
        launch_rpm_limit: Rpm(20_000),
        launch_cut_cycles: 0,
        flat_shift_rpm_min: Rpm(20_000),
        flat_shift_cut_cycles: 0,
        knock_threshold_x100: 500,
        knock_retard_step_deg10: 20,
        knock_retard_max_deg10: 200,
        knock_recovery_step_deg10: 10,
        knock_recovery_delay_cycles: 2,
        tps_adc_min_counts: 0,
        tps_adc_max_counts: 4095,
        idle_target_rpm: Rpm(900),
        idle_base_duty_x1000: 0,
        idle_kp_x1000: 0,
        idle_ki_x1000: 0,
        idle_timing_enabled: false,
        idle_timing_pid_enabled: false,
        idle_timing_rpm_max: Rpm(1200),
        idle_timing_tps_max_x100: 200,
        idle_advance_curve_deg10: signed_curve_2x2(&[500, 1000], 0, 0),
        idle_timing_kp_x1000: 0,
        idle_timing_ki_x1000: 0,
        idle_timing_min_trim_deg10: -300,
        idle_timing_max_trim_deg10: 300,
        clt_timing_corr_curve_deg10: signed_curve_2x2(&[0, 1000], 0, 0),
        iat_timing_corr_curve_deg10: signed_curve_2x2(&[0, 1000], 0, 0),
        lambda_kp_x1000: 0,
        lambda_ki_x1000: 0,
        o2_sensor_mode: ecu_spec::O2SensorMode::WidebandLinear,
        o2_wideband_afr_min_x100: 500,
        o2_wideband_afr_max_x100: 3000,
        o2_narrowband_threshold_counts: 2048,
        o2_narrowband_hysteresis_counts: 64,
        o2_narrowband_rich_afr_x100: 1400,
        o2_narrowband_lean_afr_x100: 1550,
        required_fuel_us: 3000,
        pref_kpa10: 1000,
        stoich_afr_x100: 1470,
        trim_policy: TrimPolicy::Identity,
        pw_max_policy: ecu_spec::PwMaxPolicy::Fixed,
        pw_max_us: 25_000,
        injection_angle_mode: mode,
        cylinder_phase_deg10: ecu_spec::CylinderArrayU16 {
            count: 4,
            values: [0, 1800, 3600, 5400, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        },
    })
}

fn generated_calibration_template() -> impl Strategy<Value = Calibration> {
    (
        -1000i16..1000,
        -100i16..100,
        -50i16..50,
        -500i32..500,
        -200i16..200,
        -100i16..100,
        -500i32..500,
        -100i16..100,
        -100i16..100,
        -5000i32..5000,
    )
        .prop_map(
            move |(
                ve_delta,
                afr_delta,
                spark_delta,
                dwell_delta,
                deadtime_delta,
                corr_delta,
                required_fuel_delta,
                pref_delta,
                stoich_delta,
                pw_max_delta,
            )| {
                let mut cal = canonical_calibration(InjectionAngleMode::EndOfInjection).0;
                let c = &mut cal;

                let ve = (8000i32 + ve_delta as i32) as u16;
                let afr = (1470i32 + afr_delta as i32) as u16;
                let spark = 150i32 + spark_delta as i32;
                let dwell = (2500i64 + dwell_delta as i64) as u32;
                let deadtime = (800i32 + deadtime_delta as i32) as u16;
                let corr = (1000i32 + corr_delta as i32) as u16;
                let required_fuel = (3000i64 + required_fuel_delta as i64) as u32;
                let pref = (1000i32 + pref_delta as i32) as u16;
                let stoich = (1470i32 + stoich_delta as i32) as u16;
                let pw_max = (25_000i64 + pw_max_delta as i64) as u32;

                c.ve_table = table_u16_2x2(axis(&[500, 1000]), axis(&[500, 1000]), ve, ve, ve, ve);
                c.afr_target_table =
                    table_u16_2x2(axis(&[500, 1000]), axis(&[500, 1000]), afr, afr, afr, afr);
                c.spark_advance_table_deg10 = table_i16_2x2(
                    axis(&[500, 1000]),
                    axis(&[500, 1000]),
                    spark as i16,
                    spark as i16,
                    spark as i16,
                    spark as i16,
                );
                c.dwell_table_us.rpm_axis = axis(&[500, 1000]);
                c.dwell_table_us.load_axis = axis(&[500, 1000]);
                c.dwell_table_us.values[0][0] = dwell;
                c.dwell_table_us.values[0][1] = dwell;
                c.dwell_table_us.values[1][0] = dwell;
                c.dwell_table_us.values[1][1] = dwell;
                c.injection_target_table_deg10 =
                    table_u16_2x2(axis(&[500, 1000]), axis(&[500, 1000]), 360, 360, 360, 360);
                c.deadtime_table_us.rpm_axis = axis(&[1000, 13000]);
                c.deadtime_table_us.load_axis = axis(&[1000, 3000]);
                c.deadtime_table_us.values[0][0] = deadtime;
                c.deadtime_table_us.values[0][1] = deadtime;
                c.deadtime_table_us.values[1][0] = deadtime;
                c.deadtime_table_us.values[1][1] = deadtime;
                c.clt_corr_curve.axis = axis(&[0, 100]);
                c.clt_corr_curve.values[0] = corr;
                c.clt_corr_curve.values[1] = corr;
                c.iat_corr_curve.axis = axis(&[0, 100]);
                c.iat_corr_curve.values[0] = corr;
                c.iat_corr_curve.values[1] = corr;
                c.baro_corr_curve.axis = axis(&[0, 100]);
                c.baro_corr_curve.values[0] = corr;
                c.baro_corr_curve.values[1] = corr;
                c.required_fuel_us = required_fuel;
                c.pref_kpa10 = pref;
                c.stoich_afr_x100 = stoich;
                c.pw_max_us = pw_max;
                cal
            },
        )
}

fn valid_calibration(mode: InjectionAngleMode) -> impl Strategy<Value = ValidatedCalibration> {
    prop_oneof![
        Just(canonical_calibration(mode)),
        generated_calibration_template().prop_map(move |mut cal| {
            cal.injection_angle_mode = mode;
            ValidatedCalibration(cal)
        }),
    ]
}

fn valid_calibration_pair() -> impl Strategy<Value = (ValidatedCalibration, ValidatedCalibration)> {
    generated_calibration_template().prop_map(|mut cal| {
        let mut eoi = cal;
        eoi.injection_angle_mode = InjectionAngleMode::EndOfInjection;
        cal.injection_angle_mode = InjectionAngleMode::StartOfInjection;
        (ValidatedCalibration(eoi), ValidatedCalibration(cal))
    })
}

fn valid_input() -> impl Strategy<Value = InputSnapshot> {
    // Shrink target: a running state around the center of the table.
    (
        0u32..100_000,
        0u16..10_000,
        500u16..1500,
        500u16..1500,
        -200i16..1200,
        -200i16..1200,
        500u16..1500,
        10_000u16..15_000,
        prop_oneof![Just(SyncState::Unsynced), Just(SyncState::Synced)],
        any::<bool>(),
        any::<bool>(),
        prop_oneof![
            Just(AfrOverride::None),
            (500u16..3000).prop_map(|value| AfrOverride::Some(AfrX100(value))),
        ],
    )
        .prop_map(
            |(
                t_us,
                rpm,
                map_kpa10,
                load_kpa10,
                clt_c10,
                iat_c10,
                baro_kpa10,
                vbatt_mv,
                sync,
                fuel_cut,
                spark_cut,
                target_afr_override_x100,
            )| {
                InputSnapshot {
                    t_us: Micros(t_us),
                    rpm: Rpm(rpm),
                    map_kpa10: Kpa10(map_kpa10),
                    load_kpa10: Kpa10(load_kpa10),
                    tps_x100: 0,
                    clt_c10: TempC10(clt_c10),
                    iat_c10: TempC10(iat_c10),
                    baro_kpa10: Kpa10(baro_kpa10),
                    vbatt_mv: Millivolts(vbatt_mv),
                    knock_intensity_x100: 0,
                    launch_armed: false,
                    flat_shift_armed: false,
                    sync,
                    fuel_cut,
                    spark_cut,
                    mode: EngineMode::Running,
                    target_afr_override_x100,
                }
            },
        )
}

fn valid_axis_pair() -> impl Strategy<Value = (u16, u16)> {
    // Shrink target: the smallest useful increasing pair.
    (1u16..60_000, 1u16..5000).prop_map(|(start, gap)| (start, start + gap))
}

fn valid_axis_triplet() -> impl Strategy<Value = (u16, u16, u16)> {
    // Shrink target: the smallest useful increasing triplet.
    (1u16..55_000, 1u16..4000, 1u16..4000).prop_map(|(start, gap1, gap2)| {
        let mid = start + gap1;
        (start, mid, mid + gap2)
    })
}

fn invalid_axis_pair() -> impl Strategy<Value = (u16, u16)> {
    // Shrink target: adjacent equal axis entries.
    (0u16..60_000).prop_flat_map(|right| (right..60_000).prop_map(move |left| (left, right)))
}

fn apply_ratio_floor(value: u32, ratio_x1000: u16) -> u32 {
    (value.saturating_mul(ratio_x1000 as u32)) / 1000
}

fn expected_cut_reason(inputs: ArbiterInputs) -> u8 {
    if inputs.safety_latched {
        1
    } else if inputs.rev_limit.hard_rev_fuel_cut {
        2
    } else if inputs.launch_cut {
        3
    } else if inputs.flat_shift.flat_shift_cut {
        4
    } else if inputs.dfco_cut {
        5
    } else if inputs.rev_limit.soft_rev_spark_cut {
        6
    } else if inputs.knock.knock_active {
        7
    } else {
        0
    }
}

proptest! {
    #[test]
    fn prop_lerp_breakpoint_exact(
        x0 in 0u16..60_000,
        gap in 1u16..5000,
        y0 in 0u16..30_000,
        y1 in 0u16..30_000,
    ) {
        let x1 = x0 + gap;
        prop_assert_eq!(lerp_u16(x0, x1, y0, y1, x0), y0);
        prop_assert_eq!(lerp_u16(x0, x1, y0, y1, x1), y1);
    }

    #[test]
    fn prop_bilerp_grid_exact(
        (rpm0, rpm1) in valid_axis_pair(),
        (load0, load1) in valid_axis_pair(),
        c00 in 0u16..20_000,
        c01 in 0u16..20_000,
        c10 in 0u16..20_000,
        c11 in 0u16..20_000,
    ) {
        let table = table_u16_2x2(axis(&[rpm0, rpm1]), axis(&[load0, load1]), c00, c01, c10, c11);
        prop_assert_eq!(bilerp_u16(&table, Rpm(rpm0), Kpa10(load0)), c00);
        prop_assert_eq!(bilerp_u16(&table, Rpm(rpm1), Kpa10(load0)), c01);
        prop_assert_eq!(bilerp_u16(&table, Rpm(rpm0), Kpa10(load1)), c10);
        prop_assert_eq!(bilerp_u16(&table, Rpm(rpm1), Kpa10(load1)), c11);
    }

    #[test]
    fn prop_lower_clip_equivalence(
        (rpm0, rpm1) in valid_axis_pair(),
        (load0, load1) in valid_axis_pair(),
        c00 in 0u16..20_000,
        c01 in 0u16..20_000,
        c10 in 0u16..20_000,
        c11 in 0u16..20_000,
    ) {
        let table = table_u16_2x2(axis(&[rpm0, rpm1]), axis(&[load0, load1]), c00, c01, c10, c11);
        let clipped = bilerp_u16(&table, Rpm(rpm0.saturating_sub(1)), Kpa10(load0.saturating_sub(1)));
        let lower = bilerp_u16(&table, Rpm(rpm0), Kpa10(load0));
        prop_assert_eq!(clipped, lower);
        prop_assert_eq!(clipped, c00);
    }

    #[test]
    fn prop_upper_clip_equivalence(
        (rpm0, rpm1) in valid_axis_pair(),
        (load0, load1) in valid_axis_pair(),
        c00 in 0u16..20_000,
        c01 in 0u16..20_000,
        c10 in 0u16..20_000,
        c11 in 0u16..20_000,
    ) {
        let table = table_u16_2x2(axis(&[rpm0, rpm1]), axis(&[load0, load1]), c00, c01, c10, c11);
        let clipped = bilerp_u16(&table, Rpm(rpm1 + 1), Kpa10(load1 + 1));
        let upper = bilerp_u16(&table, Rpm(rpm1), Kpa10(load1));
        prop_assert_eq!(clipped, upper);
        prop_assert_eq!(clipped, c11);
    }

    #[test]
    fn prop_segment_boundary_policy(
        (a, b, c) in valid_axis_triplet(),
        x0 in 0u16..10_000,
        x1 in 0u16..10_000,
        x2 in 0u16..10_000,
    ) {
        let axis = axis(&[a, b, c]);
        prop_assert_eq!(find_segment(&axis, a), 0);
        prop_assert_eq!(find_segment(&axis, b), 1);
        prop_assert_eq!(find_segment(&axis, c), 1);
        prop_assert_eq!(lerp_u16(a, b, x0, x1, a), x0);
        prop_assert_eq!(lerp_u16(b, c, x1, x2, b), x1);
    }

    #[test]
    fn prop_cell_boundedness(
        rpm0 in 1u16..55_000,
        rpm_gap in 2u16..5000,
        load0 in 1u16..55_000,
        load_gap in 2u16..5000,
        c00 in 0u16..20_000,
        c01 in 0u16..20_000,
        c10 in 0u16..20_000,
        c11 in 0u16..20_000,
    ) {
        let rpm1 = rpm0 + rpm_gap;
        let load1 = load0 + load_gap;
        let table = table_u16_2x2(axis(&[rpm0, rpm1]), axis(&[load0, load1]), c00, c01, c10, c11);
        let rpm = rpm0 + (rpm_gap / 2);
        let load = load0 + (load_gap / 2);
        let out = bilerp_u16(&table, Rpm(rpm), Kpa10(load));
        let lo = c00.min(c01).min(c10).min(c11);
        let hi = c00.max(c01).max(c10).max(c11);
        prop_assert!(out >= lo);
        prop_assert!(out <= hi);
    }

    #[test]
    fn prop_edge_continuity(
        base in 0u16..1000,
        rpm_slope in 0u16..100,
        load_slope in 0u16..100,
        cross_slope in 0u16..25,
    ) {
        let left = table_u16_from_surface(
            axis(&[0, 2]),
            axis(&[0, 2]),
            base as u32,
            rpm_slope as u32,
            load_slope as u32,
            cross_slope as u32,
        );
        let right = table_u16_from_surface(
            axis(&[2, 4]),
            axis(&[0, 2]),
            base as u32,
            rpm_slope as u32,
            load_slope as u32,
            cross_slope as u32,
        );
        let shared_load = Kpa10(1);
        prop_assert_eq!(bilerp_u16(&left, Rpm(2), shared_load), bilerp_u16(&right, Rpm(2), shared_load));
    }

    #[test]
    fn prop_constant_table(
        value in 0u16..30_000,
        rpm in any::<u16>(),
        load in any::<u16>(),
    ) {
        let table = table_u16_2x2(
            axis(&[500, 1000]),
            axis(&[500, 1000]),
            value,
            value,
            value,
            value,
        );
        prop_assert_eq!(bilerp_u16(&table, Rpm(rpm), Kpa10(load)), value);
    }

    #[test]
    fn prop_bilinear_surface(
        base in 0u16..1000,
        rpm_slope in 0u16..100,
        load_slope in 0u16..100,
        cross_slope in 0u16..25,
    ) {
        let table = table_u16_from_surface(
            axis(&[0, 2]),
            axis(&[0, 2]),
            base as u32,
            rpm_slope as u32,
            load_slope as u32,
            cross_slope as u32,
        );
        prop_assert_eq!(
            bilerp_u16(&table, Rpm(1), Kpa10(1)),
            base + rpm_slope + load_slope + cross_slope
        );
    }

    #[test]
    fn prop_additive_offset(
        (rpm0, rpm1) in valid_axis_pair(),
        (load0, load1) in valid_axis_pair(),
        c00 in 0u16..20_000,
        c01 in 0u16..20_000,
        c10 in 0u16..20_000,
        c11 in 0u16..20_000,
        offset in 1u16..15_000,
    ) {
        prop_assume!((c00 as u32 + offset as u32) <= 30_000);
        prop_assume!((c01 as u32 + offset as u32) <= 30_000);
        prop_assume!((c10 as u32 + offset as u32) <= 30_000);
        prop_assume!((c11 as u32 + offset as u32) <= 30_000);
        let table = table_u16_2x2(axis(&[rpm0, rpm1]), axis(&[load0, load1]), c00, c01, c10, c11);
        let shifted = table_u16_2x2(
            axis(&[rpm0, rpm1]),
            axis(&[load0, load1]),
            c00 + offset,
            c01 + offset,
            c10 + offset,
            c11 + offset,
        );
        let rpm = rpm0 + ((rpm1 - rpm0) / 2);
        let load = load0 + ((load1 - load0) / 2);
        let base = bilerp_u16(&table, Rpm(rpm), Kpa10(load));
        let shifted_out = bilerp_u16(&shifted, Rpm(rpm), Kpa10(load));
        prop_assert_eq!(shifted_out, base + offset);
    }

    #[test]
    fn prop_multiplicative_scale(
        base in 0u16..1000,
        rpm_slope in 0u16..100,
        load_slope in 0u16..100,
        cross_slope in 0u16..25,
        scale in 1u16..16,
    ) {
        let table = table_u16_from_surface(
            axis(&[0, 2]),
            axis(&[0, 2]),
            base as u32,
            rpm_slope as u32,
            load_slope as u32,
            cross_slope as u32,
        );
        let mut load_idx = 0usize;
        while load_idx < 2 {
            let mut rpm_idx = 0usize;
            while rpm_idx < 2 {
                prop_assume!((table.values[load_idx][rpm_idx] as u32) * (scale as u32) <= 30_000);
                rpm_idx += 1;
            }
            load_idx += 1;
        }
        let scaled = scale_table(&table, scale);
        let base_out = bilerp_u16(&table, Rpm(1), Kpa10(1));
        let scaled_out = bilerp_u16(&scaled, Rpm(1), Kpa10(1));
        prop_assert_eq!(scaled_out, base_out * scale);
    }

    #[test]
    fn prop_norm7200_range(value in any::<i32>()) {
        let out = norm7200(value);
        prop_assert!(out.0 < 7200);
    }

    #[test]
    fn prop_cyc_distance_symmetry(a in 0u16..7200, b in 0u16..7200) {
        let left = cyc7200_distance(ecu_spec::Degrees10(a), ecu_spec::Degrees10(b));
        let right = cyc7200_distance(ecu_spec::Degrees10(b), ecu_spec::Degrees10(a));
        prop_assert_eq!(left, right);
        prop_assert!(left <= 3600);
    }

    #[test]
    fn prop_duration_monotonicity(
        pw_base in 0u32..10_000,
        pw_extra in 0u32..10_000,
        rpm in 0u16..10_000,
    ) {
        let low = duration_us_to_deg10(PulseWidthUs(pw_base), Rpm(rpm)).0;
        let high = duration_us_to_deg10(PulseWidthUs(pw_base + pw_extra), Rpm(rpm)).0;
        prop_assert!(high >= low);
    }

    #[test]
    fn prop_duration_us_to_deg10_matches_integer_formula(
        dwell_us in 0u32..200_000,
        rpm in 0u16..8_000,
    ) {
        let expected = ((dwell_us as u64 * rpm as u64 * 6u64) / 100_000u64) as u16;
        let actual = duration_us_to_deg10(PulseWidthUs(dwell_us), Rpm(rpm)).0;
        prop_assert_eq!(actual, expected);
    }

    #[test]
    fn prop_target_afr_lookup_matches_bilinear_table(
        c00 in 500u16..3000,
        c01 in 500u16..3000,
        c10 in 500u16..3000,
        c11 in 500u16..3000,
        rpm in 0u16..2000,
        load_kpa10 in 0u16..2000,
    ) {
        let mut cal = canonical_calibration(InjectionAngleMode::EndOfInjection).0;
        cal.afr_target_table = table_u16_2x2(
            axis(&[500, 1000]),
            axis(&[500, 1000]),
            c00,
            c01,
            c10,
            c11,
        );
        let cal = validate_calibration(cal).expect("valid calibration");
        let mut input = InputSnapshot {
            rpm: Rpm(rpm),
            load_kpa10: Kpa10(load_kpa10),
            target_afr_override_x100: AfrOverride::None,
            ..InputSnapshot::default()
        };
        input.map_kpa10 = input.load_kpa10;

        let expected = bilerp_u16(&cal.0.afr_target_table, input.rpm, input.load_kpa10);
        let actual = lookup_target_afr(&cal, input).0;
        prop_assert_eq!(actual, expected);
    }

    #[test]
    fn prop_target_afr_override_clamps_and_takes_precedence(
        table_afr in 500u16..3000,
        override_afr in 0u16..4000,
        rpm in 0u16..2000,
        load_kpa10 in 0u16..2000,
    ) {
        let mut cal = canonical_calibration(InjectionAngleMode::EndOfInjection).0;
        cal.afr_target_table = uniform_u16_table(table_afr);
        let cal = validate_calibration(cal).expect("valid calibration");
        let input = InputSnapshot {
            rpm: Rpm(rpm),
            load_kpa10: Kpa10(load_kpa10),
            target_afr_override_x100: AfrOverride::Some(AfrX100(override_afr)),
            ..InputSnapshot::default()
        };

        let expected = override_afr.clamp(500, 3000);
        prop_assert_eq!(lookup_target_afr(&cal, input).0, expected);
    }

    #[test]
    fn prop_schedule_dwell_duration_matches_dwell_lookup_formula(
        dwell_us in 1u32..20_001,
        rpm in 0u16..8_000,
        load_kpa10 in 0u16..2000,
    ) {
        let mut cal = canonical_calibration(InjectionAngleMode::EndOfInjection).0;
        cal.dwell_table_us.rpm_axis = axis(&[500, 1000]);
        cal.dwell_table_us.load_axis = axis(&[500, 1000]);
        cal.dwell_table_us.values[0][0] = dwell_us;
        cal.dwell_table_us.values[0][1] = dwell_us;
        cal.dwell_table_us.values[1][0] = dwell_us;
        cal.dwell_table_us.values[1][1] = dwell_us;
        let cal = validate_calibration(cal).expect("valid calibration");
        let input = InputSnapshot {
            rpm: Rpm(rpm),
            load_kpa10: Kpa10(load_kpa10),
            sync: SyncState::Synced,
            mode: EngineMode::Running,
            ..InputSnapshot::default()
        };
        let schedule = schedule_all_cylinders(
            &cal,
            input,
            FuelOutput {
                pw_corr_us: PulseWidthUs(1000),
            },
        );

        let expected = ((dwell_us as u64 * rpm as u64 * 6u64) / 100_000u64) as u16;
        prop_assert_eq!(schedule.dwell_us, PulseWidthUs(dwell_us));
        prop_assert_eq!(schedule.dwell_duration_deg10.0, expected);
    }

    #[test]
    fn prop_step_determinism(
        cal in valid_calibration(InjectionAngleMode::EndOfInjection),
        input in valid_input(),
        state in valid_state(),
    ) {
        let first = step(&cal, input, &state);
        let second = step(&cal, input, &state);
        prop_assert_eq!(first, second);
    }

    #[test]
    fn prop_step_idempotent(
        cal in valid_calibration(InjectionAngleMode::EndOfInjection),
        input in valid_input(),
        state in valid_state(),
    ) {
        let first = step(&cal, input, &state);
        let second = step(&cal, input, &state);
        prop_assert_eq!(first.output, second.output);
        prop_assert_eq!(first.next_state, second.next_state);
    }

    #[test]
    fn prop_state_monotonic_rpm(
        input in valid_input(),
        rpm_bump in 1u16..2000,
    ) {
        prop_assume!(input.rpm.0 >= 600);
        prop_assume!(input.rpm.0 <= 8000);
        prop_assume!(input.rpm.0 <= 8000 - rpm_bump);

        let mut cal = canonical_calibration(InjectionAngleMode::EndOfInjection);
        cal.0.ve_table = table_u16_2x2(
            axis(&[600, 8000]),
            axis(&[500, 1000]),
            1000,
            1000,
            2000,
            2000,
        );
        let state = LogicalState::default();
        let higher_rpm = InputSnapshot {
            rpm: Rpm(input.rpm.0 + rpm_bump),
            ..input
        };
        let lower = step(&cal, input, &state);
        let higher = step(&cal, higher_rpm, &state);
        prop_assert!(higher.output.ve_pct_x100.0 >= lower.output.ve_pct_x100.0);
        prop_assert_eq!(higher.next_state.math.last_valid_map_kpa10, higher_rpm.map_kpa10);
        prop_assert_eq!(higher.next_state.math.last_valid_load_kpa10, higher_rpm.load_kpa10);
    }

    #[test]
    fn prop_fuel_cut_suppression(
        cal in valid_calibration(InjectionAngleMode::EndOfInjection),
        input in valid_input(),
        state in valid_state(),
    ) {
        let mut cut_input = input;
        cut_input.fuel_cut = true;
        cut_input.spark_cut = false;
        cut_input.sync = SyncState::Synced;
        let result = step(&cal, cut_input, &state);
        prop_assert_eq!(result.output.pw_corr_us.0, 0);
        prop_assert_eq!(result.output.cut_reason_code, 1);
        prop_assert!(result.output.fuel_cut);
        prop_assert!(result.output.spark_cut);
        prop_assert_eq!(result.output.events.len, 0);
        prop_assert_eq!(result.output.diagnostic, DiagnosticCode::FuelCutActive);
    }

    #[test]
    fn prop_spark_cut_suppression(
        cal in valid_calibration(InjectionAngleMode::EndOfInjection),
        input in valid_input(),
        state in valid_state(),
    ) {
        let mut cut_input = input;
        cut_input.fuel_cut = false;
        cut_input.spark_cut = true;
        cut_input.sync = SyncState::Synced;
        let result = step(&cal, cut_input, &state);
        prop_assert_eq!(result.output.cut_reason_code, 1);
        prop_assert!(result.output.fuel_cut);
        prop_assert!(result.output.spark_cut);
        prop_assert_eq!(result.output.events.len, 0);
        prop_assert_eq!(result.output.diagnostic, DiagnosticCode::FuelCutActive);
    }

    #[test]
    fn prop_injection_angle_mode(
        cal_pair in valid_calibration_pair(),
        input in valid_input(),
        state in valid_state(),
    ) {
        let (eoi_cal, soi_cal) = cal_pair;
        let eoi = step(&eoi_cal, input, &state);
        let soi = step(&soi_cal, input, &state);
        prop_assert_eq!(eoi.output.spark_deg10, soi.output.spark_deg10);
        prop_assert_eq!(eoi.output.dwell_start_deg10, soi.output.dwell_start_deg10);
        prop_assert_eq!(eoi.output.eoi_deg10, soi.output.soi_deg10);
    }

    #[test]
    fn prop_crank_domain_validity(
        cal in valid_calibration(InjectionAngleMode::EndOfInjection),
        input in valid_input(),
    ) {
        let result = step(&cal, input, &LogicalState::default());
        let count = cal.0.cylinder_phase_deg10.count as usize;
        let mut idx = 0usize;
        while idx < count {
            let soi = result.output.soi_deg10.values[idx];
            let eoi = result.output.eoi_deg10.values[idx];
            let spark = result.output.spark_deg10.values[idx];
            let dwell_start = result.output.dwell_start_deg10.values[idx];
            prop_assert!(soi < 7200);
            prop_assert!(eoi < 7200);
            prop_assert!(spark < 7200);
            prop_assert!(dwell_start < 7200);
            idx += 1;
        }
    }

    #[test]
    fn prop_invalid_axes_are_rejected(
        cal in valid_calibration(InjectionAngleMode::EndOfInjection),
        (left, right) in invalid_axis_pair(),
    ) {
        let mut cal = cal;
        cal.0.ve_table.rpm_axis = axis(&[left, right]);
        let result = validate_calibration(cal.0);
        prop_assert!(matches!(result, Err(ValidationError::AxisNotStrictlyIncreasing)));
    }

    #[test]
    fn prop_deadtime_lookup_matches_bilinear_table(
        c00 in 0u16..20_000,
        c01 in 0u16..20_000,
        c10 in 0u16..20_000,
        c11 in 0u16..20_000,
        vbat_mv in 0u16..18_000,
        pressure_kpa10 in 0u16..3000,
    ) {
        let table = table_u16_2x2(
            axis(&[8000, 16_000]),
            axis(&[1000, 3000]),
            c00,
            c01,
            c10,
            c11,
        );
        let expected = bilerp_u16(&table, Rpm(vbat_mv), Kpa10(pressure_kpa10)) as u32;
        let actual = deadtime_lookup(&table, Millivolts(vbat_mv), Kpa10(pressure_kpa10)).0;
        prop_assert_eq!(actual, expected);
    }

    #[test]
    fn prop_deadtime_monotone_voltage(
        pressure_kpa10 in 1000u16..3000,
        vbat0 in 0u16..17_500,
        dv in 0u16..500,
    ) {
        let vbat1 = vbat0.saturating_add(dv).min(18_000);
        let table = table_u16_2x2(
            axis(&[1000, 13000]),
            axis(&[1000, 3000]),
            1400,
            900,
            1200,
            800,
        );
        let lo = deadtime_lookup(&table, Millivolts(vbat0), Kpa10(pressure_kpa10)).0;
        let hi = deadtime_lookup(&table, Millivolts(vbat1), Kpa10(pressure_kpa10)).0;
        prop_assert!(hi <= lo);
        prop_assert!((800..=1400).contains(&lo));
        prop_assert!((800..=1400).contains(&hi));
    }

    #[test]
    fn prop_clt_correction_lookup_matches_linear_curve(
        cold_corr in 500u16..2000,
        hot_corr in 500u16..2000,
        clt_c10 in -500i16..1500,
    ) {
        let mut cal = canonical_calibration(InjectionAngleMode::EndOfInjection).0;
        cal.clt_corr_curve.axis = axis(&[0, 1000]);
        cal.clt_corr_curve.values[0] = cold_corr;
        cal.clt_corr_curve.values[1] = hot_corr;
        let cal = validate_calibration(cal).expect("valid calibration");
        let input = InputSnapshot {
            clt_c10: TempC10(clt_c10),
            ..InputSnapshot::default()
        };
        let curve_input = if clt_c10 < 0 { 0 } else { clt_c10 as u16 };
        let expected = lerp_u16(0, 1000, cold_corr, hot_corr, curve_input);
        let actual = ecu_spec::lookup_clt_corr_x1000(&cal, input).0;
        prop_assert_eq!(actual, expected);
    }

    #[test]
    fn prop_base_pulse_width_matches_required_fuel_times_ve(
        required_fuel_us in 1u32..100_000,
        ve_x100 in 0u16..30_000,
    ) {
        let mut cal = canonical_calibration(InjectionAngleMode::EndOfInjection).0;
        cal.required_fuel_us = required_fuel_us;
        let cal = validate_calibration(cal).expect("valid calibration");
        let expected = (required_fuel_us as u64 * ve_x100 as u64 / 10_000u64) as u32;
        let actual = ecu_spec::compute_pw_base_us(&cal, ecu_spec::VePctX100(ve_x100)).0;
        prop_assert_eq!(actual, expected);
    }

    #[test]
    fn prop_spark_timing_lookup_matches_bilinear_table(
        c00 in -7200i16..7200,
        c01 in -7200i16..7200,
        c10 in -7200i16..7200,
        c11 in -7200i16..7200,
        rpm in 0u16..2000,
        load_kpa10 in 0u16..2000,
    ) {
        let mut cal = canonical_calibration(InjectionAngleMode::EndOfInjection).0;
        cal.spark_advance_table_deg10 =
            table_i16_2x2(axis(&[500, 1000]), axis(&[500, 1000]), c00, c01, c10, c11);
        let cal = validate_calibration(cal).expect("valid calibration");
        let input = InputSnapshot {
            rpm: Rpm(rpm),
            load_kpa10: Kpa10(load_kpa10),
            ..InputSnapshot::default()
        };
        let expected = ecu_spec::bilerp_i16(&cal.0.spark_advance_table_deg10, input.rpm, input.load_kpa10);
        let actual = ecu_spec::compute_spark_advance_deg10(&cal, input).0;
        prop_assert_eq!(actual, expected);
    }

    #[test]
    fn prop_idle_timing_disabled_is_zero_noop(
        rpm in 0u16..3000,
        tps_x100 in 0u16..1000,
        acc in -1000i32..1000,
    ) {
        let cal = canonical_calibration(InjectionAngleMode::EndOfInjection);
        let input = InputSnapshot {
            rpm: Rpm(rpm),
            tps_x100,
            mode: EngineMode::Running,
            sync: SyncState::Synced,
            ..InputSnapshot::default()
        };
        let state = LogicalState {
            idle_timing_integrator_state: PiIntegratorState {
                acc,
                ..PiIntegratorState::zero()
            },
            ..LogicalState::default()
        };
        let result = idle_timing_step(&cal, input, &state);
        prop_assert_eq!(result.trim_deg10, 0);
        prop_assert!(!result.active);
        prop_assert_eq!(result.integrator_state.acc, acc);
    }

    #[test]
    fn prop_idle_advance_curve_matches_piecewise_linear_lookup(
        low in -300i16..300,
        high in -300i16..300,
        rpm in 0u16..2000,
    ) {
        let mut cal = canonical_calibration(InjectionAngleMode::EndOfInjection).0;
        cal.idle_advance_curve_deg10 = signed_curve_2x2(&[500, 1000], low, high);
        let cal = validate_calibration(cal).expect("valid calibration");
        let rpm_clipped = rpm.clamp(500, 1000);
        let expected = lerp_i16(500, 1000, low, high, rpm_clipped);
        let actual = lookup_idle_advance_deg10(&cal, Rpm(rpm)).0;
        prop_assert_eq!(actual, expected);
    }

    #[test]
    fn prop_idle_timing_pid_trim_is_clamped(
        base in -200i16..200,
        target in 700u16..1300,
        rpm in 400u16..1600,
        kp in 0u16..500,
        ki in 0u16..500,
        acc in -500i32..500,
        min_trim in -500i16..0,
        max_trim in 0i16..500,
    ) {
        prop_assume!(min_trim <= max_trim);
        let mut cal = canonical_calibration(InjectionAngleMode::EndOfInjection).0;
        cal.idle_target_rpm = Rpm(target);
        cal.idle_timing_enabled = true;
        cal.idle_timing_pid_enabled = true;
        cal.idle_timing_rpm_max = Rpm(2000);
        cal.idle_timing_tps_max_x100 = 500;
        cal.idle_advance_curve_deg10 = signed_curve_2x2(&[500, 1000], base, base);
        cal.idle_timing_kp_x1000 = kp;
        cal.idle_timing_ki_x1000 = ki;
        cal.idle_timing_min_trim_deg10 = min_trim;
        cal.idle_timing_max_trim_deg10 = max_trim;
        let cal = validate_calibration(cal).expect("valid calibration");
        let input = InputSnapshot {
            rpm: Rpm(rpm),
            tps_x100: 0,
            mode: EngineMode::Running,
            sync: SyncState::Synced,
            ..InputSnapshot::default()
        };
        let state = LogicalState {
            idle_timing_integrator_state: PiIntegratorState {
                acc,
                ..PiIntegratorState::zero()
            },
            ..LogicalState::default()
        };
        let result = idle_timing_step_with_base(&cal, input, &state, SignedDegrees10(base));
        prop_assert!(result.active);
        prop_assert!(result.trim_deg10 >= min_trim);
        prop_assert!(result.trim_deg10 <= max_trim);
        prop_assert!(result.integrator_state.acc >= -7200);
        prop_assert!(result.integrator_state.acc <= 7200);
    }

    #[test]
    fn prop_idle_timing_gates_off_when_not_running_synced_or_idle(
        rpm in 1u16..3000,
        tps_x100 in 0u16..1000,
        fuel_cut in any::<bool>(),
        spark_cut in any::<bool>(),
    ) {
        let mut cal = canonical_calibration(InjectionAngleMode::EndOfInjection).0;
        cal.idle_timing_enabled = true;
        cal.idle_timing_pid_enabled = true;
        cal.idle_timing_rpm_max = Rpm(1200);
        cal.idle_timing_tps_max_x100 = 200;
        cal.idle_advance_curve_deg10 = signed_curve_2x2(&[500, 1000], 100, 100);
        cal.idle_timing_min_trim_deg10 = -300;
        cal.idle_timing_max_trim_deg10 = 300;
        let cal = validate_calibration(cal).expect("valid calibration");
        let input = InputSnapshot {
            rpm: Rpm(rpm),
            tps_x100,
            mode: EngineMode::Running,
            sync: SyncState::Synced,
            fuel_cut,
            spark_cut,
            ..InputSnapshot::default()
        };
        let result = idle_timing_step(&cal, input, &LogicalState::default());
        let expected_active = rpm <= 1200 && tps_x100 <= 200 && !fuel_cut && !spark_cut;
        prop_assert_eq!(result.active, expected_active);
        if !expected_active {
            prop_assert_eq!(result.trim_deg10, 0);
        }
    }

    #[test]
    fn prop_idle_advance_replaces_and_blends_running_advance(
        idle_advance in -300i16..300,
        running_advance in -300i16..300,
        tps_x100 in 0u16..400,
        rpm in 500u16..1000,
    ) {
        let mut cal = canonical_calibration(InjectionAngleMode::EndOfInjection).0;
        cal.idle_timing_enabled = true;
        cal.idle_timing_rpm_max = Rpm(1200);
        cal.idle_timing_tps_max_x100 = 200;
        cal.idle_advance_curve_deg10 = signed_curve_2x2(&[500, 1000], idle_advance, idle_advance);
        let cal = validate_calibration(cal).expect("valid calibration");
        let input = InputSnapshot {
            rpm: Rpm(rpm),
            tps_x100,
            mode: EngineMode::Running,
            sync: SyncState::Synced,
            ..InputSnapshot::default()
        };
        let selected =
            select_idle_or_running_advance_deg10(&cal, input, SignedDegrees10(running_advance)).0;
        let expected = if tps_x100 <= 100 {
            idle_advance
        } else if tps_x100 >= 200 {
            running_advance
        } else {
            let delta = running_advance as i32 - idle_advance as i32;
            (idle_advance as i32 + delta * (tps_x100 as i32 - 100) / 100) as i16
        };
        prop_assert_eq!(selected, expected);
    }

    #[test]
    fn prop_idle_timing_trim_includes_base_replacement_and_temperature_corrections(
        running_advance in -300i16..300,
        idle_advance in -300i16..300,
        clt_corr in -200i16..200,
        iat_corr in -200i16..200,
    ) {
        let mut cal = canonical_calibration(InjectionAngleMode::EndOfInjection).0;
        cal.idle_timing_enabled = true;
        cal.idle_timing_pid_enabled = false;
        cal.idle_timing_rpm_max = Rpm(1200);
        cal.idle_timing_tps_max_x100 = 200;
        cal.idle_advance_curve_deg10 = signed_curve_2x2(&[500, 1000], idle_advance, idle_advance);
        cal.clt_timing_corr_curve_deg10 = signed_curve_2x2(&[0, 1000], clt_corr, clt_corr);
        cal.iat_timing_corr_curve_deg10 = signed_curve_2x2(&[0, 1000], iat_corr, iat_corr);
        cal.idle_timing_min_trim_deg10 = -7200;
        cal.idle_timing_max_trim_deg10 = 7200;
        let cal = validate_calibration(cal).expect("valid calibration");
        let input = InputSnapshot {
            rpm: Rpm(900),
            tps_x100: 0,
            clt_c10: TempC10(800),
            iat_c10: TempC10(250),
            mode: EngineMode::Running,
            sync: SyncState::Synced,
            ..InputSnapshot::default()
        };
        let result =
            idle_timing_step_with_base(&cal, input, &LogicalState::default(), SignedDegrees10(running_advance));
        let expected = idle_advance as i32 - running_advance as i32 + clt_corr as i32 + iat_corr as i32;
        prop_assert_eq!(result.trim_deg10, expected as i16);
    }

    #[test]
    fn prop_idle_timing_trim_shifts_spark_and_dwell_start_angles(
        base_advance in -300i16..300,
        trim in -200i16..200,
        phase in 0u16..7200,
        rpm in 500u16..2000,
        dwell_us in 1u32..10_000,
    ) {
        let mut cal = canonical_calibration(InjectionAngleMode::EndOfInjection).0;
        cal.spark_advance_table_deg10 = uniform_i16_table(base_advance);
        cal.dwell_table_us = uniform_u32_table(dwell_us);
        cal.cylinder_phase_deg10.count = 1;
        cal.cylinder_phase_deg10.values[0] = phase;
        let cal = validate_calibration(cal).expect("valid calibration");
        let input = InputSnapshot {
            rpm: Rpm(rpm),
            load_kpa10: Kpa10(500),
            sync: SyncState::Synced,
            mode: EngineMode::Running,
            ..InputSnapshot::default()
        };
        let schedule = schedule_all_cylinders_with_advance_trim(
            &cal,
            input,
            FuelOutput {
                pw_corr_us: PulseWidthUs(1000),
            },
            SignedDegrees10(trim),
        );
        let dwell_deg10 = ((dwell_us as u64 * rpm as u64 * 6u64) / 100_000u64) as u16;
        let spark = norm7200(phase as i32 - base_advance as i32 - trim as i32);
        let dwell_start = norm7200(spark.0 as i32 - dwell_deg10 as i32);
        prop_assert_eq!(schedule.spark_advance_deg10, SignedDegrees10(base_advance.saturating_add(trim)));
        prop_assert_eq!(schedule.spark_deg10.values[0], spark.0);
        prop_assert_eq!(schedule.dwell_start_deg10.values[0], dwell_start.0);
    }

    #[test]
    fn prop_vbat_correction_bounds(vbat_mv in 0u16..18_000) {
        let curve = ecu_spec::Curve16 {
            axis: axis(&[8000, 12000, 16000]),
            values: [1200, 1000, 900, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        };
        let corr = vbat_correction(&curve, Millivolts(vbat_mv)).0;
        prop_assert!((900..=1200).contains(&corr));
    }

    #[test]
    fn prop_running_engine_low_voltage_is_raw_observable_not_nominal_clamped(
        vbat_mv in 9_000u16..=11_000,
    ) {
        let input = SensorPlausibilityInput {
            t_us: Micros(500_000),
            rpm: Rpm(2_000),
            clt_c10: TempC10(800),
            iat_c10: TempC10(250),
            map_kpa10: 700,
            tps_x100: 2_000,
            maf_x100: 12_000,
            o2_afr_x100: 1_470,
            knock_intensity_x100: 100,
            baro_kpa10: 1_000,
            vbat_mv,
        };
        let result = sensor_plausibility_step(input, SensorPlausibilityState::default());
        prop_assert_eq!(result.next_state.last_vbat_mv, vbat_mv);
        prop_assert_ne!(result.next_state.last_vbat_mv, 12_000);
        prop_assert_ne!(result.next_state.last_vbat_mv, 13_500);
        prop_assert_eq!(result.diagnostic, DiagnosticCode::None);
    }

    #[test]
    fn prop_oracle_voltage_plausibility_uses_raw_voltage_before_slew(
        raw_vbat_mv in 9_000u16..=11_000,
        previous_vbat_mv in 12_000u16..=14_500,
        dt_us in 1_000u32..=20_000,
    ) {
        let cal = canonical_calibration(InjectionAngleMode::EndOfInjection);
        let input = InputSnapshot {
            t_us: Micros(10_000u32.saturating_add(dt_us)),
            rpm: Rpm(2_000),
            load_kpa10: Kpa10(700),
            map_kpa10: Kpa10(700),
            tps_x100: 2_000,
            clt_c10: TempC10(800),
            iat_c10: TempC10(250),
            baro_kpa10: Kpa10(1_000),
            vbatt_mv: Millivolts(raw_vbat_mv),
            mode: EngineMode::Running,
            sync: SyncState::Synced,
            ..InputSnapshot::default()
        };
        let state = LogicalState {
            sensor_slew_state: SensorSlewState {
                initialized: true,
                last_t_us: Micros(10_000),
                clt_c10: input.clt_c10,
                iat_c10: input.iat_c10,
                map_kpa10: input.map_kpa10.0,
                tps_x100: input.tps_x100,
                maf_x100: 0,
                o2_afr_x100: 1470,
                knock_intensity_x100: input.knock_intensity_x100,
                baro_kpa10: input.baro_kpa10.0,
                vbat_mv: previous_vbat_mv,
                ..SensorSlewState::default()
            },
            ..LogicalState::default()
        };

        let result = step(&cal, input, &state);
        prop_assert_eq!(
            result.next_state.sensor_plausibility_state.last_vbat_mv,
            raw_vbat_mv
        );
        prop_assert_ne!(
            result.next_state.sensor_plausibility_state.last_vbat_mv,
            previous_vbat_mv
        );
        prop_assert!(result.next_state.sensor_slew_state.vbat_mv != raw_vbat_mv || previous_vbat_mv == raw_vbat_mv);
    }

    #[test]
    fn prop_low_voltage_deadtime_uses_measured_voltage_not_11v_floor(
        vbat_mv in 9_000u16..=10_999,
        pressure_kpa10 in 1_000u16..=3_000,
    ) {
        let table = table_u16_2x2(
            axis(&[9000, 11000]),
            axis(&[1000, 3000]),
            1700,
            1200,
            1500,
            1000,
        );
        let measured = deadtime_lookup(&table, Millivolts(vbat_mv), Kpa10(pressure_kpa10)).0;
        let incorrectly_clamped =
            deadtime_lookup(&table, Millivolts(11_000), Kpa10(pressure_kpa10)).0;
        let expected = bilerp_u16(&table, Rpm(vbat_mv), Kpa10(pressure_kpa10)) as u32;
        prop_assert_eq!(measured, expected);
        prop_assert!(measured >= incorrectly_clamped);
        if vbat_mv < 10_950 {
            prop_assert!(measured > incorrectly_clamped);
        }
    }

    #[test]
    fn prop_low_voltage_vbat_correction_uses_measured_voltage_not_nominal_floor(
        vbat_mv in 9_000u16..=10_999,
    ) {
        let curve = ecu_spec::Curve16 {
            axis: axis(&[9000, 11000, 13_500]),
            values: [1300, 1100, 1000, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        };
        let measured = vbat_correction(&curve, Millivolts(vbat_mv)).0;
        let incorrectly_clamped_11v = vbat_correction(&curve, Millivolts(11_000)).0;
        let nominal = vbat_correction(&curve, Millivolts(13_500)).0;
        let expected = lerp_u16(9000, 11000, 1300, 1100, vbat_mv);
        prop_assert_eq!(measured, expected);
        prop_assert!(measured >= incorrectly_clamped_11v);
        prop_assert!(measured > nominal);
        if vbat_mv < 10_950 {
            prop_assert!(measured > incorrectly_clamped_11v);
        }
    }

    #[test]
    fn prop_baro_correction_bounds(baro_kpa10 in 0u16..3000) {
        let curve = ecu_spec::Curve16 {
            axis: axis(&[700, 850, 1000]),
            values: [700, 850, 1000, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        };
        let corr = baro_correction(&curve, Kpa10(baro_kpa10)).0;
        prop_assert!((700..=1000).contains(&corr));
    }

    #[test]
    fn prop_enrichment_ordering(
        pw_air in 1u32..20_000,
        cranking in 500u16..2000,
        afterstart in 500u16..2000,
        warmup in 500u16..2000,
        clt in 500u16..2000,
        iat in 500u16..2000,
        baro in 500u16..2000,
        vbat in 500u16..2000,
        afr in 500u16..2000,
        lambda in 500u16..2000,
        deadtime in 0u32..2000,
        ae_pulse in 0u32..2000,
    ) {
        let mut cal = canonical_calibration(InjectionAngleMode::EndOfInjection);
        cal.0.pw_max_us = 200_000;
        let mut invalid = cal.0;
        invalid.ve_table.rpm_axis = axis(&[1000, 1000]);
        prop_assume!(validate_calibration(invalid).is_err());

        let parts = ecu_spec::FuelParts {
            pw_air_us: PulseWidthUs(pw_air),
            ae_pulse_us: PulseWidthUs(ae_pulse),
            deadtime_us: PulseWidthUs(deadtime),
            clt_corr_x1000: RatioX1000(clt),
            iat_corr_x1000: RatioX1000(iat),
            baro_corr_x1000: RatioX1000(baro),
            vbat_corr_x1000: RatioX1000(vbat),
            cranking_corr_x1000: RatioX1000(cranking),
            afterstart_corr_x1000: RatioX1000(afterstart),
            warmup_corr_x1000: RatioX1000(warmup),
            afr_corr_x1000: RatioX1000(afr),
            lambda_corr_x1000: RatioX1000(lambda),
        };
        let got = ecu_spec::compute_pw_corr_us(
            &cal,
            &LogicalState::default(),
            InputSnapshot::default(),
            parts,
        )
        .0;

        let mut expected = pw_air;
        expected = apply_ratio_floor(expected, cranking);
        expected = apply_ratio_floor(expected, afterstart);
        expected = apply_ratio_floor(expected, warmup);
        expected = apply_ratio_floor(expected, clt);
        expected = apply_ratio_floor(expected, iat);
        expected = apply_ratio_floor(expected, baro);
        expected = apply_ratio_floor(expected, vbat);
        expected = apply_ratio_floor(expected, afr);
        expected = apply_ratio_floor(expected, lambda);
        expected = expected.saturating_add(ae_pulse);
        expected = expected.saturating_add(deadtime);
        expected = expected.min(cal.0.pw_max_us);
        prop_assert_eq!(got, expected);
    }

    #[test]
    fn prop_arbiter_priority_total(
        safety_latched in any::<bool>(),
        hard_rev in any::<bool>(),
        launch_cut in any::<bool>(),
        flat_shift_cut in any::<bool>(),
        dfco_cut in any::<bool>(),
        soft_rev in any::<bool>(),
        knock_active in any::<bool>(),
    ) {
        let input = ArbiterInputs {
            safety_latched,
            dfco_cut,
            rev_limit: RevLimitResult {
                soft_rev_spark_cut: soft_rev,
                hard_rev_fuel_cut: hard_rev,
                soft_retard_deg10: 0,
                soft_active: soft_rev,
                hard_active: hard_rev,
            },
            launch_cut,
            flat_shift: FlatShiftResult {
                flat_shift_cut,
                active: flat_shift_cut,
                cut_cycle_count: 0,
            },
            knock: KnockResult {
                advance_trim_deg10: 0,
                knock_active,
                next_state: KnockState::default(),
            },
        };
        let out = arbiter_step(input);
        prop_assert_eq!(out.cut_reason_code, expected_cut_reason(input));
    }

    #[test]
    fn prop_pi_clamp_idempotent(
        rpm in 0u16..4000,
        target in 0u16..4000,
        kp in 0u16..2000,
        ki in 0u16..2000,
        lambda_error in -2000i32..2000,
        acc0 in -2000i32..2000,
        freeze_gate in any::<bool>(),
    ) {
        let mut cal = canonical_calibration(InjectionAngleMode::EndOfInjection);
        cal.0.idle_target_rpm = Rpm(target);
        cal.0.idle_kp_x1000 = kp;
        cal.0.idle_ki_x1000 = ki;
        cal.0.lambda_kp_x1000 = kp;
        cal.0.lambda_ki_x1000 = ki;
        let mut input = InputSnapshot {
            rpm: Rpm(rpm),
            clt_c10: TempC10(800),
            ..InputSnapshot::default()
        };
        if freeze_gate {
            input.clt_c10 = TempC10(600);
        }
        let state = LogicalState {
            ae: AeState {
                active: freeze_gate,
                ..AeState::default()
            },
            idle_integrator_state: PiIntegratorState {
                acc: acc0,
                ..PiIntegratorState::zero()
            },
            lambda_integrator_state: PiIntegratorState {
                acc: acc0,
                ..PiIntegratorState::zero()
            },
            ..LogicalState::default()
        };
        let idle1 = idle_step(&cal, input, &state);
        let state2 = LogicalState {
            idle_integrator_state: idle1.integrator_state,
            ..state
        };
        let idle2 = idle_step(&cal, input, &state2);
        prop_assert!((-2000..=2000).contains(&idle1.integrator_state.acc));
        prop_assert!((-2000..=2000).contains(&idle2.integrator_state.acc));
        if idle1.integrator_state.frozen {
            prop_assert_eq!(idle1.integrator_state.acc, idle2.integrator_state.acc);
        }

        let lam1 = lambda_step_with_error(&cal, input, &state, freeze_gate, lambda_error);
        let state3 = LogicalState {
            lambda_integrator_state: lam1.integrator_state,
            ..state
        };
        let lam2 = lambda_step_with_error(&cal, input, &state3, freeze_gate, lambda_error);
        prop_assert!((-2000..=2000).contains(&lam1.integrator_state.acc));
        prop_assert!((-2000..=2000).contains(&lam2.integrator_state.acc));
        if lam1.integrator_state.frozen {
            prop_assert_eq!(lam1.integrator_state.acc, lam2.integrator_state.acc);
        }
    }

    #[test]
    fn prop_sensor_curve_clamp(
        adc_counts in any::<u16>(),
        was_rich in any::<bool>(),
        tps_min in 0u16..4095,
        tps_max in 0u16..4095,
    ) {
        prop_assume!(tps_min < tps_max);
        let mut cal = canonical_calibration(InjectionAngleMode::EndOfInjection).0;
        cal.tps_adc_min_counts = tps_min;
        cal.tps_adc_max_counts = tps_max;
        cal.o2_sensor_mode = O2SensorMode::WidebandLinear;

        let clt = clt_from_counts(adc_counts).0;
        let iat = iat_from_counts(adc_counts).0;
        let map = map_from_counts(adc_counts).0;
        let tps = tps_from_counts(&cal, adc_counts);
        let maf = maf_from_counts(adc_counts);
        let o2 = o2_from_counts(&cal, adc_counts, was_rich).afr_x100.0;
        let knock = knock_from_window(adc_counts);
        let baro = baro_from_counts(adc_counts).0;
        let vbat = vbat_from_counts(adc_counts).0;

        prop_assert!((-400..=1200).contains(&clt));
        prop_assert!((-400..=1100).contains(&iat));
        prop_assert!((100..=3000).contains(&map));
        prop_assert!(tps <= 10_000);
        prop_assert!(maf <= 9300);
        prop_assert!((500..=3000).contains(&o2));
        prop_assert!(knock <= 10_000);
        prop_assert!((500..=1200).contains(&baro));
        prop_assert!((6000..=18_000).contains(&vbat));
    }

    #[test]
    fn prop_sensor_slew_limit(
        dt_us in 1000u32..250_000,
        map in any::<u16>(),
        tps in any::<u16>(),
        baro in any::<u16>(),
        vbat in any::<u16>(),
    ) {
        let prev = SensorSlewState {
            initialized: true,
            last_t_us: Micros(10_000),
            clt_c10: TempC10(100),
            iat_c10: TempC10(50),
            map_kpa10: 1200,
            tps_x100: 2500,
            maf_x100: 3000,
            o2_afr_x100: 1470,
            knock_intensity_x100: 250,
            baro_kpa10: 1000,
            vbat_mv: 12_000,
            ..SensorSlewState::default()
        };
        let input = SensorSlewInput {
            t_us: Micros(prev.last_t_us.0.saturating_add(dt_us)),
            clt_c10: TempC10(1200),
            iat_c10: TempC10(-300),
            map_kpa10: map,
            tps_x100: tps,
            maf_x100: 60_000,
            o2_afr_x100: 3000,
            knock_intensity_x100: 10_000,
            baro_kpa10: baro,
            vbat_mv: vbat,
        };
        let out = sensor_slew_step(input, prev);
        let max_map = (2000u64 * dt_us as u64) / 1_000_000;
        let max_tps = (50_000u64 * dt_us as u64) / 1_000_000;
        let max_baro = (50u64 * dt_us as u64) / 1_000_000;
        let max_vbat = (5000u64 * dt_us as u64) / 1_000_000;
        prop_assert!((out.limited.map_kpa10 as i32 - prev.map_kpa10 as i32).unsigned_abs() as u64 <= max_map);
        prop_assert!((out.limited.tps_x100 as i32 - prev.tps_x100 as i32).unsigned_abs() as u64 <= max_tps);
        prop_assert!((out.limited.baro_kpa10 as i32 - prev.baro_kpa10 as i32).unsigned_abs() as u64 <= max_baro);
        prop_assert!((out.limited.vbat_mv as i32 - prev.vbat_mv as i32).unsigned_abs() as u64 <= max_vbat);
    }

    #[test]
    fn prop_trigger_sync_totality(
        start_sync in prop_oneof![
            Just(TriggerSyncState::NoSync),
            Just(TriggerSyncState::PreSync),
            Just(TriggerSyncState::Synced),
            Just(TriggerSyncState::SyncLoss),
        ],
        dt0 in 0u32..500_000,
        dt1 in 0u32..500_000,
        dt2 in 0u32..500_000,
        dt3 in 0u32..500_000,
    ) {
        let stream = [dt0, dt1, dt2, dt3];
        prop_assume!(stream.iter().any(|dt| *dt > 0));

        let mut state = TriggerState {
            sync_state: start_sync,
            trigger_state: start_sync,
            ..TriggerState::default()
        };
        let mut ts = 0u32;
        for dt in stream {
            ts = ts.saturating_add(dt);
            let step = trigger_60_2_step(state, Micros(ts));
            prop_assert!(step.angle_deg10.0 < 7200);
            prop_assert!(step.rpm_estimate.0 <= 20_000);
            state = step.state;
        }
    }

    #[test]
    fn prop_cam_none_passthrough(
        phase in prop_oneof![
            Just(CamPhase::Unknown),
            Just(CamPhase::PhaseA),
            Just(CamPhase::PhaseB),
        ],
        sync_state in prop_oneof![
            Just(TriggerSyncState::NoSync),
            Just(TriggerSyncState::PreSync),
            Just(TriggerSyncState::Synced),
            Just(TriggerSyncState::SyncLoss),
        ],
        tooth_index in any::<u8>(),
    ) {
        let state = CamPhaseState { phase };
        let trigger = TriggerState {
            sync_state,
            trigger_state: sync_state,
            tooth_index,
            ..TriggerState::default()
        };
        let out = cam_phase_step(state, trigger, None::<CamTooth>);
        prop_assert_eq!(out.state, state);
        prop_assert_eq!(out.phase, state.phase);
        prop_assert!(!out.edge_applied);
    }
}
