use ecu_spec::{
    ae_step_with_deltas, arbiter_step, baro_correction, bilerp_u16, cranking_corr_x1000,
    deadtime_lookup, dfco_step, find_segment, flat_shift_step, idle_step, knock_step,
    lambda_step_with_error, launch_step, lerp_i16, lerp_u16, rev_limit_step, step, vbat_correction,
    AeCurves, AfrOverride, ArbiterInputs, Axis16, Calibration, Curve16, CylinderArrayU16,
    DiagnosticCode, EngineMode, EventKind, FlatShiftResult, FuelModel, InjectionAngleMode,
    InputSnapshot, KnockResult, Kpa10, LogicalState, Millivolts, ObservableOutput, PwMaxPolicy,
    RevLimitResult, Rpm, SignedCurve16, SignedDegrees10, SyncState, Table2D16, TempC10, TrimPolicy,
    ValidatedCalibration,
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
        rpm_axis: axis(&[500, 1000]),
        load_axis: axis(&[500, 1000]),
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

fn table_u16_2x2(a00: u16, a01: u16, a10: u16, a11: u16) -> Table2D16<u16> {
    Table2D16 {
        rpm_axis: axis(&[500, 1000]),
        load_axis: axis(&[500, 1000]),
        values: {
            let mut values = [[0u16; 16]; 16];
            values[0][0] = a00;
            values[0][1] = a01;
            values[1][0] = a10;
            values[1][1] = a11;
            values
        },
    }
}

fn table_u16_3x3(values_3x3: [[u16; 3]; 3]) -> Table2D16<u16> {
    let mut values = [[0u16; 16]; 16];
    let mut load = 0usize;
    while load < 3 {
        let mut rpm = 0usize;
        while rpm < 3 {
            values[load][rpm] = values_3x3[load][rpm];
            rpm += 1;
        }
        load += 1;
    }
    Table2D16 {
        rpm_axis: axis(&[500, 1000, 1500]),
        load_axis: axis(&[500, 1000, 1500]),
        values,
    }
}

fn table_i16(value: i16) -> Table2D16<i16> {
    Table2D16 {
        rpm_axis: axis(&[500, 1000]),
        load_axis: axis(&[500, 1000]),
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

fn signed_curve(value: i16) -> SignedCurve16 {
    let mut curve = SignedCurve16 {
        axis: axis(&[500, 1000]),
        ..SignedCurve16::default()
    };
    curve.values[0] = value;
    curve.values[1] = value;
    curve
}

fn table_u32(value: u32) -> Table2D16<u32> {
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

fn canonical_input() -> InputSnapshot {
    InputSnapshot {
        t_us: ecu_spec::Micros::new(0),
        rpm: Rpm::new(1000),
        map_kpa10: Kpa10::new(1000),
        load_kpa10: Kpa10::new(1000),
        tps_x100: 0,
        clt_c10: TempC10::new(800),
        iat_c10: TempC10::new(250),
        baro_kpa10: Kpa10::new(1000),
        vbatt_mv: Millivolts::new(12_000),
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

fn interpolation_input() -> InputSnapshot {
    InputSnapshot {
        t_us: ecu_spec::Micros::new(0),
        rpm: Rpm::new(750),
        map_kpa10: Kpa10::new(1000),
        load_kpa10: Kpa10::new(750),
        tps_x100: 0,
        clt_c10: TempC10::new(800),
        iat_c10: TempC10::new(250),
        baro_kpa10: Kpa10::new(1000),
        vbatt_mv: Millivolts::new(12_000),
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

fn interpolation_calibration() -> ValidatedCalibration {
    ValidatedCalibration(Calibration {
        fuel_model: FuelModel::SpeedDensityRequiredFuel,
        ve_table: table_u16_2x2(6000, 8000, 10000, 14000),
        afr_target_table: table_u16_2x2(1400, 1500, 1600, 1800),
        spark_advance_table_deg10: table_i16(150),
        dwell_table_us: table_u32(2500),
        injection_target_table_deg10: table_u16(360),
        deadtime_table_us: table_u16_2x2(800, 800, 800, 800),
        clt_corr_curve: Curve16 {
            axis: axis(&[0, 100]),
            values: {
                let mut values = [0u16; 16];
                values[0] = 1000;
                values[1] = 1000;
                values
            },
        },
        iat_corr_curve: Curve16 {
            axis: axis(&[0, 100]),
            values: {
                let mut values = [0u16; 16];
                values[0] = 1000;
                values[1] = 1000;
                values
            },
        },
        baro_corr_curve: Curve16 {
            axis: axis(&[0, 100]),
            values: {
                let mut values = [0u16; 16];
                values[0] = 1000;
                values[1] = 1000;
                values
            },
        },
        vbat_corr_curve: Curve16 {
            axis: axis(&[8000, 16000]),
            values: {
                let mut values = [0u16; 16];
                values[0] = 1000;
                values[1] = 1000;
                values
            },
        },
        cranking_curve: Curve16 {
            axis: axis(&[0, 100]),
            values: {
                let mut values = [0u16; 16];
                values[0] = 1000;
                values[1] = 1000;
                values
            },
        },
        afterstart_table: table_u16_2x2(1000, 1000, 1000, 1000),
        afterstart_window_cycles: 0,
        warmup_curve: Curve16 {
            axis: axis(&[0, 100]),
            values: {
                let mut values = [0u16; 16];
                values[0] = 1000;
                values[1] = 1000;
                values
            },
        },
        ae_tps_threshold_curve: Curve16 {
            axis: axis(&[500, 7000]),
            values: {
                let mut values = [0u16; 16];
                values[0] = 20000;
                values[1] = 20000;
                values
            },
        },
        ae_map_threshold_curve: Curve16 {
            axis: axis(&[100, 3000]),
            values: {
                let mut values = [0u16; 16];
                values[0] = 20000;
                values[1] = 20000;
                values
            },
        },
        ae_shot_curve_us: Curve16 {
            axis: axis(&[100, 3000]),
            values: {
                let mut values = [0u16; 16];
                values[0] = 0;
                values[1] = 0;
                values
            },
        },
        ae_decay_steps_curve: Curve16 {
            axis: axis(&[100, 3000]),
            values: {
                let mut values = [0u16; 16];
                values[0] = 0;
                values[1] = 0;
                values
            },
        },
        ae_decay_ratio_curve_x1000: Curve16 {
            axis: axis(&[0, 16]),
            values: {
                let mut values = [0u16; 16];
                values[0] = 1000;
                values[1] = 1000;
                values
            },
        },
        dfco_entry_rpm: Rpm::new(20_000),
        dfco_exit_rpm: Rpm::new(19_000),
        dfco_entry_tps_x100: 0,
        dfco_exit_tps_x100: 100,
        dfco_entry_map_kpa10: Kpa10::new(0),
        dfco_delay_cycles: 0,
        soft_rev_rpm: Rpm::new(19_500),
        hard_rev_rpm: Rpm::new(20_000),
        rev_hysteresis_rpm: Rpm::new(100),
        soft_retard_max_deg10: 0,
        launch_rpm_limit: Rpm::new(20_000),
        launch_cut_cycles: 0,
        flat_shift_rpm_min: Rpm::new(20_000),
        flat_shift_cut_cycles: 0,
        knock_threshold_x100: 500,
        knock_retard_step_deg10: 20,
        knock_retard_max_deg10: 200,
        knock_recovery_step_deg10: 10,
        knock_recovery_delay_cycles: 2,
        tps_adc_min_counts: 0,
        tps_adc_max_counts: 4095,
        idle_target_rpm: Rpm::new(900),
        idle_base_duty_x1000: 0,
        idle_kp_x1000: 0,
        idle_ki_x1000: 0,
        idle_timing_enabled: false,
        idle_timing_pid_enabled: false,
        idle_timing_rpm_max: Rpm::new(1200),
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
        pw_max_policy: PwMaxPolicy::Fixed,
        pw_max_us: 25_000,
        injection_angle_mode: InjectionAngleMode::EndOfInjection,
        cylinder_phase_deg10: CylinderArrayU16 {
            count: 4,
            values: [0, 1800, 3600, 5400, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        },
    })
}

fn cycle_distance(from: u16, to: u16) -> u16 {
    ((to as u32 + 7200 - from as u32) % 7200) as u16
}

fn canonical_calibration(mode: InjectionAngleMode) -> ValidatedCalibration {
    ValidatedCalibration(Calibration {
        fuel_model: FuelModel::SpeedDensityRequiredFuel,
        ve_table: table_u16(8000),
        afr_target_table: table_u16(1470),
        spark_advance_table_deg10: table_i16(150),
        dwell_table_us: table_u32(2500),
        injection_target_table_deg10: table_u16(360),
        deadtime_table_us: table_u16_2x2(800, 800, 800, 800),
        clt_corr_curve: Curve16 {
            axis: axis(&[0, 100]),
            values: {
                let mut values = [0u16; 16];
                values[0] = 1000;
                values[1] = 1000;
                values
            },
        },
        iat_corr_curve: Curve16 {
            axis: axis(&[0, 100]),
            values: {
                let mut values = [0u16; 16];
                values[0] = 1000;
                values[1] = 1000;
                values
            },
        },
        baro_corr_curve: Curve16 {
            axis: axis(&[0, 100]),
            values: {
                let mut values = [0u16; 16];
                values[0] = 1000;
                values[1] = 1000;
                values
            },
        },
        vbat_corr_curve: Curve16 {
            axis: axis(&[8000, 16000]),
            values: {
                let mut values = [0u16; 16];
                values[0] = 1000;
                values[1] = 1000;
                values
            },
        },
        cranking_curve: Curve16 {
            axis: axis(&[0, 100]),
            values: {
                let mut values = [0u16; 16];
                values[0] = 1000;
                values[1] = 1000;
                values
            },
        },
        afterstart_table: table_u16_2x2(1000, 1000, 1000, 1000),
        afterstart_window_cycles: 0,
        warmup_curve: Curve16 {
            axis: axis(&[0, 100]),
            values: {
                let mut values = [0u16; 16];
                values[0] = 1000;
                values[1] = 1000;
                values
            },
        },
        ae_tps_threshold_curve: Curve16 {
            axis: axis(&[500, 7000]),
            values: {
                let mut values = [0u16; 16];
                values[0] = 20000;
                values[1] = 20000;
                values
            },
        },
        ae_map_threshold_curve: Curve16 {
            axis: axis(&[100, 3000]),
            values: {
                let mut values = [0u16; 16];
                values[0] = 20000;
                values[1] = 20000;
                values
            },
        },
        ae_shot_curve_us: Curve16 {
            axis: axis(&[100, 3000]),
            values: {
                let mut values = [0u16; 16];
                values[0] = 0;
                values[1] = 0;
                values
            },
        },
        ae_decay_steps_curve: Curve16 {
            axis: axis(&[100, 3000]),
            values: {
                let mut values = [0u16; 16];
                values[0] = 0;
                values[1] = 0;
                values
            },
        },
        ae_decay_ratio_curve_x1000: Curve16 {
            axis: axis(&[0, 16]),
            values: {
                let mut values = [0u16; 16];
                values[0] = 1000;
                values[1] = 1000;
                values
            },
        },
        dfco_entry_rpm: Rpm::new(20_000),
        dfco_exit_rpm: Rpm::new(19_000),
        dfco_entry_tps_x100: 0,
        dfco_exit_tps_x100: 100,
        dfco_entry_map_kpa10: Kpa10::new(0),
        dfco_delay_cycles: 0,
        soft_rev_rpm: Rpm::new(19_500),
        hard_rev_rpm: Rpm::new(20_000),
        rev_hysteresis_rpm: Rpm::new(100),
        soft_retard_max_deg10: 0,
        launch_rpm_limit: Rpm::new(20_000),
        launch_cut_cycles: 0,
        flat_shift_rpm_min: Rpm::new(20_000),
        flat_shift_cut_cycles: 0,
        knock_threshold_x100: 500,
        knock_retard_step_deg10: 20,
        knock_retard_max_deg10: 200,
        knock_recovery_step_deg10: 10,
        knock_recovery_delay_cycles: 2,
        tps_adc_min_counts: 0,
        tps_adc_max_counts: 4095,
        idle_target_rpm: Rpm::new(900),
        idle_base_duty_x1000: 0,
        idle_kp_x1000: 0,
        idle_ki_x1000: 0,
        idle_timing_enabled: false,
        idle_timing_pid_enabled: false,
        idle_timing_rpm_max: Rpm::new(1200),
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
        pw_max_policy: PwMaxPolicy::Fixed,
        pw_max_us: 25_000,
        injection_angle_mode: mode,
        cylinder_phase_deg10: CylinderArrayU16 {
            count: 4,
            values: [0, 1800, 3600, 5400, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        },
    })
}

fn assert_canonical_output(output: &ObservableOutput) {
    assert_eq!(output.ve_pct_x100.get(), 8000);
    assert_eq!(output.target_afr_x100.get(), 1470);
    assert_eq!(output.pw_base_us.get(), 2400);
    assert_eq!(output.pw_air_us.get(), 2400);
    assert_eq!(output.pw_corr_us.get(), 3200);
    assert_eq!(output.lambda_correction_x1000, 1000);
    assert_eq!(output.spark_advance_deg10, SignedDegrees10::new(150));
    assert_eq!(output.dwell_us.get(), 2500);
    assert_eq!(output.diagnostic, DiagnosticCode::None);
    assert_eq!(output.events.len, 16);
}

#[test]
fn canonical_interpolation_example_matches_expected_values() {
    let cal = interpolation_calibration();
    let input = interpolation_input();
    let result = step(&cal, input, &LogicalState::default());
    assert_eq!(result.output.ve_pct_x100.get(), 9500);
    assert_eq!(result.output.target_afr_x100.get(), 1575);
}

#[test]
fn canonical_base_and_corrected_fuel_examples_match_expected_values() {
    let cal = canonical_calibration(InjectionAngleMode::EndOfInjection);
    let input = canonical_input();
    let result = step(&cal, input, &LogicalState::default());
    assert_canonical_output(&result.output);
}

#[test]
fn canonical_end_of_injection_schedule_matches_expected_angles() {
    let cal = canonical_calibration(InjectionAngleMode::EndOfInjection);
    let input = canonical_input();
    let result = step(&cal, input, &LogicalState::default());
    assert_eq!(result.output.soi_deg10.values[0], 6648);
    assert_eq!(result.output.eoi_deg10.values[0], 6840);
    assert_eq!(result.output.spark_deg10.values[0], 7050);
    assert_eq!(result.output.dwell_start_deg10.values[0], 6900);
    assert_eq!(result.output.soi_deg10.values[1], 1248);
    assert_eq!(result.output.eoi_deg10.values[1], 1440);
    assert_eq!(result.output.spark_deg10.values[1], 1650);
    assert_eq!(result.output.dwell_start_deg10.values[1], 1500);
    assert_eq!(result.output.soi_deg10.values[2], 3048);
    assert_eq!(result.output.eoi_deg10.values[2], 3240);
    assert_eq!(result.output.spark_deg10.values[2], 3450);
    assert_eq!(result.output.dwell_start_deg10.values[2], 3300);
    assert_eq!(result.output.soi_deg10.values[3], 4848);
    assert_eq!(result.output.eoi_deg10.values[3], 5040);
    assert_eq!(result.output.spark_deg10.values[3], 5250);
    assert_eq!(result.output.dwell_start_deg10.values[3], 5100);
    assert_eq!(
        cycle_distance(
            result.output.soi_deg10.values[0],
            result.output.eoi_deg10.values[0]
        ),
        192
    );
    assert_eq!(
        cycle_distance(
            result.output.dwell_start_deg10.values[0],
            result.output.spark_deg10.values[0]
        ),
        150
    );
    assert_eq!(
        result.output.events.events[0].kind,
        EventKind::InjectionOpen
    );
    assert_eq!(
        result.output.events.events[1].kind,
        EventKind::InjectionClose
    );
    assert_eq!(
        result.output.events.events[2].kind,
        EventKind::CoilChargeStart
    );
    assert_eq!(result.output.events.events[3].kind, EventKind::CoilFire);
}

#[test]
fn canonical_start_of_injection_schedule_matches_expected_anchor() {
    let cal = canonical_calibration(InjectionAngleMode::StartOfInjection);
    let input = canonical_input();
    let result = step(&cal, input, &LogicalState::default());
    assert_eq!(result.output.soi_deg10.values[0], 6840);
    assert_eq!(result.output.eoi_deg10.values[0], 7032);
    assert_eq!(result.output.soi_deg10.values[1], 1440);
    assert_eq!(result.output.eoi_deg10.values[1], 1632);
    assert_eq!(result.output.soi_deg10.values[2], 3240);
    assert_eq!(result.output.eoi_deg10.values[2], 3432);
    assert_eq!(result.output.soi_deg10.values[3], 5040);
    assert_eq!(result.output.eoi_deg10.values[3], 5232);
}

#[test]
fn canonical_fuel_cut_example_suppresses_injection_events() {
    let cal = canonical_calibration(InjectionAngleMode::EndOfInjection);
    let mut input = canonical_input();
    input.fuel_cut = true;
    let result = step(&cal, input, &LogicalState::default());
    assert_eq!(result.output.pw_corr_us.get(), 0);
    assert_eq!(result.output.cut_reason_code, 1);
    assert!(result.output.fuel_cut);
    assert!(result.output.spark_cut);
    assert_eq!(result.output.events.len, 0);
    assert_eq!(result.output.diagnostic, DiagnosticCode::FuelCutActive);
}

#[test]
fn canonical_spark_cut_example_suppresses_spark_events() {
    let cal = canonical_calibration(InjectionAngleMode::EndOfInjection);
    let mut input = canonical_input();
    input.spark_cut = true;
    let result = step(&cal, input, &LogicalState::default());
    assert_eq!(result.output.cut_reason_code, 1);
    assert!(result.output.fuel_cut);
    assert!(result.output.spark_cut);
    assert_eq!(result.output.events.len, 0);
    assert_eq!(result.output.diagnostic, DiagnosticCode::FuelCutActive);
}

#[test]
fn interpolation_last_segment_right_closure_and_boundary_equality_examples() {
    let table = table_u16_3x3([[1000, 2000, 3000], [1000, 2000, 3000], [1000, 2000, 3000]]);
    assert_eq!(find_segment(&table.rpm_axis, 1500), 1);
    // x==x2 uses last segment right-closure and reproduces the top breakpoint value.
    assert_eq!(bilerp_u16(&table, Rpm::new(1500), Kpa10::new(500)), 3000);
    // x==x1 lands exactly on the shared boundary and reproduces column 1.
    assert_eq!(bilerp_u16(&table, Rpm::new(1000), Kpa10::new(500)), 2000);
}

#[test]
fn interpolation_decreasing_u16_examples_match_hand_computation() {
    // 1000 + floor((200-1000)*1/4) = 1000 + floor(-800/4) = 800.
    assert_eq!(lerp_u16(0, 4, 1000, 200, 1), 800);

    let table = table_u16_2x2(1000, 600, 900, 500);
    // row0@x=750=1000+floor((600-1000)*250/500)=800; row1@x=750=700; y-mid => 800+floor((700-800)*250/500)=750.
    assert_eq!(bilerp_u16(&table, Rpm::new(750), Kpa10::new(750)), 750);
}

#[test]
fn interpolation_distinct_corner_bilinear_cell_example() {
    let table = table_u16_2x2(1000, 1400, 2000, 2600);
    // row0@x=750=1200; row1@x=750=2300; y-mid => 1200+floor((2300-1200)*250/500)=1750.
    assert_eq!(bilerp_u16(&table, Rpm::new(750), Kpa10::new(750)), 1750);
}

#[test]
fn interpolation_negative_slope_i16_curve_example() {
    // 300 + floor((-100-300)*1/4) = 300 + floor(-400/4) = 200.
    assert_eq!(lerp_i16(0, 4, 300, -100, 1), 200);
}

#[test]
fn phase2_deadtime_midpoint_vector_matches_hand_computation() {
    let mut values = [[0u16; 16]; 16];
    values[0][0] = 100;
    values[0][1] = 200;
    values[1][0] = 300;
    values[1][1] = 500;
    let table = Table2D16 {
        rpm_axis: axis(&[1000, 13000]),
        load_axis: axis(&[1000, 3000]),
        values,
    };
    // row0=100+floor((200-100)*(7000-1000)/12000)=150
    // row1=300+floor((500-300)*(7000-1000)/12000)=400
    // y-mid => 150+floor((400-150)*(2000-1000)/2000)=275
    assert_eq!(
        deadtime_lookup(&table, Millivolts::new(7000), Kpa10::new(2000)).get(),
        275
    );
}

#[test]
fn phase2_vbat_edge_vectors_match_hand_computation() {
    let mut values = [0u16; 16];
    values[0] = 1200;
    values[1] = 1000;
    values[2] = 900;
    let curve = Curve16 {
        axis: axis(&[8000, 12000, 16000]),
        values,
    };
    // Below minimum axis clamps to first cell.
    assert_eq!(vbat_correction(&curve, Millivolts::new(7000)).get(), 1200);
    // Above maximum axis clamps to last cell.
    assert_eq!(vbat_correction(&curve, Millivolts::new(17000)).get(), 900);
}

#[test]
fn phase2_baro_edge_vectors_match_hand_computation() {
    let mut values = [0u16; 16];
    values[0] = 700;
    values[1] = 850;
    values[2] = 1000;
    let curve = Curve16 {
        axis: axis(&[700, 850, 1000]),
        values,
    };
    assert_eq!(baro_correction(&curve, Kpa10::new(700)).get(), 700);
    assert_eq!(baro_correction(&curve, Kpa10::new(1000)).get(), 1000);
}

#[test]
fn phase2_cranking_vector_matches_hand_computation() {
    let mut values = [0u16; 16];
    values[0] = 1800;
    values[1] = 1200;
    values[2] = 1000;
    let curve = Curve16 {
        axis: axis(&[0, 400, 800]),
        values,
    };
    assert_eq!(
        cranking_corr_x1000(&curve, EngineMode::Cranking, TempC10::new(400)).get(),
        1200
    );
    assert_eq!(
        cranking_corr_x1000(&curve, EngineMode::Running, TempC10::new(400)).get(),
        1000
    );
}

#[test]
fn phase2_ae_decay_vector_matches_hand_computation() {
    let input = InputSnapshot {
        rpm: Rpm::new(2000),
        load_kpa10: Kpa10::new(1000),
        map_kpa10: Kpa10::new(1000),
        ..InputSnapshot::default()
    };
    let tps_threshold_curve = Curve16 {
        axis: axis(&[1000, 3000]),
        values: {
            let mut values = [0u16; 16];
            values[0] = 500;
            values[1] = 500;
            values
        },
    };
    let map_threshold_curve = Curve16 {
        axis: axis(&[500, 1500]),
        values: {
            let mut values = [0u16; 16];
            values[0] = 500;
            values[1] = 500;
            values
        },
    };
    let shot_curve_us = Curve16 {
        axis: axis(&[500, 1500]),
        values: [0; 16],
    };
    let decay_steps_curve = Curve16 {
        axis: axis(&[500, 1500]),
        values: [0; 16],
    };
    let decay_ratio_curve_x1000 = Curve16 {
        axis: axis(&[0, 3]),
        values: {
            let mut values = [0u16; 16];
            values[0] = 800;
            values[1] = 800;
            values
        },
    };
    let result = ae_step_with_deltas(
        AeCurves {
            tps_threshold_curve: &tps_threshold_curve,
            map_threshold_curve: &map_threshold_curve,
            shot_curve_us: &shot_curve_us,
            decay_steps_curve: &decay_steps_curve,
            decay_ratio_curve_x1000: &decay_ratio_curve_x1000,
        },
        input,
        ecu_spec::AeState {
            active: true,
            pulse_us: 1000,
            decay_steps_remaining: 2,
        },
        0,
        0,
    );
    // 1000 * 800 / 1000 = 800 with floor integer ratio math.
    assert_eq!(result.ae_pulse_us.get(), 800);
    assert_eq!(result.next_state.decay_steps_remaining, 1);
}

#[test]
fn phase2_dfco_hysteresis_vector_matches_hand_computation() {
    let mut cal = canonical_calibration(InjectionAngleMode::EndOfInjection);
    cal.0.dfco_entry_rpm = Rpm::new(2000);
    cal.0.dfco_exit_rpm = Rpm::new(1800);
    cal.0.dfco_entry_tps_x100 = 200;
    cal.0.dfco_exit_tps_x100 = 300;
    cal.0.dfco_entry_map_kpa10 = Kpa10::new(500);
    cal.0.dfco_delay_cycles = 1;
    let mut input = canonical_input();
    input.mode = EngineMode::Running;
    input.rpm = Rpm::new(3000);
    input.tps_x100 = 0;
    input.map_kpa10 = Kpa10::new(350);
    let state = LogicalState {
        dfco_active: true,
        ..LogicalState::default()
    };
    let hold = dfco_step(&cal, input, &state);
    assert!(hold.fuel_cut);
    input.tps_x100 = 600;
    let release = dfco_step(&cal, input, &state);
    assert!(!release.fuel_cut);
}

#[test]
fn phase2_rev_limit_soft_and_hard_vectors_match_hand_computation() {
    let mut cal = canonical_calibration(InjectionAngleMode::EndOfInjection);
    cal.0.soft_rev_rpm = Rpm::new(4500);
    cal.0.hard_rev_rpm = Rpm::new(6000);
    cal.0.rev_hysteresis_rpm = Rpm::new(100);
    cal.0.soft_retard_max_deg10 = 120;
    let soft = rev_limit_step(
        &cal,
        InputSnapshot {
            rpm: Rpm::new(4600),
            ..InputSnapshot::default()
        },
        &LogicalState::default(),
    );
    assert!(soft.soft_rev_spark_cut);
    assert!(!soft.hard_rev_fuel_cut);
    assert_eq!(soft.soft_retard_deg10, -120);
    let hard = rev_limit_step(
        &cal,
        InputSnapshot {
            rpm: Rpm::new(6100),
            ..InputSnapshot::default()
        },
        &LogicalState::default(),
    );
    assert!(hard.hard_rev_fuel_cut);
}

#[test]
fn phase2_idle_pi_step_vector_matches_hand_computation() {
    let mut cal = canonical_calibration(InjectionAngleMode::EndOfInjection);
    cal.0.idle_target_rpm = Rpm::new(1000);
    cal.0.idle_base_duty_x1000 = 300;
    cal.0.idle_kp_x1000 = 200;
    cal.0.idle_ki_x1000 = 100;
    let input = InputSnapshot {
        rpm: Rpm::new(900),
        clt_c10: TempC10::new(800),
        ..InputSnapshot::default()
    };
    // error=100, p=floor(100*200/1000)=20, i=floor(100*100/1000)=10, duty=300+20+10=330.
    let result = idle_step(&cal, input, &LogicalState::default());
    assert_eq!(result.integrator_state.acc, 10);
    assert_eq!(result.duty_x1000, 330);
}

#[test]
fn phase2_lambda_pi_step_vector_matches_hand_computation() {
    let mut cal = canonical_calibration(InjectionAngleMode::EndOfInjection);
    cal.0.lambda_kp_x1000 = 200;
    cal.0.lambda_ki_x1000 = 100;
    let input = InputSnapshot {
        clt_c10: TempC10::new(800),
        ..InputSnapshot::default()
    };
    // error=100, p=20, i=10, correction=1000+20+10=1030.
    let result = lambda_step_with_error(&cal, input, &LogicalState::default(), false, 100);
    assert_eq!(result.integrator_state.acc, 10);
    assert_eq!(result.correction_x1000, 1030);
}

#[test]
fn phase2_knock_retard_vector_matches_hand_computation() {
    let mut cal = canonical_calibration(InjectionAngleMode::EndOfInjection);
    cal.0.knock_threshold_x100 = 200;
    cal.0.knock_retard_step_deg10 = 30;
    cal.0.knock_retard_max_deg10 = 80;
    let state = LogicalState {
        knock_state: ecu_spec::KnockState {
            retard_deg10: 60,
            recovery_counter: 0,
            detected: true,
        },
        ..LogicalState::default()
    };
    let result = knock_step(
        &cal,
        InputSnapshot {
            knock_intensity_x100: 200,
            ..InputSnapshot::default()
        },
        &state,
    );
    assert_eq!(result.next_state.retard_deg10, 80);
    assert_eq!(result.advance_trim_deg10, -80);
}

#[test]
fn phase2_launch_vector_matches_hand_computation() {
    let mut cal = canonical_calibration(InjectionAngleMode::EndOfInjection);
    cal.0.launch_rpm_limit = Rpm::new(5000);
    cal.0.launch_cut_cycles = 2;
    let result = launch_step(
        &cal,
        InputSnapshot {
            launch_armed: true,
            rpm: Rpm::new(5500),
            ..InputSnapshot::default()
        },
        &LogicalState {
            launch_cut_cycle_count: 1,
            ..LogicalState::default()
        },
    );
    // phase=1%(2+1)=1 < 2 => cut; next count increments to 2.
    assert!(result.launch_cut);
    assert_eq!(result.cut_cycle_count, 2);
}

#[test]
fn phase2_flat_shift_vector_matches_hand_computation() {
    let mut cal = canonical_calibration(InjectionAngleMode::EndOfInjection);
    cal.0.flat_shift_rpm_min = Rpm::new(5000);
    cal.0.flat_shift_cut_cycles = 2;
    let result = flat_shift_step(
        &cal,
        InputSnapshot {
            flat_shift_armed: true,
            rpm: Rpm::new(6000),
            ..InputSnapshot::default()
        },
        &LogicalState {
            flat_shift_cut_cycle_count: 1,
            ..LogicalState::default()
        },
    );
    // phase=1%(2+1)=1 < 2 => cut; next count increments to 2.
    assert!(result.flat_shift_cut);
    assert_eq!(result.cut_cycle_count, 2);
}

#[test]
fn phase2_arbiter_priority_vector_matches_hand_computation() {
    let all_active = ArbiterInputs {
        safety_latched: true,
        dfco_cut: true,
        rev_limit: RevLimitResult {
            soft_rev_spark_cut: true,
            hard_rev_fuel_cut: true,
            soft_retard_deg10: -100,
            soft_active: true,
            hard_active: true,
        },
        launch_cut: true,
        flat_shift: FlatShiftResult {
            flat_shift_cut: true,
            active: true,
            cut_cycle_count: 1,
        },
        knock: KnockResult {
            advance_trim_deg10: -50,
            knock_active: true,
            next_state: ecu_spec::KnockState {
                retard_deg10: 50,
                recovery_counter: 0,
                detected: true,
            },
        },
    };
    let safety_wins = arbiter_step(all_active);
    assert_eq!(safety_wins.cut_reason_code, 1);
    assert!(safety_wins.fuel_cut);
    assert!(safety_wins.spark_cut);

    let hard_rev_wins = arbiter_step(ArbiterInputs {
        safety_latched: false,
        ..all_active
    });
    assert_eq!(hard_rev_wins.cut_reason_code, 2);
    assert!(hard_rev_wins.fuel_cut);
    assert!(hard_rev_wins.spark_cut);
}
