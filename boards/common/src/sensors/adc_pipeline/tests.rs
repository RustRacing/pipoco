use super::*;

fn adc_counts_for_mv(mv: u16, vref_mv: u16, adc_bits: u8) -> u16 {
    let max = (1u32 << adc_bits) - 1;
    ((mv as u32 * max + (vref_mv as u32 / 2)) / vref_mv as u32) as u16
}

fn default_raw_with_map(map: u16) -> RawCounts {
    RawCounts {
        map,
        maf: None,
        tps: 200,
        clt: 2048,
        iat: 2048,
        vbatt: 2048,
        lambda: None,
        baro: None,
    }
}

#[test]
fn test_clamp_slew_u16_limits_delta() {
    // prev=1000, new=2000, dt=100ms, max_rate=1000 units/s => max_delta=100
    let out = clamp_slew_u16(1000, 2000, 1000, 100_000);
    assert_eq!(out, 1100);
    // Negative direction
    let out2 = clamp_slew_u16(1000, 0, 1000, 100_000);
    assert_eq!(out2, 900);
    // Within limits
    let out3 = clamp_slew_u16(1000, 1050, 1000, 100_000);
    assert_eq!(out3, 1050);
}

#[test]
fn test_clamp_slew_u8_limits_delta() {
    // prev=10%, new=90%, dt=50ms, max_rate=200%/s => max_delta=10%
    let out = clamp_slew_u8(10, 90, 200, 50_000);
    assert_eq!(out, 20);
    // Negative direction
    let out2 = clamp_slew_u8(50, 0, 200, 50_000);
    assert_eq!(out2, 40);
    // Within limits
    let out3 = clamp_slew_u8(10, 15, 200, 50_000);
    assert_eq!(out3, 15);
}

#[test]
fn convert_all_leaves_unwired_maf_at_zero() {
    let out = convert_all(
        AdcConfig {
            vref_mv: 5000,
            adc_bits: 12,
            vbatt_scale_num: 1,
            vbatt_scale_den: 1,
            clt_bias: ThermistorBias::PULLUP_2490,
            iat_bias: ThermistorBias::PULLUP_2490,
            lambda_cal: LambdaAdcCalibration::WIDEBAND_0V5_TO_4V5,
            baro_cal: BaroAdcCalibration::LINEAR_0V5_TO_4V5,
        },
        &SensorsCal::default(),
        RawCounts {
            map: 2048,
            maf: None,
            tps: 200,
            clt: 2048,
            iat: 2048,
            vbatt: 2048,
            lambda: None,
            baro: None,
        },
    );

    assert!(!out.maf_valid);
    assert_eq!(out.maf_x100, 0);
    assert!(!out.lambda_valid);
}

#[test]
fn convert_all_converts_maf_counts_to_source_native_airflow() {
    let out = convert_all(
        AdcConfig {
            vref_mv: 5000,
            adc_bits: 12,
            vbatt_scale_num: 1,
            vbatt_scale_den: 1,
            clt_bias: ThermistorBias::PULLUP_2490,
            iat_bias: ThermistorBias::PULLUP_2490,
            lambda_cal: LambdaAdcCalibration::WIDEBAND_0V5_TO_4V5,
            baro_cal: BaroAdcCalibration::LINEAR_0V5_TO_4V5,
        },
        &SensorsCal::default(),
        RawCounts {
            map: 2048,
            maf: Some(2048),
            tps: 200,
            clt: 2048,
            iat: 2048,
            vbatt: 2048,
            lambda: None,
            baro: None,
        },
    );

    assert!(out.maf_valid);
    assert!(out.maf_x100 > 0);
    assert!(out.maf_x100 < 9300);
}

#[test]
fn convert_all_reports_lambda_validity_explicitly() {
    let cfg = AdcConfig {
        vref_mv: 5000,
        adc_bits: 12,
        vbatt_scale_num: 1,
        vbatt_scale_den: 1,
        clt_bias: ThermistorBias::PULLUP_2490,
        iat_bias: ThermistorBias::PULLUP_2490,
        lambda_cal: LambdaAdcCalibration::WIDEBAND_0V5_TO_4V5,
        baro_cal: BaroAdcCalibration::LINEAR_0V5_TO_4V5,
    };

    let missing = convert_all(cfg, &SensorsCal::default(), default_raw_with_map(2048));
    let present = convert_all(
        cfg,
        &SensorsCal::default(),
        RawCounts {
            lambda: Some(adc_counts_for_mv(2500, 5000, 12)),
            ..default_raw_with_map(2048)
        },
    );

    assert!(!missing.lambda_valid);
    assert_eq!(missing.lambda_x100, 100);
    assert!(present.lambda_valid);
    assert!((100..=104).contains(&present.lambda_x100));
}

#[test]
fn convert_all_uses_configured_lambda_adc_calibration() {
    let cfg = AdcConfig {
        vref_mv: 5000,
        adc_bits: 12,
        vbatt_scale_num: 1,
        vbatt_scale_den: 1,
        clt_bias: ThermistorBias::PULLUP_2490,
        iat_bias: ThermistorBias::PULLUP_2490,
        lambda_cal: LambdaAdcCalibration {
            mv_min: 1000,
            lambda_min_x100: 80,
            mv_max: 4000,
            lambda_max_x100: 120,
        },
        baro_cal: BaroAdcCalibration::LINEAR_0V5_TO_4V5,
    };

    let out = convert_all(
        cfg,
        &SensorsCal::default(),
        RawCounts {
            lambda: Some(adc_counts_for_mv(2500, 5000, 12)),
            ..default_raw_with_map(2048)
        },
    );

    assert!(out.lambda_valid);
    assert!((99..=101).contains(&out.lambda_x100));
}

#[test]
fn convert_all_handles_invalid_lambda_calibration_span() {
    let cfg = AdcConfig {
        vref_mv: 5000,
        adc_bits: 12,
        vbatt_scale_num: 1,
        vbatt_scale_den: 1,
        clt_bias: ThermistorBias::PULLUP_2490,
        iat_bias: ThermistorBias::PULLUP_2490,
        lambda_cal: LambdaAdcCalibration {
            mv_min: 4000,
            lambda_min_x100: 80,
            mv_max: 1000,
            lambda_max_x100: 120,
        },
        baro_cal: BaroAdcCalibration::LINEAR_0V5_TO_4V5,
    };

    let out = convert_all(
        cfg,
        &SensorsCal::default(),
        RawCounts {
            lambda: Some(adc_counts_for_mv(2500, 5000, 12)),
            ..default_raw_with_map(2048)
        },
    );

    assert!(out.lambda_valid);
    assert_eq!(out.lambda_x100, 100);
}

#[test]
fn convert_all_reports_dedicated_baro_validity_and_pressure() {
    let cfg = AdcConfig {
        vref_mv: 5000,
        adc_bits: 12,
        vbatt_scale_num: 1,
        vbatt_scale_den: 1,
        clt_bias: ThermistorBias::PULLUP_2490,
        iat_bias: ThermistorBias::PULLUP_2490,
        lambda_cal: LambdaAdcCalibration::WIDEBAND_0V5_TO_4V5,
        baro_cal: BaroAdcCalibration::LINEAR_0V5_TO_4V5,
    };

    let missing = convert_all(cfg, &SensorsCal::default(), default_raw_with_map(2048));
    let present = convert_all(
        cfg,
        &SensorsCal::default(),
        RawCounts {
            baro: Some(adc_counts_for_mv(2500, 5000, 12)),
            ..default_raw_with_map(2048)
        },
    );

    assert!(!missing.baro_valid);
    assert_eq!(missing.baro_kpa_x10, 0);
    assert!(present.baro_valid);
    assert!((848..=852).contains(&present.baro_kpa_x10));
}

#[test]
fn convert_all_handles_invalid_baro_calibration_span() {
    let cfg = AdcConfig {
        vref_mv: 5000,
        adc_bits: 12,
        vbatt_scale_num: 1,
        vbatt_scale_den: 1,
        clt_bias: ThermistorBias::PULLUP_2490,
        iat_bias: ThermistorBias::PULLUP_2490,
        lambda_cal: LambdaAdcCalibration::WIDEBAND_0V5_TO_4V5,
        baro_cal: BaroAdcCalibration {
            mv_min: 4500,
            kpa_min_x10: 500,
            mv_max: 500,
            kpa_max_x10: 1200,
        },
    };

    let out = convert_all(
        cfg,
        &SensorsCal::default(),
        RawCounts {
            baro: Some(adc_counts_for_mv(2500, 5000, 12)),
            ..default_raw_with_map(2048)
        },
    );

    assert!(out.baro_valid);
    assert_eq!(out.baro_kpa_x10, 0);
}

#[test]
fn convert_all_maps_mpxh6400a_nominal_endpoint_counts() {
    let cfg = AdcConfig {
        vref_mv: 5000,
        adc_bits: 12,
        vbatt_scale_num: 1,
        vbatt_scale_den: 1,
        clt_bias: ThermistorBias::PULLUP_2490,
        iat_bias: ThermistorBias::PULLUP_2490,
        lambda_cal: LambdaAdcCalibration::WIDEBAND_0V5_TO_4V5,
        baro_cal: BaroAdcCalibration::LINEAR_0V5_TO_4V5,
    };
    let cal = SensorsCal::mpxh6400a_5v_scaled(1, 1);

    let low = convert_all(
        cfg,
        &cal,
        default_raw_with_map(adc_counts_for_mv(200, 5000, 12)),
    );
    let high = convert_all(
        cfg,
        &cal,
        default_raw_with_map(adc_counts_for_mv(4800, 5000, 12)),
    );

    assert!((198..=202).contains(&low.map_kpa_x10));
    assert!((3998..=4002).contains(&high.map_kpa_x10));
}

#[test]
fn convert_all_maps_mpx5700ap_nominal_endpoint_counts() {
    let cfg = AdcConfig {
        vref_mv: 5000,
        adc_bits: 12,
        vbatt_scale_num: 1,
        vbatt_scale_den: 1,
        clt_bias: ThermistorBias::PULLUP_2490,
        iat_bias: ThermistorBias::PULLUP_2490,
        lambda_cal: LambdaAdcCalibration::WIDEBAND_0V5_TO_4V5,
        baro_cal: BaroAdcCalibration::LINEAR_0V5_TO_4V5,
    };
    let cal = SensorsCal::mpx5700ap_5v_scaled(1, 1);

    let low = convert_all(
        cfg,
        &cal,
        default_raw_with_map(adc_counts_for_mv(296, 5000, 12)),
    );
    let high = convert_all(
        cfg,
        &cal,
        default_raw_with_map(adc_counts_for_mv(4700, 5000, 12)),
    );

    assert!((148..=152).contains(&low.map_kpa_x10));
    assert!((6998..=7002).contains(&high.map_kpa_x10));
}

#[test]
fn thermistor_bias_is_board_configured_not_hardcoded() {
    let raw = RawCounts {
        map: 2048,
        maf: None,
        tps: 200,
        clt: 2048,
        iat: 2048,
        vbatt: 2048,
        lambda: None,
        baro: None,
    };
    let base_cfg = AdcConfig {
        vref_mv: 5000,
        adc_bits: 12,
        vbatt_scale_num: 1,
        vbatt_scale_den: 1,
        clt_bias: ThermistorBias::PULLUP_2490,
        iat_bias: ThermistorBias::PULLUP_2490,
        lambda_cal: LambdaAdcCalibration::WIDEBAND_0V5_TO_4V5,
        baro_cal: BaroAdcCalibration::LINEAR_0V5_TO_4V5,
    };
    let board_cfg = AdcConfig {
        clt_bias: ThermistorBias {
            known_ohms: 10_000,
            divider: DividerConfig::PullupTop,
        },
        iat_bias: ThermistorBias {
            known_ohms: 10_000,
            divider: DividerConfig::PullupTop,
        },
        ..base_cfg
    };

    let base = convert_all(base_cfg, &SensorsCal::default(), raw);
    let board = convert_all(board_cfg, &SensorsCal::default(), raw);

    assert_ne!(base.clt_c, board.clt_c);
    assert_ne!(base.iat_c, board.iat_c);
}

#[test]
fn convert_all_handles_invalid_vbatt_scale_without_dividing_by_zero() {
    let out = convert_all(
        AdcConfig {
            vref_mv: 5000,
            adc_bits: 12,
            vbatt_scale_num: 1,
            vbatt_scale_den: 0,
            clt_bias: ThermistorBias::PULLUP_2490,
            iat_bias: ThermistorBias::PULLUP_2490,
            lambda_cal: LambdaAdcCalibration::WIDEBAND_0V5_TO_4V5,
            baro_cal: BaroAdcCalibration::LINEAR_0V5_TO_4V5,
        },
        &SensorsCal::default(),
        default_raw_with_map(2048),
    );

    assert_eq!(out.vbatt_mv, 0);
}

#[test]
fn convert_all_handles_invalid_tps_span_without_dividing_by_zero() {
    let mut cal = SensorsCal::default();
    cal.tps_min_counts = 3000;
    cal.tps_max_counts = 1000;

    let out = convert_all(
        AdcConfig {
            vref_mv: 5000,
            adc_bits: 12,
            vbatt_scale_num: 1,
            vbatt_scale_den: 1,
            clt_bias: ThermistorBias::PULLUP_2490,
            iat_bias: ThermistorBias::PULLUP_2490,
            lambda_cal: LambdaAdcCalibration::WIDEBAND_0V5_TO_4V5,
            baro_cal: BaroAdcCalibration::LINEAR_0V5_TO_4V5,
        },
        &cal,
        default_raw_with_map(2048),
    );

    assert_eq!(out.tps_percent, 0);
}
