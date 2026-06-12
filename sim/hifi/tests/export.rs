use ecu_sim_hifi::{
    default_burn_model_export_config, default_loss_config_export,
    default_open_system_export_config, default_plant_config, export_ve_table, fit_loss_config,
    format_burn_curve, format_loss_config, format_ve_table, generate_default_artifacts,
    parse_burn_curve, parse_loss_config, parse_ve_table, BurnModel, CylinderGeometry,
    ExportedBurnCurve, ExportedLossConfig, GasProperties, IntegratorConfig, LossFitSample,
    ManifoldConfig, OpenSystemConfig, PumpingFitSample, ResidualConfig, ThrottleConfig,
    ValveTiming, BURN_CURVE_POINTS, DEFAULT_VE_LOAD_AXIS_KPA10, DEFAULT_VE_RPM_AXIS,
};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct BurnCurve {
    burn_fraction_x10000: [u16; BURN_CURVE_POINTS],
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct VeTable {
    rpm_axis: [u32; 8],
    load_axis: [u16; 8],
    ve_x1000: [[u16; 8]; 8],
}

impl BurnCurve {
    const fn is_valid(self) -> bool {
        if self.burn_fraction_x10000[0] != 0
            || self.burn_fraction_x10000[BURN_CURVE_POINTS - 1] != 10_000
        {
            return false;
        }

        let mut i = 1;
        while i < BURN_CURVE_POINTS {
            if self.burn_fraction_x10000[i] < self.burn_fraction_x10000[i - 1]
                || self.burn_fraction_x10000[i] > 10_000
            {
                return false;
            }
            i += 1;
        }
        true
    }
}

impl VeTable {
    fn is_valid(&self) -> bool {
        self.rpm_axis.windows(2).all(|window| window[0] < window[1])
            && self
                .load_axis
                .windows(2)
                .all(|window| window[0] < window[1])
    }
}

#[test]
fn exported_burn_curve_round_trips_through_core_shape_contract() {
    let exported = ExportedBurnCurve::from_burn_model(BurnModel::SingleWiebe {
        a: 5.0,
        m: 2.0,
        duration_rad: 45.0_f64.to_radians(),
        spark_to_soc_delay_rad: 5.0_f64.to_radians(),
    });

    let core_curve = BurnCurve {
        burn_fraction_x10000: exported.burn_fraction_x10000,
    };

    assert_eq!(exported.burn_fraction_x10000.len(), BURN_CURVE_POINTS);
    assert!(exported.is_valid());
    assert!(core_curve.is_valid());
    assert_eq!(core_curve.burn_fraction_x10000[0], 0);
    assert_eq!(
        core_curve.burn_fraction_x10000[BURN_CURVE_POINTS - 1],
        10_000
    );
    assert!(core_curve
        .burn_fraction_x10000
        .windows(2)
        .all(|window| window[0] <= window[1]));
}

#[test]
fn exported_burn_curve_keeps_plateau_safe_quantized_tail() {
    let exported = ExportedBurnCurve::from_burn_model(BurnModel::DoubleWiebe {
        premixed_fraction: 0.4,
        premixed_a: 6.0,
        premixed_m: 2.0,
        premixed_duration_rad: 18.0_f64.to_radians(),
        main_a: 5.0,
        main_m: 1.4,
        main_duration_rad: 48.0_f64.to_radians(),
        spark_to_soc_delay_rad: 3.0_f64.to_radians(),
    });

    assert!(exported.is_valid());
    assert!(exported
        .burn_fraction_x10000
        .iter()
        .all(|value| *value <= 10_000));
    assert_eq!(exported.burn_fraction_x10000[0], 0);
    assert_eq!(exported.burn_fraction_x10000[BURN_CURVE_POINTS - 1], 10_000);
}

#[test]
fn exported_loss_config_matches_integer_round_trip_shape() {
    fn fmep(rpm: f64, load_kpa: f64) -> f64 {
        20_000.0 + 8_000.0 * (rpm / 1000.0) + 1_000.0 * (rpm / 1000.0).powi(2) + 50.0 * load_kpa
    }

    let fmep_samples = [
        LossFitSample {
            rpm: 1000.0,
            load_kpa: 20.0,
            fmep_pa: fmep(1000.0, 20.0),
        },
        LossFitSample {
            rpm: 2000.0,
            load_kpa: 35.0,
            fmep_pa: fmep(2000.0, 35.0),
        },
        LossFitSample {
            rpm: 3000.0,
            load_kpa: 60.0,
            fmep_pa: fmep(3000.0, 60.0),
        },
        LossFitSample {
            rpm: 3000.0,
            load_kpa: 10.0,
            fmep_pa: fmep(3000.0, 10.0),
        },
        LossFitSample {
            rpm: 4000.0,
            load_kpa: 80.0,
            fmep_pa: fmep(4000.0, 80.0),
        },
        LossFitSample {
            rpm: 1500.0,
            load_kpa: 45.0,
            fmep_pa: fmep(1500.0, 45.0),
        },
    ];
    let pumping_samples = [
        PumpingFitSample {
            throttle_position: 1.0,
            pmep_pa: 5_000.0,
        },
        PumpingFitSample {
            throttle_position: 0.5,
            pmep_pa: 8_000.0,
        },
        PumpingFitSample {
            throttle_position: 0.0,
            pmep_pa: 11_000.0,
        },
    ];

    let exported = fit_loss_config(&fmep_samples, &pumping_samples, 321).unwrap();
    let expected = ExportedLossConfig {
        fmep_base_pa: 20_000,
        fmep_rpm_pa_per_krpm: 8_000,
        fmep_rpm2_pa_per_krpm2: 1_000,
        fmep_load_pa_per_kpa: 50,
        pumping_base_pa: 5_000,
        pumping_throttle_pa_per_x1000: 6,
        accessory_torque_nm_x100: 321,
    };

    assert_eq!(exported, expected);
}

fn base_open_system_config() -> OpenSystemConfig {
    OpenSystemConfig {
        geometry: CylinderGeometry::new(0.086, 0.086, 0.143, 10.0),
        fresh_gas: GasProperties::new(287.0, 718.0),
        burned_gas: GasProperties::new(300.0, 800.0),
        integrator: IntegratorConfig { step_deg: 1.0 },
        manifold: ManifoldConfig {
            volume_m3: 0.002,
            temperature_k: 300.0,
            ambient_pressure_pa: 101_325.0,
        },
        throttle: ThrottleConfig {
            max_area_m2: 2.0e-4,
            discharge_coefficient: 0.8,
        },
        throttle_position: 0.5,
        intake_valve: ValveTiming {
            open_angle_rad: 340.0_f64.to_radians(),
            close_angle_rad: 580.0_f64.to_radians(),
            max_lift_m: 0.008,
            seat_diameter_m: 0.032,
            discharge_coefficient: 0.7,
        },
        exhaust_valve: ValveTiming {
            open_angle_rad: 120.0_f64.to_radians(),
            close_angle_rad: 360.0_f64.to_radians(),
            max_lift_m: 0.007,
            seat_diameter_m: 0.028,
            discharge_coefficient: 0.7,
        },
        residual: ResidualConfig {
            residual_temperature_k: 900.0,
        },
        initial_cylinder_pressure_pa: 101_325.0,
        initial_cylinder_temperature_k: 330.0,
        initial_residual_fraction: 0.08,
        exhaust_backpressure_pa: 110_000.0,
        exhaust_temperature_k: 850.0,
        rpm: 2500.0,
        trapped_mass_tolerance_kg: 1.0e-6,
        residual_tolerance: 1.0e-4,
        max_cycles: 8,
    }
}

#[test]
fn exported_ve_table_matches_core_axis_contract() {
    let exported = export_ve_table(
        base_open_system_config(),
        DEFAULT_VE_RPM_AXIS,
        DEFAULT_VE_LOAD_AXIS_KPA10,
    )
    .unwrap();

    let core_table = VeTable {
        rpm_axis: DEFAULT_VE_RPM_AXIS,
        load_axis: DEFAULT_VE_LOAD_AXIS_KPA10,
        ve_x1000: exported.ve_x1000,
    };

    assert!(exported.is_valid());
    assert!(core_table.is_valid());
}

#[test]
fn exported_ve_table_trends_up_with_load_for_a_fixed_rpm_row() {
    let exported = export_ve_table(
        base_open_system_config(),
        DEFAULT_VE_RPM_AXIS,
        DEFAULT_VE_LOAD_AXIS_KPA10,
    )
    .unwrap();
    let row = exported.ve_x1000[3];

    assert!(row[0] <= row[row.len() - 1]);
}

#[test]
fn generated_artifacts_round_trip_through_core_public_types() {
    let generated = generate_default_artifacts().unwrap();

    let burn_text = format_burn_curve(&generated.burn_curve);
    let parsed_burn = parse_burn_curve(&burn_text).unwrap();
    let core_burn = BurnCurve {
        burn_fraction_x10000: parsed_burn.burn_fraction_x10000,
    };
    let reexported_burn = format_burn_curve(&ExportedBurnCurve::from_quantized(
        core_burn.burn_fraction_x10000,
    ));
    assert_eq!(burn_text, reexported_burn);

    let ve_text = format_ve_table(&generated.ve_table);
    let parsed_ve = parse_ve_table(&ve_text).unwrap();
    let core_ve = VeTable {
        rpm_axis: parsed_ve.rpm_axis,
        load_axis: parsed_ve.load_axis_kpa10,
        ve_x1000: parsed_ve.ve_x1000,
    };
    let reexported_ve = format_ve_table(&ecu_sim_hifi::ExportedVeTable::new(
        core_ve.rpm_axis,
        core_ve.load_axis,
        core_ve.ve_x1000,
    ));
    assert_eq!(ve_text, reexported_ve);

    let loss_text = format_loss_config(&generated.loss_config);
    let parsed_loss = parse_loss_config(&loss_text).unwrap();
    let core_loss = ExportedLossConfig {
        fmep_base_pa: parsed_loss.fmep_base_pa,
        fmep_rpm_pa_per_krpm: parsed_loss.fmep_rpm_pa_per_krpm,
        fmep_rpm2_pa_per_krpm2: parsed_loss.fmep_rpm2_pa_per_krpm2,
        fmep_load_pa_per_kpa: parsed_loss.fmep_load_pa_per_kpa,
        pumping_base_pa: parsed_loss.pumping_base_pa,
        pumping_throttle_pa_per_x1000: parsed_loss.pumping_throttle_pa_per_x1000,
        accessory_torque_nm_x100: parsed_loss.accessory_torque_nm_x100,
    };
    let reexported_loss = format_loss_config(&core_loss);
    assert_eq!(loss_text, reexported_loss);
}

#[test]
fn committed_artifacts_match_deterministic_generation_byte_for_byte() {
    let generated = generate_default_artifacts().unwrap();

    assert_eq!(
        format_burn_curve(&generated.burn_curve),
        include_str!("../artifacts/burn_curve.txt")
    );
    assert_eq!(
        format_ve_table(&generated.ve_table),
        include_str!("../artifacts/ve_table.txt")
    );
    assert_eq!(
        format_loss_config(&generated.loss_config),
        include_str!("../artifacts/loss_config.txt")
    );
}

#[test]
fn default_plant_config_derives_from_source_defaults() {
    let config = default_plant_config();
    let open = default_open_system_export_config();
    let losses = default_loss_config_export();

    assert_eq!(config.validate(), Ok(()));
    assert_eq!(config.geometry, open.geometry);
    assert_eq!(config.manifold, open.manifold);
    assert_eq!(
        config.combustion.burn_model,
        default_burn_model_export_config()
    );
    assert_eq!(config.losses.fmep_base_pa, f64::from(losses.fmep_base_pa));
    assert_eq!(
        config.losses.fmep_rpm_pa_per_krpm,
        f64::from(losses.fmep_rpm_pa_per_krpm)
    );
    assert_eq!(
        config.losses.fmep_rpm2_pa_per_krpm2,
        f64::from(losses.fmep_rpm2_pa_per_krpm2)
    );
    assert_eq!(
        config.losses.fmep_load_pa_per_kpa,
        f64::from(losses.fmep_load_pa_per_kpa)
    );
}
