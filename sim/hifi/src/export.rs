use crate::{
    cylinder::converge_open_system_cycles,
    params::{
        BurnModel, CylinderGeometry, GasProperties, IntegratorConfig, ManifoldConfig,
        OpenSystemConfig, OpenSystemConfigError, ResidualConfig, ThrottleConfig, ValveTiming,
    },
};
use std::fs;
use std::path::Path;

pub const BURN_CURVE_POINTS: usize = 33;
pub const VE_TABLE_AXIS_POINTS: usize = 8;

pub const DEFAULT_VE_RPM_AXIS: [u32; VE_TABLE_AXIS_POINTS] =
    [500, 1000, 1500, 2000, 3000, 4000, 5000, 6000];
pub const DEFAULT_VE_LOAD_AXIS_KPA10: [u16; VE_TABLE_AXIS_POINTS] =
    [200, 350, 500, 650, 800, 950, 1100, 1250];
pub const ARTIFACT_FORMAT_VERSION: &str = "hifi-artifact-v1";

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ExportedBurnCurve {
    pub burn_fraction_x10000: [u16; BURN_CURVE_POINTS],
}

impl ExportedBurnCurve {
    pub fn from_burn_model(model: BurnModel) -> Self {
        let duration_rad = burn_duration_rad(model);
        let mut burn_fraction_x10000 = [0_u16; BURN_CURVE_POINTS];

        for (index, slot) in burn_fraction_x10000.iter_mut().enumerate() {
            *slot = if index == 0 {
                0
            } else if index + 1 == BURN_CURVE_POINTS {
                10_000
            } else {
                let elapsed_rad = duration_rad * index as f64 / (BURN_CURVE_POINTS - 1) as f64;
                quantize_fraction_x10000(burn_fraction_for_elapsed_rad(model, elapsed_rad))
            };
        }

        Self {
            burn_fraction_x10000,
        }
    }

    pub const fn from_quantized(burn_fraction_x10000: [u16; BURN_CURVE_POINTS]) -> Self {
        Self {
            burn_fraction_x10000,
        }
    }

    pub fn is_valid(self) -> bool {
        if self.burn_fraction_x10000[0] != 0
            || self.burn_fraction_x10000[BURN_CURVE_POINTS - 1] != 10_000
        {
            return false;
        }

        self.burn_fraction_x10000
            .windows(2)
            .all(|window| window[0] <= window[1] && window[1] <= 10_000)
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ExportedVeTable {
    pub rpm_axis: [u32; VE_TABLE_AXIS_POINTS],
    pub load_axis_kpa10: [u16; VE_TABLE_AXIS_POINTS],
    pub ve_x1000: [[u16; VE_TABLE_AXIS_POINTS]; VE_TABLE_AXIS_POINTS],
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct GeneratedArtifacts {
    pub burn_curve: ExportedBurnCurve,
    pub ve_table: ExportedVeTable,
    pub loss_config: ExportedLossConfig,
}

impl ExportedVeTable {
    pub const fn new(
        rpm_axis: [u32; VE_TABLE_AXIS_POINTS],
        load_axis_kpa10: [u16; VE_TABLE_AXIS_POINTS],
        ve_x1000: [[u16; VE_TABLE_AXIS_POINTS]; VE_TABLE_AXIS_POINTS],
    ) -> Self {
        Self {
            rpm_axis,
            load_axis_kpa10,
            ve_x1000,
        }
    }

    pub const fn default_axes() -> ([u32; VE_TABLE_AXIS_POINTS], [u16; VE_TABLE_AXIS_POINTS]) {
        (DEFAULT_VE_RPM_AXIS, DEFAULT_VE_LOAD_AXIS_KPA10)
    }

    pub fn is_valid(self) -> bool {
        self.rpm_axis.windows(2).all(|window| window[0] < window[1])
            && self
                .load_axis_kpa10
                .windows(2)
                .all(|window| window[0] < window[1])
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct LossFitSample {
    pub rpm: f64,
    pub load_kpa: f64,
    pub fmep_pa: f64,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PumpingFitSample {
    pub throttle_position: f64,
    pub pmep_pa: f64,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ExportedLossConfig {
    pub fmep_base_pa: i32,
    pub fmep_rpm_pa_per_krpm: i32,
    pub fmep_rpm2_pa_per_krpm2: i32,
    pub fmep_load_pa_per_kpa: i32,
    pub pumping_base_pa: i32,
    pub pumping_throttle_pa_per_x1000: i32,
    pub accessory_torque_nm_x100: i32,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum ExportError {
    InsufficientSamples,
    InvalidSample,
    SingularFit,
    Io(String),
    Parse(&'static str),
}

pub fn export_ve_table(
    base: OpenSystemConfig,
    rpm_axis: [u32; VE_TABLE_AXIS_POINTS],
    load_axis_kpa10: [u16; VE_TABLE_AXIS_POINTS],
) -> Result<ExportedVeTable, OpenSystemConfigError> {
    let mut ve_x1000 = [[0_u16; VE_TABLE_AXIS_POINTS]; VE_TABLE_AXIS_POINTS];

    for (rpm_index, rpm) in rpm_axis.iter().enumerate() {
        let mut manifold_map_kpa10 = Vec::with_capacity(17);
        let mut ve_by_map = Vec::with_capacity(17);
        for throttle_step in 0..=16 {
            let throttle_position = 1.0 - (throttle_step as f64) / 16.0;
            let mut point = base;
            point.rpm = f64::from(*rpm);
            point.throttle_position = throttle_position;
            let cycle = converge_open_system_cycles(point)?;
            let map_kpa10 = cycle
                .cycle
                .samples
                .last()
                .map(|sample| sample.manifold_pressure_pa / 100.0)
                .unwrap_or(0.0);
            let ve = cycle.cycle.volumetric_efficiency;
            manifold_map_kpa10.push(map_kpa10);
            ve_by_map.push(ve);
        }

        for (load_index, load_kpa10) in load_axis_kpa10.iter().enumerate() {
            let sampled_ve = match interpolate_ve_from_map_axis(
                *load_kpa10 as f64,
                &manifold_map_kpa10,
                &ve_by_map,
            ) {
                Some(ve) => ve,
                None => {
                    let last_ve = *ve_by_map.last().unwrap_or(&0.0);
                    last_ve
                }
            };
            ve_x1000[rpm_index][load_index] = quantize_ve_x1000(sampled_ve);
        }
    }

    Ok(ExportedVeTable::new(rpm_axis, load_axis_kpa10, ve_x1000))
}

fn interpolate_ve_from_map_axis(
    load_kpa10: f64,
    map_axis_kpa10: &[f64],
    ve_axis: &[f64],
) -> Option<f64> {
    if map_axis_kpa10.is_empty() || ve_axis.is_empty() {
        return None;
    }

    let mut pairs: Vec<(f64, f64)> = map_axis_kpa10
        .iter()
        .zip(ve_axis.iter())
        .map(|(map_kpa10, ve)| (*map_kpa10, *ve))
        .collect();
    pairs.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap_or(std::cmp::Ordering::Equal));

    if load_kpa10 <= pairs[0].0 {
        return Some(pairs[0].1);
    }
    if load_kpa10 >= pairs[pairs.len() - 1].0 {
        return Some(pairs[pairs.len() - 1].1);
    }

    for window in pairs.windows(2) {
        let (x0, y0) = window[0];
        let (x1, y1) = window[1];
        if (x0..=x1).contains(&load_kpa10) {
            let t = if (x1 - x0).abs() > 0.0 {
                (load_kpa10 - x0) / (x1 - x0)
            } else {
                0.0
            };
            return Some(y0 + t * (y1 - y0));
        }
    }
    None
}

pub fn fit_loss_config(
    fmep_samples: &[LossFitSample],
    pumping_samples: &[PumpingFitSample],
    accessory_torque_nm_x100: i32,
) -> Result<ExportedLossConfig, ExportError> {
    if fmep_samples.len() < 4 || pumping_samples.len() < 2 {
        return Err(ExportError::InsufficientSamples);
    }

    let mut fmep_rows = Vec::with_capacity(fmep_samples.len());
    let mut fmep_targets = Vec::with_capacity(fmep_samples.len());
    for sample in fmep_samples {
        if !sample.rpm.is_finite() || !sample.load_kpa.is_finite() || !sample.fmep_pa.is_finite() {
            return Err(ExportError::InvalidSample);
        }
        fmep_rows.push([
            1.0,
            sample.rpm / 1000.0,
            (sample.rpm / 1000.0).powi(2),
            sample.load_kpa,
        ]);
        fmep_targets.push(sample.fmep_pa);
    }

    let mut pumping_rows = Vec::with_capacity(pumping_samples.len());
    let mut pumping_targets = Vec::with_capacity(pumping_samples.len());
    for sample in pumping_samples {
        if !sample.throttle_position.is_finite()
            || !sample.pmep_pa.is_finite()
            || !(0.0..=1.0).contains(&sample.throttle_position)
        {
            return Err(ExportError::InvalidSample);
        }
        pumping_rows.push([1.0, 1000.0 * (1.0 - sample.throttle_position)]);
        pumping_targets.push(sample.pmep_pa);
    }

    let fmep = solve_normal_equations_4(&fmep_rows, &fmep_targets)?;
    let pumping = solve_normal_equations_2(&pumping_rows, &pumping_targets)?;

    Ok(ExportedLossConfig {
        fmep_base_pa: round_i32(fmep[0]),
        fmep_rpm_pa_per_krpm: round_i32(fmep[1]),
        fmep_rpm2_pa_per_krpm2: round_i32(fmep[2]),
        fmep_load_pa_per_kpa: round_i32(fmep[3]),
        pumping_base_pa: round_i32(pumping[0]),
        pumping_throttle_pa_per_x1000: round_i32(pumping[1]),
        accessory_torque_nm_x100,
    })
}

pub fn default_open_system_export_config() -> OpenSystemConfig {
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

/// Canonical hifi runtime default.
///
/// Committed artifacts are consumed by the downstream integer layers, not by
/// the hifi runtime path.
pub fn default_plant_config() -> crate::PlantConfig {
    let open = default_open_system_export_config();
    let losses = default_loss_config_export();

    crate::PlantConfig {
        geometry: open.geometry,
        gas: open.fresh_gas,
        burned_gas: open.burned_gas,
        wall: crate::ThermalBoundary {
            wall_temperature_k: 420.0,
        },
        integrator: open.integrator,
        manifold: open.manifold,
        throttle: open.throttle,
        intake_valve: open.intake_valve,
        exhaust_valve: open.exhaust_valve,
        residual: crate::ResidualConfig {
            residual_temperature_k: open.residual.residual_temperature_k,
        },
        combustion: crate::CombustionConfig {
            burn_model: default_burn_model_export_config(),
            spark_angle_rad: 18.0_f64.to_radians(),
            fuel_mass_kg: 1.8e-5,
            fuel_lhv_j_per_kg: 43.0e6,
            combustion_efficiency: 0.96,
            stoich_afr: 14.7,
        },
        woschni: crate::WoschniConfig {
            c: 8.0,
            c1: 2.28,
            c2: 0.00324,
            t_ref_k: 300.0,
            p_ref_pa: 101_325.0,
            v_ref_m3: 5.0e-4,
        },
        losses: crate::LossCorrelationConfig {
            fmep_base_pa: f64::from(losses.fmep_base_pa),
            fmep_rpm_pa_per_krpm: f64::from(losses.fmep_rpm_pa_per_krpm),
            fmep_rpm2_pa_per_krpm2: f64::from(losses.fmep_rpm2_pa_per_krpm2),
            fmep_load_pa_per_kpa: f64::from(losses.fmep_load_pa_per_kpa),
        },
        injector: crate::InjectorConfig {
            injector_flow_kg_per_s: 0.02,
            injector_deadtime_s: 0.0007,
        },
        spark: crate::SparkConfig { dwell_s: 0.002 },
        cylinders: vec![
            crate::PlantCylinderConfig {
                phase_offset_rad: 0.0,
            },
            crate::PlantCylinderConfig {
                phase_offset_rad: core::f64::consts::PI,
            },
            crate::PlantCylinderConfig {
                phase_offset_rad: core::f64::consts::TAU,
            },
            crate::PlantCylinderConfig {
                phase_offset_rad: 3.0 * core::f64::consts::PI,
            },
        ],
        initial_pressure_pa: open.initial_cylinder_pressure_pa,
        initial_temperature_k: open.initial_cylinder_temperature_k,
        initial_residual_fraction: open.initial_residual_fraction,
        exhaust_backpressure_pa: open.exhaust_backpressure_pa,
        exhaust_temperature_k: open.exhaust_temperature_k,
        crank_inertia_kg_m2: 0.2,
    }
}

pub fn default_burn_model_export_config() -> BurnModel {
    BurnModel::SingleWiebe {
        a: 5.0,
        m: 2.0,
        duration_rad: 45.0_f64.to_radians(),
        spark_to_soc_delay_rad: 5.0_f64.to_radians(),
    }
}

pub fn default_loss_config_export() -> ExportedLossConfig {
    fit_loss_config(
        &[
            LossFitSample {
                rpm: 1000.0,
                load_kpa: 20.0,
                fmep_pa: 29_000.0,
            },
            LossFitSample {
                rpm: 2000.0,
                load_kpa: 35.0,
                fmep_pa: 42_750.0,
            },
            LossFitSample {
                rpm: 3000.0,
                load_kpa: 60.0,
                fmep_pa: 56_000.0,
            },
            LossFitSample {
                rpm: 3000.0,
                load_kpa: 10.0,
                fmep_pa: 53_500.0,
            },
            LossFitSample {
                rpm: 4000.0,
                load_kpa: 80.0,
                fmep_pa: 72_000.0,
            },
            LossFitSample {
                rpm: 1500.0,
                load_kpa: 45.0,
                fmep_pa: 35_750.0,
            },
        ],
        &[
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
        ],
        0,
    )
    .expect("default export fit samples should be well-conditioned")
}

pub fn generate_default_artifacts() -> Result<GeneratedArtifacts, ExportError> {
    Ok(GeneratedArtifacts {
        burn_curve: ExportedBurnCurve::from_burn_model(default_burn_model_export_config()),
        ve_table: export_ve_table(
            default_open_system_export_config(),
            DEFAULT_VE_RPM_AXIS,
            DEFAULT_VE_LOAD_AXIS_KPA10,
        )
        .map_err(|_| ExportError::Parse("default open-system config should be valid"))?,
        loss_config: default_loss_config_export(),
    })
}

pub fn write_default_artifacts(dir: &Path) -> Result<(), ExportError> {
    let generated = generate_default_artifacts()?;
    fs::create_dir_all(dir).map_err(|err| ExportError::Io(err.to_string()))?;
    fs::write(
        dir.join("burn_curve.txt"),
        format_burn_curve(&generated.burn_curve),
    )
    .map_err(|err| ExportError::Io(err.to_string()))?;
    fs::write(
        dir.join("ve_table.txt"),
        format_ve_table(&generated.ve_table),
    )
    .map_err(|err| ExportError::Io(err.to_string()))?;
    fs::write(
        dir.join("loss_config.txt"),
        format_loss_config(&generated.loss_config),
    )
    .map_err(|err| ExportError::Io(err.to_string()))?;
    Ok(())
}

pub fn format_burn_curve(curve: &ExportedBurnCurve) -> String {
    let values = curve
        .burn_fraction_x10000
        .iter()
        .map(u16::to_string)
        .collect::<Vec<_>>()
        .join(",");
    format!(
        "# generated by ecu-sim-hifi\n# format={ARTIFACT_FORMAT_VERSION}\nkind=burn_curve\nburn_fraction_x10000={values}\n"
    )
}

pub fn format_ve_table(table: &ExportedVeTable) -> String {
    let rpm_axis = table
        .rpm_axis
        .iter()
        .map(u32::to_string)
        .collect::<Vec<_>>()
        .join(",");
    let load_axis = table
        .load_axis_kpa10
        .iter()
        .map(u16::to_string)
        .collect::<Vec<_>>()
        .join(",");
    let rows = table
        .ve_x1000
        .iter()
        .map(|row| row.iter().map(u16::to_string).collect::<Vec<_>>().join(","))
        .collect::<Vec<_>>()
        .join(";");
    format!(
        "# generated by ecu-sim-hifi\n# format={ARTIFACT_FORMAT_VERSION}\nkind=ve_table\nrpm_axis={rpm_axis}\nload_axis_kpa10={load_axis}\nve_x1000={rows}\n"
    )
}

pub fn format_loss_config(config: &ExportedLossConfig) -> String {
    format!(
        "# generated by ecu-sim-hifi\n# format={ARTIFACT_FORMAT_VERSION}\nkind=loss_config\nfmep_base_pa={}\nfmep_rpm_pa_per_krpm={}\nfmep_rpm2_pa_per_krpm2={}\nfmep_load_pa_per_kpa={}\npumping_base_pa={}\npumping_throttle_pa_per_x1000={}\naccessory_torque_nm_x100={}\n",
        config.fmep_base_pa,
        config.fmep_rpm_pa_per_krpm,
        config.fmep_rpm2_pa_per_krpm2,
        config.fmep_load_pa_per_kpa,
        config.pumping_base_pa,
        config.pumping_throttle_pa_per_x1000,
        config.accessory_torque_nm_x100,
    )
}

pub fn parse_burn_curve(text: &str) -> Result<ExportedBurnCurve, ExportError> {
    let values = parse_key_value(text, "burn_fraction_x10000")?;
    let parsed = parse_u16_array::<BURN_CURVE_POINTS>(values)?;
    Ok(ExportedBurnCurve::from_quantized(parsed))
}

pub fn parse_ve_table(text: &str) -> Result<ExportedVeTable, ExportError> {
    let rpm_axis = parse_u32_array::<VE_TABLE_AXIS_POINTS>(parse_key_value(text, "rpm_axis")?)?;
    let load_axis_kpa10 =
        parse_u16_array::<VE_TABLE_AXIS_POINTS>(parse_key_value(text, "load_axis_kpa10")?)?;
    let rows_text = parse_key_value(text, "ve_x1000")?;
    let rows = rows_text.split(';').collect::<Vec<_>>();
    if rows.len() != VE_TABLE_AXIS_POINTS {
        return Err(ExportError::Parse("ve table row count mismatch"));
    }
    let mut ve_x1000 = [[0_u16; VE_TABLE_AXIS_POINTS]; VE_TABLE_AXIS_POINTS];
    for (index, row) in rows.iter().enumerate() {
        ve_x1000[index] = parse_u16_array::<VE_TABLE_AXIS_POINTS>(row)?;
    }
    Ok(ExportedVeTable::new(rpm_axis, load_axis_kpa10, ve_x1000))
}

pub fn parse_loss_config(text: &str) -> Result<ExportedLossConfig, ExportError> {
    Ok(ExportedLossConfig {
        fmep_base_pa: parse_i32_value(parse_key_value(text, "fmep_base_pa")?)?,
        fmep_rpm_pa_per_krpm: parse_i32_value(parse_key_value(text, "fmep_rpm_pa_per_krpm")?)?,
        fmep_rpm2_pa_per_krpm2: parse_i32_value(parse_key_value(text, "fmep_rpm2_pa_per_krpm2")?)?,
        fmep_load_pa_per_kpa: parse_i32_value(parse_key_value(text, "fmep_load_pa_per_kpa")?)?,
        pumping_base_pa: parse_i32_value(parse_key_value(text, "pumping_base_pa")?)?,
        pumping_throttle_pa_per_x1000: parse_i32_value(parse_key_value(
            text,
            "pumping_throttle_pa_per_x1000",
        )?)?,
        accessory_torque_nm_x100: parse_i32_value(parse_key_value(
            text,
            "accessory_torque_nm_x100",
        )?)?,
    })
}

fn burn_duration_rad(model: BurnModel) -> f64 {
    match model {
        BurnModel::SingleWiebe { duration_rad, .. } => duration_rad,
        BurnModel::DoubleWiebe {
            premixed_duration_rad,
            main_duration_rad,
            ..
        } => premixed_duration_rad.max(main_duration_rad),
    }
}

fn burn_fraction_for_elapsed_rad(model: BurnModel, elapsed_rad: f64) -> f64 {
    match model {
        BurnModel::SingleWiebe {
            a, m, duration_rad, ..
        } => wiebe_component(elapsed_rad, a, m, duration_rad),
        BurnModel::DoubleWiebe {
            premixed_fraction,
            premixed_a,
            premixed_m,
            premixed_duration_rad,
            main_a,
            main_m,
            main_duration_rad,
            ..
        } => {
            premixed_fraction
                * wiebe_component(elapsed_rad, premixed_a, premixed_m, premixed_duration_rad)
                + (1.0 - premixed_fraction)
                    * wiebe_component(elapsed_rad, main_a, main_m, main_duration_rad)
        }
    }
}

fn wiebe_component(elapsed_rad: f64, a: f64, m: f64, duration_rad: f64) -> f64 {
    if elapsed_rad <= 0.0 {
        return 0.0;
    }
    if elapsed_rad >= duration_rad {
        return 1.0;
    }

    let normalized = (elapsed_rad / duration_rad).clamp(0.0, 1.0);
    1.0 - (-a * normalized.powf(m + 1.0)).exp()
}

fn quantize_fraction_x10000(fraction: f64) -> u16 {
    (fraction.clamp(0.0, 1.0) * 10_000.0).round() as u16
}

fn quantize_ve_x1000(ve: f64) -> u16 {
    (ve.max(0.0) * 1000.0).round().clamp(0.0, u16::MAX as f64) as u16
}

fn round_i32(value: f64) -> i32 {
    value.round().clamp(i32::MIN as f64, i32::MAX as f64) as i32
}

fn solve_normal_equations_4(rows: &[[f64; 4]], targets: &[f64]) -> Result<[f64; 4], ExportError> {
    let mut ata = [[0.0; 4]; 4];
    let mut atb = [0.0; 4];

    for (row, target) in rows.iter().zip(targets) {
        for i in 0..4 {
            atb[i] += row[i] * target;
            for j in 0..4 {
                ata[i][j] += row[i] * row[j];
            }
        }
    }

    gaussian_solve_4(ata, atb)
}

fn solve_normal_equations_2(rows: &[[f64; 2]], targets: &[f64]) -> Result<[f64; 2], ExportError> {
    let mut ata = [[0.0; 2]; 2];
    let mut atb = [0.0; 2];

    for (row, target) in rows.iter().zip(targets) {
        for i in 0..2 {
            atb[i] += row[i] * target;
            for j in 0..2 {
                ata[i][j] += row[i] * row[j];
            }
        }
    }

    gaussian_solve_2(ata, atb)
}

fn gaussian_solve_4(mut a: [[f64; 4]; 4], mut b: [f64; 4]) -> Result<[f64; 4], ExportError> {
    for pivot in 0..4 {
        let mut best = pivot;
        for row in (pivot + 1)..4 {
            if a[row][pivot].abs() > a[best][pivot].abs() {
                best = row;
            }
        }
        if a[best][pivot].abs() < 1.0e-12 {
            return Err(ExportError::SingularFit);
        }
        if best != pivot {
            a.swap(best, pivot);
            b.swap(best, pivot);
        }

        let pivot_value = a[pivot][pivot];
        let mut col = pivot;
        while col < 4 {
            a[pivot][col] /= pivot_value;
            col += 1;
        }
        b[pivot] /= pivot_value;

        for row in 0..4 {
            if row == pivot {
                continue;
            }
            let factor = a[row][pivot];
            let mut col = pivot;
            while col < 4 {
                a[row][col] -= factor * a[pivot][col];
                col += 1;
            }
            b[row] -= factor * b[pivot];
        }
    }

    Ok(b)
}

fn gaussian_solve_2(mut a: [[f64; 2]; 2], mut b: [f64; 2]) -> Result<[f64; 2], ExportError> {
    if a[0][0].abs() < a[1][0].abs() {
        a.swap(0, 1);
        b.swap(0, 1);
    }
    if a[0][0].abs() < 1.0e-12 {
        return Err(ExportError::SingularFit);
    }

    let factor = a[1][0] / a[0][0];
    a[1][1] -= factor * a[0][1];
    b[1] -= factor * b[0];
    if a[1][1].abs() < 1.0e-12 {
        return Err(ExportError::SingularFit);
    }

    let x1 = b[1] / a[1][1];
    let x0 = (b[0] - a[0][1] * x1) / a[0][0];
    Ok([x0, x1])
}

fn parse_key_value<'a>(text: &'a str, key: &str) -> Result<&'a str, ExportError> {
    text.lines()
        .find_map(|line| line.strip_prefix(&format!("{key}=")))
        .ok_or(ExportError::Parse("missing key"))
}

fn parse_u16_array<const N: usize>(csv: &str) -> Result<[u16; N], ExportError> {
    let mut values = [0_u16; N];
    let parts = csv.split(',').collect::<Vec<_>>();
    if parts.len() != N {
        return Err(ExportError::Parse("array length mismatch"));
    }
    for (index, part) in parts.iter().enumerate() {
        values[index] = part
            .trim()
            .parse()
            .map_err(|_| ExportError::Parse("invalid u16 value"))?;
    }
    Ok(values)
}

fn parse_u32_array<const N: usize>(csv: &str) -> Result<[u32; N], ExportError> {
    let mut values = [0_u32; N];
    let parts = csv.split(',').collect::<Vec<_>>();
    if parts.len() != N {
        return Err(ExportError::Parse("array length mismatch"));
    }
    for (index, part) in parts.iter().enumerate() {
        values[index] = part
            .trim()
            .parse()
            .map_err(|_| ExportError::Parse("invalid u32 value"))?;
    }
    Ok(values)
}

fn parse_i32_value(text: &str) -> Result<i32, ExportError> {
    text.trim()
        .parse()
        .map_err(|_| ExportError::Parse("invalid i32 value"))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn exported_burn_curve_is_monotonic_and_pinned() {
        let curve = ExportedBurnCurve::from_burn_model(BurnModel::SingleWiebe {
            a: 5.0,
            m: 2.0,
            duration_rad: 40.0_f64.to_radians(),
            spark_to_soc_delay_rad: 5.0_f64.to_radians(),
        });

        assert!(curve.is_valid());
        assert_eq!(curve.burn_fraction_x10000[0], 0);
        assert_eq!(curve.burn_fraction_x10000[BURN_CURVE_POINTS - 1], 10_000);
    }

    #[test]
    fn double_wiebe_export_reaches_full_burn() {
        let curve = ExportedBurnCurve::from_burn_model(BurnModel::DoubleWiebe {
            premixed_fraction: 0.35,
            premixed_a: 6.0,
            premixed_m: 2.5,
            premixed_duration_rad: 15.0_f64.to_radians(),
            main_a: 5.0,
            main_m: 1.5,
            main_duration_rad: 45.0_f64.to_radians(),
            spark_to_soc_delay_rad: 4.0_f64.to_radians(),
        });

        assert!(curve.is_valid());
        assert!(curve.burn_fraction_x10000[16] > 0);
        assert_eq!(curve.burn_fraction_x10000[BURN_CURVE_POINTS - 1], 10_000);
    }

    #[test]
    fn least_squares_recovers_loss_coefficients_from_exact_samples() {
        let fmep_samples = [
            LossFitSample {
                rpm: 1000.0,
                load_kpa: 30.0,
                fmep_pa: 20_000.0 + 8_000.0 * 1.0 + 1_000.0 * 1.0 + 50.0 * 30.0,
            },
            LossFitSample {
                rpm: 2000.0,
                load_kpa: 40.0,
                fmep_pa: 20_000.0 + 8_000.0 * 2.0 + 1_000.0 * 4.0 + 50.0 * 40.0,
            },
            LossFitSample {
                rpm: 3000.0,
                load_kpa: 60.0,
                fmep_pa: 20_000.0 + 8_000.0 * 3.0 + 1_000.0 * 9.0 + 50.0 * 60.0,
            },
            LossFitSample {
                rpm: 4000.0,
                load_kpa: 80.0,
                fmep_pa: 20_000.0 + 8_000.0 * 4.0 + 1_000.0 * 16.0 + 50.0 * 80.0,
            },
        ];
        let pumping_samples = [
            PumpingFitSample {
                throttle_position: 1.0,
                pmep_pa: 5_000.0,
            },
            PumpingFitSample {
                throttle_position: 0.0,
                pmep_pa: 15_000.0,
            },
        ];

        let fitted = fit_loss_config(&fmep_samples, &pumping_samples, 250).unwrap();

        assert_eq!(fitted.fmep_base_pa, 20_000);
        assert_eq!(fitted.fmep_rpm_pa_per_krpm, 8_000);
        assert_eq!(fitted.fmep_rpm2_pa_per_krpm2, 1_000);
        assert_eq!(fitted.fmep_load_pa_per_kpa, 50);
        assert_eq!(fitted.pumping_base_pa, 5_000);
        assert_eq!(fitted.pumping_throttle_pa_per_x1000, 10);
        assert_eq!(fitted.accessory_torque_nm_x100, 250);
    }

    #[test]
    fn exported_ve_table_preserves_axis_ordering() {
        let table = ExportedVeTable::new(
            DEFAULT_VE_RPM_AXIS,
            DEFAULT_VE_LOAD_AXIS_KPA10,
            [[1000; VE_TABLE_AXIS_POINTS]; VE_TABLE_AXIS_POINTS],
        );

        assert!(table.is_valid());
    }

    #[test]
    fn generated_artifacts_round_trip_through_text_format() {
        let generated = generate_default_artifacts().unwrap();

        assert_eq!(
            parse_burn_curve(&format_burn_curve(&generated.burn_curve)).unwrap(),
            generated.burn_curve
        );
        assert_eq!(
            parse_ve_table(&format_ve_table(&generated.ve_table)).unwrap(),
            generated.ve_table
        );
        assert_eq!(
            parse_loss_config(&format_loss_config(&generated.loss_config)).unwrap(),
            generated.loss_config
        );
    }
}
