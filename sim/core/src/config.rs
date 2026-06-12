use crate::types::*;

const DEFAULT_HIFI_BURN_CURVE_X10000: [u16; WIEBE_POINTS] = [
    0, 2, 12, 41, 97, 189, 324, 510, 752, 1053, 1415, 1838, 2318, 2848, 3421, 4025, 4647, 5275,
    5893, 6489, 7050, 7566, 8030, 8438, 8787, 9078, 9316, 9504, 9649, 9758, 9838, 9894, 10000,
];

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct PlantConfig<const CYL: usize> {
    pub cylinder_count: u8,
    pub cylinder_phase_deg10: [CrankDeg10; CYL],
    pub displacement_cc: u32,
    pub compression_ratio_x100: u16,
    pub crank_inertia_x1000: u32,
    pub friction_torque_nm_x100: TorqueNmX100,
    pub starter_torque_nm_x100: TorqueNmX100,
    pub physics_mode: PlantPhysicsMode,
    pub engine: EngineGeometryConfig,
    pub crank: CrankConfig,
    pub trigger: TriggerConfig,
    pub air: AirConfig,
    pub fuel: FuelConfig,
    pub spark: SparkConfig,
    pub combustion: CombustionConfig,
    pub valve_events: ValveEvents,
    pub residual: ResidualGasConfig,
    pub thermo: ThermoConfig,
    pub losses: LossConfig,
    pub knock: KnockConfig,
    pub dyno: DynoConfig,
    pub sensors: SensorConfig,
}

impl PlantConfig<4> {
    pub const fn default_four() -> Self {
        Self {
            cylinder_count: 4,
            cylinder_phase_deg10: [
                CrankDeg10(0),
                CrankDeg10(1800),
                CrankDeg10(3600),
                CrankDeg10(5400),
            ],
            displacement_cc: 2000,
            compression_ratio_x100: 1000,
            crank_inertia_x1000: 1000,
            friction_torque_nm_x100: TorqueNmX100(1200),
            starter_torque_nm_x100: TorqueNmX100(5000),
            physics_mode: PlantPhysicsMode::SyntheticTorque,
            engine: EngineGeometryConfig {
                bore_um: 86_000,
                stroke_um: 86_000,
                rod_length_um: 143_000,
                displacement_cc: 2000,
                compression_ratio_x100: 1000,
                displacement_tolerance_pct: 5,
            },
            crank: CrankConfig {
                inertia_kg_m2_x1e6: 1_000_000,
                starter_torque_nm_x100: TorqueNmX100(5000),
                min_substep_us: Micros(10),
                max_substep_us: Micros(1000),
                max_substep_deg10: 5,
            },
            trigger: TriggerConfig {
                crank_teeth: 36,
                missing_teeth: 1,
                cam_pulses: 1,
            },
            air: AirConfig {
                load_mode: LoadMode::SpeedDensity,
                idle_map_kpa10: Kpa10(350),
                wide_open_map_kpa10: Kpa10(1000),
                manifold_volume_cc: 2000,
                manifold_filling_enabled: false,
                throttle_area_mm2: 1800,
                throttle_discharge_coeff_x1000: 700,
                ve_table: VeTable::constant(1000),
                reference_air_temp_k10: Kelvin10(2930),
                reference_pressure_kpa10: Kpa10(1013),
            },
            fuel: FuelConfig {
                enabled: true,
                stoich_afr_x100: 1470,
                fuel_lhv_j_per_kg: 43_000_000,
                wall_film_enabled: false,
                wall_film_deposit_x1000: 0,
                wall_film_tau_ms: 100,
            },
            spark: SparkConfig {
                min_dwell_us: Micros(1000),
                mbt_deg10: Degrees10(180),
                max_advance_deg10: Degrees10(450),
                ignition_delay_deg10: 50,
            },
            combustion: CombustionConfig {
                min_air_mass_ug: MassUg(1000),
                min_lambda_x1000: 700,
                max_lambda_x1000: 1300,
                torque_scale_x100: 100,
                burn_duration_deg10: 450,
                ca50_target_at_mbt_deg10: 100,
                pmax_target_at_mbt_deg10: 160,
                ca50_sensitivity_x1000: 3,
                pmax_sensitivity_x1000: 1,
                burn_curve: BurnCurve::default_hifi_generated(),
            },
            valve_events: ValveEvents {
                ivo_deg_btdc_x10: 0,
                ivc_deg_abdc_x10: 500,
                evo_deg_bbdc_x10: 500,
                evc_deg_atdc_x10: 0,
                intake_lift_mm_x100: 900,
                exhaust_lift_mm_x100: 850,
                intake_duration_deg_x10: 2400,
                exhaust_duration_deg_x10: 2350,
            },
            residual: ResidualGasConfig {
                enabled: false,
                base_fraction_x1000: 50,
                overlap_gain_x1000: 2,
                low_map_gain_x1000: 1,
                exhaust_backpressure_gain_x1000: 1,
                scavenging_gain_x1000: 5,
            },
            thermo: ThermoConfig {
                p_ref_pa: PressurePa(101_325),
                initial_cylinder_pressure_pa: PressurePa(101_325),
                initial_cylinder_temp_k10: Kelvin10(2930),
                gamma_x1000: 1350,
                r_air_j_per_kg_k: 287,
                cv_air_j_per_kg_k: 718,
                wall_heat_loss_x1000: 0,
            },
            losses: LossConfig {
                fmep_base_pa: PressurePa(20_000),
                fmep_rpm_pa_per_krpm: 8_000,
                fmep_rpm2_pa_per_krpm2: 1_000,
                fmep_load_pa_per_kpa: 0,
                pumping_base_pa: PressurePa(5_000),
                pumping_throttle_pa_per_x1000: 0,
                accessory_torque_nm_x100: TorqueNmX100(0),
            },
            knock: KnockConfig {
                fuel_octane_x10: 950,
                risk_threshold_x1000: 650,
                pmax_weight_x1000: 1,
                temp_weight_x1000: 1,
                advance_weight_x1000: 1,
                compression_weight_x1000: 1,
                octane_credit_x1000: 1,
                rich_margin_credit_x1000: 1,
            },
            dyno: DynoConfig {
                mode: DynoMode::Disabled,
                fixed_load_torque_nm_x100: TorqueNmX100(0),
                target_rpm: Rpm(0),
                sweep_start_rpm: Rpm(0),
                sweep_end_rpm: Rpm(0),
                sweep_step_rpm: Rpm(0),
                hold_cycles_before_sample: 20,
                sample_cycles: 4,
                rpm_error_limit: Rpm(10),
                pid_kp_x1000: 1000,
                pid_ki_x1000: 0,
                pid_kd_x1000: 0,
            },
            sensors: SensorConfig {
                sensor_saturation_enabled: false,
                lambda_transport_enabled: false,
                lambda_delay_crank_deg: 720,
                lambda_sensor_tau_ms: 0,
                lambda_exhaust_mixing_x1000: 0,
            },
        }
    }
}

impl<const CYL: usize> PlantConfig<CYL> {
    pub fn validate(&self) -> Result<(), PlantConfigError> {
        if CYL > MAX_CYLINDERS {
            return Err(PlantConfigError::TooManyCylinders);
        }
        if self.cylinder_count == 0 || self.cylinder_count as usize > CYL {
            return Err(PlantConfigError::InvalidCylinderCount);
        }
        for i in 0..self.cylinder_count as usize {
            if !self.cylinder_phase_deg10[i].is_normalized() {
                return Err(PlantConfigError::PhaseOutOfRange);
            }
            for j in (i + 1)..self.cylinder_count as usize {
                if self.cylinder_phase_deg10[i] == self.cylinder_phase_deg10[j] {
                    return Err(PlantConfigError::DuplicateCylinderPhase);
                }
            }
        }
        if self.displacement_cc == 0 || self.engine.displacement_cc == 0 {
            return Err(PlantConfigError::InvalidDisplacement);
        }
        if self.compression_ratio_x100 == 0 || self.engine.compression_ratio_x100 <= 100 {
            return Err(PlantConfigError::InvalidCompression);
        }
        if self.crank_inertia_x1000 == 0 {
            return Err(PlantConfigError::InvalidInertia);
        }
        if self.engine.bore_um == 0
            || self.engine.stroke_um == 0
            || self.engine.rod_length_um <= self.engine.stroke_um / 2
        {
            return Err(PlantConfigError::InvalidGeometry);
        }
        if self.crank.inertia_kg_m2_x1e6 == 0
            || self.crank.min_substep_us.0 == 0
            || self.crank.max_substep_us.0 < self.crank.min_substep_us.0
            || self.crank.max_substep_deg10 == 0
        {
            return Err(PlantConfigError::InvalidCrankConfig);
        }
        if self.trigger.crank_teeth == 0
            || self.trigger.crank_teeth as u32 > CYCLE_DEG10
            || self.trigger.missing_teeth >= self.trigger.crank_teeth
        {
            return Err(PlantConfigError::InvalidTriggerPattern);
        }
        if self.fuel.enabled && self.fuel.stoich_afr_x100 == 0 {
            return Err(PlantConfigError::InvalidFuelConfig);
        }
        match self.physics_mode {
            PlantPhysicsMode::SyntheticTorque
            | PlantPhysicsMode::KinematicPressurePulse
            | PlantPhysicsMode::PolytropicWiebe
            | PlantPhysicsMode::SingleZoneIdealGas => {}
        }
        if !self.air.ve_table.is_valid() {
            return Err(PlantConfigError::InvalidVeTable);
        }
        if !self.combustion.burn_curve.is_valid() {
            return Err(PlantConfigError::InvalidBurnCurve);
        }
        Ok(())
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PlantPhysicsMode {
    SyntheticTorque,
    KinematicPressurePulse,
    PolytropicWiebe,
    SingleZoneIdealGas,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct EngineGeometryConfig {
    pub bore_um: u32,
    pub stroke_um: u32,
    pub rod_length_um: u32,
    pub displacement_cc: u32,
    pub compression_ratio_x100: u16,
    pub displacement_tolerance_pct: u8,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct CrankConfig {
    pub inertia_kg_m2_x1e6: u32,
    pub starter_torque_nm_x100: TorqueNmX100,
    pub min_substep_us: Micros,
    pub max_substep_us: Micros,
    pub max_substep_deg10: u16,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct TriggerConfig {
    pub crank_teeth: u16,
    pub missing_teeth: u16,
    pub cam_pulses: u8,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct AirConfig {
    pub load_mode: LoadMode,
    pub idle_map_kpa10: Kpa10,
    pub wide_open_map_kpa10: Kpa10,
    pub manifold_volume_cc: u32,
    pub manifold_filling_enabled: bool,
    pub throttle_area_mm2: u32,
    pub throttle_discharge_coeff_x1000: u16,
    pub ve_table: VeTable,
    pub reference_air_temp_k10: Kelvin10,
    pub reference_pressure_kpa10: Kpa10,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum LoadMode {
    SpeedDensity,
    RelativeLoad,
    TpsMapBlend,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct VeTable {
    pub rpm_axis: [Rpm; MAX_TABLE_AXIS_POINTS],
    pub load_axis: [Kpa10; MAX_TABLE_AXIS_POINTS],
    pub ve_x1000: [[u16; MAX_TABLE_AXIS_POINTS]; MAX_TABLE_AXIS_POINTS],
}

impl VeTable {
    pub const fn constant(value_x1000: u16) -> Self {
        Self {
            rpm_axis: [
                Rpm(500),
                Rpm(1000),
                Rpm(1500),
                Rpm(2000),
                Rpm(3000),
                Rpm(4000),
                Rpm(5000),
                Rpm(6000),
            ],
            load_axis: [
                Kpa10(200),
                Kpa10(350),
                Kpa10(500),
                Kpa10(650),
                Kpa10(800),
                Kpa10(950),
                Kpa10(1100),
                Kpa10(1250),
            ],
            ve_x1000: [[value_x1000; MAX_TABLE_AXIS_POINTS]; MAX_TABLE_AXIS_POINTS],
        }
    }

    pub const fn is_valid(&self) -> bool {
        let mut i = 0;
        while i + 1 < MAX_TABLE_AXIS_POINTS {
            if self.rpm_axis[i].0 >= self.rpm_axis[i + 1].0
                || self.load_axis[i].0 >= self.load_axis[i + 1].0
            {
                return false;
            }
            i += 1;
        }

        true
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct FuelConfig {
    pub enabled: bool,
    pub stoich_afr_x100: u16,
    pub fuel_lhv_j_per_kg: u32,
    pub wall_film_enabled: bool,
    pub wall_film_deposit_x1000: u16,
    pub wall_film_tau_ms: u16,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct SparkConfig {
    pub min_dwell_us: Micros,
    pub mbt_deg10: Degrees10,
    pub max_advance_deg10: Degrees10,
    pub ignition_delay_deg10: u16,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct CombustionConfig {
    pub min_air_mass_ug: MassUg,
    pub min_lambda_x1000: u16,
    pub max_lambda_x1000: u16,
    pub torque_scale_x100: u16,
    pub burn_duration_deg10: u16,
    pub ca50_target_at_mbt_deg10: i16,
    pub pmax_target_at_mbt_deg10: i16,
    pub ca50_sensitivity_x1000: u16,
    pub pmax_sensitivity_x1000: u16,
    pub burn_curve: BurnCurve,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ValveEvents {
    pub ivo_deg_btdc_x10: i16,
    pub ivc_deg_abdc_x10: i16,
    pub evo_deg_bbdc_x10: i16,
    pub evc_deg_atdc_x10: i16,
    pub intake_lift_mm_x100: u16,
    pub exhaust_lift_mm_x100: u16,
    pub intake_duration_deg_x10: u16,
    pub exhaust_duration_deg_x10: u16,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ResidualGasConfig {
    pub enabled: bool,
    pub base_fraction_x1000: u16,
    pub overlap_gain_x1000: u16,
    pub low_map_gain_x1000: u16,
    pub exhaust_backpressure_gain_x1000: u16,
    pub scavenging_gain_x1000: u16,
}

pub const WIEBE_POINTS: usize = 33;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct BurnCurve {
    pub burn_fraction_x10000: [u16; WIEBE_POINTS],
}

impl BurnCurve {
    pub const fn default_hifi_generated() -> Self {
        Self {
            burn_fraction_x10000: DEFAULT_HIFI_BURN_CURVE_X10000,
        }
    }

    pub const fn is_valid(&self) -> bool {
        if self.burn_fraction_x10000[0] != 0 || self.burn_fraction_x10000[WIEBE_POINTS - 1] != 10000
        {
            return false;
        }

        let mut i = 1;
        while i < WIEBE_POINTS {
            if self.burn_fraction_x10000[i] < self.burn_fraction_x10000[i - 1]
                || self.burn_fraction_x10000[i] > 10000
            {
                return false;
            }
            i += 1;
        }
        true
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ThermoConfig {
    pub p_ref_pa: PressurePa,
    pub initial_cylinder_pressure_pa: PressurePa,
    pub initial_cylinder_temp_k10: Kelvin10,
    pub gamma_x1000: u16,
    pub r_air_j_per_kg_k: u16,
    pub cv_air_j_per_kg_k: u16,
    pub wall_heat_loss_x1000: u16,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct LossConfig {
    pub fmep_base_pa: PressurePa,
    pub fmep_rpm_pa_per_krpm: i32,
    pub fmep_rpm2_pa_per_krpm2: i32,
    pub fmep_load_pa_per_kpa: i32,
    pub pumping_base_pa: PressurePa,
    pub pumping_throttle_pa_per_x1000: i32,
    pub accessory_torque_nm_x100: TorqueNmX100,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct KnockConfig {
    pub fuel_octane_x10: u16,
    pub risk_threshold_x1000: u16,
    pub pmax_weight_x1000: u16,
    pub temp_weight_x1000: u16,
    pub advance_weight_x1000: u16,
    pub compression_weight_x1000: u16,
    pub octane_credit_x1000: u16,
    pub rich_margin_credit_x1000: u16,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum DynoMode {
    Disabled,
    FixedLoad,
    TargetRpmHold,
    TargetRpmSweep,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct DynoConfig {
    pub mode: DynoMode,
    pub fixed_load_torque_nm_x100: TorqueNmX100,
    pub target_rpm: Rpm,
    pub sweep_start_rpm: Rpm,
    pub sweep_end_rpm: Rpm,
    pub sweep_step_rpm: Rpm,
    pub hold_cycles_before_sample: u16,
    pub sample_cycles: u16,
    pub rpm_error_limit: Rpm,
    pub pid_kp_x1000: i32,
    pub pid_ki_x1000: i32,
    pub pid_kd_x1000: i32,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct SensorConfig {
    pub sensor_saturation_enabled: bool,
    pub lambda_transport_enabled: bool,
    pub lambda_delay_crank_deg: u16,
    pub lambda_sensor_tau_ms: u16,
    pub lambda_exhaust_mixing_x1000: u16,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PlantConfigError {
    InvalidCylinderCount,
    TooManyCylinders,
    DuplicateCylinderPhase,
    PhaseOutOfRange,
    InvalidDisplacement,
    InvalidCompression,
    InvalidInertia,
    InvalidGeometry,
    InvalidCrankConfig,
    InvalidTriggerPattern,
    InvalidFuelConfig,
    UnsupportedPhysicsMode,
    InvalidVeTable,
    InvalidBurnCurve,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn default_config_is_valid() {
        assert_eq!(PlantConfig::<4>::default_four().validate(), Ok(()));
    }

    #[test]
    fn duplicate_phase_is_rejected() {
        let mut cfg = PlantConfig::<4>::default_four();
        cfg.cylinder_phase_deg10[1] = cfg.cylinder_phase_deg10[0];
        assert_eq!(
            cfg.validate(),
            Err(PlantConfigError::DuplicateCylinderPhase)
        );
    }

    #[test]
    fn invalid_geometry_is_rejected() {
        let mut cfg = PlantConfig::<4>::default_four();
        cfg.engine.rod_length_um = cfg.engine.stroke_um / 2;

        assert_eq!(cfg.validate(), Err(PlantConfigError::InvalidGeometry));
    }

    #[test]
    fn invalid_burn_curve_is_rejected() {
        let mut cfg = PlantConfig::<4>::default_four();
        cfg.combustion.burn_curve.burn_fraction_x10000[4] = 1;

        assert_eq!(cfg.validate(), Err(PlantConfigError::InvalidBurnCurve));
    }

    #[test]
    fn invalid_ve_table_axis_is_rejected() {
        let mut cfg = PlantConfig::<4>::default_four();
        cfg.air.ve_table.rpm_axis[3] = cfg.air.ve_table.rpm_axis[2];

        assert_eq!(cfg.validate(), Err(PlantConfigError::InvalidVeTable));
    }

    #[test]
    fn all_declared_physics_modes_are_supported() {
        let mut cfg = PlantConfig::<4>::default_four();
        cfg.physics_mode = PlantPhysicsMode::SingleZoneIdealGas;

        assert_eq!(cfg.validate(), Ok(()));
    }

    #[test]
    fn default_burn_curve_matches_generated_hifi_artifact() {
        let generated = parse_burn_curve(include_str!("../../hifi/artifacts/burn_curve.txt"));

        assert_eq!(
            BurnCurve::default_hifi_generated().burn_fraction_x10000,
            generated.burn_fraction_x10000
        );
    }

    fn parse_burn_curve(contents: &str) -> ExportedBurnCurveArtifact {
        const EXPECTED_POINTS: usize = WIEBE_POINTS;

        let values = contents
            .lines()
            .find_map(|line| line.strip_prefix("burn_fraction_x10000="))
            .expect("burn fraction line should be present");

        let mut parsed = [0_u16; EXPECTED_POINTS];
        let mut count = 0_usize;

        for value in values.split(',') {
            assert!(
                count < EXPECTED_POINTS,
                "burn curve artifact has too many points"
            );
            parsed[count] = value
                .trim()
                .parse::<u16>()
                .expect("burn curve artifact values should parse as u16");
            count += 1;
        }

        assert_eq!(
            count, EXPECTED_POINTS,
            "burn curve artifact should be fixed size"
        );
        ExportedBurnCurveArtifact {
            burn_fraction_x10000: parsed,
        }
    }

    struct ExportedBurnCurveArtifact {
        burn_fraction_x10000: [u16; WIEBE_POINTS],
    }
}
