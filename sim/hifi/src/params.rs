use core::f64::consts::PI;

const ENGINE_CYCLE_RAD: f64 = 4.0 * core::f64::consts::PI;

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct GasProperties {
    pub r_j_per_kg_k: f64,
    pub cv_j_per_kg_k: f64,
}

impl GasProperties {
    pub const fn new(r_j_per_kg_k: f64, cv_j_per_kg_k: f64) -> Self {
        Self {
            r_j_per_kg_k,
            cv_j_per_kg_k,
        }
    }

    pub fn gamma(self) -> f64 {
        (self.cv_j_per_kg_k + self.r_j_per_kg_k) / self.cv_j_per_kg_k
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CylinderGeometry {
    pub bore_m: f64,
    pub stroke_m: f64,
    pub rod_length_m: f64,
    pub compression_ratio: f64,
}

impl CylinderGeometry {
    pub const fn new(
        bore_m: f64,
        stroke_m: f64,
        rod_length_m: f64,
        compression_ratio: f64,
    ) -> Self {
        Self {
            bore_m,
            stroke_m,
            rod_length_m,
            compression_ratio,
        }
    }

    pub fn piston_area_m2(self) -> f64 {
        PI * self.bore_m * self.bore_m / 4.0
    }

    pub fn swept_volume_m3(self) -> f64 {
        self.piston_area_m2() * self.stroke_m
    }

    pub fn clearance_volume_m3(self) -> f64 {
        self.swept_volume_m3() / (self.compression_ratio - 1.0)
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ThermalBoundary {
    pub wall_temperature_k: f64,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct InitialChargeState {
    pub pressure_pa: f64,
    pub temperature_k: f64,
    pub crank_angle_rad: f64,
}

impl InitialChargeState {
    pub fn crank_angle_deg(self) -> f64 {
        self.crank_angle_rad.to_degrees()
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct IntegratorConfig {
    pub step_deg: f64,
}

impl IntegratorConfig {
    pub fn step_rad(self) -> f64 {
        self.step_deg.to_radians()
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct MotoredCylinderConfig {
    pub geometry: CylinderGeometry,
    pub wall: ThermalBoundary,
    pub initial_charge: InitialChargeState,
    pub gas: GasProperties,
    pub integrator: IntegratorConfig,
}

impl MotoredCylinderConfig {
    pub fn validate(self) -> Result<(), MotoredConfigError> {
        if !(self.geometry.bore_m > 0.0
            && self.geometry.stroke_m > 0.0
            && self.geometry.rod_length_m > self.geometry.stroke_m / 2.0)
        {
            return Err(MotoredConfigError::InvalidGeometry);
        }
        if self.geometry.compression_ratio <= 1.0 {
            return Err(MotoredConfigError::InvalidCompressionRatio);
        }
        if !(self.initial_charge.pressure_pa > 0.0 && self.initial_charge.temperature_k > 0.0) {
            return Err(MotoredConfigError::InvalidInitialCharge);
        }
        if !(self.gas.r_j_per_kg_k > 0.0 && self.gas.cv_j_per_kg_k > 0.0) {
            return Err(MotoredConfigError::InvalidGasProperties);
        }
        if self.integrator.step_deg <= 0.0 {
            return Err(MotoredConfigError::InvalidStepSize);
        }
        Ok(())
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum MotoredConfigError {
    InvalidGeometry,
    InvalidCompressionRatio,
    InvalidInitialCharge,
    InvalidGasProperties,
    InvalidStepSize,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ValveTiming {
    pub open_angle_rad: f64,
    pub close_angle_rad: f64,
    pub max_lift_m: f64,
    pub seat_diameter_m: f64,
    pub discharge_coefficient: f64,
}

impl ValveTiming {
    pub fn duration_rad(self) -> f64 {
        (self.close_angle_rad - self.open_angle_rad).max(0.0)
    }

    pub fn seat_area_m2(self) -> f64 {
        PI * self.seat_diameter_m * self.seat_diameter_m / 4.0
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ThrottleConfig {
    pub max_area_m2: f64,
    pub discharge_coefficient: f64,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ManifoldConfig {
    pub volume_m3: f64,
    pub temperature_k: f64,
    pub ambient_pressure_pa: f64,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ResidualConfig {
    pub residual_temperature_k: f64,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct OpenSystemConfig {
    pub geometry: CylinderGeometry,
    pub fresh_gas: GasProperties,
    pub burned_gas: GasProperties,
    pub integrator: IntegratorConfig,
    pub manifold: ManifoldConfig,
    pub throttle: ThrottleConfig,
    pub throttle_position: f64,
    pub intake_valve: ValveTiming,
    pub exhaust_valve: ValveTiming,
    pub residual: ResidualConfig,
    pub initial_cylinder_pressure_pa: f64,
    pub initial_cylinder_temperature_k: f64,
    pub initial_residual_fraction: f64,
    pub exhaust_backpressure_pa: f64,
    pub exhaust_temperature_k: f64,
    pub rpm: f64,
    pub trapped_mass_tolerance_kg: f64,
    pub residual_tolerance: f64,
    pub max_cycles: usize,
}

impl OpenSystemConfig {
    pub fn validate(self) -> Result<(), OpenSystemConfigError> {
        if !(self.geometry.bore_m > 0.0
            && self.geometry.stroke_m > 0.0
            && self.geometry.rod_length_m > self.geometry.stroke_m / 2.0)
        {
            return Err(OpenSystemConfigError::InvalidGeometry);
        }
        if self.geometry.compression_ratio <= 1.0 {
            return Err(OpenSystemConfigError::InvalidCompressionRatio);
        }
        if !(self.fresh_gas.r_j_per_kg_k > 0.0
            && self.fresh_gas.cv_j_per_kg_k > 0.0
            && self.burned_gas.r_j_per_kg_k > 0.0
            && self.burned_gas.cv_j_per_kg_k > 0.0)
        {
            return Err(OpenSystemConfigError::InvalidGasProperties);
        }
        if self.integrator.step_deg <= 0.0 {
            return Err(OpenSystemConfigError::InvalidStepSize);
        }
        if !(self.manifold.volume_m3 > 0.0
            && self.manifold.temperature_k > 0.0
            && self.manifold.ambient_pressure_pa > 0.0)
        {
            return Err(OpenSystemConfigError::InvalidManifold);
        }
        if !(0.0..=1.0).contains(&self.throttle_position) {
            return Err(OpenSystemConfigError::InvalidThrottlePosition);
        }
        if !(self.throttle.max_area_m2 > 0.0 && self.throttle.discharge_coefficient > 0.0) {
            return Err(OpenSystemConfigError::InvalidThrottle);
        }
        if !valid_valve_timing(self.intake_valve) || !valid_valve_timing(self.exhaust_valve) {
            return Err(OpenSystemConfigError::InvalidValveTiming);
        }
        if !(self.initial_cylinder_pressure_pa > 0.0
            && self.initial_cylinder_temperature_k > 0.0
            && self.exhaust_backpressure_pa > 0.0
            && self.exhaust_temperature_k > 0.0)
        {
            return Err(OpenSystemConfigError::InvalidBoundaryState);
        }
        if !(0.0..=1.0).contains(&self.initial_residual_fraction) {
            return Err(OpenSystemConfigError::InvalidResidualFraction);
        }
        if self.rpm <= 0.0 {
            return Err(OpenSystemConfigError::InvalidRpm);
        }
        if !(self.trapped_mass_tolerance_kg > 0.0
            && self.residual_tolerance > 0.0
            && self.max_cycles > 0)
        {
            return Err(OpenSystemConfigError::InvalidConvergenceConfig);
        }
        Ok(())
    }
}

fn valid_valve_timing(timing: ValveTiming) -> bool {
    timing.close_angle_rad > timing.open_angle_rad
        && timing.max_lift_m >= 0.0
        && timing.seat_diameter_m > 0.0
        && timing.discharge_coefficient > 0.0
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum OpenSystemConfigError {
    InvalidGeometry,
    InvalidCompressionRatio,
    InvalidGasProperties,
    InvalidStepSize,
    InvalidManifold,
    InvalidThrottlePosition,
    InvalidThrottle,
    InvalidValveTiming,
    InvalidBoundaryState,
    InvalidResidualFraction,
    InvalidRpm,
    InvalidConvergenceConfig,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum BurnModel {
    SingleWiebe {
        a: f64,
        m: f64,
        duration_rad: f64,
        spark_to_soc_delay_rad: f64,
    },
    DoubleWiebe {
        premixed_fraction: f64,
        premixed_a: f64,
        premixed_m: f64,
        premixed_duration_rad: f64,
        main_a: f64,
        main_m: f64,
        main_duration_rad: f64,
        spark_to_soc_delay_rad: f64,
    },
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CombustionConfig {
    pub burn_model: BurnModel,
    pub spark_angle_rad: f64,
    pub fuel_mass_kg: f64,
    pub fuel_lhv_j_per_kg: f64,
    pub combustion_efficiency: f64,
    pub stoich_afr: f64,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct WoschniConfig {
    pub c: f64,
    pub c1: f64,
    pub c2: f64,
    pub t_ref_k: f64,
    pub p_ref_pa: f64,
    pub v_ref_m3: f64,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct LossCorrelationConfig {
    pub fmep_base_pa: f64,
    pub fmep_rpm_pa_per_krpm: f64,
    pub fmep_rpm2_pa_per_krpm2: f64,
    pub fmep_load_pa_per_kpa: f64,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct FiredCycleConfig {
    pub geometry: CylinderGeometry,
    pub wall: ThermalBoundary,
    pub gas: GasProperties,
    pub burned_gas: GasProperties,
    pub integrator: IntegratorConfig,
    pub initial_pressure_pa: f64,
    pub initial_temperature_k: f64,
    pub initial_mass_kg: Option<f64>,
    pub initial_burned_fraction: f64,
    pub rpm: f64,
    pub closed_cycle_start_rad: f64,
    pub closed_cycle_end_rad: f64,
    pub manifold_pressure_pa: f64,
    pub open_system_pmep_pa: f64,
    pub combustion: CombustionConfig,
    pub woschni: WoschniConfig,
    pub losses: LossCorrelationConfig,
}

impl FiredCycleConfig {
    pub fn validate(self) -> Result<(), FiredCycleConfigError> {
        if !(self.geometry.bore_m > 0.0
            && self.geometry.stroke_m > 0.0
            && self.geometry.rod_length_m > self.geometry.stroke_m / 2.0)
        {
            return Err(FiredCycleConfigError::InvalidGeometry);
        }
        if self.geometry.compression_ratio <= 1.0 {
            return Err(FiredCycleConfigError::InvalidCompressionRatio);
        }
        if !(self.gas.r_j_per_kg_k > 0.0
            && self.gas.cv_j_per_kg_k > 0.0
            && self.burned_gas.r_j_per_kg_k > 0.0
            && self.burned_gas.cv_j_per_kg_k > 0.0)
        {
            return Err(FiredCycleConfigError::InvalidGasProperties);
        }
        if self.integrator.step_deg <= 0.0 {
            return Err(FiredCycleConfigError::InvalidStepSize);
        }
        if !(self.initial_pressure_pa > 0.0
            && self.initial_temperature_k > 0.0
            && self
                .initial_mass_kg
                .is_none_or(|mass| mass.is_finite() && mass > 0.0)
            && self.manifold_pressure_pa > 0.0
            && self.rpm > 0.0
            && self.closed_cycle_start_rad.is_finite()
            && self.closed_cycle_end_rad.is_finite()
            && self.closed_cycle_end_rad > self.closed_cycle_start_rad)
        {
            return Err(FiredCycleConfigError::InvalidInitialState);
        }
        if !(0.0..=1.0).contains(&self.initial_burned_fraction) {
            return Err(FiredCycleConfigError::InvalidInitialBurnedFraction);
        }
        if !(self.combustion.fuel_mass_kg >= 0.0
            && self.combustion.fuel_lhv_j_per_kg > 0.0
            && (0.0..=1.0).contains(&self.combustion.combustion_efficiency)
            && self.combustion.stoich_afr > 0.0)
        {
            return Err(FiredCycleConfigError::InvalidCombustion);
        }
        if !valid_burn_model(self.combustion.burn_model) {
            return Err(FiredCycleConfigError::InvalidBurnModel);
        }
        if !(self.woschni.c > 0.0
            && self.woschni.c1 >= 0.0
            && self.woschni.c2 >= 0.0
            && self.woschni.t_ref_k > 0.0
            && self.woschni.p_ref_pa > 0.0
            && self.woschni.v_ref_m3 > 0.0)
        {
            return Err(FiredCycleConfigError::InvalidWoschni);
        }
        Ok(())
    }
}

fn valid_burn_model(model: BurnModel) -> bool {
    match model {
        BurnModel::SingleWiebe {
            a, m, duration_rad, ..
        } => a > 0.0 && m >= 0.0 && duration_rad > 0.0,
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
            (0.0..=1.0).contains(&premixed_fraction)
                && premixed_a > 0.0
                && premixed_m >= 0.0
                && premixed_duration_rad > 0.0
                && main_a > 0.0
                && main_m >= 0.0
                && main_duration_rad > 0.0
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum FiredCycleConfigError {
    InvalidGeometry,
    InvalidCompressionRatio,
    InvalidGasProperties,
    InvalidStepSize,
    InvalidInitialState,
    InvalidInitialBurnedFraction,
    InvalidCombustion,
    InvalidBurnModel,
    InvalidWoschni,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct KnockModelConfig {
    pub octane_number: f64,
    pub a: f64,
    pub n1: f64,
    pub n2: f64,
    pub b: f64,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct InjectorConfig {
    pub injector_flow_kg_per_s: f64,
    pub injector_deadtime_s: f64,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SparkConfig {
    pub dwell_s: f64,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PlantCylinderConfig {
    pub phase_offset_rad: f64,
}

#[derive(Clone, Debug, PartialEq)]
pub struct PlantConfig {
    pub geometry: CylinderGeometry,
    pub gas: GasProperties,
    pub burned_gas: GasProperties,
    pub wall: ThermalBoundary,
    pub integrator: IntegratorConfig,
    pub manifold: ManifoldConfig,
    pub throttle: ThrottleConfig,
    pub intake_valve: ValveTiming,
    pub exhaust_valve: ValveTiming,
    pub residual: ResidualConfig,
    pub combustion: CombustionConfig,
    pub woschni: WoschniConfig,
    pub losses: LossCorrelationConfig,
    pub injector: InjectorConfig,
    pub spark: SparkConfig,
    pub cylinders: Vec<PlantCylinderConfig>,
    pub initial_pressure_pa: f64,
    pub initial_temperature_k: f64,
    pub initial_residual_fraction: f64,
    pub exhaust_backpressure_pa: f64,
    pub exhaust_temperature_k: f64,
    pub crank_inertia_kg_m2: f64,
}

impl PlantConfig {
    pub fn validate(&self) -> Result<(), PlantConfigError> {
        MotoredCylinderConfig {
            geometry: self.geometry,
            wall: self.wall,
            initial_charge: InitialChargeState {
                pressure_pa: self.initial_pressure_pa,
                temperature_k: self.initial_temperature_k,
                crank_angle_rad: 0.0,
            },
            gas: self.gas,
            integrator: self.integrator,
        }
        .validate()
        .map_err(|_| PlantConfigError::InvalidModelConfig)?;

        OpenSystemConfig {
            geometry: self.geometry,
            fresh_gas: self.gas,
            burned_gas: self.burned_gas,
            integrator: self.integrator,
            manifold: self.manifold,
            throttle: self.throttle,
            throttle_position: 0.5,
            intake_valve: self.intake_valve,
            exhaust_valve: self.exhaust_valve,
            residual: self.residual,
            initial_cylinder_pressure_pa: self.initial_pressure_pa,
            initial_cylinder_temperature_k: self.initial_temperature_k,
            initial_residual_fraction: self.initial_residual_fraction,
            exhaust_backpressure_pa: self.exhaust_backpressure_pa,
            exhaust_temperature_k: self.exhaust_temperature_k,
            rpm: 2500.0,
            trapped_mass_tolerance_kg: 1.0e-6,
            residual_tolerance: 1.0e-4,
            max_cycles: 1,
        }
        .validate()
        .map_err(|_| PlantConfigError::InvalidModelConfig)?;

        FiredCycleConfig {
            geometry: self.geometry,
            wall: self.wall,
            gas: self.gas,
            burned_gas: self.burned_gas,
            integrator: self.integrator,
            initial_pressure_pa: self.initial_pressure_pa,
            initial_temperature_k: self.initial_temperature_k,
            initial_mass_kg: None,
            initial_burned_fraction: self.initial_residual_fraction,
            rpm: 2500.0,
            closed_cycle_start_rad: self.intake_valve.close_angle_rad,
            closed_cycle_end_rad: if self.exhaust_valve.open_angle_rad
                <= self.intake_valve.close_angle_rad
            {
                self.exhaust_valve.open_angle_rad + ENGINE_CYCLE_RAD
            } else {
                self.exhaust_valve.open_angle_rad
            },
            manifold_pressure_pa: self.manifold.ambient_pressure_pa,
            open_system_pmep_pa: 0.0,
            combustion: self.combustion,
            woschni: self.woschni,
            losses: self.losses,
        }
        .validate()
        .map_err(|_| PlantConfigError::InvalidModelConfig)?;

        if self.cylinders.is_empty() {
            return Err(PlantConfigError::InvalidCylinderCount);
        }
        if !(self.injector.injector_flow_kg_per_s > 0.0 && self.injector.injector_deadtime_s >= 0.0)
        {
            return Err(PlantConfigError::InvalidInjector);
        }
        if self.spark.dwell_s < 0.0 {
            return Err(PlantConfigError::InvalidSpark);
        }
        if self
            .cylinders
            .iter()
            .any(|cylinder| !cylinder.phase_offset_rad.is_finite())
        {
            return Err(PlantConfigError::InvalidCylinderPhase);
        }
        if !(self.crank_inertia_kg_m2.is_finite() && self.crank_inertia_kg_m2 > 0.0) {
            return Err(PlantConfigError::InvalidCrankInertia);
        }
        Ok(())
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PlantConfigError {
    InvalidCylinderCount,
    InvalidInjector,
    InvalidSpark,
    InvalidCylinderPhase,
    InvalidThrottlePosition,
    InvalidTimingWindow,
    InvalidModelConfig,
    InvalidCrankInertia,
}
