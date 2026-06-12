#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ClosedSystemState {
    pub mass_kg: f64,
    pub temperature_k: f64,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CompositionState {
    pub fresh_mass_kg: f64,
    pub burned_mass_kg: f64,
}

impl CompositionState {
    pub fn total_mass_kg(self) -> f64 {
        self.fresh_mass_kg + self.burned_mass_kg
    }

    pub fn residual_fraction(self) -> f64 {
        let total = self.total_mass_kg();
        if total <= 0.0 {
            0.0
        } else {
            self.burned_mass_kg / total
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct OpenSystemCylinderState {
    pub mass_kg: f64,
    pub temperature_k: f64,
    pub composition: CompositionState,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ManifoldState {
    pub mass_kg: f64,
    pub temperature_k: f64,
    pub composition: CompositionState,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CycleConvergence {
    pub iteration_count: usize,
    pub converged: bool,
    pub trapped_mass_delta_kg: f64,
    pub residual_fraction_delta: f64,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct MotoredSample {
    pub crank_angle_rad: f64,
    pub volume_m3: f64,
    pub pressure_pa: f64,
    pub temperature_k: f64,
    pub wall_area_m2: f64,
}

#[derive(Clone, Debug, PartialEq)]
pub struct MotoredCycleResult {
    pub samples: Vec<MotoredSample>,
    pub indicated_work_j: f64,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct OpenSystemSample {
    pub crank_angle_rad: f64,
    pub cylinder_pressure_pa: f64,
    pub manifold_pressure_pa: f64,
    pub cylinder_temperature_k: f64,
    pub manifold_mass_kg: f64,
    pub cylinder_mass_kg: f64,
    pub residual_fraction: f64,
}

#[derive(Clone, Debug, PartialEq)]
pub struct OpenSystemCycleResult {
    pub samples: Vec<OpenSystemSample>,
    pub pumping_work_j: f64,
    pub pmep_pa: f64,
    pub throttle_boundary_mass_kg: f64,
    pub exhaust_boundary_mass_kg: f64,
    pub trapped_fresh_mass_kg: f64,
    pub volumetric_efficiency: f64,
    pub residual_fraction: f64,
    pub cylinder_state: OpenSystemCylinderState,
    pub manifold_state: ManifoldState,
    pub boundary_enthalpy_j: f64,
    pub boundary_piston_work_j: f64,
    pub exhaust_enthalpy_j: f64,
    pub exhaust_enthalpy_temperature_j: f64,
}

#[derive(Clone, Debug, PartialEq)]
pub struct ConvergedOpenSystemResult {
    pub cycle: OpenSystemCycleResult,
    pub convergence: CycleConvergence,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct FiredCycleSample {
    pub crank_angle_rad: f64,
    pub pressure_pa: f64,
    pub temperature_k: f64,
    pub burn_fraction: f64,
    pub heat_release_rate_j_per_rad: f64,
    pub wall_heat_rate_j_per_rad: f64,
}

#[derive(Clone, Debug, PartialEq)]
pub struct FiredCycleResult {
    pub samples: Vec<FiredCycleSample>,
    pub closed_cycle_start_rad: f64,
    pub closed_cycle_end_rad: f64,
    pub imep_gross_pa: f64,
    pub pmep_pa: f64,
    pub fmep_pa: f64,
    pub bmep_pa: f64,
    pub brake_torque_nm: f64,
    pub lambda: f64,
    pub ca10_rad: Option<f64>,
    pub ca50_rad: Option<f64>,
    pub ca90_rad: Option<f64>,
    pub pmax_pa: f64,
    pub pmax_angle_rad: f64,
    pub indicated_work_j: f64,
    pub wall_heat_j: f64,
    pub exhaust_enthalpy_j: f64,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct KnockObservation {
    pub knock_integral: f64,
    pub knock_margin: f64,
    pub predicted_onset_angle_rad: Option<f64>,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ExhaustObservation {
    pub mean_exhaust_temperature_k: f64,
    pub exhaust_enthalpy_j: f64,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CylinderCommand {
    pub fuel_mass_kg: f64,
    pub spark_angle_rad: f64,
    pub dwell_s: f64,
}

#[derive(Clone, Debug, PartialEq)]
pub struct PlantStepInput {
    pub now_s: f64,
    pub window_s: f64,
    pub crank_angle_rad: f64,
    pub rpm: f64,
    pub throttle_position: f64,
    pub load_torque_nm: f64,
    pub cylinders: Vec<CylinderCommand>,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CylinderPlantTrace {
    pub brake_torque_nm: f64,
    pub lambda: f64,
    pub knock_margin: f64,
    pub egt_k: f64,
}

#[derive(Clone, Debug, PartialEq)]
pub struct PlantStepOutput {
    pub crank_angle_rad: f64,
    pub rpm: f64,
    pub manifold_pressure_pa: f64,
    pub lambda: f64,
    pub egt_k: f64,
    pub knock_margin: f64,
    pub brake_torque_nm: f64,
    pub cylinders: Vec<CylinderPlantTrace>,
}
