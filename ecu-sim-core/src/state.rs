use crate::{exhaust::ExhaustThermalState, types::*};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum MisfireReason {
    NoFuel,
    NoSpark,
    TooLean,
    TooRich,
    BadSparkTiming,
    FuelCut,
    SparkCut,
    InvalidCylinder,
    InsufficientDwell,
    AirMassTooLow,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct CylinderState {
    pub fuel_mass_ug: MassUg,
    pub air_mass_ug: MassUg,
    pub last_injection_pw_us: Micros,
    pub last_soi_deg10: Option<CrankDeg10>,
    pub last_eoi_deg10: Option<CrankDeg10>,
    pub last_spark_advance_deg10: Degrees10,
    pub last_dwell_us: Micros,
    pub combustion_quality_x1000: u16,
    pub knock_risk_x1000: u16,
    pub misfire: Option<MisfireReason>,
}

impl CylinderState {
    pub const fn new() -> Self {
        Self {
            fuel_mass_ug: MassUg(0),
            air_mass_ug: MassUg(0),
            last_injection_pw_us: Micros(0),
            last_soi_deg10: None,
            last_eoi_deg10: None,
            last_spark_advance_deg10: Degrees10(0),
            last_dwell_us: Micros(0),
            combustion_quality_x1000: 0,
            knock_risk_x1000: 0,
            misfire: None,
        }
    }
}

impl Default for CylinderState {
    fn default() -> Self {
        Self::new()
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct CylinderPhysicsState {
    pub pressure_pa: PressurePa,
    pub temperature_k10: Kelvin10,
    pub air_mass_ug: MassUg,
    pub fuel_mass_ug: MassUg,
    pub wall_film_ug: MassUg,
    pub last_burn_fraction_x10000: u16,
    pub residual_fraction_x1000: u16,
    pub pmax_pa: PressurePa,
    pub pmax_angle_deg10: CrankDeg10,
    pub ca10_deg10: Option<CrankDeg10>,
    pub ca50_deg10: Option<CrankDeg10>,
    pub ca90_deg10: Option<CrankDeg10>,
    pub indicated_work_micro_j: EnergyMicroJ,
    pub indicated_torque_nm_x100: TorqueNmX100,
}

impl CylinderPhysicsState {
    pub const fn new() -> Self {
        Self {
            pressure_pa: PressurePa(101_325),
            temperature_k10: Kelvin10(2930),
            air_mass_ug: MassUg(0),
            fuel_mass_ug: MassUg(0),
            wall_film_ug: MassUg(0),
            last_burn_fraction_x10000: 0,
            residual_fraction_x1000: 0,
            pmax_pa: PressurePa(0),
            pmax_angle_deg10: CrankDeg10(0),
            ca10_deg10: None,
            ca50_deg10: None,
            ca90_deg10: None,
            indicated_work_micro_j: EnergyMicroJ(0),
            indicated_torque_nm_x100: TorqueNmX100(0),
        }
    }
}

impl Default for CylinderPhysicsState {
    fn default() -> Self {
        Self::new()
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct CycleAccumulator<const CYL: usize> {
    pub cycle_start_angle_deg10: CrankDeg10,
    pub brake_work_micro_j: EnergyMicroJ,
    pub indicated_work_micro_j: EnergyMicroJ,
    pub pumping_work_micro_j: EnergyMicroJ,
    pub friction_work_micro_j: EnergyMicroJ,
    pub combustion_count: u16,
    pub misfire_count: u16,
    pub completed_cycle_count: u32,
    pub cylinder_work_micro_j: [EnergyMicroJ; CYL],
}

impl<const CYL: usize> CycleAccumulator<CYL> {
    pub const fn new() -> Self {
        Self {
            cycle_start_angle_deg10: CrankDeg10(0),
            brake_work_micro_j: EnergyMicroJ(0),
            indicated_work_micro_j: EnergyMicroJ(0),
            pumping_work_micro_j: EnergyMicroJ(0),
            friction_work_micro_j: EnergyMicroJ(0),
            combustion_count: 0,
            misfire_count: 0,
            completed_cycle_count: 0,
            cylinder_work_micro_j: [EnergyMicroJ(0); CYL],
        }
    }

    pub fn reset_current_cycle(&mut self, start_angle: CrankDeg10) {
        let completed = self.completed_cycle_count;
        *self = Self::new();
        self.completed_cycle_count = completed;
        self.cycle_start_angle_deg10 = start_angle;
    }
}

impl<const CYL: usize> Default for CycleAccumulator<CYL> {
    fn default() -> Self {
        Self::new()
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct PlantState<const CYL: usize> {
    pub timestamp_us: Micros,
    pub rpm: Rpm,
    pub crank_angle_deg10: CrankDeg10,
    pub map_kpa10: Kpa10,
    pub tps_x1000: u16,
    pub clt_c10: Celsius10,
    pub iat_c10: Celsius10,
    pub battery_mv: Millivolts,
    pub cylinders: [CylinderState; CYL],
    pub physics: [CylinderPhysicsState; CYL],
    pub cycle: CycleAccumulator<CYL>,
    pub last_torque_nm_x100: TorqueNmX100,
    pub filtered_torque_nm_x100: TorqueNmX100,
    pub last_lambda_x1000: u16,
    pub exhaust: ExhaustThermalState,
    pub lambda_transport_ring: [u16; MAX_CYLINDERS],
    pub lambda_transport_idx: usize,
    pub lambda_transport_filled: usize,
    pub trigger_sequence: u32,
    pub rpm_delta_remainder: i64,
    pub dyno_pid_integral_x100: i32,
    pub dyno_previous_error_rpm: i32,
    pub dyno_current_target_rpm: Rpm,
    pub dyno_hold_cycle_count: u16,
    pub dyno_sample_cycle_count: u16,
    pub dyno_sample_torque_sum: i64,
    pub dyno_last_completed_cycle_count: u32,
    pub last_dyno_load_torque_nm_x100: TorqueNmX100,
}

impl<const CYL: usize> PlantState<CYL> {
    pub const fn new() -> Self {
        Self {
            timestamp_us: Micros(0),
            rpm: Rpm(0),
            crank_angle_deg10: CrankDeg10(0),
            map_kpa10: Kpa10(350),
            tps_x1000: 0,
            clt_c10: Celsius10(200),
            iat_c10: Celsius10(200),
            battery_mv: Millivolts(12000),
            cylinders: [CylinderState::new(); CYL],
            physics: [CylinderPhysicsState::new(); CYL],
            cycle: CycleAccumulator::new(),
            last_torque_nm_x100: TorqueNmX100(0),
            filtered_torque_nm_x100: TorqueNmX100(0),
            last_lambda_x1000: 1000,
            exhaust: ExhaustThermalState::ambient(),
            lambda_transport_ring: [1000; MAX_CYLINDERS],
            lambda_transport_idx: 0,
            lambda_transport_filled: 0,
            trigger_sequence: 0,
            rpm_delta_remainder: 0,
            dyno_pid_integral_x100: 0,
            dyno_previous_error_rpm: 0,
            dyno_current_target_rpm: Rpm(0),
            dyno_hold_cycle_count: 0,
            dyno_sample_cycle_count: 0,
            dyno_sample_torque_sum: 0,
            dyno_last_completed_cycle_count: 0,
            last_dyno_load_torque_nm_x100: TorqueNmX100(0),
        }
    }

    pub fn from_initial(initial: InitialPlantState<CYL>) -> Self {
        let mut state = Self::new();
        state.timestamp_us = initial.timestamp_us;
        state.rpm = initial.rpm;
        state.crank_angle_deg10 = CrankDeg10::new_normalized(initial.crank_angle_deg10.0);
        let mut i = 0;
        while i < CYL {
            state.cylinders[i].fuel_mass_ug = initial.cylinder_fuel_mass_ug[i];
            i += 1;
        }
        state
    }
}

impl<const CYL: usize> Default for PlantState<CYL> {
    fn default() -> Self {
        Self::new()
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct InitialPlantState<const CYL: usize> {
    pub timestamp_us: Micros,
    pub rpm: Rpm,
    pub crank_angle_deg10: CrankDeg10,
    pub cylinder_fuel_mass_ug: [MassUg; CYL],
}

impl<const CYL: usize> InitialPlantState<CYL> {
    pub const fn new() -> Self {
        Self::stopped()
    }

    pub const fn stopped() -> Self {
        Self {
            timestamp_us: Micros(0),
            rpm: Rpm(0),
            crank_angle_deg10: CrankDeg10(0),
            cylinder_fuel_mass_ug: [MassUg(0); CYL],
        }
    }
}

impl<const CYL: usize> Default for InitialPlantState<CYL> {
    fn default() -> Self {
        Self::new()
    }
}
