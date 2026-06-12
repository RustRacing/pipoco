use super::output::{
    scenario_dfco_decel_step, scenario_initial_clt_c10, scenario_initial_iat_c10,
    scenario_initial_rpm,
};

fn scenario_initial_temp_k10(kind: ScenarioKind) -> u16 {
    match kind {
        ScenarioKind::ColdStart => 2610,
        ScenarioKind::HotRestart => 3610,
        ScenarioKind::DfcoDecel => 3580,
        ScenarioKind::SyncLossRecovery => 3510,
        ScenarioKind::Smoke => 3300,
    }
}

/// Scenario configuration for a headless driver run.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum ScenarioKind {
    #[default]
    Smoke,
    ColdStart,
    HotRestart,
    DfcoDecel,
    SyncLossRecovery,
}

/// Typed signals recorded while a scenario executes.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct DriverScenarioSignals {
    pub cold_start_sync_acquired: u16,
    pub hot_restart_count: u16,
    pub hot_restart_sync_recovered: u16,
    pub dfco_suppressed_injection_outputs: u16,
    pub sync_gap_injected: u16,
    pub sync_loss_detected: u16,
    pub sync_recovered: u16,
}

/// Plant backend used by a scenario run.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum ScenarioBackend {
    #[default]
    Harness,
    Hifi,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ScenarioConfig {
    pub kind: ScenarioKind,
    pub backend: ScenarioBackend,
    pub start_us: u32,
    pub crank_period_us: u32,
    pub tick_period_us: u32,
    pub steps: u16,
    pub starter_steps: u16,
    pub throttle_x1000: u16,
    pub load_torque_x100: i32,
    pub max_events_per_step: usize,
    pub suppress_injection_to_plant: bool,
    pub suppress_ignition_to_plant: bool,
}

impl Default for ScenarioConfig {
    fn default() -> Self {
        Self {
            kind: ScenarioKind::Smoke,
            backend: ScenarioBackend::Harness,
            start_us: 1_000,
            crank_period_us: 500,
            tick_period_us: 500,
            steps: 24,
            starter_steps: 8,
            throttle_x1000: 1_000,
            load_torque_x100: 300,
            max_events_per_step: 16,
            suppress_injection_to_plant: false,
            suppress_ignition_to_plant: false,
        }
    }
}

impl ScenarioConfig {
    pub fn with_backend(mut self, backend: ScenarioBackend) -> Self {
        self.backend = backend;
        self
    }

    pub fn cold_start() -> Self {
        Self {
            kind: ScenarioKind::ColdStart,
            starter_steps: 12,
            steps: 32,
            load_torque_x100: 260,
            ..Default::default()
        }
    }

    pub fn hot_restart() -> Self {
        Self {
            kind: ScenarioKind::HotRestart,
            starter_steps: 4,
            steps: 32,
            load_torque_x100: 220,
            ..Default::default()
        }
    }

    pub fn dfco_decel() -> Self {
        Self {
            kind: ScenarioKind::DfcoDecel,
            starter_steps: 6,
            steps: 28,
            load_torque_x100: 240,
            ..Default::default()
        }
    }

    pub fn sync_loss_recovery() -> Self {
        Self {
            kind: ScenarioKind::SyncLossRecovery,
            starter_steps: 8,
            steps: 30,
            load_torque_x100: 260,
            ..Default::default()
        }
    }
}

/// Report from a completed driver run.
#[derive(Debug, Clone, PartialEq)]
pub struct DriverRunReport {
    pub trace: crate::trace::FixedDriverTrace<{ crate::trace::DRIVER_TRACE_CAP }>,
    pub ecu_snapshot: ecu_sim_ffi::EcuSimSnapshot,
    pub plant_snapshot: ecu_sim::plant::PlantSnapshot,
    pub hifi_step: Option<crate::X86HifiAdapterStep>,
    pub observability: crate::trace::DriverObservability,
    pub scenario_signals: DriverScenarioSignals,
    pub total_outputs: u16,
    pub injection_outputs: u16,
    pub ignition_outputs: u16,
    pub combustion_events: u32,
    pub pending_overflow_count: u32,
}

/// Backward-compatible alias while the hifi path is promoted into the main scenario report.
pub type HifiDriverRunReport = DriverRunReport;

/// Default smoke init configuration.
pub fn default_smoke_init_cfg() -> ecu_sim_ffi::EcuSimInitCfg {
    ecu_sim_ffi::EcuSimInitCfg {
        cylinders: 4,
        has_cam: 1,
        inj_mode: ecu_sim_ffi::EcuSimInjMode::Batch as i32,
        ign_mode: ecu_sim_ffi::EcuSimIgnMode::Wasted as i32,
        firing_len: 4,
        firing_order: [1, 3, 4, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        inj_count: 1,
        inj_channels: [1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        ign_count: 1,
        ign_channels: [1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct HifiScenarioFrame {
    pub throttle_x1000: u16,
    pub load_torque_x100: i32,
    pub clt_c10: i16,
    pub iat_c10: i16,
    pub fuel_mass_kg: f64,
    pub spark_advance_deg10: u16,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct HifiScenarioCalibrations {
    pub fuel_mass_kg: f64,
    pub spark_advance_deg10: u16,
}

pub fn default_hifi_scenario_config() -> ecu_sim_hifi::PlantConfig {
    ecu_sim_hifi::default_plant_config()
}

pub fn hifi_scenario_initial_temperature_k(kind: ScenarioKind) -> f64 {
    f64::from(scenario_initial_temp_k10(kind)) / 10.0
}

pub fn hifi_scenario_initial_rpm(kind: ScenarioKind) -> u16 {
    scenario_initial_rpm(kind)
}

pub fn hifi_scenario_calibrations(kind: ScenarioKind) -> HifiScenarioCalibrations {
    match kind {
        ScenarioKind::Smoke => HifiScenarioCalibrations {
            fuel_mass_kg: 1.8e-5,
            spark_advance_deg10: 180,
        },
        ScenarioKind::ColdStart => HifiScenarioCalibrations {
            fuel_mass_kg: 2.0e-5,
            spark_advance_deg10: 160,
        },
        ScenarioKind::HotRestart => HifiScenarioCalibrations {
            fuel_mass_kg: 1.8e-5,
            spark_advance_deg10: 160,
        },
        ScenarioKind::DfcoDecel => HifiScenarioCalibrations {
            fuel_mass_kg: 1.8e-5,
            spark_advance_deg10: 180,
        },
        ScenarioKind::SyncLossRecovery => HifiScenarioCalibrations {
            fuel_mass_kg: 1.8e-5,
            spark_advance_deg10: 180,
        },
    }
}

pub fn hifi_scenario_step_frame(config: ScenarioConfig, step_index: u16) -> HifiScenarioFrame {
    let calibrations = hifi_scenario_calibrations(config.kind);
    let mut throttle_x1000 = config.throttle_x1000;
    let mut load_torque_x100 = config.load_torque_x100;
    let mut clt_c10 = scenario_initial_clt_c10(config.kind);
    let mut iat_c10 = scenario_initial_iat_c10(config.kind);

    if config.kind == ScenarioKind::DfcoDecel {
        if let Some(decel_step) = scenario_dfco_decel_step(config.kind) {
            if step_index >= decel_step {
                throttle_x1000 = 0;
                load_torque_x100 = 120;
                clt_c10 = 850;
                iat_c10 = 320;
            }
        }
    }

    if config.kind == ScenarioKind::ColdStart {
        clt_c10 = -120;
        iat_c10 = -90;
    }

    if config.kind == ScenarioKind::HotRestart {
        clt_c10 = 880;
        iat_c10 = 600;
    }

    if config.kind == ScenarioKind::SyncLossRecovery {
        clt_c10 = 780;
        iat_c10 = 280;
    }

    HifiScenarioFrame {
        throttle_x1000,
        load_torque_x100,
        clt_c10,
        iat_c10,
        fuel_mass_kg: calibrations.fuel_mass_kg,
        spark_advance_deg10: calibrations.spark_advance_deg10,
    }
}
