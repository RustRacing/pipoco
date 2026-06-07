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

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ScenarioConfig {
    pub kind: ScenarioKind,
    pub start_us: u32,
    pub crank_period_us: u32,
    pub tick_period_us: u32,
    pub steps: u16,
    pub starter_steps: u16,
    pub throttle_x100: u16,
    pub load_torque_x100: i32,
    pub max_events_per_step: usize,
    pub suppress_injection_to_plant: bool,
    pub suppress_ignition_to_plant: bool,
}

impl Default for ScenarioConfig {
    fn default() -> Self {
        Self {
            kind: ScenarioKind::Smoke,
            start_us: 1_000,
            crank_period_us: 500,
            tick_period_us: 500,
            steps: 24,
            starter_steps: 8,
            throttle_x100: 1_200,
            load_torque_x100: 300,
            max_events_per_step: 16,
            suppress_injection_to_plant: false,
            suppress_ignition_to_plant: false,
        }
    }
}

impl ScenarioConfig {
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
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct DriverRunReport {
    pub trace: crate::trace::FixedDriverTrace<{ crate::trace::DRIVER_TRACE_CAP }>,
    pub ecu_snapshot: ecu_sim_ffi::EcuSimSnapshot,
    pub plant_snapshot: ecu_sim::plant::PlantSnapshot,
    pub observability: crate::trace::DriverObservability,
    pub scenario_signals: DriverScenarioSignals,
    pub total_outputs: u16,
    pub injection_outputs: u16,
    pub ignition_outputs: u16,
    pub combustion_events: u32,
    pub pending_overflow_count: u32,
}

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
