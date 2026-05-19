//! Deterministic headless smoke scenario for the ECU driver.

use crate::trace::FixedDriverTrace;
use crate::DriverError;
use ecu_domain::{Micros, Rpm};
use ecu_io::{OutputLevel, OutputTransitionKind};
use ecu_sim::plant::{
    ClosedLoopPlant, FixedPlantProfile, InjectorModel, PlantControls, PlantLimits,
};

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
        inj_channels: [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        ign_count: 1,
        ign_channels: [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    }
}

/// Pending output queue for sorting and due-time dispatch.
struct PendingOutputQueue<const N: usize> {
    events: [Option<ecu_io::OutputTransition>; N],
    len: usize,
    overflow_count: u32,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct OutputLevelTracker {
    injector: [OutputLevel; 16],
    ignition: [OutputLevel; 16],
    idle: [OutputLevel; 16],
    fan: [OutputLevel; 16],
}

impl OutputLevelTracker {
    const fn new() -> Self {
        Self {
            injector: [OutputLevel::Low; 16],
            ignition: [OutputLevel::Low; 16],
            idle: [OutputLevel::Low; 16],
            fan: [OutputLevel::Low; 16],
        }
    }

    fn update(&mut self, event: ecu_io::OutputTransition) -> bool {
        let idx = event.channel.get() as usize;
        if idx >= 16 {
            return true;
        }
        let level = match event.kind {
            OutputTransitionKind::Injector => &mut self.injector[idx],
            OutputTransitionKind::Ignition => &mut self.ignition[idx],
            OutputTransitionKind::Idle => &mut self.idle[idx],
            OutputTransitionKind::Fan => &mut self.fan[idx],
        };
        if *level == event.level {
            false
        } else {
            *level = event.level;
            true
        }
    }
}

impl Default for OutputLevelTracker {
    fn default() -> Self {
        Self::new()
    }
}

impl<const N: usize> PendingOutputQueue<N> {
    fn new() -> Self {
        Self {
            events: [None; N],
            len: 0,
            overflow_count: 0,
        }
    }

    fn push_sorted(&mut self, event: ecu_io::OutputTransition) -> Result<(), DriverError> {
        if self.len >= N {
            self.overflow_count = self.overflow_count.saturating_add(1);
            return Err(DriverError::PendingOutputOverflow);
        }

        // Insert in sorted order: (at_us, kind, channel, level)
        let mut idx = self.len;
        while idx > 0 {
            let Some(prev) = self.events[idx - 1] else {
                break;
            };
            if (
                event.at_us.get(),
                sort_key_kind(event.kind),
                event.channel.get(),
                sort_level(&event.level),
            ) >= (
                prev.at_us.get(),
                sort_key_kind(prev.kind),
                prev.channel.get(),
                sort_level(&prev.level),
            ) {
                break;
            }
            idx -= 1;
        }
        // Shift elements to make room
        let mut i = self.len;
        while i > idx {
            self.events[i] = self.events[i - 1];
            i -= 1;
        }
        self.events[idx] = Some(event);
        self.len += 1;
        Ok(())
    }

    fn drain_due<const M: usize>(
        &mut self,
        now_us: u32,
        plant: &mut ClosedLoopPlant<FixedPlantProfile>,
        trace: &mut crate::trace::FixedDriverTrace<M>,
        levels: &mut OutputLevelTracker,
    ) -> Result<(), DriverError> {
        let mut i = 0;
        while i < self.len {
            let Some(event) = self.events[i] else {
                i += 1;
                continue;
            };
            if event.at_us.get() <= now_us {
                let changed = levels.update(event);
                if changed {
                    plant.apply_transition(event).map_err(DriverError::Plant)?;
                }
                trace.push(output_trace_record(event, if changed { 0 } else { 1 }))?;
                // Remove from queue
                let mut j = i;
                while j < self.len - 1 {
                    self.events[j] = self.events[j + 1];
                    j += 1;
                }
                self.events[self.len - 1] = None;
                self.len -= 1;
            } else {
                i += 1;
            }
        }
        Ok(())
    }

    fn pending_overflow(&self) -> u32 {
        self.overflow_count
    }
}

impl<const N: usize> Default for PendingOutputQueue<N> {
    fn default() -> Self {
        Self::new()
    }
}

fn sort_key_kind(kind: OutputTransitionKind) -> i32 {
    match kind {
        OutputTransitionKind::Injector => 0,
        OutputTransitionKind::Ignition => 1,
        OutputTransitionKind::Idle => 2,
        OutputTransitionKind::Fan => 3,
    }
}

fn sort_level(level: &ecu_io::OutputLevel) -> i32 {
    match level {
        ecu_io::OutputLevel::Low => 0,
        ecu_io::OutputLevel::High => 1,
    }
}

fn output_trace_record(
    event: ecu_io::OutputTransition,
    status: i32,
) -> crate::trace::DriverTraceRecord {
    crate::trace::DriverTraceRecord {
        at_us: event.at_us.get(),
        kind: crate::trace::DriverTraceKind::Output,
        status,
        rpm: 0,
        map_kpa10: 0,
        angle_x10: 0,
        output_kind: match event.kind {
            OutputTransitionKind::Injector => 0,
            OutputTransitionKind::Ignition => 1,
            OutputTransitionKind::Idle => 2,
            OutputTransitionKind::Fan => 3,
        },
        channel: event.channel.get(),
        high: match event.level {
            ecu_io::OutputLevel::Low => 0,
            ecu_io::OutputLevel::High => 1,
        },
        combustion_events: 0,
        synced: 0,
        tooth: 0,
        diagnostic_code: 0,
        fault_severity: 0,
        cancel_reason: 0,
        control_mode: 0,
        observability: crate::trace::DriverObservability::default(),
    }
}

fn suppress_output_to_plant(config: ScenarioConfig, kind: OutputTransitionKind) -> bool {
    match kind {
        OutputTransitionKind::Injector => config.suppress_injection_to_plant,
        OutputTransitionKind::Ignition => config.suppress_ignition_to_plant,
        OutputTransitionKind::Idle | OutputTransitionKind::Fan => false,
    }
}

fn scenario_initial_rpm(kind: ScenarioKind) -> u16 {
    match kind {
        ScenarioKind::ColdStart => 250,
        ScenarioKind::HotRestart | ScenarioKind::DfcoDecel | ScenarioKind::SyncLossRecovery => 850,
        ScenarioKind::Smoke => 850,
    }
}

fn scenario_initial_clt_c10(kind: ScenarioKind) -> i16 {
    match kind {
        ScenarioKind::ColdStart => -120,
        ScenarioKind::HotRestart => 880,
        ScenarioKind::DfcoDecel => 820,
        ScenarioKind::SyncLossRecovery => 780,
        ScenarioKind::Smoke => 800,
    }
}

fn scenario_initial_iat_c10(kind: ScenarioKind) -> i16 {
    match kind {
        ScenarioKind::ColdStart => -90,
        ScenarioKind::HotRestart => 600,
        ScenarioKind::DfcoDecel => 320,
        ScenarioKind::SyncLossRecovery => 280,
        ScenarioKind::Smoke => 250,
    }
}

fn scenario_restart_step(kind: ScenarioKind) -> Option<u16> {
    match kind {
        ScenarioKind::HotRestart => Some(10),
        _ => None,
    }
}

fn scenario_dfco_decel_step(kind: ScenarioKind) -> Option<u16> {
    match kind {
        ScenarioKind::DfcoDecel => Some(12),
        _ => None,
    }
}

fn scenario_sync_loss_gap_step(kind: ScenarioKind) -> Option<u16> {
    match kind {
        ScenarioKind::SyncLossRecovery => Some(14),
        _ => None,
    }
}

fn scenario_sync_loss_gap_us(kind: ScenarioKind) -> Option<u32> {
    match kind {
        ScenarioKind::SyncLossRecovery => Some(5_000),
        _ => None,
    }
}

fn scenario_starter_on(config: ScenarioConfig, step_index: u16, restart_step: Option<u16>) -> bool {
    if step_index < config.starter_steps {
        return true;
    }
    match restart_step {
        Some(restart_step) => {
            let restart_end = restart_step.saturating_add(config.starter_steps);
            step_index >= restart_step && step_index < restart_end
        }
        None => false,
    }
}

/// Run the headless smoke scenario with the given configuration.
pub fn run_headless_smoke(config: ScenarioConfig) -> Result<DriverRunReport, DriverError> {
    // Validate config
    if config.crank_period_us == 0 || config.tick_period_us == 0 || config.steps < 12 {
        return Err(DriverError::FfiStatus(
            ecu_sim_ffi::EcuSimStatus::ErrInvalid,
        ));
    }
    if config.max_events_per_step > ecu_sim_ffi::ECU_SIM_MAX_EVENTS {
        return Err(DriverError::FfiStatus(
            ecu_sim_ffi::EcuSimStatus::ErrInvalid,
        ));
    }

    let mut client = crate::ffi_client::EcuFfiClient::acquire();
    client.reset();
    let init_cfg = default_smoke_init_cfg();
    client.init(init_cfg)?;

    let conservative_limits = PlantLimits::conservative();
    let mut plant = ClosedLoopPlant::new(
        FixedPlantProfile::inline_four(),
        InjectorModel::gasoline(240),
    )
    .with_limits(PlantLimits {
        min_dwell_us: 400,
        ..conservative_limits
    });
    plant.set_initial_rpm(Rpm::new(scenario_initial_rpm(config.kind)));

    let mut trace = FixedDriverTrace::<{ crate::trace::DRIVER_TRACE_CAP }>::new();
    let mut pending_queue: PendingOutputQueue<256> = PendingOutputQueue::new();
    let mut output_levels = OutputLevelTracker::new();
    let empty_observability = crate::trace::DriverObservability::default();
    let mut scenario_signals = DriverScenarioSignals::default();

    let mut total_outputs: u16 = 0;
    let mut injection_outputs: u16 = 0;
    let mut ignition_outputs: u16 = 0;
    let mut next_crank_edge_us = config.start_us;
    let mut crank_edges_seen: u32 = 0;
    let mut cam_edge_emitted = false;
    let mut last_sensor_frame =
        plant.sensor_frame(Micros::new(config.start_us), PlantControls::idle());
    let restart_step = scenario_restart_step(config.kind);
    let dfco_decel_step = scenario_dfco_decel_step(config.kind);
    let sync_loss_gap_step = scenario_sync_loss_gap_step(config.kind);
    let sync_loss_gap_us = scenario_sync_loss_gap_us(config.kind);
    let mut hot_restart_done = false;
    let mut sync_loss_gap_injected = false;
    let mut sync_loss_observed = false;
    let mut sync_recovered = false;
    let mut sync_loss_time_offset_us: u32 = 0;
    let mut dfco_decel_started = false;

    // Record init
    trace.push(crate::trace::DriverTraceRecord {
        at_us: config.start_us,
        kind: crate::trace::DriverTraceKind::Init,
        status: 0,
        rpm: 0,
        map_kpa10: 0,
        angle_x10: 0,
        output_kind: -1,
        channel: 0,
        high: 0,
        combustion_events: 0,
        synced: 0,
        tooth: 0,
        diagnostic_code: 0,
        fault_severity: 0,
        cancel_reason: 0,
        control_mode: 0,
        observability: empty_observability,
    })?;

    // Main loop
    for step_index in 0..config.steps {
        let step_offset_us = (step_index as u32)
            .checked_mul(config.tick_period_us)
            .ok_or(DriverError::FfiStatus(
                ecu_sim_ffi::EcuSimStatus::ErrInvalid,
            ))?;
        let base_now_us =
            config
                .start_us
                .checked_add(step_offset_us)
                .ok_or(DriverError::FfiStatus(
                    ecu_sim_ffi::EcuSimStatus::ErrInvalid,
                ))?;
        let now_us =
            base_now_us
                .checked_add(sync_loss_time_offset_us)
                .ok_or(DriverError::FfiStatus(
                    ecu_sim_ffi::EcuSimStatus::ErrInvalid,
                ))?;

        if config.kind == ScenarioKind::HotRestart {
            if let Some(restart_step) = restart_step {
                if !hot_restart_done && step_index == restart_step {
                    client.reset();
                    client.init(init_cfg)?;
                    trace.push(crate::trace::DriverTraceRecord {
                        at_us: now_us,
                        kind: crate::trace::DriverTraceKind::Init,
                        status: 0,
                        rpm: 0,
                        map_kpa10: 0,
                        angle_x10: 0,
                        output_kind: -1,
                        channel: 0,
                        high: 0,
                        combustion_events: 0,
                        synced: 0,
                        tooth: 0,
                        diagnostic_code: 0,
                        fault_severity: 0,
                        cancel_reason: 0,
                        control_mode: 0,
                        observability: empty_observability,
                    })?;
                    pending_queue = PendingOutputQueue::new();
                    output_levels = OutputLevelTracker::new();
                    next_crank_edge_us = now_us;
                    crank_edges_seen = 0;
                    cam_edge_emitted = false;
                    hot_restart_done = true;
                    scenario_signals.hot_restart_count =
                        scenario_signals.hot_restart_count.saturating_add(1);
                }
            }
        }

        // Build controls
        let mut throttle_x100 = config.throttle_x100;
        let mut load_torque_x100 = config.load_torque_x100;
        let starter_on = scenario_starter_on(config, step_index, restart_step);
        let mut clt_c10 = scenario_initial_clt_c10(config.kind);
        let mut iat_c10 = scenario_initial_iat_c10(config.kind);

        if config.kind == ScenarioKind::DfcoDecel {
            if let Some(decel_step) = dfco_decel_step {
                if step_index >= decel_step {
                    dfco_decel_started = true;
                    throttle_x100 = 0;
                    load_torque_x100 = 120;
                }
            }
            if dfco_decel_started {
                clt_c10 = 850;
                iat_c10 = 320;
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

        let controls = PlantControls {
            throttle_x100,
            starter_on,
            load_torque_x100,
            vbatt_mv: 12_500,
            fault: ecu_sim::plant::PlantFault::None,
        };

        // Get sensor frame from plant
        let mut sensor_frame = plant.sensor_frame(Micros::new(now_us), controls);
        sensor_frame.tps_x100 = throttle_x100;
        sensor_frame.clt_c10 = clt_c10;
        sensor_frame.iat_c10 = iat_c10;
        sensor_frame.vbatt_mv = 12_500;
        last_sensor_frame = sensor_frame;
        trace.push(crate::trace::DriverTraceRecord {
            at_us: now_us,
            kind: crate::trace::DriverTraceKind::Sensor,
            status: 0,
            rpm: sensor_frame.rpm.get(),
            map_kpa10: sensor_frame.map_kpa10.get(),
            angle_x10: sensor_frame.angle_x10.get(),
            output_kind: -1,
            channel: 0,
            high: 0,
            combustion_events: 0,
            synced: 0,
            tooth: 0,
            diagnostic_code: 0,
            fault_severity: 0,
            cancel_reason: 0,
            control_mode: 0,
            observability: empty_observability,
        })?;

        // Feed sensors to ECU
        let ffi_frame = crate::ffi_client::sensor_frame_to_ffi(sensor_frame);
        client.set_sensors(ffi_frame)?;
        if config.kind == ScenarioKind::SyncLossRecovery && sync_loss_gap_injected {
            let snap = client.snapshot()?;
            if !sync_loss_observed && snap.synced == 0 {
                sync_loss_observed = true;
                crank_edges_seen = 0;
                cam_edge_emitted = false;
                scenario_signals.sync_loss_detected =
                    scenario_signals.sync_loss_detected.saturating_add(1);
            }
        }

        // Feed all crank edges that are due at this step's timestamp.
        while next_crank_edge_us <= now_us {
            client.on_crank_edge(next_crank_edge_us)?;
            trace.push(crate::trace::DriverTraceRecord {
                at_us: next_crank_edge_us,
                kind: crate::trace::DriverTraceKind::CrankEdge,
                status: 0,
                rpm: sensor_frame.rpm.get(),
                map_kpa10: sensor_frame.map_kpa10.get(),
                angle_x10: sensor_frame.angle_x10.get(),
                output_kind: -1,
                channel: 0,
                high: 0,
                combustion_events: 0,
                synced: 0,
                tooth: 0,
                diagnostic_code: 0,
                fault_severity: 0,
                cancel_reason: 0,
                control_mode: 0,
                observability: empty_observability,
            })?;
            crank_edges_seen = crank_edges_seen.saturating_add(1);

            // Emit a single cam edge on the second crank edge to establish sync.
            if !cam_edge_emitted && crank_edges_seen == 2 {
                client.on_cam_edge(next_crank_edge_us)?;
                trace.push(crate::trace::DriverTraceRecord {
                    at_us: next_crank_edge_us,
                    kind: crate::trace::DriverTraceKind::CamEdge,
                    status: 0,
                    rpm: sensor_frame.rpm.get(),
                    map_kpa10: sensor_frame.map_kpa10.get(),
                    angle_x10: sensor_frame.angle_x10.get(),
                    output_kind: -1,
                    channel: 0,
                    high: 0,
                    combustion_events: 0,
                    synced: 0,
                    tooth: 0,
                    diagnostic_code: 0,
                    fault_severity: 0,
                    cancel_reason: 0,
                    control_mode: 0,
                    observability: empty_observability,
                })?;
                cam_edge_emitted = true;

                if config.kind == ScenarioKind::ColdStart {
                    let snap = client.snapshot()?;
                    if snap.synced == 1 {
                        scenario_signals.cold_start_sync_acquired =
                            scenario_signals.cold_start_sync_acquired.saturating_add(1);
                    }
                }
                if config.kind == ScenarioKind::HotRestart && hot_restart_done {
                    let snap = client.snapshot()?;
                    if snap.synced == 1 {
                        scenario_signals.hot_restart_sync_recovered = scenario_signals
                            .hot_restart_sync_recovered
                            .saturating_add(1);
                    }
                }
            }

            if config.kind == ScenarioKind::SyncLossRecovery {
                if let Some(gap_step) = sync_loss_gap_step {
                    if !sync_loss_gap_injected && step_index == gap_step {
                        if let Some(gap_us) = sync_loss_gap_us {
                            next_crank_edge_us = next_crank_edge_us.checked_add(gap_us).ok_or(
                                DriverError::FfiStatus(ecu_sim_ffi::EcuSimStatus::ErrInvalid),
                            )?;
                            sync_loss_time_offset_us =
                                sync_loss_time_offset_us.checked_add(gap_us).ok_or(
                                    DriverError::FfiStatus(ecu_sim_ffi::EcuSimStatus::ErrInvalid),
                                )?;
                            sync_loss_gap_injected = true;
                            scenario_signals.sync_gap_injected =
                                scenario_signals.sync_gap_injected.saturating_add(1);
                        }
                    }
                }
                if sync_loss_gap_injected {
                    let snap = client.snapshot()?;
                    if !sync_loss_observed && snap.synced == 0 {
                        sync_loss_observed = true;
                        crank_edges_seen = 0;
                        cam_edge_emitted = false;
                        scenario_signals.sync_loss_detected =
                            scenario_signals.sync_loss_detected.saturating_add(1);
                    } else if sync_loss_observed && snap.synced == 1 && !sync_recovered {
                        sync_recovered = true;
                        scenario_signals.sync_recovered =
                            scenario_signals.sync_recovered.saturating_add(1);
                    }
                }
            }

            next_crank_edge_us = next_crank_edge_us
                .checked_add(config.crank_period_us)
                .ok_or(DriverError::FfiStatus(
                    ecu_sim_ffi::EcuSimStatus::ErrInvalid,
                ))?;
        }

        // FFI step
        client.step(now_us)?;
        if config.kind == ScenarioKind::SyncLossRecovery && sync_loss_gap_injected {
            let snap = client.snapshot()?;
            if !sync_loss_observed && snap.synced == 0 {
                sync_loss_observed = true;
                crank_edges_seen = 0;
                cam_edge_emitted = false;
                scenario_signals.sync_loss_detected =
                    scenario_signals.sync_loss_detected.saturating_add(1);
            } else if sync_loss_observed && snap.synced == 1 && !sync_recovered {
                sync_recovered = true;
                scenario_signals.sync_recovered = scenario_signals.sync_recovered.saturating_add(1);
            }
        }
        trace.push(crate::trace::DriverTraceRecord {
            at_us: now_us,
            kind: crate::trace::DriverTraceKind::Step,
            status: 0,
            rpm: sensor_frame.rpm.get(),
            map_kpa10: sensor_frame.map_kpa10.get(),
            angle_x10: sensor_frame.angle_x10.get(),
            output_kind: -1,
            channel: 0,
            high: 0,
            combustion_events: 0,
            synced: 0,
            tooth: 0,
            diagnostic_code: 0,
            fault_severity: 0,
            cancel_reason: 0,
            control_mode: 0,
            observability: empty_observability,
        })?;

        // Dequeue the ABI maximum, then enforce the scenario's per-step limit.
        let mut events = [ecu_sim_ffi::EcuSimOutputEvent::ZERO; ecu_sim_ffi::ECU_SIM_MAX_EVENTS];
        let count = client.dequeue_events(&mut events)?;
        if count > config.max_events_per_step {
            return Err(DriverError::EventOverflow);
        }

        // Convert and queue events
        for event in &events[..count] {
            let transition = crate::ffi_client::output_event_to_transition(*event)?;
            match transition.kind {
                OutputTransitionKind::Injector => {
                    injection_outputs = injection_outputs.saturating_add(1)
                }
                OutputTransitionKind::Ignition => {
                    ignition_outputs = ignition_outputs.saturating_add(1)
                }
                _ => {}
            }
            total_outputs = total_outputs.saturating_add(1);
            let suppress_to_plant = if config.kind == ScenarioKind::DfcoDecel
                && dfco_decel_started
                && matches!(transition.kind, OutputTransitionKind::Injector)
            {
                true
            } else {
                suppress_output_to_plant(config, transition.kind)
            };
            if suppress_to_plant {
                trace.push(output_trace_record(transition, 2))?;
                if config.kind == ScenarioKind::DfcoDecel
                    && matches!(transition.kind, OutputTransitionKind::Injector)
                {
                    // This records driver-to-plant suppression during the DFCO
                    // scenario; it is not an ECU-owned DFCO fuel-cut assertion.
                    scenario_signals.dfco_suppressed_injection_outputs = scenario_signals
                        .dfco_suppressed_injection_outputs
                        .saturating_add(1);
                }
            } else {
                pending_queue.push_sorted(transition)?;
            }
        }

        // Drain due events
        pending_queue.drain_due(now_us, &mut plant, &mut trace, &mut output_levels)?;

        // Advance plant
        let plant_snap = plant.advance_to(Micros::new(now_us), controls);
        trace.push(crate::trace::DriverTraceRecord {
            at_us: now_us,
            kind: crate::trace::DriverTraceKind::PlantAdvance,
            status: 0,
            rpm: plant_snap.rpm.get(),
            map_kpa10: plant_snap.map_kpa10.get(),
            angle_x10: plant_snap.crank_angle_deg10.get(),
            output_kind: -1,
            channel: 0,
            high: 0,
            combustion_events: plant_snap.combustion_events,
            synced: 0,
            tooth: 0,
            diagnostic_code: 0,
            fault_severity: 0,
            cancel_reason: 0,
            control_mode: 0,
            observability: empty_observability,
        })?;
    }

    // Final drain
    let final_due_us = config
        .start_us
        .checked_add(
            (config.steps as u32)
                .checked_mul(config.tick_period_us)
                .ok_or(DriverError::FfiStatus(
                    ecu_sim_ffi::EcuSimStatus::ErrInvalid,
                ))?,
        )
        .and_then(|base| base.checked_add(sync_loss_time_offset_us))
        .and_then(|base| base.checked_add(5_000))
        .ok_or(DriverError::FfiStatus(
            ecu_sim_ffi::EcuSimStatus::ErrInvalid,
        ))?;
    pending_queue.drain_due(final_due_us, &mut plant, &mut trace, &mut output_levels)?;
    let _ = plant.advance_to(Micros::new(final_due_us), PlantControls::idle());

    // Final ECU snapshot
    let ecu_snapshot = client.snapshot()?;
    let observability = crate::ffi_client::observability_from_snapshot_and_sensor_frame(
        &ecu_snapshot,
        last_sensor_frame,
    );

    // Plant snapshot
    let plant_snapshot = plant.snapshot(PlantControls::idle());
    trace.push(crate::trace::DriverTraceRecord {
        at_us: final_due_us,
        kind: crate::trace::DriverTraceKind::Snapshot,
        status: 0,
        rpm: ecu_snapshot.rpm,
        map_kpa10: plant_snapshot.map_kpa10.get(),
        angle_x10: ecu_snapshot.angle_x10,
        output_kind: -1,
        channel: 0,
        high: 0,
        combustion_events: plant_snapshot.combustion_events,
        synced: ecu_snapshot.synced,
        tooth: ecu_snapshot.tooth,
        diagnostic_code: ecu_snapshot.fault_code,
        fault_severity: ecu_snapshot.fault_severity,
        cancel_reason: ecu_snapshot.cancel_reason,
        control_mode: ecu_snapshot.control_mode,
        observability,
    })?;

    // Final assertions
    if ecu_snapshot.synced != 1 {
        return Err(DriverError::ScenarioDidNotSync);
    }
    if injection_outputs == 0 {
        return Err(DriverError::ScenarioDidNotEmitInjection);
    }
    if ignition_outputs == 0 {
        return Err(DriverError::ScenarioDidNotEmitIgnition);
    }
    if plant_snapshot.combustion_events == 0 {
        return Err(DriverError::ScenarioDidNotCombust);
    }
    if trace.overflow_count() > 0 {
        return Err(DriverError::TraceOverflow);
    }
    if pending_queue.pending_overflow() > 0 {
        return Err(DriverError::PendingOutputOverflow);
    }

    Ok(DriverRunReport {
        trace,
        ecu_snapshot,
        plant_snapshot,
        observability,
        scenario_signals,
        total_outputs,
        injection_outputs,
        ignition_outputs,
        combustion_events: plant_snapshot.combustion_events,
        pending_overflow_count: pending_queue.pending_overflow(),
    })
}

/// Run the default headless smoke scenario.
pub fn run_default_headless_smoke() -> Result<DriverRunReport, DriverError> {
    run_headless_smoke(ScenarioConfig::default())
}

/// Run a deterministic cold-start scenario.
pub fn run_cold_start_scenario() -> Result<DriverRunReport, DriverError> {
    run_headless_smoke(ScenarioConfig::cold_start())
}

/// Run a deterministic hot-restart scenario.
pub fn run_hot_restart_scenario() -> Result<DriverRunReport, DriverError> {
    run_headless_smoke(ScenarioConfig::hot_restart())
}

/// Run a deterministic DFCO decel scenario.
pub fn run_dfco_decel_scenario() -> Result<DriverRunReport, DriverError> {
    run_headless_smoke(ScenarioConfig::dfco_decel())
}

/// Run a deterministic sync-loss recovery scenario.
pub fn run_sync_loss_recovery_scenario() -> Result<DriverRunReport, DriverError> {
    run_headless_smoke(ScenarioConfig::sync_loss_recovery())
}

/// Run the default smoke twice and compare results for determinism.
pub fn run_default_headless_smoke_twice() -> Result<(DriverRunReport, DriverRunReport), DriverError>
{
    let report1 = run_default_headless_smoke()?;
    let report2 = run_default_headless_smoke()?;
    if report1 != report2 {
        return Err(DriverError::ScenarioTraceMismatch);
    }
    Ok((report1, report2))
}

#[cfg(test)]
mod tests {
    use super::*;

    fn require_report(result: Result<DriverRunReport, DriverError>) -> DriverRunReport {
        let Ok(report) = result else {
            unreachable!("scenario should succeed in this unit test")
        };
        report
    }

    #[test]
    fn headless_smoke_syncs_and_emits_outputs() {
        let report = require_report(run_default_headless_smoke());
        assert_eq!(report.ecu_snapshot.synced, 1, "ECU should be synced");
        assert!(
            report.injection_outputs > 0,
            "should emit injection outputs"
        );
        assert!(report.ignition_outputs > 0, "should emit ignition outputs");
    }

    #[test]
    fn headless_smoke_outputs_drive_plant_combustion() {
        let report = require_report(run_default_headless_smoke());
        assert!(
            report.combustion_events > 0,
            "plant should have combustion events"
        );
    }

    #[test]
    fn headless_smoke_is_deterministic_across_repeated_runs() {
        let Ok((r1, r2)) = run_default_headless_smoke_twice() else {
            unreachable!("determinism run should succeed")
        };
        assert_eq!(r1, r2, "two smoke runs should produce identical reports");
    }

    #[test]
    fn invalid_scenario_config_is_rejected() {
        let bad_crank = ScenarioConfig {
            crank_period_us: 0,
            ..Default::default()
        };
        let result = run_headless_smoke(bad_crank);
        assert!(result.is_err(), "zero crank_period should be rejected");

        let bad_tick = ScenarioConfig {
            tick_period_us: 0,
            ..Default::default()
        };
        let result_tick = run_headless_smoke(bad_tick);
        assert!(result_tick.is_err(), "zero tick_period should be rejected");

        let bad_steps = ScenarioConfig {
            steps: 10,
            ..Default::default()
        };
        let result_steps = run_headless_smoke(bad_steps);
        assert!(result_steps.is_err(), "steps < 12 should be rejected");

        let bad_events = ScenarioConfig {
            max_events_per_step: ecu_sim_ffi::ECU_SIM_MAX_EVENTS + 1,
            ..Default::default()
        };
        let result_events = run_headless_smoke(bad_events);
        assert!(
            result_events.is_err(),
            "max_events_per_step above the ABI limit should be rejected"
        );
    }

    #[test]
    fn timestamp_overflow_is_rejected() {
        let config = ScenarioConfig {
            start_us: u32::MAX - 8,
            tick_period_us: 2,
            steps: 12,
            ..Default::default()
        };

        assert_eq!(
            run_headless_smoke(config),
            Err(DriverError::FfiStatus(
                ecu_sim_ffi::EcuSimStatus::ErrInvalid
            )),
            "scenario timestamp overflow must be explicit instead of wrapping"
        );
    }

    #[test]
    fn pending_output_queue_orders_and_drains_due_events() {
        let mut queue: PendingOutputQueue<8> = PendingOutputQueue::new();
        let mut plant = ClosedLoopPlant::new(
            FixedPlantProfile::inline_four(),
            InjectorModel::gasoline(240),
        );
        let mut trace = FixedDriverTrace::<8>::new();
        let mut levels = OutputLevelTracker::new();

        // Insert events out of order, including a duplicate unchanged transition.
        let e1 = ecu_io::OutputTransition {
            at_us: Micros::new(300),
            kind: OutputTransitionKind::Ignition,
            channel: ecu_domain::ChannelId::new(0),
            level: ecu_io::OutputLevel::High,
        };
        let e2 = ecu_io::OutputTransition {
            at_us: Micros::new(100),
            kind: OutputTransitionKind::Injector,
            channel: ecu_domain::ChannelId::new(0),
            level: ecu_io::OutputLevel::High,
        };
        let e3 = ecu_io::OutputTransition {
            at_us: Micros::new(100),
            kind: OutputTransitionKind::Injector,
            channel: ecu_domain::ChannelId::new(0),
            level: ecu_io::OutputLevel::High,
        };
        let e4 = ecu_io::OutputTransition {
            at_us: Micros::new(400),
            kind: OutputTransitionKind::Ignition,
            channel: ecu_domain::ChannelId::new(0),
            level: ecu_io::OutputLevel::Low,
        };

        assert_eq!(queue.push_sorted(e1), Ok(()));
        assert_eq!(queue.push_sorted(e2), Ok(()));
        assert_eq!(queue.push_sorted(e3), Ok(()));
        assert_eq!(queue.push_sorted(e4), Ok(()));

        // Drain at t=150 - both 100us events should fire and the second one should
        // remain a duplicate unchanged transition.
        assert_eq!(
            queue.drain_due(150, &mut plant, &mut trace, &mut levels),
            Ok(())
        );
        assert_eq!(trace.len(), 2, "should have two trace records after drain");
        assert_eq!(trace.get(0).map(|record| record.at_us), Some(100));
        assert_eq!(trace.get(1).map(|record| record.at_us), Some(100));
        assert_eq!(trace.get(0).map(|record| record.status), Some(0));
        assert_eq!(trace.get(1).map(|record| record.status), Some(1));
        assert_eq!(queue.len, 2, "future events should remain queued");
        assert_eq!(queue.events[0].map(|event| event.at_us.get()), Some(300));
        assert_eq!(queue.events[1].map(|event| event.at_us.get()), Some(400));

        // Drain at t=350 - should apply the 300us event and retain the future one.
        assert_eq!(
            queue.drain_due(350, &mut plant, &mut trace, &mut levels),
            Ok(())
        );
        assert_eq!(trace.len(), 3);
        assert_eq!(trace.get(2).map(|record| record.at_us), Some(300));
        assert_eq!(queue.len, 1);
        assert_eq!(queue.events[0].map(|event| event.at_us.get()), Some(400));

        // Drain at t=450 - should apply the final future event.
        assert_eq!(
            queue.drain_due(450, &mut plant, &mut trace, &mut levels),
            Ok(())
        );
        assert_eq!(trace.len(), 4);
        assert_eq!(queue.len, 0, "queue should be empty");
    }

    #[test]
    fn crank_period_changes_crank_edge_cadence() {
        let default_report = require_report(run_default_headless_smoke());
        let slower_crank_report = require_report(run_headless_smoke(ScenarioConfig {
            crank_period_us: 1_000,
            ..Default::default()
        }));

        let mut default_crank_edges = 0usize;
        for i in 0..default_report.trace.len() {
            if let Some(rec) = default_report.trace.get(i) {
                if rec.kind == crate::trace::DriverTraceKind::CrankEdge {
                    default_crank_edges += 1;
                }
            }
        }

        let mut slower_crank_edges = 0usize;
        for i in 0..slower_crank_report.trace.len() {
            if let Some(rec) = slower_crank_report.trace.get(i) {
                if rec.kind == crate::trace::DriverTraceKind::CrankEdge {
                    slower_crank_edges += 1;
                }
            }
        }

        assert!(
            slower_crank_edges < default_crank_edges,
            "slower crank period must produce fewer crank-edge records: default={}, slower={}",
            default_crank_edges,
            slower_crank_edges
        );
    }
}
