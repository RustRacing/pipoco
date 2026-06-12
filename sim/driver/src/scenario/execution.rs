use crate::trace::FixedDriverTrace;
use crate::DriverError;
use ecu_board_api::OutputTransitionBatch;
use ecu_domain::{Micros, Rpm};
use ecu_io::OutputTransitionKind;
use ecu_sim::plant::{
    ClosedLoopPlant, FixedPlantProfile, InjectorModel, PlantControls, PlantLimits,
};
use ecu_sim_hifi::PlantConfig as HifiPlantConfig;

use super::config::{
    default_smoke_init_cfg, DriverRunReport, DriverScenarioSignals, HifiDriverRunReport,
};
use super::output::{
    output_trace_record, scenario_dfco_decel_step, scenario_initial_clt_c10,
    scenario_initial_iat_c10, scenario_initial_rpm, scenario_restart_step, scenario_starter_on,
    scenario_sync_loss_gap_step, scenario_sync_loss_gap_us, suppress_output_to_plant,
    OutputLevelTracker, PendingOutputQueue,
};
use super::{ScenarioBackend, ScenarioConfig, ScenarioKind};

/// Run the headless smoke scenario with the given configuration.
pub fn run_headless_smoke(config: ScenarioConfig) -> Result<DriverRunReport, DriverError> {
    match config.backend {
        ScenarioBackend::Harness => run_headless_smoke_harness(config),
        ScenarioBackend::Hifi => run_headless_hifi_smoke(config),
    }
}

fn run_headless_smoke_harness(config: ScenarioConfig) -> Result<DriverRunReport, DriverError> {
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
    let plant_profile = FixedPlantProfile::inline_four();
    let mut plant = ClosedLoopPlant::new(plant_profile, InjectorModel::gasoline(240)).with_limits(
        PlantLimits {
            min_dwell_us: 400,
            ..conservative_limits
        },
    );
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
            crate::output_validation::validate_scenario_output_channel(plant_profile, transition)?;
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
        hifi_step: None,
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

/// Run the default headless smoke scenario against a selected backend.
pub fn run_default_headless_scenario(
    backend: ScenarioBackend,
) -> Result<DriverRunReport, DriverError> {
    run_headless_smoke(ScenarioConfig::default().with_backend(backend))
}

/// Run a deterministic cold-start scenario.
pub fn run_cold_start_scenario() -> Result<DriverRunReport, DriverError> {
    run_headless_smoke(ScenarioConfig::cold_start())
}

/// Run a deterministic cold-start scenario against a selected backend.
pub fn run_cold_start_scenario_with_backend(
    backend: ScenarioBackend,
) -> Result<DriverRunReport, DriverError> {
    run_headless_smoke(ScenarioConfig::cold_start().with_backend(backend))
}

/// Run a deterministic hot-restart scenario.
pub fn run_hot_restart_scenario() -> Result<DriverRunReport, DriverError> {
    run_headless_smoke(ScenarioConfig::hot_restart())
}

/// Run a deterministic hot-restart scenario against a selected backend.
pub fn run_hot_restart_scenario_with_backend(
    backend: ScenarioBackend,
) -> Result<DriverRunReport, DriverError> {
    run_headless_smoke(ScenarioConfig::hot_restart().with_backend(backend))
}

/// Run a deterministic DFCO decel scenario.
pub fn run_dfco_decel_scenario() -> Result<DriverRunReport, DriverError> {
    run_headless_smoke(ScenarioConfig::dfco_decel())
}

/// Run a deterministic DFCO decel scenario against a selected backend.
pub fn run_dfco_decel_scenario_with_backend(
    backend: ScenarioBackend,
) -> Result<DriverRunReport, DriverError> {
    run_headless_smoke(ScenarioConfig::dfco_decel().with_backend(backend))
}

/// Run a deterministic sync-loss recovery scenario.
pub fn run_sync_loss_recovery_scenario() -> Result<DriverRunReport, DriverError> {
    run_headless_smoke(ScenarioConfig::sync_loss_recovery())
}

/// Run a deterministic sync-loss recovery scenario against a selected backend.
pub fn run_sync_loss_recovery_scenario_with_backend(
    backend: ScenarioBackend,
) -> Result<DriverRunReport, DriverError> {
    run_headless_smoke(ScenarioConfig::sync_loss_recovery().with_backend(backend))
}

/// Run the headless scenario against the hifi plant instead of sim/harness.
pub fn run_headless_hifi_smoke(config: ScenarioConfig) -> Result<HifiDriverRunReport, DriverError> {
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
    let mut last_sensor_frame = initial_hifi_sensor_frame(config.start_us, config.kind);
    let mut last_hifi_step = initial_hifi_adapter_step(config.start_us, config.kind);
    let hifi_config = default_hifi_scenario_config();

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

        let mut throttle_x100 = config.throttle_x100;
        let mut load_torque_x100 = config.load_torque_x100;
        let _starter_on = scenario_starter_on(config, step_index, restart_step);
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

        let mut ecu_sensor_frame = last_sensor_frame;
        ecu_sensor_frame.at_us = Micros::new(now_us);
        ecu_sensor_frame.tps_x100 = throttle_x100;
        ecu_sensor_frame.clt_c10 = clt_c10;
        ecu_sensor_frame.iat_c10 = iat_c10;
        ecu_sensor_frame.vbatt_mv = 12_500;
        trace.push(crate::trace::DriverTraceRecord {
            at_us: now_us,
            kind: crate::trace::DriverTraceKind::Sensor,
            status: 0,
            rpm: ecu_sensor_frame.rpm.get(),
            map_kpa10: ecu_sensor_frame.map_kpa10.get(),
            angle_x10: ecu_sensor_frame.angle_x10.get(),
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

        client.set_sensors(crate::ffi_client::sensor_frame_to_ffi(ecu_sensor_frame))?;
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

        while next_crank_edge_us <= now_us {
            client.on_crank_edge(next_crank_edge_us)?;
            trace.push(crate::trace::DriverTraceRecord {
                at_us: next_crank_edge_us,
                kind: crate::trace::DriverTraceKind::CrankEdge,
                status: 0,
                rpm: ecu_sensor_frame.rpm.get(),
                map_kpa10: ecu_sensor_frame.map_kpa10.get(),
                angle_x10: ecu_sensor_frame.angle_x10.get(),
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

            if !cam_edge_emitted && crank_edges_seen == 2 {
                client.on_cam_edge(next_crank_edge_us)?;
                trace.push(crate::trace::DriverTraceRecord {
                    at_us: next_crank_edge_us,
                    kind: crate::trace::DriverTraceKind::CamEdge,
                    status: 0,
                    rpm: ecu_sensor_frame.rpm.get(),
                    map_kpa10: ecu_sensor_frame.map_kpa10.get(),
                    angle_x10: ecu_sensor_frame.angle_x10.get(),
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

        client.step(now_us)?;
        trace.push(crate::trace::DriverTraceRecord {
            at_us: now_us,
            kind: crate::trace::DriverTraceKind::Step,
            status: 0,
            rpm: ecu_sensor_frame.rpm.get(),
            map_kpa10: ecu_sensor_frame.map_kpa10.get(),
            angle_x10: ecu_sensor_frame.angle_x10.get(),
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

        let mut events = [ecu_sim_ffi::EcuSimOutputEvent::ZERO; ecu_sim_ffi::ECU_SIM_MAX_EVENTS];
        let count = client.dequeue_events(&mut events)?;
        if count > config.max_events_per_step {
            return Err(DriverError::EventOverflow);
        }

        for event in &events[..count] {
            let transition = crate::ffi_client::output_event_to_transition(*event)?;
            crate::output_validation::validate_scenario_output_channel(
                FixedPlantProfile::inline_four(),
                transition,
            )?;
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
                    scenario_signals.dfco_suppressed_injection_outputs = scenario_signals
                        .dfco_suppressed_injection_outputs
                        .saturating_add(1);
                }
            } else {
                pending_queue.push_sorted(transition)?;
            }
        }

        let mut plant_batch = OutputTransitionBatch::<256>::new();
        pending_queue.drain_due_to_hifi_batch(
            now_us,
            &mut trace,
            &mut output_levels,
            &mut plant_batch,
        )?;

        last_hifi_step = crate::run_hifi_adapter_step::<4, 256>(
            &hifi_config,
            &plant_batch,
            crate::X86HifiAdapterStepInput {
                now_us,
                window_us: config.tick_period_us.max(1),
                throttle_x1000: throttle_x100 / 10,
                battery_mv: 12_500,
                load_torque_nm_x100: load_torque_x100,
                injector_flow_kg_per_s: hifi_config.injector.injector_flow_kg_per_s,
                injector_deadtime_us: (hifi_config.injector.injector_deadtime_s * 1_000_000.0)
                    .round() as u32,
                crank_ref: Some(crate::X86HifiCrankReference {
                    step_start_us: now_us.saturating_sub(config.tick_period_us.max(1)),
                    step_end_us: now_us,
                    step_end_crank_angle_rad: last_hifi_step.plant_output.crank_angle_rad,
                    rpm: last_hifi_step.plant_output.rpm,
                }),
            },
        )
        .map_err(DriverError::HifiPlant)?;
        last_sensor_frame =
            sensor_frame_from_hifi_step(last_hifi_step.sensor_frame, clt_c10, iat_c10);

        trace.push(crate::trace::DriverTraceRecord {
            at_us: now_us,
            kind: crate::trace::DriverTraceKind::PlantAdvance,
            status: 0,
            rpm: last_hifi_step.sensor_frame.rpm.get(),
            map_kpa10: last_hifi_step.sensor_frame.map_kpa10,
            angle_x10: last_sensor_frame.angle_x10.get(),
            output_kind: -1,
            channel: 0,
            high: 0,
            combustion_events: u32::from(last_hifi_step.plant_output.brake_torque_nm > 0.0),
            synced: 0,
            tooth: 0,
            diagnostic_code: 0,
            fault_severity: 0,
            cancel_reason: 0,
            control_mode: 0,
            observability: empty_observability,
        })?;
    }

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
    let mut final_batch = OutputTransitionBatch::<256>::new();
    pending_queue.drain_due_to_hifi_batch(
        final_due_us,
        &mut trace,
        &mut output_levels,
        &mut final_batch,
    )?;
    if !final_batch.is_empty() {
        last_hifi_step = crate::run_hifi_adapter_step::<4, 256>(
            &hifi_config,
            &final_batch,
            crate::X86HifiAdapterStepInput {
                now_us: final_due_us,
                window_us: config.tick_period_us.max(1),
                throttle_x1000: 0,
                battery_mv: 12_500,
                load_torque_nm_x100: 0,
                injector_flow_kg_per_s: hifi_config.injector.injector_flow_kg_per_s,
                injector_deadtime_us: (hifi_config.injector.injector_deadtime_s * 1_000_000.0)
                    .round() as u32,
                crank_ref: Some(crate::X86HifiCrankReference {
                    step_start_us: final_due_us.saturating_sub(config.tick_period_us.max(1)),
                    step_end_us: final_due_us,
                    step_end_crank_angle_rad: last_hifi_step.plant_output.crank_angle_rad,
                    rpm: last_hifi_step.plant_output.rpm,
                }),
            },
        )
        .map_err(DriverError::HifiPlant)?;
        last_sensor_frame = sensor_frame_from_hifi_step(
            last_hifi_step.sensor_frame,
            last_sensor_frame.clt_c10,
            last_sensor_frame.iat_c10,
        );
    }

    let ecu_snapshot = client.snapshot()?;
    let observability = crate::ffi_client::observability_from_snapshot_and_sensor_frame(
        &ecu_snapshot,
        last_sensor_frame,
    );
    trace.push(crate::trace::DriverTraceRecord {
        at_us: final_due_us,
        kind: crate::trace::DriverTraceKind::Snapshot,
        status: 0,
        rpm: ecu_snapshot.rpm,
        map_kpa10: last_sensor_frame.map_kpa10.get(),
        angle_x10: ecu_snapshot.angle_x10,
        output_kind: -1,
        channel: 0,
        high: 0,
        combustion_events: u32::from(last_hifi_step.plant_output.brake_torque_nm > 0.0),
        synced: ecu_snapshot.synced,
        tooth: ecu_snapshot.tooth,
        diagnostic_code: ecu_snapshot.fault_code,
        fault_severity: ecu_snapshot.fault_severity,
        cancel_reason: ecu_snapshot.cancel_reason,
        control_mode: ecu_snapshot.control_mode,
        observability,
    })?;

    if ecu_snapshot.synced != 1 {
        return Err(DriverError::ScenarioDidNotSync);
    }
    if injection_outputs == 0 {
        return Err(DriverError::ScenarioDidNotEmitInjection);
    }
    if ignition_outputs == 0 {
        return Err(DriverError::ScenarioDidNotEmitIgnition);
    }
    if trace.overflow_count() > 0 {
        return Err(DriverError::TraceOverflow);
    }
    if pending_queue.pending_overflow() > 0 {
        return Err(DriverError::PendingOutputOverflow);
    }

    let final_controls = PlantControls {
        throttle_x100: last_sensor_frame.tps_x100,
        starter_on: false,
        load_torque_x100: 0,
        vbatt_mv: last_sensor_frame.vbatt_mv,
        fault: ecu_sim::plant::PlantFault::None,
    };

    let combustion_events = u32::from(last_hifi_step.plant_output.brake_torque_nm > 0.0);
    Ok(HifiDriverRunReport {
        trace,
        ecu_snapshot,
        plant_snapshot: synthesize_hifi_plant_snapshot(
            final_due_us,
            final_controls,
            &last_sensor_frame,
            &last_hifi_step,
            injection_outputs,
            ignition_outputs,
        ),
        hifi_step: Some(last_hifi_step),
        observability,
        scenario_signals,
        total_outputs,
        injection_outputs,
        ignition_outputs,
        combustion_events,
        pending_overflow_count: pending_queue.pending_overflow(),
    })
}

fn default_hifi_scenario_config() -> HifiPlantConfig {
    ecu_sim_hifi::default_plant_config()
}

fn initial_hifi_sensor_frame(start_us: u32, kind: ScenarioKind) -> ecu_io::SensorFrame {
    ecu_io::SensorFrame {
        at_us: Micros::new(start_us),
        rpm: Rpm::new(scenario_initial_rpm(kind)),
        map_kpa10: ecu_domain::Kpa10::new(900),
        maf_x100: ecu_domain::MassAirFlowX100::new(0),
        maf_valid: false,
        knock_x100: ecu_domain::KnockLevelX100::new(0),
        knock_valid: false,
        cam_phase_deg10: None,
        angle_x10: ecu_domain::Degrees10::new(0),
        tps_x100: 0,
        clt_c10: scenario_initial_clt_c10(kind),
        iat_c10: scenario_initial_iat_c10(kind),
        vbatt_mv: 12_500,
        baro_kpa10: ecu_domain::Kpa10::new(1013),
        vehicle_speed_kph10: ecu_domain::VehicleSpeedKph10::new(0),
        vehicle_speed_valid: false,
        lambda_valid: true,
        lambda_x100: ecu_domain::Lambda100::new(100),
    }
}

fn initial_hifi_adapter_step(start_us: u32, kind: ScenarioKind) -> crate::X86HifiAdapterStep {
    crate::X86HifiAdapterStep {
        bridge: crate::X86HifiPlantBridgeFrame {
            plant_input: ecu_sim_hifi::PlantStepInput {
                now_s: start_us as f64 / 1_000_000.0,
                window_s: 1.0e-6,
                crank_angle_rad: 0.0,
                rpm: f64::from(scenario_initial_rpm(kind)),
                throttle_position: 0.0,
                load_torque_nm: 0.0,
                cylinders: vec![
                    ecu_sim_hifi::CylinderCommand {
                        fuel_mass_kg: 0.0,
                        spark_angle_rad: 0.0,
                        dwell_s: 0.0,
                    };
                    4
                ],
            },
            diagnostics: crate::X86PlantBridgeDiagnostics::empty(),
        },
        plant_output: ecu_sim_hifi::PlantStepOutput {
            crank_angle_rad: 0.0,
            rpm: f64::from(scenario_initial_rpm(kind)),
            manifold_pressure_pa: 90_000.0,
            lambda: 1.0,
            egt_k: 850.0,
            knock_margin: 1.0,
            brake_torque_nm: 0.0,
            cylinders: Vec::new(),
        },
        trigger_edges: Vec::new(),
        sensor_frame: crate::SimSensorFrame {
            timestamp_us: start_us,
            rpm: Rpm::new(scenario_initial_rpm(kind)),
            crank_angle_deg10: 0,
            map_kpa10: 900,
            tps_x1000: 0,
            clt_c10: scenario_initial_clt_c10(kind),
            iat_c10: scenario_initial_iat_c10(kind),
            lambda_x1000: 1000,
            battery_mv: 12_500,
            knock_intensity_x100: 0,
        },
    }
}

fn sensor_frame_from_hifi_step(
    step: crate::SimSensorFrame,
    clt_c10: i16,
    iat_c10: i16,
) -> ecu_io::SensorFrame {
    ecu_io::SensorFrame {
        at_us: Micros::new(step.timestamp_us),
        rpm: step.rpm,
        map_kpa10: ecu_domain::Kpa10::new(step.map_kpa10),
        maf_x100: ecu_domain::MassAirFlowX100::new(0),
        maf_valid: false,
        knock_x100: ecu_domain::KnockLevelX100::new(step.knock_intensity_x100),
        knock_valid: true,
        cam_phase_deg10: None,
        angle_x10: ecu_domain::Degrees10::new(step.crank_angle_deg10 as i16),
        tps_x100: step.tps_x1000.saturating_mul(10),
        clt_c10,
        iat_c10,
        vbatt_mv: step.battery_mv,
        baro_kpa10: ecu_domain::Kpa10::new(1013),
        vehicle_speed_kph10: ecu_domain::VehicleSpeedKph10::new(0),
        vehicle_speed_valid: false,
        lambda_valid: true,
        lambda_x100: ecu_domain::Lambda100::new((step.lambda_x1000 / 10).min(255)),
    }
}

fn synthesize_hifi_plant_snapshot(
    now_us: u32,
    controls: PlantControls,
    sensor_frame: &ecu_io::SensorFrame,
    step: &crate::X86HifiAdapterStep,
    injection_outputs: u16,
    ignition_outputs: u16,
) -> ecu_sim::plant::PlantSnapshot {
    ecu_sim::plant::PlantSnapshot {
        now_us: Micros::new(now_us),
        rpm: sensor_frame.rpm,
        map_kpa10: sensor_frame.map_kpa10,
        crank_angle_deg10: sensor_frame.angle_x10,
        throttle_x100: controls.throttle_x100,
        vbatt_mv: controls.vbatt_mv,
        last_torque_x100: (step.plant_output.brake_torque_nm * 100.0).round() as i32,
        combustion_events: u32::from(step.plant_output.brake_torque_nm > 0.0),
        injector_events: u32::from(injection_outputs),
        spark_events: u32::from(ignition_outputs),
    }
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
