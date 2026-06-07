use crate::trace::FixedDriverTrace;
use crate::DriverError;
use ecu_domain::{Micros, Rpm};
use ecu_io::OutputTransitionKind;
use ecu_sim::plant::{
    ClosedLoopPlant, FixedPlantProfile, InjectorModel, PlantControls, PlantLimits,
};

use super::config::{default_smoke_init_cfg, DriverRunReport, DriverScenarioSignals};
use super::output::{
    output_trace_record, scenario_dfco_decel_step, scenario_initial_clt_c10,
    scenario_initial_iat_c10, scenario_initial_rpm, scenario_restart_step, scenario_starter_on,
    scenario_sync_loss_gap_step, scenario_sync_loss_gap_us, suppress_output_to_plant,
    OutputLevelTracker, PendingOutputQueue,
};
use super::{ScenarioConfig, ScenarioKind};

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
