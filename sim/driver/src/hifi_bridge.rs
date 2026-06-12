use ecu_board_api::{
    EcuOutput, EdgeKind, OutputLevel, OutputTransitionBatch, SensorSnapshot, TriggerEdge,
};
use ecu_domain::{EnginePhase, Kpa10, Lambda100, Micros, Percent, Rpm, SyncState, Ticks};
use ecu_io::{
    OutputLevel as IoOutputLevel, OutputTransition as IoOutputTransition, OutputTransitionKind,
};
use ecu_sim_hifi::{CylinderCommand, PlantConfigError, PlantStepInput, PlantStepOutput};

use crate::embedded_loop::{
    SimBoard, SimBoardLoop, SimBoardLoopError, SimBoardTraceKind, SimBoardTraceRecord,
    SimEdgePolarity, SimSensorFrame, SimTriggerEdge, SimTriggerLine,
};
use crate::output_validation::{validate_x86_output_channel, OutputChannelKind};
use crate::plant_bridge::X86PlantBridgeDiagnostics;
use crate::x86_runtime_board::{
    run_x86_runtime_tick, X86RuntimeBoard, X86RuntimeBoardError, X86RuntimeTickResult,
};

const HIFI_TRIGGER_NOMINAL_TEETH: u16 = 60;
const HIFI_TRIGGER_MISSING_TEETH: u16 = 2;
const HIFI_TRIGGER_OBSERVED_TEETH: u16 = HIFI_TRIGGER_NOMINAL_TEETH - HIFI_TRIGGER_MISSING_TEETH;
const HIFI_ENGINE_CYCLE_DEG10: f64 = 7200.0;
const HIFI_TOOTH_SPACING_DEG10: f64 = HIFI_ENGINE_CYCLE_DEG10 / HIFI_TRIGGER_NOMINAL_TEETH as f64;
const HIFI_CAM_EDGE_OFFSET_US: u32 = 1;

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct X86HifiCrankReference {
    pub step_start_us: u32,
    pub step_end_us: u32,
    pub step_end_crank_angle_rad: f64,
    pub rpm: f64,
}

#[derive(Clone, Copy)]
struct TriggerWindow {
    step_start_us: u32,
    window_us: u32,
    start_angle_deg10: f64,
    distance_deg10: f64,
}

#[derive(Debug, Clone, PartialEq)]
pub struct X86HifiPlantBridgeFrame {
    pub plant_input: PlantStepInput,
    pub diagnostics: X86PlantBridgeDiagnostics,
}

#[derive(Debug, Clone, PartialEq)]
pub struct X86HifiAdapterStep {
    pub bridge: X86HifiPlantBridgeFrame,
    pub plant_output: PlantStepOutput,
    pub trigger_edges: Vec<SimTriggerEdge>,
    pub sensor_frame: SimSensorFrame,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct X86HifiAdapterStepInput {
    pub now_us: u32,
    pub window_us: u32,
    pub throttle_x1000: u16,
    pub battery_mv: u16,
    pub load_torque_nm_x100: i32,
    pub injector_flow_kg_per_s: f64,
    pub injector_deadtime_us: u32,
    pub crank_ref: Option<X86HifiCrankReference>,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum X86HifiBoardStepError<E> {
    Plant(PlantConfigError),
    Board(E),
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum X86HifiRuntimeTickError {
    Plant(PlantConfigError),
    RuntimeBoard(X86RuntimeBoardError),
}

#[derive(Debug, Clone, PartialEq)]
pub struct X86HifiLoopPlant {
    pub config: ecu_sim_hifi::PlantConfig,
    pub injector_flow_kg_per_s: f64,
    pub injector_deadtime_us: u32,
    pub last_crank_reference: Option<X86HifiCrankReference>,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum X86HifiLoopTickError<E> {
    Loop(SimBoardLoopError<E>),
    Plant(PlantConfigError),
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct X86HifiBridgeParams<'a> {
    pub now_us: u32,
    pub window_us: u32,
    pub throttle_x1000: u16,
    pub load_torque_nm_x100: i32,
    pub injector_flow_kg_per_s: f64,
    pub injector_deadtime_us: u32,
    pub crank_ref: Option<X86HifiCrankReference>,
    pub cylinder_phase_offsets: &'a [f64],
}

pub fn bridge_output_transitions_to_hifi_input<const CYL: usize, const N: usize>(
    transitions: &OutputTransitionBatch<N>,
    params: X86HifiBridgeParams<'_>,
) -> X86HifiPlantBridgeFrame {
    let (crank_angle_rad, rpm) = match params.crank_ref {
        Some(reference) => (reference.step_end_crank_angle_rad, reference.rpm),
        None => (0.0, 250.0),
    };
    let mut frame = X86HifiPlantBridgeFrame {
        plant_input: PlantStepInput {
            now_s: params.now_us as f64 / 1_000_000.0,
            window_s: params.window_us as f64 / 1_000_000.0,
            crank_angle_rad,
            rpm,
            throttle_position: (f64::from(params.throttle_x1000) / 1000.0).clamp(0.0, 1.0),
            load_torque_nm: f64::from(params.load_torque_nm_x100) / 100.0,
            cylinders: vec![
                CylinderCommand {
                    fuel_mass_kg: 0.0,
                    spark_angle_rad: 0.0,
                    dwell_s: 0.0,
                };
                CYL
            ],
        },
        diagnostics: X86PlantBridgeDiagnostics::empty(),
    };
    let mut injector_high_at_us = [None; CYL];
    let mut ignition_high_at_us = [None; CYL];

    for transition in transitions.iter() {
        let Some(validated) = validate_x86_output_channel::<CYL>(transition.output) else {
            frame.diagnostics.out_of_range_channel_count += 1;
            continue;
        };
        let channel = usize::from(validated.channel);
        let high_at_us = match validated.kind {
            OutputChannelKind::Injector => &mut injector_high_at_us[channel],
            OutputChannelKind::Ignition => &mut ignition_high_at_us[channel],
        };

        match transition.level {
            OutputLevel::High => {
                if high_at_us.is_some() {
                    frame.diagnostics.duplicate_high_count += 1;
                } else {
                    *high_at_us = Some(transition.at.get());
                }
            }
            OutputLevel::Low => {
                let Some(start_us) = high_at_us.take() else {
                    frame.diagnostics.orphan_low_count += 1;
                    continue;
                };
                let end_us = transition.at.get();
                if end_us < start_us {
                    frame.diagnostics.out_of_order_transition_count += 1;
                    continue;
                }

                let width_us = end_us - start_us;
                let command = &mut frame.plant_input.cylinders[channel];
                match validated.kind {
                    OutputChannelKind::Injector => {
                        if width_us == 0 {
                            frame.diagnostics.short_pulse_width_count += 1;
                        }
                        let effective_open_us =
                            width_us.saturating_sub(params.injector_deadtime_us);
                        command.fuel_mass_kg += params.injector_flow_kg_per_s
                            * (effective_open_us as f64 / 1_000_000.0);
                    }
                    OutputChannelKind::Ignition => {
                        if width_us == 0 {
                            frame.diagnostics.short_dwell_count += 1;
                        }
                        let spark_angle_rad = params
                            .crank_ref
                            .and_then(|reference| {
                                crank_angle_at_us(reference, end_us).and_then(|fire_angle_rad| {
                                    params
                                        .cylinder_phase_offsets
                                        .get(channel)
                                        .map(|phase_offset| {
                                            ignition_advance_from_fire(
                                                fire_angle_rad,
                                                firing_tdc_for_phase(*phase_offset),
                                            )
                                        })
                                })
                            })
                            .unwrap_or(0.0);
                        command.dwell_s = width_us as f64 / 1_000_000.0;
                        command.spark_angle_rad = spark_angle_rad;
                    }
                }
            }
        }
    }

    frame.diagnostics.open_high_count = injector_high_at_us
        .iter()
        .chain(ignition_high_at_us.iter())
        .filter(|entry| entry.is_some())
        .count() as u32;

    frame
}

fn crank_angle_at_us(reference: X86HifiCrankReference, event_us: u32) -> Option<f64> {
    if reference.rpm <= 0.0 || !reference.rpm.is_finite() {
        return None;
    }
    if event_us < reference.step_start_us || event_us > reference.step_end_us {
        return None;
    }

    let dt_us = f64::from(reference.step_end_us.saturating_sub(event_us));
    let omega_rad_per_s = reference.rpm * core::f64::consts::TAU / 60.0;
    let delta_angle = omega_rad_per_s * (dt_us / 1_000_000.0);
    Some((reference.step_end_crank_angle_rad - delta_angle).rem_euclid(core::f64::consts::TAU))
}

fn firing_tdc_for_phase(phase_offset_rad: f64) -> f64 {
    (phase_offset_rad + core::f64::consts::TAU).rem_euclid(core::f64::consts::TAU)
}

fn ignition_advance_from_fire(fire_angle_rad: f64, firing_tdc_rad: f64) -> f64 {
    let mut advance = (firing_tdc_rad - fire_angle_rad).rem_euclid(core::f64::consts::TAU);
    if advance > core::f64::consts::PI {
        advance -= core::f64::consts::TAU;
    }
    if advance > core::f64::consts::FRAC_PI_2 {
        advance -= core::f64::consts::PI;
    }
    if advance < -core::f64::consts::FRAC_PI_2 {
        advance += core::f64::consts::PI;
    }
    advance
}

pub fn quantize_hifi_output_to_sensor_frame(
    now_us: u32,
    throttle_x1000: u16,
    battery_mv: u16,
    output: &PlantStepOutput,
) -> SimSensorFrame {
    let rpm = output.rpm.round().clamp(0.0, u16::MAX as f64) as u16;
    let crank_angle_deg10 = (output.crank_angle_rad.to_degrees() * 10.0)
        .round()
        .rem_euclid(7200.0) as u16;
    let map_kpa10 = (output.manifold_pressure_pa / 100.0)
        .round()
        .clamp(0.0, u16::MAX as f64) as u16;
    let lambda_x1000 = (output.lambda * 1000.0).round().clamp(0.0, u16::MAX as f64) as u16;
    let knock_intensity_x100 = ((1.0 - output.knock_margin).max(0.0) * 100.0)
        .round()
        .clamp(0.0, u16::MAX as f64) as u16;

    SimSensorFrame {
        timestamp_us: now_us,
        rpm: Rpm::new(rpm),
        crank_angle_deg10,
        map_kpa10,
        tps_x1000: throttle_x1000,
        clt_c10: 0,
        iat_c10: 0,
        lambda_x1000,
        battery_mv,
        knock_intensity_x100,
    }
}

pub fn run_hifi_adapter_step<const CYL: usize, const N: usize>(
    config: &ecu_sim_hifi::PlantConfig,
    transitions: &OutputTransitionBatch<N>,
    input: X86HifiAdapterStepInput,
) -> Result<X86HifiAdapterStep, PlantConfigError> {
    let bridge = bridge_output_transitions_to_hifi_input::<CYL, N>(
        transitions,
        X86HifiBridgeParams {
            now_us: input.now_us,
            window_us: input.window_us,
            throttle_x1000: input.throttle_x1000,
            load_torque_nm_x100: input.load_torque_nm_x100,
            injector_flow_kg_per_s: input.injector_flow_kg_per_s,
            injector_deadtime_us: input.injector_deadtime_us,
            crank_ref: input.crank_ref,
            cylinder_phase_offsets: &config
                .cylinders
                .iter()
                .map(|cylinder| cylinder.phase_offset_rad)
                .collect::<Vec<_>>(),
        },
    );
    let plant_output = ecu_sim_hifi::advance_plant_step(config, &bridge.plant_input)?;
    let trigger_edges = synthesize_hifi_trigger_edges(
        input.now_us,
        input.window_us,
        plant_output.crank_angle_rad,
        plant_output.rpm,
    );
    let sensor_frame = quantize_hifi_output_to_sensor_frame(
        input.now_us,
        input.throttle_x1000,
        input.battery_mv,
        &plant_output,
    );

    Ok(X86HifiAdapterStep {
        bridge,
        plant_output,
        trigger_edges,
        sensor_frame,
    })
}

pub fn drive_hifi_board_step<B, const CYL: usize, const N: usize>(
    board: &mut B,
    config: &ecu_sim_hifi::PlantConfig,
    transitions: &OutputTransitionBatch<N>,
    input: X86HifiAdapterStepInput,
) -> Result<X86HifiAdapterStep, X86HifiBoardStepError<B::Error>>
where
    B: SimBoard<PlantOutputs = X86HifiAdapterStep>,
{
    let step = run_hifi_adapter_step::<CYL, N>(config, transitions, input)
        .map_err(X86HifiBoardStepError::Plant)?;
    board
        .feed_trigger_edges(&step.trigger_edges)
        .map_err(X86HifiBoardStepError::Board)?;
    board
        .feed_sensor_frame(step.sensor_frame)
        .map_err(X86HifiBoardStepError::Board)?;
    board
        .publish_plant_outputs(step.clone())
        .map_err(X86HifiBoardStepError::Board)?;
    Ok(step)
}

pub fn drive_hifi_runtime_board_tick<const CYL: usize, const N: usize>(
    board: &mut X86RuntimeBoard,
    config: &ecu_sim_hifi::PlantConfig,
    transitions: &OutputTransitionBatch<N>,
    input: X86HifiAdapterStepInput,
) -> Result<(X86HifiAdapterStep, X86RuntimeTickResult), X86HifiRuntimeTickError> {
    let step = run_hifi_adapter_step::<CYL, N>(config, transitions, input)
        .map_err(X86HifiRuntimeTickError::Plant)?;
    let runtime_edges = runtime_trigger_edges(&step.trigger_edges);
    let runtime_snapshot = runtime_sensor_snapshot(step.sensor_frame);
    board.set_clock(Micros::new(input.now_us));
    board
        .set_trigger_edges(&runtime_edges)
        .map_err(X86HifiRuntimeTickError::RuntimeBoard)?;
    board.set_sensor_snapshot(runtime_snapshot);
    let tick = run_x86_runtime_tick(board).map_err(X86HifiRuntimeTickError::RuntimeBoard)?;
    Ok((step, tick))
}

pub fn drive_hifi_loop_tick<
    B,
    const CYL: usize,
    const MAX_OUTPUTS: usize,
    const MAX_TRIGGER_EDGES: usize,
    const MAX_TRACE: usize,
>(
    loop_state: &mut SimBoardLoop<B, X86HifiLoopPlant, MAX_TRIGGER_EDGES, MAX_OUTPUTS, MAX_TRACE>,
) -> Result<X86HifiAdapterStep, X86HifiLoopTickError<B::Error>>
where
    B: SimBoard<
        OutputBuffer = OutputTransitionBatch<MAX_OUTPUTS>,
        PlantOutputs = X86HifiAdapterStep,
    >,
{
    let previous_us = loop_state.report.last_now_us;
    let now_us = loop_state.tick().map_err(X86HifiLoopTickError::Loop)?;
    loop_state
        .queue_trace_record(SimBoardTraceRecord::new(
            now_us,
            SimBoardTraceKind::Tick,
            0,
            now_us as i32,
            0,
        ))
        .map_err(X86HifiLoopTickError::Loop)?;

    let driver_input = loop_state
        .board
        .read_driver_input()
        .map_err(|err| X86HifiLoopTickError::Loop(SimBoardLoopError::Board(err)))?;
    loop_state
        .queue_trace_record(SimBoardTraceRecord::new(
            now_us,
            SimBoardTraceKind::DriverInput,
            0,
            i32::from(driver_input.throttle_x1000),
            driver_input.load_torque_nm_x100.get(),
        ))
        .map_err(X86HifiLoopTickError::Loop)?;

    let environment = loop_state
        .board
        .read_environment()
        .map_err(|err| X86HifiLoopTickError::Loop(SimBoardLoopError::Board(err)))?;
    loop_state
        .queue_trace_record(SimBoardTraceRecord::new(
            now_us,
            SimBoardTraceKind::Environment,
            0,
            environment.ambient_pressure_pa,
            i32::from(environment.battery_mv),
        ))
        .map_err(X86HifiLoopTickError::Loop)?;

    let window_us = if loop_state.report.tick_count > 1 {
        now_us.saturating_sub(previous_us)
    } else {
        1
    };

    let mut outputs = OutputTransitionBatch::<MAX_OUTPUTS>::new();
    loop_state
        .board
        .collect_ecu_outputs(&mut outputs)
        .map_err(|err| X86HifiLoopTickError::Loop(SimBoardLoopError::Board(err)))?;
    for transition in outputs.iter() {
        loop_state
            .queue_output_transition(to_io_output_transition(*transition))
            .map_err(X86HifiLoopTickError::Loop)?;
        loop_state
            .queue_trace_record(SimBoardTraceRecord::new(
                transition.at.get(),
                SimBoardTraceKind::OutputTransition,
                match transition.output {
                    EcuOutput::Injector(channel) | EcuOutput::Ignition(channel) => channel.get(),
                },
                match transition.level {
                    OutputLevel::Low => 0,
                    OutputLevel::High => 1,
                },
                0,
            ))
            .map_err(X86HifiLoopTickError::Loop)?;
    }

    let step = run_hifi_adapter_step::<CYL, MAX_OUTPUTS>(
        &loop_state.plant.config,
        &outputs,
        X86HifiAdapterStepInput {
            now_us,
            window_us,
            throttle_x1000: driver_input.throttle_x1000,
            battery_mv: environment.battery_mv,
            load_torque_nm_x100: driver_input.load_torque_nm_x100.get(),
            injector_flow_kg_per_s: loop_state.plant.injector_flow_kg_per_s,
            injector_deadtime_us: loop_state.plant.injector_deadtime_us,
            crank_ref: loop_state.plant.last_crank_reference,
        },
    )
    .map_err(X86HifiLoopTickError::Plant)?;

    loop_state.plant.last_crank_reference = Some(X86HifiCrankReference {
        step_start_us: previous_us,
        step_end_us: now_us,
        step_end_crank_angle_rad: step.plant_output.crank_angle_rad,
        rpm: step.plant_output.rpm,
    });

    for edge in &step.trigger_edges {
        loop_state
            .queue_trigger_edge(*edge)
            .map_err(X86HifiLoopTickError::Loop)?;
        loop_state
            .queue_trace_record(SimBoardTraceRecord::new(
                edge.timestamp_us,
                SimBoardTraceKind::TriggerEdge,
                match edge.line {
                    SimTriggerLine::Crank => 0,
                    SimTriggerLine::Cam => 1,
                },
                i32::from(edge.angle_deg10),
                match edge.polarity {
                    SimEdgePolarity::Rising => 1,
                    SimEdgePolarity::Falling => 0,
                },
            ))
            .map_err(X86HifiLoopTickError::Loop)?;
    }

    loop_state
        .board
        .feed_trigger_edges(&step.trigger_edges)
        .map_err(|err| X86HifiLoopTickError::Loop(SimBoardLoopError::Board(err)))?;
    loop_state
        .board
        .feed_sensor_frame(step.sensor_frame)
        .map_err(|err| X86HifiLoopTickError::Loop(SimBoardLoopError::Board(err)))?;
    loop_state
        .board
        .publish_plant_outputs(step.clone())
        .map_err(|err| X86HifiLoopTickError::Loop(SimBoardLoopError::Board(err)))?;
    loop_state
        .queue_trace_record(SimBoardTraceRecord::new(
            now_us,
            SimBoardTraceKind::PlantOutput,
            0,
            step.sensor_frame.rpm.get() as i32,
            step.sensor_frame.map_kpa10 as i32,
        ))
        .map_err(X86HifiLoopTickError::Loop)?;

    Ok(step)
}

fn to_io_output_transition(transition: ecu_board_api::OutputTransition) -> IoOutputTransition {
    let (kind, channel) = match transition.output {
        EcuOutput::Injector(channel) => (OutputTransitionKind::Injector, channel),
        EcuOutput::Ignition(channel) => (OutputTransitionKind::Ignition, channel),
    };
    let level = match transition.level {
        OutputLevel::Low => IoOutputLevel::Low,
        OutputLevel::High => IoOutputLevel::High,
    };
    IoOutputTransition {
        at_us: Micros::new(transition.at.get()),
        kind,
        channel,
        level,
    }
}

fn runtime_trigger_edges(edges: &[SimTriggerEdge]) -> Vec<TriggerEdge> {
    edges
        .iter()
        .map(|edge| {
            let kind = match edge.polarity {
                SimEdgePolarity::Rising => EdgeKind::Rising,
                SimEdgePolarity::Falling => EdgeKind::Falling,
            };
            TriggerEdge::new(kind, Ticks::new(edge.timestamp_us))
        })
        .collect()
}

fn runtime_sensor_snapshot(frame: SimSensorFrame) -> SensorSnapshot {
    let sync_state = if frame.rpm.get() > 0 {
        SyncState::Locked { cam_ref: false }
    } else {
        SyncState::Unsynced
    };
    let engine_phase = if frame.rpm.get() > 0 {
        EnginePhase::Running
    } else {
        EnginePhase::Off
    };

    SensorSnapshot::new(
        Micros::new(frame.timestamp_us),
        frame.rpm,
        Kpa10::new(frame.map_kpa10),
        Percent::new((frame.tps_x1000 / 10).min(100) as u8),
        frame.clt_c10,
        frame.iat_c10,
        frame.battery_mv,
        Lambda100::new((frame.lambda_x1000 / 10).min(Lambda100::MAX_PLAUSIBLE)),
        sync_state,
        engine_phase,
    )
}

pub fn synthesize_hifi_trigger_edges(
    step_end_us: u32,
    window_us: u32,
    end_crank_angle_rad: f64,
    rpm: f64,
) -> Vec<SimTriggerEdge> {
    if window_us == 0 || !end_crank_angle_rad.is_finite() || !rpm.is_finite() || rpm <= 0.0 {
        return Vec::new();
    }

    let distance_deg10 = rpm * window_us as f64 * 60.0 / 1_000_000.0;
    if !distance_deg10.is_finite() || distance_deg10 <= 0.0 {
        return Vec::new();
    }

    let end_angle_deg10 = normalize_deg10_f64(end_crank_angle_rad.to_degrees() * 10.0);
    let window = TriggerWindow {
        step_start_us: step_end_us.saturating_sub(window_us),
        window_us,
        start_angle_deg10: end_angle_deg10 - distance_deg10,
        distance_deg10,
    };
    let cycle_start = (window.start_angle_deg10 / HIFI_ENGINE_CYCLE_DEG10).floor() as i32 - 1;
    let cycle_end = (end_angle_deg10 / HIFI_ENGINE_CYCLE_DEG10).ceil() as i32 + 1;
    let mut edges = Vec::new();

    for cycle in cycle_start..=cycle_end {
        let cycle_offset_deg10 = f64::from(cycle) * HIFI_ENGINE_CYCLE_DEG10;
        for tooth in 0..HIFI_TRIGGER_OBSERVED_TEETH {
            let tooth_angle_deg10 =
                cycle_offset_deg10 + f64::from(tooth) * HIFI_TOOTH_SPACING_DEG10;
            push_trigger_edge_if_in_window(
                &mut edges,
                window,
                tooth_angle_deg10,
                SimTriggerLine::Crank,
                normalize_deg10_u16(tooth_angle_deg10),
            );
        }

        let cam_window = TriggerWindow {
            step_start_us: window.step_start_us.saturating_add(HIFI_CAM_EDGE_OFFSET_US),
            window_us: window.window_us.saturating_sub(HIFI_CAM_EDGE_OFFSET_US),
            ..window
        };
        push_trigger_edge_if_in_window(
            &mut edges,
            cam_window,
            cycle_offset_deg10,
            SimTriggerLine::Cam,
            normalize_deg10_u16(cycle_offset_deg10),
        );
    }

    edges.sort_unstable_by_key(|edge| {
        let line_order = match edge.line {
            SimTriggerLine::Crank => 1_u8,
            SimTriggerLine::Cam => 0_u8,
        };
        (edge.timestamp_us, line_order, edge.angle_deg10)
    });
    edges
}

fn push_trigger_edge_if_in_window(
    edges: &mut Vec<SimTriggerEdge>,
    window: TriggerWindow,
    edge_angle_deg10: f64,
    line: SimTriggerLine,
    angle_deg10: u16,
) {
    if edge_angle_deg10 <= window.start_angle_deg10
        || edge_angle_deg10 > window.start_angle_deg10 + window.distance_deg10
    {
        return;
    }

    let fraction = (edge_angle_deg10 - window.start_angle_deg10) / window.distance_deg10;
    let timestamp_us = window
        .step_start_us
        .saturating_add((fraction * f64::from(window.window_us)).round() as u32);
    edges.push(SimTriggerEdge::new(
        timestamp_us,
        line,
        SimEdgePolarity::Rising,
        angle_deg10,
    ));
}

fn normalize_deg10_f64(angle_deg10: f64) -> f64 {
    angle_deg10.rem_euclid(HIFI_ENGINE_CYCLE_DEG10)
}

fn normalize_deg10_u16(angle_deg10: f64) -> u16 {
    normalize_deg10_f64(angle_deg10)
        .round()
        .clamp(0.0, u16::MAX as f64) as u16
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::embedded_loop::{SimBoard, SimBoardTraceRecord, SimDriverInput, SimEnvironment};
    use ecu_board_api::EcuOutput;
    use ecu_domain::{ChannelId, Ticks};

    #[derive(Default)]
    struct MockBoard {
        now_us: u32,
        driver_input: SimDriverInput,
        environment: SimEnvironment,
        collected_outputs: OutputTransitionBatch<128>,
        trigger_edges: Vec<SimTriggerEdge>,
        sensor_frames: Vec<SimSensorFrame>,
        plant_outputs: Vec<X86HifiAdapterStep>,
        traces: Vec<SimBoardTraceRecord>,
    }

    impl SimBoard for MockBoard {
        type Error = ();
        type OutputBuffer = OutputTransitionBatch<128>;
        type PlantOutputs = X86HifiAdapterStep;

        fn now_micros(&self) -> u32 {
            self.now_us
        }

        fn read_driver_input(&mut self) -> Result<SimDriverInput, Self::Error> {
            Ok(self.driver_input)
        }

        fn read_environment(&mut self) -> Result<SimEnvironment, Self::Error> {
            Ok(self.environment)
        }

        fn feed_trigger_edges(&mut self, edges: &[SimTriggerEdge]) -> Result<(), Self::Error> {
            self.trigger_edges.extend_from_slice(edges);
            Ok(())
        }

        fn feed_sensor_frame(&mut self, sensors: SimSensorFrame) -> Result<(), Self::Error> {
            self.sensor_frames.push(sensors);
            Ok(())
        }

        fn collect_ecu_outputs(&mut self, out: &mut Self::OutputBuffer) -> Result<(), Self::Error> {
            for transition in self.collected_outputs.iter() {
                out.push(*transition).unwrap();
            }
            Ok(())
        }

        fn publish_plant_outputs(
            &mut self,
            outputs: Self::PlantOutputs,
        ) -> Result<(), Self::Error> {
            self.plant_outputs.push(outputs);
            Ok(())
        }

        fn write_trace(&mut self, record: SimBoardTraceRecord) -> Result<(), Self::Error> {
            self.traces.push(record);
            Ok(())
        }
    }

    fn transition(
        output: EcuOutput,
        level: OutputLevel,
        at_us: u32,
    ) -> ecu_board_api::OutputTransition {
        ecu_board_api::OutputTransition::new(output, level, Ticks::new(at_us))
    }

    fn transition_batch<const N: usize>(
        transitions: [ecu_board_api::OutputTransition; N],
    ) -> OutputTransitionBatch<128> {
        let mut batch = OutputTransitionBatch::<128>::new();
        for transition in transitions {
            batch.push(transition).unwrap();
        }
        batch
    }

    #[test]
    fn injector_and_ignition_pulses_bridge_into_hifi_commands() {
        let batch = transition_batch([
            transition(
                EcuOutput::Injector(ChannelId::new(0)),
                OutputLevel::High,
                100,
            ),
            transition(
                EcuOutput::Injector(ChannelId::new(0)),
                OutputLevel::Low,
                4_100,
            ),
            transition(
                EcuOutput::Ignition(ChannelId::new(1)),
                OutputLevel::High,
                200,
            ),
            transition(
                EcuOutput::Ignition(ChannelId::new(1)),
                OutputLevel::Low,
                1_700,
            ),
        ]);

        let bridged = bridge_output_transitions_to_hifi_input::<2, 128>(
            &batch,
            X86HifiBridgeParams {
                now_us: 10_000,
                window_us: 20_000,
                throttle_x1000: 500,
                load_torque_nm_x100: 2_500,
                injector_flow_kg_per_s: 0.02,
                injector_deadtime_us: 500,
                crank_ref: None,
                cylinder_phase_offsets: &[0.0, core::f64::consts::PI],
            },
        );

        assert!(bridged.diagnostics.is_clean());
        assert!((bridged.plant_input.now_s - 0.01).abs() < 1.0e-12);
        assert!((bridged.plant_input.window_s - 0.02).abs() < 1.0e-12);
        assert!((bridged.plant_input.load_torque_nm - 25.0).abs() < 1.0e-12);
        assert!((bridged.plant_input.throttle_position - 0.5).abs() < 1.0e-12);
        assert!((bridged.plant_input.cylinders[0].fuel_mass_kg - 0.00007).abs() < 1.0e-12);
        assert!((bridged.plant_input.cylinders[1].dwell_s - 0.0015).abs() < 1.0e-12);
    }

    #[test]
    fn bridge_reports_duplicate_orphan_and_open_high_diagnostics() {
        let batch = transition_batch([
            transition(
                EcuOutput::Injector(ChannelId::new(0)),
                OutputLevel::High,
                100,
            ),
            transition(
                EcuOutput::Injector(ChannelId::new(0)),
                OutputLevel::High,
                120,
            ),
            transition(
                EcuOutput::Ignition(ChannelId::new(1)),
                OutputLevel::Low,
                140,
            ),
        ]);

        let bridged = bridge_output_transitions_to_hifi_input::<2, 128>(
            &batch,
            X86HifiBridgeParams {
                now_us: 0,
                window_us: 10_000,
                throttle_x1000: 0,
                load_torque_nm_x100: 0,
                injector_flow_kg_per_s: 0.02,
                injector_deadtime_us: 0,
                crank_ref: None,
                cylinder_phase_offsets: &[0.0, core::f64::consts::PI],
            },
        );

        assert_eq!(bridged.diagnostics.duplicate_high_count, 1);
        assert_eq!(bridged.diagnostics.orphan_low_count, 1);
        assert_eq!(bridged.diagnostics.open_high_count, 1);
    }

    #[test]
    fn bridge_accumulates_multiple_injection_pulses_per_cylinder() {
        let batch = transition_batch([
            transition(
                EcuOutput::Injector(ChannelId::new(0)),
                OutputLevel::High,
                100,
            ),
            transition(
                EcuOutput::Injector(ChannelId::new(0)),
                OutputLevel::Low,
                1_100,
            ),
            transition(
                EcuOutput::Injector(ChannelId::new(0)),
                OutputLevel::High,
                2_000,
            ),
            transition(
                EcuOutput::Injector(ChannelId::new(0)),
                OutputLevel::Low,
                4_000,
            ),
        ]);

        let bridged = bridge_output_transitions_to_hifi_input::<1, 128>(
            &batch,
            X86HifiBridgeParams {
                now_us: 0,
                window_us: 10_000,
                throttle_x1000: 0,
                load_torque_nm_x100: 0,
                injector_flow_kg_per_s: 0.01,
                injector_deadtime_us: 0,
                crank_ref: None,
                cylinder_phase_offsets: &[0.0],
            },
        );

        assert!((bridged.plant_input.cylinders[0].fuel_mass_kg - 0.00003).abs() < 1.0e-12);
    }

    #[test]
    fn ignition_high_to_low_transition_sets_spark_advance_from_fire_timing() {
        let batch = transition_batch([
            transition(
                EcuOutput::Ignition(ChannelId::new(0)),
                OutputLevel::High,
                200,
            ),
            transition(
                EcuOutput::Ignition(ChannelId::new(0)),
                OutputLevel::Low,
                8_888,
            ),
        ]);
        let firing_tdc_deg10 = 200.0_f64;
        let firing_tdc_rad = firing_tdc_deg10.to_radians();

        let bridged = bridge_output_transitions_to_hifi_input::<1, 128>(
            &batch,
            X86HifiBridgeParams {
                now_us: 10_000,
                window_us: 10_000,
                throttle_x1000: 500,
                load_torque_nm_x100: 0,
                injector_flow_kg_per_s: 0.0,
                injector_deadtime_us: 0,
                crank_ref: Some(X86HifiCrankReference {
                    step_start_us: 0,
                    step_end_us: 10_000,
                    step_end_crank_angle_rad: 20.0_f64.to_radians(),
                    rpm: 3_000.0,
                }),
                cylinder_phase_offsets: &[firing_tdc_rad],
            },
        );

        assert!(
            (bridged.plant_input.cylinders[0].spark_angle_rad - 20.0_f64.to_radians()).abs() < 0.01,
            "spark advance was {}",
            bridged.plant_input.cylinders[0].spark_angle_rad,
        );
    }

    #[test]
    fn quantized_sensor_frame_maps_hifi_output_into_driver_units() {
        let output = PlantStepOutput {
            crank_angle_rad: core::f64::consts::PI,
            rpm: 1234.6,
            manifold_pressure_pa: 98_765.0,
            lambda: 1.012,
            egt_k: 900.0,
            knock_margin: 0.75,
            brake_torque_nm: 42.0,
            cylinders: Vec::new(),
        };

        let frame = quantize_hifi_output_to_sensor_frame(42_000, 333, 12_800, &output);

        assert_eq!(frame.timestamp_us, 42_000);
        assert_eq!(frame.rpm.get(), 1235);
        assert_eq!(frame.crank_angle_deg10, 1800);
        assert_eq!(frame.map_kpa10, 988);
        assert_eq!(frame.tps_x1000, 333);
        assert_eq!(frame.lambda_x1000, 1012);
        assert_eq!(frame.battery_mv, 12_800);
        assert_eq!(frame.knock_intensity_x100, 25);
    }

    fn test_hifi_config() -> ecu_sim_hifi::PlantConfig {
        let mut cfg = ecu_sim_hifi::default_plant_config();
        cfg.cylinders = vec![
            ecu_sim_hifi::PlantCylinderConfig {
                phase_offset_rad: 0.0,
            },
            ecu_sim_hifi::PlantCylinderConfig {
                phase_offset_rad: core::f64::consts::PI,
            },
        ];
        cfg.combustion.spark_angle_rad = 15.0_f64.to_radians();
        cfg
    }

    #[test]
    fn board_step_publishes_trigger_edges_sensor_frame_and_outputs() {
        let batch = transition_batch([
            transition(
                EcuOutput::Injector(ChannelId::new(0)),
                OutputLevel::High,
                100,
            ),
            transition(
                EcuOutput::Injector(ChannelId::new(0)),
                OutputLevel::Low,
                4_100,
            ),
            transition(
                EcuOutput::Ignition(ChannelId::new(1)),
                OutputLevel::High,
                200,
            ),
            transition(
                EcuOutput::Ignition(ChannelId::new(1)),
                OutputLevel::Low,
                1_700,
            ),
        ]);
        let mut board = MockBoard::default();

        let step = drive_hifi_board_step::<_, 2, 128>(
            &mut board,
            &test_hifi_config(),
            &batch,
            X86HifiAdapterStepInput {
                now_us: 10_000,
                window_us: 20_000,
                throttle_x1000: 500,
                battery_mv: 12_800,
                load_torque_nm_x100: 2_500,
                injector_flow_kg_per_s: 0.02,
                injector_deadtime_us: 500,
                crank_ref: None,
            },
        )
        .unwrap();

        assert_eq!(board.sensor_frames, vec![step.sensor_frame]);
        assert_eq!(board.plant_outputs, vec![step.clone()]);
        assert_eq!(board.trigger_edges, step.trigger_edges);
        assert!(!board.trigger_edges.is_empty());
    }

    #[test]
    fn loop_tick_collects_outputs_and_updates_loop_report() {
        let collected_outputs = transition_batch([
            transition(
                EcuOutput::Injector(ChannelId::new(0)),
                OutputLevel::High,
                100,
            ),
            transition(
                EcuOutput::Injector(ChannelId::new(0)),
                OutputLevel::Low,
                4_100,
            ),
            transition(
                EcuOutput::Ignition(ChannelId::new(1)),
                OutputLevel::High,
                200,
            ),
            transition(
                EcuOutput::Ignition(ChannelId::new(1)),
                OutputLevel::Low,
                1_700,
            ),
        ]);
        let board = MockBoard {
            now_us: 10_000,
            driver_input: SimDriverInput {
                throttle_x1000: 500,
                load_torque_nm_x100: crate::embedded_loop::TorqueNmX100::new(2_500),
                ..SimDriverInput::idle()
            },
            environment: SimEnvironment {
                battery_mv: 12_800,
                ..SimEnvironment::standard()
            },
            collected_outputs,
            ..MockBoard::default()
        };
        let plant = X86HifiLoopPlant {
            config: test_hifi_config(),
            injector_flow_kg_per_s: 0.02,
            injector_deadtime_us: 500,
            last_crank_reference: None,
        };
        let mut loop_state = SimBoardLoop::<_, _, 128, 128, 128>::new(
            board,
            plant,
            crate::SimBoardLoopConfig::default(),
        );

        let step = drive_hifi_loop_tick::<_, 2, 128, 128, 128>(&mut loop_state).unwrap();

        assert_eq!(loop_state.report.tick_count, 1);
        assert_eq!(loop_state.report.output_transitions_queued, 4);
        assert_eq!(
            loop_state.report.trigger_edges_queued,
            step.trigger_edges.len() as u64
        );
        assert_eq!(loop_state.board.sensor_frames, vec![step.sensor_frame]);
        assert_eq!(loop_state.board.plant_outputs, vec![step.clone()]);
        assert_eq!(loop_state.board.trigger_edges, step.trigger_edges);
        assert!(!loop_state.trace().is_empty());
    }

    #[test]
    fn synthesized_trigger_edges_cover_one_full_cycle_with_60_minus_2_and_cam() {
        let edges = synthesize_hifi_trigger_edges(40_000, 40_000, 0.0, 3000.0);

        let crank_edges = edges
            .iter()
            .filter(|edge| edge.line == SimTriggerLine::Crank)
            .count();
        let cam_edges = edges
            .iter()
            .filter(|edge| edge.line == SimTriggerLine::Cam)
            .count();

        assert_eq!(crank_edges, usize::from(HIFI_TRIGGER_OBSERVED_TEETH));
        assert_eq!(cam_edges, 1);
        assert!(edges
            .windows(2)
            .all(|window| window[0].timestamp_us <= window[1].timestamp_us));
        assert!(edges
            .iter()
            .any(|edge| edge.line == SimTriggerLine::Cam && edge.angle_deg10 == 0));
        assert!(edges
            .iter()
            .any(|edge| edge.line == SimTriggerLine::Crank && edge.angle_deg10 == 6840));
        assert!(edges
            .iter()
            .any(|edge| edge.line == SimTriggerLine::Crank && edge.angle_deg10 == 0));
    }

    #[test]
    fn synthesized_trigger_edges_remain_monotonic_across_cycle_wrap() {
        let edges = synthesize_hifi_trigger_edges(25_000, 10_000, 20.0_f64.to_radians(), 2000.0);

        assert!(!edges.is_empty());
        assert!(edges
            .windows(2)
            .all(|window| window[0].timestamp_us <= window[1].timestamp_us));
        assert!(edges.iter().any(|edge| edge.line == SimTriggerLine::Cam));
        assert!(edges.iter().any(|edge| edge.angle_deg10 < 900));
        assert!(edges.iter().any(|edge| edge.angle_deg10 > 6000));
    }
}
