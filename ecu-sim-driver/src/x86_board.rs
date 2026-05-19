//! x86 board glue for the deterministic simulator driver.
//!
//! This module owns the host-side `EcuApp`, deterministic time, pin capture,
//! the x86 plant bridge, and trace/report bookkeeping. It keeps the adapter
//! surface board-owned so tests can translate captured output transitions into
//! plant frames without re-implementing the mapping logic.

use core::cell::Cell;
use std::rc::Rc;

use ecu_core::app::EcuApp;
use ecu_core::hal::{OutputPin, TimeSource};
use ecu_domain::{ChannelId, Micros};
use ecu_io::{OutputLevel, OutputTransition, OutputTransitionKind};
use ecu_sim_core as core_plant;

use crate::embedded_loop::{
    FixedOutputQueue, FixedTraceBuffer, SimBoard, SimBoardLoopConfig, SimBoardLoopReport,
    SimBoardTraceKind, SimBoardTraceRecord, SimBufferOverflow, SimDriverInput, SimEdgePolarity,
    SimEnvironment, SimSensorFrame, SimTriggerEdge, SimTriggerLine,
};

/// Board-side errors.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum X86BoardError {
    TimeWentBackwards { previous_us: u32, now_us: u32 },
    OutputOverflow(SimBufferOverflow),
    TraceOverflow(SimBufferOverflow),
}

/// Explicit channel mapping and adapter calibration for the x86 plant bridge.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct X86PlantBridgeConfig {
    pub injector_cylinders: [Option<core_plant::CylinderIndex>; 8],
    pub ignition_cylinders: [Option<core_plant::CylinderIndex>; 8],
    pub approximate_target_fuel_ug_per_injection: u32,
    pub approximate_min_pulse_width_us: u32,
    pub approximate_min_spark_dwell_us: u32,
    pub coil_energy_x1000: u16,
}

impl X86PlantBridgeConfig {
    pub const fn new(
        injector_cylinders: [Option<core_plant::CylinderIndex>; 8],
        ignition_cylinders: [Option<core_plant::CylinderIndex>; 8],
        approximate_target_fuel_ug_per_injection: u32,
        approximate_min_pulse_width_us: u32,
        approximate_min_spark_dwell_us: u32,
        coil_energy_x1000: u16,
    ) -> Self {
        Self {
            injector_cylinders,
            ignition_cylinders,
            approximate_target_fuel_ug_per_injection,
            approximate_min_pulse_width_us,
            approximate_min_spark_dwell_us,
            coil_energy_x1000,
        }
    }
}

impl Default for X86PlantBridgeConfig {
    fn default() -> Self {
        Self::new(
            [
                Some(core_plant::CylinderIndex(0)),
                Some(core_plant::CylinderIndex(1)),
                None,
                None,
                None,
                None,
                None,
                None,
            ],
            [
                None,
                None,
                Some(core_plant::CylinderIndex(0)),
                Some(core_plant::CylinderIndex(1)),
                None,
                None,
                None,
                None,
            ],
            3_500,
            1,
            3_000,
            1_000,
        )
    }
}

/// Diagnostics emitted by the x86 plant bridge.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct X86PlantBridgeDiagnostics {
    pub duplicate_high_count: u32,
    pub orphan_low_count: u32,
    pub open_high_count: u32,
    pub unmapped_channel_count: u32,
    pub out_of_range_channel_count: u32,
    pub out_of_order_transition_count: u32,
    pub short_pulse_width_count: u32,
    pub short_dwell_count: u32,
    pub capacity_overflow_count: u32,
}

impl X86PlantBridgeDiagnostics {
    pub const fn empty() -> Self {
        Self {
            duplicate_high_count: 0,
            orphan_low_count: 0,
            open_high_count: 0,
            unmapped_channel_count: 0,
            out_of_range_channel_count: 0,
            out_of_order_transition_count: 0,
            short_pulse_width_count: 0,
            short_dwell_count: 0,
            capacity_overflow_count: 0,
        }
    }

    pub const fn is_clean(self) -> bool {
        self.duplicate_high_count == 0
            && self.orphan_low_count == 0
            && self.open_high_count == 0
            && self.unmapped_channel_count == 0
            && self.out_of_range_channel_count == 0
            && self.out_of_order_transition_count == 0
            && self.short_pulse_width_count == 0
            && self.short_dwell_count == 0
            && self.capacity_overflow_count == 0
    }
}

/// Bridge result that carries the translated ECU output frame and diagnostics.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct X86PlantBridgeFrame<const CYL: usize, const MAX_EVENTS: usize> {
    pub ecu_outputs: core_plant::EcuOutputFrame<CYL, MAX_EVENTS>,
    pub diagnostics: X86PlantBridgeDiagnostics,
}

/// Compact plant-output snapshot published by the x86 board loop.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct X86PlantOutputSummary {
    pub timestamp_us: u32,
    pub rpm: u16,
    pub crank_angle_deg10: u16,
    pub map_kpa10: u16,
    pub lambda_x1000: u16,
    pub combustion_count: u16,
    pub combustion_torque_nm_x100: i32,
    pub ignored_injection_count: u16,
    pub ignored_spark_count: u16,
    pub diagnostic_count: u16,
    pub diagnostic_overflow_count: u16,
}

impl X86PlantOutputSummary {
    pub fn from_plant_output<const CYL: usize, const MAX_EDGES: usize, const MAX_EVENTS: usize>(
        output: &core_plant::PlantStepOutput<CYL, MAX_EDGES, MAX_EVENTS>,
    ) -> Self {
        let combustion_count = output
            .combustion
            .cylinders
            .iter()
            .filter(|cylinder| cylinder.torque_nm_x100.0 != 0)
            .count() as u16;
        Self {
            timestamp_us: output.sensors.timestamp_us.0,
            rpm: output.sensors.rpm.0.min(u16::MAX as u32) as u16,
            crank_angle_deg10: output.sensors.crank_angle_deg10.0,
            map_kpa10: output.sensors.map_kpa10.0,
            lambda_x1000: output.sensors.lambda_x1000,
            combustion_count,
            combustion_torque_nm_x100: output.combustion.total_torque_nm_x100.0,
            ignored_injection_count: output
                .consumed_events
                .ignored_injection_count
                .min(u16::MAX as usize) as u16,
            ignored_spark_count: output
                .consumed_events
                .ignored_spark_count
                .min(u16::MAX as usize) as u16,
            diagnostic_count: output.diagnostics.events.len().min(u16::MAX as usize) as u16,
            diagnostic_overflow_count: output.diagnostics.overflow_count,
        }
    }
}

/// ECU state snapshot used by the x86 closed-loop harness.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct X86EcuSummary {
    pub synced: bool,
    pub rpm: u16,
    pub map_kpa10: u16,
    pub tooth_count: u8,
    pub battery_mv: u16,
    pub final_pw_us: u32,
    pub commanded_advance_x10: i16,
}

/// Cut modes used by the reusable x86 closed-loop runner.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum X86ClosedLoopCut {
    None,
    Injection,
    Ignition,
}

/// Reusable x86 closed-loop run result.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct X86ClosedLoopRunResult<const EDGE_CAP: usize, const EVENT_CAP: usize> {
    pub ecu: X86EcuSummary,
    pub final_plant_output: core_plant::PlantStepOutput<4, EDGE_CAP, EVENT_CAP>,
    pub last_published_plant_output: Option<X86PlantOutputSummary>,
    pub output_history: Vec<OutputTransition>,
    pub trace: Vec<SimBoardTraceRecord>,
    pub report: SimBoardLoopReport,
    pub bridge_diagnostics: X86PlantBridgeDiagnostics,
    pub injector_outputs_seen: u32,
    pub ignition_outputs_seen: u32,
    pub combustion_seen: u32,
    pub ignored_injection_events: u32,
    pub ignored_spark_events: u32,
}

/// Deterministic host time source backed by shared interior mutability.
#[derive(Debug, Clone)]
pub struct X86HostTime {
    micros: Rc<Cell<u32>>,
}

impl X86HostTime {
    pub fn new() -> Self {
        Self {
            micros: Rc::new(Cell::new(0)),
        }
    }

    pub fn set_micros(&self, micros: u32) {
        self.micros.set(micros);
    }

    pub fn micros(&self) -> u32 {
        self.micros.get()
    }
}

impl Default for X86HostTime {
    fn default() -> Self {
        Self::new()
    }
}

impl TimeSource for X86HostTime {
    fn micros(&self) -> u32 {
        self.micros.get()
    }
}

/// Host output pin with stable identity and observable level.
#[derive(Debug)]
struct X86BoardPin {
    channel: ChannelId,
    kind: OutputTransitionKind,
    level: Cell<OutputLevel>,
}

impl X86BoardPin {
    const fn new(channel: ChannelId, kind: OutputTransitionKind) -> Self {
        Self {
            channel,
            kind,
            level: Cell::new(OutputLevel::Low),
        }
    }

    fn level(&self) -> OutputLevel {
        self.level.get()
    }
}

impl OutputPin for X86BoardPin {
    fn set_high(&mut self) {
        self.level.set(OutputLevel::High);
    }

    fn set_low(&mut self) {
        self.level.set(OutputLevel::Low);
    }
}

/// x86 board adapter for the live ECU runtime.
pub struct X86SimBoard<const OUT: usize, const TRACE: usize> {
    pub config: SimBoardLoopConfig,
    pub report: SimBoardLoopReport,
    time: X86HostTime,
    app: EcuApp<X86HostTime>,
    plant_bridge_config: X86PlantBridgeConfig,
    bridge_injector_high_at_us: [Option<u32>; 8],
    bridge_ignition_high_at_us: [Option<u32>; 8],
    last_published_plant_output: Option<X86PlantOutputSummary>,
    driver_input: SimDriverInput,
    environment: SimEnvironment,
    pins: [X86BoardPin; 8],
    pending_outputs: FixedOutputQueue<OUT>,
    trace: FixedTraceBuffer<TRACE>,
}

impl<const OUT: usize, const TRACE: usize> X86SimBoard<OUT, TRACE> {
    pub fn new() -> Self {
        Self::with_config(SimBoardLoopConfig::default())
    }

    pub fn with_config(config: SimBoardLoopConfig) -> Self {
        let time = X86HostTime::new();
        let app = EcuApp::new(time.clone());
        Self {
            config,
            report: SimBoardLoopReport::new(),
            time,
            app,
            plant_bridge_config: X86PlantBridgeConfig::default(),
            bridge_injector_high_at_us: [None; 8],
            bridge_ignition_high_at_us: [None; 8],
            last_published_plant_output: None,
            driver_input: SimDriverInput::idle(),
            environment: SimEnvironment::standard(),
            pins: Self::default_pins(),
            pending_outputs: FixedOutputQueue::new(),
            trace: FixedTraceBuffer::new(),
        }
    }

    pub fn ecu_state(&self) -> &ecu_core::EcuState {
        self.app.state()
    }

    pub fn ecu_state_mut(&mut self) -> &mut ecu_core::EcuState {
        self.app.state_mut()
    }

    pub fn plant_bridge_config(&self) -> X86PlantBridgeConfig {
        self.plant_bridge_config
    }

    pub fn set_plant_bridge_config(&mut self, config: X86PlantBridgeConfig) {
        self.plant_bridge_config = config;
    }

    pub fn set_driver_input(&mut self, input: SimDriverInput) {
        self.driver_input = input;
    }

    pub fn set_environment(&mut self, environment: SimEnvironment) {
        self.environment = environment;
    }

    pub fn pending_output_count(&self) -> usize {
        self.pending_outputs.len()
    }

    pub fn trace_count(&self) -> usize {
        self.trace.len()
    }

    pub fn trace_overflow_count(&self) -> u32 {
        self.trace.overflow_count()
    }

    pub fn output_overflow_count(&self) -> u32 {
        self.pending_outputs.overflow_count()
    }

    pub fn trace_records(&self) -> impl Iterator<Item = SimBoardTraceRecord> + '_ {
        self.trace.iter()
    }

    pub fn last_published_plant_output(&self) -> Option<X86PlantOutputSummary> {
        self.last_published_plant_output
    }

    pub fn bridge_open_high_diagnostics(&self) -> X86PlantBridgeDiagnostics {
        let open_high_count = self
            .bridge_injector_high_at_us
            .iter()
            .chain(self.bridge_ignition_high_at_us.iter())
            .filter(|started_at| started_at.is_some())
            .count()
            .min(u32::MAX as usize) as u32;

        X86PlantBridgeDiagnostics {
            open_high_count,
            ..X86PlantBridgeDiagnostics::empty()
        }
    }

    pub fn ecu_summary(&self) -> X86EcuSummary {
        let state = self.ecu_state();
        X86EcuSummary {
            synced: state.synced(),
            rpm: state.rpm(),
            map_kpa10: state.map_kpa_x10(),
            tooth_count: state.tooth_count(),
            battery_mv: state.battery_voltage_mv(),
            final_pw_us: state.final_pw_output().0,
            commanded_advance_x10: state.commanded_advance_x10_output(),
        }
    }

    pub fn capture_outputs(
        &mut self,
        out: &mut FixedOutputQueue<OUT>,
    ) -> Result<(), X86BoardError> {
        out.clear();
        for transition in self.pending_outputs.iter() {
            out.push_sorted(transition)
                .map_err(X86BoardError::OutputOverflow)?;
        }
        self.pending_outputs.clear();
        Ok(())
    }

    /// Translate captured output transitions into a plant frame.
    ///
    /// `crank_angle_deg10` and `spark_angle_deg10` are adapter approximations
    /// for the current root-output layout; callers should pass the board-side
    /// values they want to test, not infer them inside the test harness.
    pub fn build_core_output_frame<const CYL: usize, const MAX_EVENTS: usize, I>(
        &mut self,
        transitions: I,
        crank_angle_deg10: u16,
        spark_angle_deg10: u16,
    ) -> X86PlantBridgeFrame<CYL, MAX_EVENTS>
    where
        I: IntoIterator<Item = OutputTransition>,
    {
        let mut frame = X86PlantBridgeFrame {
            ecu_outputs: core_plant::EcuOutputFrame::<CYL, MAX_EVENTS>::empty(),
            diagnostics: X86PlantBridgeDiagnostics::empty(),
        };

        for transition in transitions {
            let channel = transition.channel.get() as usize;
            if channel >= self.plant_bridge_config.injector_cylinders.len() {
                if matches!(
                    transition.kind,
                    OutputTransitionKind::Injector | OutputTransitionKind::Ignition
                ) {
                    frame.diagnostics.out_of_range_channel_count = frame
                        .diagnostics
                        .out_of_range_channel_count
                        .saturating_add(1);
                }
                continue;
            }

            let (mapping, high_at_us, is_injector) = match transition.kind {
                OutputTransitionKind::Injector => (
                    self.plant_bridge_config.injector_cylinders[channel],
                    &mut self.bridge_injector_high_at_us[channel],
                    true,
                ),
                OutputTransitionKind::Ignition => (
                    self.plant_bridge_config.ignition_cylinders[channel],
                    &mut self.bridge_ignition_high_at_us[channel],
                    false,
                ),
                OutputTransitionKind::Idle | OutputTransitionKind::Fan => continue,
            };

            let Some(cylinder) = mapping else {
                frame.diagnostics.unmapped_channel_count =
                    frame.diagnostics.unmapped_channel_count.saturating_add(1);
                continue;
            };

            match transition.level {
                OutputLevel::High => {
                    if high_at_us.is_some() {
                        frame.diagnostics.duplicate_high_count =
                            frame.diagnostics.duplicate_high_count.saturating_add(1);
                    } else {
                        *high_at_us = Some(transition.at_us.get());
                    }
                }
                OutputLevel::Low => {
                    let Some(start_us) = high_at_us.take() else {
                        frame.diagnostics.orphan_low_count =
                            frame.diagnostics.orphan_low_count.saturating_add(1);
                        continue;
                    };

                    let end_us = transition.at_us.get();
                    if end_us < start_us {
                        frame.diagnostics.out_of_order_transition_count = frame
                            .diagnostics
                            .out_of_order_transition_count
                            .saturating_add(1);
                        continue;
                    }

                    let width_us = end_us - start_us;
                    if is_injector {
                        if width_us < self.plant_bridge_config.approximate_min_pulse_width_us {
                            frame.diagnostics.short_pulse_width_count =
                                frame.diagnostics.short_pulse_width_count.saturating_add(1);
                        }
                        let pulse_width_us =
                            width_us.max(self.plant_bridge_config.approximate_min_pulse_width_us);
                        let flow_ug_per_us = if pulse_width_us == 0 {
                            0
                        } else {
                            (self
                                .plant_bridge_config
                                .approximate_target_fuel_ug_per_injection
                                / pulse_width_us)
                                .max(1)
                        };
                        let command = core_plant::InjectionCommand {
                            cylinder,
                            mode: core_plant::InjectionTimingMode::StartOfInjection,
                            angle_deg10: core_plant::CrankDeg10(crank_angle_deg10),
                            pulse_width_us: core_plant::Micros(pulse_width_us),
                            injector_flow_ug_per_us: core_plant::MicrogramsPerMicros(
                                flow_ug_per_us,
                            ),
                            deadtime_us: core_plant::Micros(0),
                        };
                        if frame.ecu_outputs.injection_events.push(command).is_err() {
                            frame.diagnostics.capacity_overflow_count =
                                frame.diagnostics.capacity_overflow_count.saturating_add(1);
                        }
                    } else {
                        if width_us < self.plant_bridge_config.approximate_min_spark_dwell_us {
                            frame.diagnostics.short_dwell_count =
                                frame.diagnostics.short_dwell_count.saturating_add(1);
                        }
                        let dwell_us =
                            width_us.max(self.plant_bridge_config.approximate_min_spark_dwell_us);
                        let command = core_plant::SparkCommand {
                            cylinder,
                            spark_angle_deg10: core_plant::CrankDeg10(spark_angle_deg10),
                            dwell_us: core_plant::Micros(dwell_us),
                            coil_energy_x1000: self.plant_bridge_config.coil_energy_x1000,
                        };
                        if frame.ecu_outputs.spark_events.push(command).is_err() {
                            frame.diagnostics.capacity_overflow_count =
                                frame.diagnostics.capacity_overflow_count.saturating_add(1);
                        }
                    }
                }
            }
        }

        frame
    }

    pub fn drive_until(&mut self, now_us: u32) -> Result<(), X86BoardError> {
        if !self.config.allow_time_regression && now_us < self.report.last_now_us {
            self.report.time_regressions = self.report.time_regressions.saturating_add(1);
            return Err(X86BoardError::TimeWentBackwards {
                previous_us: self.report.last_now_us,
                now_us,
            });
        }

        self.report.tick_count = self.report.tick_count.saturating_add(1);
        self.report.last_now_us = now_us;
        self.time.set_micros(now_us);
        self.write_trace(SimBoardTraceRecord::new(
            now_us,
            SimBoardTraceKind::Tick,
            0,
            now_us as i32,
            self.report.tick_count as i32,
        ))?;

        let old_levels = self.current_levels();
        let mut outputs = Self::make_output_slice(&mut self.pins);
        self.app.drive_outputs(now_us, &mut outputs);

        for (idx, old) in old_levels.iter().copied().enumerate() {
            let (channel, kind, new) = {
                let pin = &self.pins[idx];
                (pin.channel, pin.kind, pin.level())
            };
            if old != new {
                let transition = OutputTransition {
                    at_us: Micros::new(now_us),
                    kind,
                    channel,
                    level: new,
                };
                self.report.output_transitions_queued =
                    self.report.output_transitions_queued.saturating_add(1);
                self.pending_outputs
                    .push_sorted(transition)
                    .map_err(X86BoardError::OutputOverflow)?;
                self.write_trace(SimBoardTraceRecord::new(
                    now_us,
                    SimBoardTraceKind::OutputTransition,
                    channel.get(),
                    if new == OutputLevel::High { 1 } else { 0 },
                    transition_kind_rank(kind) as i32,
                ))?;
            }
        }

        Ok(())
    }

    fn set_time_us(&self, now_us: u32) {
        self.time.set_micros(now_us);
    }

    fn current_levels(&self) -> [OutputLevel; 8] {
        [
            self.pins[0].level(),
            self.pins[1].level(),
            self.pins[2].level(),
            self.pins[3].level(),
            self.pins[4].level(),
            self.pins[5].level(),
            self.pins[6].level(),
            self.pins[7].level(),
        ]
    }

    fn default_pins() -> [X86BoardPin; 8] {
        [
            X86BoardPin::new(ChannelId::new(0), OutputTransitionKind::Injector),
            X86BoardPin::new(ChannelId::new(1), OutputTransitionKind::Injector),
            X86BoardPin::new(ChannelId::new(2), OutputTransitionKind::Ignition),
            X86BoardPin::new(ChannelId::new(3), OutputTransitionKind::Ignition),
            X86BoardPin::new(ChannelId::new(4), OutputTransitionKind::Idle),
            X86BoardPin::new(ChannelId::new(5), OutputTransitionKind::Idle),
            X86BoardPin::new(ChannelId::new(6), OutputTransitionKind::Fan),
            X86BoardPin::new(ChannelId::new(7), OutputTransitionKind::Fan),
        ]
    }

    fn make_output_slice(pins: &mut [X86BoardPin; 8]) -> [&mut dyn OutputPin; 8] {
        let ptr = pins.as_mut_ptr();
        // SAFETY: Each pointer targets a distinct element in the fixed array.
        unsafe {
            [
                &mut *ptr,
                &mut *ptr.add(1),
                &mut *ptr.add(2),
                &mut *ptr.add(3),
                &mut *ptr.add(4),
                &mut *ptr.add(5),
                &mut *ptr.add(6),
                &mut *ptr.add(7),
            ]
        }
    }
}

impl<const OUT: usize, const TRACE: usize> Default for X86SimBoard<OUT, TRACE> {
    fn default() -> Self {
        Self::new()
    }
}

impl<const OUT: usize, const TRACE: usize> SimBoard for X86SimBoard<OUT, TRACE> {
    type Error = X86BoardError;
    type OutputBuffer = FixedOutputQueue<OUT>;
    type PlantOutputs = X86PlantOutputSummary;

    fn now_micros(&self) -> u32 {
        self.time.micros()
    }

    fn read_driver_input(&mut self) -> Result<SimDriverInput, Self::Error> {
        Ok(self.driver_input)
    }

    fn read_environment(&mut self) -> Result<SimEnvironment, Self::Error> {
        Ok(self.environment)
    }

    fn feed_trigger_edges(&mut self, edges: &[SimTriggerEdge]) -> Result<(), Self::Error> {
        for edge in edges {
            self.set_time_us(edge.timestamp_us);
            self.report.trigger_edges_queued = self.report.trigger_edges_queued.saturating_add(1);
            self.write_trace(SimBoardTraceRecord::new(
                edge.timestamp_us,
                SimBoardTraceKind::TriggerEdge,
                trigger_line_rank(edge.line),
                trigger_edge_value(edge.polarity) as i32,
                edge.angle_deg10 as i32,
            ))?;

            match edge.line {
                SimTriggerLine::Crank => self.app.on_timestamp(edge.timestamp_us),
                SimTriggerLine::Cam => self.app.on_cam_edge(),
            }
        }
        Ok(())
    }

    fn feed_sensor_frame(&mut self, sensors: SimSensorFrame) -> Result<(), Self::Error> {
        self.set_time_us(sensors.timestamp_us);
        self.write_trace(SimBoardTraceRecord::new(
            sensors.timestamp_us,
            SimBoardTraceKind::SensorFrame,
            0,
            sensors.rpm.get() as i32,
            sensors.map_kpa10 as i32,
        ))?;

        let state = self.app.state_mut();
        state.set_rpm(sensors.rpm.get());
        state.set_map_kpa_x10(sensors.map_kpa10);
        state.set_clt_x10(sensors.clt_c10);
        state.set_iat_x10(sensors.iat_c10);
        state.set_tps_percent((sensors.tps_x1000.min(1000) / 10) as u8);
        state.set_battery_voltage_mv(sensors.battery_mv);
        Ok(())
    }

    fn collect_ecu_outputs(&mut self, out: &mut Self::OutputBuffer) -> Result<(), Self::Error> {
        self.capture_outputs(out)
    }

    fn publish_plant_outputs(&mut self, outputs: Self::PlantOutputs) -> Result<(), Self::Error> {
        self.last_published_plant_output = Some(outputs);
        self.write_trace(SimBoardTraceRecord::new(
            outputs.timestamp_us,
            SimBoardTraceKind::PlantOutput,
            outputs.combustion_count.min(u8::MAX as u16) as u8,
            outputs.rpm as i32,
            outputs.combustion_torque_nm_x100,
        ))?;
        Ok(())
    }

    fn write_trace(&mut self, record: SimBoardTraceRecord) -> Result<(), Self::Error> {
        if !self.config.trace_enabled {
            return Ok(());
        }

        self.trace.push(record).map_err(|overflow| {
            self.report.trace_overflow_count = self.trace.overflow_count();
            X86BoardError::TraceOverflow(overflow)
        })?;
        self.report.trace_records_queued = self.report.trace_records_queued.saturating_add(1);
        Ok(())
    }
}

fn trigger_line_rank(line: SimTriggerLine) -> u8 {
    match line {
        SimTriggerLine::Crank => 0,
        SimTriggerLine::Cam => 1,
    }
}

fn trigger_edge_value(polarity: SimEdgePolarity) -> u8 {
    match polarity {
        SimEdgePolarity::Rising => 1,
        SimEdgePolarity::Falling => 0,
    }
}

fn transition_kind_rank(kind: OutputTransitionKind) -> u8 {
    match kind {
        OutputTransitionKind::Injector => 0,
        OutputTransitionKind::Ignition => 1,
        OutputTransitionKind::Idle => 2,
        OutputTransitionKind::Fan => 3,
    }
}

fn accumulate_bridge_diagnostics(
    total: &mut X86PlantBridgeDiagnostics,
    step: X86PlantBridgeDiagnostics,
) {
    total.duplicate_high_count = total
        .duplicate_high_count
        .saturating_add(step.duplicate_high_count);
    total.orphan_low_count = total.orphan_low_count.saturating_add(step.orphan_low_count);
    total.open_high_count = total.open_high_count.saturating_add(step.open_high_count);
    total.unmapped_channel_count = total
        .unmapped_channel_count
        .saturating_add(step.unmapped_channel_count);
    total.out_of_range_channel_count = total
        .out_of_range_channel_count
        .saturating_add(step.out_of_range_channel_count);
    total.out_of_order_transition_count = total
        .out_of_order_transition_count
        .saturating_add(step.out_of_order_transition_count);
    total.short_pulse_width_count = total
        .short_pulse_width_count
        .saturating_add(step.short_pulse_width_count);
    total.short_dwell_count = total
        .short_dwell_count
        .saturating_add(step.short_dwell_count);
    total.capacity_overflow_count = total
        .capacity_overflow_count
        .saturating_add(step.capacity_overflow_count);
}

fn publish_plant_outputs_for_step<
    const OUT: usize,
    const TRACE: usize,
    const EDGE_CAP: usize,
    const EVENT_CAP: usize,
>(
    board: &mut X86SimBoard<OUT, TRACE>,
    plant_output: &core_plant::PlantStepOutput<4, EDGE_CAP, EVENT_CAP>,
) {
    let summary = X86PlantOutputSummary::from_plant_output(plant_output);
    board
        .publish_plant_outputs(summary)
        .expect("publish plant outputs");
}

fn merge_ecu_outputs<const CYL: usize, const EVENT_CAP: usize>(
    a: &core_plant::EcuOutputFrame<CYL, EVENT_CAP>,
    b: &core_plant::EcuOutputFrame<CYL, EVENT_CAP>,
) -> core_plant::EcuOutputFrame<CYL, EVENT_CAP> {
    let mut merged = core_plant::EcuOutputFrame::<CYL, EVENT_CAP>::empty();
    merged.fuel_cut = a.fuel_cut || b.fuel_cut;
    merged.spark_cut = a.spark_cut || b.spark_cut;
    merged.idle_command_x1000 = a.idle_command_x1000.max(b.idle_command_x1000);
    for command in a.injection_events.as_slice() {
        merged
            .injection_events
            .push(*command)
            .expect("merge injector events");
    }
    for command in b.injection_events.as_slice() {
        merged
            .injection_events
            .push(*command)
            .expect("merge injector events");
    }
    for command in a.spark_events.as_slice() {
        merged
            .spark_events
            .push(*command)
            .expect("merge spark events");
    }
    for command in b.spark_events.as_slice() {
        merged
            .spark_events
            .push(*command)
            .expect("merge spark events");
    }
    merged
}

fn has_fuel_and_spark<const CYL: usize, const EVENT_CAP: usize>(
    frame: &core_plant::EcuOutputFrame<CYL, EVENT_CAP>,
) -> bool {
    !frame.injection_events.as_slice().is_empty() && !frame.spark_events.as_slice().is_empty()
}

fn to_sim_trigger_edge(edge: core_plant::TriggerEdge) -> SimTriggerEdge {
    SimTriggerEdge {
        timestamp_us: edge.timestamp_us.0,
        line: match edge.channel {
            core_plant::TriggerChannel::Crank => SimTriggerLine::Crank,
            core_plant::TriggerChannel::Cam => SimTriggerLine::Cam,
        },
        polarity: match edge.edge {
            core_plant::EdgePolarity::Rising => SimEdgePolarity::Rising,
            core_plant::EdgePolarity::Falling => SimEdgePolarity::Falling,
        },
        angle_deg10: edge.crank_angle_deg10.0,
    }
}

fn to_sim_sensor_frame(
    sensors: core_plant::SensorSnapshot,
    driver: SimDriverInput,
) -> SimSensorFrame {
    SimSensorFrame {
        timestamp_us: sensors.timestamp_us.0,
        rpm: ecu_domain::Rpm::new(sensors.rpm.0.min(u16::MAX as u32) as u16),
        crank_angle_deg10: sensors.crank_angle_deg10.0,
        map_kpa10: sensors.map_kpa10.0,
        tps_x1000: driver.throttle_x1000,
        clt_c10: sensors.clt_c10.0,
        iat_c10: sensors.iat_c10.0,
        lambda_x1000: sensors.lambda_x1000,
        battery_mv: sensors.battery_mv.0,
        knock_intensity_x100: sensors.knock_intensity_x100,
    }
}

fn startup_sensor_frame(
    timestamp_us: u32,
    driver: SimDriverInput,
    environment: SimEnvironment,
) -> SimSensorFrame {
    SimSensorFrame {
        timestamp_us,
        rpm: ecu_domain::Rpm::new(6_000),
        crank_angle_deg10: 0,
        map_kpa10: 600,
        tps_x1000: driver.throttle_x1000,
        clt_c10: 800,
        iat_c10: 300,
        lambda_x1000: 1_000,
        battery_mv: environment.battery_mv,
        knock_intensity_x100: 0,
    }
}

fn apply_startup_aid<const EVENT_CAP: usize>(
    step_idx: usize,
    board_synced: bool,
    input: &mut core_plant::PlantStepInput<4, EVENT_CAP>,
) {
    input.driver.starter_enabled = step_idx < 8 || !board_synced;
}

impl X86ClosedLoopCut {
    fn apply_to<const CYL: usize, const EVENT_CAP: usize>(
        self,
        ecu_outputs: &mut core_plant::EcuOutputFrame<CYL, EVENT_CAP>,
    ) {
        match self {
            X86ClosedLoopCut::None => {}
            X86ClosedLoopCut::Injection => ecu_outputs.fuel_cut = true,
            X86ClosedLoopCut::Ignition => ecu_outputs.spark_cut = true,
        }
    }
}

/// Reusable x86 closed-loop choreography.
///
/// The generic board-state and trace bookkeeping stays in `SimBoardLoop`; this
/// helper owns the x86-specific plant reset, sensor/edge translation, ECU
/// output collection, and feedback-bridge accumulation used by the x86 board
/// tests.
pub fn run_x86_closed_loop_engine<
    const OUT: usize,
    const TRACE: usize,
    const EDGE_CAP: usize,
    const EVENT_CAP: usize,
>(
    cut: X86ClosedLoopCut,
) -> X86ClosedLoopRunResult<EDGE_CAP, EVENT_CAP> {
    const BOARD_TICK_US: u32 = 10_000;
    const BOARD_POLL_US: u32 = 50;
    const RUN_END_US: u32 = 160_000;

    let mut board: X86SimBoard<OUT, TRACE> = X86SimBoard::new();
    let driver = SimDriverInput {
        throttle_x1000: 800,
        requested_rpm: ecu_domain::Rpm::new(6_000),
        load_torque_nm_x100: crate::embedded_loop::TorqueNmX100::new(260),
        mode: crate::embedded_loop::SimControlMode::ClosedLoopEngine,
    };
    let environment = SimEnvironment {
        ambient_pressure_pa: 101_325,
        ambient_temp_k_x10: 2_931,
        battery_mv: 12_500,
    };
    board.set_driver_input(driver);
    board.set_environment(environment);

    let driver = board.read_driver_input().expect("driver input");
    let environment = board.read_environment().expect("environment");

    board
        .feed_sensor_frame(startup_sensor_frame(0, driver, environment))
        .expect("initial sensor frame");

    let mut plant = core_plant::Plant::<4, EDGE_CAP, EVENT_CAP>::new({
        let mut config = core_plant::PlantConfig::<4>::default_four();
        config.trigger = core_plant::TriggerConfig {
            crank_teeth: 60,
            missing_teeth: 2,
            cam_pulses: 1,
        };
        config
    });
    plant.reset(core_plant::InitialPlantState {
        timestamp_us: core_plant::Micros(0),
        rpm: core_plant::Rpm(6_000),
        crank_angle_deg10: core_plant::CrankDeg10(0),
        ..core_plant::InitialPlantState::new()
    });

    let mut plant_input =
        core_plant::PlantStepInput::<4, EVENT_CAP>::idle(core_plant::Micros(BOARD_TICK_US));
    plant_input.driver.throttle_x1000 = driver.throttle_x1000;
    plant_input.driver.load_torque_nm_x100 = core_plant::TorqueNmX100(driver.load_torque_nm_x100.0);
    plant_input.environment = core_plant::EnvironmentInput {
        ambient_c10: core_plant::Celsius10(293),
        coolant_c10: core_plant::Celsius10(800),
        battery_mv: core_plant::Millivolts(environment.battery_mv),
    };

    let mut plant_output = core_plant::PlantStepOutput::<4, EDGE_CAP, EVENT_CAP>::empty();
    let mut bridge_diagnostics = X86PlantBridgeDiagnostics::empty();
    let mut output_history = Vec::new();
    let mut captured_outputs = FixedOutputQueue::<OUT>::new();
    let mut combustion_seen = 0u32;
    let mut ignored_injection_events = 0u32;
    let mut ignored_spark_events = 0u32;
    let mut board_synced = false;
    let mut pending_ecu_outputs = core_plant::EcuOutputFrame::<4, EVENT_CAP>::empty();
    let mut pending_ecu_outputs_ready = false;
    let mut drive_time = board.now_micros();

    for step_idx in 0..(RUN_END_US / BOARD_TICK_US) as usize {
        apply_startup_aid(step_idx, board_synced, &mut plant_input);
        plant_input.ecu_outputs = if pending_ecu_outputs_ready {
            pending_ecu_outputs_ready = false;
            let ready_outputs = pending_ecu_outputs;
            pending_ecu_outputs = core_plant::EcuOutputFrame::<4, EVENT_CAP>::empty();
            ready_outputs
        } else {
            core_plant::EcuOutputFrame::<4, EVENT_CAP>::empty()
        };
        cut.apply_to(&mut plant_input.ecu_outputs);
        plant
            .step(&plant_input, &mut plant_output)
            .expect("core plant step");

        let step_end_us = plant_output.sensors.timestamp_us.0;
        let trigger_edges: Vec<_> = plant_output
            .trigger_edges
            .as_slice()
            .iter()
            .copied()
            .map(to_sim_trigger_edge)
            .collect();
        if !trigger_edges.is_empty() {
            board
                .feed_trigger_edges(&trigger_edges)
                .expect("feed plant trigger edges");
            drive_time = trigger_edges
                .iter()
                .map(|edge| edge.timestamp_us)
                .max()
                .unwrap_or(drive_time);
        }

        let mut transitions = Vec::new();
        while drive_time < step_end_us {
            drive_time = drive_time.saturating_add(BOARD_POLL_US).min(step_end_us);
            board.drive_until(drive_time).expect("drive board");
            board
                .collect_ecu_outputs(&mut captured_outputs)
                .expect("capture ECU outputs");
            transitions.extend(captured_outputs.iter());
            captured_outputs.clear();
        }

        let sensor_frame = if step_end_us <= 73_000 {
            startup_sensor_frame(step_end_us, driver, environment)
        } else {
            to_sim_sensor_frame(plant_output.sensors, driver)
        };
        board
            .feed_sensor_frame(sensor_frame)
            .expect("feed plant sensors");

        let post_flush_end = step_end_us.saturating_add(BOARD_POLL_US);
        while drive_time < post_flush_end {
            drive_time = drive_time.saturating_add(BOARD_POLL_US).min(post_flush_end);
            board.drive_until(drive_time).expect("flush board");
            board
                .collect_ecu_outputs(&mut captured_outputs)
                .expect("capture ECU outputs");
            transitions.extend(captured_outputs.iter());
            captured_outputs.clear();
        }
        output_history.extend(transitions.iter().copied());

        let spark_angle_deg10 = board.ecu_state().commanded_advance_x10_output().max(180) as u16;
        let bridge_frame = board.build_core_output_frame(
            transitions.iter().copied(),
            plant_output.sensors.crank_angle_deg10.0,
            spark_angle_deg10,
        );
        accumulate_bridge_diagnostics(&mut bridge_diagnostics, bridge_frame.diagnostics);
        pending_ecu_outputs = merge_ecu_outputs(&pending_ecu_outputs, &bridge_frame.ecu_outputs);
        cut.apply_to(&mut pending_ecu_outputs);
        if has_fuel_and_spark(&pending_ecu_outputs) {
            pending_ecu_outputs_ready = true;
        }
        captured_outputs.clear();
        board_synced = board.ecu_state().synced();

        combustion_seen = combustion_seen.saturating_add(
            plant_output
                .combustion
                .cylinders
                .iter()
                .filter(|event| event.torque_nm_x100.0 > 0)
                .count() as u32,
        );
        ignored_injection_events = ignored_injection_events
            .saturating_add(plant_output.consumed_events.ignored_injection_count as u32);
        ignored_spark_events = ignored_spark_events
            .saturating_add(plant_output.consumed_events.ignored_spark_count as u32);
        publish_plant_outputs_for_step(&mut board, &plant_output);
    }

    let ecu = board.ecu_summary();
    let trace = board.trace_records().collect::<Vec<_>>();
    let injector_outputs_seen = output_history
        .iter()
        .filter(|transition| matches!(transition.kind, OutputTransitionKind::Injector))
        .count() as u32;
    let ignition_outputs_seen = output_history
        .iter()
        .filter(|transition| matches!(transition.kind, OutputTransitionKind::Ignition))
        .count() as u32;

    accumulate_bridge_diagnostics(
        &mut bridge_diagnostics,
        board.bridge_open_high_diagnostics(),
    );

    assert_eq!(board.report.output_overflow_count, 0);
    assert_eq!(board.report.trace_overflow_count, 0);
    assert!(
        bridge_diagnostics.is_clean(),
        "x86 plant bridge diagnostics should be clean: {:?}",
        bridge_diagnostics
    );

    X86ClosedLoopRunResult {
        ecu,
        final_plant_output: plant_output,
        last_published_plant_output: board.last_published_plant_output(),
        output_history,
        trace,
        report: board.report,
        bridge_diagnostics,
        injector_outputs_seen,
        ignition_outputs_seen,
        combustion_seen,
        ignored_injection_events,
        ignored_spark_events,
    }
}
