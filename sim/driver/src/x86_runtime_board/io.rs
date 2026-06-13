use ecu_board_api::{
    legacy::{CalibrationPage, CalibrationStore},
    AuxCommandBatch, AuxOutputSink, EcuClock, EcuOutput, EdgeBatch, OutputLevel, OutputScheduler,
    OutputTransition, OutputTransitionBatch, SensorSnapshot, TelemetryFrame, TelemetrySink,
    TriggerEdge, TriggerEdgeSource,
};
use ecu_domain::{EnginePhase, Kpa10, Lambda100, Micros, Percent, Rpm, SyncState, Ticks};

use super::types::X86RuntimeBoardError;
use super::{
    X86_ACTIVE_OUTPUT_CAP, X86_AUX_COMMAND_CAP, X86_CAL_PAGE_COUNT, X86_CAL_PAGE_SIZE,
    X86_OUTPUT_CHANNEL_CAP, X86_OUTPUT_TRANSITION_CAP, X86_TRIGGER_EDGE_CAP,
};

pub(super) struct DeterministicClock {
    now_us: Micros,
}

impl DeterministicClock {
    pub(super) const fn new(now_us: Micros) -> Self {
        Self { now_us }
    }

    pub(super) fn set_now_us(&mut self, now_us: Micros) {
        self.now_us = now_us;
    }
}

impl Default for DeterministicClock {
    fn default() -> Self {
        Self::new(Micros::new(1_000))
    }
}

impl EcuClock for DeterministicClock {
    fn now_us(&self) -> Micros {
        self.now_us
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(super) struct FixedTriggerEdgeSource {
    edges: EdgeBatch<X86_TRIGGER_EDGE_CAP>,
    drained_total: usize,
}

impl FixedTriggerEdgeSource {
    pub(super) const fn new() -> Self {
        Self {
            edges: EdgeBatch::new(),
            drained_total: 0,
        }
    }

    pub(super) fn set_edges(&mut self, edges: &[TriggerEdge]) -> Result<(), X86RuntimeBoardError> {
        self.edges.clear();
        for edge in edges {
            self.edges
                .push(*edge)
                .map_err(|_| X86RuntimeBoardError::TriggerEdgeOverflow)?;
        }
        Ok(())
    }
}

impl Default for FixedTriggerEdgeSource {
    fn default() -> Self {
        Self::new()
    }
}

impl TriggerEdgeSource<X86_TRIGGER_EDGE_CAP> for FixedTriggerEdgeSource {
    type Error = X86RuntimeBoardError;

    fn drain_edges(
        &mut self,
        out: &mut EdgeBatch<X86_TRIGGER_EDGE_CAP>,
    ) -> Result<(), Self::Error> {
        out.clear();
        for edge in self.edges.iter() {
            out.push(*edge)
                .map_err(|_| X86RuntimeBoardError::TriggerEdgeOverflow)?;
            self.drained_total += 1;
        }
        self.edges.clear();
        Ok(())
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(super) struct FixedSensorSource {
    snapshot: SensorSnapshot,
    sample_count: usize,
}

impl FixedSensorSource {
    pub(super) const fn new(snapshot: SensorSnapshot) -> Self {
        Self {
            snapshot,
            sample_count: 0,
        }
    }

    pub(super) fn set_snapshot(&mut self, snapshot: SensorSnapshot) {
        self.snapshot = snapshot;
    }
}

impl Default for FixedSensorSource {
    fn default() -> Self {
        Self::new(SensorSnapshot::new(
            Micros::new(1_000),
            Rpm::new(3_000),
            Kpa10::new(450),
            Percent::new(12),
            840,
            550,
            12_500,
            Lambda100::new(100),
            SyncState::Locked { cam_ref: false },
            EnginePhase::Running,
        ))
    }
}

impl ecu_board_api::SensorSource for FixedSensorSource {
    type Error = X86RuntimeBoardError;

    fn sample(&mut self, now_us: Micros) -> Result<SensorSnapshot, Self::Error> {
        self.sample_count += 1;
        let mut snapshot = self.snapshot;
        snapshot.now_us = now_us;
        Ok(snapshot)
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(super) struct RecordingOutputScheduler {
    history: OutputTransitionBatch<X86_OUTPUT_TRANSITION_CAP>,
    safe_state_history: OutputTransitionBatch<X86_OUTPUT_TRANSITION_CAP>,
    active_since: [Option<Ticks>; X86_ACTIVE_OUTPUT_CAP],
    safe_state_at: Ticks,
    cancel_all_count: usize,
    force_safe_state_count: usize,
}

impl RecordingOutputScheduler {
    pub(super) const fn new() -> Self {
        Self {
            history: OutputTransitionBatch::new(),
            safe_state_history: OutputTransitionBatch::new(),
            active_since: [None; X86_ACTIVE_OUTPUT_CAP],
            safe_state_at: Ticks::new(0),
            cancel_all_count: 0,
            force_safe_state_count: 0,
        }
    }

    pub(super) fn history(&self) -> OutputTransitionBatch<X86_OUTPUT_TRANSITION_CAP> {
        self.history
    }

    pub(super) fn safe_state_history(&self) -> OutputTransitionBatch<X86_OUTPUT_TRANSITION_CAP> {
        self.safe_state_history
    }

    pub(super) fn mark_output_high(&mut self, output: EcuOutput, at: Ticks) {
        if let Some(index) = active_output_index(output) {
            self.active_since[index] = Some(at);
        }
    }

    pub(super) fn set_safe_state_at(&mut self, now_us: Micros) {
        self.safe_state_at = Ticks::new(now_us.get());
    }

    fn record_transition(&mut self, transition: OutputTransition) {
        if let Some(index) = active_output_index(transition.output) {
            match transition.level {
                OutputLevel::Low => self.active_since[index] = None,
                OutputLevel::High => self.active_since[index] = Some(transition.at),
            }
        }
    }
}

impl Default for RecordingOutputScheduler {
    fn default() -> Self {
        Self::new()
    }
}

impl OutputScheduler<X86_OUTPUT_TRANSITION_CAP> for RecordingOutputScheduler {
    type Error = X86RuntimeBoardError;

    fn schedule(
        &mut self,
        batch: &OutputTransitionBatch<X86_OUTPUT_TRANSITION_CAP>,
    ) -> Result<(), Self::Error> {
        for transition in batch.iter() {
            self.history
                .push(*transition)
                .map_err(|_| X86RuntimeBoardError::OutputOverflow)?;
            self.record_transition(*transition);
        }
        Ok(())
    }

    fn cancel_all(&mut self) -> Result<(), Self::Error> {
        self.cancel_all_count += 1;
        Ok(())
    }

    fn force_safe_state(&mut self) {
        self.force_safe_state_count += 1;
        for index in 0..X86_ACTIVE_OUTPUT_CAP {
            if self.active_since[index].is_some() {
                if let Some(output) = output_from_active_index(index) {
                    let transition =
                        OutputTransition::new(output, OutputLevel::Low, self.safe_state_at);
                    if self.safe_state_history.push(transition).is_ok() {
                        self.active_since[index] = None;
                    }
                }
            }
        }
    }
}

fn active_output_index(output: EcuOutput) -> Option<usize> {
    match output {
        EcuOutput::Injector(channel) => {
            let channel = usize::from(channel.get());
            (channel < X86_OUTPUT_CHANNEL_CAP).then_some(channel)
        }
        EcuOutput::Ignition(channel) => {
            let channel = usize::from(channel.get());
            (channel < X86_OUTPUT_CHANNEL_CAP).then_some(X86_OUTPUT_CHANNEL_CAP + channel)
        }
    }
}

fn output_from_active_index(index: usize) -> Option<EcuOutput> {
    if index < X86_OUTPUT_CHANNEL_CAP {
        Some(EcuOutput::Injector(ecu_domain::ChannelId::new(index as u8)))
    } else if index < X86_ACTIVE_OUTPUT_CAP {
        Some(EcuOutput::Ignition(ecu_domain::ChannelId::new(
            (index - X86_OUTPUT_CHANNEL_CAP) as u8,
        )))
    } else {
        None
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(super) struct RecordingAuxOutputSink {
    history: AuxCommandBatch<X86_AUX_COMMAND_CAP>,
    apply_count: usize,
}

impl RecordingAuxOutputSink {
    pub(super) const fn new() -> Self {
        Self {
            history: AuxCommandBatch::new(),
            apply_count: 0,
        }
    }

    pub(super) fn history(&self) -> AuxCommandBatch<X86_AUX_COMMAND_CAP> {
        self.history
    }
}

impl Default for RecordingAuxOutputSink {
    fn default() -> Self {
        Self::new()
    }
}

impl AuxOutputSink<X86_AUX_COMMAND_CAP> for RecordingAuxOutputSink {
    type Error = X86RuntimeBoardError;

    fn apply_aux(
        &mut self,
        batch: &AuxCommandBatch<X86_AUX_COMMAND_CAP>,
    ) -> Result<(), Self::Error> {
        for command in batch.iter() {
            self.history
                .push(*command)
                .map_err(|_| X86RuntimeBoardError::AuxOverflow)?;
        }
        self.apply_count += 1;
        Ok(())
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(super) struct RecordingTelemetrySink {
    last: Option<TelemetryFrame>,
    publish_count: usize,
}

impl RecordingTelemetrySink {
    pub(super) const fn new() -> Self {
        Self {
            last: None,
            publish_count: 0,
        }
    }

    pub(super) fn last(&self) -> Option<TelemetryFrame> {
        self.last
    }
}

impl Default for RecordingTelemetrySink {
    fn default() -> Self {
        Self::new()
    }
}

impl TelemetrySink for RecordingTelemetrySink {
    type Error = X86RuntimeBoardError;

    fn publish(&mut self, frame: &TelemetryFrame) -> Result<(), Self::Error> {
        self.publish_count += 1;
        self.last = Some(*frame);
        Ok(())
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(super) struct FixedCalibrationStore {
    pages: [[u8; X86_CAL_PAGE_SIZE]; X86_CAL_PAGE_COUNT],
    page_lens: [usize; X86_CAL_PAGE_COUNT],
}

impl FixedCalibrationStore {
    pub(super) const fn new() -> Self {
        Self {
            pages: [[0; X86_CAL_PAGE_SIZE]; X86_CAL_PAGE_COUNT],
            page_lens: [0; X86_CAL_PAGE_COUNT],
        }
    }

    fn page_index(page: CalibrationPage) -> usize {
        (page.get() as usize) % X86_CAL_PAGE_COUNT
    }
}

impl Default for FixedCalibrationStore {
    fn default() -> Self {
        Self::new()
    }
}

impl CalibrationStore for FixedCalibrationStore {
    type Error = X86RuntimeBoardError;

    fn read_page(&mut self, page: CalibrationPage, out: &mut [u8]) -> Result<usize, Self::Error> {
        let idx = Self::page_index(page);
        let len = self.page_lens[idx].min(out.len());
        out[..len].copy_from_slice(&self.pages[idx][..len]);
        Ok(len)
    }

    fn write_page(&mut self, page: CalibrationPage, bytes: &[u8]) -> Result<(), Self::Error> {
        if bytes.len() > X86_CAL_PAGE_SIZE {
            return Err(X86RuntimeBoardError::CalibrationOverflow);
        }
        let idx = Self::page_index(page);
        self.pages[idx][..bytes.len()].copy_from_slice(bytes);
        self.page_lens[idx] = bytes.len();
        Ok(())
    }
}
