use crate::kv::ab::StoreIntegrityStatus;
use crate::live_inputs::SplitSyncState;
use crate::outputs::ScheduledActionExecutor;
use crate::sensor_sample::map_speed_density_capture_sample;
#[cfg(feature = "transport-can")]
use ecu_board_api::BoardSensorValidityFlags;
use ecu_board_api::{
    BoardSensorSnapshot, BoardSensorSnapshotCapture, CaptureSample, CaptureSampleSource,
    CaptureSink, CommonActionTelemetry, CommonAfterstartTelemetry, CommonAfterstartWindowMode,
    CommonCamEdgeTelemetry, CommonControlReasonTelemetry, CommonControlTelemetry, CommonCutReason,
    CommonDecisionTelemetry, CommonDiagnosticsTelemetry, CommonEngineTelemetry,
    CommonEnrichmentTelemetry, CommonFaultTransitionAction, CommonFaultTransitionEventId,
    CommonFaultTransitionEventTelemetry, CommonFaultTransitionTelemetry, CommonFrontierFaultAction,
    CommonFrontierFaultEventId, CommonFrontierFaultTelemetry, CommonFrontierTelemetry,
    CommonFuelObservationTelemetry, CommonFuelStrategyMode, CommonHighRateLogTelemetry,
    CommonIgnitionLimitReason, CommonLambdaActivity, CommonLambdaCorrectionTelemetry,
    CommonLambdaDisableReason, CommonLambdaMode, CommonLambdaTelemetry, CommonLimpActionLevel,
    CommonLimpActionSource, CommonLimpActionTelemetry, CommonPendingInputTelemetry,
    CommonProtectionAction, CommonProtectionLevel, CommonProtectionPersistence,
    CommonProtectionSource, CommonProtectionTelemetry, CommonRuntimeFaultTelemetry,
    CommonSchedulerMode, CommonSchedulerOwnershipTelemetry, CommonSchedulerReservationTelemetry,
    CommonSchedulerStateSummaryTelemetry, CommonSchedulerWindowTelemetry,
    CommonShiftArmingTelemetry, CommonStartupTelemetry, CommonStartupWindowMode,
    CommonSyncTelemetryState, CommonTorqueLimitReason, CommonTorqueTelemetry,
    CommonTransientEnrichmentTelemetry, CommonTriggerEdgeTelemetry, CommonValidatedInputTelemetry,
    CommonWarmupTelemetry, CommonWarmupTemperatureMode, EngineTimeAuthorityTelemetry, Watchdog,
};
use ecu_calibration::{
    CalibrationPackageIdentity, PersistedCalibrationBlob, PersistedCalibrationStore,
};
#[cfg(feature = "transport-can")]
use ecu_domain::diag::{DiagCode, DiagSource};
use ecu_domain::diag::{DiagEvent, DiagLog};
#[cfg(feature = "transport-can")]
use ecu_domain::voltage::{BROWNOUT_CRITICAL_MV, OVERVOLTAGE_MV};
use ecu_domain::EngineTimeAuthority;
use ecu_domain::{ControlMode, Degrees10, Kpa10, Micros, Rpm};
use ecu_io::{
    OutputAssemblyCounters, OutputStage, SignalAssemblyCounters, SignalStage, StageOutcome, TraceId,
};
use ecu_runtime::{
    Action, ActionExecutor, AuthorityStepInputs, ControlInputs, DecoderObservation, EngineRuntime,
    RuntimeFuelStrategy, RuntimeSemanticCalibration, RuntimeSemanticState, RuntimeSnapshot,
    StepResult, TransportPublisher,
};
pub use ecu_scheduler::ScheduledTimingMetrics;
use ecu_ts::pages::DIAG_LOG_ENTRY_COUNT;

#[cfg(feature = "transport-can")]
const OIL_PRESSURE_MIN_KPA10: Kpa10 = Kpa10::new(1_000);
#[cfg(feature = "transport-can")]
const FUEL_PRESSURE_MIN_KPA10: Kpa10 = Kpa10::new(2_500);

/// Board-like event surface for the runtime-driven board adapters.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BoardEvent {
    TriggerEdge {
        at_us: Micros,
        rpm: Rpm,
        angle_x10: Degrees10,
        authority: EngineTimeAuthority,
        synced: bool,
    },
    CamEdge {
        at_us: Micros,
        cam_seen: bool,
    },
    /// Logical board snapshot capture projected to the runtime's MAP load path.
    SensorSnapshotCapture {
        capture: BoardSensorSnapshotCapture,
    },
    PressureSnapshot {
        now_us: Micros,
        oil_pressure_kpa10: Kpa10,
        fuel_pressure_kpa10: Kpa10,
        oil_valid: bool,
        fuel_valid: bool,
    },
    Tick {
        now_us: Micros,
        control: ControlInputs,
    },
}

#[cfg(feature = "transport-can")]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
struct SharedLiveDiagState {
    low_voltage_active: bool,
    overvoltage_active: bool,
    knock_active: bool,
    oil_pressure_low_active: bool,
    fuel_pressure_low_active: bool,
    lambda_invalid_active: bool,
    map_range: SharedDiagLatchState,
    tps_range: SharedDiagLatchState,
}

#[cfg(feature = "transport-can")]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
struct SharedDiagLatchState {
    active: bool,
    start_us: u32,
    in_range_since_us: u32,
    total_us: u32,
}

#[cfg(feature = "transport-can")]
impl SharedDiagLatchState {
    const fn is_active(self) -> bool {
        self.active
    }

    fn latch(&mut self, since: Micros) {
        self.active = true;
        self.start_us = since.get();
        self.in_range_since_us = 0;
    }

    fn clear(&mut self, _at: Micros) {
        self.active = false;
    }
}

#[cfg(feature = "transport-can")]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct SharedSensorLimits {
    map_min_kpa_x10: u16,
    map_max_kpa_x10: u16,
    tps_min_percent: u8,
    tps_max_percent: u8,
    clear_time_s: u16,
}

#[cfg(feature = "transport-can")]
impl Default for SharedSensorLimits {
    fn default() -> Self {
        Self {
            map_min_kpa_x10: 100,
            map_max_kpa_x10: 3000,
            tps_min_percent: 0,
            tps_max_percent: 100,
            clear_time_s: 3,
        }
    }
}

#[cfg(feature = "transport-can")]
fn shared_live_diag_event(
    code: DiagCode,
    timestamp: Micros,
    source: DiagSource,
    context: Option<u32>,
) -> DiagEvent {
    DiagEvent {
        code,
        timestamp,
        source,
        context,
        start_us: timestamp.get(),
        end_us: 0,
    }
}

#[cfg(feature = "transport-can")]
fn shared_live_knock_threshold_x100(strategy: &RuntimeFuelStrategy) -> Option<u16> {
    match strategy {
        RuntimeFuelStrategy::DirectPulseWidthTable(_) => None,
        RuntimeFuelStrategy::SpeedDensityVe { calibration, .. }
        | RuntimeFuelStrategy::AlphaN { calibration, .. }
        | RuntimeFuelStrategy::Maf { calibration, .. } => Some(calibration.knock_threshold_x100),
    }
}

impl BoardEvent {
    pub const fn observability_kind(self) -> CommonObservabilityRecordKind {
        match self {
            BoardEvent::TriggerEdge { .. } => CommonObservabilityRecordKind::TriggerEdge,
            BoardEvent::CamEdge { .. } => CommonObservabilityRecordKind::CamEdge,
            BoardEvent::SensorSnapshotCapture { .. } => {
                CommonObservabilityRecordKind::SensorSnapshotCapture
            }
            BoardEvent::PressureSnapshot { .. } => {
                CommonObservabilityRecordKind::SensorSnapshotCapture
            }
            BoardEvent::Tick { .. } => CommonObservabilityRecordKind::Tick,
        }
    }
}

/// Adapter error split by IO capability.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BoardAdapterError<S, C, A, W, T, P> {
    Sensor(S),
    Capture(C),
    Action(A),
    Watchdog(W),
    Transport(T),
    Persistence(P),
}

pub type AdapterResult<S, C, A, W, T, P, Output> =
    Result<Output, BoardAdapterError<S, C, A, W, T, P>>;

pub trait SchedulerObservabilitySource {
    fn timing_metrics(&self) -> ScheduledTimingMetrics;
    fn active_queue_count(&self) -> u8;
    fn free_queue_slots(&self) -> u8;
    fn queue_capacity(&self) -> u8;
    fn frontier_telemetry(&self) -> CommonFrontierTelemetry;
    fn scheduler_ownership_telemetry(&self) -> CommonSchedulerOwnershipTelemetry;
    fn scheduler_reservation_telemetry(&self) -> CommonSchedulerReservationTelemetry;
    fn scheduler_state_summary_telemetry(&self) -> CommonSchedulerStateSummaryTelemetry;
    fn scheduler_window_telemetry(&self) -> CommonSchedulerWindowTelemetry;
}

/// Adapter-local wrapper for applying an event and recording its observability sample.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ApplyAndRecordError<S, C, A, W, T, P> {
    Apply(BoardAdapterError<S, C, A, W, T, P>),
    Record(CommonObservabilityRecordTraceOverflow),
}

/// Adapter-local wrapper for polling the sensor and recording its observability sample.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PollSensorAndRecordError<S, C, A, W, T, P> {
    Poll(BoardAdapterError<S, C, A, W, T, P>),
    Record(CommonObservabilityRecordTraceOverflow),
}

/// Adapter-local wrapper for polling the sensor and pushing both observability traces.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PollSensorAndPushPairError<S, C, A, W, T, P> {
    Poll(BoardAdapterError<S, C, A, W, T, P>),
    Record(CommonObservabilityRecordTraceOverflow),
    Sample(CommonObservabilityTraceOverflow),
}

/// Adapter-local wrapper for pushing a paired observability sample and record.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PushObservabilityPairError {
    Record(CommonObservabilityRecordTraceOverflow),
    Sample(CommonObservabilityTraceOverflow),
}

/// Adapter-local wrapper for applying an event and pushing both observability traces.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ApplyAndPushPairError<S, C, A, W, T, P> {
    Apply(BoardAdapterError<S, C, A, W, T, P>),
    Record(CommonObservabilityRecordTraceOverflow),
    Sample(CommonObservabilityTraceOverflow),
}

/// Compact read-only diagnostics snapshot for the common adapter surface.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CommonDiagnosticsSnapshot {
    pub sync_state: SplitSyncState,
    pub fault_state: ecu_runtime::FaultState,
    pub timing_metrics: ScheduledTimingMetrics,
}

/// Compact read-only observability snapshot for common logging and inspection.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct CommonObservabilitySnapshot {
    pub diagnostics: CommonDiagnosticsTelemetry,
    pub fault_state: ecu_runtime::FaultState,
    pub fault: CommonRuntimeFaultTelemetry,
    pub lambda: CommonLambdaTelemetry,
    pub lambda_correction: CommonLambdaCorrectionTelemetry,
    pub warmup: CommonWarmupTelemetry,
    pub startup: CommonStartupTelemetry,
    pub afterstart: CommonAfterstartTelemetry,
    pub transient_enrichment: CommonTransientEnrichmentTelemetry,
    pub protection: CommonProtectionTelemetry,
    pub limp_action: CommonLimpActionTelemetry,
    pub high_rate_log: CommonHighRateLogTelemetry,
    pub sync_state: SplitSyncState,
    pub decision: CommonDecisionTelemetry,
    pub fuel_strategy_mode: CommonFuelStrategyMode,
    pub shift_arming: CommonShiftArmingTelemetry,
    pub pending_input: CommonPendingInputTelemetry,
    pub actions: CommonActionTelemetry,
    pub control: CommonControlTelemetry,
    pub control_reasons: CommonControlReasonTelemetry,
    pub fuel: CommonFuelObservationTelemetry,
    pub enrichment: CommonEnrichmentTelemetry,
    pub torque: CommonTorqueTelemetry,
    pub trigger_edge: CommonTriggerEdgeTelemetry,
    pub cam_edge: CommonCamEdgeTelemetry,
    pub engine: CommonEngineTelemetry,
    pub validated: CommonValidatedInputTelemetry,
    pub frontier: CommonFrontierTelemetry,
    pub scheduler_ownership: CommonSchedulerOwnershipTelemetry,
    pub scheduler_reservations: CommonSchedulerReservationTelemetry,
    pub scheduler_state_summary: CommonSchedulerStateSummaryTelemetry,
    pub scheduler_window: CommonSchedulerWindowTelemetry,
    pub engine_time: EngineTimeAuthorityTelemetry,
    pub fault_transition: CommonFaultTransitionTelemetry,
    pub calibration: CalibrationPackageIdentity,
    pub capture_sample: Option<CaptureSample>,
    pub logical_sensor_capture: Option<BoardSensorSnapshotCapture>,
    pub logical_sensor_snapshot: Option<BoardSensorSnapshot>,
    pub signal_assembly_counters: SignalAssemblyCounters,
    pub output_assembly_counters: OutputAssemblyCounters,
}

/// Timestamped read-only observability sample for common logging and inspection.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct CommonObservabilitySample {
    pub at_us: Micros,
    pub snapshot: CommonObservabilitySnapshot,
}

impl CommonObservabilitySnapshot {
    pub fn signal_assembly_counters(&self) -> SignalAssemblyCounters {
        self.signal_assembly_counters
    }

    pub fn output_assembly_counters(&self) -> OutputAssemblyCounters {
        self.output_assembly_counters
    }
}

/// Logical board event tag for a common observability record.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CommonObservabilityRecordKind {
    TriggerEdge,
    CamEdge,
    ShiftArming,
    CalibrationStagedDirty,
    FuelModelConfig,
    SpeedDensitySemanticConfig,
    RuntimeFuelStrategyConfig,
    SensorSnapshotCapture,
    SensorPoll,
    Tick,
}

/// Tagged common observability record.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CommonObservabilityRecord {
    pub kind: CommonObservabilityRecordKind,
    pub sample: CommonObservabilitySample,
}

/// Fixed-capacity observability trace overflow.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CommonObservabilityTraceOverflow {
    pub capacity: usize,
}

/// Read-only status snapshot for a common observability trace.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct CommonObservabilityTraceStatus {
    pub len: usize,
    pub capacity: usize,
    pub free_slots: usize,
    pub overflow_count: u32,
}

/// Drain result for common observability traces.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CommonObservabilityTraceDrainReport {
    pub drained: usize,
    pub status: CommonObservabilityTraceStatus,
}

/// Cycle report for a common observability trace drain.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CommonObservabilityTraceCycleReport {
    pub drained: usize,
    pub overflow_count: u32,
    pub status: CommonObservabilityTraceStatus,
}

/// Paired cycle report for draining sample and record traces together.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CommonObservabilityDrainCycleReport {
    pub sample: CommonObservabilityTraceCycleReport,
    pub record: CommonObservabilityTraceCycleReport,
}

/// Paired read-only status snapshot for common observability traces.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct CommonObservabilityTracePairStatus {
    pub sample: CommonObservabilityTraceStatus,
    pub record: CommonObservabilityTraceStatus,
}

#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
struct LastLambdaInputs {
    seen: bool,
    lambda_valid: bool,
    measured_lambda100: ecu_domain::Lambda100,
    requested_open_loop: bool,
}

/// Owned pair of common observability traces.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct FixedCommonObservabilityTracePair<const S: usize, const R: usize> {
    sample: FixedCommonObservabilityTrace<S>,
    record: FixedCommonObservabilityRecordTrace<R>,
}

impl<const S: usize, const R: usize> FixedCommonObservabilityTracePair<S, R> {
    pub const fn new() -> Self {
        Self {
            sample: FixedCommonObservabilityTrace::new(),
            record: FixedCommonObservabilityRecordTrace::new(),
        }
    }

    pub fn sample(&self) -> &FixedCommonObservabilityTrace<S> {
        &self.sample
    }

    pub fn sample_mut(&mut self) -> &mut FixedCommonObservabilityTrace<S> {
        &mut self.sample
    }

    pub fn record(&self) -> &FixedCommonObservabilityRecordTrace<R> {
        &self.record
    }

    pub fn record_mut(&mut self) -> &mut FixedCommonObservabilityRecordTrace<R> {
        &mut self.record
    }

    pub fn split_mut(
        &mut self,
    ) -> (
        &mut FixedCommonObservabilityTrace<S>,
        &mut FixedCommonObservabilityRecordTrace<R>,
    ) {
        (&mut self.sample, &mut self.record)
    }

    pub fn status(&self) -> CommonObservabilityTracePairStatus {
        common_observability_trace_status(&self.sample, &self.record)
    }

    pub fn drain_cycle<const SO: usize, const RO: usize>(
        &mut self,
        sample_out: &mut [CommonObservabilitySample; SO],
        record_out: &mut [CommonObservabilityRecord; RO],
    ) -> CommonObservabilityDrainCycleReport {
        let mut sample_slots = [None; SO];
        let mut record_slots = [None; RO];
        let report = drain_common_observability_cycle(
            &mut self.sample,
            &mut sample_slots,
            &mut self.record,
            &mut record_slots,
        );

        for (slot, value) in sample_out.iter_mut().zip(sample_slots) {
            if let Some(value) = value {
                *slot = value;
            }
        }
        for (slot, value) in record_out.iter_mut().zip(record_slots) {
            if let Some(value) = value {
                *slot = value;
            }
        }

        report
    }
}

impl<const S: usize, const R: usize> Default for FixedCommonObservabilityTracePair<S, R> {
    fn default() -> Self {
        Self::new()
    }
}

/// Fixed-capacity trace for common observability samples.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct FixedCommonObservabilityTrace<const N: usize> {
    items: [Option<CommonObservabilitySample>; N],
    len: usize,
    overflow_count: u32,
}

impl<const N: usize> FixedCommonObservabilityTrace<N> {
    pub const fn new() -> Self {
        Self {
            items: [None; N],
            len: 0,
            overflow_count: 0,
        }
    }

    pub const fn capacity(&self) -> usize {
        N
    }

    pub fn len(&self) -> usize {
        self.len
    }

    pub fn is_empty(&self) -> bool {
        self.len == 0
    }

    pub fn overflow_count(&self) -> u32 {
        self.overflow_count
    }

    pub fn take_overflow_count(&mut self) -> u32 {
        let overflow_count = self.overflow_count;
        self.overflow_count = 0;
        overflow_count
    }

    pub fn free_slots(&self) -> usize {
        N.saturating_sub(self.len)
    }

    pub const fn status(&self) -> CommonObservabilityTraceStatus {
        CommonObservabilityTraceStatus {
            len: self.len,
            capacity: N,
            free_slots: N.saturating_sub(self.len),
            overflow_count: self.overflow_count,
        }
    }

    pub fn get(&self, index: usize) -> Option<CommonObservabilitySample> {
        if index < self.len {
            self.items[index]
        } else {
            None
        }
    }

    pub fn push(
        &mut self,
        sample: CommonObservabilitySample,
    ) -> Result<(), CommonObservabilityTraceOverflow> {
        if self.len == N {
            self.overflow_count = self.overflow_count.saturating_add(1);
            return Err(CommonObservabilityTraceOverflow { capacity: N });
        }

        self.items[self.len] = Some(sample);
        self.len += 1;
        Ok(())
    }

    pub fn drain_into(&mut self, out: &mut [Option<CommonObservabilitySample>]) -> usize {
        if out.is_empty() {
            return 0;
        }

        let count = self.len.min(out.len());
        for (idx, slot) in out.iter_mut().enumerate() {
            if idx < count {
                *slot = self.items[idx];
            } else {
                *slot = None;
            }
        }

        let remaining = self.len - count;
        for idx in 0..remaining {
            self.items[idx] = self.items[count + idx];
        }
        for idx in remaining..self.len {
            self.items[idx] = None;
        }
        self.len = remaining;
        count
    }

    pub fn drain_with_status(
        &mut self,
        out: &mut [Option<CommonObservabilitySample>],
    ) -> CommonObservabilityTraceDrainReport {
        let drained = self.drain_into(out);
        CommonObservabilityTraceDrainReport {
            drained,
            status: self.status(),
        }
    }

    pub fn drain_cycle(
        &mut self,
        out: &mut [Option<CommonObservabilitySample>],
    ) -> CommonObservabilityTraceCycleReport {
        let overflow_count = self.take_overflow_count();
        let drained = self.drain_into(out);
        CommonObservabilityTraceCycleReport {
            drained,
            overflow_count,
            status: self.status(),
        }
    }

    pub fn clear(&mut self) {
        self.len = 0;
    }
}

impl<const N: usize> Default for FixedCommonObservabilityTrace<N> {
    fn default() -> Self {
        Self::new()
    }
}

/// Fixed-capacity common observability record trace overflow.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CommonObservabilityRecordTraceOverflow {
    pub capacity: usize,
}

/// Fixed-capacity trace for tagged common observability records.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct FixedCommonObservabilityRecordTrace<const N: usize> {
    items: [Option<CommonObservabilityRecord>; N],
    len: usize,
    overflow_count: u32,
}

impl<const N: usize> FixedCommonObservabilityRecordTrace<N> {
    pub const fn new() -> Self {
        Self {
            items: [None; N],
            len: 0,
            overflow_count: 0,
        }
    }

    pub const fn capacity(&self) -> usize {
        N
    }

    pub fn len(&self) -> usize {
        self.len
    }

    pub fn is_empty(&self) -> bool {
        self.len == 0
    }

    pub fn overflow_count(&self) -> u32 {
        self.overflow_count
    }

    pub fn take_overflow_count(&mut self) -> u32 {
        let overflow_count = self.overflow_count;
        self.overflow_count = 0;
        overflow_count
    }

    pub fn free_slots(&self) -> usize {
        N.saturating_sub(self.len)
    }

    pub const fn status(&self) -> CommonObservabilityTraceStatus {
        CommonObservabilityTraceStatus {
            len: self.len,
            capacity: N,
            free_slots: N.saturating_sub(self.len),
            overflow_count: self.overflow_count,
        }
    }

    pub fn get(&self, index: usize) -> Option<CommonObservabilityRecord> {
        if index < self.len {
            self.items[index]
        } else {
            None
        }
    }

    pub fn push(
        &mut self,
        record: CommonObservabilityRecord,
    ) -> Result<(), CommonObservabilityRecordTraceOverflow> {
        if self.len == N {
            self.overflow_count = self.overflow_count.saturating_add(1);
            return Err(CommonObservabilityRecordTraceOverflow { capacity: N });
        }

        self.items[self.len] = Some(record);
        self.len += 1;
        Ok(())
    }

    pub fn drain_into(&mut self, out: &mut [Option<CommonObservabilityRecord>]) -> usize {
        if out.is_empty() {
            return 0;
        }

        let count = self.len.min(out.len());
        for (idx, slot) in out.iter_mut().enumerate() {
            if idx < count {
                *slot = self.items[idx];
            } else {
                *slot = None;
            }
        }

        let remaining = self.len - count;
        for idx in 0..remaining {
            self.items[idx] = self.items[count + idx];
        }
        for idx in remaining..self.len {
            self.items[idx] = None;
        }
        self.len = remaining;
        count
    }

    pub fn drain_with_status(
        &mut self,
        out: &mut [Option<CommonObservabilityRecord>],
    ) -> CommonObservabilityTraceDrainReport {
        let drained = self.drain_into(out);
        CommonObservabilityTraceDrainReport {
            drained,
            status: self.status(),
        }
    }

    pub fn drain_cycle(
        &mut self,
        out: &mut [Option<CommonObservabilityRecord>],
    ) -> CommonObservabilityTraceCycleReport {
        let overflow_count = self.take_overflow_count();
        let drained = self.drain_into(out);
        CommonObservabilityTraceCycleReport {
            drained,
            overflow_count,
            status: self.status(),
        }
    }

    pub fn clear(&mut self) {
        self.len = 0;
    }
}

impl<const N: usize> Default for FixedCommonObservabilityRecordTrace<N> {
    fn default() -> Self {
        Self::new()
    }
}

pub fn drain_common_observability_cycle<
    const S: usize,
    const R: usize,
    const SO: usize,
    const RO: usize,
>(
    sample_trace: &mut FixedCommonObservabilityTrace<S>,
    sample_out: &mut [Option<CommonObservabilitySample>; SO],
    record_trace: &mut FixedCommonObservabilityRecordTrace<R>,
    record_out: &mut [Option<CommonObservabilityRecord>; RO],
) -> CommonObservabilityDrainCycleReport {
    let sample = sample_trace.drain_cycle(sample_out);
    let record = record_trace.drain_cycle(record_out);
    CommonObservabilityDrainCycleReport { sample, record }
}

pub fn common_observability_trace_status<const S: usize, const R: usize>(
    sample_trace: &FixedCommonObservabilityTrace<S>,
    record_trace: &FixedCommonObservabilityRecordTrace<R>,
) -> CommonObservabilityTracePairStatus {
    CommonObservabilityTracePairStatus {
        sample: sample_trace.status(),
        record: record_trace.status(),
    }
}

fn common_sync_telemetry_state(sync_state: SplitSyncState) -> CommonSyncTelemetryState {
    match sync_state {
        SplitSyncState::NoSignal => CommonSyncTelemetryState::NoSignal,
        SplitSyncState::Unsynced => CommonSyncTelemetryState::Unsynced,
        SplitSyncState::CrankSynced => CommonSyncTelemetryState::CrankSynced,
        SplitSyncState::CamSynced => CommonSyncTelemetryState::CamSynced,
        SplitSyncState::FullSequentialAuthorized => {
            CommonSyncTelemetryState::FullSequentialAuthorized
        }
        SplitSyncState::SyncSuspect => CommonSyncTelemetryState::SyncSuspect,
        SplitSyncState::SyncLost => CommonSyncTelemetryState::SyncLost,
    }
}

fn common_lambda_mode(mode: ecu_runtime::LambdaMode) -> CommonLambdaMode {
    match mode {
        ecu_runtime::LambdaMode::OpenLoop => CommonLambdaMode::OpenLoop,
        ecu_runtime::LambdaMode::ClosedLoop => CommonLambdaMode::ClosedLoop,
    }
}

fn common_lambda_disable_reason(
    reason: ecu_runtime::LambdaDisableReason,
) -> CommonLambdaDisableReason {
    match reason {
        ecu_runtime::LambdaDisableReason::None => CommonLambdaDisableReason::None,
        ecu_runtime::LambdaDisableReason::RequestedOpenLoop => {
            CommonLambdaDisableReason::RequestedOpenLoop
        }
        ecu_runtime::LambdaDisableReason::SensorInvalid => CommonLambdaDisableReason::SensorInvalid,
        ecu_runtime::LambdaDisableReason::WarmupGate => CommonLambdaDisableReason::WarmupGate,
        ecu_runtime::LambdaDisableReason::LowLoadGate => CommonLambdaDisableReason::LowLoadGate,
        ecu_runtime::LambdaDisableReason::StartupDelay => CommonLambdaDisableReason::StartupDelay,
        ecu_runtime::LambdaDisableReason::PowerReductionCut => {
            CommonLambdaDisableReason::PowerReductionCut
        }
        ecu_runtime::LambdaDisableReason::AccelerationEnrichment => {
            CommonLambdaDisableReason::AccelerationEnrichment
        }
    }
}

fn common_lambda_telemetry(
    reasons: CommonControlReasonTelemetry,
    last_inputs: LastLambdaInputs,
) -> CommonLambdaTelemetry {
    if !last_inputs.seen {
        return CommonLambdaTelemetry::default();
    }

    if reasons.lambda_active {
        return CommonLambdaTelemetry {
            activity: CommonLambdaActivity::Active,
            reason: CommonLambdaDisableReason::None,
        };
    }

    match reasons.lambda_disable_reason {
        CommonLambdaDisableReason::RequestedOpenLoop => CommonLambdaTelemetry {
            activity: CommonLambdaActivity::Frozen,
            reason: CommonLambdaDisableReason::RequestedOpenLoop,
        },
        CommonLambdaDisableReason::SensorInvalid => CommonLambdaTelemetry {
            activity: CommonLambdaActivity::Inactive,
            reason: CommonLambdaDisableReason::SensorInvalid,
        },
        CommonLambdaDisableReason::WarmupGate => CommonLambdaTelemetry {
            activity: CommonLambdaActivity::Inactive,
            reason: CommonLambdaDisableReason::WarmupGate,
        },
        CommonLambdaDisableReason::LowLoadGate => CommonLambdaTelemetry {
            activity: CommonLambdaActivity::Inactive,
            reason: CommonLambdaDisableReason::LowLoadGate,
        },
        CommonLambdaDisableReason::StartupDelay => CommonLambdaTelemetry {
            activity: CommonLambdaActivity::Inactive,
            reason: CommonLambdaDisableReason::StartupDelay,
        },
        CommonLambdaDisableReason::PowerReductionCut => CommonLambdaTelemetry {
            activity: CommonLambdaActivity::Frozen,
            reason: CommonLambdaDisableReason::PowerReductionCut,
        },
        CommonLambdaDisableReason::AccelerationEnrichment => CommonLambdaTelemetry {
            activity: CommonLambdaActivity::Frozen,
            reason: CommonLambdaDisableReason::AccelerationEnrichment,
        },
        CommonLambdaDisableReason::None | CommonLambdaDisableReason::OpenLoop => {
            if matches!(reasons.lambda_mode, CommonLambdaMode::OpenLoop) {
                CommonLambdaTelemetry {
                    activity: CommonLambdaActivity::Inactive,
                    reason: CommonLambdaDisableReason::OpenLoop,
                }
            } else {
                CommonLambdaTelemetry::default()
            }
        }
    }
}

fn common_lambda_correction_telemetry(
    control: CommonControlTelemetry,
    reasons: CommonControlReasonTelemetry,
    status: CommonLambdaTelemetry,
    last_inputs: LastLambdaInputs,
) -> CommonLambdaCorrectionTelemetry {
    if !last_inputs.seen {
        return CommonLambdaCorrectionTelemetry::default();
    }

    CommonLambdaCorrectionTelemetry {
        measured_lambda: last_inputs.measured_lambda100,
        target_lambda: control.lambda_target,
        trim_x100: reasons.lambda_trim_x100,
        status,
    }
}

fn common_ignition_limit_reason(
    reason: ecu_runtime::IgnitionLimitReason,
) -> CommonIgnitionLimitReason {
    match reason {
        ecu_runtime::IgnitionLimitReason::None => CommonIgnitionLimitReason::None,
        ecu_runtime::IgnitionLimitReason::Knock => CommonIgnitionLimitReason::Knock,
        ecu_runtime::IgnitionLimitReason::Torque => CommonIgnitionLimitReason::Torque,
        ecu_runtime::IgnitionLimitReason::RevLimiter => CommonIgnitionLimitReason::RevLimiter,
    }
}

fn common_torque_limit_reason(reason: ecu_runtime::TorqueLimitReason) -> CommonTorqueLimitReason {
    match reason {
        ecu_runtime::TorqueLimitReason::None => CommonTorqueLimitReason::None,
        ecu_runtime::TorqueLimitReason::Idle => CommonTorqueLimitReason::Idle,
        ecu_runtime::TorqueLimitReason::Driver => CommonTorqueLimitReason::Driver,
        ecu_runtime::TorqueLimitReason::RevLimiter => CommonTorqueLimitReason::RevLimiter,
        ecu_runtime::TorqueLimitReason::Knock => CommonTorqueLimitReason::Knock,
        ecu_runtime::TorqueLimitReason::LimpMode => CommonTorqueLimitReason::LimpMode,
    }
}

fn common_scheduler_mode(mode: ecu_scheduler::SchedulerMode) -> CommonSchedulerMode {
    match mode {
        ecu_scheduler::SchedulerMode::Idle => CommonSchedulerMode::Idle,
        ecu_scheduler::SchedulerMode::Armed => CommonSchedulerMode::Armed,
        ecu_scheduler::SchedulerMode::Suspended => CommonSchedulerMode::Suspended,
    }
}

fn common_frontier_fault_telemetry(
    reason: ecu_board_api::TimingIslandStopReason,
) -> CommonFrontierFaultTelemetry {
    use ecu_board_api::TimingIslandStopReason::{
        AdmittedEventRejected, BoardOutputFault, HeartbeatExpired, HorizonExpired, None,
        PermitDenied, SyncLost, TimingFault,
    };

    match reason {
        None => CommonFrontierFaultTelemetry {
            event_id: CommonFrontierFaultEventId::None,
            severity: ecu_domain::FaultSeverity::Info,
            action: CommonFrontierFaultAction::None,
        },
        SyncLost => CommonFrontierFaultTelemetry {
            event_id: CommonFrontierFaultEventId::SyncLost,
            severity: ecu_domain::FaultSeverity::Warning,
            action: CommonFrontierFaultAction::SafeStateTransition,
        },
        HeartbeatExpired => CommonFrontierFaultTelemetry {
            event_id: CommonFrontierFaultEventId::HeartbeatExpired,
            severity: ecu_domain::FaultSeverity::Warning,
            action: CommonFrontierFaultAction::OutputSuppressed,
        },
        HorizonExpired => CommonFrontierFaultTelemetry {
            event_id: CommonFrontierFaultEventId::HorizonExpired,
            severity: ecu_domain::FaultSeverity::Info,
            action: CommonFrontierFaultAction::OutputSuppressed,
        },
        PermitDenied => CommonFrontierFaultTelemetry {
            event_id: CommonFrontierFaultEventId::PermitDenied,
            severity: ecu_domain::FaultSeverity::Warning,
            action: CommonFrontierFaultAction::SafeStateTransition,
        },
        TimingFault => CommonFrontierFaultTelemetry {
            event_id: CommonFrontierFaultEventId::TimingFault,
            severity: ecu_domain::FaultSeverity::Critical,
            action: CommonFrontierFaultAction::SafeStateTransition,
        },
        AdmittedEventRejected => CommonFrontierFaultTelemetry {
            event_id: CommonFrontierFaultEventId::AdmittedEventRejected,
            severity: ecu_domain::FaultSeverity::Critical,
            action: CommonFrontierFaultAction::SafeStateTransition,
        },
        BoardOutputFault => CommonFrontierFaultTelemetry {
            event_id: CommonFrontierFaultEventId::BoardOutputFault,
            severity: ecu_domain::FaultSeverity::Critical,
            action: CommonFrontierFaultAction::SafeStateTransition,
        },
    }
}

fn common_fault_transition_event(
    previous: ecu_runtime::FaultState,
    current: ecu_runtime::FaultState,
) -> CommonFaultTransitionEventTelemetry {
    let event_id = if previous.fault == ecu_domain::FaultCode::None
        && current.fault != ecu_domain::FaultCode::None
    {
        CommonFaultTransitionEventId::FaultEntered
    } else if previous.fault != ecu_domain::FaultCode::None
        && current.fault == ecu_domain::FaultCode::None
    {
        CommonFaultTransitionEventId::FaultCleared
    } else {
        CommonFaultTransitionEventId::FaultUpdated
    };

    let action = if current.fault == ecu_domain::FaultCode::None {
        CommonFaultTransitionAction::Cleared
    } else if current.cancel_reason == ecu_domain::CancelReason::SafetyShutdown
        || current.severity == ecu_domain::FaultSeverity::Critical
    {
        CommonFaultTransitionAction::Shutdown
    } else if current.severity == ecu_domain::FaultSeverity::Warning
        || current.fault == ecu_domain::FaultCode::SensorOutOfRange
    {
        CommonFaultTransitionAction::LimpHome
    } else {
        CommonFaultTransitionAction::ObserveOnly
    };

    CommonFaultTransitionEventTelemetry {
        event_id,
        severity: current.severity,
        action,
    }
}

fn common_runtime_fault_telemetry(state: ecu_runtime::FaultState) -> CommonRuntimeFaultTelemetry {
    let active = state.fault != ecu_domain::FaultCode::None;
    let action = if !active {
        CommonFaultTransitionAction::None
    } else if state.cancel_reason == ecu_domain::CancelReason::SafetyShutdown
        || state.severity == ecu_domain::FaultSeverity::Critical
    {
        CommonFaultTransitionAction::Shutdown
    } else if state.severity == ecu_domain::FaultSeverity::Warning
        || state.fault == ecu_domain::FaultCode::SensorOutOfRange
    {
        CommonFaultTransitionAction::LimpHome
    } else {
        CommonFaultTransitionAction::ObserveOnly
    };

    CommonRuntimeFaultTelemetry {
        active,
        fault_code: state.fault,
        severity: state.severity,
        cancel_reason: state.cancel_reason,
        action,
    }
}

fn common_protection_telemetry(
    decision: CommonDecisionTelemetry,
    fault: CommonRuntimeFaultTelemetry,
    frontier_fault: CommonFrontierFaultTelemetry,
) -> CommonProtectionTelemetry {
    if frontier_fault.action != CommonFrontierFaultAction::None {
        return CommonProtectionTelemetry {
            level: if frontier_fault.action == CommonFrontierFaultAction::OutputSuppressed {
                CommonProtectionLevel::Degraded
            } else {
                CommonProtectionLevel::ShutdownDriving
            },
            source: CommonProtectionSource::FrontierFault,
            action: match frontier_fault.action {
                CommonFrontierFaultAction::None => CommonProtectionAction::None,
                CommonFrontierFaultAction::OutputSuppressed => {
                    CommonProtectionAction::OutputSuppressed
                }
                CommonFrontierFaultAction::SafeStateTransition => {
                    CommonProtectionAction::SafeStateTransition
                }
            },
            persistence: CommonProtectionPersistence::LatchedUntilRecovery,
        };
    }

    if fault.active {
        let action = match fault.action {
            CommonFaultTransitionAction::None | CommonFaultTransitionAction::Cleared => {
                CommonProtectionAction::None
            }
            CommonFaultTransitionAction::ObserveOnly => CommonProtectionAction::ObserveOnly,
            CommonFaultTransitionAction::LimpHome => CommonProtectionAction::LimpHome,
            CommonFaultTransitionAction::Shutdown => CommonProtectionAction::Shutdown,
        };

        return CommonProtectionTelemetry {
            level: if action == CommonProtectionAction::Shutdown {
                CommonProtectionLevel::ShutdownDriving
            } else {
                CommonProtectionLevel::Degraded
            },
            source: CommonProtectionSource::RuntimeFault,
            action,
            persistence: CommonProtectionPersistence::LatchedUntilClear,
        };
    }

    match decision.control_mode {
        ControlMode::Shutdown => CommonProtectionTelemetry {
            level: CommonProtectionLevel::ShutdownDriving,
            source: CommonProtectionSource::ControlMode,
            action: CommonProtectionAction::Shutdown,
            persistence: CommonProtectionPersistence::Reversible,
        },
        ControlMode::LimpHome => CommonProtectionTelemetry {
            level: CommonProtectionLevel::Degraded,
            source: CommonProtectionSource::ControlMode,
            action: CommonProtectionAction::LimpHome,
            persistence: CommonProtectionPersistence::Reversible,
        },
        _ => CommonProtectionTelemetry::default(),
    }
}

fn common_limp_action_telemetry(
    actions: CommonActionTelemetry,
    protection: CommonProtectionTelemetry,
    frontier_fault: CommonFrontierFaultTelemetry,
) -> CommonLimpActionTelemetry {
    let apply_aux = actions.apply_aux_count > 0 || actions.apply_aux_command_count > 0;
    let mapped_source = match protection.source {
        CommonProtectionSource::RuntimeFault => CommonLimpActionSource::RuntimeFault,
        CommonProtectionSource::FrontierFault => CommonLimpActionSource::FrontierFault,
        CommonProtectionSource::ControlMode => CommonLimpActionSource::ControlMode,
        CommonProtectionSource::None => CommonLimpActionSource::None,
    };

    if frontier_fault.action != CommonFrontierFaultAction::None {
        return CommonLimpActionTelemetry {
            level: if frontier_fault.action == CommonFrontierFaultAction::OutputSuppressed {
                CommonLimpActionLevel::OutputSuppressed
            } else {
                CommonLimpActionLevel::ShutdownDriving
            },
            source: CommonLimpActionSource::FrontierFault,
            cancel_scheduler: actions.cancel_scheduler,
            cancel_reason: actions.cancel_reason,
            apply_aux,
            aux_command_count: actions.apply_aux_command_count,
            persistence: CommonProtectionPersistence::LatchedUntilRecovery,
        };
    }

    if actions.cancel_scheduler && actions.cancel_reason == ecu_domain::CancelReason::SyncLoss {
        return CommonLimpActionTelemetry {
            level: CommonLimpActionLevel::OutputSuppressed,
            source: if mapped_source == CommonLimpActionSource::None {
                CommonLimpActionSource::SyncAuthority
            } else {
                mapped_source
            },
            cancel_scheduler: true,
            cancel_reason: actions.cancel_reason,
            apply_aux,
            aux_command_count: actions.apply_aux_command_count,
            persistence: CommonProtectionPersistence::LatchedUntilRecovery,
        };
    }

    if actions.cancel_scheduler {
        return CommonLimpActionTelemetry {
            level: CommonLimpActionLevel::ShutdownDriving,
            source: mapped_source,
            cancel_scheduler: actions.cancel_scheduler,
            cancel_reason: actions.cancel_reason,
            apply_aux,
            aux_command_count: actions.apply_aux_command_count,
            persistence: protection.persistence,
        };
    }

    if apply_aux {
        return CommonLimpActionTelemetry {
            level: CommonLimpActionLevel::AuxOnly,
            source: mapped_source,
            cancel_scheduler: false,
            cancel_reason: actions.cancel_reason,
            apply_aux: true,
            aux_command_count: actions.apply_aux_command_count,
            persistence: protection.persistence,
        };
    }

    CommonLimpActionTelemetry::default()
}

fn common_high_rate_log_telemetry(
    decision: CommonDecisionTelemetry,
    fault: CommonRuntimeFaultTelemetry,
    frontier_fault: CommonFrontierFaultTelemetry,
    diagnostics: CommonDiagnosticsTelemetry,
    calibration: CalibrationPackageIdentity,
) -> CommonHighRateLogTelemetry {
    CommonHighRateLogTelemetry {
        decision,
        fault,
        frontier_fault,
        late_event_count: diagnostics.late_event_count,
        max_lateness_us: diagnostics.max_lateness_us,
        calibration_checksum: calibration.checksum.get(),
    }
}

fn common_fuel_cut_reason(snapshot: &RuntimeSnapshot) -> CommonCutReason {
    if !snapshot.fuel_cut {
        return CommonCutReason::None;
    }

    if snapshot.safety_latched {
        CommonCutReason::SafetyLatched
    } else if snapshot.direct_fuel_cut_request {
        CommonCutReason::DirectRequest
    } else if matches!(snapshot.engine.mode, ControlMode::Shutdown) {
        CommonCutReason::Shutdown
    } else if snapshot.rev_hard_active {
        CommonCutReason::HardRev
    } else if snapshot.launch_active {
        CommonCutReason::Launch
    } else if snapshot.flat_shift_active {
        CommonCutReason::FlatShift
    } else if snapshot.fuel_cut && !snapshot.spark_cut {
        CommonCutReason::FuelOnly
    } else if snapshot.rev_soft_active {
        CommonCutReason::SoftRev
    } else if snapshot.direct_spark_cut_request {
        CommonCutReason::DirectRequest
    } else {
        CommonCutReason::None
    }
}

fn common_spark_cut_reason(snapshot: &RuntimeSnapshot) -> CommonCutReason {
    if !snapshot.spark_cut {
        return CommonCutReason::None;
    }

    if snapshot.safety_latched {
        CommonCutReason::SafetyLatched
    } else if snapshot.direct_spark_cut_request {
        CommonCutReason::DirectRequest
    } else if matches!(snapshot.engine.mode, ControlMode::Shutdown) {
        CommonCutReason::Shutdown
    } else if snapshot.rev_hard_active {
        CommonCutReason::HardRev
    } else if snapshot.launch_active {
        CommonCutReason::Launch
    } else if snapshot.flat_shift_active {
        CommonCutReason::FlatShift
    } else if snapshot.rev_soft_active {
        CommonCutReason::SoftRev
    } else if snapshot.spark_cut && !snapshot.fuel_cut {
        CommonCutReason::SparkOnly
    } else if snapshot.legacy_cut_reason_code == 7 {
        CommonCutReason::KnockRetard
    } else if snapshot.direct_fuel_cut_request {
        CommonCutReason::DirectRequest
    } else {
        CommonCutReason::None
    }
}

fn common_fuel_strategy_mode(strategy: &RuntimeFuelStrategy) -> CommonFuelStrategyMode {
    match strategy {
        RuntimeFuelStrategy::DirectPulseWidthTable(_) => {
            CommonFuelStrategyMode::DirectPulseWidthTable
        }
        RuntimeFuelStrategy::SpeedDensityVe { .. } => CommonFuelStrategyMode::SpeedDensityVe,
        RuntimeFuelStrategy::AlphaN { .. } => CommonFuelStrategyMode::AlphaN,
        RuntimeFuelStrategy::Maf { .. } => CommonFuelStrategyMode::Maf,
    }
}

pub(super) fn common_action_telemetry(result: &StepResult) -> CommonActionTelemetry {
    let mut action_telemetry = CommonActionTelemetry::default();

    for action in result.actions.iter() {
        action_telemetry.total_action_count = action_telemetry.total_action_count.saturating_add(1);
        match action {
            Action::ArmScheduler { .. } => {
                action_telemetry.arm_scheduler_count =
                    action_telemetry.arm_scheduler_count.saturating_add(1);
            }
            Action::ArmInjection(_) => {
                action_telemetry.arm_injection_count =
                    action_telemetry.arm_injection_count.saturating_add(1);
            }
            Action::ArmIgnition(_) => {
                action_telemetry.arm_ignition_count =
                    action_telemetry.arm_ignition_count.saturating_add(1);
            }
            Action::ApplyAux(commands) => {
                action_telemetry.apply_aux_count =
                    action_telemetry.apply_aux_count.saturating_add(1);
                action_telemetry.apply_aux_command_count = action_telemetry
                    .apply_aux_command_count
                    .saturating_add(commands.len().min(u8::MAX as usize) as u8);
            }
            Action::PublishSnapshot => {
                action_telemetry.publish_snapshot = true;
                action_telemetry.publish_snapshot_count =
                    action_telemetry.publish_snapshot_count.saturating_add(1);
            }
            Action::PersistCalibration => {
                action_telemetry.persist_calibration = true;
                action_telemetry.persist_calibration_count =
                    action_telemetry.persist_calibration_count.saturating_add(1);
            }
            Action::CancelScheduler(cancel_reason) => {
                if action_telemetry.cancel_scheduler
                    && action_telemetry.cancel_reason != cancel_reason
                {
                    action_telemetry.multiple_cancel_reasons = true;
                }
                action_telemetry.cancel_scheduler = true;
                action_telemetry.cancel_reason = cancel_reason;
                action_telemetry.cancel_scheduler_count =
                    action_telemetry.cancel_scheduler_count.saturating_add(1);
            }
            Action::Idle => {
                action_telemetry.idle_count = action_telemetry.idle_count.saturating_add(1);
            }
        }
    }

    action_telemetry
}

fn common_action_output_count(action: Action) -> u32 {
    match action {
        Action::ArmScheduler { .. } => 2,
        Action::ArmInjection(_) | Action::ArmIgnition(_) => 1,
        Action::ApplyAux(commands) => commands.len() as u32,
        Action::CancelScheduler(_)
        | Action::PublishSnapshot
        | Action::PersistCalibration
        | Action::Idle => 0,
    }
}

fn initial_inputs() -> AuthorityStepInputs {
    AuthorityStepInputs {
        now_us: Micros::new(0),
        rpm: 0,
        load_kpa10: 0,
        angle_x10: 0,
        authority: EngineTimeAuthority::none(),
        flat_shift_armed: false,
        launch_armed: false,
        safety_latch_request: false,
    }
}

/// Shared board adapter that turns board events into runtime updates and IO actions.
///
/// Invariants:
/// - board events update runtime input state only
/// - control decisions come from `EngineRuntime::step_with_authority`
/// - action delivery is limited to I/O plumbing and publishing
#[derive(Debug, Clone, PartialEq)]
pub struct BoardAdapter<S, C, A, W, T, P> {
    runtime: EngineRuntime,
    sensor: S,
    capture: C,
    actions: A,
    watchdog: W,
    transport: T,
    store: P,
    pending_inputs: AuthorityStepInputs,
    action_telemetry: CommonActionTelemetry,
    control_reasons: CommonControlReasonTelemetry,
    last_lambda_inputs: LastLambdaInputs,
    fault_transition: CommonFaultTransitionTelemetry,
    staged_dirty: bool,
    fuel: CommonFuelObservationTelemetry,
    enrichment: CommonEnrichmentTelemetry,
    warmup: CommonWarmupTelemetry,
    startup: CommonStartupTelemetry,
    afterstart: CommonAfterstartTelemetry,
    transient_enrichment: CommonTransientEnrichmentTelemetry,
    torque: CommonTorqueTelemetry,
    trigger_edge: CommonTriggerEdgeTelemetry,
    cam_edge: CommonCamEdgeTelemetry,
    validated: CommonValidatedInputTelemetry,
    signal_assembly_counters: SignalAssemblyCounters,
    output_assembly_counters: OutputAssemblyCounters,
    capture_sample: Option<CaptureSample>,
    logical_sensor_capture: Option<BoardSensorSnapshotCapture>,
    diag_log: DiagLog<DIAG_LOG_ENTRY_COUNT>,
    store_integrity_latched: bool,
    #[cfg(feature = "transport-can")]
    retained_obd2_history: crate::transport_service::Obd2RetainedDiagnosticHistory,
    #[cfg(feature = "transport-can")]
    provisioned_obd2_identity: crate::transport_service::ProvisionedObd2Identity,
    #[cfg(feature = "transport-can")]
    obd2_identity_key_lifecycle_status: Option<ecu_transport::CanObd2IdentityKeyLifecycleStatus>,
    #[cfg(feature = "transport-can")]
    obd2_flash_write_fault_status: Option<ecu_transport::CanObd2FlashWriteFaultStatus>,
    #[cfg(feature = "transport-can")]
    shared_live_diag_state: SharedLiveDiagState,
    #[cfg(feature = "transport-can")]
    sensor_limits: SharedSensorLimits,
}

impl<S, C, A, W, T, P> BoardAdapter<S, C, A, W, T, P>
where
    S: CaptureSampleSource,
    C: CaptureSink,
    A: ActionExecutor,
    W: Watchdog,
    T: TransportPublisher,
    P: PersistedCalibrationStore,
{
    pub fn new(sensor: S, capture: C, actions: A, watchdog: W, transport: T, store: P) -> Self {
        Self {
            runtime: EngineRuntime::new(),
            sensor,
            capture,
            actions,
            watchdog,
            transport,
            store,
            pending_inputs: initial_inputs(),
            action_telemetry: CommonActionTelemetry::default(),
            control_reasons: CommonControlReasonTelemetry::default(),
            last_lambda_inputs: LastLambdaInputs::default(),
            fault_transition: CommonFaultTransitionTelemetry::default(),
            staged_dirty: false,
            fuel: CommonFuelObservationTelemetry::default(),
            enrichment: CommonEnrichmentTelemetry::default(),
            warmup: CommonWarmupTelemetry::default(),
            startup: CommonStartupTelemetry::default(),
            afterstart: CommonAfterstartTelemetry::default(),
            transient_enrichment: CommonTransientEnrichmentTelemetry::default(),
            torque: CommonTorqueTelemetry::default(),
            trigger_edge: CommonTriggerEdgeTelemetry::default(),
            cam_edge: CommonCamEdgeTelemetry::default(),
            validated: CommonValidatedInputTelemetry::default(),
            signal_assembly_counters: SignalAssemblyCounters::default(),
            output_assembly_counters: OutputAssemblyCounters::default(),
            capture_sample: None,
            logical_sensor_capture: None,
            diag_log: DiagLog::new(),
            store_integrity_latched: false,
            #[cfg(feature = "transport-can")]
            retained_obd2_history: crate::transport_service::Obd2RetainedDiagnosticHistory::new(
                crate::transport_service::obd2_current_data_from_board_inputs(None, None),
            ),
            #[cfg(feature = "transport-can")]
            provisioned_obd2_identity: crate::transport_service::ProvisionedObd2Identity::default(),
            #[cfg(feature = "transport-can")]
            obd2_identity_key_lifecycle_status: Some(
                ecu_transport::CanObd2IdentityKeyLifecycleStatus::absent(),
            ),
            #[cfg(feature = "transport-can")]
            obd2_flash_write_fault_status: None,
            #[cfg(feature = "transport-can")]
            shared_live_diag_state: SharedLiveDiagState::default(),
            #[cfg(feature = "transport-can")]
            sensor_limits: SharedSensorLimits::default(),
        }
    }

    pub fn runtime(&self) -> &EngineRuntime {
        &self.runtime
    }

    pub fn fault_state(&self) -> ecu_runtime::FaultState {
        self.runtime.snapshot().faults
    }

    pub fn sync_state(&self) -> SplitSyncState {
        SplitSyncState::from_authority(self.runtime.engine_time_authority())
    }

    pub fn calibration_package_identity(&self) -> CalibrationPackageIdentity {
        CalibrationPackageIdentity::from_snapshot_with_staged_dirty(
            self.runtime.calibration_snapshot(),
            self.staged_dirty,
        )
    }

    #[cfg(feature = "transport-can")]
    pub const fn provisioned_obd2_identity(
        &self,
    ) -> crate::transport_service::ProvisionedObd2Identity {
        self.provisioned_obd2_identity
    }

    #[cfg(feature = "transport-can")]
    pub fn set_provisioned_obd2_identity(
        &mut self,
        identity: crate::transport_service::ProvisionedObd2Identity,
    ) {
        self.provisioned_obd2_identity = identity;
    }

    #[cfg(feature = "transport-can")]
    pub fn install_provisioned_obd2_identity_record(
        &mut self,
        record: crate::transport_service::Obd2ProvisionedIdentityRecord,
    ) {
        self.set_provisioned_obd2_identity(record.into_provisioned_identity());
    }

    #[cfg(feature = "transport-can")]
    pub const fn obd2_identity_key_lifecycle_status(
        &self,
    ) -> Option<ecu_transport::CanObd2IdentityKeyLifecycleStatus> {
        self.obd2_identity_key_lifecycle_status
    }

    #[cfg(feature = "transport-can")]
    pub fn set_obd2_identity_key_lifecycle_status(
        &mut self,
        status: Option<ecu_transport::CanObd2IdentityKeyLifecycleStatus>,
    ) {
        self.obd2_identity_key_lifecycle_status = status;
    }

    #[cfg(feature = "transport-can")]
    pub const fn obd2_flash_write_fault_status(
        &self,
    ) -> Option<ecu_transport::CanObd2FlashWriteFaultStatus> {
        self.obd2_flash_write_fault_status
    }

    #[cfg(feature = "transport-can")]
    pub fn set_obd2_flash_write_fault_status(
        &mut self,
        status: Option<ecu_transport::CanObd2FlashWriteFaultStatus>,
    ) {
        self.obd2_flash_write_fault_status = status;
    }

    pub fn engine_time_telemetry(&self) -> EngineTimeAuthorityTelemetry {
        EngineTimeAuthorityTelemetry::new(self.runtime.engine_time_authority())
    }

    pub fn action_telemetry(&self) -> CommonActionTelemetry {
        self.action_telemetry
    }

    pub fn fault_transition_telemetry(&self) -> CommonFaultTransitionTelemetry {
        self.fault_transition
    }

    pub fn diag_log(&self) -> &DiagLog<DIAG_LOG_ENTRY_COUNT> {
        &self.diag_log
    }

    #[cfg(feature = "transport-can")]
    pub fn obd2_retained_history_snapshot(
        &self,
    ) -> crate::transport_service::Obd2RetainedDiagnosticHistorySnapshot {
        self.retained_obd2_history.snapshot()
    }

    #[cfg(feature = "transport-can")]
    pub fn restore_obd2_retained_history(
        &mut self,
        snapshot: &crate::transport_service::Obd2RetainedDiagnosticHistorySnapshot,
    ) {
        self.retained_obd2_history.restore_from_snapshot(snapshot);
    }

    pub fn push_live_diag_event(&mut self, event: DiagEvent) {
        self.push_shared_diag_log_event(event);
    }

    pub fn record_store_integrity_status(&mut self, status: StoreIntegrityStatus) {
        if self.store_integrity_latched {
            return;
        }
        if matches!(
            status,
            StoreIntegrityStatus::Corrupt | StoreIntegrityStatus::ValidWithCorruptSibling
        ) {
            self.push_live_diag_event(DiagEvent {
                code: ecu_domain::diag::DiagCode::PersistCrcFault,
                timestamp: Micros::new(0),
                source: ecu_domain::diag::DiagSource::User,
                context: None,
                start_us: 0,
                end_us: 0,
            });
            self.store_integrity_latched = true;
        }
    }

    pub fn control_reason_telemetry(&self) -> CommonControlReasonTelemetry {
        self.control_reasons
    }

    pub fn lambda_telemetry(&self) -> CommonLambdaTelemetry {
        common_lambda_telemetry(self.control_reasons, self.last_lambda_inputs)
    }

    pub fn lambda_correction_telemetry(&self) -> CommonLambdaCorrectionTelemetry {
        let runtime_snapshot = self.runtime.snapshot();
        common_lambda_correction_telemetry(
            CommonControlTelemetry {
                fuel_pulse_width: runtime_snapshot.control.fuel_pulse_width,
                ignition_advance: runtime_snapshot.control.ignition_advance,
                dwell: runtime_snapshot.control.dwell,
                lambda_target: runtime_snapshot.control.lambda_target,
                torque_limit_x100: runtime_snapshot.control.torque_limit_x100,
            },
            self.control_reason_telemetry(),
            self.lambda_telemetry(),
            self.last_lambda_inputs,
        )
    }

    pub fn fuel_strategy_mode(&self) -> CommonFuelStrategyMode {
        common_fuel_strategy_mode(self.runtime.fuel_strategy())
    }

    pub fn shift_arming_telemetry(&self) -> CommonShiftArmingTelemetry {
        CommonShiftArmingTelemetry {
            launch_armed: self.pending_inputs.launch_armed,
            flat_shift_armed: self.pending_inputs.flat_shift_armed,
        }
    }

    pub fn pending_input_telemetry(&self) -> CommonPendingInputTelemetry {
        CommonPendingInputTelemetry {
            now_us: self.pending_inputs.now_us,
            rpm: Rpm::new(self.pending_inputs.rpm.min(u16::MAX as u32) as u16),
            load_kpa10: Kpa10::new(self.pending_inputs.load_kpa10.min(u16::MAX as u32) as u16),
            angle_x10: Degrees10::new(
                self.pending_inputs
                    .angle_x10
                    .clamp(i16::MIN as i32, i16::MAX as i32) as i16,
            ),
            authority: self.pending_inputs.authority,
        }
    }

    pub fn torque_telemetry(&self) -> CommonTorqueTelemetry {
        self.torque
    }

    pub fn trigger_edge_telemetry(&self) -> CommonTriggerEdgeTelemetry {
        self.trigger_edge
    }

    pub fn cam_edge_telemetry(&self) -> CommonCamEdgeTelemetry {
        self.cam_edge
    }

    pub fn signal_assembly_counters(&self) -> SignalAssemblyCounters {
        let mut counters = self.signal_assembly_counters;
        counters.merge_from(self.runtime.signal_assembly_counters());
        counters
    }

    pub fn output_assembly_counters(&self) -> OutputAssemblyCounters {
        let mut counters = self.output_assembly_counters;
        counters.merge_from(self.runtime.output_assembly_counters());
        counters
    }

    pub fn fuel_observation_telemetry(&self) -> CommonFuelObservationTelemetry {
        self.fuel
    }

    pub fn enrichment_telemetry(&self) -> CommonEnrichmentTelemetry {
        self.enrichment
    }

    pub fn warmup_telemetry(&self) -> CommonWarmupTelemetry {
        self.warmup
    }

    pub fn startup_telemetry(&self) -> CommonStartupTelemetry {
        self.startup
    }

    pub fn afterstart_telemetry(&self) -> CommonAfterstartTelemetry {
        self.afterstart
    }

    pub fn transient_enrichment_telemetry(&self) -> CommonTransientEnrichmentTelemetry {
        self.transient_enrichment
    }

    pub fn validated_input_telemetry(&self) -> CommonValidatedInputTelemetry {
        self.validated
    }

    pub fn capture_sample(&self) -> Option<CaptureSample> {
        self.capture_sample
    }

    pub fn logical_sensor_capture(&self) -> Option<BoardSensorSnapshotCapture> {
        self.logical_sensor_capture
    }

    pub fn logical_sensor_snapshot(&self) -> Option<BoardSensorSnapshot> {
        self.logical_sensor_capture()
            .map(|capture| capture.snapshot)
    }

    pub fn clear_diagnostics(&mut self) -> ecu_domain::diag::DiagClearSummary {
        let summary = ecu_domain::diag::DiagClearSummary {
            cleared_active_count: u8::from(
                self.runtime.snapshot().faults.fault != ecu_domain::FaultCode::None,
            ),
            cleared_log_entries: self
                .diag_log
                .events
                .iter()
                .filter(|entry| entry.is_some())
                .count() as u8,
            ..ecu_domain::diag::DiagClearSummary::default()
        };
        self.runtime.set_fault_state(
            ecu_domain::FaultCode::None,
            ecu_domain::FaultSeverity::Info,
            ecu_domain::CancelReason::Manual,
        );
        self.diag_log = DiagLog::new();
        self.store_integrity_latched = false;
        #[cfg(feature = "transport-can")]
        self.retained_obd2_history.clear();
        self.reset_shared_live_diag_state();
        summary
    }

    #[cfg(feature = "transport-can")]
    fn reset_shared_live_diag_state(&mut self) {
        self.shared_live_diag_state = SharedLiveDiagState::default();
    }

    #[cfg(not(feature = "transport-can"))]
    fn reset_shared_live_diag_state(&mut self) {}

    #[cfg(feature = "transport-can")]
    fn current_obd2_value_source(&self) -> ecu_transport::Message {
        crate::transport_service::obd2_current_data_from_board_inputs(
            self.logical_sensor_capture(),
            self.capture_sample(),
        )
    }

    #[cfg(feature = "transport-can")]
    fn current_obd2_diag_event_from_live_fault(&self) -> Option<DiagEvent> {
        let current_data_value_source = self.current_obd2_value_source();
        crate::transport_service::obd2_diag_event_from_runtime_fault(
            self.fault_state(),
            &current_data_value_source,
        )
    }

    #[cfg(feature = "transport-can")]
    fn current_obd2_recent_diag_events(
        &self,
    ) -> [Option<DiagEvent>; crate::transport_service::OBD2_LIVE_DIAG_EVENT_INGRESS_CAP] {
        let current_data_value_source = self.current_obd2_value_source();
        [
            crate::transport_service::obd2_diag_event_from_fault_transition(
                self.fault_transition_telemetry(),
                &current_data_value_source,
            ),
            None,
        ]
    }

    #[cfg(feature = "transport-can")]
    fn reseed_obd2_retained_history_from_live_state(&mut self) {
        let current_data_value_source = self.current_obd2_value_source();
        self.retained_obd2_history.observe_live_diagnostic(
            self.fault_state(),
            self.current_obd2_diag_event_from_live_fault(),
            crate::transport_service::obd2_diag_log_events_array(self.diag_log()),
            self.current_obd2_recent_diag_events(),
            current_data_value_source,
        );
    }

    #[cfg(not(feature = "transport-can"))]
    fn reseed_obd2_retained_history_from_live_state(&mut self) {}

    fn push_shared_diag_log_event(&mut self, event: DiagEvent) {
        self.diag_log.push(event);
        self.reseed_obd2_retained_history_from_live_state();
    }

    #[cfg(feature = "transport-can")]
    fn observe_shared_sensor_recovery_sources(
        &mut self,
        now_us: Micros,
        snapshot: BoardSensorSnapshot,
    ) {
        let now_us_raw = now_us.get();
        let lim = self.sensor_limits;
        let raw_map_kpa_x10 = snapshot.map_kpa10.get();
        let raw_tps_percent = (snapshot.tps_x100 / 100).min(u8::MAX as u16) as u8;
        let clear_time_us = u32::from(lim.clear_time_s) * 1_000_000;

        let map_oob =
            raw_map_kpa_x10 < lim.map_min_kpa_x10 || raw_map_kpa_x10 > lim.map_max_kpa_x10;
        if map_oob {
            if !self.shared_live_diag_state.map_range.is_active() {
                self.shared_live_diag_state
                    .map_range
                    .latch(Micros::new(now_us_raw));
            }
        } else if self.shared_live_diag_state.map_range.is_active() {
            if self.shared_live_diag_state.map_range.in_range_since_us == 0 {
                self.shared_live_diag_state.map_range.in_range_since_us = now_us_raw;
            }
            if now_us_raw.wrapping_sub(self.shared_live_diag_state.map_range.in_range_since_us)
                >= clear_time_us
            {
                let dur = now_us_raw.wrapping_sub(self.shared_live_diag_state.map_range.start_us);
                self.shared_live_diag_state.map_range.total_us = self
                    .shared_live_diag_state
                    .map_range
                    .total_us
                    .saturating_add(dur);
                let start_us = self.shared_live_diag_state.map_range.start_us;
                self.push_shared_diag_log_event(DiagEvent {
                    code: DiagCode::MapRange,
                    timestamp: now_us,
                    source: DiagSource::Sensor,
                    context: Some(u32::from(raw_map_kpa_x10)),
                    start_us,
                    end_us: now_us_raw,
                });
                self.shared_live_diag_state
                    .map_range
                    .clear(Micros::new(now_us_raw));
                self.shared_live_diag_state.map_range = SharedDiagLatchState::default();
            }
        }

        let tps_oob =
            raw_tps_percent < lim.tps_min_percent || raw_tps_percent > lim.tps_max_percent;
        if tps_oob {
            if !self.shared_live_diag_state.tps_range.is_active() {
                self.shared_live_diag_state
                    .tps_range
                    .latch(Micros::new(now_us_raw));
            }
        } else if self.shared_live_diag_state.tps_range.is_active() {
            if self.shared_live_diag_state.tps_range.in_range_since_us == 0 {
                self.shared_live_diag_state.tps_range.in_range_since_us = now_us_raw;
            }
            if now_us_raw.wrapping_sub(self.shared_live_diag_state.tps_range.in_range_since_us)
                >= clear_time_us
            {
                let dur = now_us_raw.wrapping_sub(self.shared_live_diag_state.tps_range.start_us);
                self.shared_live_diag_state.tps_range.total_us = self
                    .shared_live_diag_state
                    .tps_range
                    .total_us
                    .saturating_add(dur);
                let start_us = self.shared_live_diag_state.tps_range.start_us;
                self.push_shared_diag_log_event(DiagEvent {
                    code: DiagCode::TpsRange,
                    timestamp: now_us,
                    source: DiagSource::Sensor,
                    context: Some(u32::from(raw_tps_percent)),
                    start_us,
                    end_us: now_us_raw,
                });
                self.shared_live_diag_state
                    .tps_range
                    .clear(Micros::new(now_us_raw));
                self.shared_live_diag_state.tps_range = SharedDiagLatchState::default();
            }
        }
    }

    #[cfg(feature = "transport-can")]
    fn observe_shared_live_diag_sources(&mut self, now_us: Micros) {
        let Some(capture) = self.logical_sensor_capture() else {
            return;
        };

        let snapshot = capture.snapshot;
        self.observe_shared_sensor_recovery_sources(now_us, snapshot);
        let low_voltage_active = snapshot.vbatt_mv > 0 && snapshot.vbatt_mv < BROWNOUT_CRITICAL_MV;
        if low_voltage_active && !self.shared_live_diag_state.low_voltage_active {
            self.push_shared_diag_log_event(shared_live_diag_event(
                DiagCode::LowVoltage,
                now_us,
                DiagSource::Sensor,
                Some(u32::from(snapshot.vbatt_mv)),
            ));
        }
        self.shared_live_diag_state.low_voltage_active = low_voltage_active;

        let overvoltage_active = snapshot.vbatt_mv > OVERVOLTAGE_MV;
        if overvoltage_active && !self.shared_live_diag_state.overvoltage_active {
            self.push_shared_diag_log_event(shared_live_diag_event(
                DiagCode::Overvoltage,
                now_us,
                DiagSource::Sensor,
                Some(u32::from(snapshot.vbatt_mv)),
            ));
        }
        self.shared_live_diag_state.overvoltage_active = overvoltage_active;

        let knock_active = snapshot.validity.contains(BoardSensorValidityFlags::KNOCK)
            && shared_live_knock_threshold_x100(self.runtime.fuel_strategy())
                .map(|threshold| snapshot.knock_x100.get() >= threshold)
                .unwrap_or(false);
        if knock_active && !self.shared_live_diag_state.knock_active {
            self.push_shared_diag_log_event(shared_live_diag_event(
                DiagCode::KnockDetected,
                now_us,
                DiagSource::Sensor,
                Some(u32::from(snapshot.knock_x100.get())),
            ));
        }
        self.shared_live_diag_state.knock_active = knock_active;

        let lambda_invalid_active = snapshot.lambda_x100.get() > 0
            && !snapshot.validity.contains(BoardSensorValidityFlags::LAMBDA);
        if lambda_invalid_active && !self.shared_live_diag_state.lambda_invalid_active {
            self.push_shared_diag_log_event(shared_live_diag_event(
                DiagCode::LambdaInvalid,
                now_us,
                DiagSource::Sensor,
                Some(u32::from(snapshot.lambda_x100.get())),
            ));
        }
        self.shared_live_diag_state.lambda_invalid_active = lambda_invalid_active;
    }

    #[cfg(feature = "transport-can")]
    fn observe_shared_pressure_diag_sources(
        &mut self,
        now_us: Micros,
        oil_pressure_kpa10: Kpa10,
        fuel_pressure_kpa10: Kpa10,
        oil_valid: bool,
        fuel_valid: bool,
    ) {
        let oil_pressure_low_active = oil_valid && oil_pressure_kpa10 < OIL_PRESSURE_MIN_KPA10;
        if oil_pressure_low_active && !self.shared_live_diag_state.oil_pressure_low_active {
            self.push_shared_diag_log_event(shared_live_diag_event(
                DiagCode::OilPressureLow,
                now_us,
                DiagSource::Sensor,
                Some(u32::from(oil_pressure_kpa10.get())),
            ));
        }
        self.shared_live_diag_state.oil_pressure_low_active = oil_pressure_low_active;

        let fuel_pressure_low_active = fuel_valid && fuel_pressure_kpa10 < FUEL_PRESSURE_MIN_KPA10;
        if fuel_pressure_low_active && !self.shared_live_diag_state.fuel_pressure_low_active {
            self.push_shared_diag_log_event(shared_live_diag_event(
                DiagCode::FuelPressureLow,
                now_us,
                DiagSource::Sensor,
                Some(u32::from(fuel_pressure_kpa10.get())),
            ));
        }
        self.shared_live_diag_state.fuel_pressure_low_active = fuel_pressure_low_active;
    }

    #[cfg(not(feature = "transport-can"))]
    fn observe_shared_live_diag_sources(&mut self, _now_us: Micros) {}

    #[cfg(feature = "transport-can")]
    fn push_fault_transition_live_diag_event(&mut self) {
        let current_data_value_source =
            crate::transport_service::obd2_current_data_from_board_inputs(
                self.logical_sensor_capture(),
                self.capture_sample(),
            );
        if let Some(diag_event) = crate::transport_service::obd2_diag_event_from_fault_transition(
            self.fault_transition,
            &current_data_value_source,
        ) {
            self.push_shared_diag_log_event(diag_event);
        }
    }

    #[cfg(not(feature = "transport-can"))]
    fn push_fault_transition_live_diag_event(&mut self) {}

    pub fn configure_fuel_model(&mut self, fuel_model: ecu_runtime::BaseFuelModel) {
        self.runtime.configure_fuel_model(fuel_model);
    }

    pub fn configure_speed_density_semantic(
        &mut self,
        calibration: RuntimeSemanticCalibration,
        state: RuntimeSemanticState,
    ) {
        self.runtime.configure_speed_density_ve(calibration, state);
    }

    pub fn configure_runtime_fuel_strategy(&mut self, strategy: RuntimeFuelStrategy) {
        self.runtime.configure_runtime_fuel_model(strategy);
    }

    pub fn set_staged_dirty(&mut self, dirty: bool) {
        self.staged_dirty = dirty;
        self.runtime.set_staged_dirty(dirty);
    }

    pub fn set_shift_arming(&mut self, launch_armed: bool, flat_shift_armed: bool) {
        self.pending_inputs.launch_armed = launch_armed;
        self.pending_inputs.flat_shift_armed = flat_shift_armed;
    }

    fn record_signal_assembly_stage(&mut self, stage: SignalStage, timestamp: Micros) {
        self.signal_assembly_counters.record(
            stage,
            StageOutcome::Accepted,
            TraceId::default(),
            0,
            timestamp,
            0,
        );
    }

    fn record_output_assembly_stage(
        &mut self,
        stage: OutputStage,
        outcome: StageOutcome,
        command_id: u32,
        timestamp: Micros,
    ) {
        self.output_assembly_counters.record(
            stage,
            outcome,
            TraceId::default(),
            command_id,
            timestamp,
            0,
        );
    }

    pub fn sensor(&mut self) -> &mut S {
        &mut self.sensor
    }

    pub fn capture(&mut self) -> &mut C {
        &mut self.capture
    }

    pub fn actions(&mut self) -> &mut A {
        &mut self.actions
    }

    pub fn watchdog(&mut self) -> &mut W {
        &mut self.watchdog
    }

    pub fn transport(&mut self) -> &mut T {
        &mut self.transport
    }

    pub fn store(&mut self) -> &mut P {
        &mut self.store
    }

    #[allow(clippy::type_complexity)]
    pub fn poll_sensor(
        &mut self,
    ) -> AdapterResult<S::Error, C::Error, A::Error, W::Error, T::Error, P::Error, CaptureSample>
    {
        let sample = self.sensor.sample().map_err(BoardAdapterError::Sensor)?;
        self.pending_inputs.now_us = sample.at_us;
        self.pending_inputs.rpm = u32::from(sample.rpm.get());
        self.pending_inputs.load_kpa10 = u32::from(sample.load_kpa10.get());
        self.pending_inputs.angle_x10 = i32::from(sample.angle_x10.get());
        self.capture_sample = Some(sample);
        self.capture
            .capture(sample)
            .map_err(BoardAdapterError::Capture)?;
        Ok(sample)
    }

    #[allow(clippy::type_complexity)]
    pub fn apply_event(
        &mut self,
        event: BoardEvent,
    ) -> AdapterResult<S::Error, C::Error, A::Error, W::Error, T::Error, P::Error, Option<StepResult>>
    {
        match event {
            BoardEvent::TriggerEdge {
                at_us,
                rpm,
                angle_x10,
                authority,
                synced,
            } => {
                self.pending_inputs.now_us = at_us;
                self.pending_inputs.rpm = u32::from(rpm.get());
                self.pending_inputs.angle_x10 = i32::from(angle_x10.get());
                self.pending_inputs.authority = authority;
                self.trigger_edge = CommonTriggerEdgeTelemetry {
                    seen: true,
                    at_us,
                    rpm,
                    angle_x10,
                    authority,
                    synced,
                };
                let sample = CaptureSample {
                    at_us,
                    rpm,
                    load_kpa10: self.runtime.snapshot().engine.load_kpa10,
                    angle_x10,
                };
                self.capture_sample = Some(sample);
                self.capture
                    .capture(sample)
                    .map_err(BoardAdapterError::Capture)?;
                self.record_signal_assembly_stage(SignalStage::SignalCapture, at_us);
                self.runtime.apply_sensor_sample(
                    rpm,
                    self.runtime.snapshot().engine.load_kpa10,
                    angle_x10,
                );
                let _ = synced;
                self.runtime.set_engine_time_authority(authority);
                Ok(None)
            }
            BoardEvent::CamEdge { at_us, cam_seen } => {
                self.pending_inputs.now_us = at_us;
                self.cam_edge = CommonCamEdgeTelemetry {
                    seen: true,
                    at_us,
                    cam_seen,
                };
                self.runtime
                    .apply_decoder_observation(DecoderObservation::Cam(
                        ecu_runtime::CamObservation { at_us, cam_seen },
                    ));
                self.pending_inputs.authority = self.runtime.engine_time_authority();
                Ok(None)
            }
            BoardEvent::SensorSnapshotCapture { capture } => {
                let sample = map_speed_density_capture_sample(capture);
                self.capture_sample = Some(sample);
                self.logical_sensor_capture = Some(capture);
                self.pending_inputs.now_us = sample.at_us;
                self.pending_inputs.rpm = u32::from(sample.rpm.get());
                self.pending_inputs.load_kpa10 = u32::from(sample.load_kpa10.get());
                self.pending_inputs.angle_x10 = i32::from(sample.angle_x10.get());
                self.runtime
                    .apply_sensor_sample(sample.rpm, sample.load_kpa10, sample.angle_x10);
                self.capture
                    .capture(sample)
                    .map_err(BoardAdapterError::Capture)?;
                self.record_signal_assembly_stage(SignalStage::SignalCapture, sample.at_us);
                Ok(None)
            }
            BoardEvent::PressureSnapshot {
                now_us,
                oil_pressure_kpa10,
                fuel_pressure_kpa10,
                oil_valid,
                fuel_valid,
            } => {
                self.pending_inputs.now_us = now_us;
                #[cfg(feature = "transport-can")]
                self.observe_shared_pressure_diag_sources(
                    now_us,
                    oil_pressure_kpa10,
                    fuel_pressure_kpa10,
                    oil_valid,
                    fuel_valid,
                );
                let _ = (
                    oil_pressure_kpa10,
                    fuel_pressure_kpa10,
                    oil_valid,
                    fuel_valid,
                );
                Ok(None)
            }
            BoardEvent::Tick { now_us, control } => {
                self.pending_inputs.now_us = now_us;
                let pre_step_fault_state = self.runtime.snapshot().faults;
                let result = self
                    .runtime
                    .step_with_authority(self.pending_inputs, control);
                self.execute_step(&result)?;
                self.observe_shared_live_diag_sources(now_us);
                let post_step_fault_state = self.runtime.snapshot().faults;
                let cached_fault_state = if self.fault_transition.changed {
                    ecu_runtime::FaultState {
                        fault: self.fault_transition.current_fault,
                        severity: self.fault_transition.current_severity,
                        cancel_reason: self.fault_transition.current_cancel_reason,
                    }
                } else {
                    ecu_runtime::FaultState::default()
                };
                if post_step_fault_state != pre_step_fault_state
                    || post_step_fault_state != cached_fault_state
                {
                    let previous_fault_state = if post_step_fault_state != pre_step_fault_state {
                        pre_step_fault_state
                    } else {
                        cached_fault_state
                    };
                    self.fault_transition = CommonFaultTransitionTelemetry {
                        changed: true,
                        at_us: now_us,
                        event: common_fault_transition_event(
                            previous_fault_state,
                            post_step_fault_state,
                        ),
                        previous_fault: previous_fault_state.fault,
                        previous_severity: previous_fault_state.severity,
                        previous_cancel_reason: previous_fault_state.cancel_reason,
                        current_fault: post_step_fault_state.fault,
                        current_severity: post_step_fault_state.severity,
                        current_cancel_reason: post_step_fault_state.cancel_reason,
                    };
                    self.push_fault_transition_live_diag_event();
                }
                self.action_telemetry = common_action_telemetry(&result);
                self.control_reasons = CommonControlReasonTelemetry {
                    lambda_mode: common_lambda_mode(result.control.lambda.mode),
                    lambda_active: result.control.lambda.active,
                    lambda_trim_x100: result.control.lambda.trim_x100,
                    lambda_disable_reason: common_lambda_disable_reason(
                        result.control.lambda.disable_reason,
                    ),
                    ignition_limit_reason: common_ignition_limit_reason(
                        result.control.ignition.limit_reason,
                    ),
                    torque_limit_reason: common_torque_limit_reason(result.control.torque.reason),
                };
                self.last_lambda_inputs = LastLambdaInputs {
                    seen: true,
                    lambda_valid: control.lambda.lambda_valid,
                    measured_lambda100: control.lambda.measured_lambda100,
                    requested_open_loop: control.lambda.requested_open_loop,
                };
                self.validated = CommonValidatedInputTelemetry {
                    rpm: result.validated.rpm,
                    load_kpa10: result.validated.load_kpa10,
                    angle_x10: result.validated.angle_x10,
                    clamped: result.validated.clamped,
                };
                self.fuel = CommonFuelObservationTelemetry {
                    base_fuel_pulse_width: result.control.base_fuel,
                    enriched_fuel_pulse_width: result.control.enriched_fuel,
                };
                self.enrichment = CommonEnrichmentTelemetry {
                    startup_x100: result.control.enrichment.startup_x100,
                    warmup_x100: result
                        .control
                        .fuel_intent
                        .observations
                        .warmup_correction_x100,
                    after_start_x100: result.control.enrichment.after_start_x100,
                    acceleration_x100: result.control.enrichment.acceleration_x100,
                    total_x100: result.control.enrichment.total_x100(),
                };
                self.warmup = CommonWarmupTelemetry {
                    active: result.control.fuel_intent.observations.warmup_active,
                    correction_x100: result
                        .control
                        .fuel_intent
                        .observations
                        .warmup_correction_x100,
                    temperature_mode: match result
                        .control
                        .fuel_intent
                        .observations
                        .warmup_temperature_mode
                    {
                        ecu_runtime::FuelWarmupTemperatureMode::Inactive => {
                            CommonWarmupTemperatureMode::Inactive
                        }
                        ecu_runtime::FuelWarmupTemperatureMode::ColdClamp => {
                            CommonWarmupTemperatureMode::ColdClamp
                        }
                        ecu_runtime::FuelWarmupTemperatureMode::Interpolating => {
                            CommonWarmupTemperatureMode::Interpolating
                        }
                        ecu_runtime::FuelWarmupTemperatureMode::HotClamp => {
                            CommonWarmupTemperatureMode::HotClamp
                        }
                        ecu_runtime::FuelWarmupTemperatureMode::NeutralFallback => {
                            CommonWarmupTemperatureMode::NeutralFallback
                        }
                    },
                };
                self.startup = CommonStartupTelemetry {
                    active: result.control.fuel_intent.observations.startup_active,
                    remaining_window: result
                        .control
                        .fuel_intent
                        .observations
                        .startup_window_remaining,
                    window_mode: match result.control.fuel_intent.observations.startup_window_mode {
                        ecu_runtime::FuelStartupWindowMode::Inactive => {
                            CommonStartupWindowMode::Inactive
                        }
                        ecu_runtime::FuelStartupWindowMode::Milliseconds => {
                            CommonStartupWindowMode::Milliseconds
                        }
                    },
                };
                self.afterstart = CommonAfterstartTelemetry {
                    active: result.control.fuel_intent.observations.afterstart_active,
                    remaining_window: result
                        .control
                        .fuel_intent
                        .observations
                        .afterstart_window_remaining,
                    window_mode: match result
                        .control
                        .fuel_intent
                        .observations
                        .afterstart_window_mode
                    {
                        ecu_runtime::FuelAfterstartWindowMode::Inactive => {
                            CommonAfterstartWindowMode::Inactive
                        }
                        ecu_runtime::FuelAfterstartWindowMode::Milliseconds => {
                            CommonAfterstartWindowMode::Milliseconds
                        }
                        ecu_runtime::FuelAfterstartWindowMode::Cycles => {
                            CommonAfterstartWindowMode::Cycles
                        }
                    },
                };
                self.transient_enrichment = CommonTransientEnrichmentTelemetry {
                    acceleration_active: result
                        .control
                        .fuel_intent
                        .observations
                        .lambda_ae_freeze_active,
                    acceleration_pulse_us: result.control.fuel_intent.observations.ae_pulse_us,
                    acceleration_decay_steps_remaining: result
                        .control
                        .fuel_intent
                        .observations
                        .ae_decay_steps_remaining,
                };
                self.torque = CommonTorqueTelemetry {
                    request_x1000: result.torque_observations.request_x1000,
                    allowed_x1000: result.torque_observations.allowed_x1000,
                    actuated_x1000: result.torque_observations.actuated_x1000,
                };
                Ok(Some(result))
            }
        }
    }

    #[allow(clippy::type_complexity)]
    pub fn execute_step(
        &mut self,
        result: &StepResult,
    ) -> AdapterResult<S::Error, C::Error, A::Error, W::Error, T::Error, P::Error, ()> {
        let mut output_command_id = 0;
        for action in result.actions.iter() {
            match action {
                Action::PublishSnapshot => self
                    .transport
                    .publish_snapshot(&self.runtime.snapshot())
                    .map_err(BoardAdapterError::Transport)?,
                Action::PersistCalibration => self
                    .store
                    .save(&PersistedCalibrationBlob::new(
                        self.runtime.calibration_snapshot(),
                    ))
                    .map_err(BoardAdapterError::Persistence)?,
                Action::Idle => {}
                other => {
                    let output_count = common_action_output_count(other);
                    let at_us = self.pending_inputs.now_us;
                    self.actions
                        .execute(other)
                        .map_err(BoardAdapterError::Action)?;
                    for _ in 0..output_count {
                        output_command_id += 1;
                        self.record_output_assembly_stage(
                            OutputStage::OutputArmer,
                            StageOutcome::Armed,
                            output_command_id,
                            at_us,
                        );
                    }
                }
            }
        }

        self.watchdog.feed().map_err(BoardAdapterError::Watchdog)?;
        Ok(())
    }
}

#[cfg(feature = "transport-can")]
impl<S, C, A, W, T, P> crate::transport_service::Obd2RetainedHistoryOwner
    for BoardAdapter<S, C, A, W, T, P>
where
    S: CaptureSampleSource,
    C: CaptureSink,
    A: ActionExecutor,
    W: Watchdog,
    T: TransportPublisher,
    P: PersistedCalibrationStore,
{
    fn obd2_retained_history_snapshot(
        &self,
    ) -> crate::transport_service::Obd2RetainedDiagnosticHistorySnapshot {
        BoardAdapter::obd2_retained_history_snapshot(self)
    }

    fn restore_obd2_retained_history(
        &mut self,
        snapshot: &crate::transport_service::Obd2RetainedDiagnosticHistorySnapshot,
    ) {
        BoardAdapter::restore_obd2_retained_history(self, snapshot);
    }
}

#[cfg(feature = "transport-can")]
impl<S, C, A, W, T, P> crate::transport_service::DiagnosticClearOwner
    for BoardAdapter<S, C, A, W, T, P>
where
    S: CaptureSampleSource,
    C: CaptureSink,
    A: ActionExecutor,
    W: Watchdog,
    T: TransportPublisher,
    P: PersistedCalibrationStore,
{
    fn clear_diagnostics(&mut self) -> ecu_domain::diag::DiagClearSummary {
        BoardAdapter::clear_diagnostics(self)
    }
}

#[cfg(feature = "transport-can")]
impl<S, C, A, W, T, P> crate::transport_service::LiveObd2RequestOwner
    for BoardAdapter<S, C, A, W, T, P>
where
    S: CaptureSampleSource,
    C: CaptureSink,
    A: ActionExecutor,
    W: Watchdog,
    T: TransportPublisher,
    P: PersistedCalibrationStore,
{
    fn fault_state(&self) -> ecu_runtime::FaultState {
        self.fault_state()
    }

    fn current_obd2_sensor_data(&self) -> ecu_transport::Message {
        crate::transport_service::obd2_current_data_from_board_inputs(
            self.logical_sensor_capture(),
            self.capture_sample(),
        )
    }

    fn obd2_retained_history(&self) -> &crate::transport_service::Obd2RetainedDiagnosticHistory {
        &self.retained_obd2_history
    }

    fn obd2_retained_history_mut(
        &mut self,
    ) -> &mut crate::transport_service::Obd2RetainedDiagnosticHistory {
        &mut self.retained_obd2_history
    }

    fn current_obd2_diag_event(&self) -> Option<ecu_domain::diag::DiagEvent> {
        let current_data_value_source =
            crate::transport_service::obd2_current_data_from_board_inputs(
                self.logical_sensor_capture(),
                self.capture_sample(),
            );
        crate::transport_service::obd2_diag_event_from_runtime_fault(
            self.fault_state(),
            &current_data_value_source,
        )
    }

    fn obd2_diag_log_events(&self) -> [Option<ecu_domain::diag::DiagEvent>; DIAG_LOG_ENTRY_COUNT] {
        crate::transport_service::obd2_diag_log_events_array(self.diag_log())
    }

    fn recent_obd2_diag_events(
        &self,
    ) -> [Option<ecu_domain::diag::DiagEvent>;
           crate::transport_service::OBD2_LIVE_DIAG_EVENT_INGRESS_CAP] {
        let current_data_value_source =
            crate::transport_service::obd2_current_data_from_board_inputs(
                self.logical_sensor_capture(),
                self.capture_sample(),
            );
        [
            crate::transport_service::obd2_diag_event_from_fault_transition(
                self.fault_transition_telemetry(),
                &current_data_value_source,
            ),
            None,
        ]
    }

    fn fallback_obd2_vehicle_identity(&self) -> crate::transport_service::Obd2VehicleIdentity {
        crate::transport_service::Obd2VehicleIdentity::from_calibration_identity(
            self.calibration_package_identity(),
        )
    }

    fn provisioned_obd2_identity(&self) -> crate::transport_service::ProvisionedObd2Identity {
        self.provisioned_obd2_identity
    }

    fn obd2_identity_key_lifecycle_status(
        &self,
    ) -> Option<ecu_transport::CanObd2IdentityKeyLifecycleStatus> {
        self.obd2_identity_key_lifecycle_status
    }

    fn obd2_flash_write_fault_status(&self) -> Option<ecu_transport::CanObd2FlashWriteFaultStatus> {
        self.obd2_flash_write_fault_status
    }
}

impl<const N: usize> SchedulerObservabilitySource for ScheduledActionExecutor<N> {
    fn timing_metrics(&self) -> ScheduledTimingMetrics {
        self.timing_metrics()
    }

    fn active_queue_count(&self) -> u8 {
        self.queue().active_count().min(u8::MAX as usize) as u8
    }

    fn free_queue_slots(&self) -> u8 {
        self.queue().free_slots().min(u8::MAX as usize) as u8
    }

    fn queue_capacity(&self) -> u8 {
        self.queue().capacity().min(u8::MAX as usize) as u8
    }

    fn frontier_telemetry(&self) -> CommonFrontierTelemetry {
        let frontier = self.frontier();
        CommonFrontierTelemetry {
            active_horizon_id: frontier.active_horizon_id(),
            horizon_start_us: frontier.horizon_start_us(),
            horizon_end_us: frontier.horizon_end_us(),
            last_accepted_horizon_id: frontier.last_accepted_horizon_id(),
            last_accepted_horizon_start_us: frontier.last_accepted_horizon_start_us(),
            last_accepted_horizon_end_us: frontier.last_accepted_horizon_end_us(),
            heartbeat_deadline_us: frontier.heartbeat_deadline_us(),
            active_permit_mask: frontier.active_permit_mask(),
            active_stop_reason: frontier.active_stop_reason(),
            fault: common_frontier_fault_telemetry(frontier.active_stop_reason()),
        }
    }

    fn scheduler_ownership_telemetry(&self) -> CommonSchedulerOwnershipTelemetry {
        let frontier = self.frontier();
        CommonSchedulerOwnershipTelemetry {
            mode: common_scheduler_mode(frontier.mode()),
            active_groups: frontier.active_groups(),
            injection_count: frontier.injection_count(),
            ignition_count: frontier.ignition_count(),
        }
    }

    fn scheduler_reservation_telemetry(&self) -> CommonSchedulerReservationTelemetry {
        let reserved_channels = self.frontier().reserved_channels();
        CommonSchedulerReservationTelemetry {
            injector_channels: reserved_channels[0],
            ignition_channels: reserved_channels[1],
            idle_channels: reserved_channels[2],
            fan_channels: reserved_channels[3],
        }
    }

    fn scheduler_state_summary_telemetry(&self) -> CommonSchedulerStateSummaryTelemetry {
        CommonSchedulerStateSummaryTelemetry {
            armed: self.frontier().is_armed(),
        }
    }

    fn scheduler_window_telemetry(&self) -> CommonSchedulerWindowTelemetry {
        let frontier = self.frontier();
        CommonSchedulerWindowTelemetry {
            last_injection_start: frontier.last_injection_start(),
            last_injection_end: frontier.last_injection_end(),
            last_ignition_start: frontier.last_ignition_start(),
            last_ignition_end: frontier.last_ignition_end(),
        }
    }
}

impl<S, C, A, W, T, P> BoardAdapter<S, C, A, W, T, P>
where
    S: CaptureSampleSource,
    C: CaptureSink,
    A: ActionExecutor + SchedulerObservabilitySource,
    W: Watchdog,
    T: TransportPublisher,
    P: PersistedCalibrationStore,
{
    pub fn diagnostics_telemetry(&self) -> CommonDiagnosticsTelemetry {
        let timing_metrics = self.actions.timing_metrics();
        let fault_state = self.fault_state();
        let runtime_snapshot = self.runtime.snapshot();
        let actions = self.action_telemetry;
        let fault = common_runtime_fault_telemetry(fault_state);
        let lambda = self.lambda_telemetry();
        let lambda_correction = self.lambda_correction_telemetry();
        let warmup = self.warmup_telemetry();
        let startup = self.startup_telemetry();
        let afterstart = self.afterstart_telemetry();
        let transient_enrichment = self.transient_enrichment_telemetry();
        let decision = CommonDecisionTelemetry {
            control_mode: runtime_snapshot.engine.mode,
            rev_soft_active: runtime_snapshot.rev_soft_active,
            rev_hard_active: runtime_snapshot.rev_hard_active,
            launch_active: runtime_snapshot.launch_active,
            flat_shift_active: runtime_snapshot.flat_shift_active,
            fuel_cut: runtime_snapshot.fuel_cut,
            spark_cut: runtime_snapshot.spark_cut,
            fuel_cut_reason: CommonCutReason::None,
            spark_cut_reason: CommonCutReason::None,
        };
        let decision = CommonDecisionTelemetry {
            fuel_cut_reason: common_fuel_cut_reason(&runtime_snapshot),
            spark_cut_reason: common_spark_cut_reason(&runtime_snapshot),
            ..decision
        };
        let protection =
            common_protection_telemetry(decision, fault, self.actions.frontier_telemetry().fault);
        let limp_action = common_limp_action_telemetry(
            actions,
            protection,
            self.actions.frontier_telemetry().fault,
        );
        CommonDiagnosticsTelemetry {
            sync_state: common_sync_telemetry_state(self.sync_state()),
            fault_code: fault_state.fault,
            fault_severity: fault_state.severity,
            cancel_reason: fault_state.cancel_reason,
            fault,
            lambda,
            lambda_correction,
            warmup,
            startup,
            afterstart,
            transient_enrichment,
            protection,
            limp_action,
            late_event_count: timing_metrics.late_event_count,
            max_lateness_us: timing_metrics
                .max_lateness_us
                .map_or(0, ecu_domain::Micros::get),
            queue_high_water_mark: timing_metrics.queue_high_water_mark,
            last_drain_count: timing_metrics.last_drain_count,
            active_queue_count: self.actions.active_queue_count(),
            free_queue_slots: self.actions.free_queue_slots(),
            queue_capacity: self.actions.queue_capacity(),
        }
    }

    pub fn observability_snapshot(&self) -> CommonObservabilitySnapshot {
        let runtime_snapshot = self.runtime.snapshot();
        let diagnostics = self.diagnostics_telemetry();
        let fault = common_runtime_fault_telemetry(self.fault_state());
        let lambda = self.lambda_telemetry();
        let lambda_correction = self.lambda_correction_telemetry();
        let warmup = self.warmup_telemetry();
        let startup = self.startup_telemetry();
        let afterstart = self.afterstart_telemetry();
        let transient_enrichment = self.transient_enrichment_telemetry();
        let decision = CommonDecisionTelemetry {
            control_mode: runtime_snapshot.engine.mode,
            rev_soft_active: runtime_snapshot.rev_soft_active,
            rev_hard_active: runtime_snapshot.rev_hard_active,
            launch_active: runtime_snapshot.launch_active,
            flat_shift_active: runtime_snapshot.flat_shift_active,
            fuel_cut: runtime_snapshot.fuel_cut,
            spark_cut: runtime_snapshot.spark_cut,
            fuel_cut_reason: CommonCutReason::None,
            spark_cut_reason: CommonCutReason::None,
        };
        let decision = CommonDecisionTelemetry {
            fuel_cut_reason: common_fuel_cut_reason(&runtime_snapshot),
            spark_cut_reason: common_spark_cut_reason(&runtime_snapshot),
            ..decision
        };
        let frontier = self.actions.frontier_telemetry();
        let protection = common_protection_telemetry(decision, fault, frontier.fault);
        let limp_action =
            common_limp_action_telemetry(self.action_telemetry, protection, frontier.fault);
        CommonObservabilitySnapshot {
            diagnostics,
            fault_state: self.fault_state(),
            fault,
            lambda,
            lambda_correction,
            warmup,
            startup,
            afterstart,
            transient_enrichment,
            protection,
            limp_action,
            high_rate_log: common_high_rate_log_telemetry(
                decision,
                fault,
                frontier.fault,
                diagnostics,
                self.calibration_package_identity(),
            ),
            sync_state: self.sync_state(),
            decision,
            fuel_strategy_mode: self.fuel_strategy_mode(),
            shift_arming: self.shift_arming_telemetry(),
            pending_input: self.pending_input_telemetry(),
            actions: self.action_telemetry(),
            control: CommonControlTelemetry {
                fuel_pulse_width: runtime_snapshot.control.fuel_pulse_width,
                ignition_advance: runtime_snapshot.control.ignition_advance,
                dwell: runtime_snapshot.control.dwell,
                lambda_target: runtime_snapshot.control.lambda_target,
                torque_limit_x100: runtime_snapshot.control.torque_limit_x100,
            },
            control_reasons: self.control_reason_telemetry(),
            fuel: self.fuel_observation_telemetry(),
            enrichment: self.enrichment_telemetry(),
            torque: self.torque_telemetry(),
            trigger_edge: self.trigger_edge_telemetry(),
            cam_edge: self.cam_edge_telemetry(),
            engine: CommonEngineTelemetry {
                rpm: runtime_snapshot.engine.rpm,
                load_kpa10: runtime_snapshot.engine.load_kpa10,
                angle_x10: runtime_snapshot.engine.angle_x10,
                phase: runtime_snapshot.engine.phase,
            },
            validated: self.validated_input_telemetry(),
            frontier,
            scheduler_ownership: self.actions.scheduler_ownership_telemetry(),
            scheduler_reservations: self.actions.scheduler_reservation_telemetry(),
            scheduler_state_summary: self.actions.scheduler_state_summary_telemetry(),
            scheduler_window: self.actions.scheduler_window_telemetry(),
            engine_time: self.engine_time_telemetry(),
            fault_transition: self.fault_transition_telemetry(),
            calibration: self.calibration_package_identity(),
            capture_sample: self.capture_sample(),
            logical_sensor_capture: self.logical_sensor_capture(),
            logical_sensor_snapshot: self.logical_sensor_snapshot(),
            signal_assembly_counters: self.signal_assembly_counters(),
            output_assembly_counters: self.output_assembly_counters(),
        }
    }

    pub fn observability_sample(&self) -> CommonObservabilitySample {
        CommonObservabilitySample {
            at_us: self.pending_inputs.now_us,
            snapshot: self.observability_snapshot(),
        }
    }

    pub fn push_observability_record<const M: usize>(
        &self,
        trace: &mut FixedCommonObservabilityRecordTrace<M>,
        kind: CommonObservabilityRecordKind,
    ) -> Result<(), CommonObservabilityRecordTraceOverflow> {
        trace.push(CommonObservabilityRecord {
            kind,
            sample: self.observability_sample(),
        })
    }

    pub fn configure_fuel_model_and_record<const M: usize>(
        &mut self,
        fuel_model: ecu_runtime::BaseFuelModel,
        trace: &mut FixedCommonObservabilityRecordTrace<M>,
    ) -> Result<(), CommonObservabilityRecordTraceOverflow> {
        self.configure_fuel_model(fuel_model);
        self.push_observability_record(trace, CommonObservabilityRecordKind::FuelModelConfig)
    }
}

impl<S, C, W, T, P, const N: usize> BoardAdapter<S, C, ScheduledActionExecutor<N>, W, T, P>
where
    S: CaptureSampleSource,
    C: CaptureSink,
    W: Watchdog,
    T: TransportPublisher,
    P: PersistedCalibrationStore,
{
    pub fn set_shift_arming_and_record<const M: usize>(
        &mut self,
        launch_armed: bool,
        flat_shift_armed: bool,
        trace: &mut FixedCommonObservabilityRecordTrace<M>,
    ) -> Result<(), CommonObservabilityRecordTraceOverflow> {
        self.set_shift_arming(launch_armed, flat_shift_armed);
        trace.push(CommonObservabilityRecord {
            kind: CommonObservabilityRecordKind::ShiftArming,
            sample: self.observability_sample(),
        })
    }

    pub fn set_shift_arming_and_push_pair<const SM: usize, const RM: usize>(
        &mut self,
        launch_armed: bool,
        flat_shift_armed: bool,
        sample_trace: &mut FixedCommonObservabilityTrace<SM>,
        record_trace: &mut FixedCommonObservabilityRecordTrace<RM>,
    ) -> Result<(), PushObservabilityPairError> {
        self.set_shift_arming(launch_armed, flat_shift_armed);
        self.push_observability_pair(
            sample_trace,
            record_trace,
            CommonObservabilityRecordKind::ShiftArming,
        )
    }

    pub fn set_shift_arming_and_push_to_trace_pair<const SM: usize, const RM: usize>(
        &mut self,
        launch_armed: bool,
        flat_shift_armed: bool,
        traces: &mut FixedCommonObservabilityTracePair<SM, RM>,
    ) -> Result<(), PushObservabilityPairError> {
        let (sample_trace, record_trace) = traces.split_mut();
        self.set_shift_arming_and_push_pair(
            launch_armed,
            flat_shift_armed,
            sample_trace,
            record_trace,
        )
    }

    pub fn set_staged_dirty_and_record<const M: usize>(
        &mut self,
        dirty: bool,
        trace: &mut FixedCommonObservabilityRecordTrace<M>,
    ) -> Result<(), CommonObservabilityRecordTraceOverflow> {
        self.set_staged_dirty(dirty);
        trace.push(CommonObservabilityRecord {
            kind: CommonObservabilityRecordKind::CalibrationStagedDirty,
            sample: self.observability_sample(),
        })
    }

    pub fn set_staged_dirty_and_push_pair<const SM: usize, const RM: usize>(
        &mut self,
        dirty: bool,
        sample_trace: &mut FixedCommonObservabilityTrace<SM>,
        record_trace: &mut FixedCommonObservabilityRecordTrace<RM>,
    ) -> Result<(), PushObservabilityPairError> {
        self.set_staged_dirty(dirty);
        self.push_observability_pair(
            sample_trace,
            record_trace,
            CommonObservabilityRecordKind::CalibrationStagedDirty,
        )
    }

    pub fn set_staged_dirty_and_push_to_trace_pair<const SM: usize, const RM: usize>(
        &mut self,
        dirty: bool,
        traces: &mut FixedCommonObservabilityTracePair<SM, RM>,
    ) -> Result<(), PushObservabilityPairError> {
        let (sample_trace, record_trace) = traces.split_mut();
        self.set_staged_dirty_and_push_pair(dirty, sample_trace, record_trace)
    }

    pub fn configure_fuel_model_and_push_pair<const SM: usize, const RM: usize>(
        &mut self,
        fuel_model: ecu_runtime::BaseFuelModel,
        sample_trace: &mut FixedCommonObservabilityTrace<SM>,
        record_trace: &mut FixedCommonObservabilityRecordTrace<RM>,
    ) -> Result<(), PushObservabilityPairError> {
        self.configure_fuel_model(fuel_model);
        self.push_observability_pair(
            sample_trace,
            record_trace,
            CommonObservabilityRecordKind::FuelModelConfig,
        )
    }

    pub fn configure_fuel_model_and_push_to_trace_pair<const SM: usize, const RM: usize>(
        &mut self,
        fuel_model: ecu_runtime::BaseFuelModel,
        traces: &mut FixedCommonObservabilityTracePair<SM, RM>,
    ) -> Result<(), PushObservabilityPairError> {
        let (sample_trace, record_trace) = traces.split_mut();
        self.configure_fuel_model_and_push_pair(fuel_model, sample_trace, record_trace)
    }

    pub fn configure_speed_density_semantic_and_record<const M: usize>(
        &mut self,
        calibration: RuntimeSemanticCalibration,
        state: RuntimeSemanticState,
        trace: &mut FixedCommonObservabilityRecordTrace<M>,
    ) -> Result<(), CommonObservabilityRecordTraceOverflow> {
        self.configure_speed_density_semantic(calibration, state);
        trace.push(CommonObservabilityRecord {
            kind: CommonObservabilityRecordKind::SpeedDensitySemanticConfig,
            sample: self.observability_sample(),
        })
    }

    pub fn configure_speed_density_semantic_and_push_pair<const SM: usize, const RM: usize>(
        &mut self,
        calibration: RuntimeSemanticCalibration,
        state: RuntimeSemanticState,
        sample_trace: &mut FixedCommonObservabilityTrace<SM>,
        record_trace: &mut FixedCommonObservabilityRecordTrace<RM>,
    ) -> Result<(), PushObservabilityPairError> {
        self.configure_speed_density_semantic(calibration, state);
        self.push_observability_pair(
            sample_trace,
            record_trace,
            CommonObservabilityRecordKind::SpeedDensitySemanticConfig,
        )
    }

    pub fn configure_speed_density_semantic_and_push_to_trace_pair<
        const SM: usize,
        const RM: usize,
    >(
        &mut self,
        calibration: RuntimeSemanticCalibration,
        state: RuntimeSemanticState,
        traces: &mut FixedCommonObservabilityTracePair<SM, RM>,
    ) -> Result<(), PushObservabilityPairError> {
        let (sample_trace, record_trace) = traces.split_mut();
        self.configure_speed_density_semantic_and_push_pair(
            calibration,
            state,
            sample_trace,
            record_trace,
        )
    }

    pub fn configure_runtime_fuel_strategy_and_record<const M: usize>(
        &mut self,
        strategy: RuntimeFuelStrategy,
        trace: &mut FixedCommonObservabilityRecordTrace<M>,
    ) -> Result<(), CommonObservabilityRecordTraceOverflow> {
        self.configure_runtime_fuel_strategy(strategy);
        trace.push(CommonObservabilityRecord {
            kind: CommonObservabilityRecordKind::RuntimeFuelStrategyConfig,
            sample: self.observability_sample(),
        })
    }

    pub fn configure_runtime_fuel_strategy_and_push_pair<const SM: usize, const RM: usize>(
        &mut self,
        strategy: RuntimeFuelStrategy,
        sample_trace: &mut FixedCommonObservabilityTrace<SM>,
        record_trace: &mut FixedCommonObservabilityRecordTrace<RM>,
    ) -> Result<(), PushObservabilityPairError> {
        self.configure_runtime_fuel_strategy(strategy);
        self.push_observability_pair(
            sample_trace,
            record_trace,
            CommonObservabilityRecordKind::RuntimeFuelStrategyConfig,
        )
    }

    pub fn configure_runtime_fuel_strategy_and_push_to_trace_pair<
        const SM: usize,
        const RM: usize,
    >(
        &mut self,
        strategy: RuntimeFuelStrategy,
        traces: &mut FixedCommonObservabilityTracePair<SM, RM>,
    ) -> Result<(), PushObservabilityPairError> {
        let (sample_trace, record_trace) = traces.split_mut();
        self.configure_runtime_fuel_strategy_and_push_pair(strategy, sample_trace, record_trace)
    }

    pub fn diagnostics_snapshot(&self) -> CommonDiagnosticsSnapshot {
        CommonDiagnosticsSnapshot {
            sync_state: self.sync_state(),
            fault_state: self.fault_state(),
            timing_metrics: self.actions.timing_metrics(),
        }
    }

    pub fn decision_telemetry(&self) -> CommonDecisionTelemetry {
        let snapshot = self.runtime.snapshot();
        let decision = CommonDecisionTelemetry {
            control_mode: snapshot.engine.mode,
            rev_soft_active: snapshot.rev_soft_active,
            rev_hard_active: snapshot.rev_hard_active,
            launch_active: snapshot.launch_active,
            flat_shift_active: snapshot.flat_shift_active,
            fuel_cut: snapshot.fuel_cut,
            spark_cut: snapshot.spark_cut,
            fuel_cut_reason: CommonCutReason::None,
            spark_cut_reason: CommonCutReason::None,
        };
        CommonDecisionTelemetry {
            fuel_cut_reason: common_fuel_cut_reason(&snapshot),
            spark_cut_reason: common_spark_cut_reason(&snapshot),
            ..decision
        }
    }

    pub fn control_telemetry(&self) -> CommonControlTelemetry {
        let snapshot = self.runtime.snapshot();
        CommonControlTelemetry {
            fuel_pulse_width: snapshot.control.fuel_pulse_width,
            ignition_advance: snapshot.control.ignition_advance,
            dwell: snapshot.control.dwell,
            lambda_target: snapshot.control.lambda_target,
            torque_limit_x100: snapshot.control.torque_limit_x100,
        }
    }

    pub fn engine_telemetry(&self) -> CommonEngineTelemetry {
        let snapshot = self.runtime.snapshot();
        CommonEngineTelemetry {
            rpm: snapshot.engine.rpm,
            load_kpa10: snapshot.engine.load_kpa10,
            angle_x10: snapshot.engine.angle_x10,
            phase: snapshot.engine.phase,
        }
    }

    pub fn runtime_fault_telemetry(&self) -> CommonRuntimeFaultTelemetry {
        common_runtime_fault_telemetry(self.fault_state())
    }

    pub fn protection_telemetry(&self) -> CommonProtectionTelemetry {
        common_protection_telemetry(
            self.decision_telemetry(),
            self.runtime_fault_telemetry(),
            self.frontier_telemetry().fault,
        )
    }

    pub fn limp_action_telemetry(&self) -> CommonLimpActionTelemetry {
        common_limp_action_telemetry(
            self.action_telemetry(),
            self.protection_telemetry(),
            self.frontier_telemetry().fault,
        )
    }

    pub fn high_rate_log_telemetry(&self) -> CommonHighRateLogTelemetry {
        let diagnostics = self.diagnostics_telemetry();
        common_high_rate_log_telemetry(
            self.decision_telemetry(),
            self.runtime_fault_telemetry(),
            self.frontier_telemetry().fault,
            diagnostics,
            self.calibration_package_identity(),
        )
    }

    pub fn frontier_telemetry(&self) -> CommonFrontierTelemetry {
        let frontier = self.actions.frontier();
        CommonFrontierTelemetry {
            active_horizon_id: frontier.active_horizon_id(),
            horizon_start_us: frontier.horizon_start_us(),
            horizon_end_us: frontier.horizon_end_us(),
            last_accepted_horizon_id: frontier.last_accepted_horizon_id(),
            last_accepted_horizon_start_us: frontier.last_accepted_horizon_start_us(),
            last_accepted_horizon_end_us: frontier.last_accepted_horizon_end_us(),
            heartbeat_deadline_us: frontier.heartbeat_deadline_us(),
            active_permit_mask: frontier.active_permit_mask(),
            active_stop_reason: frontier.active_stop_reason(),
            fault: common_frontier_fault_telemetry(frontier.active_stop_reason()),
        }
    }

    pub fn scheduler_ownership_telemetry(&self) -> CommonSchedulerOwnershipTelemetry {
        let frontier = self.actions.frontier();
        CommonSchedulerOwnershipTelemetry {
            mode: common_scheduler_mode(frontier.mode()),
            active_groups: frontier.active_groups(),
            injection_count: frontier.injection_count(),
            ignition_count: frontier.ignition_count(),
        }
    }

    pub fn scheduler_reservation_telemetry(&self) -> CommonSchedulerReservationTelemetry {
        let reserved_channels = self.actions.frontier().reserved_channels();
        CommonSchedulerReservationTelemetry {
            injector_channels: reserved_channels[0],
            ignition_channels: reserved_channels[1],
            idle_channels: reserved_channels[2],
            fan_channels: reserved_channels[3],
        }
    }

    pub fn scheduler_state_summary_telemetry(&self) -> CommonSchedulerStateSummaryTelemetry {
        CommonSchedulerStateSummaryTelemetry {
            armed: self.actions.frontier().is_armed(),
        }
    }

    pub fn scheduler_window_telemetry(&self) -> CommonSchedulerWindowTelemetry {
        let frontier = self.actions.frontier();
        CommonSchedulerWindowTelemetry {
            last_injection_start: frontier.last_injection_start(),
            last_injection_end: frontier.last_injection_end(),
            last_ignition_start: frontier.last_ignition_start(),
            last_ignition_end: frontier.last_ignition_end(),
        }
    }

    #[allow(clippy::type_complexity)]
    pub fn apply_event_and_record<const M: usize>(
        &mut self,
        event: BoardEvent,
        trace: &mut FixedCommonObservabilityRecordTrace<M>,
    ) -> Result<
        Option<StepResult>,
        ApplyAndRecordError<
            S::Error,
            C::Error,
            <ScheduledActionExecutor<N> as ActionExecutor>::Error,
            W::Error,
            T::Error,
            P::Error,
        >,
    > {
        let result = self
            .apply_event(event)
            .map_err(ApplyAndRecordError::Apply)?;
        trace
            .push(CommonObservabilityRecord {
                kind: event.observability_kind(),
                sample: self.observability_sample(),
            })
            .map_err(ApplyAndRecordError::Record)?;
        Ok(result)
    }

    pub fn push_observability_sample<const M: usize>(
        &mut self,
        trace: &mut FixedCommonObservabilityTrace<M>,
    ) -> Result<(), CommonObservabilityTraceOverflow> {
        trace.push(self.observability_sample())
    }

    pub fn push_observability_pair<const SM: usize, const RM: usize>(
        &mut self,
        sample_trace: &mut FixedCommonObservabilityTrace<SM>,
        record_trace: &mut FixedCommonObservabilityRecordTrace<RM>,
        kind: CommonObservabilityRecordKind,
    ) -> Result<(), PushObservabilityPairError> {
        let sample = self.observability_sample();

        record_trace
            .push(CommonObservabilityRecord { kind, sample })
            .map_err(PushObservabilityPairError::Record)?;
        sample_trace
            .push(sample)
            .map_err(PushObservabilityPairError::Sample)?;
        Ok(())
    }

    pub fn push_observability_to_trace_pair<const SM: usize, const RM: usize>(
        &mut self,
        traces: &mut FixedCommonObservabilityTracePair<SM, RM>,
        kind: CommonObservabilityRecordKind,
    ) -> Result<(), PushObservabilityPairError> {
        let (sample, record) = traces.split_mut();
        self.push_observability_pair(sample, record, kind)
    }

    #[allow(clippy::type_complexity)]
    pub fn apply_event_and_push_pair<const SM: usize, const RM: usize>(
        &mut self,
        event: BoardEvent,
        sample_trace: &mut FixedCommonObservabilityTrace<SM>,
        record_trace: &mut FixedCommonObservabilityRecordTrace<RM>,
    ) -> Result<
        Option<StepResult>,
        ApplyAndPushPairError<
            S::Error,
            C::Error,
            <ScheduledActionExecutor<N> as ActionExecutor>::Error,
            W::Error,
            T::Error,
            P::Error,
        >,
    > {
        let result = self
            .apply_event(event)
            .map_err(ApplyAndPushPairError::Apply)?;
        let sample = self.observability_sample();

        record_trace
            .push(CommonObservabilityRecord {
                kind: event.observability_kind(),
                sample,
            })
            .map_err(ApplyAndPushPairError::Record)?;
        sample_trace
            .push(sample)
            .map_err(ApplyAndPushPairError::Sample)?;
        Ok(result)
    }

    #[allow(clippy::type_complexity)]
    pub fn apply_event_and_push_to_trace_pair<const SM: usize, const RM: usize>(
        &mut self,
        event: BoardEvent,
        traces: &mut FixedCommonObservabilityTracePair<SM, RM>,
    ) -> Result<
        Option<StepResult>,
        ApplyAndPushPairError<
            S::Error,
            C::Error,
            <ScheduledActionExecutor<N> as ActionExecutor>::Error,
            W::Error,
            T::Error,
            P::Error,
        >,
    > {
        let (sample_trace, record_trace) = traces.split_mut();
        self.apply_event_and_push_pair(event, sample_trace, record_trace)
    }

    #[allow(clippy::type_complexity)]
    pub fn poll_sensor_and_record<const M: usize>(
        &mut self,
        trace: &mut FixedCommonObservabilityRecordTrace<M>,
    ) -> Result<
        CaptureSample,
        PollSensorAndRecordError<
            S::Error,
            C::Error,
            <ScheduledActionExecutor<N> as ActionExecutor>::Error,
            W::Error,
            T::Error,
            P::Error,
        >,
    > {
        let sample = self.poll_sensor().map_err(PollSensorAndRecordError::Poll)?;
        trace
            .push(CommonObservabilityRecord {
                kind: CommonObservabilityRecordKind::SensorPoll,
                sample: self.observability_sample(),
            })
            .map_err(PollSensorAndRecordError::Record)?;
        Ok(sample)
    }

    #[allow(clippy::type_complexity)]
    pub fn poll_sensor_and_push_pair<const SM: usize, const RM: usize>(
        &mut self,
        sample_trace: &mut FixedCommonObservabilityTrace<SM>,
        record_trace: &mut FixedCommonObservabilityRecordTrace<RM>,
    ) -> Result<
        CaptureSample,
        PollSensorAndPushPairError<
            S::Error,
            C::Error,
            <ScheduledActionExecutor<N> as ActionExecutor>::Error,
            W::Error,
            T::Error,
            P::Error,
        >,
    > {
        let sample = self
            .poll_sensor()
            .map_err(PollSensorAndPushPairError::Poll)?;
        let observability_sample = self.observability_sample();

        record_trace
            .push(CommonObservabilityRecord {
                kind: CommonObservabilityRecordKind::SensorPoll,
                sample: observability_sample,
            })
            .map_err(PollSensorAndPushPairError::Record)?;
        sample_trace
            .push(observability_sample)
            .map_err(PollSensorAndPushPairError::Sample)?;
        Ok(sample)
    }

    #[allow(clippy::type_complexity)]
    pub fn poll_sensor_and_push_to_trace_pair<const SM: usize, const RM: usize>(
        &mut self,
        traces: &mut FixedCommonObservabilityTracePair<SM, RM>,
    ) -> Result<
        CaptureSample,
        PollSensorAndPushPairError<
            S::Error,
            C::Error,
            <ScheduledActionExecutor<N> as ActionExecutor>::Error,
            W::Error,
            T::Error,
            P::Error,
        >,
    > {
        let (sample_trace, record_trace) = traces.split_mut();
        self.poll_sensor_and_push_pair(sample_trace, record_trace)
    }
}

#[cfg(test)]
mod tests;
