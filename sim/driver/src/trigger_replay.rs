//! Deterministic trigger replay fixtures for host-side simulator tests.

use ecu_board_api::EdgeKind;
use ecu_domain::{AbsoluteTimeAuthority, Degrees10, EngineTimeAuthority, Micros, SyncState, Ticks};
use ecu_runtime::runtime_full_sequential_authorized;
use ecu_trigger::{
    MissingToothDecoder, MissingToothDecoderConfig, MissingToothDecoderEvent, PollLevelPolarity,
    RuntimeSecondaryTriggerMode, RuntimeSecondaryTriggerProfile, SyncLossReason,
    TriggerAngleAuthority, TriggerDiagnostics, TriggerEdge, TriggerLevel, TriggerSpeed,
    TriggerValidationError, DEFAULT_MISSING_TOOTH_GAP_RATIO_X1000,
};

pub const SIXTY_MINUS_TWO_NOMINAL_TEETH: u8 = 60;
pub const SIXTY_MINUS_TWO_MISSING_TEETH: u8 = 2;
pub const SIXTY_MINUS_TWO_OBSERVED_TEETH: u8 =
    SIXTY_MINUS_TWO_NOMINAL_TEETH - SIXTY_MINUS_TWO_MISSING_TEETH;

const NORMAL_TOOTH_TICKS: u32 = 1_000;
const MISSING_TOOTH_GAP_TICKS: u32 =
    NORMAL_TOOTH_TICKS * (SIXTY_MINUS_TWO_MISSING_TEETH as u32 + 1);
const MINIMUM_EDGE_INTERVAL_TICKS: u32 = 50;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ReplayTriggerChannel {
    Crank,
    Cam,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ReplayCamLevel {
    Low,
    High,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SyntheticTriggerEdge {
    pub at: Ticks,
    pub channel: ReplayTriggerChannel,
    pub kind: EdgeKind,
    pub cam_level: Option<ReplayCamLevel>,
}

impl SyntheticTriggerEdge {
    pub const fn crank(at_ticks: u32) -> Self {
        Self::crank_with_kind(at_ticks, EdgeKind::Rising)
    }

    pub const fn crank_with_kind(at_ticks: u32, kind: EdgeKind) -> Self {
        Self {
            at: Ticks::new(at_ticks),
            channel: ReplayTriggerChannel::Crank,
            kind,
            cam_level: None,
        }
    }

    pub const fn cam(at_ticks: u32, kind: EdgeKind, level: ReplayCamLevel) -> Self {
        Self {
            at: Ticks::new(at_ticks),
            channel: ReplayTriggerChannel::Cam,
            kind,
            cam_level: Some(level),
        }
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct TriggerReplayFixture {
    pub name: &'static str,
    pub decoder_config: MissingToothDecoderConfig,
    pub edges: Vec<SyntheticTriggerEdge>,
}

impl TriggerReplayFixture {
    pub fn sixty_minus_two_with_cam() -> Self {
        let mut edges = Vec::new();
        let mut timestamp = push_initial_missing_tooth_lock(&mut edges, EdgeKind::Rising);
        timestamp = push_locked_sixty_minus_two_cycle(
            &mut edges,
            timestamp,
            EdgeKind::Rising,
            Some(ReplayCamLevel::High),
        );
        let _ = push_locked_sixty_minus_two_cycle(
            &mut edges,
            timestamp,
            EdgeKind::Rising,
            Some(ReplayCamLevel::High),
        );

        Self {
            name: "60-2-with-cam",
            decoder_config: expert_sixty_minus_two_config(),
            edges,
        }
    }

    pub fn wrong_primary_edge_with_certified_profile() -> Self {
        let mut edges = Vec::new();
        let timestamp = push_initial_missing_tooth_lock(&mut edges, EdgeKind::Falling);
        let _ = push_locked_sixty_minus_two_cycle(
            &mut edges,
            timestamp,
            EdgeKind::Falling,
            Some(ReplayCamLevel::High),
        );

        Self {
            name: "wrong-primary-edge-certified-profile",
            decoder_config: certified_sixty_minus_two_config(),
            edges,
        }
    }

    pub fn wrong_tooth_count_after_lock() -> Self {
        let mut edges = Vec::new();
        let mut timestamp = push_initial_missing_tooth_lock(&mut edges, EdgeKind::Rising);
        for _ in 2..=SIXTY_MINUS_TWO_OBSERVED_TEETH {
            timestamp = timestamp.saturating_add(NORMAL_TOOTH_TICKS);
            edges.push(SyntheticTriggerEdge::crank(timestamp));
        }
        timestamp = timestamp.saturating_add(NORMAL_TOOTH_TICKS);
        edges.push(SyntheticTriggerEdge::crank(timestamp));

        Self {
            name: "wrong-tooth-count-after-lock",
            decoder_config: certified_sixty_minus_two_config(),
            edges,
        }
    }

    pub fn false_gap_after_primary_lock() -> Self {
        let mut edges = Vec::new();
        let mut timestamp = push_initial_missing_tooth_lock(&mut edges, EdgeKind::Rising);

        for _ in 0..4 {
            timestamp = timestamp.saturating_add(NORMAL_TOOTH_TICKS);
            edges.push(SyntheticTriggerEdge::crank(timestamp));
        }

        timestamp = timestamp.saturating_add(MISSING_TOOTH_GAP_TICKS);
        edges.push(SyntheticTriggerEdge::crank(timestamp));

        Self {
            name: "false-gap-after-primary-lock",
            decoder_config: certified_sixty_minus_two_config(),
            edges,
        }
    }

    pub fn false_edge_noise_after_lock() -> Self {
        let mut edges = Vec::new();
        let mut timestamp = push_initial_missing_tooth_lock(&mut edges, EdgeKind::Rising);
        edges.push(SyntheticTriggerEdge::crank(
            timestamp.saturating_add(MINIMUM_EDGE_INTERVAL_TICKS - 1),
        ));
        timestamp = timestamp.saturating_add(NORMAL_TOOTH_TICKS);
        edges.push(SyntheticTriggerEdge::crank(timestamp));

        Self {
            name: "false-edge-noise-after-lock",
            decoder_config: expert_sixty_minus_two_config(),
            edges,
        }
    }

    pub fn missing_cam() -> Self {
        let mut edges = Vec::new();
        let timestamp = push_initial_missing_tooth_lock(&mut edges, EdgeKind::Rising);
        let _ = push_locked_sixty_minus_two_cycle(&mut edges, timestamp, EdgeKind::Rising, None);

        Self {
            name: "missing-cam",
            decoder_config: certified_sixty_minus_two_config(),
            edges,
        }
    }

    pub fn wrong_cam_level() -> Self {
        let mut edges = Vec::new();
        let mut timestamp = push_initial_missing_tooth_lock(&mut edges, EdgeKind::Rising);
        timestamp = push_locked_sixty_minus_two_cycle(
            &mut edges,
            timestamp,
            EdgeKind::Rising,
            Some(ReplayCamLevel::Low),
        );
        let _ = push_locked_sixty_minus_two_cycle(
            &mut edges,
            timestamp,
            EdgeKind::Rising,
            Some(ReplayCamLevel::Low),
        );

        Self {
            name: "wrong-cam-level",
            decoder_config: certified_sixty_minus_two_poll_level_config(),
            edges,
        }
    }

    pub fn sync_loss_after_primary_lock() -> Self {
        Self {
            name: "sync-loss-after-primary-lock",
            ..Self::wrong_tooth_count_after_lock()
        }
    }

    pub fn replay(&self) -> Result<TriggerReplay, TriggerReplayError> {
        replay_trigger_edges(self.decoder_config, &self.edges)
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TriggerReplayError {
    InvalidDecoderConfig(TriggerValidationError),
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct TriggerReplay {
    pub frames: Vec<TriggerReplayFrame>,
    pub final_diagnostics: TriggerDiagnostics,
    pub unsupported_cam_edges: usize,
}

impl TriggerReplay {
    pub fn final_authority(&self) -> EngineTimeAuthority {
        self.final_diagnostics.authority
    }

    pub fn authority_transitions(&self) -> Vec<EngineTimeAuthority> {
        let mut transitions = Vec::new();
        let mut previous = None;

        for frame in &self.frames {
            let authority = frame.authority();
            if previous != Some(authority) {
                transitions.push(authority);
                previous = Some(authority);
            }
        }

        transitions
    }

    pub fn compatibility_transitions(&self) -> Vec<SyncState> {
        let mut transitions = Vec::new();
        let mut previous = None;

        for frame in &self.frames {
            let sync = frame.compatibility_summary();
            if previous != Some(sync) {
                transitions.push(sync);
                previous = Some(sync);
            }
        }

        transitions
    }

    pub fn reached_certified_authority(&self) -> bool {
        self.frames
            .iter()
            .any(|frame| frame.authority().absolute == AbsoluteTimeAuthority::CertifiedProfile)
    }

    pub fn reached_full_sequential_authority(&self) -> bool {
        self.frames
            .iter()
            .any(TriggerReplayFrame::allows_full_sequential_authority)
    }

    pub fn sync_loss_reasons(&self) -> Vec<SyncLossReason> {
        self.frames
            .iter()
            .filter_map(|frame| frame.sync_loss)
            .collect()
    }
}

/// Stable per-edge replay record for later tooth/composite log viewers.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct TriggerReplayFrame {
    /// Zero-based replay position. Use this as the stable row key in viewers.
    pub index: usize,
    /// The synthetic edge that was replayed at this position.
    pub edge: SyntheticTriggerEdge,
    /// Decoder feedback emitted after ingesting `edge`.
    pub decoder_event: Option<MissingToothDecoderEvent>,
    /// Sync-loss reason recorded for the same edge, if any.
    pub sync_loss: Option<SyncLossReason>,
    /// Full post-edge diagnostic snapshot.
    pub diagnostics: TriggerDiagnostics,
    /// Marks cam edges that the current replay mode can only approximate.
    pub unsupported_cam_edge: bool,
}

impl TriggerReplayFrame {
    pub fn authority(&self) -> EngineTimeAuthority {
        self.diagnostics.authority
    }

    pub fn compatibility_summary(&self) -> SyncState {
        self.authority().compatibility_summary()
    }

    pub fn allows_full_sequential_authority(&self) -> bool {
        runtime_full_sequential_authorized(self.authority())
    }
}

pub fn replay_trigger_edges(
    decoder_config: MissingToothDecoderConfig,
    edges: &[SyntheticTriggerEdge],
) -> Result<TriggerReplay, TriggerReplayError> {
    let mut decoder = MissingToothDecoder::try_new(decoder_config)
        .map_err(TriggerReplayError::InvalidDecoderConfig)?;
    let mut frames = Vec::with_capacity(edges.len());
    let unsupported_cam_edges = 0usize;

    for (index, edge) in edges.iter().copied().enumerate() {
        let mut decoder_event = None;
        let mut sync_loss = None;
        let unsupported_cam_edge = false;

        match edge.channel {
            ReplayTriggerChannel::Crank => {
                match decoder.ingest_primary_edge_with_kind(edge.at, trigger_edge(edge.kind)) {
                    Ok(event) => decoder_event = Some(event),
                    Err(reason) => sync_loss = Some(reason),
                }
            }
            ReplayTriggerChannel::Cam => match decoder_config.secondary.mode {
                RuntimeSecondaryTriggerMode::SingleToothCam => {
                    if let Err(reason) = decoder.ingest_secondary_edge(trigger_edge(edge.kind)) {
                        sync_loss = Some(reason);
                    }
                }
                RuntimeSecondaryTriggerMode::PollLevel => {
                    if let Some(level) = edge.cam_level {
                        if let Err(reason) = decoder.ingest_secondary_level(trigger_level(level)) {
                            sync_loss = Some(reason);
                        }
                    } else if let Err(reason) =
                        decoder.ingest_secondary_edge(trigger_edge(edge.kind))
                    {
                        sync_loss = Some(reason);
                    }
                }
                RuntimeSecondaryTriggerMode::None => {
                    if let Err(reason) = decoder.ingest_secondary_edge(trigger_edge(edge.kind)) {
                        sync_loss = Some(reason);
                    }
                }
            },
        }

        frames.push(TriggerReplayFrame {
            index,
            edge,
            decoder_event,
            sync_loss,
            diagnostics: decoder.diagnostics(),
            unsupported_cam_edge,
        });
    }

    Ok(TriggerReplay {
        frames,
        final_diagnostics: decoder.diagnostics(),
        unsupported_cam_edges,
    })
}

fn expert_sixty_minus_two_config() -> MissingToothDecoderConfig {
    sixty_minus_two_config(TriggerAngleAuthority::ExpertManual(Degrees10::new(840)))
}

fn certified_sixty_minus_two_config() -> MissingToothDecoderConfig {
    sixty_minus_two_config(TriggerAngleAuthority::CertifiedProfile(Degrees10::new(840)))
}

fn certified_sixty_minus_two_poll_level_config() -> MissingToothDecoderConfig {
    MissingToothDecoderConfig {
        secondary: RuntimeSecondaryTriggerProfile {
            mode: RuntimeSecondaryTriggerMode::PollLevel,
            edge: TriggerEdge::Rising,
            poll_level: PollLevelPolarity::ActiveHigh,
        },
        ..certified_sixty_minus_two_config()
    }
}

fn sixty_minus_two_config(
    trigger_angle_atdc_deg10: TriggerAngleAuthority,
) -> MissingToothDecoderConfig {
    MissingToothDecoderConfig {
        nominal_teeth: SIXTY_MINUS_TWO_NOMINAL_TEETH,
        missing_teeth: SIXTY_MINUS_TWO_MISSING_TEETH,
        primary_speed: TriggerSpeed::Crank,
        primary_edge: TriggerEdge::Rising,
        secondary: RuntimeSecondaryTriggerProfile {
            mode: RuntimeSecondaryTriggerMode::SingleToothCam,
            edge: TriggerEdge::Rising,
            poll_level: PollLevelPolarity::ActiveHigh,
        },
        trigger_angle_atdc_deg10,
        tooth_angle_multiplier: 1,
        minimum_edge_interval: Ticks::new(MINIMUM_EDGE_INTERVAL_TICKS),
        gap_ratio_threshold_x1000: DEFAULT_MISSING_TOOTH_GAP_RATIO_X1000,
    }
}

fn push_initial_missing_tooth_lock(
    edges: &mut Vec<SyntheticTriggerEdge>,
    crank_edge_kind: EdgeKind,
) -> u32 {
    edges.push(SyntheticTriggerEdge::crank_with_kind(0, crank_edge_kind));
    edges.push(SyntheticTriggerEdge::crank_with_kind(
        NORMAL_TOOTH_TICKS,
        crank_edge_kind,
    ));
    edges.push(SyntheticTriggerEdge::crank_with_kind(
        NORMAL_TOOTH_TICKS + MISSING_TOOTH_GAP_TICKS,
        crank_edge_kind,
    ));
    NORMAL_TOOTH_TICKS + MISSING_TOOTH_GAP_TICKS
}

fn push_locked_sixty_minus_two_cycle(
    edges: &mut Vec<SyntheticTriggerEdge>,
    mut timestamp: u32,
    crank_edge_kind: EdgeKind,
    cam_level: Option<ReplayCamLevel>,
) -> u32 {
    if let Some(cam_level) = cam_level {
        edges.push(SyntheticTriggerEdge::cam(
            timestamp.saturating_add(MINIMUM_EDGE_INTERVAL_TICKS),
            EdgeKind::Rising,
            cam_level,
        ));
    }

    for _ in 2..=SIXTY_MINUS_TWO_OBSERVED_TEETH {
        timestamp = timestamp.saturating_add(NORMAL_TOOTH_TICKS);
        edges.push(SyntheticTriggerEdge::crank_with_kind(
            timestamp,
            crank_edge_kind,
        ));
    }

    timestamp = timestamp.saturating_add(MISSING_TOOTH_GAP_TICKS);
    edges.push(SyntheticTriggerEdge::crank_with_kind(
        timestamp,
        crank_edge_kind,
    ));

    timestamp
}

fn trigger_edge(kind: EdgeKind) -> TriggerEdge {
    match kind {
        EdgeKind::Rising => TriggerEdge::Rising,
        EdgeKind::Falling => TriggerEdge::Falling,
    }
}

fn trigger_level(level: ReplayCamLevel) -> TriggerLevel {
    match level {
        ReplayCamLevel::Low => TriggerLevel::Low,
        ReplayCamLevel::High => TriggerLevel::High,
    }
}

pub fn replay_frame_time_us(frame: &TriggerReplayFrame) -> Micros {
    Micros::new(frame.edge.at.get())
}
