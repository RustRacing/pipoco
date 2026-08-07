use super::*;
use crate::frontier::{
    TimingIslandMetricSnapshot, TimingIslandPermitMask, TimingIslandStopReason,
    TimingIslandSyncLossReason, HEARTBEAT_EXPIRY_US, HORIZON_SEQUENCE_BITS, MAX_HORIZON_US,
};
use crate::safety::{SafetyGateInput, SafetyGateReason, SafetyGateStatus, SafetyPermitMask};
use crate::timing_island::{
    EdgeBatch, TimingIslandCommand, TimingIslandEvent, TimingIslandFaultStatus,
    TimingIslandRejectReason, TriggerEdge,
};
use crate::wire::{
    decode_safety_gate_input, decode_safety_gate_status, decode_timing_island_command,
    decode_timing_island_event, encode_safety_gate_input, encode_safety_gate_status,
    encode_timing_island_command, encode_timing_island_event, TimingIslandCodecError,
    SAFETY_GATE_INPUT_WIRE_LEN, SAFETY_GATE_STATUS_WIRE_LEN, TIMING_ISLAND_COMMAND_TAG_ARM_OUTPUT,
    TIMING_ISLAND_COMMAND_TAG_FEED_WATCHDOG, TIMING_ISLAND_COMMAND_WIRE_LEN,
    TIMING_ISLAND_EVENT_TAG_REJECTED, TIMING_ISLAND_EVENT_WIRE_LEN, TIMING_ISLAND_WIRE_VERSION,
};
use crate::{
    CommonAfterstartTelemetry, CommonAfterstartWindowMode, CommonFaultTransitionAction,
    CommonFaultTransitionEventId, CommonFaultTransitionEventTelemetry, CommonFrontierFaultAction,
    CommonFrontierFaultEventId, CommonFrontierFaultTelemetry, CommonFrontierTelemetry,
    CommonFuelStrategyMode, CommonHighRateLogTelemetry, CommonLambdaActivity,
    CommonLambdaCorrectionTelemetry, CommonLambdaDisableReason, CommonLambdaTelemetry,
    CommonLimpActionLevel, CommonLimpActionSource, CommonLimpActionTelemetry,
    CommonPendingInputTelemetry, CommonProtectionAction, CommonProtectionLevel,
    CommonProtectionPersistence, CommonProtectionSource, CommonProtectionTelemetry,
    CommonRuntimeFaultTelemetry, CommonSchedulerMode, CommonSchedulerOwnershipTelemetry,
    CommonSchedulerReservationTelemetry, CommonSchedulerStateSummaryTelemetry,
    CommonSchedulerWindowTelemetry, CommonShiftArmingTelemetry, CommonStartupTelemetry,
    CommonStartupWindowMode, CommonTransientEnrichmentTelemetry,
};
use core::mem::needs_drop;
use ecu_domain::{
    AbsoluteTimeAuthority, CancelReason, ChannelId, ControlMode, CrankSyncState, Degrees10,
    DwellUs, EnginePhase, EngineTimeAuthority, FaultCode, FaultSeverity, Kpa10, Lambda100, Micros,
    Percent, PhaseSyncState, PulseWidthUs, Rpm, SyncState, Ticks,
};

#[test]
fn edge_batch_push_until_full_then_rejects() {
    let mut batch = EdgeBatch::<4>::new();

    for idx in 0..4 {
        assert!(batch
            .push(TriggerEdge::new(EdgeKind::Rising, Ticks::new(idx)))
            .is_ok());
    }

    assert_eq!(batch.len(), 4);
    assert_eq!(
        batch.push(TriggerEdge::new(EdgeKind::Falling, Ticks::new(99))),
        Err(TriggerEdge::new(EdgeKind::Falling, Ticks::new(99)))
    );

    let mut seen = 0;
    for (idx, edge) in batch.iter().enumerate() {
        assert_eq!(edge.kind, EdgeKind::Rising);
        assert_eq!(edge.at, Ticks::new(idx as u32));
        seen += 1;
    }

    assert_eq!(seen, 4);
}

#[test]
fn output_transition_batch_clear_resets_len_but_preserves_capacity() {
    let mut batch = OutputTransitionBatch::<2>::new();

    assert!(batch
        .push(OutputTransition::new(
            EcuOutput::Injector(ChannelId::new(1)),
            OutputLevel::High,
            Ticks::new(10),
        ))
        .is_ok());
    assert!(batch
        .push(OutputTransition::new(
            EcuOutput::Ignition(ChannelId::new(2)),
            OutputLevel::Low,
            Ticks::new(11),
        ))
        .is_ok());

    assert!(batch.push(OutputTransition::EMPTY).is_err());
    batch.clear();

    assert!(batch.is_empty());
    assert_eq!(batch.capacity(), 2);

    assert!(batch
        .push(OutputTransition::new(
            EcuOutput::Injector(ChannelId::new(3)),
            OutputLevel::High,
            Ticks::new(12),
        ))
        .is_ok());
    assert_eq!(batch.len(), 1);
    assert_eq!(batch.as_slice()[0].at, Ticks::new(12));
}

#[test]
fn core_types_are_copy_and_do_not_need_drop() {
    fn assert_copy<T: Copy>() {}

    assert_copy::<TriggerEdge>();
    assert_copy::<OutputTransition>();
    assert_copy::<AuxCommand>();
    assert_copy::<CommonSyncTelemetryState>();
    assert_copy::<CommonDiagnosticsTelemetry>();
    assert_copy::<CommonHighRateLogTelemetry>();
    assert_copy::<CommonAfterstartTelemetry>();
    assert_copy::<CommonAfterstartWindowMode>();
    assert_copy::<CommonWarmupTelemetry>();
    assert_copy::<CommonWarmupTemperatureMode>();
    assert_copy::<CommonStartupTelemetry>();
    assert_copy::<CommonStartupWindowMode>();
    assert_copy::<CommonLambdaActivity>();
    assert_copy::<CommonLambdaCorrectionTelemetry>();
    assert_copy::<CommonLambdaDisableReason>();
    assert_copy::<CommonLambdaTelemetry>();
    assert_copy::<CommonLimpActionLevel>();
    assert_copy::<CommonLimpActionSource>();
    assert_copy::<CommonLimpActionTelemetry>();
    assert_copy::<CommonTransientEnrichmentTelemetry>();
    assert_copy::<CommonProtectionLevel>();
    assert_copy::<CommonProtectionSource>();
    assert_copy::<CommonProtectionAction>();
    assert_copy::<CommonProtectionPersistence>();
    assert_copy::<CommonProtectionTelemetry>();
    assert_copy::<CommonCutReason>();
    assert_copy::<CommonDecisionTelemetry>();
    assert_copy::<CommonShiftArmingTelemetry>();
    assert_copy::<CommonSchedulerMode>();
    assert_copy::<CommonSchedulerOwnershipTelemetry>();
    assert_copy::<CommonSchedulerReservationTelemetry>();
    assert_copy::<CommonSchedulerStateSummaryTelemetry>();
    assert_copy::<CommonSchedulerWindowTelemetry>();
    assert_copy::<CommonControlTelemetry>();
    assert_copy::<CommonFrontierTelemetry>();
    assert_copy::<CommonLambdaMode>();
    assert_copy::<CommonIgnitionLimitReason>();
    assert_copy::<CommonTorqueLimitReason>();
    assert_copy::<CommonControlReasonTelemetry>();
    assert_copy::<CommonPendingInputTelemetry>();
    assert_copy::<CommonFuelObservationTelemetry>();
    assert_copy::<CommonEnrichmentTelemetry>();
    assert_copy::<CommonActionTelemetry>();
    assert_copy::<CommonTorqueTelemetry>();
    assert_copy::<CommonEngineTelemetry>();
    assert_copy::<CommonTriggerEdgeTelemetry>();
    assert_copy::<CommonCamEdgeTelemetry>();
    assert_copy::<CommonValidatedInputTelemetry>();
    assert_copy::<CommonRuntimeFaultTelemetry>();
    assert_copy::<CommonFaultTransitionTelemetry>();
    assert_copy::<EngineTimeAuthorityTelemetry>();
    assert_copy::<SensorSnapshot>();
    assert_copy::<TelemetryFrame>();
    assert_copy::<EdgeBatch<4>>();
    assert_copy::<OutputTransitionBatch<4>>();
    assert_copy::<AuxCommandBatch<4>>();

    assert!(!needs_drop::<TriggerEdge>());
    assert!(!needs_drop::<OutputTransition>());
    assert!(!needs_drop::<AuxCommand>());
    assert!(!needs_drop::<CommonSyncTelemetryState>());
    assert!(!needs_drop::<CommonDiagnosticsTelemetry>());
    assert!(!needs_drop::<CommonHighRateLogTelemetry>());
    assert!(!needs_drop::<CommonAfterstartTelemetry>());
    assert!(!needs_drop::<CommonAfterstartWindowMode>());
    assert!(!needs_drop::<CommonWarmupTelemetry>());
    assert!(!needs_drop::<CommonWarmupTemperatureMode>());
    assert!(!needs_drop::<CommonStartupTelemetry>());
    assert!(!needs_drop::<CommonStartupWindowMode>());
    assert!(!needs_drop::<CommonLambdaActivity>());
    assert!(!needs_drop::<CommonLambdaCorrectionTelemetry>());
    assert!(!needs_drop::<CommonLambdaDisableReason>());
    assert!(!needs_drop::<CommonLambdaTelemetry>());
    assert!(!needs_drop::<CommonLimpActionLevel>());
    assert!(!needs_drop::<CommonLimpActionSource>());
    assert!(!needs_drop::<CommonLimpActionTelemetry>());
    assert!(!needs_drop::<CommonTransientEnrichmentTelemetry>());
    assert!(!needs_drop::<CommonProtectionLevel>());
    assert!(!needs_drop::<CommonProtectionSource>());
    assert!(!needs_drop::<CommonProtectionAction>());
    assert!(!needs_drop::<CommonProtectionPersistence>());
    assert!(!needs_drop::<CommonProtectionTelemetry>());
    assert!(!needs_drop::<CommonCutReason>());
    assert!(!needs_drop::<CommonDecisionTelemetry>());
    assert!(!needs_drop::<CommonShiftArmingTelemetry>());
    assert!(!needs_drop::<CommonSchedulerMode>());
    assert!(!needs_drop::<CommonSchedulerOwnershipTelemetry>());
    assert!(!needs_drop::<CommonSchedulerReservationTelemetry>());
    assert!(!needs_drop::<CommonSchedulerStateSummaryTelemetry>());
    assert!(!needs_drop::<CommonSchedulerWindowTelemetry>());
    assert!(!needs_drop::<CommonControlTelemetry>());
    assert!(!needs_drop::<CommonFrontierTelemetry>());
    assert!(!needs_drop::<CommonLambdaMode>());
    assert!(!needs_drop::<CommonIgnitionLimitReason>());
    assert!(!needs_drop::<CommonTorqueLimitReason>());
    assert!(!needs_drop::<CommonControlReasonTelemetry>());
    assert!(!needs_drop::<CommonPendingInputTelemetry>());
    assert!(!needs_drop::<CommonFuelObservationTelemetry>());
    assert!(!needs_drop::<CommonEnrichmentTelemetry>());
    assert!(!needs_drop::<CommonActionTelemetry>());
    assert!(!needs_drop::<CommonTorqueTelemetry>());
    assert!(!needs_drop::<CommonEngineTelemetry>());
    assert!(!needs_drop::<CommonTriggerEdgeTelemetry>());
    assert!(!needs_drop::<CommonCamEdgeTelemetry>());
    assert!(!needs_drop::<CommonValidatedInputTelemetry>());
    assert!(!needs_drop::<CommonRuntimeFaultTelemetry>());
    assert!(!needs_drop::<CommonFaultTransitionTelemetry>());
    assert!(!needs_drop::<EngineTimeAuthorityTelemetry>());
    assert!(!needs_drop::<SensorSnapshot>());
    assert!(!needs_drop::<ProfileId>());
    assert!(!needs_drop::<IgnitionProfileId>());
    assert!(!needs_drop::<PinMapId>());
    assert!(!needs_drop::<RuntimeBuildId>());
    assert!(!needs_drop::<IgnitionProfileMode>());
    assert!(!needs_drop::<TelemetryFrame>());
    assert!(!needs_drop::<EdgeBatch<4>>());
    assert!(!needs_drop::<OutputTransitionBatch<4>>());
    assert!(!needs_drop::<AuxCommandBatch<4>>());
}

#[test]
fn common_diagnostics_telemetry_defaults_cleanly() {
    assert_eq!(
        CommonDiagnosticsTelemetry::default(),
        CommonDiagnosticsTelemetry {
            sync_state: CommonSyncTelemetryState::NoSignal,
            fault_code: FaultCode::None,
            fault_severity: FaultSeverity::Info,
            cancel_reason: CancelReason::Manual,
            fault: CommonRuntimeFaultTelemetry::default(),
            lambda: CommonLambdaTelemetry::default(),
            lambda_correction: CommonLambdaCorrectionTelemetry::default(),
            warmup: CommonWarmupTelemetry::default(),
            startup: CommonStartupTelemetry::default(),
            afterstart: CommonAfterstartTelemetry::default(),
            transient_enrichment: CommonTransientEnrichmentTelemetry::default(),
            protection: CommonProtectionTelemetry::default(),
            limp_action: CommonLimpActionTelemetry::default(),
            late_event_count: 0,
            max_lateness_us: 0,
            queue_high_water_mark: 0,
            last_drain_count: 0,
            active_queue_count: 0,
            free_queue_slots: 0,
            queue_capacity: 0,
        }
    );
}

#[test]
fn common_warmup_telemetry_defaults_cleanly() {
    assert_eq!(
        CommonWarmupTelemetry::default(),
        CommonWarmupTelemetry {
            active: false,
            correction_x100: 0,
            temperature_mode: CommonWarmupTemperatureMode::Inactive,
        }
    );
}

#[test]
fn common_startup_telemetry_defaults_cleanly() {
    assert_eq!(
        CommonStartupTelemetry::default(),
        CommonStartupTelemetry {
            active: false,
            remaining_window: 0,
            window_mode: CommonStartupWindowMode::Inactive,
        }
    );
}

#[test]
fn common_afterstart_telemetry_defaults_cleanly() {
    assert_eq!(
        CommonAfterstartTelemetry::default(),
        CommonAfterstartTelemetry {
            active: false,
            remaining_window: 0,
            window_mode: CommonAfterstartWindowMode::Inactive,
        }
    );
}

#[test]
fn common_transient_enrichment_telemetry_defaults_cleanly() {
    assert_eq!(
        CommonTransientEnrichmentTelemetry::default(),
        CommonTransientEnrichmentTelemetry {
            acceleration_active: false,
            acceleration_pulse_us: 0,
            acceleration_decay_steps_remaining: 0,
        }
    );
}

#[test]
fn common_lambda_telemetry_defaults_cleanly() {
    assert_eq!(
        CommonLambdaTelemetry::default(),
        CommonLambdaTelemetry {
            activity: CommonLambdaActivity::Inactive,
            reason: CommonLambdaDisableReason::None,
        }
    );
}

#[test]
fn common_lambda_correction_telemetry_defaults_cleanly() {
    assert_eq!(
        CommonLambdaCorrectionTelemetry::default(),
        CommonLambdaCorrectionTelemetry {
            measured_lambda: Lambda100::default(),
            target_lambda: Lambda100::default(),
            trim_x100: 0,
            status: CommonLambdaTelemetry::default(),
        }
    );
}

#[test]
fn common_limp_action_telemetry_defaults_cleanly() {
    assert_eq!(
        CommonLimpActionTelemetry::default(),
        CommonLimpActionTelemetry {
            level: CommonLimpActionLevel::Inactive,
            source: CommonLimpActionSource::None,
            cancel_scheduler: false,
            cancel_reason: CancelReason::Manual,
            apply_aux: false,
            aux_command_count: 0,
            persistence: CommonProtectionPersistence::Inactive,
        }
    );
}

#[test]
fn common_protection_telemetry_defaults_cleanly() {
    assert_eq!(
        CommonProtectionTelemetry::default(),
        CommonProtectionTelemetry {
            level: CommonProtectionLevel::Inactive,
            source: CommonProtectionSource::None,
            action: CommonProtectionAction::None,
            persistence: CommonProtectionPersistence::Inactive,
        }
    );
}

#[test]
fn common_runtime_fault_telemetry_defaults_cleanly() {
    assert_eq!(
        CommonRuntimeFaultTelemetry::default(),
        CommonRuntimeFaultTelemetry {
            active: false,
            fault_code: FaultCode::None,
            severity: FaultSeverity::Info,
            cancel_reason: CancelReason::Manual,
            action: CommonFaultTransitionAction::None,
        }
    );
}

#[test]
fn common_high_rate_log_telemetry_defaults_cleanly() {
    assert_eq!(
        CommonHighRateLogTelemetry::default(),
        CommonHighRateLogTelemetry {
            decision: CommonDecisionTelemetry::default(),
            fault: CommonRuntimeFaultTelemetry::default(),
            frontier_fault: CommonFrontierFaultTelemetry::default(),
            late_event_count: 0,
            max_lateness_us: 0,
            calibration_checksum: 0,
        }
    );
}

#[test]
fn common_decision_telemetry_defaults_cleanly() {
    assert_eq!(
        CommonDecisionTelemetry::default(),
        CommonDecisionTelemetry {
            control_mode: ControlMode::default(),
            rev_soft_active: false,
            rev_hard_active: false,
            launch_active: false,
            flat_shift_active: false,
            fuel_cut: false,
            spark_cut: false,
            fuel_cut_reason: CommonCutReason::None,
            spark_cut_reason: CommonCutReason::None,
        }
    );
}

#[test]
fn common_shift_arming_telemetry_defaults_cleanly() {
    assert_eq!(
        CommonShiftArmingTelemetry::default(),
        CommonShiftArmingTelemetry {
            launch_armed: false,
            flat_shift_armed: false,
        }
    );
}

#[test]
fn common_scheduler_mode_defaults_cleanly() {
    assert_eq!(CommonSchedulerMode::default(), CommonSchedulerMode::Idle);
}

#[test]
fn common_scheduler_ownership_telemetry_defaults_cleanly() {
    assert_eq!(
        CommonSchedulerOwnershipTelemetry::default(),
        CommonSchedulerOwnershipTelemetry {
            mode: CommonSchedulerMode::Idle,
            active_groups: 0,
            injection_count: 0,
            ignition_count: 0,
        }
    );
}

#[test]
fn common_scheduler_reservation_telemetry_defaults_cleanly() {
    assert_eq!(
        CommonSchedulerReservationTelemetry::default(),
        CommonSchedulerReservationTelemetry {
            injector_channels: 0,
            ignition_channels: 0,
            idle_channels: 0,
            fan_channels: 0,
        }
    );
}

#[test]
fn common_scheduler_state_summary_telemetry_defaults_cleanly() {
    assert_eq!(
        CommonSchedulerStateSummaryTelemetry::default(),
        CommonSchedulerStateSummaryTelemetry { armed: false }
    );
}

#[test]
fn common_scheduler_window_telemetry_defaults_cleanly() {
    assert_eq!(
        CommonSchedulerWindowTelemetry::default(),
        CommonSchedulerWindowTelemetry {
            last_injection_start: None,
            last_injection_end: None,
            last_ignition_start: None,
            last_ignition_end: None,
        }
    );
}

#[test]
fn common_pending_input_telemetry_defaults_cleanly() {
    assert_eq!(
        CommonPendingInputTelemetry::default(),
        CommonPendingInputTelemetry {
            now_us: Micros::new(0),
            rpm: Rpm::default(),
            load_kpa10: Kpa10::default(),
            angle_x10: Degrees10::default(),
            authority: EngineTimeAuthority::default(),
        }
    );
}

#[test]
fn common_pending_input_telemetry_is_copy_and_does_not_need_drop() {
    fn assert_copy<T: Copy>() {}

    assert_copy::<CommonPendingInputTelemetry>();
    assert!(!needs_drop::<CommonPendingInputTelemetry>());
}

#[test]
fn common_control_telemetry_defaults_cleanly() {
    assert_eq!(
        CommonControlTelemetry::default(),
        CommonControlTelemetry {
            fuel_pulse_width: PulseWidthUs::default(),
            ignition_advance: Degrees10::default(),
            dwell: DwellUs::default(),
            lambda_target: Lambda100::default(),
            torque_limit_x100: u16::default(),
        }
    );
}

#[test]
fn common_frontier_telemetry_defaults_cleanly() {
    assert_eq!(
        CommonFrontierTelemetry::default(),
        CommonFrontierTelemetry {
            active_horizon_id: None,
            horizon_start_us: None,
            horizon_end_us: None,
            last_accepted_horizon_id: None,
            last_accepted_horizon_start_us: None,
            last_accepted_horizon_end_us: None,
            heartbeat_deadline_us: None,
            active_permit_mask: TimingIslandPermitMask::NONE,
            active_stop_reason: TimingIslandStopReason::None,
            fault: CommonFrontierFaultTelemetry::default(),
        }
    );
}

#[test]
fn timing_island_frontier_contract_constants_are_stable() {
    assert_eq!(HEARTBEAT_EXPIRY_US, Micros::new(20_000));
    assert_eq!(MAX_HORIZON_US, Micros::new(10_000));
    assert_eq!(HORIZON_SEQUENCE_BITS, 32);
    assert_eq!(TimingIslandStopReason::None as u8, 0);
    assert_eq!(TimingIslandStopReason::SyncLost as u8, 1);
    assert_eq!(TimingIslandStopReason::HeartbeatExpired as u8, 2);
    assert_eq!(TimingIslandStopReason::HorizonExpired as u8, 3);
    assert_eq!(TimingIslandStopReason::PermitDenied as u8, 4);
    assert_eq!(TimingIslandStopReason::TimingFault as u8, 5);
    assert_eq!(TimingIslandStopReason::AdmittedEventRejected as u8, 6);
    assert_eq!(TimingIslandStopReason::BoardOutputFault as u8, 7);
}

#[test]
fn timing_island_permit_mask_filters_unknown_bits_and_defaults_to_deny() {
    assert_eq!(TimingIslandPermitMask::NONE.bits(), 0);
    assert!(TimingIslandPermitMask::NONE.is_empty());
    assert_eq!(TimingIslandPermitMask::IGNITION, 1 << 0);
    assert_eq!(TimingIslandPermitMask::INJECTOR, 1 << 1);
    assert_eq!(TimingIslandPermitMask::BOUNDED_AUX, 1 << 2);
    assert_eq!(
        TimingIslandPermitMask::KNOWN_BITS,
        TimingIslandPermitMask::IGNITION
            | TimingIslandPermitMask::INJECTOR
            | TimingIslandPermitMask::BOUNDED_AUX
    );
    assert_eq!(TimingIslandPermitMask::KNOWN_BITS & (1 << 3), 0);
    assert_eq!(TimingIslandPermitMask::new(1 << 3).bits(), 0);
    assert_eq!(
        TimingIslandPermitMask::new(u32::MAX).bits(),
        TimingIslandPermitMask::KNOWN_BITS
    );
    assert_eq!(
        TimingIslandPermitMask::new(TimingIslandPermitMask::KNOWN_BITS | (1 << 31)).bits(),
        TimingIslandPermitMask::KNOWN_BITS
    );
}

#[test]
fn timing_island_metric_snapshot_exposes_minimum_frontier_fields() {
    let snapshot = TimingIslandMetricSnapshot::new(
        SyncState::Locked { cam_ref: true },
        TimingIslandSyncLossReason::DecoderFault,
        true,
        Some(42),
        Some(Micros::new(1_000)),
        Some(Micros::new(500)),
        TimingIslandPermitMask::ALL,
        TimingIslandStopReason::AdmittedEventRejected,
        3,
        7,
    );

    assert_eq!(snapshot.sync_state, SyncState::Locked { cam_ref: true });
    assert_eq!(
        snapshot.sync_loss_reason,
        TimingIslandSyncLossReason::DecoderFault
    );
    assert!(snapshot.phase_freshness);
    assert_eq!(snapshot.last_accepted_horizon_id, Some(42));
    assert_eq!(
        snapshot.last_accepted_horizon_age_us,
        Some(Micros::new(1_000))
    );
    assert_eq!(snapshot.heartbeat_age_us, Some(Micros::new(500)));
    assert_eq!(snapshot.active_permit_mask, TimingIslandPermitMask::ALL);
    assert_eq!(
        snapshot.active_stop_reason,
        TimingIslandStopReason::AdmittedEventRejected
    );
    assert_eq!(snapshot.dropped_or_rejected_event_count, 3);
    assert_eq!(snapshot.late_event_count, 7);
}

#[test]
fn common_frontier_fault_telemetry_defaults_cleanly() {
    assert_eq!(
        CommonFrontierFaultTelemetry::default(),
        CommonFrontierFaultTelemetry {
            event_id: CommonFrontierFaultEventId::None,
            severity: FaultSeverity::Info,
            action: CommonFrontierFaultAction::None,
        }
    );
}

#[test]
fn common_control_reason_enums_default_cleanly() {
    assert_eq!(CommonLambdaMode::default(), CommonLambdaMode::OpenLoop);
    assert_eq!(
        CommonLambdaActivity::default(),
        CommonLambdaActivity::Inactive
    );
    assert_eq!(
        CommonLambdaDisableReason::default(),
        CommonLambdaDisableReason::None
    );
    assert_eq!(
        CommonIgnitionLimitReason::default(),
        CommonIgnitionLimitReason::None
    );
    assert_eq!(
        CommonTorqueLimitReason::default(),
        CommonTorqueLimitReason::None
    );
}

#[test]
fn common_control_reason_telemetry_defaults_cleanly() {
    assert_eq!(
        CommonControlReasonTelemetry::default(),
        CommonControlReasonTelemetry {
            lambda_mode: CommonLambdaMode::OpenLoop,
            lambda_active: false,
            lambda_trim_x100: 0,
            lambda_disable_reason: CommonLambdaDisableReason::None,
            ignition_limit_reason: CommonIgnitionLimitReason::None,
            torque_limit_reason: CommonTorqueLimitReason::None,
        }
    );
}

#[test]
fn common_fuel_strategy_mode_defaults_cleanly() {
    assert_eq!(
        CommonFuelStrategyMode::default(),
        CommonFuelStrategyMode::DirectPulseWidthTable
    );
}

#[test]
fn common_fuel_strategy_mode_is_copy_and_does_not_need_drop() {
    fn assert_copy<T: Copy>() {}

    assert_copy::<CommonFuelStrategyMode>();
    assert!(!needs_drop::<CommonFuelStrategyMode>());
}

#[test]
fn common_fuel_observation_telemetry_defaults_cleanly() {
    assert_eq!(
        CommonFuelObservationTelemetry::default(),
        CommonFuelObservationTelemetry {
            base_fuel_pulse_width: PulseWidthUs::default(),
            enriched_fuel_pulse_width: PulseWidthUs::default(),
        }
    );
}

#[test]
fn common_enrichment_telemetry_defaults_cleanly() {
    assert_eq!(
        CommonEnrichmentTelemetry::default(),
        CommonEnrichmentTelemetry {
            startup_x100: 0,
            warmup_x100: 0,
            after_start_x100: 0,
            acceleration_x100: 0,
            total_x100: 0,
        }
    );
}

#[test]
fn common_action_telemetry_defaults_cleanly() {
    assert_eq!(
        CommonActionTelemetry::default(),
        CommonActionTelemetry {
            total_action_count: 0,
            arm_scheduler_count: 0,
            arm_injection_count: 0,
            arm_ignition_count: 0,
            apply_aux_count: 0,
            apply_aux_command_count: 0,
            idle_count: 0,
            publish_snapshot_count: 0,
            publish_snapshot: false,
            persist_calibration: false,
            persist_calibration_count: 0,
            cancel_scheduler: false,
            cancel_reason: CancelReason::Manual,
            cancel_scheduler_count: 0,
            multiple_cancel_reasons: false,
        }
    );
}

#[test]
fn common_torque_telemetry_defaults_cleanly() {
    assert_eq!(
        CommonTorqueTelemetry::default(),
        CommonTorqueTelemetry {
            request_x1000: u16::default(),
            allowed_x1000: u16::default(),
            actuated_x1000: u16::default(),
        }
    );
}

#[test]
fn common_engine_telemetry_defaults_cleanly() {
    assert_eq!(
        CommonEngineTelemetry::default(),
        CommonEngineTelemetry {
            rpm: Rpm::default(),
            load_kpa10: Kpa10::default(),
            angle_x10: Degrees10::default(),
            phase: EnginePhase::default(),
        }
    );
}

#[test]
fn common_trigger_edge_telemetry_defaults_cleanly() {
    assert_eq!(
        CommonTriggerEdgeTelemetry::default(),
        CommonTriggerEdgeTelemetry {
            seen: false,
            at_us: Micros::new(0),
            rpm: Rpm::default(),
            angle_x10: Degrees10::default(),
            authority: EngineTimeAuthority::none(),
            synced: false,
        }
    );
}

#[test]
fn common_cam_edge_telemetry_defaults_cleanly() {
    assert_eq!(
        CommonCamEdgeTelemetry::default(),
        CommonCamEdgeTelemetry {
            seen: false,
            at_us: Micros::new(0),
            cam_seen: false,
        }
    );
}

#[test]
fn common_validated_input_telemetry_defaults_cleanly() {
    assert_eq!(
        CommonValidatedInputTelemetry::default(),
        CommonValidatedInputTelemetry {
            rpm: Rpm::default(),
            load_kpa10: Kpa10::default(),
            angle_x10: Degrees10::default(),
            clamped: false,
        }
    );
}

#[test]
fn common_fault_transition_telemetry_defaults_cleanly() {
    assert_eq!(
        CommonFaultTransitionTelemetry::default(),
        CommonFaultTransitionTelemetry {
            changed: false,
            at_us: Micros::new(0),
            event: CommonFaultTransitionEventTelemetry::default(),
            previous_fault: FaultCode::None,
            previous_severity: FaultSeverity::Info,
            previous_cancel_reason: CancelReason::Manual,
            current_fault: FaultCode::None,
            current_severity: FaultSeverity::Info,
            current_cancel_reason: CancelReason::Manual,
        }
    );
}

#[test]
fn common_fault_transition_event_telemetry_defaults_cleanly() {
    assert_eq!(
        CommonFaultTransitionEventTelemetry::default(),
        CommonFaultTransitionEventTelemetry {
            event_id: CommonFaultTransitionEventId::None,
            severity: FaultSeverity::Info,
            action: CommonFaultTransitionAction::None,
        }
    );
}

#[test]
fn common_sync_telemetry_state_is_explicit_and_stable() {
    assert_eq!(CommonSyncTelemetryState::NoSignal as u8, 0);
    assert_eq!(CommonSyncTelemetryState::Unsynced as u8, 1);
    assert_eq!(CommonSyncTelemetryState::CrankSynced as u8, 2);
    assert_eq!(CommonSyncTelemetryState::FullSequentialAuthorized as u8, 3);
    assert_eq!(CommonSyncTelemetryState::SyncLost as u8, 4);
    assert_eq!(CommonSyncTelemetryState::CamSynced as u8, 5);
    assert_eq!(CommonSyncTelemetryState::SyncSuspect as u8, 6);
}

#[test]
fn aux_output_generic_variants_construct_and_round_trip() {
    let variants = [
        AuxOutput::SafetyRelay(0),
        AuxOutput::SafetyRelay(1),
        AuxOutput::SafetyRelay(7),
        AuxOutput::Indicator(0),
        AuxOutput::Indicator(2),
        AuxOutput::FrequencyOut(0),
        AuxOutput::FrequencyOut(3),
        AuxOutput::Digital(ChannelId::new(7)),
        AuxOutput::Digital(ChannelId::new(8)),
        AuxOutput::Pwm(ChannelId::new(8)),
        AuxOutput::Digital(ChannelId::new(9)),
        AuxOutput::Pwm(ChannelId::new(10)),
    ];

    let mut batch = AuxCommandBatch::<12>::new();
    for (idx, output) in variants.into_iter().enumerate() {
        assert!(batch
            .push(AuxCommand::new(
                output,
                if idx % 2 == 0 {
                    AuxValue::Off
                } else {
                    AuxValue::Level(OutputLevel::High)
                },
            ))
            .is_ok());
    }

    assert_eq!(batch.len(), 12);
    assert_eq!(
        batch.as_slice()[9].output,
        AuxOutput::Pwm(ChannelId::new(8))
    );
    assert_eq!(
        batch.as_slice()[10].output,
        AuxOutput::Digital(ChannelId::new(9))
    );
    assert_eq!(
        batch.as_slice()[11].output,
        AuxOutput::Pwm(ChannelId::new(10))
    );
}

#[test]
fn telemetry_frame_identity_fields_round_trip() {
    let snapshot = SensorSnapshot::new(
        Micros::new(123),
        Rpm::new(456),
        Kpa10::new(789),
        Percent::new(12),
        34,
        56,
        12_345,
        Lambda100::new(98),
        SyncState::Locked { cam_ref: false },
        EnginePhase::Running,
    );

    let frame = TelemetryFrame::new(
        snapshot,
        ProfileId::new(11),
        IgnitionProfileId::new(22),
        IgnitionProfileMode::SequentialCop,
        PinMapId::new(33),
        RuntimeBuildId::new(44),
        ControlMode::ClosedLoop,
        FaultCode::default(),
        FaultSeverity::default(),
        Degrees10::new(15),
        DwellUs::new(2500),
        PulseWidthUs::new(3700),
    );

    assert_eq!(frame.snapshot, snapshot);
    assert_eq!(frame.profile_id, ProfileId::new(11));
    assert_eq!(frame.ignition_profile_id, IgnitionProfileId::new(22));
    assert_eq!(
        frame.ignition_profile_mode,
        IgnitionProfileMode::SequentialCop
    );
    assert_eq!(frame.pin_map_id, PinMapId::new(33));
    assert_eq!(frame.runtime_build_id, RuntimeBuildId::new(44));
    assert_eq!(frame.control_mode, ControlMode::ClosedLoop);
    assert_eq!(
        frame.snapshot.engine_time.source(),
        AbsoluteTimeAuthority::GeometryOnly
    );
    assert!(!frame.snapshot.engine_time.full_sequential_authorized);
}

#[test]
fn telemetry_reports_explicit_engine_time_authority_source_and_gate() {
    let authority = EngineTimeAuthority::new(
        CrankSyncState::PrimaryLocked,
        PhaseSyncState::CamValidated720,
        AbsoluteTimeAuthority::ExpertManual,
        EngineTimeAuthority::MAX_CONFIDENCE_X1000,
        0,
    );
    let snapshot = SensorSnapshot::new_with_engine_time_authority(
        Micros::new(123),
        Rpm::new(456),
        Kpa10::new(789),
        Percent::new(12),
        34,
        56,
        12_345,
        Lambda100::new(98),
        authority,
        EnginePhase::Running,
    );

    assert_eq!(snapshot.sync_state, SyncState::Locked { cam_ref: false });
    assert_eq!(
        snapshot.engine_time.summary,
        SyncState::Locked { cam_ref: false }
    );
    assert_eq!(
        snapshot.engine_time.source(),
        AbsoluteTimeAuthority::ExpertManual
    );
    assert!(snapshot.engine_time.full_sequential_authorized);
}

#[test]
fn geometry_only_authority_does_not_authorize_full_sequential() {
    let authority = EngineTimeAuthority::new(
        CrankSyncState::PrimaryLocked,
        PhaseSyncState::CamValidated720,
        AbsoluteTimeAuthority::GeometryOnly,
        EngineTimeAuthority::MAX_CONFIDENCE_X1000,
        0,
    );
    let telemetry = EngineTimeAuthorityTelemetry::new(authority);

    assert_eq!(telemetry.summary, SyncState::Locked { cam_ref: false });
    assert_eq!(telemetry.source(), AbsoluteTimeAuthority::GeometryOnly);
    assert!(!telemetry.full_sequential_authorized);
}

#[test]
fn timing_island_command_wire_round_trips_all_variants() {
    let commands = [
        TimingIslandCommand::ArmOutput(OutputTransition::new(
            EcuOutput::Injector(ChannelId::new(1)),
            OutputLevel::High,
            Ticks::new(100),
        )),
        TimingIslandCommand::ApplyAux(AuxCommand::new(
            AuxOutput::Pwm(ChannelId::new(2)),
            AuxValue::Duty(Percent::new(45)),
        )),
        TimingIslandCommand::CancelAll(CancelReason::SafetyShutdown),
        TimingIslandCommand::FeedWatchdog,
        TimingIslandCommand::UpdatePermitMask(SafetyPermitMask::new(0x55aa_00ff)),
    ];

    for command in commands {
        let mut bytes = [0u8; TIMING_ISLAND_COMMAND_WIRE_LEN];

        assert_eq!(
            encode_timing_island_command(command, &mut bytes),
            Ok(TIMING_ISLAND_COMMAND_WIRE_LEN)
        );
        assert_eq!(decode_timing_island_command(&bytes), Ok(command));
    }
}

#[test]
fn timing_island_event_wire_round_trips_representative_variants() {
    let authority = EngineTimeAuthority::new(
        CrankSyncState::PrimaryLocked,
        PhaseSyncState::CamValidated720,
        AbsoluteTimeAuthority::CertifiedProfile,
        EngineTimeAuthority::MAX_CONFIDENCE_X1000,
        3,
    );
    let transition = OutputTransition::new(
        EcuOutput::Ignition(ChannelId::new(4)),
        OutputLevel::Low,
        Ticks::new(2_000),
    );
    let events = [
        TimingIslandEvent::OutputArmed(transition),
        TimingIslandEvent::OutputCompleted(transition),
        TimingIslandEvent::AuxApplied(AuxCommand::new(
            AuxOutput::SafetyRelay(0),
            AuxValue::Level(OutputLevel::High),
        )),
        TimingIslandEvent::Cancelled(CancelReason::SyncLoss),
        TimingIslandEvent::Rejected(TimingIslandRejectReason::BackendFault),
        TimingIslandEvent::CrankEdge(TriggerEdge::new(EdgeKind::Rising, Ticks::new(30))),
        TimingIslandEvent::CamEdge(TriggerEdge::new(EdgeKind::Falling, Ticks::new(60))),
        TimingIslandEvent::SyncStatus(EngineTimeAuthorityTelemetry::new(authority)),
        TimingIslandEvent::FaultStatus(TimingIslandFaultStatus::new(
            FaultCode::SafetyCut,
            FaultSeverity::Critical,
            Ticks::new(90),
        )),
    ];

    for event in events {
        let mut bytes = [0u8; TIMING_ISLAND_EVENT_WIRE_LEN];

        assert_eq!(
            encode_timing_island_event(event, &mut bytes),
            Ok(TIMING_ISLAND_EVENT_WIRE_LEN)
        );
        assert_eq!(decode_timing_island_event(&bytes), Ok(event));
    }
}

#[test]
fn safety_gate_wire_round_trips_input_and_status() {
    let input = SafetyGateInput {
        now_us: Micros::new(1234),
        kill_n: true,
        power_good: true,
        watchdog_ok: false,
        timing_backend_alive: true,
        backend_alive: false,
        sync_authority_ok: true,
        driver_faults: 0x11,
        requested_permit_mask: SafetyPermitMask::new(0x33),
    };
    let status = SafetyGateStatus::denied(
        SafetyGateReason::WatchdogTimeout,
        FaultCode::SafetyCut,
        FaultSeverity::Critical,
        0x11,
        Micros::new(1234),
    );
    let mut input_bytes = [0u8; SAFETY_GATE_INPUT_WIRE_LEN];
    let mut status_bytes = [0u8; SAFETY_GATE_STATUS_WIRE_LEN];

    assert_eq!(
        encode_safety_gate_input(input, &mut input_bytes),
        Ok(SAFETY_GATE_INPUT_WIRE_LEN)
    );
    assert_eq!(decode_safety_gate_input(&input_bytes), Ok(input));
    assert_eq!(
        encode_safety_gate_status(status, &mut status_bytes),
        Ok(SAFETY_GATE_STATUS_WIRE_LEN)
    );
    assert_eq!(decode_safety_gate_status(&status_bytes), Ok(status));
}

#[test]
fn wire_codecs_reject_malformed_frames() {
    let mut command = [0u8; TIMING_ISLAND_COMMAND_WIRE_LEN];
    let mut event = [0u8; TIMING_ISLAND_EVENT_WIRE_LEN];
    let mut safety_input = [0u8; SAFETY_GATE_INPUT_WIRE_LEN];
    let mut safety_status = [0u8; SAFETY_GATE_STATUS_WIRE_LEN];

    assert_eq!(
        encode_timing_island_command(TimingIslandCommand::FeedWatchdog, &mut command[..3]),
        Err(TimingIslandCodecError::ShortBuffer)
    );
    assert_eq!(
        decode_timing_island_command(&command[..3]),
        Err(TimingIslandCodecError::ShortBuffer)
    );

    command[0] = TIMING_ISLAND_WIRE_VERSION + 1;
    command[1] = TIMING_ISLAND_COMMAND_TAG_FEED_WATCHDOG;
    assert_eq!(
        decode_timing_island_command(&command),
        Err(TimingIslandCodecError::UnsupportedVersion(
            TIMING_ISLAND_WIRE_VERSION + 1
        ))
    );

    command[0] = TIMING_ISLAND_WIRE_VERSION;
    command[1] = 0xff;
    assert_eq!(
        decode_timing_island_command(&command),
        Err(TimingIslandCodecError::UnknownTag(0xff))
    );

    command[1] = TIMING_ISLAND_COMMAND_TAG_ARM_OUTPUT;
    command[2] = 0xff;
    assert_eq!(
        decode_timing_island_command(&command),
        Err(TimingIslandCodecError::InvalidField)
    );

    event[0] = TIMING_ISLAND_WIRE_VERSION;
    event[1] = TIMING_ISLAND_EVENT_TAG_REJECTED;
    event[2] = 0xff;
    assert_eq!(
        decode_timing_island_event(&event),
        Err(TimingIslandCodecError::InvalidField)
    );

    safety_input[0] = TIMING_ISLAND_WIRE_VERSION;
    safety_input[1] = 0b1100_0000;
    assert_eq!(
        decode_safety_gate_input(&safety_input),
        Err(TimingIslandCodecError::InvalidField)
    );

    safety_status[0] = TIMING_ISLAND_WIRE_VERSION;
    safety_status[1] = 0xff;
    assert_eq!(
        decode_safety_gate_status(&safety_status),
        Err(TimingIslandCodecError::InvalidField)
    );
}

#[test]
fn output_scheduler_safe_state_has_fallible_observable_hook() {
    #[derive(Default)]
    struct Scheduler {
        safe_state_forced: bool,
    }

    impl OutputScheduler<1> for Scheduler {
        type Error = ();

        fn schedule(&mut self, _batch: &OutputTransitionBatch<1>) -> Result<(), Self::Error> {
            Ok(())
        }

        fn cancel_all(&mut self) -> Result<(), Self::Error> {
            Ok(())
        }

        fn force_safe_state(&mut self) {
            self.safe_state_forced = true;
        }
    }

    #[derive(Default)]
    struct FallibleScheduler;

    impl OutputScheduler<1> for FallibleScheduler {
        type Error = &'static str;

        fn schedule(&mut self, _batch: &OutputTransitionBatch<1>) -> Result<(), Self::Error> {
            Ok(())
        }

        fn cancel_all(&mut self) -> Result<(), Self::Error> {
            Ok(())
        }

        fn force_safe_state(&mut self) {}

        fn try_force_safe_state(&mut self) -> Result<(), Self::Error> {
            Err("driver fault")
        }
    }

    let mut scheduler = Scheduler::default();
    assert_eq!(scheduler.try_force_safe_state(), Ok(()));
    assert!(scheduler.safe_state_forced);

    let mut fallible = FallibleScheduler;
    assert_eq!(fallible.try_force_safe_state(), Err("driver fault"));
}
