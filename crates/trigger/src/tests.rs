use super::*;
use ecu_domain::{
    AbsoluteTimeAuthority, CrankSyncState, Degrees10, EngineTimeAuthority, Micros, PhaseSyncState,
    Rpm, SyncState, Ticks,
};
use proptest::prelude::*;

const VALID_60_MINUS_2: TriggerProfile = TriggerProfile {
    pattern: TriggerPattern::MissingTooth {
        nominal_teeth: 60,
        missing_teeth: 2,
    },
    primary_speed: TriggerSpeed::Crank,
    primary_edge: TriggerEdge::Rising,
    secondary: SecondaryTriggerProfile {
        mode: SecondaryTriggerMode::SingleToothCam,
        edge: TriggerEdge::Falling,
        poll_level: PollLevelPolarity::ActiveHigh,
    },
    trigger_angle_atdc_deg10: TriggerAngleAuthority::Unknown,
    tooth_angle_multiplier: 1,
    filter: TriggerFilter::Weak,
    resync: ResyncPolicy::OnSyncLoss,
    startup: StartupSyncPolicy {
        skip_revolutions: 2,
        require_full_cycle: true,
    },
    latency: EngineTimeLatency {
        primary_edge_delay_us: Micros::new(0),
        secondary_edge_delay_us: Micros::new(0),
        output_schedule_delay_us: Micros::new(0),
    },
};

#[test]
fn missing_tooth_profile_validates_and_reports_observed_teeth() {
    assert_eq!(VALID_60_MINUS_2.validate(), Ok(()));
    assert_eq!(VALID_60_MINUS_2.pattern.observed_primary_teeth(), Some(58));
}

#[test]
fn model_sync_thresholds_match_implementation() {
    assert_eq!(MODEL_MAX_GOOD, 2);
    assert_eq!(MODEL_MAX_BAD, 2);
    assert_eq!(
        VALID_60_MINUS_2.startup.skip_revolutions, MODEL_MAX_GOOD,
        "reference profile sync-acquire threshold must equal trigger.tla MaxGood"
    );
    assert!(VALID_60_MINUS_2.startup.authority_ready(MODEL_MAX_GOOD));
    assert!(!VALID_60_MINUS_2
        .startup
        .authority_ready(MODEL_MAX_GOOD.saturating_sub(1)));
}

#[test]
fn unknown_trigger_angle_cannot_claim_certified_authority() {
    assert_eq!(
        TriggerAngleAuthority::Unknown.absolute_authority(),
        AbsoluteTimeAuthority::None
    );
    assert_eq!(
        VALID_60_MINUS_2.declared_absolute_authority(),
        AbsoluteTimeAuthority::None
    );

    let certified = TriggerAngleAuthority::CertifiedProfile(Degrees10::new(840));
    assert_eq!(
        certified.absolute_authority(),
        AbsoluteTimeAuthority::CertifiedProfile
    );
}

#[test]
fn angle_normalization_stays_in_engine_cycle_range() {
    assert_eq!(normalize_engine_cycle_deg10(Degrees10::new(0)).get(), 0);
    assert_eq!(normalize_engine_cycle_deg10(Degrees10::new(7200)).get(), 0);
    assert_eq!(
        normalize_engine_cycle_deg10(Degrees10::new(-1)).get(),
        ENGINE_CYCLE_DEGREES10 - 1
    );
    assert_eq!(
        TriggerAngleAuthority::ExpertManual(Degrees10::new(7215))
            .normalized_angle_deg10()
            .map(Degrees10::get),
        Some(15)
    );
}

#[test]
fn validation_rejects_invalid_missing_tooth_combinations() {
    assert_eq!(
        TriggerPattern::MissingTooth {
            nominal_teeth: 0,
            missing_teeth: 1,
        }
        .validate(),
        Err(TriggerValidationError::NominalTeethZero)
    );
    assert_eq!(
        TriggerPattern::MissingTooth {
            nominal_teeth: 60,
            missing_teeth: 0,
        }
        .validate(),
        Err(TriggerValidationError::MissingTeethZero)
    );
    assert_eq!(
        TriggerPattern::MissingTooth {
            nominal_teeth: 2,
            missing_teeth: 2,
        }
        .validate(),
        Err(TriggerValidationError::MissingTeethNotLessThanNominal)
    );
    assert_eq!(
        TriggerPattern::MissingTooth {
            nominal_teeth: 2,
            missing_teeth: 1,
        }
        .validate(),
        Err(TriggerValidationError::NotEnoughObservedTeeth)
    );
}

#[test]
fn validation_rejects_other_zero_sized_profile_parts() {
    assert_eq!(
        TriggerPattern::DualWheel { primary_teeth: 0 }.validate(),
        Err(TriggerValidationError::PrimaryTeethZero)
    );
    assert_eq!(
        TriggerPattern::NonUniform {
            pattern_id: PatternId::new(0),
        }
        .validate(),
        Err(TriggerValidationError::PatternIdZero)
    );
    assert_eq!(
        SecondaryTriggerMode::MultiToothCam { teeth: 0 }.validate(),
        Err(TriggerValidationError::SecondaryTeethZero)
    );
    assert_eq!(
        TriggerProfile {
            tooth_angle_multiplier: 0,
            ..VALID_60_MINUS_2
        }
        .validate(),
        Err(TriggerValidationError::ToothAngleMultiplierZero)
    );
}

#[test]
fn diagnostics_defaults_are_safe_and_allocation_free() {
    let diagnostics = TriggerDiagnostics::default();
    assert_eq!(diagnostics.observation, DecoderObservation::default());
    assert_eq!(diagnostics.authority, EngineTimeAuthority::none());
    assert_eq!(diagnostics.last_sync_loss, None);
    assert_eq!(diagnostics.authority.crank, CrankSyncState::NoSignal);
    assert_eq!(diagnostics.authority.phase, PhaseSyncState::Unknown);
}

fn decoder_config(trigger_angle: Degrees10) -> MissingToothDecoderConfig {
    MissingToothDecoderConfig {
        nominal_teeth: 60,
        missing_teeth: 2,
        primary_speed: TriggerSpeed::Crank,
        primary_edge: TriggerEdge::Rising,
        secondary: RuntimeSecondaryTriggerProfile {
            mode: RuntimeSecondaryTriggerMode::None,
            edge: TriggerEdge::Rising,
            poll_level: PollLevelPolarity::ActiveHigh,
        },
        trigger_angle_atdc_deg10: TriggerAngleAuthority::ExpertManual(trigger_angle),
        tooth_angle_multiplier: 1,
        minimum_edge_interval: Ticks::new(50),
        gap_ratio_threshold_x1000: DEFAULT_MISSING_TOOTH_GAP_RATIO_X1000,
    }
}

fn new_decoder(config: MissingToothDecoderConfig) -> MissingToothDecoder {
    match MissingToothDecoder::try_new(config) {
        Ok(decoder) => decoder,
        Err(error) => panic!("decoder config rejected: {error:?}"),
    }
}

fn lock_60_minus_2(decoder: &mut MissingToothDecoder) {
    assert_eq!(
        decoder.ingest_primary_edge(Ticks::new(0)),
        Ok(MissingToothDecoderEvent::FirstEdge)
    );
    assert_eq!(
        decoder.ingest_primary_edge(Ticks::new(1000)),
        Ok(MissingToothDecoderEvent::Searching)
    );
    assert_eq!(
        decoder.ingest_primary_edge(Ticks::new(4000)),
        Ok(MissingToothDecoderEvent::Gap { current_tooth: 1 })
    );
}

fn single_tooth_cam_config(trigger_angle: Degrees10) -> MissingToothDecoderConfig {
    MissingToothDecoderConfig {
        secondary: RuntimeSecondaryTriggerProfile {
            mode: RuntimeSecondaryTriggerMode::SingleToothCam,
            edge: TriggerEdge::Falling,
            poll_level: PollLevelPolarity::ActiveHigh,
        },
        ..decoder_config(trigger_angle)
    }
}

fn poll_level_config(trigger_angle: Degrees10) -> MissingToothDecoderConfig {
    MissingToothDecoderConfig {
        secondary: RuntimeSecondaryTriggerProfile {
            mode: RuntimeSecondaryTriggerMode::PollLevel,
            edge: TriggerEdge::Rising,
            poll_level: PollLevelPolarity::ActiveHigh,
        },
        ..decoder_config(trigger_angle)
    }
}

fn profiled_config(
    trigger_angle: Degrees10,
    secondary: RuntimeSecondaryTriggerProfile,
    startup: StartupSyncPolicy,
    latency: EngineTimeLatency,
) -> RuntimeMissingToothProfile {
    RuntimeMissingToothProfile {
        nominal_teeth: 60,
        missing_teeth: 2,
        primary_speed: TriggerSpeed::Crank,
        primary_edge: TriggerEdge::Rising,
        secondary,
        trigger_angle_atdc_deg10: TriggerAngleAuthority::ExpertManual(trigger_angle),
        tooth_angle_multiplier: 1,
        startup,
        latency,
    }
}

fn import_profile(
    trigger_angle: Degrees10,
    secondary: SecondaryTriggerProfile,
    filter: TriggerFilter,
    resync: ResyncPolicy,
) -> TriggerProfile {
    TriggerProfile {
        pattern: TriggerPattern::MissingTooth {
            nominal_teeth: 60,
            missing_teeth: 2,
        },
        primary_speed: TriggerSpeed::Crank,
        primary_edge: TriggerEdge::Rising,
        secondary,
        trigger_angle_atdc_deg10: TriggerAngleAuthority::ExpertManual(trigger_angle),
        tooth_angle_multiplier: 1,
        filter,
        resync,
        startup: StartupSyncPolicy::default(),
        latency: EngineTimeLatency::default(),
    }
}

fn profiled_decoder(profile: RuntimeMissingToothProfile) -> ProfiledMissingToothDecoder {
    match ProfiledMissingToothDecoder::try_new(
        profile,
        Ticks::new(50),
        DEFAULT_MISSING_TOOTH_GAP_RATIO_X1000,
    ) {
        Ok(decoder) => decoder,
        Err(error) => panic!("profile rejected: {error:?}"),
    }
}

fn first_profile_lock(decoder: &mut ProfiledMissingToothDecoder) {
    assert_eq!(
        decoder.ingest_primary_edge(Ticks::new(0)),
        Ok(MissingToothDecoderEvent::FirstEdge)
    );
    assert_eq!(
        decoder.ingest_primary_edge(Ticks::new(1000)),
        Ok(MissingToothDecoderEvent::Searching)
    );
    assert_eq!(
        decoder.ingest_primary_edge(Ticks::new(4000)),
        Ok(MissingToothDecoderEvent::Gap { current_tooth: 1 })
    );
}

fn drive_missing_tooth_cycle(
    decoder: &mut ProfiledMissingToothDecoder,
    mut timestamp: u32,
    normal_interval: u32,
) -> u32 {
    for expected_tooth in 2..=58 {
        timestamp = timestamp.saturating_add(normal_interval);
        assert_eq!(
            decoder.ingest_primary_edge(Ticks::new(timestamp)),
            Ok(MissingToothDecoderEvent::Tooth {
                current_tooth: expected_tooth,
            })
        );
    }

    timestamp = timestamp.saturating_add(normal_interval * 2);
    assert_eq!(
        decoder.ingest_primary_edge(Ticks::new(timestamp)),
        Ok(MissingToothDecoderEvent::Gap { current_tooth: 1 })
    );
    timestamp
}

#[test]
fn profiled_decoder_rejects_unsupported_filter_policy() {
    let profile = import_profile(
        Degrees10::new(840),
        SecondaryTriggerProfile::default(),
        TriggerFilter::Weak,
        ResyncPolicy::OnSyncLoss,
    );

    assert_eq!(
        ProfiledMissingToothDecoder::try_new_from_import_profile(
            profile,
            Ticks::new(50),
            DEFAULT_MISSING_TOOTH_GAP_RATIO_X1000,
        ),
        Err(TriggerValidationError::UnsupportedProfileFilter)
    );
}

#[test]
fn profiled_decoder_rejects_unsupported_resync_policy() {
    for resync in [ResyncPolicy::Disabled, ResyncPolicy::EveryCycle] {
        let profile = import_profile(
            Degrees10::new(840),
            SecondaryTriggerProfile::default(),
            TriggerFilter::Off,
            resync,
        );

        assert_eq!(
            ProfiledMissingToothDecoder::try_new_from_import_profile(
                profile,
                Ticks::new(50),
                DEFAULT_MISSING_TOOTH_GAP_RATIO_X1000,
            ),
            Err(TriggerValidationError::UnsupportedResyncPolicy)
        );
    }
}

#[test]
fn import_profile_validation_does_not_make_unsupported_secondary_runtime_ready() {
    for secondary_mode in [
        SecondaryTriggerMode::FourMinusOneCam,
        SecondaryTriggerMode::MultiToothCam { teeth: 4 },
        SecondaryTriggerMode::OemPattern {
            pattern_id: PatternId::new(7),
        },
    ] {
        let profile = import_profile(
            Degrees10::new(840),
            SecondaryTriggerProfile {
                mode: secondary_mode,
                edge: TriggerEdge::Rising,
                poll_level: PollLevelPolarity::ActiveHigh,
            },
            TriggerFilter::Off,
            ResyncPolicy::OnSyncLoss,
        );

        assert_eq!(profile.validate(), Ok(()));
        assert_eq!(
            RuntimeMissingToothProfile::from_import_profile(profile),
            Err(TriggerValidationError::UnsupportedSecondaryTriggerMode)
        );
        assert_eq!(
            ProfiledMissingToothDecoder::try_new_from_import_profile(
                profile,
                Ticks::new(50),
                DEFAULT_MISSING_TOOTH_GAP_RATIO_X1000,
            ),
            Err(TriggerValidationError::UnsupportedSecondaryTriggerMode)
        );
    }
}

#[test]
fn false_edge_below_filter_threshold_is_ignored() {
    let mut decoder = new_decoder(decoder_config(Degrees10::new(100)));
    assert_eq!(
        decoder.ingest_primary_edge(Ticks::new(1000)),
        Ok(MissingToothDecoderEvent::FirstEdge)
    );
    assert_eq!(
        decoder.ingest_primary_edge(Ticks::new(2000)),
        Ok(MissingToothDecoderEvent::Searching)
    );
    assert_eq!(
        decoder.ingest_primary_edge(Ticks::new(2010)),
        Ok(MissingToothDecoderEvent::IgnoredByFilter)
    );

    let observation = decoder.observation();
    assert_eq!(observation.last_primary_interval, Ticks::new(1000));
    assert_eq!(observation.primary_rpm, Rpm::new(1000));

    assert_eq!(
        decoder.ingest_primary_edge(Ticks::new(3000)),
        Ok(MissingToothDecoderEvent::Searching)
    );
    assert_eq!(
        decoder.observation().last_primary_interval,
        Ticks::new(1000)
    );
}

#[test]
fn sixty_minus_two_detects_tooth_one_after_gap_and_reports_rpm() {
    let mut decoder = new_decoder(decoder_config(Degrees10::new(840)));
    lock_60_minus_2(&mut decoder);

    let observation = decoder.observation();
    assert!(decoder.is_primary_locked());
    assert_eq!(observation.current_tooth, 1);
    assert_eq!(observation.detected_gap_ratio_x1000, 3000);
    assert_eq!(observation.primary_rpm, Rpm::new(1000));
    assert_eq!(observation.crank_angle_deg10.map(Degrees10::get), Some(840));
    assert_eq!(decoder.diagnostics().last_sync_loss, None);
}

#[test]
fn missing_tooth_decoder_uses_wrapping_elapsed_time_across_timer_rollover() {
    let mut decoder = new_decoder(decoder_config(Degrees10::new(840)));
    let mut timestamp = u32::MAX - 1_500;

    assert_eq!(
        decoder.ingest_primary_edge(Ticks::new(timestamp)),
        Ok(MissingToothDecoderEvent::FirstEdge)
    );

    timestamp = timestamp.wrapping_add(1_000);
    assert_eq!(
        decoder.ingest_primary_edge(Ticks::new(timestamp)),
        Ok(MissingToothDecoderEvent::Searching)
    );

    timestamp = timestamp.wrapping_add(3_000);
    assert_eq!(
        decoder.ingest_primary_edge(Ticks::new(timestamp)),
        Ok(MissingToothDecoderEvent::Gap { current_tooth: 1 })
    );

    let observation = decoder.observation();
    assert!(decoder.is_primary_locked());
    assert_eq!(observation.current_tooth, 1);
    assert_eq!(observation.last_primary_interval, Ticks::new(3_000));
    assert_eq!(observation.detected_gap_ratio_x1000, 3_000);
    assert_eq!(decoder.diagnostics().last_sync_loss, None);
}

#[test]
fn wrong_primary_edge_is_rejected_without_advancing_decoder() {
    let mut decoder = new_decoder(decoder_config(Degrees10::new(840)));

    assert_eq!(
        decoder.ingest_primary_edge_with_kind(Ticks::new(0), TriggerEdge::Falling),
        Err(SyncLossReason::UnexpectedPrimaryEdge)
    );
    assert!(!decoder.is_primary_locked());
    assert_eq!(decoder.observation().last_primary_interval, Ticks::new(0));
    assert_eq!(
        decoder.diagnostics().last_sync_loss,
        Some(SyncLossReason::UnexpectedPrimaryEdge)
    );

    lock_60_minus_2(&mut decoder);
    assert!(decoder.is_primary_locked());
}

#[test]
fn single_tooth_cam_requires_matching_edge_after_primary_lock() {
    let mut decoder = new_decoder(single_tooth_cam_config(Degrees10::new(840)));

    assert_eq!(decoder.ingest_secondary_edge(TriggerEdge::Falling), Ok(()));
    assert_eq!(
        decoder.diagnostics().authority.phase,
        PhaseSyncState::Unknown
    );

    lock_60_minus_2(&mut decoder);
    assert_eq!(
        decoder.ingest_secondary_edge(TriggerEdge::Rising),
        Err(SyncLossReason::PhaseMismatch)
    );
    assert_eq!(
        decoder.diagnostics().authority.phase,
        PhaseSyncState::CrankOnly360
    );

    assert_eq!(decoder.ingest_secondary_edge(TriggerEdge::Falling), Ok(()));
    let diagnostics = decoder.diagnostics();
    assert!(diagnostics.observation.cam_seen);
    assert_eq!(diagnostics.authority.phase, PhaseSyncState::CamValidated720);
    assert_eq!(
        diagnostics.authority.absolute,
        AbsoluteTimeAuthority::ExpertManual
    );
}

#[test]
fn single_tooth_cam_rejects_matching_edge_outside_tooth_one_window() {
    let mut decoder = new_decoder(single_tooth_cam_config(Degrees10::new(840)));
    lock_60_minus_2(&mut decoder);
    assert_eq!(
        decoder.ingest_primary_edge(Ticks::new(5_000)),
        Ok(MissingToothDecoderEvent::Tooth { current_tooth: 2 })
    );

    assert_eq!(
        decoder.ingest_secondary_edge(TriggerEdge::Falling),
        Err(SyncLossReason::PhaseMismatch)
    );
    let diagnostics = decoder.diagnostics();
    assert_eq!(
        diagnostics.last_sync_loss,
        Some(SyncLossReason::PhaseMismatch)
    );
    assert_eq!(diagnostics.authority.phase, PhaseSyncState::CrankOnly360);
    assert_eq!(diagnostics.authority.absolute, AbsoluteTimeAuthority::None);
}

#[test]
fn poll_level_validates_level_at_tooth_one_before_phase_promotion() {
    let mut decoder = new_decoder(poll_level_config(Degrees10::new(840)));
    lock_60_minus_2(&mut decoder);

    assert_eq!(
        decoder.ingest_secondary_level(TriggerLevel::Low),
        Err(SyncLossReason::PhaseMismatch)
    );
    assert_eq!(
        decoder.diagnostics().authority.phase,
        PhaseSyncState::CrankOnly360
    );

    assert_eq!(decoder.ingest_secondary_level(TriggerLevel::High), Ok(()));
    let diagnostics = decoder.diagnostics();
    assert!(diagnostics.observation.cam_seen);
    assert_eq!(diagnostics.authority.phase, PhaseSyncState::CamValidated720);
    assert_eq!(
        diagnostics.authority.absolute,
        AbsoluteTimeAuthority::ExpertManual
    );

    assert_eq!(
        decoder.ingest_primary_edge(Ticks::new(5000)),
        Ok(MissingToothDecoderEvent::Tooth { current_tooth: 2 })
    );
    assert_eq!(
        decoder.ingest_secondary_level(TriggerLevel::High),
        Err(SyncLossReason::PhaseMismatch)
    );
}

#[test]
fn profiled_decoder_holds_absolute_authority_until_startup_policy_is_satisfied() {
    let profile = profiled_config(
        Degrees10::new(840),
        RuntimeSecondaryTriggerProfile {
            mode: RuntimeSecondaryTriggerMode::SingleToothCam,
            edge: TriggerEdge::Falling,
            poll_level: PollLevelPolarity::ActiveHigh,
        },
        StartupSyncPolicy {
            skip_revolutions: 2,
            require_full_cycle: true,
        },
        EngineTimeLatency::default(),
    );
    let mut decoder = profiled_decoder(profile);

    assert_eq!(
        decoder.ingest_primary_edge(Ticks::new(0)),
        Ok(MissingToothDecoderEvent::FirstEdge)
    );
    assert_eq!(
        decoder.ingest_primary_edge(Ticks::new(1000)),
        Ok(MissingToothDecoderEvent::Searching)
    );
    assert_eq!(
        decoder.ingest_primary_edge(Ticks::new(4000)),
        Ok(MissingToothDecoderEvent::Gap { current_tooth: 1 })
    );
    assert_eq!(
        decoder.authority().compatibility_summary(),
        SyncState::Provisional
    );
    assert_eq!(decoder.authority().absolute, AbsoluteTimeAuthority::None);

    assert_eq!(decoder.ingest_secondary_edge(TriggerEdge::Falling), Ok(()));
    assert_eq!(
        decoder.authority().compatibility_summary(),
        SyncState::Provisional
    );
    assert_eq!(decoder.authority().absolute, AbsoluteTimeAuthority::None);

    let mut timestamp = 4000;
    timestamp = drive_missing_tooth_cycle(&mut decoder, timestamp, 1000);
    assert_eq!(
        decoder.authority().compatibility_summary(),
        SyncState::Provisional
    );
    assert_eq!(decoder.authority().absolute, AbsoluteTimeAuthority::None);

    assert_eq!(decoder.ingest_secondary_edge(TriggerEdge::Falling), Ok(()));
    let _ = drive_missing_tooth_cycle(&mut decoder, timestamp, 1000);
    let authority = decoder.authority();
    assert_eq!(authority.phase, PhaseSyncState::CamValidated720);
    assert_eq!(authority.absolute, AbsoluteTimeAuthority::ExpertManual);
    assert_eq!(
        authority.compatibility_summary(),
        SyncState::Locked { cam_ref: false }
    );
}

#[test]
fn profiled_diagnostics_are_startup_gated_and_raw_diagnostics_stay_explicit() {
    let profile = profiled_config(
        Degrees10::new(840),
        RuntimeSecondaryTriggerProfile {
            mode: RuntimeSecondaryTriggerMode::SingleToothCam,
            edge: TriggerEdge::Falling,
            poll_level: PollLevelPolarity::ActiveHigh,
        },
        StartupSyncPolicy {
            skip_revolutions: 2,
            require_full_cycle: true,
        },
        EngineTimeLatency::default(),
    );
    let mut decoder = profiled_decoder(profile);

    first_profile_lock(&mut decoder);
    assert_eq!(decoder.ingest_secondary_edge(TriggerEdge::Falling), Ok(()));

    let raw = decoder.raw_diagnostics();
    let gated = decoder.diagnostics();

    assert_eq!(raw.authority.phase, PhaseSyncState::CamValidated720);
    assert_eq!(raw.authority.absolute, AbsoluteTimeAuthority::ExpertManual);
    assert_eq!(gated.authority.phase, PhaseSyncState::CrankOnly360);
    assert_eq!(gated.authority.absolute, AbsoluteTimeAuthority::None);
    assert_eq!(gated.observation, raw.observation);
    assert_eq!(gated.last_sync_loss, raw.last_sync_loss);
}

#[test]
fn profiled_decoder_compensates_angle_queries_for_latency() {
    let zero_latency = profiled_config(
        Degrees10::new(0),
        RuntimeSecondaryTriggerProfile {
            mode: RuntimeSecondaryTriggerMode::None,
            edge: TriggerEdge::Rising,
            poll_level: PollLevelPolarity::ActiveHigh,
        },
        StartupSyncPolicy::default(),
        EngineTimeLatency::default(),
    );
    let delayed_latency = profiled_config(
        Degrees10::new(0),
        RuntimeSecondaryTriggerProfile {
            mode: RuntimeSecondaryTriggerMode::None,
            edge: TriggerEdge::Rising,
            poll_level: PollLevelPolarity::ActiveHigh,
        },
        StartupSyncPolicy::default(),
        EngineTimeLatency {
            primary_edge_delay_us: Micros::new(100),
            secondary_edge_delay_us: Micros::new(50),
            output_schedule_delay_us: Micros::new(100),
        },
    );

    let mut zero = profiled_decoder(zero_latency);
    let mut delayed = profiled_decoder(delayed_latency);

    for decoder in [&mut zero, &mut delayed] {
        assert_eq!(
            decoder.ingest_primary_edge(Ticks::new(0)),
            Ok(MissingToothDecoderEvent::FirstEdge)
        );
        assert_eq!(
            decoder.ingest_primary_edge(Ticks::new(1000)),
            Ok(MissingToothDecoderEvent::Searching)
        );
        assert_eq!(
            decoder.ingest_primary_edge(Ticks::new(4000)),
            Ok(MissingToothDecoderEvent::Gap { current_tooth: 1 })
        );
    }

    assert_eq!(
        zero.crank_angle_at(Ticks::new(4500)).map(Degrees10::get),
        Some(31)
    );
    assert_eq!(
        delayed.crank_angle_at(Ticks::new(4500)).map(Degrees10::get),
        Some(15)
    );
}

#[test]
fn crank_angle_wraps_to_engine_cycle_range() {
    let mut decoder = new_decoder(decoder_config(Degrees10::new(7190)));
    lock_60_minus_2(&mut decoder);
    assert_eq!(
        decoder.ingest_primary_edge(Ticks::new(5000)),
        Ok(MissingToothDecoderEvent::Tooth { current_tooth: 2 })
    );

    let angle_at_edge = decoder.observation().crank_angle_deg10.map(Degrees10::get);
    let angle_between_edges = decoder.crank_angle_at(Ticks::new(5500)).map(Degrees10::get);
    assert_eq!(angle_at_edge, Some(50));
    assert_eq!(angle_between_edges, Some(80));
    for timestamp in [Ticks::new(5000), Ticks::new(25_000), Ticks::new(125_000)] {
        let angle = decoder.crank_angle_at(timestamp).map(Degrees10::get);
        assert!(matches!(angle, Some(0..=7199)));
    }
}

#[test]
fn wrong_tooth_count_drops_sync_and_records_reason() {
    let mut decoder = new_decoder(decoder_config(Degrees10::new(0)));
    lock_60_minus_2(&mut decoder);

    let mut timestamp = 4000;
    for expected_tooth in 2..=58 {
        timestamp += 1000;
        assert_eq!(
            decoder.ingest_primary_edge(Ticks::new(timestamp)),
            Ok(MissingToothDecoderEvent::Tooth {
                current_tooth: expected_tooth,
            })
        );
    }

    timestamp += 1000;
    assert_eq!(
        decoder.ingest_primary_edge(Ticks::new(timestamp)),
        Err(SyncLossReason::WrongToothCount)
    );
    let diagnostics = decoder.diagnostics();
    assert!(!decoder.is_primary_locked());
    assert_eq!(
        diagnostics.last_sync_loss,
        Some(SyncLossReason::WrongToothCount)
    );
    assert_eq!(diagnostics.authority.crank, CrankSyncState::SyncLost);
    assert_eq!(diagnostics.authority.sync_loss_count, 1);
}

#[test]
fn cranking_jitter_does_not_falsely_certify_full_authority() {
    let mut decoder = new_decoder(decoder_config(Degrees10::new(120)));
    let timestamps = [
        Ticks::new(0),
        Ticks::new(1000),
        Ticks::new(2400),
        Ticks::new(3300),
        Ticks::new(4600),
        Ticks::new(5600),
    ];

    for timestamp in timestamps {
        let _ = decoder.ingest_primary_edge(timestamp);
    }

    let diagnostics = decoder.diagnostics();
    assert!(!decoder.is_primary_locked());
    assert_ne!(
        diagnostics.authority.compatibility_summary(),
        SyncState::Locked { cam_ref: false }
    );
    assert_eq!(diagnostics.authority.absolute, AbsoluteTimeAuthority::None);
    assert!(
        diagnostics.observation.detected_gap_ratio_x1000 < DEFAULT_MISSING_TOOTH_GAP_RATIO_X1000
    );
}

#[test]
fn fixed_capacity_batch_ingests_edges_without_allocation() {
    let mut batch: PrimaryEdgeBatch<4> = PrimaryEdgeBatch::new();
    assert_eq!(batch.capacity(), 4);
    assert_eq!(batch.push(PrimaryEdgeSample::new(Ticks::new(0))), Ok(()));
    assert_eq!(batch.push(PrimaryEdgeSample::new(Ticks::new(1000))), Ok(()));
    assert_eq!(batch.push(PrimaryEdgeSample::new(Ticks::new(4000))), Ok(()));
    assert_eq!(batch.push(PrimaryEdgeSample::new(Ticks::new(4010))), Ok(()));
    assert_eq!(
        batch.push(PrimaryEdgeSample::new(Ticks::new(5000))),
        Err(PrimaryEdgeBatchError::Full)
    );

    let mut decoder = new_decoder(decoder_config(Degrees10::new(0)));
    let result = decoder.ingest_primary_edges(&batch);
    assert_eq!(result.processed_edges, 3);
    assert_eq!(result.ignored_edges, 1);
    assert_eq!(
        result.last_event,
        Some(MissingToothDecoderEvent::IgnoredByFilter)
    );
    assert!(decoder.is_primary_locked());
    batch.clear();
    assert!(batch.is_empty());
}

#[test]
fn deterministic_small_generated_streams_do_not_panic_or_overrun() {
    for nominal_teeth in 3..=12 {
        for missing_teeth in 1..nominal_teeth {
            if nominal_teeth - missing_teeth < 2 {
                continue;
            }

            let config = MissingToothDecoderConfig {
                nominal_teeth,
                missing_teeth,
                primary_speed: TriggerSpeed::Crank,
                primary_edge: TriggerEdge::Rising,
                secondary: RuntimeSecondaryTriggerProfile {
                    mode: RuntimeSecondaryTriggerMode::None,
                    edge: TriggerEdge::Rising,
                    poll_level: PollLevelPolarity::ActiveHigh,
                },
                trigger_angle_atdc_deg10: TriggerAngleAuthority::ExpertManual(Degrees10::new(0)),
                tooth_angle_multiplier: 1,
                minimum_edge_interval: Ticks::new(1),
                gap_ratio_threshold_x1000: DEFAULT_MISSING_TOOTH_GAP_RATIO_X1000,
            };

            let Ok(mut decoder) = MissingToothDecoder::try_new(config) else {
                continue;
            };
            let observed_teeth = nominal_teeth - missing_teeth;
            let normal_interval = 100;
            let gap_interval = normal_interval * u32::from(missing_teeth + 1);
            let mut timestamp = 0;
            let mut batch: PrimaryEdgeBatch<32> = PrimaryEdgeBatch::new();

            assert_eq!(
                batch.push(PrimaryEdgeSample::new(Ticks::new(timestamp))),
                Ok(())
            );
            timestamp += normal_interval;
            assert_eq!(
                batch.push(PrimaryEdgeSample::new(Ticks::new(timestamp))),
                Ok(())
            );
            timestamp += gap_interval;
            assert_eq!(
                batch.push(PrimaryEdgeSample::new(Ticks::new(timestamp))),
                Ok(())
            );
            for _ in 1..observed_teeth {
                timestamp += normal_interval;
                assert_eq!(
                    batch.push(PrimaryEdgeSample::new(Ticks::new(timestamp))),
                    Ok(())
                );
            }
            timestamp += gap_interval;
            assert_eq!(
                batch.push(PrimaryEdgeSample::new(Ticks::new(timestamp))),
                Ok(())
            );

            let result = decoder.ingest_primary_edges(&batch);
            assert_eq!(usize::from(result.processed_edges), batch.len());
            assert_eq!(result.ignored_edges, 0);
            assert_eq!(result.last_sync_loss, None);
        }
    }
}

proptest! {
    #[test]
    fn prop_missing_tooth_stream_keeps_gap_and_angle_order(
        nominal_teeth in 3u8..16,
        missing_teeth in 1u8..8,
        normal_interval in 50u32..1000u32,
    ) {
        prop_assume!(missing_teeth < nominal_teeth);
        prop_assume!(nominal_teeth - missing_teeth >= 2);

        let config = MissingToothDecoderConfig {
            nominal_teeth,
            missing_teeth,
            primary_speed: TriggerSpeed::Crank,
            primary_edge: TriggerEdge::Rising,
            secondary: RuntimeSecondaryTriggerProfile {
                mode: RuntimeSecondaryTriggerMode::None,
                edge: TriggerEdge::Rising,
                poll_level: PollLevelPolarity::ActiveHigh,
            },
            trigger_angle_atdc_deg10: TriggerAngleAuthority::ExpertManual(Degrees10::new(0)),
            tooth_angle_multiplier: 1,
            minimum_edge_interval: Ticks::new(1),
            gap_ratio_threshold_x1000: DEFAULT_MISSING_TOOTH_GAP_RATIO_X1000,
        };

        let mut decoder = MissingToothDecoder::try_new(config)
            .expect("valid missing-tooth config was rejected");

        let observed_teeth = nominal_teeth - missing_teeth;
        let mut timestamp = 0u32;

        prop_assert_eq!(
            decoder.ingest_primary_edge(Ticks::new(timestamp)),
            Ok(MissingToothDecoderEvent::FirstEdge)
        );
        timestamp = timestamp.saturating_add(normal_interval);
        prop_assert_eq!(
            decoder.ingest_primary_edge(Ticks::new(timestamp)),
            Ok(MissingToothDecoderEvent::Searching)
        );
        timestamp = timestamp.saturating_add(normal_interval * u32::from(missing_teeth + 1));
        prop_assert_eq!(
            decoder.ingest_primary_edge(Ticks::new(timestamp)),
            Ok(MissingToothDecoderEvent::Gap { current_tooth: 1 })
        );

        let mut last_angle = decoder
            .observation()
            .crank_angle_deg10
            .map(Degrees10::get)
            .unwrap_or_default();

        prop_assert_eq!(last_angle, 0);

        for expected_tooth in 2..=observed_teeth {
            timestamp = timestamp.saturating_add(normal_interval);
            prop_assert_eq!(
                decoder.ingest_primary_edge(Ticks::new(timestamp)),
                Ok(MissingToothDecoderEvent::Tooth {
                    current_tooth: expected_tooth,
                })
            );
            let angle = decoder
                .observation()
                .crank_angle_deg10
                .map(Degrees10::get)
                .unwrap_or_default();
            prop_assert!(angle >= last_angle);
            last_angle = angle;
        }
    }
}
