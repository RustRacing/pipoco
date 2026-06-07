use crate::test_support::observe_scheduler;
use crate::*;
use ecu_domain::Rpm;
use ecu_spec::{
    default_reference_calibration, schedule_all_cylinders, AfrOverride, EngineMode, InputSnapshot,
    Kpa10, Millivolts, SyncState,
};

fn canonical_input() -> InputSnapshot {
    InputSnapshot {
        t_us: ecu_spec::Micros(0),
        rpm: ecu_spec::Rpm(1000),
        map_kpa10: Kpa10(1000),
        load_kpa10: Kpa10(1000),
        tps_x100: 0,
        clt_c10: ecu_spec::TempC10(800),
        iat_c10: ecu_spec::TempC10(250),
        baro_kpa10: Kpa10(1000),
        vbatt_mv: Millivolts(12_000),
        knock_intensity_x100: 0,
        launch_armed: false,
        flat_shift_armed: false,
        sync: SyncState::Synced,
        fuel_cut: false,
        spark_cut: false,
        mode: EngineMode::Running,
        target_afr_override_x100: AfrOverride::None,
    }
}

fn assert_angle_within(
    field: &str,
    runtime: u16,
    oracle: u16,
    tolerance: u16,
    input: &InputSnapshot,
    fixture: &str,
) {
    let difference = runtime.abs_diff(oracle);
    assert!(
            difference <= tolerance,
            "input_snapshot={input:?}\ncalibration_fixture={fixture}\nruntime_output={runtime}\noracle_output={oracle}\nfield={field}\ndifference={difference}\ntolerance={tolerance}"
        );
}

#[test]
fn exclusive_channel_carries_group_and_identifier() {
    let channel = ExclusiveChannel::new(OutputGroup::Injector, ChannelId::new(2));

    assert_eq!(channel.group(), OutputGroup::Injector);
    assert_eq!(channel.channel().get(), 2);
}

#[test]
fn output_plans_round_trip() {
    let inj = InjectionPlan {
        output: ExclusiveChannel::new(OutputGroup::Injector, ChannelId::new(1)),
        pulse_width: PulseWidthUs::new(2500),
    };
    let ign = IgnitionPlan {
        output: ExclusiveChannel::new(OutputGroup::Ignition, ChannelId::new(0)),
        dwell: DwellUs::new(1800),
        advance: Degrees10::new(125),
    };

    assert_eq!(inj.pulse_width.get(), 2500);
    assert_eq!(ign.dwell.get(), 1800);
    assert_eq!(ign.advance.get(), 125);
}

#[test]
fn ignition_scheduler_uses_crank_angle_and_topology() {
    let crank = CrankSnapshot::new(
        Micros::new(1_000),
        Rpm::new(3_000),
        Degrees10::new(0),
        EngineTimeAuthority::none(),
    );
    let scheduler = IgnitionScheduler::new(SparkOutputProfile::crank_only_wasted_spark(6));
    let timed = scheduler.plan_event(
        crank,
        1,
        SparkPlan::new(DwellUs::new(1_500), Degrees10::new(100)),
    );

    assert_eq!(scheduler.event_count(), 3);
    assert_eq!(timed.plan.output.group(), OutputGroup::Ignition);
    assert_eq!(timed.plan.output.channel(), ChannelId::new(1));
    assert_eq!(timed.plan.dwell, DwellUs::new(1_500));
    assert_eq!(timed.plan.advance, Degrees10::new(100));
    assert!(timed.end_at.get() > timed.start_at.get());
    assert!(timed.start_at.get() >= crank.now_us.get());
}

#[test]
fn ignition_scheduler_single_coil_collapses_all_events_to_channel_zero() {
    let crank = CrankSnapshot::new(
        Micros::new(1_000),
        Rpm::new(3_000),
        Degrees10::new(0),
        EngineTimeAuthority::none(),
    );
    let scheduler = IgnitionScheduler::new(SparkOutputProfile::crank_only_single_coil(6));

    assert_eq!(scheduler.event_count(), 3);
    for event_index in 0..scheduler.event_count() {
        let timed = scheduler.plan_event(
            crank,
            event_index,
            SparkPlan::new(DwellUs::new(1_000), Degrees10::new(80)),
        );
        assert_eq!(timed.plan.output.group(), OutputGroup::Ignition);
        assert_eq!(timed.plan.output.channel(), ChannelId::new(0));
    }
}

#[test]
fn injection_scheduler_uses_fuel_topology_without_ignition() {
    let crank = CrankSnapshot::new(
        Micros::new(2_000),
        Rpm::new(2_000),
        Degrees10::new(900),
        EngineTimeAuthority::none(),
    );
    let scheduler = InjectionScheduler::new(FuelOutputProfile::batch(3));
    let timed = scheduler.plan_event(crank, 2, FuelPlan::new(PulseWidthUs::new(2_500)));

    assert_eq!(scheduler.event_count(), 3);
    assert_eq!(timed.plan.output.group(), OutputGroup::Injector);
    assert_eq!(timed.plan.output.channel(), ChannelId::new(2));
    assert_eq!(timed.plan.pulse_width, PulseWidthUs::new(2_500));
    assert_eq!(timed.start_at, Micros::new(2_000));
    assert_eq!(timed.end_at, Micros::new(4_500));
}

#[test]
fn output_profiles_normalize_direct_event_indices() {
    let spark = SparkOutputProfile::crank_only_wasted_spark(6);
    let fuel = FuelOutputProfile::batch(3);

    assert_eq!(spark.events_per_crank_rev(), 3);
    assert_eq!(spark.ignition_channel(4), ChannelId::new(1));
    assert_eq!(spark.event_tdc_angle_deg10(4), 1200);
    assert_eq!(fuel.events_per_pulse(), 3);
    assert_eq!(fuel.injector_channel(4), ChannelId::new(1));
}

#[test]
fn ignition_scheduler_handles_wraparound_deadlines() {
    let crank = CrankSnapshot::new(
        Micros::new(1_000),
        Rpm::new(6_000),
        Degrees10::new(3400),
        EngineTimeAuthority::none(),
    );
    let scheduler = IgnitionScheduler::new(SparkOutputProfile::crank_only_wasted_spark(4));
    let timed = scheduler.plan_event(
        crank,
        0,
        SparkPlan::new(DwellUs::new(100), Degrees10::new(100)),
    );

    assert_eq!(timed.plan.output.channel(), ChannelId::new(0));
    assert_eq!(timed.start_at, Micros::new(1_177));
    assert_eq!(timed.end_at, Micros::new(1_277));
}

#[test]
fn injection_scheduler_zero_pulse_width_still_exports_valid_deadline() {
    let crank = CrankSnapshot::new(
        Micros::new(2_000),
        Rpm::new(2_000),
        Degrees10::new(900),
        EngineTimeAuthority::none(),
    );
    let scheduler = InjectionScheduler::new(FuelOutputProfile::single_point());
    let timed = scheduler.plan_event(crank, 99, FuelPlan::new(PulseWidthUs::new(0)));

    assert_eq!(timed.plan.output.group(), OutputGroup::Injector);
    assert_eq!(timed.plan.output.channel(), ChannelId::new(0));
    assert_eq!(timed.start_at, Micros::new(2_000));
    assert_eq!(timed.end_at, Micros::new(2_001));
}

#[test]
fn fuel_and_spark_profiles_clamp_zero_counts_to_one_event() {
    let spark = SparkOutputProfile::crank_only_wasted_spark(0);
    let fuel = FuelOutputProfile::batch(0);

    assert_eq!(IgnitionScheduler::new(spark).event_count(), 1);
    assert_eq!(spark.ignition_channel(0), ChannelId::new(0));
    assert_eq!(InjectionScheduler::new(fuel).event_count(), 1);
    assert_eq!(fuel.injector_channel(0), ChannelId::new(0));
}

#[test]
fn fuel_and_spark_profiles_clamp_large_counts_to_supported_capacity() {
    let spark = SparkOutputProfile::crank_only_wasted_spark(40);
    let fuel = FuelOutputProfile::batch(40);

    assert_eq!(IgnitionScheduler::new(spark).event_count(), 8);
    assert_eq!(spark.ignition_channel(9), ChannelId::new(1));
    assert_eq!(InjectionScheduler::new(fuel).event_count(), 8);
    assert_eq!(fuel.injector_channel(9), ChannelId::new(1));
}

#[test]
fn plans_default_to_disabled_or_empty_state() {
    let inj = InjectionPlan::default();
    let ign = IgnitionPlan::default();

    assert_eq!(inj.output.group(), OutputGroup::Injector);
    assert_eq!(ign.output.group(), OutputGroup::Injector);
}

#[test]
fn scheduler_state_transitions_between_modes() {
    let mut state = SchedulerState::new();

    assert_eq!(state.mode(), SchedulerMode::Idle);
    assert_eq!(state.active_groups(), 0);

    state.arm_group(OutputGroup::Injector);
    assert_eq!(state.mode(), SchedulerMode::Armed);
    assert!(state.is_armed());

    state.cancel_group(OutputGroup::Injector);
    assert_eq!(state.mode(), SchedulerMode::Idle);
    assert!(!state.is_armed());
}

#[test]
fn cancel_all_and_safety_paths_clear_everything() {
    let mut state = SchedulerState::new();
    state.arm_group(OutputGroup::Injector);
    state.arm_group(OutputGroup::Ignition);
    state.arm_group(OutputGroup::Idle);

    state.on_geometry_commit();
    assert_eq!(state.mode(), SchedulerMode::Armed);
    assert_eq!(state.active_groups(), OutputGroup::Idle.mask());

    state.on_sync_loss();
    assert_eq!(state.mode(), SchedulerMode::Suspended);
    assert_eq!(state.active_groups(), 0);

    state.arm_group(OutputGroup::Fan);
    state.on_hard_safety_shutdown();
    assert_eq!(state.mode(), SchedulerMode::Suspended);
    assert_eq!(state.active_groups(), 0);
}

#[test]
fn schedule_converts_deadlines_and_arms_groups() {
    let mut state = SchedulerState::new();
    let plan = InjectionPlan {
        output: ExclusiveChannel::new(OutputGroup::Injector, ChannelId::new(1)),
        pulse_width: PulseWidthUs::new(1200),
    };

    let timed = state
        .schedule_injection(Micros::new(100), Micros::new(150), Micros::new(350), plan)
        .expect("valid injection schedule");

    assert_eq!(timed.start_at.get(), 150);
    assert_eq!(timed.end_at.get(), 350);
    assert_eq!(state.mode(), SchedulerMode::Armed);
    assert!(state.is_armed());
}

#[test]
fn schedule_rejects_stale_and_impossible_windows() {
    let mut state = SchedulerState::new();
    let inj = InjectionPlan {
        output: ExclusiveChannel::new(OutputGroup::Injector, ChannelId::new(1)),
        pulse_width: PulseWidthUs::new(1200),
    };
    let ign = IgnitionPlan {
        output: ExclusiveChannel::new(OutputGroup::Ignition, ChannelId::new(0)),
        dwell: DwellUs::new(1200),
        advance: Degrees10::new(120),
    };

    assert_eq!(
        state.schedule_injection(Micros::new(100), Micros::new(100), Micros::new(350), inj),
        Err(ScheduleError::StaleDeadline)
    );
    assert_eq!(
        state.schedule_ignition(Micros::new(100), Micros::new(150), Micros::new(150), ign),
        Err(ScheduleError::ImpossibleDeadline)
    );
}

#[test]
fn suspended_state_rejects_scheduling() {
    let mut state = SchedulerState::new();
    state.on_sync_loss();

    let plan = InjectionPlan {
        output: ExclusiveChannel::new(OutputGroup::Injector, ChannelId::new(1)),
        pulse_width: PulseWidthUs::new(900),
    };

    assert_eq!(
        state.schedule_injection(Micros::new(1), Micros::new(2), Micros::new(4), plan),
        Err(ScheduleError::Suspended)
    );
}

#[test]
fn schedule_rejects_conflicting_channels() {
    let mut state = SchedulerState::new();
    let first = InjectionPlan {
        output: ExclusiveChannel::new(OutputGroup::Injector, ChannelId::new(1)),
        pulse_width: PulseWidthUs::new(1200),
    };
    let second = InjectionPlan {
        output: ExclusiveChannel::new(OutputGroup::Injector, ChannelId::new(1)),
        pulse_width: PulseWidthUs::new(900),
    };

    let _ = state
        .schedule_injection(Micros::new(100), Micros::new(150), Micros::new(350), first)
        .expect("first schedule should succeed");

    assert_eq!(
        state.schedule_injection(Micros::new(200), Micros::new(250), Micros::new(450), second),
        Err(ScheduleError::ConflictingChannel)
    );
}

#[test]
fn schedule_rejects_channels_outside_bitset_width() {
    let mut state = SchedulerState::new();
    let plan = InjectionPlan {
        output: ExclusiveChannel::new(OutputGroup::Injector, ChannelId::new(128)),
        pulse_width: PulseWidthUs::new(1200),
    };

    assert_eq!(
        state.schedule_injection(Micros::new(100), Micros::new(150), Micros::new(350), plan),
        Err(ScheduleError::InvalidChannel)
    );

    let observed = observe_scheduler(&state);
    assert_eq!(observed.active_groups, 0);
    assert_eq!(observed.injection_count, 0);
}

#[test]
fn scheduler_observation_reports_active_counts_and_last_deadlines() {
    let mut state = SchedulerState::new();
    let inj = InjectionPlan {
        output: ExclusiveChannel::new(OutputGroup::Injector, ChannelId::new(1)),
        pulse_width: PulseWidthUs::new(1200),
    };
    let ign = IgnitionPlan {
        output: ExclusiveChannel::new(OutputGroup::Ignition, ChannelId::new(2)),
        dwell: DwellUs::new(2000),
        advance: Degrees10::new(150),
    };

    state
        .schedule_injection(Micros::new(100), Micros::new(150), Micros::new(350), inj)
        .expect("valid injection schedule");
    state
        .schedule_ignition(Micros::new(100), Micros::new(180), Micros::new(420), ign)
        .expect("valid ignition schedule");

    let observed = observe_scheduler(&state);
    assert_eq!(observed.injection_count, 1);
    assert_eq!(observed.ignition_count, 1);
    assert_eq!(observed.last_injection_start, Some(Micros::new(150)));
    assert_eq!(observed.last_injection_end, Some(Micros::new(350)));
    assert_eq!(observed.last_ignition_start, Some(Micros::new(180)));
    assert_eq!(observed.last_ignition_end, Some(Micros::new(420)));

    state.cancel_group(OutputGroup::Injector);
    let observed = observe_scheduler(&state);
    assert_eq!(observed.injection_count, 0);
    assert_eq!(observed.ignition_count, 1);
}

#[test]
fn scheduler_differential_mapping_matches_oracle_angles() {
    let input = canonical_input();
    let oracle = schedule_all_cylinders(
        &default_reference_calibration(),
        input,
        ecu_spec::FuelOutput {
            pw_corr_us: ecu_spec::PulseWidthUs(3200),
        },
    );

    let mut state = SchedulerState::new();
    let inj = InjectionPlan {
        output: ExclusiveChannel::new(OutputGroup::Injector, ChannelId::new(1)),
        pulse_width: PulseWidthUs::new(3200),
    };
    let ign = IgnitionPlan {
        output: ExclusiveChannel::new(OutputGroup::Ignition, ChannelId::new(1)),
        dwell: DwellUs::new(2500),
        advance: Degrees10::new(150),
    };

    let timed_inj = state
        .schedule_injection(Micros::new(0), Micros::new(6648), Micros::new(6840), inj)
        .expect("valid injection schedule");
    let timed_ign = state
        .schedule_ignition(Micros::new(0), Micros::new(6900), Micros::new(7050), ign)
        .expect("valid ignition schedule");

    assert_angle_within(
        "InjectionOpen.angle_deg10",
        timed_inj.start_at.get() as u16,
        oracle.soi_deg10.values[0],
        1,
        &input,
        "canonical_reference_calibration",
    );
    assert_angle_within(
        "InjectionClose.angle_deg10",
        timed_inj.end_at.get() as u16,
        oracle.eoi_deg10.values[0],
        1,
        &input,
        "canonical_reference_calibration",
    );
    assert_angle_within(
        "CoilChargeStart.angle_deg10",
        timed_ign.start_at.get() as u16,
        oracle.dwell_start_deg10.values[0],
        1,
        &input,
        "canonical_reference_calibration",
    );
    assert_angle_within(
        "CoilFire.angle_deg10",
        timed_ign.end_at.get() as u16,
        oracle.spark_deg10.values[0],
        1,
        &input,
        "canonical_reference_calibration",
    );
}

#[test]
fn injection_export_produces_high_at_start_low_at_end() {
    let inj_plan = InjectionPlan {
        output: ExclusiveChannel::new(OutputGroup::Injector, ChannelId::new(1)),
        pulse_width: PulseWidthUs::new(1500),
    };
    let timed = TimedInjectionPlan {
        plan: inj_plan,
        start_at: Micros::new(1000),
        end_at: Micros::new(2500),
    };

    let export = timed.export_transitions::<4>().expect("valid plan");
    assert_eq!(export.len, 2);

    // First transition should be high at start
    let t0 = export.transitions[0].expect("transition 0");
    assert_eq!(t0.kind, ScheduledTransitionKind::Injector);
    assert_eq!(t0.channel.get(), 1);
    assert_eq!(t0.at_us.get(), 1000);
    assert!(matches!(t0.level, ScheduledLevel::High));

    // Second transition should be low at end
    let t1 = export.transitions[1].expect("transition 1");
    assert_eq!(t1.kind, ScheduledTransitionKind::Injector);
    assert_eq!(t1.channel.get(), 1);
    assert_eq!(t1.at_us.get(), 2500);
    assert!(matches!(t1.level, ScheduledLevel::Low));
}

#[test]
fn ignition_export_produces_high_at_dwell_start_low_at_fire() {
    let ign_plan = IgnitionPlan {
        output: ExclusiveChannel::new(OutputGroup::Ignition, ChannelId::new(0)),
        dwell: DwellUs::new(3000),
        advance: Degrees10::new(150),
    };
    let timed = TimedIgnitionPlan {
        plan: ign_plan,
        start_at: Micros::new(500),
        end_at: Micros::new(3500),
    };

    let export = timed.export_transitions::<4>().expect("valid plan");
    assert_eq!(export.len, 2);

    // First transition should be high at dwell start
    let t0 = export.transitions[0].expect("transition 0");
    assert_eq!(t0.kind, ScheduledTransitionKind::Ignition);
    assert_eq!(t0.channel.get(), 0);
    assert_eq!(t0.at_us.get(), 500);
    assert!(matches!(t0.level, ScheduledLevel::High));

    // Second transition should be low at fire time
    let t1 = export.transitions[1].expect("transition 1");
    assert_eq!(t1.kind, ScheduledTransitionKind::Ignition);
    assert_eq!(t1.channel.get(), 0);
    assert_eq!(t1.at_us.get(), 3500);
    assert!(matches!(t1.level, ScheduledLevel::Low));
}

#[test]
fn export_preserves_channel_ids() {
    let inj_plan = InjectionPlan {
        output: ExclusiveChannel::new(OutputGroup::Injector, ChannelId::new(3)),
        pulse_width: PulseWidthUs::new(2000),
    };
    let timed = TimedInjectionPlan {
        plan: inj_plan,
        start_at: Micros::new(100),
        end_at: Micros::new(2100),
    };

    let export = timed.export_transitions::<4>().expect("valid plan");
    assert_eq!(export.transitions[0].expect("t0").channel.get(), 3);
    assert_eq!(export.transitions[1].expect("t1").channel.get(), 3);
}

#[test]
fn export_equal_timestamps_sorts_deterministically() {
    // When start == end, should error
    let inj_plan = InjectionPlan {
        output: ExclusiveChannel::new(OutputGroup::Injector, ChannelId::new(1)),
        pulse_width: PulseWidthUs::new(1000),
    };
    let timed = TimedInjectionPlan {
        plan: inj_plan,
        start_at: Micros::new(500),
        end_at: Micros::new(500),
    };
    assert!(matches!(
        timed.export_transitions::<4>(),
        Err(ScheduleError::ImpossibleDeadline)
    ));
}

#[test]
fn export_rejects_insufficient_capacity() {
    let inj_plan = InjectionPlan {
        output: ExclusiveChannel::new(OutputGroup::Injector, ChannelId::new(1)),
        pulse_width: PulseWidthUs::new(1500),
    };
    let timed = TimedInjectionPlan {
        plan: inj_plan,
        start_at: Micros::new(100),
        end_at: Micros::new(1600),
    };
    assert!(matches!(
        timed.export_transitions::<0>(),
        Err(ScheduleError::ImpossibleDeadline)
    ));
    assert!(matches!(
        timed.export_transitions::<1>(),
        Err(ScheduleError::ImpossibleDeadline)
    ));
}

#[test]
fn model_capacity_constants_match_implementation() {
    assert_eq!(MODEL_MAX_PENDING, 4);
    assert_eq!(MODEL_MAX_OUTPUTS, 4);

    let inj_plan = InjectionPlan {
        output: ExclusiveChannel::new(OutputGroup::Injector, ChannelId::new(1)),
        pulse_width: PulseWidthUs::new(1500),
    };
    let timed = TimedInjectionPlan {
        plan: inj_plan,
        start_at: Micros::new(100),
        end_at: Micros::new(1600),
    };

    let export = timed
        .export_transitions::<MODEL_MAX_PENDING>()
        .expect("model-pending capacity must hold a per-cylinder event window");
    assert!((export.len as usize) <= MODEL_MAX_OUTPUTS);

    let queue = ScheduledTransitionQueue::<MODEL_MAX_PENDING>::new();
    assert_eq!(queue.capacity(), MODEL_MAX_PENDING);
    assert!(ScheduledTransitionQueue::<MODEL_MAX_PENDING>::metadata_capacity_supported());
}

#[test]
fn transition_queue_rejects_capacity_that_cannot_be_reported_in_u8_metadata() {
    let mut queue = ScheduledTransitionQueue::<256>::new();

    assert!(!ScheduledTransitionQueue::<256>::metadata_capacity_supported());
    assert_eq!(
        queue.enqueue_transition(ScheduledTransition {
            at_us: Micros::new(100),
            kind: ScheduledTransitionKind::Injector,
            channel: ChannelId::new(1),
            level: ScheduledLevel::High,
        }),
        Err(ScheduleError::QueueFull)
    );

    let mut drained = TransitionDrainBuffer::<256>::new();
    assert!(!TransitionDrainBuffer::<256>::metadata_capacity_supported());
    assert_eq!(queue.drain_due(Micros::new(200), &mut drained), 0);
    assert_eq!(drained.len, 0);
}

#[test]
fn transition_queue_saturates_at_max_pending_then_recovers_after_drain() {
    const MAX_PENDING: usize = 4;
    let mut queue = ScheduledTransitionQueue::<MAX_PENDING>::new();

    fn armed(channel: u8, at_us: u32) -> ScheduledTransition {
        ScheduledTransition {
            at_us: Micros::new(at_us),
            kind: ScheduledTransitionKind::Injector,
            channel: ChannelId::new(channel),
            level: ScheduledLevel::High,
        }
    }

    // Fill the queue exactly to MaxPending; every enqueue must be accepted.
    for slot in 0..MAX_PENDING {
        assert_eq!(
            queue.enqueue_transition(armed(slot as u8, 100 + slot as u32 * 10)),
            Ok(())
        );
    }
    assert!(queue.is_full());
    assert_eq!(queue.active_count(), MAX_PENDING);
    assert_eq!(queue.free_slots(), 0);

    let armed_before = queue.snapshot();

    // Saturation: the overflowing enqueue is rejected atomically and the
    // existing armed transitions are left untouched (no silent overwrite).
    assert_eq!(
        queue.enqueue_transition(armed(MAX_PENDING as u8, 999)),
        Err(ScheduleError::QueueFull)
    );
    assert_eq!(queue.active_count(), MAX_PENDING);
    assert_eq!(
        queue.snapshot(),
        armed_before,
        "armed outputs must not change"
    );

    // Recovery: draining due transitions frees the queue so it accepts again.
    let mut drained = TransitionDrainBuffer::<MAX_PENDING>::new();
    let count = queue.drain_due(Micros::new(1_000), &mut drained);
    assert_eq!(count, MAX_PENDING);
    assert_eq!(queue.active_count(), 0);
    assert!(!queue.is_full());

    assert_eq!(
        queue.enqueue_transition(armed(0, 2_000)),
        Ok(()),
        "queue must accept work again after draining"
    );
    assert_eq!(queue.active_count(), 1);
}
