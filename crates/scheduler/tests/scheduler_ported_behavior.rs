use ecu_domain::{ChannelId, Degrees10, DwellUs, Micros, PulseWidthUs};
use ecu_scheduler::test_support::observe_scheduler;
use ecu_scheduler::{
    ExclusiveChannel, IgnitionPlan, InjectionPlan, OutputGroup, ScheduleError, ScheduledLevel,
    ScheduledTransition, ScheduledTransitionKind, ScheduledTransitionQueue, SchedulerState,
    TimedIgnitionPlan, TimedInjectionPlan, TransitionDrainBuffer,
};

#[test]
fn exported_transitions_stay_in_order_without_jitter() {
    let mut observed = Vec::new();

    for i in 0..10u32 {
        let injection = TimedInjectionPlan {
            plan: InjectionPlan {
                output: ExclusiveChannel::new(OutputGroup::Injector, ChannelId::new(1)),
                pulse_width: PulseWidthUs::new(3200),
            },
            start_at: Micros::new(100 + i * 50),
            end_at: Micros::new(110 + i * 50),
        };
        let export = injection
            .export_transitions::<4>()
            .expect("valid injection export");
        assert_eq!(export.len, 2);

        let start = export.transitions[0].expect("start transition");
        let end = export.transitions[1].expect("end transition");
        assert!(matches!(start.level, ScheduledLevel::High));
        assert!(matches!(end.level, ScheduledLevel::Low));
        assert_eq!(start.channel.get(), 1);
        assert_eq!(end.channel.get(), 1);
        assert_eq!(start.at_us.get(), 100 + i * 50);
        assert_eq!(end.at_us.get(), 110 + i * 50);

        observed.push(start);
        observed.push(end);
    }

    assert_eq!(observed.len(), 20);
    assert!(observed
        .windows(2)
        .all(|pair| pair[0].at_us.get() < pair[1].at_us.get()));

    for (index, transition) in observed.iter().enumerate() {
        if index % 2 == 0 {
            assert!(matches!(transition.level, ScheduledLevel::High));
        } else {
            assert!(matches!(transition.level, ScheduledLevel::Low));
        }
    }
}

#[test]
fn stale_deadline_rejection_keeps_existing_scheduler_state() {
    let mut state = SchedulerState::new();
    let plan = InjectionPlan {
        output: ExclusiveChannel::new(OutputGroup::Injector, ChannelId::new(1)),
        pulse_width: PulseWidthUs::new(1200),
    };

    state
        .schedule_injection(Micros::new(10), Micros::new(20), Micros::new(30), plan)
        .expect("initial schedule should succeed");

    let before = observe_scheduler(&state);
    assert_eq!(before.active_groups, OutputGroup::Injector.mask());
    assert_eq!(before.injection_count, 1);

    let retry = state.schedule_injection(Micros::new(100), Micros::new(90), Micros::new(120), plan);
    assert_eq!(retry, Err(ScheduleError::StaleDeadline));

    let after = observe_scheduler(&state);
    assert_eq!(after.mode, before.mode);
    assert_eq!(after.active_groups, before.active_groups);
    assert_eq!(after.reserved_channels, before.reserved_channels);
    assert_eq!(after.injection_count, before.injection_count);
}

#[test]
fn ignition_fire_before_dwell_start_keeps_existing_scheduler_state() {
    let mut state = SchedulerState::new();
    let plan = IgnitionPlan {
        output: ExclusiveChannel::new(OutputGroup::Ignition, ChannelId::new(0)),
        dwell: DwellUs::new(2000),
        advance: Degrees10::new(150),
    };

    state
        .schedule_ignition(Micros::new(10), Micros::new(20), Micros::new(30), plan)
        .expect("initial ignition schedule should succeed");

    let before = observe_scheduler(&state);
    assert_eq!(before.active_groups, OutputGroup::Ignition.mask());
    assert_eq!(before.ignition_count, 1);

    let retry = state.schedule_ignition(Micros::new(100), Micros::new(150), Micros::new(120), plan);
    assert_eq!(retry, Err(ScheduleError::StaleDeadline));

    let after = observe_scheduler(&state);
    assert_eq!(after.mode, before.mode);
    assert_eq!(after.active_groups, before.active_groups);
    assert_eq!(after.reserved_channels, before.reserved_channels);
    assert_eq!(after.ignition_count, before.ignition_count);
}

#[test]
fn ignition_export_matches_injection_export_shape() {
    let ignition = TimedIgnitionPlan {
        plan: IgnitionPlan {
            output: ExclusiveChannel::new(OutputGroup::Ignition, ChannelId::new(0)),
            dwell: DwellUs::new(2500),
            advance: Degrees10::new(150),
        },
        start_at: Micros::new(500),
        end_at: Micros::new(3500),
    };

    let export = ignition
        .export_transitions::<4>()
        .expect("valid ignition export");
    assert_eq!(export.len, 2);

    let start = export.transitions[0].expect("start transition");
    let end = export.transitions[1].expect("end transition");
    assert!(matches!(start.level, ScheduledLevel::High));
    assert!(matches!(end.level, ScheduledLevel::Low));
    assert_eq!(start.channel.get(), 0);
    assert_eq!(end.channel.get(), 0);
    assert_eq!(start.at_us.get(), 500);
    assert_eq!(end.at_us.get(), 3500);
}

#[test]
fn injection_export_preserves_start_before_end_across_timer_wrap() {
    let injection = TimedInjectionPlan {
        plan: InjectionPlan {
            output: ExclusiveChannel::new(OutputGroup::Injector, ChannelId::new(1)),
            pulse_width: PulseWidthUs::new(64),
        },
        start_at: Micros::new(u32::MAX - 31),
        end_at: Micros::new(32),
    };

    let export = injection
        .export_transitions::<2>()
        .expect("wrapped injection export is valid");
    let start = export.transitions[0].expect("start transition");
    let end = export.transitions[1].expect("end transition");

    assert_eq!(start.level, ScheduledLevel::High);
    assert_eq!(start.at_us.get(), u32::MAX - 31);
    assert_eq!(end.level, ScheduledLevel::Low);
    assert_eq!(end.at_us.get(), 32);
}

#[test]
fn ignition_export_preserves_dwell_before_fire_across_timer_wrap() {
    let ignition = TimedIgnitionPlan {
        plan: IgnitionPlan {
            output: ExclusiveChannel::new(OutputGroup::Ignition, ChannelId::new(0)),
            dwell: DwellUs::new(64),
            advance: Degrees10::new(150),
        },
        start_at: Micros::new(u32::MAX - 31),
        end_at: Micros::new(32),
    };

    let export = ignition
        .export_transitions::<2>()
        .expect("wrapped ignition export is valid");
    let start = export.transitions[0].expect("dwell start transition");
    let end = export.transitions[1].expect("fire transition");

    assert_eq!(start.level, ScheduledLevel::High);
    assert_eq!(start.at_us.get(), u32::MAX - 31);
    assert_eq!(end.level, ScheduledLevel::Low);
    assert_eq!(end.at_us.get(), 32);
}

fn transition(at_us: u32, channel: u8, level: ScheduledLevel) -> ScheduledTransition {
    ScheduledTransition {
        at_us: Micros::new(at_us),
        kind: ScheduledTransitionKind::Injector,
        channel: ChannelId::new(channel),
        level,
    }
}

#[test]
fn transition_queue_drains_in_deadline_order_without_jitter() {
    let mut queue = ScheduledTransitionQueue::<24>::new();
    for i in 0..10u32 {
        let on = 100 + i * 50;
        let off = on + 10;
        queue
            .enqueue_transition(transition(on, 1, ScheduledLevel::High))
            .expect("on transition fits");
        queue
            .enqueue_transition(transition(off, 1, ScheduledLevel::Low))
            .expect("off transition fits");
    }

    let mut drained = TransitionDrainBuffer::<24>::new();
    let count = queue.drain_due(Micros::new(1000), &mut drained);

    assert_eq!(count, 20);
    assert_eq!(drained.len, 20);
    assert_eq!(queue.active_count(), 0);

    let transitions = drained.as_slice();
    assert!(transitions.windows(2).all(|pair| {
        pair[0].expect("transition").at_us.get() < pair[1].expect("transition").at_us.get()
    }));

    let mut high_count = 0;
    let mut low_count = 0;
    for (index, item) in transitions.iter().enumerate() {
        let item = item.expect("drained transition");
        assert_eq!(item.channel.get(), 1);
        if index % 2 == 0 {
            assert_eq!(item.level, ScheduledLevel::High);
            high_count += 1;
        } else {
            assert_eq!(item.level, ScheduledLevel::Low);
            low_count += 1;
        }
    }
    assert_eq!(high_count, 10);
    assert_eq!(low_count, 10);
}

#[test]
fn transition_queue_keeps_future_events_after_partial_drain() {
    let mut queue = ScheduledTransitionQueue::<4>::new();
    queue
        .enqueue_transition(transition(100, 1, ScheduledLevel::High))
        .expect("fits");
    queue
        .enqueue_transition(transition(200, 1, ScheduledLevel::Low))
        .expect("fits");

    let mut drained = TransitionDrainBuffer::<4>::new();
    assert_eq!(queue.drain_due(Micros::new(150), &mut drained), 1);
    assert_eq!(drained.len, 1);
    assert_eq!(
        drained.transitions[0].expect("due transition").at_us.get(),
        100
    );
    assert_eq!(queue.active_count(), 1);

    assert_eq!(queue.drain_due(Micros::new(250), &mut drained), 1);
    assert_eq!(
        drained.transitions[0]
            .expect("future transition")
            .at_us
            .get(),
        200
    );
    assert_eq!(queue.active_count(), 0);
}

#[test]
fn transition_queue_cutoff_cancels_only_future_channel_events() {
    let mut queue = ScheduledTransitionQueue::<4>::new();
    queue
        .enqueue_transition(transition(100, 1, ScheduledLevel::High))
        .expect("fits");
    queue
        .enqueue_transition(transition(150, 1, ScheduledLevel::Low))
        .expect("fits");
    queue
        .enqueue_transition(transition(100_000, 1, ScheduledLevel::High))
        .expect("fits");
    queue
        .enqueue_transition(transition(100_000, 2, ScheduledLevel::High))
        .expect("fits");

    queue.cancel_channel_after(ChannelId::new(1), Micros::new(160));
    let snapshot = queue.snapshot();

    let mut ch1_high = 0;
    let mut ch1_low = 0;
    let mut ch1_future = 0;
    let mut ch2_future = 0;
    for item in snapshot.transitions.iter().flatten() {
        if item.channel.get() == 1 {
            if item.at_us.get() >= 100_000 {
                ch1_future += 1;
            }
            match item.level {
                ScheduledLevel::High => ch1_high += 1,
                ScheduledLevel::Low => ch1_low += 1,
            }
        }
        if item.channel.get() == 2 && item.at_us.get() >= 100_000 {
            ch2_future += 1;
        }
    }

    assert_eq!(
        ch1_future, 0,
        "far future event removed for selected channel"
    );
    assert_eq!(ch1_high, 1, "imminent ON remains");
    assert_eq!(ch1_low, 1, "imminent OFF remains");
    assert_eq!(ch2_future, 1, "other channel is untouched");
}

#[test]
fn transition_queue_uses_wrapping_time_for_due_events() {
    let mut queue = ScheduledTransitionQueue::<2>::new();
    queue
        .enqueue_transition(transition(u32::MAX - 15, 1, ScheduledLevel::High))
        .expect("fits");
    queue
        .enqueue_transition(transition(500, 1, ScheduledLevel::Low))
        .expect("fits");

    let mut drained = TransitionDrainBuffer::<2>::new();
    assert_eq!(queue.drain_due(Micros::new(20), &mut drained), 1);
    assert_eq!(
        drained.transitions[0]
            .expect("wrapped transition")
            .at_us
            .get(),
        u32::MAX - 15
    );
    assert_eq!(queue.active_count(), 1);
}

#[test]
fn transition_queue_starts_empty() {
    let queue = ScheduledTransitionQueue::<4>::new();

    assert_eq!(queue.active_count(), 0);
    assert_eq!(queue.capacity(), 4);
    assert_eq!(queue.free_slots(), 4);
    assert!(!queue.is_full());
}

#[test]
fn transition_queue_reports_capacity_and_full_errors() {
    let mut queue = ScheduledTransitionQueue::<2>::new();
    assert_eq!(queue.capacity(), 2);
    assert_eq!(queue.free_slots(), 2);
    assert!(!queue.is_full());

    queue
        .enqueue_transition(transition(100, 1, ScheduledLevel::High))
        .expect("first fits");
    queue
        .enqueue_transition(transition(110, 1, ScheduledLevel::Low))
        .expect("second fits");

    assert!(queue.is_full());
    assert_eq!(queue.free_slots(), 0);
    assert_eq!(
        queue.enqueue_transition(transition(120, 1, ScheduledLevel::High)),
        Err(ScheduleError::QueueFull)
    );
}

#[test]
fn transition_queue_sync_loss_and_shutdown_clear_pending_work() {
    let mut queue = ScheduledTransitionQueue::<4>::new();
    queue
        .enqueue_transition(transition(100, 1, ScheduledLevel::High))
        .expect("fits");
    queue
        .enqueue_transition(transition(110, 1, ScheduledLevel::Low))
        .expect("fits");

    queue.on_sync_loss();
    assert_eq!(queue.active_count(), 0);

    queue
        .enqueue_transition(transition(200, 1, ScheduledLevel::High))
        .expect("fits");
    queue.on_hard_safety_shutdown();
    assert_eq!(queue.active_count(), 0);
}

#[test]
fn transition_queue_can_enqueue_export_atomically() {
    let injection = TimedInjectionPlan {
        plan: InjectionPlan {
            output: ExclusiveChannel::new(OutputGroup::Injector, ChannelId::new(3)),
            pulse_width: PulseWidthUs::new(800),
        },
        start_at: Micros::new(100),
        end_at: Micros::new(120),
    };
    let export = injection
        .export_transitions::<2>()
        .expect("valid injection export");
    let mut queue = ScheduledTransitionQueue::<2>::new();

    queue.enqueue_export(&export).expect("both transitions fit");
    assert_eq!(queue.active_count(), 2);

    let mut drained = TransitionDrainBuffer::<2>::new();
    assert_eq!(queue.drain_due(Micros::new(200), &mut drained), 2);
    assert_eq!(
        drained.transitions[0].expect("start").level,
        ScheduledLevel::High
    );
    assert_eq!(
        drained.transitions[1].expect("end").level,
        ScheduledLevel::Low
    );
}
