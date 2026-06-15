use super::{
    trigger_mask, CaptureBuffer, CaptureConfig, CaptureFrame, CaptureState, CaptureTrigger,
    TriggeredCapture,
};

#[test]
fn test_capture_buffer_basic() {
    let mut buf = CaptureBuffer::<4>::new();
    assert!(buf.try_pop().is_none());

    buf.push(100);
    buf.push(200);
    assert_eq!(buf.try_pop(), Some(100));
    assert_eq!(buf.try_pop(), Some(200));
    assert!(buf.try_pop().is_none());
}

#[test]
fn test_capture_trigger_mask() {
    assert_eq!(CaptureTrigger::Manual.mask(), 0x01);
    assert_eq!(CaptureTrigger::SyncLoss.mask(), 0x02);
    assert_eq!(CaptureTrigger::KnockDetected.mask(), 0x04);
}

#[test]
fn test_capture_config_trigger_enabled() {
    let config = CaptureConfig {
        enabled_triggers: trigger_mask::SYNC_LOSS | trigger_mask::KNOCK,
        ..CaptureConfig::DEFAULT
    };

    assert!(!config.is_trigger_enabled(CaptureTrigger::Manual));
    assert!(config.is_trigger_enabled(CaptureTrigger::SyncLoss));
    assert!(config.is_trigger_enabled(CaptureTrigger::KnockDetected));
    assert!(!config.is_trigger_enabled(CaptureTrigger::RpmSpike));
}

#[test]
fn test_capture_frame_from_state() {
    let frame = CaptureFrame::from_state(1000, 3000, 800, 50, 12500, 25, true, false, true);

    assert_eq!(frame.timestamp_us, 1000);
    assert_eq!(frame.rpm, 3000);
    assert_eq!(frame.map_kpa_x10, 800);
    assert_eq!(frame.tps_percent, 50);
    assert_eq!(frame.voltage_mv_div100, 125);
    assert_eq!(frame.stft_offset, 130);
    assert_eq!(frame.status, 0x05);
}

#[test]
fn test_triggered_capture_new() {
    let capture = TriggeredCapture::<32>::new();
    assert_eq!(capture.state, CaptureState::Armed);
    assert_eq!(capture.sample_count(), 0);
    assert!(capture.trigger_reason.is_none());
}

#[test]
fn test_triggered_capture_sample() {
    let mut capture = TriggeredCapture::<32>::new();
    capture.config.sample_interval_us = 0;

    for i in 0..10 {
        let frame = CaptureFrame {
            timestamp_us: i * 1000,
            rpm: 3000,
            ..CaptureFrame::new()
        };
        capture.sample(frame, i * 1000);
    }

    assert_eq!(capture.sample_count(), 10);
    assert!(capture.is_armed());
}

#[test]
fn test_triggered_capture_trigger() {
    let mut capture = TriggeredCapture::<32>::new();
    capture.config.sample_interval_us = 0;
    capture.config.enabled_triggers = trigger_mask::ALL;
    capture.config.post_trigger_samples = 5;

    for i in 0..10 {
        let frame = CaptureFrame {
            timestamp_us: i * 1000,
            rpm: 3000,
            ..CaptureFrame::new()
        };
        capture.sample(frame, i * 1000);
    }

    assert!(capture.is_armed());

    capture.trigger(CaptureTrigger::SyncLoss);
    assert_eq!(capture.state, CaptureState::Capturing);
    assert_eq!(capture.trigger_reason, Some(CaptureTrigger::SyncLoss));

    for i in 10..20 {
        let frame = CaptureFrame {
            timestamp_us: i * 1000,
            rpm: 3000,
            ..CaptureFrame::new()
        };
        capture.sample(frame, i * 1000);
    }

    assert!(capture.is_complete());
}

#[test]
fn test_triggered_capture_disabled_trigger() {
    let mut capture = TriggeredCapture::<32>::new();
    capture.config.enabled_triggers = trigger_mask::SYNC_LOSS;

    capture.trigger(CaptureTrigger::KnockDetected);
    assert!(capture.is_armed());
    assert!(capture.trigger_reason.is_none());

    capture.trigger(CaptureTrigger::SyncLoss);
    assert_eq!(capture.state, CaptureState::Capturing);
}

#[test]
fn test_triggered_capture_ring_buffer_wrap() {
    let mut capture = TriggeredCapture::<8>::new();
    capture.config.sample_interval_us = 0;

    for i in 0..15 {
        let frame = CaptureFrame {
            timestamp_us: i * 1000,
            rpm: (i + 1) as u16 * 100,
            ..CaptureFrame::new()
        };
        capture.sample(frame, i * 1000);
    }

    assert_eq!(capture.sample_count(), 8);
    let oldest = capture.get_sample(0).unwrap();
    assert_eq!(oldest.rpm, 800);
}

#[test]
fn test_triggered_capture_reset() {
    let mut capture = TriggeredCapture::<32>::new();
    capture.config.sample_interval_us = 0;
    capture.config.enabled_triggers = trigger_mask::ALL;

    for i in 0..5 {
        capture.sample(CaptureFrame::new(), i * 1000);
    }
    capture.trigger(CaptureTrigger::Manual);

    assert_eq!(capture.state, CaptureState::Capturing);

    capture.reset();

    assert_eq!(capture.state, CaptureState::Armed);
    assert_eq!(capture.sample_count(), 0);
    assert!(capture.trigger_reason.is_none());
}

#[test]
fn test_triggered_capture_get_sample() {
    let mut capture = TriggeredCapture::<8>::new();
    capture.config.sample_interval_us = 0;

    for i in 0..5 {
        let frame = CaptureFrame {
            timestamp_us: i * 1000,
            rpm: (i + 1) as u16 * 100,
            ..CaptureFrame::new()
        };
        capture.sample(frame, i * 1000);
    }

    assert_eq!(capture.get_sample(0).unwrap().rpm, 100);
    assert_eq!(capture.get_sample(4).unwrap().rpm, 500);
    assert!(capture.get_sample(5).is_none());
}

#[test]
fn test_capture_frame_serialization() {
    let frame = CaptureFrame {
        timestamp_us: 12345678,
        rpm: 3500,
        map_kpa_x10: 850,
        tps_percent: 75,
        voltage_mv_div100: 125,
        stft_offset: 135,
        status: 0x07,
    };

    let bytes = frame.to_bytes();
    assert_eq!(bytes.len(), CaptureFrame::SIZE);

    let ts = u32::from_le_bytes([bytes[0], bytes[1], bytes[2], bytes[3]]);
    assert_eq!(ts, 12345678);

    let rpm = u16::from_le_bytes([bytes[4], bytes[5]]);
    assert_eq!(rpm, 3500);
}

#[test]
fn test_triggered_capture_sample_rate_limiting() {
    let mut capture = TriggeredCapture::<32>::new();
    capture.config.sample_interval_us = 10_000;

    capture.sample(CaptureFrame::new(), 0);
    capture.sample(CaptureFrame::new(), 5_000);
    capture.sample(CaptureFrame::new(), 10_000);
    capture.sample(CaptureFrame::new(), 15_000);
    capture.sample(CaptureFrame::new(), 20_000);

    assert_eq!(capture.sample_count(), 3);
}

#[test]
fn test_triggered_capture_disable_enable() {
    let mut capture = TriggeredCapture::<32>::new();

    capture.disable();
    assert_eq!(capture.state, CaptureState::Disabled);

    capture.enable();
    assert_eq!(capture.state, CaptureState::Armed);
}

#[test]
fn test_triggered_capture_post_trigger_exact_count() {
    let mut capture = TriggeredCapture::<32>::new();
    capture.config.sample_interval_us = 0;
    capture.config.enabled_triggers = trigger_mask::MANUAL;
    capture.config.post_trigger_samples = 3;

    for i in 0..5u32 {
        capture.sample(
            CaptureFrame {
                rpm: 3000,
                ..CaptureFrame::new()
            },
            i * 1000,
        );
    }

    capture.trigger(CaptureTrigger::Manual);
    assert_eq!(capture.state, CaptureState::Capturing);

    capture.sample(
        CaptureFrame {
            rpm: 3000,
            ..CaptureFrame::new()
        },
        10_000,
    );
    assert_eq!(capture.state, CaptureState::Capturing);

    capture.sample(
        CaptureFrame {
            rpm: 3000,
            ..CaptureFrame::new()
        },
        11_000,
    );
    assert_eq!(capture.state, CaptureState::Capturing);

    capture.sample(
        CaptureFrame {
            rpm: 3000,
            ..CaptureFrame::new()
        },
        12_000,
    );
    assert!(
        capture.is_complete(),
        "Should be complete after 3 post-trigger samples"
    );
    assert_eq!(capture.sample_count(), 8);
}

#[test]
fn test_triggered_capture_trigger_sample_access() {
    let mut capture = TriggeredCapture::<32>::new();
    capture.config.sample_interval_us = 0;
    capture.config.enabled_triggers = trigger_mask::SYNC_LOSS;
    capture.config.post_trigger_samples = 2;

    for i in 0..5u32 {
        capture.sample(
            CaptureFrame {
                rpm: 3000,
                timestamp_us: (i + 1) * 1000,
                ..CaptureFrame::new()
            },
            i * 1000,
        );
    }

    capture.trigger(CaptureTrigger::SyncLoss);

    let trigger_sample = capture.get_trigger_sample().unwrap();
    assert_eq!(trigger_sample.timestamp_us, 5000);

    capture.sample(
        CaptureFrame {
            rpm: 3000,
            timestamp_us: 6000,
            ..CaptureFrame::new()
        },
        10_000,
    );
    capture.sample(
        CaptureFrame {
            rpm: 3000,
            timestamp_us: 7000,
            ..CaptureFrame::new()
        },
        11_000,
    );

    assert!(capture.is_complete());
    let trigger_sample = capture.get_trigger_sample().unwrap();
    assert_eq!(trigger_sample.timestamp_us, 5000);
}

#[test]
fn test_triggered_capture_complete_ignores_samples() {
    let mut capture = TriggeredCapture::<32>::new();
    capture.config.sample_interval_us = 0;
    capture.config.enabled_triggers = trigger_mask::ALL;
    capture.config.post_trigger_samples = 1;

    capture.sample(
        CaptureFrame {
            rpm: 100,
            ..CaptureFrame::new()
        },
        0,
    );
    capture.trigger(CaptureTrigger::Manual);
    capture.sample(
        CaptureFrame {
            rpm: 200,
            ..CaptureFrame::new()
        },
        1000,
    );

    assert!(capture.is_complete());
    let count_before = capture.sample_count();

    capture.sample(
        CaptureFrame {
            rpm: 300,
            ..CaptureFrame::new()
        },
        2000,
    );
    capture.sample(
        CaptureFrame {
            rpm: 400,
            ..CaptureFrame::new()
        },
        3000,
    );

    assert_eq!(capture.sample_count(), count_before);
}

#[test]
fn test_triggered_capture_retrigger_after_reset() {
    let mut capture = TriggeredCapture::<32>::new();
    capture.config.sample_interval_us = 0;
    capture.config.enabled_triggers = trigger_mask::ALL;
    capture.config.post_trigger_samples = 1;

    capture.sample(CaptureFrame::new(), 0);
    capture.trigger(CaptureTrigger::SyncLoss);
    capture.sample(CaptureFrame::new(), 1000);
    assert!(capture.is_complete());
    assert_eq!(capture.trigger_reason, Some(CaptureTrigger::SyncLoss));

    capture.reset();
    assert!(capture.is_armed());
    assert!(capture.trigger_reason.is_none());

    capture.sample(CaptureFrame::new(), 10_000);
    capture.trigger(CaptureTrigger::KnockDetected);
    assert_eq!(capture.trigger_reason, Some(CaptureTrigger::KnockDetected));
}

#[test]
fn test_triggered_capture_rpm_spike_detection() {
    let mut capture = TriggeredCapture::<32>::new();
    capture.config.sample_interval_us = 0;
    capture.config.enabled_triggers = trigger_mask::RPM_SPIKE;
    capture.config.rpm_spike_threshold = 5000;
    capture.config.post_trigger_samples = 2;

    capture.sample(
        CaptureFrame {
            rpm: 3000,
            ..CaptureFrame::new()
        },
        0,
    );
    capture.sample(
        CaptureFrame {
            rpm: 3000,
            ..CaptureFrame::new()
        },
        1000,
    );
    assert!(capture.is_armed());

    capture.sample(
        CaptureFrame {
            rpm: 3100,
            ..CaptureFrame::new()
        },
        2000,
    );

    assert_eq!(capture.state, CaptureState::Capturing);
    assert_eq!(capture.trigger_reason, Some(CaptureTrigger::RpmSpike));
}

#[test]
fn test_triggered_capture_all_trigger_types() {
    for trigger_type in [
        CaptureTrigger::Manual,
        CaptureTrigger::SyncLoss,
        CaptureTrigger::KnockDetected,
        CaptureTrigger::SensorFault,
        CaptureTrigger::LimpEntry,
        CaptureTrigger::LambdaFault,
        CaptureTrigger::LowVoltage,
    ] {
        let mut capture = TriggeredCapture::<8>::new();
        capture.config.enabled_triggers = trigger_type.mask();
        capture.config.sample_interval_us = 0;

        capture.sample(CaptureFrame::new(), 0);
        capture.trigger(trigger_type);
        assert_eq!(
            capture.trigger_reason,
            Some(trigger_type),
            "Trigger {:?} should be enabled",
            trigger_type
        );

        capture.reset();
        let other_type = if trigger_type == CaptureTrigger::Manual {
            CaptureTrigger::SyncLoss
        } else {
            CaptureTrigger::Manual
        };
        capture.trigger(other_type);
        assert!(
            capture.trigger_reason.is_none(),
            "Trigger {:?} should be disabled when only {:?} is enabled",
            other_type,
            trigger_type
        );
    }
}

#[test]
fn test_triggered_capture_pre_trigger_count() {
    let mut capture = TriggeredCapture::<8>::new();
    capture.config.sample_interval_us = 0;
    capture.config.enabled_triggers = trigger_mask::MANUAL;
    capture.config.post_trigger_samples = 2;

    for i in 0..3u32 {
        capture.sample(
            CaptureFrame {
                rpm: 3000,
                timestamp_us: (i + 1) * 1000,
                ..CaptureFrame::new()
            },
            i * 1000,
        );
    }

    assert_eq!(capture.sample_count(), 3);

    capture.trigger(CaptureTrigger::Manual);

    capture.sample(
        CaptureFrame {
            rpm: 3000,
            timestamp_us: 4000,
            ..CaptureFrame::new()
        },
        10_000,
    );
    capture.sample(
        CaptureFrame {
            rpm: 3000,
            timestamp_us: 5000,
            ..CaptureFrame::new()
        },
        11_000,
    );

    assert!(capture.is_complete());
    assert_eq!(capture.sample_count(), 5);

    assert_eq!(capture.get_sample(0).unwrap().timestamp_us, 1000);
    assert_eq!(capture.get_sample(1).unwrap().timestamp_us, 2000);
    assert_eq!(capture.get_sample(2).unwrap().timestamp_us, 3000);
    assert_eq!(capture.get_sample(3).unwrap().timestamp_us, 4000);
    assert_eq!(capture.get_sample(4).unwrap().timestamp_us, 5000);
}
