use ecu_core::scheduler::{Scheduler, Channel};

#[test]
fn deactivate_after_keeps_imminent_events() {
    let mut s = Scheduler::new();
    let ch = Channel::INJ1;
    // Schedule three events: two imminent (100, 150) and one far future (100_000)
    assert!(s.schedule_ticks(100, ch, true));
    assert!(s.schedule_ticks(150, ch, false));
    assert!(s.schedule_ticks(100_000, ch, true));

    // Deactivate anything after cutoff=160 -> should only drop the 100_000 event
    s.deactivate_channel_after(160, ch);
    let (mut on, mut off, mut future) = (0, 0, 0);
    for e in s.events_mut().iter().filter(|e| e.is_active() && e.channel().as_u8() == ch.as_u8()) {
        if e.time() >= 100_000 { future += 1; }
        if e.state() { on += 1; } else { off += 1; }
    }
    assert_eq!(future, 0, "far future event removed");
    assert_eq!(on, 1, "one ON remains");
    assert_eq!(off, 1, "one OFF remains");
}

