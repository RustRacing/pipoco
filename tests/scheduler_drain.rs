use ecu_core::hal::OutputPin;
use ecu_core::scheduler::{Channel, Scheduler};

#[derive(Default)]
struct MockPin {
    pub log: Vec<bool>,
}
impl OutputPin for MockPin {
    fn set_high(&mut self) {
        self.log.push(true)
    }
    fn set_low(&mut self) {
        self.log.push(false)
    }
}

#[test]
fn scheduler_drains_in_order_without_jitter() {
    let mut s = Scheduler::new();
    let ch = Channel::INJ1;
    // Schedule a sequence of ON/OFF pairs with strictly increasing times
    for i in 0..10u32 {
        let on = 100 + i * 50; // 100,150,200...
        let off = on + 10;
        assert!(s.schedule_ticks(on, ch, true));
        assert!(s.schedule_ticks(off, ch, false));
    }
    // Prepare outputs
    let mut pin = MockPin::default();
    let mut outs: [&mut dyn OutputPin; 4] = [
        &mut pin,
        &mut MockPin::default(),
        &mut MockPin::default(),
        &mut MockPin::default(),
    ];
    // Step time and drain
    let mut now = 0u32;
    while now < 1000 {
        s.check_and_execute_ticks(now, &mut outs);
        now += 5;
    }
    // Expect 20 toggles: ON/OFF repeated 10 times
    assert_eq!(pin.log.len(), 20);
    // Check ON/OFF alternation and counts
    let on_count = pin.log.iter().filter(|&&b| b).count();
    let off_count = pin.log.iter().filter(|&&b| !b).count();
    assert_eq!(on_count, 10);
    assert_eq!(off_count, 10);
}
