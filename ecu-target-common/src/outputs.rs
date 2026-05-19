use ecu_core::hal::OutputPin as EcuOutputPin;
use ecu_domain::ChannelId;
use ecu_io::ActionExecutor;
use ecu_runtime::Action;
use ecu_scheduler::{
    ScheduleError, ScheduledLevel, ScheduledTransition, ScheduledTransitionKind,
    ScheduledTransitionQueue, TransitionDrainBuffer,
};

/// Types that can receive drained split-scheduler transitions without heap
/// allocation.
pub trait ScheduledOutputsLike {
    fn drain_and_apply_due<const Q: usize, const D: usize>(
        &mut self,
        executor: &mut ScheduledActionExecutor<Q>,
        now: ecu_scheduler::Micros,
        drained: &mut TransitionDrainBuffer<D>,
    ) -> Result<usize, TransitionApplyError>;
}

/// Compatibility wrapper for embedded-hal v0.2 output pins.
///
/// This preserves the old public `HalOut` name while the split scheduler
/// implementation now works directly with the generic `ScheduledOutputs`
/// family.
pub struct HalOut<P> {
    pin: P,
}

impl<P> HalOut<P> {
    pub fn new(pin: P) -> Self {
        Self { pin }
    }

    pub fn into_inner(self) -> P {
        self.pin
    }
}

impl<P> EcuOutputPin for HalOut<P>
where
    P: embedded_hal::digital::v2::OutputPin,
{
    fn set_high(&mut self) {
        let _ = self.pin.set_high();
    }

    fn set_low(&mut self) {
        let _ = self.pin.set_low();
    }
}

impl<P> ScheduledOutputPin for HalOut<P>
where
    P: embedded_hal::digital::v2::OutputPin,
{
    fn set_scheduled_high(&mut self) {
        self.set_high();
    }

    fn set_scheduled_low(&mut self) {
        self.set_low();
    }
}

/// Const-generic holder for split-scheduler outputs.
///
/// This stores homogeneous injector and ignition pin arrays while keeping the
/// older `ScheduledOutputs4` wrapper available for existing callers that
/// provide distinct pin types per channel.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ScheduledOutputs<const INJ: usize, const IGN: usize, I, G> {
    injectors: [I; INJ],
    ignition: [G; IGN],
}

impl<const INJ: usize, const IGN: usize, I, G> ScheduledOutputs<INJ, IGN, I, G>
where
    I: ScheduledOutputPin,
    G: ScheduledOutputPin,
{
    pub fn new(injectors: [I; INJ], ignition: [G; IGN]) -> Self {
        Self {
            injectors,
            ignition,
        }
    }

    pub fn into_inner(self) -> ([I; INJ], [G; IGN]) {
        (self.injectors, self.ignition)
    }

    fn validate_transition(
        &self,
        transition: ScheduledTransition,
    ) -> Result<(), TransitionApplyError> {
        match transition.kind {
            ScheduledTransitionKind::Injector => {
                if transition.channel.get() as usize >= self.injectors.len() {
                    return Err(TransitionApplyError::InjectorChannelOutOfRange(
                        transition.channel,
                    ));
                }
            }
            ScheduledTransitionKind::Ignition => {
                if transition.channel.get() as usize >= self.ignition.len() {
                    return Err(TransitionApplyError::IgnitionChannelOutOfRange(
                        transition.channel,
                    ));
                }
            }
        }
        Ok(())
    }

    fn apply_transition(&mut self, transition: ScheduledTransition) {
        match transition.kind {
            ScheduledTransitionKind::Injector => {
                let pin = &mut self.injectors[transition.channel.get() as usize];
                match transition.level {
                    ScheduledLevel::High => pin.set_scheduled_high(),
                    ScheduledLevel::Low => pin.set_scheduled_low(),
                }
            }
            ScheduledTransitionKind::Ignition => {
                let pin = &mut self.ignition[transition.channel.get() as usize];
                match transition.level {
                    ScheduledLevel::High => pin.set_scheduled_high(),
                    ScheduledLevel::Low => pin.set_scheduled_low(),
                }
            }
        }
    }

    fn apply_drained_transitions<const N: usize>(
        &mut self,
        drained: &TransitionDrainBuffer<N>,
    ) -> Result<usize, TransitionApplyError> {
        let mut idx = 0;
        while idx < drained.len as usize {
            if let Some(transition) = drained.transitions[idx] {
                self.validate_transition(transition)?;
            }
            idx += 1;
        }

        let mut applied = 0;
        idx = 0;
        while idx < drained.len as usize {
            if let Some(transition) = drained.transitions[idx] {
                self.apply_transition(transition);
                applied += 1;
            }
            idx += 1;
        }
        Ok(applied)
    }

    pub fn drain_and_apply_due<const Q: usize, const D: usize>(
        &mut self,
        executor: &mut ScheduledActionExecutor<Q>,
        now: ecu_scheduler::Micros,
        drained: &mut TransitionDrainBuffer<D>,
    ) -> Result<usize, TransitionApplyError> {
        executor.drain_due(now, drained);
        self.apply_drained_transitions(drained)
    }
}

impl<const INJ: usize, const IGN: usize, I, G> ScheduledOutputsLike
    for ScheduledOutputs<INJ, IGN, I, G>
where
    I: ScheduledOutputPin,
    G: ScheduledOutputPin,
{
    fn drain_and_apply_due<const Q: usize, const D: usize>(
        &mut self,
        executor: &mut ScheduledActionExecutor<Q>,
        now: ecu_scheduler::Micros,
        drained: &mut TransitionDrainBuffer<D>,
    ) -> Result<usize, TransitionApplyError> {
        ScheduledOutputs::drain_and_apply_due(self, executor, now, drained)
    }
}

/// Holder for two injector and two ignition outputs for the split scheduler
/// path.
pub struct ScheduledOutputs4<I1, I2, G1, G2> {
    inj1: I1,
    inj2: I2,
    ign1: G1,
    ign2: G2,
}

impl<I1, I2, G1, G2> ScheduledOutputs4<I1, I2, G1, G2>
where
    I1: ScheduledOutputPin,
    I2: ScheduledOutputPin,
    G1: ScheduledOutputPin,
    G2: ScheduledOutputPin,
{
    pub fn new(inj1: I1, inj2: I2, ign1: G1, ign2: G2) -> Self {
        Self {
            inj1,
            inj2,
            ign1,
            ign2,
        }
    }

    pub fn into_inner(self) -> (I1, I2, G1, G2) {
        (self.inj1, self.inj2, self.ign1, self.ign2)
    }

    pub fn as_scheduled_pins(
        &mut self,
    ) -> (
        [&mut dyn ScheduledOutputPin; 2],
        [&mut dyn ScheduledOutputPin; 2],
    ) {
        (
            [&mut self.inj1, &mut self.inj2],
            [&mut self.ign1, &mut self.ign2],
        )
    }

    pub fn drain_and_apply_due<const Q: usize, const D: usize>(
        &mut self,
        executor: &mut ScheduledActionExecutor<Q>,
        now: ecu_scheduler::Micros,
        drained: &mut TransitionDrainBuffer<D>,
    ) -> Result<usize, TransitionApplyError> {
        let (mut injectors, mut ignition) = self.as_scheduled_pins();
        executor.drain_and_apply_due(now, drained, &mut injectors, &mut ignition)
    }
}

impl<I1, I2, G1, G2> ScheduledOutputsLike for ScheduledOutputs4<I1, I2, G1, G2>
where
    I1: ScheduledOutputPin,
    I2: ScheduledOutputPin,
    G1: ScheduledOutputPin,
    G2: ScheduledOutputPin,
{
    fn drain_and_apply_due<const Q: usize, const D: usize>(
        &mut self,
        executor: &mut ScheduledActionExecutor<Q>,
        now: ecu_scheduler::Micros,
        drained: &mut TransitionDrainBuffer<D>,
    ) -> Result<usize, TransitionApplyError> {
        ScheduledOutputs4::drain_and_apply_due(self, executor, now, drained)
    }
}

/// Backward-compatible four-output wrapper.
///
/// The legacy API stored four individually wrapped pins and exposed them as
/// trait objects for the older board adapters. This keeps that shape available
/// while the split scheduler can use `ScheduledOutputs4`/`ScheduledOutputs`
/// directly.
pub struct Outputs4<I1, I2, G1, G2> {
    inj1: HalOut<I1>,
    inj2: HalOut<I2>,
    ign1: HalOut<G1>,
    ign2: HalOut<G2>,
}

impl<I1, I2, G1, G2> Outputs4<I1, I2, G1, G2>
where
    I1: embedded_hal::digital::v2::OutputPin,
    I2: embedded_hal::digital::v2::OutputPin,
    G1: embedded_hal::digital::v2::OutputPin,
    G2: embedded_hal::digital::v2::OutputPin,
{
    pub fn new(inj1: I1, inj2: I2, ign1: G1, ign2: G2) -> Self {
        Self {
            inj1: HalOut::new(inj1),
            inj2: HalOut::new(inj2),
            ign1: HalOut::new(ign1),
            ign2: HalOut::new(ign2),
        }
    }

    pub fn into_inner(self) -> (I1, I2, G1, G2) {
        (
            self.inj1.into_inner(),
            self.inj2.into_inner(),
            self.ign1.into_inner(),
            self.ign2.into_inner(),
        )
    }

    pub fn as_pins(&mut self) -> [&mut dyn EcuOutputPin; 4] {
        [
            &mut self.inj1,
            &mut self.inj2,
            &mut self.ign1,
            &mut self.ign2,
        ]
    }

    pub fn as_scheduled_pins(
        &mut self,
    ) -> (
        [&mut dyn ScheduledOutputPin; 2],
        [&mut dyn ScheduledOutputPin; 2],
    ) {
        (
            [&mut self.inj1, &mut self.inj2],
            [&mut self.ign1, &mut self.ign2],
        )
    }

    pub fn drain_and_apply_due<const Q: usize, const D: usize>(
        &mut self,
        executor: &mut ScheduledActionExecutor<Q>,
        now: ecu_scheduler::Micros,
        drained: &mut TransitionDrainBuffer<D>,
    ) -> Result<usize, TransitionApplyError> {
        let (mut injectors, mut ignition) = self.as_scheduled_pins();
        executor.drain_and_apply_due(now, drained, &mut injectors, &mut ignition)
    }
}

impl<I1, I2, G1, G2> ScheduledOutputsLike for Outputs4<I1, I2, G1, G2>
where
    I1: embedded_hal::digital::v2::OutputPin,
    I2: embedded_hal::digital::v2::OutputPin,
    G1: embedded_hal::digital::v2::OutputPin,
    G2: embedded_hal::digital::v2::OutputPin,
{
    fn drain_and_apply_due<const Q: usize, const D: usize>(
        &mut self,
        executor: &mut ScheduledActionExecutor<Q>,
        now: ecu_scheduler::Micros,
        drained: &mut TransitionDrainBuffer<D>,
    ) -> Result<usize, TransitionApplyError> {
        Outputs4::drain_and_apply_due(self, executor, now, drained)
    }
}

/// Minimal pin trait for applying split scheduler transitions.
///
/// This deliberately does not depend on `ecu_core::hal::OutputPin`, so the
/// split scheduler/target path can migrate independently from the root core.
pub trait ScheduledOutputPin {
    fn set_scheduled_high(&mut self);
    fn set_scheduled_low(&mut self);
}

impl<P> ScheduledOutputPin for P
where
    P: embedded_hal::digital::v2::OutputPin,
{
    fn set_scheduled_high(&mut self) {
        let _ = self.set_high();
    }

    fn set_scheduled_low(&mut self) {
        let _ = self.set_low();
    }
}

/// Wrapper for embedded-hal 1.0 output pins on split scheduler paths.
///
/// `ScheduledOutputPin` already has a blanket implementation for
/// embedded-hal 0.2 pins. This wrapper avoids overlapping blanket impls while
/// letting newer board crates use the same scheduler output helper.
pub struct Hal1ScheduledOut<P> {
    pin: P,
}

impl<P> Hal1ScheduledOut<P> {
    pub fn new(pin: P) -> Self {
        Self { pin }
    }

    pub fn into_inner(self) -> P {
        self.pin
    }
}

impl<P> ScheduledOutputPin for Hal1ScheduledOut<P>
where
    P: embedded_hal_1::digital::OutputPin,
{
    fn set_scheduled_high(&mut self) {
        let _ = self.pin.set_high();
    }

    fn set_scheduled_low(&mut self) {
        let _ = self.pin.set_low();
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TransitionApplyError {
    InjectorChannelOutOfRange(ChannelId),
    IgnitionChannelOutOfRange(ChannelId),
}

pub fn apply_transition(
    transition: ScheduledTransition,
    injectors: &mut [&mut dyn ScheduledOutputPin],
    ignition: &mut [&mut dyn ScheduledOutputPin],
) -> Result<(), TransitionApplyError> {
    match transition.kind {
        ScheduledTransitionKind::Injector => {
            let pin = injectors.get_mut(transition.channel.get() as usize).ok_or(
                TransitionApplyError::InjectorChannelOutOfRange(transition.channel),
            )?;
            match transition.level {
                ScheduledLevel::High => pin.set_scheduled_high(),
                ScheduledLevel::Low => pin.set_scheduled_low(),
            }
        }
        ScheduledTransitionKind::Ignition => {
            let pin = ignition.get_mut(transition.channel.get() as usize).ok_or(
                TransitionApplyError::IgnitionChannelOutOfRange(transition.channel),
            )?;
            match transition.level {
                ScheduledLevel::High => pin.set_scheduled_high(),
                ScheduledLevel::Low => pin.set_scheduled_low(),
            }
        }
    }
    Ok(())
}

pub fn apply_drained_transitions<const N: usize>(
    drained: &TransitionDrainBuffer<N>,
    injectors: &mut [&mut dyn ScheduledOutputPin],
    ignition: &mut [&mut dyn ScheduledOutputPin],
) -> Result<usize, TransitionApplyError> {
    let mut idx = 0;
    while idx < drained.len as usize {
        if let Some(transition) = drained.transitions[idx] {
            match transition.kind {
                ScheduledTransitionKind::Injector => {
                    if transition.channel.get() as usize >= injectors.len() {
                        return Err(TransitionApplyError::InjectorChannelOutOfRange(
                            transition.channel,
                        ));
                    }
                }
                ScheduledTransitionKind::Ignition => {
                    if transition.channel.get() as usize >= ignition.len() {
                        return Err(TransitionApplyError::IgnitionChannelOutOfRange(
                            transition.channel,
                        ));
                    }
                }
            }
        }
        idx += 1;
    }

    let mut applied = 0;
    idx = 0;
    while idx < drained.len as usize {
        if let Some(transition) = drained.transitions[idx] {
            apply_transition(transition, injectors, ignition)?;
            applied += 1;
        }
        idx += 1;
    }
    Ok(applied)
}

/// Action executor that bridges runtime scheduler actions into the split
/// scheduler transition queue.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ScheduledActionExecutor<const N: usize> {
    queue: ScheduledTransitionQueue<N>,
}

impl<const N: usize> ScheduledActionExecutor<N> {
    pub const fn new() -> Self {
        Self {
            queue: ScheduledTransitionQueue::new(),
        }
    }

    pub const fn queue(&self) -> &ScheduledTransitionQueue<N> {
        &self.queue
    }

    pub fn queue_mut(&mut self) -> &mut ScheduledTransitionQueue<N> {
        &mut self.queue
    }

    pub fn drain_due<const M: usize>(
        &mut self,
        now: ecu_scheduler::Micros,
        out: &mut TransitionDrainBuffer<M>,
    ) -> usize {
        self.queue.drain_due(now, out)
    }

    pub fn drain_and_apply_due<const M: usize>(
        &mut self,
        now: ecu_scheduler::Micros,
        out: &mut TransitionDrainBuffer<M>,
        injectors: &mut [&mut dyn ScheduledOutputPin],
        ignition: &mut [&mut dyn ScheduledOutputPin],
    ) -> Result<usize, TransitionApplyError> {
        self.drain_due(now, out);
        apply_drained_transitions(out, injectors, ignition)
    }
}

impl<const N: usize> Default for ScheduledActionExecutor<N> {
    fn default() -> Self {
        Self::new()
    }
}

impl<const N: usize> ActionExecutor for ScheduledActionExecutor<N> {
    type Error = ScheduleError;

    #[allow(deprecated)]
    fn execute(&mut self, action: Action) -> Result<(), Self::Error> {
        match action {
            Action::ArmScheduler {
                injection,
                ignition,
            } => {
                let injection = injection.export_transitions::<2>()?;
                let ignition = ignition.export_transitions::<2>()?;
                if self.queue.free_slots() < injection.len as usize + ignition.len as usize {
                    return Err(ScheduleError::QueueFull);
                }
                self.queue.enqueue_export(&injection)?;
                self.queue.enqueue_export(&ignition)?;
            }
            Action::CancelScheduler(_) => self.queue.cancel_all(),
            Action::PublishSnapshot
            | Action::PersistCalibration
            | Action::ApplyAux(_)
            | Action::SetFan(_)
            | Action::Idle => {}
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use ecu_io::ActionExecutor;
    use ecu_scheduler::{
        ExclusiveChannel, IgnitionPlan as SchedulerIgnitionPlan,
        InjectionPlan as SchedulerInjectionPlan, Micros, OutputGroup, ScheduledTransition,
        ScheduledTransitionQueue, TimedIgnitionPlan, TimedInjectionPlan,
    };

    #[derive(Default, Copy, Clone, PartialEq, Eq)]
    struct RecordingPin {
        high_count: u8,
        low_count: u8,
    }

    impl embedded_hal::digital::v2::OutputPin for RecordingPin {
        type Error = core::convert::Infallible;

        fn set_low(&mut self) -> Result<(), Self::Error> {
            self.low_count = self.low_count.saturating_add(1);
            Ok(())
        }

        fn set_high(&mut self) -> Result<(), Self::Error> {
            self.high_count = self.high_count.saturating_add(1);
            Ok(())
        }
    }

    #[derive(Default)]
    struct RecordingHal1Pin {
        high_count: u8,
        low_count: u8,
    }

    impl embedded_hal_1::digital::ErrorType for RecordingHal1Pin {
        type Error = core::convert::Infallible;
    }

    impl embedded_hal_1::digital::OutputPin for RecordingHal1Pin {
        fn set_low(&mut self) -> Result<(), Self::Error> {
            self.low_count = self.low_count.saturating_add(1);
            Ok(())
        }

        fn set_high(&mut self) -> Result<(), Self::Error> {
            self.high_count = self.high_count.saturating_add(1);
            Ok(())
        }
    }

    fn transition(
        kind: ScheduledTransitionKind,
        channel: u8,
        level: ScheduledLevel,
    ) -> ScheduledTransition {
        ScheduledTransition {
            at_us: Micros::new(100),
            kind,
            channel: ChannelId::new(channel),
            level,
        }
    }

    #[test]
    fn apply_transition_routes_injector_and_ignition_outputs() {
        let mut inj0 = RecordingPin::default();
        let mut inj1 = RecordingPin::default();
        let mut ign0 = RecordingPin::default();
        let mut ign1 = RecordingPin::default();
        let mut injectors: [&mut dyn ScheduledOutputPin; 2] = [&mut inj0, &mut inj1];
        let mut ignition: [&mut dyn ScheduledOutputPin; 2] = [&mut ign0, &mut ign1];

        apply_transition(
            transition(ScheduledTransitionKind::Injector, 1, ScheduledLevel::High),
            &mut injectors,
            &mut ignition,
        )
        .expect("injector transition applies");
        apply_transition(
            transition(ScheduledTransitionKind::Ignition, 0, ScheduledLevel::Low),
            &mut injectors,
            &mut ignition,
        )
        .expect("ignition transition applies");

        assert_eq!(inj0.high_count, 0);
        assert_eq!(inj1.high_count, 1);
        assert_eq!(ign0.low_count, 1);
        assert_eq!(ign1.low_count, 0);
    }

    #[test]
    fn apply_transition_reports_channel_range_errors() {
        let mut inj0 = RecordingPin::default();
        let mut ign0 = RecordingPin::default();
        let mut injectors: [&mut dyn ScheduledOutputPin; 1] = [&mut inj0];
        let mut ignition: [&mut dyn ScheduledOutputPin; 1] = [&mut ign0];

        assert_eq!(
            apply_transition(
                transition(ScheduledTransitionKind::Injector, 1, ScheduledLevel::High),
                &mut injectors,
                &mut ignition,
            ),
            Err(TransitionApplyError::InjectorChannelOutOfRange(
                ChannelId::new(1)
            ))
        );
        assert_eq!(
            apply_transition(
                transition(ScheduledTransitionKind::Ignition, 1, ScheduledLevel::High),
                &mut injectors,
                &mut ignition,
            ),
            Err(TransitionApplyError::IgnitionChannelOutOfRange(
                ChannelId::new(1)
            ))
        );
    }

    #[test]
    fn apply_drained_transitions_applies_valid_len_only() {
        let mut drained = TransitionDrainBuffer::<4> {
            len: 2,
            transitions: [None; 4],
        };
        drained.transitions[0] = Some(transition(
            ScheduledTransitionKind::Injector,
            0,
            ScheduledLevel::High,
        ));
        drained.transitions[1] = Some(transition(
            ScheduledTransitionKind::Injector,
            0,
            ScheduledLevel::Low,
        ));
        drained.transitions[3] = Some(transition(
            ScheduledTransitionKind::Injector,
            0,
            ScheduledLevel::High,
        ));

        let mut inj0 = RecordingPin::default();
        let mut ign0 = RecordingPin::default();
        let mut injectors: [&mut dyn ScheduledOutputPin; 1] = [&mut inj0];
        let mut ignition: [&mut dyn ScheduledOutputPin; 1] = [&mut ign0];

        let applied = apply_drained_transitions(&drained, &mut injectors, &mut ignition)
            .expect("drained transitions apply");
        assert_eq!(applied, 2);
        assert_eq!(inj0.high_count, 1);
        assert_eq!(inj0.low_count, 1);
        assert_eq!(ign0.high_count, 0);
        assert_eq!(ign0.low_count, 0);
    }

    #[test]
    fn apply_drained_transitions_rejects_invalid_batch_without_partial_pin_writes() {
        let mut drained = TransitionDrainBuffer::<4> {
            len: 2,
            transitions: [None; 4],
        };
        drained.transitions[0] = Some(transition(
            ScheduledTransitionKind::Injector,
            0,
            ScheduledLevel::High,
        ));
        drained.transitions[1] = Some(transition(
            ScheduledTransitionKind::Ignition,
            1,
            ScheduledLevel::High,
        ));

        let mut inj0 = RecordingPin::default();
        let mut ign0 = RecordingPin::default();
        let mut injectors: [&mut dyn ScheduledOutputPin; 1] = [&mut inj0];
        let mut ignition: [&mut dyn ScheduledOutputPin; 1] = [&mut ign0];

        assert_eq!(
            apply_drained_transitions(&drained, &mut injectors, &mut ignition),
            Err(TransitionApplyError::IgnitionChannelOutOfRange(
                ChannelId::new(1)
            ))
        );
        assert_eq!(inj0.high_count, 0);
        assert_eq!(inj0.low_count, 0);
        assert_eq!(ign0.high_count, 0);
        assert_eq!(ign0.low_count, 0);
    }

    #[test]
    fn scheduler_queue_drain_applies_to_target_pins() {
        let mut queue = ScheduledTransitionQueue::<4>::new();
        queue
            .enqueue_transition(transition(
                ScheduledTransitionKind::Injector,
                0,
                ScheduledLevel::High,
            ))
            .expect("injector open fits");
        queue
            .enqueue_transition(transition(
                ScheduledTransitionKind::Injector,
                0,
                ScheduledLevel::Low,
            ))
            .expect("injector close fits");

        let mut drained = TransitionDrainBuffer::<4>::new();
        assert_eq!(queue.drain_due(Micros::new(200), &mut drained), 2);

        let mut inj0 = RecordingPin::default();
        let mut ign0 = RecordingPin::default();
        let mut injectors: [&mut dyn ScheduledOutputPin; 1] = [&mut inj0];
        let mut ignition: [&mut dyn ScheduledOutputPin; 1] = [&mut ign0];

        assert_eq!(
            apply_drained_transitions(&drained, &mut injectors, &mut ignition),
            Ok(2)
        );
        assert_eq!(inj0.high_count, 1);
        assert_eq!(inj0.low_count, 1);
        assert_eq!(ign0.high_count, 0);
        assert_eq!(ign0.low_count, 0);
    }

    fn timed_injection(start_at: u32, end_at: u32) -> TimedInjectionPlan {
        TimedInjectionPlan {
            plan: SchedulerInjectionPlan {
                output: ExclusiveChannel::new(OutputGroup::Injector, ChannelId::new(0)),
                pulse_width: ecu_scheduler::PulseWidthUs::new((end_at - start_at) as u16),
            },
            start_at: Micros::new(start_at),
            end_at: Micros::new(end_at),
        }
    }

    fn timed_ignition(start_at: u32, end_at: u32) -> TimedIgnitionPlan {
        TimedIgnitionPlan {
            plan: SchedulerIgnitionPlan {
                output: ExclusiveChannel::new(OutputGroup::Ignition, ChannelId::new(0)),
                dwell: ecu_scheduler::DwellUs::new((end_at - start_at) as u16),
                advance: ecu_scheduler::Degrees10::new(100),
            },
            start_at: Micros::new(start_at),
            end_at: Micros::new(end_at),
        }
    }

    #[test]
    fn scheduled_action_executor_enqueues_runtime_scheduler_action() {
        let mut executor = ScheduledActionExecutor::<4>::new();
        executor
            .execute(Action::ArmScheduler {
                injection: timed_injection(100, 120),
                ignition: timed_ignition(200, 230),
            })
            .expect("scheduler action queues");

        assert_eq!(executor.queue().active_count(), 4);

        let mut drained = TransitionDrainBuffer::<4>::new();
        assert_eq!(executor.drain_due(Micros::new(300), &mut drained), 4);
        assert_eq!(
            drained.transitions[0].expect("injector open").kind,
            ScheduledTransitionKind::Injector
        );
        assert_eq!(
            drained.transitions[2].expect("coil charge").kind,
            ScheduledTransitionKind::Ignition
        );
    }

    #[test]
    fn scheduled_action_executor_cancel_clears_queued_transitions() {
        let mut executor = ScheduledActionExecutor::<4>::new();
        executor
            .execute(Action::ArmScheduler {
                injection: timed_injection(100, 120),
                ignition: timed_ignition(200, 230),
            })
            .expect("scheduler action queues");

        executor
            .execute(Action::CancelScheduler(ecu_domain::CancelReason::SyncLoss))
            .expect("cancel clears queue");

        assert_eq!(executor.queue().active_count(), 0);
    }

    #[test]
    fn scheduled_action_executor_ignores_apply_aux() {
        let mut executor = ScheduledActionExecutor::<4>::new();

        executor
            .execute(Action::ApplyAux(Default::default()))
            .expect("aux action is ignored");

        assert_eq!(executor.queue().active_count(), 0);
    }

    #[test]
    fn scheduled_action_executor_rejects_atomic_overflow() {
        let mut executor = ScheduledActionExecutor::<3>::new();
        assert_eq!(
            executor.execute(Action::ArmScheduler {
                injection: timed_injection(100, 120),
                ignition: timed_ignition(200, 230),
            }),
            Err(ScheduleError::QueueFull)
        );
        assert_eq!(
            executor.queue().active_count(),
            0,
            "overflow must not partially enqueue"
        );
    }

    #[test]
    fn scheduled_action_executor_drain_and_apply_due_is_board_tick_ready() {
        let mut executor = ScheduledActionExecutor::<4>::new();
        executor
            .execute(Action::ArmScheduler {
                injection: timed_injection(100, 120),
                ignition: timed_ignition(200, 230),
            })
            .expect("scheduler action queues");

        let mut inj0 = RecordingPin::default();
        let mut ign0 = RecordingPin::default();
        let mut injectors: [&mut dyn ScheduledOutputPin; 1] = [&mut inj0];
        let mut ignition: [&mut dyn ScheduledOutputPin; 1] = [&mut ign0];
        let mut drained = TransitionDrainBuffer::<4>::new();

        assert_eq!(
            executor.drain_and_apply_due(
                Micros::new(1_000),
                &mut drained,
                &mut injectors,
                &mut ignition
            ),
            Ok(4)
        );
        assert_eq!(executor.queue().active_count(), 0);
        assert_eq!(inj0.high_count, 1);
        assert_eq!(inj0.low_count, 1);
        assert_eq!(ign0.high_count, 1);
        assert_eq!(ign0.low_count, 1);
    }

    #[test]
    fn scheduled_outputs4_drains_and_applies_due_transitions() {
        let mut executor = ScheduledActionExecutor::<4>::new();
        executor
            .execute(Action::ArmScheduler {
                injection: timed_injection(100, 120),
                ignition: timed_ignition(200, 230),
            })
            .expect("scheduler action queues");

        let inj0 = RecordingPin::default();
        let inj1 = RecordingPin::default();
        let ign0 = RecordingPin::default();
        let ign1 = RecordingPin::default();
        let mut outputs = ScheduledOutputs4::new(inj0, inj1, ign0, ign1);
        let mut drained = TransitionDrainBuffer::<4>::new();

        assert_eq!(
            outputs.drain_and_apply_due(&mut executor, Micros::new(1_000), &mut drained),
            Ok(4)
        );

        let (inj0, inj1, ign0, ign1) = outputs.into_inner();
        assert_eq!(inj0.high_count, 1);
        assert_eq!(inj0.low_count, 1);
        assert_eq!(inj1.high_count, 0);
        assert_eq!(inj1.low_count, 0);
        assert_eq!(ign0.high_count, 1);
        assert_eq!(ign0.low_count, 1);
        assert_eq!(ign1.high_count, 0);
        assert_eq!(ign1.low_count, 0);
    }

    #[test]
    fn scheduled_outputs4_accepts_embedded_hal_1_wrapped_pins() {
        let mut executor = ScheduledActionExecutor::<4>::new();
        executor
            .execute(Action::ArmScheduler {
                injection: timed_injection(100, 120),
                ignition: timed_ignition(200, 230),
            })
            .expect("scheduler action queues");

        let inj0 = Hal1ScheduledOut::new(RecordingHal1Pin::default());
        let inj1 = Hal1ScheduledOut::new(RecordingHal1Pin::default());
        let ign0 = Hal1ScheduledOut::new(RecordingHal1Pin::default());
        let ign1 = Hal1ScheduledOut::new(RecordingHal1Pin::default());
        let mut outputs = ScheduledOutputs4::new(inj0, inj1, ign0, ign1);
        let mut drained = TransitionDrainBuffer::<4>::new();

        assert_eq!(
            outputs.drain_and_apply_due(&mut executor, Micros::new(1_000), &mut drained),
            Ok(4)
        );

        let (inj0, inj1, ign0, ign1) = outputs.into_inner();
        let inj0 = inj0.into_inner();
        let inj1 = inj1.into_inner();
        let ign0 = ign0.into_inner();
        let ign1 = ign1.into_inner();
        assert_eq!(inj0.high_count, 1);
        assert_eq!(inj0.low_count, 1);
        assert_eq!(inj1.high_count, 0);
        assert_eq!(inj1.low_count, 0);
        assert_eq!(ign0.high_count, 1);
        assert_eq!(ign0.low_count, 1);
        assert_eq!(ign1.high_count, 0);
        assert_eq!(ign1.low_count, 0);
    }

    #[test]
    fn scheduled_outputs_generic_supports_six_injectors_and_three_ignition_channels() {
        let mut executor = ScheduledActionExecutor::<4>::new();
        executor
            .queue_mut()
            .enqueue_transition(ScheduledTransition {
                at_us: Micros::new(100),
                kind: ScheduledTransitionKind::Injector,
                channel: ecu_domain::ChannelId::new(5),
                level: ScheduledLevel::High,
            })
            .expect("injector transition fits");
        executor
            .queue_mut()
            .enqueue_transition(ScheduledTransition {
                at_us: Micros::new(100),
                kind: ScheduledTransitionKind::Ignition,
                channel: ecu_domain::ChannelId::new(2),
                level: ScheduledLevel::Low,
            })
            .expect("ignition transition fits");

        let mut outputs = ScheduledOutputs::<6, 3, RecordingPin, RecordingPin>::new(
            [RecordingPin::default(); 6],
            [RecordingPin::default(); 3],
        );
        let mut drained = TransitionDrainBuffer::<4>::new();

        assert_eq!(
            outputs.drain_and_apply_due(&mut executor, Micros::new(200), &mut drained),
            Ok(2)
        );

        let (injectors, ignition) = outputs.into_inner();
        assert_eq!(injectors[5].high_count, 1);
        assert_eq!(injectors[5].low_count, 0);
        assert_eq!(ignition[2].high_count, 0);
        assert_eq!(ignition[2].low_count, 1);
    }
}
