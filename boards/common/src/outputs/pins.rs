use ecu_domain::ChannelId;
use ecu_scheduler::{
    ScheduledLevel, ScheduledTransition, ScheduledTransitionKind, TransitionDrainBuffer,
};

use super::scheduler::ScheduledActionExecutor;

/// Raw board output banks that can receive drained split-scheduler transitions
/// without heap allocation.
///
/// This is the HAL/pin-bank side of the split scheduler path, not the logical
/// `ecu-board-api::OutputScheduler` contract.
pub trait RawScheduledOutputBank {
    fn drain_and_apply_due<const Q: usize, const D: usize>(
        &mut self,
        executor: &mut ScheduledActionExecutor<Q>,
        now: ecu_scheduler::Micros,
        drained: &mut TransitionDrainBuffer<D>,
    ) -> Result<usize, TransitionApplyError>;
}

/// Wrapper for embedded-hal v0.2 output pins on split scheduler paths.
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

impl<P> RawScheduledOutputPin for HalOut<P>
where
    P: embedded_hal::digital::v2::OutputPin,
{
    fn set_scheduled_high(&mut self) {
        let _ = self.pin.set_high();
    }

    fn set_scheduled_low(&mut self) {
        let _ = self.pin.set_low();
    }

    fn try_set_scheduled_high(&mut self) -> Result<(), ScheduledOutputPinError> {
        self.pin
            .set_high()
            .map_err(|_| ScheduledOutputPinError::SetHigh)
    }

    fn try_set_scheduled_low(&mut self) -> Result<(), ScheduledOutputPinError> {
        self.pin
            .set_low()
            .map_err(|_| ScheduledOutputPinError::SetLow)
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
    I: RawScheduledOutputPin,
    G: RawScheduledOutputPin,
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

    fn apply_transition(
        &mut self,
        transition: ScheduledTransition,
    ) -> Result<(), TransitionApplyError> {
        match transition.kind {
            ScheduledTransitionKind::Injector => {
                let pin = &mut self.injectors[transition.channel.get() as usize];
                match transition.level {
                    ScheduledLevel::High => pin.try_set_scheduled_high(),
                    ScheduledLevel::Low => pin.try_set_scheduled_low(),
                }
                .map_err(|error| TransitionApplyError::PinWrite {
                    kind: transition.kind,
                    channel: transition.channel,
                    level: transition.level,
                    error,
                })?;
            }
            ScheduledTransitionKind::Ignition => {
                let pin = &mut self.ignition[transition.channel.get() as usize];
                match transition.level {
                    ScheduledLevel::High => pin.try_set_scheduled_high(),
                    ScheduledLevel::Low => pin.try_set_scheduled_low(),
                }
                .map_err(|error| TransitionApplyError::PinWrite {
                    kind: transition.kind,
                    channel: transition.channel,
                    level: transition.level,
                    error,
                })?;
            }
        }
        Ok(())
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
                self.apply_transition(transition)?;
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

impl<const INJ: usize, const IGN: usize, I, G> RawScheduledOutputBank
    for ScheduledOutputs<INJ, IGN, I, G>
where
    I: RawScheduledOutputPin,
    G: RawScheduledOutputPin,
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
    I1: RawScheduledOutputPin,
    I2: RawScheduledOutputPin,
    G1: RawScheduledOutputPin,
    G2: RawScheduledOutputPin,
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
        [&mut dyn RawScheduledOutputPin; 2],
        [&mut dyn RawScheduledOutputPin; 2],
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

impl<I1, I2, G1, G2> RawScheduledOutputBank for ScheduledOutputs4<I1, I2, G1, G2>
where
    I1: RawScheduledOutputPin,
    I2: RawScheduledOutputPin,
    G1: RawScheduledOutputPin,
    G2: RawScheduledOutputPin,
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

/// Four-output wrapper for embedded-hal v0.2 pins on split scheduler paths.
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

    pub fn as_scheduled_pins(
        &mut self,
    ) -> (
        [&mut dyn RawScheduledOutputPin; 2],
        [&mut dyn RawScheduledOutputPin; 2],
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

impl<I1, I2, G1, G2> RawScheduledOutputBank for Outputs4<I1, I2, G1, G2>
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

/// Minimal raw HAL pin trait for applying split scheduler transitions.
///
/// This deliberately stays independent from the root core HAL so the split
/// scheduler/target path can migrate on the board boundary. It represents the
/// electrical pin operation, not a logical ECU output scheduler.
pub trait RawScheduledOutputPin {
    fn set_scheduled_high(&mut self);
    fn set_scheduled_low(&mut self);

    fn try_set_scheduled_high(&mut self) -> Result<(), ScheduledOutputPinError> {
        self.set_scheduled_high();
        Ok(())
    }

    fn try_set_scheduled_low(&mut self) -> Result<(), ScheduledOutputPinError> {
        self.set_scheduled_low();
        Ok(())
    }
}

impl<P> RawScheduledOutputPin for P
where
    P: embedded_hal::digital::v2::OutputPin,
{
    fn set_scheduled_high(&mut self) {
        let _ = self.set_high();
    }

    fn set_scheduled_low(&mut self) {
        let _ = self.set_low();
    }

    fn try_set_scheduled_high(&mut self) -> Result<(), ScheduledOutputPinError> {
        self.set_high()
            .map_err(|_| ScheduledOutputPinError::SetHigh)
    }

    fn try_set_scheduled_low(&mut self) -> Result<(), ScheduledOutputPinError> {
        self.set_low().map_err(|_| ScheduledOutputPinError::SetLow)
    }
}

/// Wrapper for embedded-hal 1.0 output pins on split scheduler paths.
///
/// `RawScheduledOutputPin` already has a blanket implementation for
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

impl<P> RawScheduledOutputPin for Hal1ScheduledOut<P>
where
    P: embedded_hal_1::digital::OutputPin,
{
    fn set_scheduled_high(&mut self) {
        let _ = self.pin.set_high();
    }

    fn set_scheduled_low(&mut self) {
        let _ = self.pin.set_low();
    }

    fn try_set_scheduled_high(&mut self) -> Result<(), ScheduledOutputPinError> {
        self.pin
            .set_high()
            .map_err(|_| ScheduledOutputPinError::SetHigh)
    }

    fn try_set_scheduled_low(&mut self) -> Result<(), ScheduledOutputPinError> {
        self.pin
            .set_low()
            .map_err(|_| ScheduledOutputPinError::SetLow)
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ScheduledOutputPinError {
    SetHigh,
    SetLow,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TransitionApplyError {
    InjectorChannelOutOfRange(ChannelId),
    IgnitionChannelOutOfRange(ChannelId),
    PinWrite {
        kind: ScheduledTransitionKind,
        channel: ChannelId,
        level: ScheduledLevel,
        error: ScheduledOutputPinError,
    },
}

pub fn apply_transition(
    transition: ScheduledTransition,
    injectors: &mut [&mut dyn RawScheduledOutputPin],
    ignition: &mut [&mut dyn RawScheduledOutputPin],
) -> Result<(), TransitionApplyError> {
    match transition.kind {
        ScheduledTransitionKind::Injector => {
            let pin = injectors.get_mut(transition.channel.get() as usize).ok_or(
                TransitionApplyError::InjectorChannelOutOfRange(transition.channel),
            )?;
            match transition.level {
                ScheduledLevel::High => pin.try_set_scheduled_high(),
                ScheduledLevel::Low => pin.try_set_scheduled_low(),
            }
            .map_err(|error| TransitionApplyError::PinWrite {
                kind: transition.kind,
                channel: transition.channel,
                level: transition.level,
                error,
            })?;
        }
        ScheduledTransitionKind::Ignition => {
            let pin = ignition.get_mut(transition.channel.get() as usize).ok_or(
                TransitionApplyError::IgnitionChannelOutOfRange(transition.channel),
            )?;
            match transition.level {
                ScheduledLevel::High => pin.try_set_scheduled_high(),
                ScheduledLevel::Low => pin.try_set_scheduled_low(),
            }
            .map_err(|error| TransitionApplyError::PinWrite {
                kind: transition.kind,
                channel: transition.channel,
                level: transition.level,
                error,
            })?;
        }
    }
    Ok(())
}

pub fn apply_drained_transitions<const N: usize>(
    drained: &TransitionDrainBuffer<N>,
    injectors: &mut [&mut dyn RawScheduledOutputPin],
    ignition: &mut [&mut dyn RawScheduledOutputPin],
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
