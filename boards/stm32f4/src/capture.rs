//! Capture-path helpers for STM32F4 trigger timestamping.

use stm32f4xx_hal::{gpio::gpioa, pac};

#[cfg(all(feature = "capture-gpio", not(feature = "capture-tim")))]
use stm32f4xx_hal::{
    gpio::{Edge, ExtiPin},
    prelude::*,
};

#[cfg(all(feature = "capture-tim", feature = "capture-gpio"))]
compile_error!("features `capture-tim` and `capture-gpio` are mutually exclusive");

ecu_target_common::capture_ring!(CAPTURE, 128);

#[cfg(all(feature = "capture-tim", not(feature = "capture-gpio")))]
pub fn configure_capture_inputs(trigger_pin: gpioa::PA0, tim2: &mut pac::TIM2) {
    let _trigger_pin = trigger_pin.into_alternate::<1>();
    tim2.ccmr1_input().modify(|_, w| w.cc1s().ti1());
    tim2.ccer.modify(|_, w| {
        w.cc1p().clear_bit();
        w.cc1e().set_bit()
    });
    tim2.dier.modify(|_, w| w.cc1ie().set_bit());
    unsafe {
        cortex_m::peripheral::NVIC::unmask(pac::Interrupt::TIM2);
    }
}

#[cfg(all(feature = "capture-gpio", not(feature = "capture-tim")))]
pub fn configure_capture_inputs(
    trigger_pin: gpioa::PA0,
    syscfg: pac::SYSCFG,
    exti: &mut pac::EXTI,
) {
    let mut trigger_pin = trigger_pin.into_pull_up_input();
    let mut syscfg = syscfg.constrain();
    trigger_pin.make_interrupt_source(&mut syscfg);
    trigger_pin.trigger_on_edge(exti, Edge::Rising);
    trigger_pin.enable_interrupt(exti);
    unsafe {
        cortex_m::peripheral::NVIC::unmask(pac::Interrupt::EXTI0);
    }
}

pub fn pop() -> Option<u32> {
    capture_pop()
}

#[cfg(all(feature = "capture-tim", not(feature = "capture-gpio")))]
pub fn handle_tim2_interrupt() {
    unsafe {
        let tim2 = &(*pac::TIM2::ptr());
        if tim2.sr.read().cc1if().bit_is_set() {
            let captured = tim2.ccr1().read().ccr().bits();
            tim2.sr.modify(|_, w| w.cc1if().clear());
            capture_push(captured);
        }
    }
}

#[cfg(all(feature = "capture-gpio", not(feature = "capture-tim")))]
pub fn handle_exti0_interrupt() {
    unsafe {
        let exti = &(*pac::EXTI::ptr());
        exti.pr.write(|w| w.pr0().set_bit());
        capture_push(stm32_capture_now());
    }
}

#[cfg(all(feature = "capture-gpio", not(feature = "capture-tim")))]
fn stm32_capture_now() -> u32 {
    // Safety: TIM2 configured during bring-up for one-microsecond tick tracking.
    unsafe { (*pac::TIM2::ptr()).cnt.read().bits() }
}
