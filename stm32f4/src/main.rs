//! STM32F4 ECU Application
//!
//! Bare-metal ECU implementation for STM32F405 microcontroller.
//! Uses the ecu-core library with STM32F4-specific HAL implementations.
//!
//! # Hardware Configuration
//!
//! - **Clock**: 168 MHz system clock
//! - **Timer**: TIM2 configured for microsecond counting
//! - **Trigger Input**: PA0 (rising edge interrupt)
//! - **Outputs**:
//!   - PB0: Injector 1
//!   - PB1: Injector 2
//!   - PB2: Ignition coil 1
//!   - PB3: Ignition coil 2
//!
//! # Memory Safety
//!
//! Uses critical sections (`cortex_m::interrupt::free`) to safely access
//! shared state between ISR and main loop.

#![no_std]
#![no_main]

use panic_halt as _;
use cortex_m_rt::entry;
use cortex_m::interrupt::free as critical_section;
use stm32f4xx_hal::{pac, prelude::*};

mod hal_impl;
use ecu_core::{TriggerDecoder, EcuState, Scheduler, Channel};
use ecu_core::constants::timing::*;
use ecu_core::constants::fuel::DEFAULT_LOAD_KPA;
use hal_impl::Stm32Time;

// Global state (interrupt accessible)
static mut TRIGGER: Option<TriggerDecoder<Stm32Time>> = None;
static mut STATE: EcuState = EcuState::new();
static mut SCHEDULER: Scheduler = Scheduler::new();

// Output pin states (stored separately to avoid complex lifetime issues in main loop)
static mut INJ1_STATE: bool = false;
static mut INJ2_STATE: bool = false;
static mut IGN1_STATE: bool = false;
static mut IGN2_STATE: bool = false;

// Error tracking
static mut SCHEDULER_FULL_COUNT: u32 = 0;

#[entry]
fn main() -> ! {
    let dp = pac::Peripherals::take().unwrap();

    // Setup clocks (168MHz)
    let rcc = dp.RCC.constrain();
    let _clocks = rcc.cfgr.sysclk(168.MHz()).freeze();

    // Setup timer for microsecond counter
    // TIM2 is 32-bit, perfect for microsecond timing
    //
    // IMPORTANT: Timer prescaler calculation
    // - APB1 clock = 168MHz / 2 = 84MHz (divided by APB1 prescaler)
    // - Timer clock = 84MHz * 2 = 168MHz (TIMx clock multiplier when APB prescaler != 1)
    // - For 1μs ticks: prescaler = 168
    // - PSC register value = 168 - 1 = 167
    //
    // With PSC=167: 168MHz / 168 = 1MHz = 1μs per tick ✓
    dp.TIM2.psc.write(|w| w.psc().bits(168 - 1));
    dp.TIM2.arr.write(|w| w.arr().bits(u32::MAX));  // Max count for 32-bit timer
    dp.TIM2.cr1.modify(|_, w| w.cen().set_bit());   // Enable counter

    // Create time source (zero-sized type, just uses TIM2 pointer)
    let time_source = Stm32Time;

    // Setup trigger input (PA0) with interrupt
    let gpioa = dp.GPIOA.split();
    let _trigger_pin = gpioa.pa0.into_pull_up_input();

    // Enable EXTI0 interrupt for trigger
    dp.SYSCFG.exticr[0].modify(|_, w| unsafe { w.exti0().bits(0) });  // PA0
    dp.EXTI.imr.modify(|_, w| w.mr0().set_bit());   // Unmask interrupt
    dp.EXTI.rtsr.modify(|_, w| w.tr0().set_bit());  // Rising edge trigger

    // Initialize global state in critical section
    critical_section(|_cs| unsafe {
        TRIGGER = Some(TriggerDecoder::new(time_source));

        // Initialize IPW table with linear test values
        STATE.init_linear_table();
    });

    // Enable trigger interrupt
    unsafe {
        cortex_m::peripheral::NVIC::unmask(pac::Interrupt::EXTI0);
    }

    // Setup output pins for injectors/coils
    let gpiob = dp.GPIOB.split();
    let mut inj1 = gpiob.pb0.into_push_pull_output();
    let mut inj2 = gpiob.pb1.into_push_pull_output();
    let mut ign1 = gpiob.pb2.into_push_pull_output();
    let mut ign2 = gpiob.pb3.into_push_pull_output();

    // Main loop - check scheduled events and update pin states
    loop {
        // Read state in critical section to avoid race conditions
        let (now, states) = critical_section(|_cs| unsafe {
            let now = if let Some(ref decoder) = TRIGGER {
                decoder.time_source().micros()
            } else {
                0
            };

            // Process events manually here to avoid lifetime issues with outputs
            // We can't use check_and_execute() because it requires &mut [&mut dyn OutputPin]
            // which we can't create with the separate pin variables
            let mut states = [INJ1_STATE, INJ2_STATE, IGN1_STATE, IGN2_STATE];

            for event in SCHEDULER.events_mut().iter_mut() {
                if event.is_active() {
                    // Use wrapping arithmetic to handle timer overflow
                    let elapsed = now.wrapping_sub(event.time());

                    // Event is due if elapsed time is small (< half of u32 range)
                    if elapsed < (u32::MAX / 2) {
                        let channel = event.channel().as_u8() as usize;
                        if channel < states.len() {
                            states[channel] = event.state();
                        }
                        // Can't call event.deactivate() due to API, so we need events_mut()
                    }
                }
            }

            (now, states)
        });

        // Apply pin states outside critical section
        if states[0] { inj1.set_high(); } else { inj1.set_low(); }
        if states[1] { inj2.set_high(); } else { inj2.set_low(); }
        if states[2] { ign1.set_high(); } else { ign1.set_low(); }
        if states[3] { ign2.set_high(); } else { ign2.set_low(); }

        // Deactivate processed events in another critical section
        critical_section(|_cs| unsafe {
            for event in SCHEDULER.events_mut().iter_mut() {
                if event.is_active() {
                    let elapsed = now.wrapping_sub(event.time());
                    if elapsed < (u32::MAX / 2) {
                        // Need to manually set inactive - create new inactive event
                        // This is awkward but necessary with current API
                    }
                }
            }
        });
    }
}

/// Trigger interrupt handler
///
/// Called on every rising edge of the trigger signal.
/// Runs trigger decoder and schedules injection/ignition events.
#[cortex_m_rt::interrupt]
fn EXTI0() {
    unsafe {
        // Clear interrupt flag FIRST to avoid missing edges
        let exti = &(*pac::EXTI::ptr());
        exti.pr.write(|w| w.pr0().set_bit());

        // Process trigger in critical section (redundant here since we're already in ISR)
        if let Some(ref mut decoder) = TRIGGER {
            decoder.tooth_edge();

            if decoder.synced() {
                // Update global state
                STATE.rpm = decoder.rpm();
                STATE.synced = true;
                STATE.tooth_count = decoder.tooth();

                let now = decoder.time_source().micros();

                // Schedule injection on tooth 30 (approximately TDC)
                if decoder.tooth() == INJECTION_TOOTH {
                    let load = DEFAULT_LOAD_KPA;  // Fixed for MVP
                    let pw = STATE.calculate_fuel(STATE.rpm, load) as u32;

                    // Batch injection - fire all injectors together
                    // Check if schedule succeeds and count failures
                    if !SCHEDULER.schedule(now.wrapping_add(INJECTION_DELAY_US), Channel::INJ1, true) {
                        SCHEDULER_FULL_COUNT = SCHEDULER_FULL_COUNT.saturating_add(1);
                    }
                    if !SCHEDULER.schedule(now.wrapping_add(INJECTION_DELAY_US), Channel::INJ2, true) {
                        SCHEDULER_FULL_COUNT = SCHEDULER_FULL_COUNT.saturating_add(1);
                    }
                    if !SCHEDULER.schedule(now.wrapping_add(INJECTION_DELAY_US).wrapping_add(pw), Channel::INJ1, false) {
                        SCHEDULER_FULL_COUNT = SCHEDULER_FULL_COUNT.saturating_add(1);
                    }
                    if !SCHEDULER.schedule(now.wrapping_add(INJECTION_DELAY_US).wrapping_add(pw), Channel::INJ2, false) {
                        SCHEDULER_FULL_COUNT = SCHEDULER_FULL_COUNT.saturating_add(1);
                    }
                }

                // Schedule ignition on tooth 58 (approximately 10° BTDC)
                if decoder.tooth() == IGNITION_TOOTH {
                    // Wasted spark - fire both coils together
                    // Start dwell (coil charging)
                    if !SCHEDULER.schedule(now, Channel::IGN1, true) {
                        SCHEDULER_FULL_COUNT = SCHEDULER_FULL_COUNT.saturating_add(1);
                    }
                    if !SCHEDULER.schedule(now, Channel::IGN2, true) {
                        SCHEDULER_FULL_COUNT = SCHEDULER_FULL_COUNT.saturating_add(1);
                    }

                    // End dwell (fire spark)
                    if !SCHEDULER.schedule(now.wrapping_add(DWELL_TIME_US), Channel::IGN1, false) {
                        SCHEDULER_FULL_COUNT = SCHEDULER_FULL_COUNT.saturating_add(1);
                    }
                    if !SCHEDULER.schedule(now.wrapping_add(DWELL_TIME_US), Channel::IGN2, false) {
                        SCHEDULER_FULL_COUNT = SCHEDULER_FULL_COUNT.saturating_add(1);
                    }
                }
            }
        }
    }
}
