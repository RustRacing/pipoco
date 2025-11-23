#![no_std]
#![no_main]

// PIO-based trigger capture example for RP2350B (feature-gated)
// Build with: --features pio-capture --bin rp2350-pio-capture-example

use cortex_m_rt::entry;
use panic_halt as _;

#[entry]
fn main() -> ! {
    // Outline:
    // 1) Configure clocks and peripherals
    // 2) Load a small PIO program to sample a GPIO and push timestamp on rising edges
    // 3) Use a timer (TIMER0) as microsecond counter; read in IRQ handler and push to buffer
    // 4) In main loop, pop timestamps and feed ecu_core::TriggerDecoder::tooth_edge_with_timestamp(ts)

    // This is a skeleton to illustrate structure; fill in with rp235x-hal PIO setup on your board.
    loop {
        cortex_m::asm::wfi();
    }
}
