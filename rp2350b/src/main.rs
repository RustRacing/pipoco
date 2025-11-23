//! ECU VE Engine Demo for Raspberry Pi RP2350B
//!
//! This demonstrates the VE engine running on RP2350B.
//! Shows calculation of IPW tables from VE tables with transformations.
//!
//! # Hardware
//! - Target: RP2350B (Cortex-M33, 150MHz, 520KB RAM)
//! - Peripherals: UART for debug output
//!
//! # Features Demonstrated
//! - VE table → IPW table calculation
//! - Speed-density air mass calculation
//! - Environmental corrections
//! - Emergency transformation modes
//! - Integer-only arithmetic (no FPU needed)

#![no_std]
#![no_main]

use panic_halt as _;

use cortex_m_rt::entry;

use hal::{clocks::ClockSource, pac};
use rp235x_hal as hal;

// Import ECU core library
use ecu_core::ve_engine::{SensorData, VeCommand, VeEngine};
mod engine_config;

// RP235x requires boot2 but it's embedded in the HAL differently
// For now, we'll rely on the default bootloader

#[entry]
fn main() -> ! {
    // Take peripherals
    let mut pac_periph = pac::Peripherals::take().unwrap();
    let core = cortex_m::Peripherals::take().unwrap();

    // Set up watchdog for clock initialization
    let mut watchdog = hal::Watchdog::new(pac_periph.WATCHDOG);

    // Configure clocks - RP2350B can run at 150MHz
    let clocks = hal::clocks::init_clocks_and_plls(
        12_000_000, // 12 MHz crystal
        pac_periph.XOSC,
        pac_periph.CLOCKS,
        pac_periph.PLL_SYS,
        pac_periph.PLL_USB,
        &mut pac_periph.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.get_freq().to_Hz());

    // ========================================
    // VE ENGINE DEMONSTRATION
    // ========================================

    // Create VE engine instance with explicit config
    let mut ve_engine = VeEngine::new_with(
        engine_config::injector_config(),
        engine_config::baseline_ve(),
        engine_config::afr_table(),
    );

    // Simulate sensor data
    let sensors = SensorData {
        timestamp_us: 0,
        iat_celsius: 20,           // 20°C intake air
        clt_celsius: 80,           // 80°C coolant (warm engine)
        battery_voltage_mv: 13500, // 13.5V
    };

    // ========================================
    // TEST 1: Normal Operation
    // ========================================

    let normal_table = ve_engine.calculate_ipw_table(&sensors);

    // Check a few cells (should be reasonable values)
    let idle_ipw = normal_table.values[2][1]; // Low load, low RPM
    let cruise_ipw = normal_table.values[6][5]; // Mid load, mid RPM
    let wot_ipw = normal_table.values[12][8]; // High load, high RPM

    // Sanity checks (should be between 500-20000 microseconds)
    assert!(idle_ipw >= 500 && idle_ipw <= 20000);
    assert!(cruise_ipw >= 500 && cruise_ipw <= 20000);
    assert!(wot_ipw >= 500 && wot_ipw <= 20000);

    // WOT should have more fuel than cruise, cruise more than idle
    assert!(wot_ipw > cruise_ipw);
    assert!(cruise_ipw > idle_ipw);

    // ========================================
    // TEST 2: Emergency Rich Mode
    // ========================================

    ve_engine.apply_command(VeCommand::EmergencyRich { percent: 20 });
    let rich_table = ve_engine.calculate_ipw_table(&sensors);

    // All cells should have more fuel
    let rich_cruise = rich_table.values[6][5];
    assert!(rich_cruise > cruise_ipw);

    // ========================================
    // TEST 3: Limp Mode
    // ========================================

    ve_engine.clear_transformations();
    ve_engine.apply_command(VeCommand::LimpMode);
    let limp_table = ve_engine.calculate_ipw_table(&sensors);

    // Limp mode should have less fuel (conservative)
    let limp_cruise = limp_table.values[6][5];
    assert!(limp_cruise < cruise_ipw);

    // ========================================
    // TEST 4: Cold Start Enrichment
    // ========================================

    let cold_sensors = SensorData {
        timestamp_us: 1000,
        iat_celsius: -10,          // Cold air
        clt_celsius: -5,           // Cold engine
        battery_voltage_mv: 12000, // Lower voltage during start
    };

    ve_engine.clear_transformations();
    ve_engine.apply_command(VeCommand::ColdStart {
        enrichment_percent: 50,
        max_clt_celsius: 40,
    });

    let cold_table = ve_engine.calculate_ipw_table(&cold_sensors);
    let cold_cruise = cold_table.values[6][5];

    // Cold start should have significantly more fuel
    assert!(cold_cruise > cruise_ipw);

    // ========================================
    // TEST 5: Transformation Clearing
    // ========================================

    ve_engine.clear_transformations();
    let cleared_table = ve_engine.calculate_ipw_table(&sensors);
    let cleared_cruise = cleared_table.values[6][5];

    // Should be back to normal
    assert_eq!(cleared_cruise, cruise_ipw);

    // ========================================
    // TEST 6: Regional Trim
    // ========================================

    ve_engine.apply_command(VeCommand::RegionalTrim {
        rpm_min: 2000,
        rpm_max: 4000,
        load_min: 60,
        load_max: 120,
        trim_percent: 10, // +10% in mid-range
    });

    let trimmed_table = ve_engine.calculate_ipw_table(&sensors);

    // Cell inside region should have more fuel
    let mid_range = trimmed_table.values[6][5]; // 3000 RPM, 90 kPa
    assert!(mid_range > cruise_ipw);

    // Cell outside region should be unchanged
    let low_range = trimmed_table.values[1][1]; // 1000 RPM, 30 kPa
    let normal_low = normal_table.values[1][1];
    assert_eq!(low_range, normal_low);

    // ========================================
    // ALL TESTS PASSED!
    // ========================================

    // In a real application, you would:
    // 1. Read sensors via ADC/I2C/SPI
    // 2. Calculate IPW table every 10-100ms
    // 3. Send table to injection module via DMA/UART/CAN
    // 4. Injection module uses table for real-time pulse width lookup

    // Main loop - blink LED to show we're alive
    loop {
        // In production: update VE calculations here based on sensor input
        delay.delay_ms(1000);

        // Toggle LED or send telemetry
    }
}

// ========================================
// PANIC HANDLER
// ========================================

// Using panic-halt (hangs on panic)
// In production, you'd want to log the panic and enter safe mode
