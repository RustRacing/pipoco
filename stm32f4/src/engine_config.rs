//! Engine-specific configuration for STM32F4 target
//! 4-cylinder, 2.0L, 440cc/min injectors @ 3 bar

use ecu_core::ve_engine::types::{InjectorConfig, VeTable, AfrTable};

pub fn injector_config() -> InjectorConfig {
    InjectorConfig {
        engine_displacement_cc: 2000,
        num_cylinders: 4,
        flow_rate_cc_min: 440,
        reference_pressure_kpa: 300,
        fuel_density_mg_cc: 750,
        dead_time_curve: [
            (90, 1500), (100, 1300), (110, 1150), (120, 1000),
            (130, 900), (140, 800), (150, 750), (160, 700),
        ],
    }
}

pub fn baseline_ve() -> VeTable { VeTable::default_safe() }
pub fn afr_table() -> AfrTable { AfrTable::default_gasoline() }

