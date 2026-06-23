use crate::{Degrees10, DwellUs, PulseWidthUs};
pub use ecu_board_api::{
    FuelOutputMode, FuelOutputProfile, IgnitionOutputProfile, InjectionOutputProfile,
    SparkOutputMode, SparkOutputProfile,
};

/// Spark-only control intent without an already-selected coil output.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct SparkPlan {
    pub dwell: DwellUs,
    pub advance: Degrees10,
}

impl SparkPlan {
    pub const fn new(dwell: DwellUs, advance: Degrees10) -> Self {
        Self { dwell, advance }
    }
}

/// Fuel-only control intent without an already-selected injector output.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct FuelPlan {
    pub pulse_width: PulseWidthUs,
}

impl FuelPlan {
    pub const fn new(pulse_width: PulseWidthUs) -> Self {
        Self { pulse_width }
    }
}
