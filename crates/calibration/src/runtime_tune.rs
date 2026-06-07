pub const FUEL_RUNTIME_TABLE_DIM: usize = 16;

pub type FuelRuntimeTable16 = [[u16; FUEL_RUNTIME_TABLE_DIM]; FUEL_RUNTIME_TABLE_DIM];

/// Canonical RPM axis for core-free 16x16 fuel runtime tables.
pub const FUEL_RUNTIME_RPM_BINS: [u16; FUEL_RUNTIME_TABLE_DIM] = [
    500, 1000, 1500, 2000, 2500, 3000, 3500, 4000, 4500, 5000, 5500, 6000, 6500, 7000, 7500, 8000,
];

/// Canonical load axis for core-free 16x16 fuel runtime tables, in kPa.
pub const FUEL_RUNTIME_LOAD_BINS: [u16; FUEL_RUNTIME_TABLE_DIM] = [
    20, 30, 40, 50, 60, 70, 80, 90, 100, 110, 120, 130, 140, 150, 160, 170,
];

/// Core-free fuel tune fields needed to build runtime fuel calibration.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct FuelRuntimeTune {
    pub ve_table: FuelRuntimeTable16,
    pub afr_table: FuelRuntimeTable16,
    pub required_fuel_us: u16,
    pub injector_deadtime_us: u16,
    pub ve_load_source: u8,
}

impl FuelRuntimeTune {
    pub const fn new(
        ve_table: FuelRuntimeTable16,
        afr_table: FuelRuntimeTable16,
        required_fuel_us: u16,
        injector_deadtime_us: u16,
        ve_load_source: u8,
    ) -> Self {
        Self {
            ve_table,
            afr_table,
            required_fuel_us,
            injector_deadtime_us,
            ve_load_source,
        }
    }
}
