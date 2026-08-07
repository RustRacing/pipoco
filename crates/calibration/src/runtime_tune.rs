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

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn fuel_runtime_tune_carries_runtime_boundary_fields() {
        let mut ve_table = [[100; FUEL_RUNTIME_TABLE_DIM]; FUEL_RUNTIME_TABLE_DIM];
        let mut afr_table = [[147; FUEL_RUNTIME_TABLE_DIM]; FUEL_RUNTIME_TABLE_DIM];
        ve_table[2][3] = 81;
        afr_table[4][5] = 132;

        let tune = FuelRuntimeTune::new(ve_table, afr_table, 2400, 775, 1);

        assert_eq!(tune.ve_table[2][3], 81);
        assert_eq!(tune.afr_table[4][5], 132);
        assert_eq!(tune.required_fuel_us, 2400);
        assert_eq!(tune.injector_deadtime_us, 775);
        assert_eq!(tune.ve_load_source, 1);
    }

    #[test]
    fn fuel_runtime_axes_match_legacy_core_defaults() {
        assert_eq!(
            FUEL_RUNTIME_RPM_BINS,
            [
                500, 1000, 1500, 2000, 2500, 3000, 3500, 4000, 4500, 5000, 5500, 6000, 6500, 7000,
                7500, 8000,
            ]
        );
        assert_eq!(
            FUEL_RUNTIME_LOAD_BINS,
            [20, 30, 40, 50, 60, 70, 80, 90, 100, 110, 120, 130, 140, 150, 160, 170,]
        );
    }

    #[test]
    fn fuel_runtime_axes_are_len_16_and_strictly_monotonic() {
        assert_eq!(FUEL_RUNTIME_RPM_BINS.len(), 16);
        assert_eq!(FUEL_RUNTIME_LOAD_BINS.len(), 16);
        for pair in FUEL_RUNTIME_RPM_BINS.windows(2) {
            assert!(pair[0] < pair[1], "RPM bins must be strictly increasing");
        }
        for pair in FUEL_RUNTIME_LOAD_BINS.windows(2) {
            assert!(pair[0] < pair[1], "load bins must be strictly increasing");
        }
    }
}
