//! IPW (Injector Pulse Width) table lookup
//!
//! This module provides 2D table lookup for fuel injection pulse widths.
//! Unlike traditional VE (Volumetric Efficiency) tables, IPW tables store the
//! final pulse width values in microseconds, eliminating the need for complex
//! calculations in the embedded module.
//!
//! # Table Structure
//!
//! The table is a 16x16 grid indexed by:
//! - X-axis: RPM (revolutions per minute)
//! - Y-axis: Load (manifold pressure in kPa or TPS in %)
//!
//! Values are pulse widths in microseconds, ready to apply to injectors.

use crate::constants::fuel::*;
use crate::interp::{bilinear_interpolate_u16, find_bin_interpolation};

/// Shared lookup contract for pulse-width tables.
pub trait TableLookup {
    fn lookup(&self, rpm: u16, load: u16) -> u16;
}

/// IPW (Injector Pulse Width) table - 16x16 grid
///
/// Stores pre-calculated pulse widths for fast lookup without complex math.
pub struct IpwTable {
    pub rpm_bins: [u16; 16],
    pub load_bins: [u16; 16],
    pub values: [[u16; 16]; 16], // Microseconds [load_idx][rpm_idx]
}

impl IpwTable {
    /// Create new table with default values
    ///
    /// Uses constants from `constants::fuel` module for consistent configuration.
    pub const fn new() -> Self {
        Self {
            rpm_bins: RPM_BINS,
            load_bins: LOAD_BINS,
            values: [[DEFAULT_PULSE_WIDTH_US; 16]; 16],
        }
    }

    /// Lookup pulse width using bilinear interpolation.
    ///
    /// # Arguments
    /// * `rpm` - Engine speed in revolutions per minute
    /// * `load` - Engine load in kPa (or TPS %)
    ///
    /// # Returns
    /// Pulse width in microseconds
    ///
    /// # Example
    /// ```
    /// use ecu_compat::IpwTable;
    /// let table = IpwTable::new();
    /// let pw = table.lookup(3000, 60);  // 3000 RPM, 60 kPa
    /// ```
    pub fn lookup(&self, rpm: u16, load: u16) -> u16 {
        TableLookup::lookup(self, rpm, load)
    }

    fn lookup_bilinear(&self, rpm: u16, load: u16) -> u16 {
        let (rx0, rx1, fx) = find_bin_interpolation(&self.rpm_bins, rpm);
        let (ly0, ly1, fy) = find_bin_interpolation(&self.load_bins, load);
        let v00 = self.values[ly0][rx0];
        let v01 = self.values[ly0][rx1];
        let v10 = self.values[ly1][rx0];
        let v11 = self.values[ly1][rx1];
        bilinear_interpolate_u16(v00, v01, v10, v11, fx, fy)
    }

    fn find_index(&self, bins: &[u16; 16], value: u16) -> usize {
        if value < bins[0] {
            return 0;
        }
        for i in 0..15 {
            if value < bins[i + 1] {
                return i;
            }
        }
        15
    }

    pub fn lookup_nearest(&self, rpm: u16, load: u16) -> u16 {
        let rpm_idx = self.find_index(&self.rpm_bins, rpm);
        let load_idx = self.find_index(&self.load_bins, load);
        self.values[load_idx][rpm_idx]
    }
}

impl Default for IpwTable {
    fn default() -> Self {
        Self::new()
    }
}

impl TableLookup for IpwTable {
    fn lookup(&self, rpm: u16, load: u16) -> u16 {
        self.lookup_bilinear(rpm, load)
    }
}

/// Nearest-neighbor IPW lookup wrapper.
pub struct IpwTableNearest(pub IpwTable);

impl TableLookup for IpwTableNearest {
    fn lookup(&self, rpm: u16, load: u16) -> u16 {
        self.0.lookup_nearest(rpm, load)
    }
}

/// Bilinear IPW lookup wrapper.
pub struct IpwTableBilinear(pub IpwTable);

impl TableLookup for IpwTableBilinear {
    fn lookup(&self, rpm: u16, load: u16) -> u16 {
        self.0.lookup_bilinear(rpm, load)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_lookup_default_values() {
        let table = IpwTable::new();

        for rpm in [500, 2000, 4000, 8000].iter() {
            for load in [20, 60, 100, 170].iter() {
                let pw = table.lookup(*rpm, *load);
                assert_eq!(pw, DEFAULT_PULSE_WIDTH_US);
            }
        }
    }

    #[test]
    fn test_bilinear_interpolation_midpoint() {
        let mut table = IpwTable::new();
        table.values[0][0] = 1000; // (20,500)
        table.values[0][1] = 2000; // (20,1000)
        table.values[1][0] = 3000; // (30,500)
        table.values[1][1] = 4000; // (30,1000)

        let pw = table.lookup(750, 25);
        assert!((2488..=2512).contains(&pw), "pw={pw}");
    }

    #[test]
    fn test_table_dimensions() {
        let table = IpwTable::new();

        assert_eq!(table.rpm_bins.len(), 16);
        assert_eq!(table.load_bins.len(), 16);
        assert_eq!(table.values.len(), 16);
        assert_eq!(table.values[0].len(), 16);
    }

    #[test]
    fn test_bins_sorted() {
        let table = IpwTable::new();

        for i in 0..15 {
            assert!(
                table.rpm_bins[i] < table.rpm_bins[i + 1],
                "RPM bins not sorted at index {i}"
            );
        }

        for i in 0..15 {
            assert!(
                table.load_bins[i] < table.load_bins[i + 1],
                "Load bins not sorted at index {i}"
            );
        }
    }
}
