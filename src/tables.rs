//! IPW (Injector Pulse Width) table lookup
//!
//! This module provides a simple 2D table lookup for fuel injection pulse widths.
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
//!
//! # No Interpolation
//!
//! For MVP simplicity, this implementation uses "nearest bin" lookup with no
//! interpolation. This causes step changes between bins but significantly
//! reduces computational complexity.

use crate::constants::fuel::*;

/// IPW (Injector Pulse Width) table - 16x16 grid
///
/// Stores pre-calculated pulse widths for fast lookup without complex math.
pub struct IpwTable {
    pub rpm_bins: [u16; 16],
    pub load_bins: [u16; 16],
    pub values: [[u16; 16]; 16],  // Microseconds [load_idx][rpm_idx]
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

    /// Lookup pulse width (no interpolation for MVP)
    ///
    /// Finds the nearest bin for both RPM and load, then returns the
    /// corresponding pulse width value.
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
    /// use ecu_core::IpwTable;
    /// let table = IpwTable::new();
    /// let pw = table.lookup(3000, 60);  // 3000 RPM, 60 kPa
    /// ```
    pub fn lookup(&self, rpm: u16, load: u16) -> u16 {
        let rpm_idx = self.find_index(&self.rpm_bins, rpm);
        let load_idx = self.find_index(&self.load_bins, load);
        self.values[load_idx][rpm_idx]
    }

    /// Find closest bin index for a value
    ///
    /// Uses simple linear search to find the bin that contains the value.
    /// For values below the first bin, returns 0.
    /// For values above the last bin, returns 15.
    ///
    /// # Arguments
    /// * `bins` - Array of bin boundaries (must be sorted ascending)
    /// * `value` - Value to locate
    ///
    /// # Returns
    /// Index of the closest bin (0-15)
    fn find_index(&self, bins: &[u16; 16], value: u16) -> usize {
        // If value is less than first bin, use first bin
        if value < bins[0] {
            return 0;
        }

        // Find first bin where value < next bin
        for i in 0..15 {
            if value < bins[i + 1] {
                return i;
            }
        }

        // Value is >= last bin, use last bin
        15
    }
}

impl Default for IpwTable {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_find_index_boundaries() {
        let table = IpwTable::new();

        // Test below first bin
        assert_eq!(table.find_index(&table.rpm_bins, 0), 0);
        assert_eq!(table.find_index(&table.rpm_bins, 499), 0);

        // Test first bin
        assert_eq!(table.find_index(&table.rpm_bins, 500), 0);
        assert_eq!(table.find_index(&table.rpm_bins, 999), 0);

        // Test middle bin
        assert_eq!(table.find_index(&table.rpm_bins, 3000), 5);

        // Test above last bin
        assert_eq!(table.find_index(&table.rpm_bins, 9000), 15);
    }

    #[test]
    fn test_lookup_default_values() {
        let table = IpwTable::new();

        // All default values should be DEFAULT_PULSE_WIDTH_US
        for rpm in [500, 2000, 4000, 8000].iter() {
            for load in [20, 60, 100, 170].iter() {
                let pw = table.lookup(*rpm, *load);
                assert_eq!(pw, DEFAULT_PULSE_WIDTH_US);
            }
        }
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

        // Verify RPM bins are sorted ascending
        for i in 0..15 {
            assert!(table.rpm_bins[i] < table.rpm_bins[i + 1],
                    "RPM bins not sorted at index {}", i);
        }

        // Verify load bins are sorted ascending
        for i in 0..15 {
            assert!(table.load_bins[i] < table.load_bins[i + 1],
                    "Load bins not sorted at index {}", i);
        }
    }
}
