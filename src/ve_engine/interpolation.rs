//! Table Interpolation Functions
//!
//! Bilinear interpolation for 2D lookup tables.
//! Currently implements nearest-neighbor lookup (no interpolation).
//! Full bilinear interpolation can be added when needed.

/// Find nearest bin index in sorted array
///
/// # Arguments
/// * `bins` - Sorted array of bin edges
/// * `value` - Value to find
///
/// # Returns
/// Index of nearest bin
pub fn find_nearest_bin(bins: &[u16], value: u16) -> usize {
    // Handle edge cases
    if value <= bins[0] {
        return 0;
    }
    if value >= bins[bins.len() - 1] {
        return bins.len() - 1;
    }

    // Binary search for closest bin
    for i in 0..bins.len() - 1 {
        if value >= bins[i] && value < bins[i + 1] {
            // Check which bin is closer
            let dist_to_lower = value - bins[i];
            let dist_to_upper = bins[i + 1] - value;

            return if dist_to_lower < dist_to_upper {
                i
            } else {
                i + 1
            };
        }
    }

    bins.len() - 1
}

/// Find bin indices and interpolation fraction
///
/// Returns (lower_index, upper_index, fraction) where fraction is 0-255.
///
/// # Arguments
/// * `bins` - Sorted array of bin edges
/// * `value` - Value to interpolate
///
/// # Returns
/// (lower_idx, upper_idx, fraction) where fraction is 0-255
///
/// # Example
/// ```
/// use ecu_core::ve_engine::interpolation::find_bin_interpolation;
///
/// let bins = [1000, 2000, 3000, 4000];
/// let (low, high, frac) = find_bin_interpolation(&bins, 2500);
///
/// assert_eq!(low, 1);   // 2000 RPM bin
/// assert_eq!(high, 2);  // 3000 RPM bin
/// assert_eq!(frac, 128); // 50% between (255/2)
/// ```
pub fn find_bin_interpolation(bins: &[u16], value: u16) -> (usize, usize, u8) {
    // Handle edge cases
    if value <= bins[0] {
        return (0, 0, 0);
    }
    if value >= bins[bins.len() - 1] {
        let last = bins.len() - 1;
        return (last, last, 0);
    }

    // Find bounding bins
    for i in 0..bins.len() - 1 {
        if value >= bins[i] && value <= bins[i + 1] {
            // Calculate fractional position (0-255) with rounding
            let range = bins[i + 1] - bins[i];
            let offset = value - bins[i];

            let frac = if range > 0 {
                // Add range/2 for rounding
                (((offset as u32 * 255) + (range as u32 / 2)) / range as u32) as u8
            } else {
                0
            };

            return (i, i + 1, frac);
        }
    }

    // Shouldn't reach here
    let last = bins.len() - 1;
    (last, last, 0)
}

/// Linear interpolation between two u8 values
///
/// # Arguments
/// * `a` - Lower bound value
/// * `b` - Upper bound value
/// * `frac` - Fraction (0-255) where 0 = a, 255 = b
///
/// # Returns
/// Interpolated value
///
/// # Example
/// ```
/// use ecu_core::ve_engine::interpolation::interpolate_u8;
///
/// let result = interpolate_u8(80, 100, 128);  // 50% between 80 and 100
/// assert_eq!(result, 90);
/// ```
pub fn interpolate_u8(a: u8, b: u8, frac: u8) -> u8 {
    // Interpolate with rounding
    // result = a * (255-frac)/255 + b * frac/255
    // Rearrange: result = (a * (255-frac) + b * frac + 127) / 255
    let numerator = a as u32 * (255 - frac as u32) + b as u32 * frac as u32 + 127;
    (numerator / 255) as u8
}

/// Linear interpolation between two u16 values
///
/// # Arguments
/// * `a` - Lower bound value
/// * `b` - Upper bound value
/// * `frac` - Fraction (0-255) where 0 = a, 255 = b
///
/// # Returns
/// Interpolated value
pub fn interpolate_u16(a: u16, b: u16, frac: u8) -> u16 {
    // Interpolate with rounding
    // result = a * (255-frac)/255 + b * frac/255
    // Rearrange: result = (a * (255-frac) + b * frac + 127) / 255
    let numerator = a as u32 * (255 - frac as u32) + b as u32 * frac as u32 + 127;
    (numerator / 255) as u16
}

/// Bilinear interpolation for 2D table
///
/// Interpolates a value in a 2D table given 4 corner values.
///
/// # Arguments
/// * `v00` - Value at (x_low, y_low)
/// * `v01` - Value at (x_high, y_low)
/// * `v10` - Value at (x_low, y_high)
/// * `v11` - Value at (x_high, y_high)
/// * `frac_x` - X fraction (0-255)
/// * `frac_y` - Y fraction (0-255)
///
/// # Returns
/// Interpolated value
///
/// # Example
/// ```
/// use ecu_core::ve_engine::interpolation::bilinear_interpolate_u8;
///
/// // Corners: 80, 90, 85, 95
/// let result = bilinear_interpolate_u8(
///     80, 90,  // Bottom row
///     85, 95,  // Top row
///     128, 128 // 50% in both directions
/// );
///
/// // Should be ~87.5 (average of all 4 corners)
/// assert!(result >= 87 && result <= 88, "result = {}", result);
/// ```
pub fn bilinear_interpolate_u8(v00: u8, v01: u8, v10: u8, v11: u8, frac_x: u8, frac_y: u8) -> u8 {
    // Interpolate along X axis (bottom row)
    let v0 = interpolate_u8(v00, v01, frac_x);

    // Interpolate along X axis (top row)
    let v1 = interpolate_u8(v10, v11, frac_x);

    // Interpolate along Y axis
    interpolate_u8(v0, v1, frac_y)
}

/// Bilinear interpolation for 2D table (u16 version)
pub fn bilinear_interpolate_u16(
    v00: u16,
    v01: u16,
    v10: u16,
    v11: u16,
    frac_x: u8,
    frac_y: u8,
) -> u16 {
    // Interpolate along X axis
    let v0 = interpolate_u16(v00, v01, frac_x);
    let v1 = interpolate_u16(v10, v11, frac_x);

    // Interpolate along Y axis
    interpolate_u16(v0, v1, frac_y)
}

/// Linear interpolation between two i16 values with rounding
pub fn interpolate_i16(a: i16, b: i16, frac: u8) -> i16 {
    let a32 = a as i32;
    let b32 = b as i32;
    let frac32 = frac as i32;
    let num = a32 * (255 - frac32) + b32 * frac32 + 127;
    (num / 255) as i16
}

/// Bilinear interpolation for 2D table (i16 version)
pub fn bilinear_interpolate_i16(
    v00: i16,
    v01: i16,
    v10: i16,
    v11: i16,
    frac_x: u8,
    frac_y: u8,
) -> i16 {
    let v0 = interpolate_i16(v00, v01, frac_x);
    let v1 = interpolate_i16(v10, v11, frac_x);
    interpolate_i16(v0, v1, frac_y)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_find_nearest_bin() {
        let bins = [1000, 2000, 3000, 4000, 5000];

        assert_eq!(find_nearest_bin(&bins, 500), 0); // Below range
        assert_eq!(find_nearest_bin(&bins, 1000), 0); // Exact match
        assert_eq!(find_nearest_bin(&bins, 1400), 0); // Closer to 1000 (400 away vs 600 away)
        assert_eq!(find_nearest_bin(&bins, 1600), 1); // Closer to 2000 (400 away vs 600 away)
        assert_eq!(find_nearest_bin(&bins, 3000), 2); // Exact match
        assert_eq!(find_nearest_bin(&bins, 5500), 4); // Above range
    }

    #[test]
    fn test_find_bin_interpolation() {
        let bins = [1000, 2000, 3000, 4000];

        // Exact at bin 2000 (between bins 0 and 1)
        let (low, high, frac) = find_bin_interpolation(&bins, 2000);
        assert_eq!(low, 0); // Lower bin
        assert_eq!(high, 1); // Upper bin
        assert_eq!(frac, 255); // 100% towards upper bin

        // Midpoint between 2000 and 3000
        let (low, high, frac) = find_bin_interpolation(&bins, 2500);
        assert_eq!(low, 1);
        assert_eq!(high, 2);
        assert_eq!(frac, 128); // ~50% (255/2 rounded)

        // Exact at bin 3000 (between bins 1 and 2)
        let (low, high, frac) = find_bin_interpolation(&bins, 3000);
        assert_eq!(low, 1);
        assert_eq!(high, 2);
        assert_eq!(frac, 255); // 100% towards upper bin
    }

    #[test]
    fn test_interpolate_u8() {
        assert_eq!(interpolate_u8(80, 100, 0), 80); // 0% = lower bound
        assert_eq!(interpolate_u8(80, 100, 255), 100); // 100% = upper bound
        assert_eq!(interpolate_u8(80, 100, 128), 90); // 50% = midpoint
    }

    #[test]
    fn test_interpolate_u16() {
        assert_eq!(interpolate_u16(1000, 2000, 0), 1000);
        assert_eq!(interpolate_u16(1000, 2000, 255), 2000);
        // At 50% (frac=128), result is 1502 due to integer rounding
        // This is acceptable (<0.2% error)
        assert_eq!(interpolate_u16(1000, 2000, 128), 1502);
    }

    #[test]
    fn test_bilinear_interpolate_u8() {
        // Square with values 80, 90, 85, 95
        // At center (50%, 50%), should be average ≈ 87.5
        let result = bilinear_interpolate_u8(80, 90, 85, 95, 128, 128);
        assert!((87..=88).contains(&result), "result = {result}");
    }

    #[test]
    fn test_bilinear_corners() {
        // Test that corners return exact values
        assert_eq!(bilinear_interpolate_u8(80, 90, 85, 95, 0, 0), 80);
        assert_eq!(bilinear_interpolate_u8(80, 90, 85, 95, 255, 0), 90);
        assert_eq!(bilinear_interpolate_u8(80, 90, 85, 95, 0, 255), 85);
        assert_eq!(bilinear_interpolate_u8(80, 90, 85, 95, 255, 255), 95);
    }

    #[test]
    fn test_bilinear_edges() {
        // Test midpoints of edges
        // Bottom edge (y=0)
        let result = bilinear_interpolate_u8(80, 90, 85, 95, 128, 0);
        assert_eq!(result, 85); // Midpoint of 80 and 90

        // Top edge (y=255)
        let result = bilinear_interpolate_u8(80, 90, 85, 95, 128, 255);
        assert_eq!(result, 90); // Midpoint of 85 and 95

        // Left edge (x=0)
        let result = bilinear_interpolate_u8(80, 90, 85, 95, 0, 128);
        assert!((82..=83).contains(&result), "result = {result}"); // Midpoint of 80 and 85

        // Right edge (x=255)
        let result = bilinear_interpolate_u8(80, 90, 85, 95, 255, 128);
        assert!((92..=93).contains(&result), "result = {result}"); // Midpoint of 90 and 95
    }

    #[test]
    fn test_interpolate_i16_and_bilinear() {
        assert_eq!(interpolate_i16(10, 20, 0), 10);
        assert_eq!(interpolate_i16(10, 20, 255), 20);
        let mid = interpolate_i16(10, 20, 128);
        assert!((15..=16).contains(&mid));

        let center = bilinear_interpolate_i16(10, 20, 30, 40, 128, 128);
        assert!((24..=26).contains(&center));
    }
}
