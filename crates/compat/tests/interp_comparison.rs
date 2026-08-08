//! Exact-value tests for the shared bilinear table interpolation.
//!
//! These assert the interpolated value itself, not that it merely lands
//! somewhere inside the corner hull. A hull-membership check passes for almost
//! any wrong answer -- including a swapped interpolation axis -- so it cannot
//! distinguish a working lookup from a broken one.
//!
//! Expected values are derived from the documented rounding formula
//! (`(a * (255 - frac) + b * frac + 127) / 255`, applied along `frac_x` and
//! then across `frac_y`), not captured from a run of the implementation.

use ecu_compat::interp::{bilinear_interpolate_i16, bilinear_interpolate_u16};

/// Fuel-shaped corners: (20 kPa, 500 rpm) .. (30 kPa, 1000 rpm).
const V00: u16 = 1000;
const V01: u16 = 2000;
const V10: u16 = 3000;
const V11: u16 = 4000;

#[test]
fn bilinear_u16_returns_each_corner_exactly() {
    assert_eq!(bilinear_interpolate_u16(V00, V01, V10, V11, 0, 0), V00);
    assert_eq!(bilinear_interpolate_u16(V00, V01, V10, V11, 255, 0), V01);
    assert_eq!(bilinear_interpolate_u16(V00, V01, V10, V11, 0, 255), V10);
    assert_eq!(bilinear_interpolate_u16(V00, V01, V10, V11, 255, 255), V11);
}

/// `frac_x` must move along `v00 -> v01` and `frac_y` across to `v10 -> v11`.
/// Asymmetric fractions are what distinguish the two axes; the midpoint alone
/// is identical under a swap and proves nothing.
#[test]
fn bilinear_u16_axes_are_not_interchangeable() {
    let x_only = bilinear_interpolate_u16(V00, V01, V10, V11, 255, 0);
    let y_only = bilinear_interpolate_u16(V00, V01, V10, V11, 0, 255);
    assert_eq!(x_only, 2000, "frac_x must interpolate v00 -> v01");
    assert_eq!(y_only, 3000, "frac_y must interpolate v00 -> v10");

    assert_eq!(bilinear_interpolate_u16(V00, V01, V10, V11, 64, 192), 2757);
}

#[test]
fn bilinear_u16_midpoint_rounds_as_documented() {
    assert_eq!(bilinear_interpolate_u16(V00, V01, V10, V11, 128, 128), 2506);
}

#[test]
fn bilinear_i16_returns_each_corner_exactly() {
    assert_eq!(bilinear_interpolate_i16(10, 20, 30, 40, 0, 0), 10);
    assert_eq!(bilinear_interpolate_i16(10, 20, 30, 40, 255, 0), 20);
    assert_eq!(bilinear_interpolate_i16(10, 20, 30, 40, 0, 255), 30);
    assert_eq!(bilinear_interpolate_i16(10, 20, 30, 40, 255, 255), 40);
}

#[test]
fn bilinear_i16_axes_are_not_interchangeable() {
    assert_eq!(bilinear_interpolate_i16(10, 20, 30, 40, 64, 192), 28);
    assert_eq!(bilinear_interpolate_i16(10, 20, 30, 40, 128, 128), 25);
}

/// Ignition advance is signed and straddles zero, so cover negative corners.
///
/// Rounding is asymmetric about zero: the `+127` term combined with Rust's
/// truncate-toward-zero division biases negative results upward, so a negative
/// corner comes back 2 counts high (-98 for -100) while positive corners are
/// exact. This pins the behavior as it actually is; changing it would shift
/// ignition advance on the negative side of every table.
#[test]
fn bilinear_i16_rounding_is_asymmetric_about_zero() {
    assert_eq!(bilinear_interpolate_i16(-100, -50, 50, 100, 0, 0), -98);
    assert_eq!(bilinear_interpolate_i16(-100, -50, 50, 100, 255, 0), -48);
    assert_eq!(bilinear_interpolate_i16(-100, -50, 50, 100, 0, 255), 50);
    assert_eq!(bilinear_interpolate_i16(-100, -50, 50, 100, 255, 255), 100);
    assert_eq!(bilinear_interpolate_i16(-100, -50, 50, 100, 128, 128), 1);
}
