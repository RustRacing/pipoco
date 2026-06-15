use ecu_compat::interp::{bilinear_interpolate_i16, bilinear_interpolate_u16};

#[test]
fn bilinear_vs_corner_within_bounds_fuel() {
    // Corners for first cell (load 20/30 kPa, rpm 500/1000)
    let v00: u16 = 1000; // (20,500)
    let v01: u16 = 2000; // (20,1000)
    let v10: u16 = 3000; // (30,500)
    let v11: u16 = 4000; // (30,1000)

    // Midpoint (frac_x=frac_y=128)
    let bil = bilinear_interpolate_u16(v00, v01, v10, v11, 128, 128);

    let min = v00.min(v01).min(v10).min(v11);
    let max = v00.max(v01).max(v10).max(v11);

    // Bilinear stays within corner range
    assert!((min..=max).contains(&bil), "bil={bil}");

    // Old nearest-neighbor (lower-left corner for cell) equals v00
    let nn = v00;
    assert!((min..=max).contains(&nn), "nn={nn}");

    // Delta bounded by half the sum of edge deltas (loose check)
    let dx = (v01 as i32 - v00 as i32).unsigned_abs();
    let dy = (v10 as i32 - v00 as i32).unsigned_abs();
    let delta = bil.abs_diff(nn);
    assert!(
        delta as u32 <= (dx + dy) / 2 + 8,
        "delta={delta}, bound={}",
        (dx + dy) / 2
    );
}

#[test]
fn bilinear_vs_corner_within_bounds_ign() {
    // Corners for ignition timing (deg*1)
    let v00: i16 = 10;
    let v01: i16 = 20;
    let v10: i16 = 30;
    let v11: i16 = 40;

    let bil = bilinear_interpolate_i16(v00, v01, v10, v11, 128, 128);
    let min = v00.min(v01).min(v10).min(v11);
    let max = v00.max(v01).max(v10).max(v11);
    assert!((min..=max).contains(&bil), "bil={bil}");

    let nn = v00; // lower-left
    assert!((min..=max).contains(&nn));

    let dx = (v01 - v00).unsigned_abs() as u32;
    let dy = (v10 - v00).unsigned_abs() as u32;
    let delta = (bil - nn).unsigned_abs() as u32;
    assert!(
        delta <= (dx + dy) / 2 + 1,
        "delta={delta}, bound={}",
        (dx + dy) / 2
    );
}
