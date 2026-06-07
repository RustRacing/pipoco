//! Shared interpolation helpers for table lookups.

pub fn find_bin_interpolation(bins: &[u16], value: u16) -> (usize, usize, u8) {
    if value <= bins[0] {
        return (0, 0, 0);
    }
    if value >= bins[bins.len() - 1] {
        let last = bins.len() - 1;
        return (last, last, 0);
    }
    for i in 0..bins.len() - 1 {
        if value >= bins[i] && value <= bins[i + 1] {
            let range = bins[i + 1] - bins[i];
            let offset = value - bins[i];
            let frac = if range > 0 {
                (((offset as u32 * 255) + (range as u32 / 2)) / range as u32) as u8
            } else {
                0
            };
            return (i, i + 1, frac);
        }
    }
    let last = bins.len() - 1;
    (last, last, 0)
}

pub fn interpolate_u16(a: u16, b: u16, frac: u8) -> u16 {
    let numerator = a as u32 * (255 - frac as u32) + b as u32 * frac as u32 + 127;
    (numerator / 255) as u16
}

pub fn bilinear_interpolate_u16(
    v00: u16,
    v01: u16,
    v10: u16,
    v11: u16,
    frac_x: u8,
    frac_y: u8,
) -> u16 {
    let v0 = interpolate_u16(v00, v01, frac_x);
    let v1 = interpolate_u16(v10, v11, frac_x);
    interpolate_u16(v0, v1, frac_y)
}

pub fn interpolate_i16(a: i16, b: i16, frac: u8) -> i16 {
    let a32 = a as i32;
    let b32 = b as i32;
    let frac32 = frac as i32;
    let num = a32 * (255 - frac32) + b32 * frac32 + 127;
    (num / 255) as i16
}

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
