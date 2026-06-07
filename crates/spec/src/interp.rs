use crate::{Axis16, Kpa10, Rpm, Table2D16};

const fn clip_u16(value: u16, lo: u16, hi: u16) -> u16 {
    if value < lo {
        lo
    } else if value > hi {
        hi
    } else {
        value
    }
}

pub fn find_segment(axis: &Axis16, x: u16) -> usize {
    let len = axis.len as usize;
    if len < 2 {
        return 0;
    }

    let clipped = clip_u16(x, axis.values[0], axis.values[len - 1]);
    let mut idx = 0usize;
    while idx + 1 < len {
        let lo = axis.values[idx];
        let hi = axis.values[idx + 1];
        if clipped >= lo && (clipped < hi || (idx + 1 == len - 1 && clipped == hi)) {
            return idx;
        }
        idx += 1;
    }

    len - 2
}

pub fn lerp_u16(x0: u16, x1: u16, y0: u16, y1: u16, x: u16) -> u16 {
    if x1 <= x0 {
        return y0;
    }

    let x_clip = clip_u16(x, x0, x1);
    let num = (x_clip - x0) as i64;
    let den = (x1 - x0) as i64;
    let delta = y1 as i64 - y0 as i64;
    let product = delta * num;
    let quotient = product / den;
    let remainder = product % den;
    let correction = if product < 0 && remainder != 0 { -1 } else { 0 };
    (y0 as i64 + quotient + correction) as u16
}

pub fn lerp_i16(x0: u16, x1: u16, y0: i16, y1: i16, x: u16) -> i16 {
    if x1 <= x0 {
        return y0;
    }

    let x_clip = clip_u16(x, x0, x1);
    let num = (x_clip - x0) as i64;
    let den = (x1 - x0) as i64;
    let delta = y1 as i64 - y0 as i64;
    let product = delta * num;
    let quotient = product / den;
    let remainder = product % den;
    let correction = if product < 0 && remainder != 0 { -1 } else { 0 };
    (y0 as i64 + quotient + correction) as i16
}

pub fn bilerp_u16(table: &Table2D16<u16>, rpm: Rpm, load: Kpa10) -> u16 {
    let rpm_len = table.rpm_axis.len as usize;
    let load_len = table.load_axis.len as usize;
    let rpm_clip = clip_u16(
        rpm.0,
        table.rpm_axis.values[0],
        table.rpm_axis.values[rpm_len - 1],
    );
    let load_clip = clip_u16(
        load.0,
        table.load_axis.values[0],
        table.load_axis.values[load_len - 1],
    );
    let rpm_seg = find_segment(&table.rpm_axis, rpm_clip);
    let load_seg = find_segment(&table.load_axis, load_clip);

    let rpm_x0 = table.rpm_axis.values[rpm_seg];
    let rpm_x1 = table.rpm_axis.values[rpm_seg + 1];
    let load_y0 = table.load_axis.values[load_seg];
    let load_y1 = table.load_axis.values[load_seg + 1];

    let low_left = table.values[load_seg][rpm_seg];
    let low_right = table.values[load_seg][rpm_seg + 1];
    let high_left = table.values[load_seg + 1][rpm_seg];
    let high_right = table.values[load_seg + 1][rpm_seg + 1];

    let lower = lerp_u16(rpm_x0, rpm_x1, low_left, low_right, rpm_clip);
    let upper = lerp_u16(rpm_x0, rpm_x1, high_left, high_right, rpm_clip);
    lerp_u16(load_y0, load_y1, lower, upper, load_clip)
}

pub fn bilerp_i16(table: &Table2D16<i16>, rpm: Rpm, load: Kpa10) -> i16 {
    let rpm_len = table.rpm_axis.len as usize;
    let load_len = table.load_axis.len as usize;
    let rpm_clip = clip_u16(
        rpm.0,
        table.rpm_axis.values[0],
        table.rpm_axis.values[rpm_len - 1],
    );
    let load_clip = clip_u16(
        load.0,
        table.load_axis.values[0],
        table.load_axis.values[load_len - 1],
    );
    let rpm_seg = find_segment(&table.rpm_axis, rpm_clip);
    let load_seg = find_segment(&table.load_axis, load_clip);

    let rpm_x0 = table.rpm_axis.values[rpm_seg];
    let rpm_x1 = table.rpm_axis.values[rpm_seg + 1];
    let load_y0 = table.load_axis.values[load_seg];
    let load_y1 = table.load_axis.values[load_seg + 1];

    let low_left = table.values[load_seg][rpm_seg];
    let low_right = table.values[load_seg][rpm_seg + 1];
    let high_left = table.values[load_seg + 1][rpm_seg];
    let high_right = table.values[load_seg + 1][rpm_seg + 1];

    let lower = lerp_i16(rpm_x0, rpm_x1, low_left, low_right, rpm_clip);
    let upper = lerp_i16(rpm_x0, rpm_x1, high_left, high_right, rpm_clip);
    lerp_i16(load_y0, load_y1, lower, upper, load_clip)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{Axis16, Kpa10, Rpm, Table2D16};

    fn axis(values: &[u16]) -> Axis16 {
        let mut axis = Axis16 {
            len: values.len() as u8,
            ..Axis16::default()
        };
        let mut idx = 0usize;
        while idx < values.len() {
            axis.values[idx] = values[idx];
            idx += 1;
        }
        axis
    }

    fn table_u16(values: [[u16; 16]; 16]) -> Table2D16<u16> {
        Table2D16 {
            rpm_axis: axis(&[100, 200, 300]),
            load_axis: axis(&[10, 20, 30]),
            values,
        }
    }

    fn table_i16(values: [[i16; 16]; 16]) -> Table2D16<i16> {
        Table2D16 {
            rpm_axis: axis(&[100, 200, 300]),
            load_axis: axis(&[10, 20, 30]),
            values,
        }
    }

    #[test]
    fn finds_segments_with_boundary_closure() {
        let axis = axis(&[10, 20, 30, 40]);
        assert_eq!(find_segment(&axis, 10), 0);
        assert_eq!(find_segment(&axis, 19), 0);
        assert_eq!(find_segment(&axis, 20), 1);
        assert_eq!(find_segment(&axis, 30), 2);
        assert_eq!(find_segment(&axis, 40), 2);
        assert_eq!(find_segment(&axis, 5), 0);
        assert_eq!(find_segment(&axis, 99), 2);
    }

    #[test]
    fn lerp_u16_reproduces_endpoints() {
        assert_eq!(lerp_u16(10, 20, 100, 200, 10), 100);
        assert_eq!(lerp_u16(10, 20, 100, 200, 20), 200);
        assert_eq!(lerp_u16(10, 20, 100, 200, 15), 150);
    }

    #[test]
    fn lerp_u16_clips_x_to_segment_bounds() {
        assert_eq!(lerp_u16(10, 20, 100, 200, 5), 100);
        assert_eq!(lerp_u16(10, 20, 100, 200, 99), 200);
    }

    #[test]
    fn lerp_u16_handles_decreasing_endpoints() {
        assert_eq!(lerp_u16(10, 20, 400, 300, 15), 350);
    }

    #[test]
    fn lerp_i16_reproduces_endpoints() {
        assert_eq!(lerp_i16(10, 20, -100, 100, 10), -100);
        assert_eq!(lerp_i16(10, 20, -100, 100, 20), 100);
        assert_eq!(lerp_i16(10, 20, -100, 100, 15), 0);
    }

    #[test]
    fn lerp_i16_clips_x_to_segment_bounds() {
        assert_eq!(lerp_i16(10, 20, -100, 100, 5), -100);
        assert_eq!(lerp_i16(10, 20, -100, 100, 99), 100);
    }

    #[test]
    fn bilerp_u16_reproduces_grid_points() {
        let mut values = [[0u16; 16]; 16];
        values[0][0] = 10;
        values[0][1] = 20;
        values[1][0] = 30;
        values[1][1] = 40;
        let table = table_u16(values);
        assert_eq!(bilerp_u16(&table, Rpm(100), Kpa10(10)), 10);
        assert_eq!(bilerp_u16(&table, Rpm(200), Kpa10(10)), 20);
        assert_eq!(bilerp_u16(&table, Rpm(100), Kpa10(20)), 30);
        assert_eq!(bilerp_u16(&table, Rpm(200), Kpa10(20)), 40);
    }

    #[test]
    fn bilerp_i16_reproduces_grid_points() {
        let mut values = [[0i16; 16]; 16];
        values[0][0] = -10;
        values[0][1] = 0;
        values[1][0] = 10;
        values[1][1] = 20;
        let table = table_i16(values);
        assert_eq!(bilerp_i16(&table, Rpm(100), Kpa10(10)), -10);
        assert_eq!(bilerp_i16(&table, Rpm(200), Kpa10(10)), 0);
        assert_eq!(bilerp_i16(&table, Rpm(100), Kpa10(20)), 10);
        assert_eq!(bilerp_i16(&table, Rpm(200), Kpa10(20)), 20);
    }

    #[test]
    fn bilerp_clips_to_edges() {
        let mut values = [[0u16; 16]; 16];
        values[0][0] = 100;
        values[0][1] = 200;
        values[0][2] = 300;
        values[1][0] = 200;
        values[1][1] = 300;
        values[1][2] = 400;
        values[2][0] = 300;
        values[2][1] = 400;
        values[2][2] = 500;
        let table = table_u16(values);
        assert_eq!(bilerp_u16(&table, Rpm(50), Kpa10(5)), 100);
        assert_eq!(bilerp_u16(&table, Rpm(500), Kpa10(500)), 500);
    }
}
