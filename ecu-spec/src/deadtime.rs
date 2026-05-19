use crate::interp::bilerp_u16;
use crate::{Kpa10, Millivolts, PulseWidthUs, Table2D16};

pub fn deadtime_lookup(
    deadtime_table_us: &Table2D16<u16>,
    vbat_mv: Millivolts,
    fuel_pressure_kpa10: Kpa10,
) -> PulseWidthUs {
    PulseWidthUs(bilerp_u16(
        deadtime_table_us,
        crate::Rpm(vbat_mv.0),
        fuel_pressure_kpa10,
    ) as u32)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::Axis16;

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

    fn deadtime_table() -> Table2D16<u16> {
        let mut values = [[0u16; 16]; 16];
        values[0][0] = 100;
        values[0][1] = 200;
        values[1][0] = 300;
        values[1][1] = 500;
        Table2D16 {
            rpm_axis: axis(&[1000, 13000]),
            load_axis: axis(&[1000, 3000]),
            values,
        }
    }

    #[test]
    fn deadtime_zero_voltage_edge_clamps_to_min_axis() {
        let table = deadtime_table();
        let deadtime = deadtime_lookup(&table, Millivolts(0), Kpa10(1000));
        assert_eq!(deadtime, PulseWidthUs(100));
    }

    #[test]
    fn deadtime_saturated_voltage_edge_clamps_to_max_axis() {
        let table = deadtime_table();
        let deadtime = deadtime_lookup(&table, Millivolts(20_000), Kpa10(3000));
        assert_eq!(deadtime, PulseWidthUs(500));
    }

    #[test]
    fn deadtime_interpolation_midpoint_uses_bilinear_floor() {
        let table = deadtime_table();
        let deadtime = deadtime_lookup(&table, Millivolts(7000), Kpa10(2000));
        assert_eq!(deadtime, PulseWidthUs(275));
    }
}
