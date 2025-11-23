//! Calibrated sensor models built from generic pieces

use super::{
    convert::{counts_to_mv, node_mv_to_resistance_ohms, DividerConfig},
    curve::Piecewise,
};

/// Trait for reading a calibrated engineering value from raw ADC counts
pub trait CalibratedSensor {
    /// Convert raw ADC counts to engineering units (scaled int)
    fn value_from_counts(&self, counts: u16) -> i32;
}

/// Resistive sensor through divider (e.g., thermistor)
pub struct ResistiveSensor<const N: usize> {
    pub vref_mv: u16,
    pub adc_bits: u8,
    pub r_known_ohms: u32,
    pub divider: DividerConfig,
    /// Calibration curve: ohms → engineering units (e.g., degC)
    pub curve: Piecewise<N>,
}

impl<const N: usize> CalibratedSensor for ResistiveSensor<N> {
    fn value_from_counts(&self, counts: u16) -> i32 {
        let mv = counts_to_mv(counts, self.vref_mv, self.adc_bits);
        let ohms =
            node_mv_to_resistance_ohms(mv, self.vref_mv as u32, self.r_known_ohms, self.divider);
        self.curve.map(ohms)
    }
}

/// Voltage-output sensor (e.g., MAP, wideband analog)
pub struct VoltageSensor<const N: usize> {
    pub vref_mv: u16,
    pub adc_bits: u8,
    /// Calibration curve: millivolts → engineering units
    pub curve: Piecewise<N>,
}

impl<const N: usize> CalibratedSensor for VoltageSensor<N> {
    fn value_from_counts(&self, counts: u16) -> i32 {
        let mv = counts_to_mv(counts, self.vref_mv, self.adc_bits);
        self.curve.map(mv)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_voltage_sensor_linear() {
        // 0..5000 mV => 0..250 kPa (x10)
        let m = VoltageSensor {
            vref_mv: 3300,
            adc_bits: 12,
            curve: Piecewise::new([0, 5000], [0, 2500]),
        };
        let y = m.value_from_counts(2048); // ~1650 mV
        assert!(y > 700 && y < 900);
    }
}
