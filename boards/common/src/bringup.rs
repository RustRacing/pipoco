use ecu_domain::{Kpa10, PulseWidthUs, Rpm};
use ecu_runtime::BaseFuelModel;

pub fn bringup_fuel_model() -> BaseFuelModel {
    let rpm_bins = [
        Rpm::new(500),
        Rpm::new(1000),
        Rpm::new(1500),
        Rpm::new(2000),
        Rpm::new(2500),
        Rpm::new(3000),
        Rpm::new(3500),
        Rpm::new(4000),
        Rpm::new(4500),
        Rpm::new(5000),
        Rpm::new(5500),
        Rpm::new(6000),
        Rpm::new(6500),
        Rpm::new(7000),
        Rpm::new(7500),
        Rpm::new(8000),
    ];
    let load_bins = [
        Kpa10::new(200),
        Kpa10::new(300),
        Kpa10::new(400),
        Kpa10::new(500),
        Kpa10::new(600),
        Kpa10::new(700),
        Kpa10::new(800),
        Kpa10::new(900),
        Kpa10::new(1000),
        Kpa10::new(1100),
        Kpa10::new(1200),
        Kpa10::new(1300),
        Kpa10::new(1400),
        Kpa10::new(1500),
        Kpa10::new(1600),
        Kpa10::new(1700),
    ];
    let mut pulse_widths = [[PulseWidthUs::new(0); 16]; 16];
    pulse_widths[0][0] = PulseWidthUs::new(1000);
    pulse_widths[5][5] = PulseWidthUs::new(2500);
    pulse_widths[15][15] = PulseWidthUs::new(4000);

    BaseFuelModel::new(rpm_bins, load_bins, pulse_widths)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn bringup_fuel_model_has_expected_anchor_cells() {
        let model = bringup_fuel_model();

        assert_eq!(
            model.calculate_base_fuel(Rpm::new(500), Kpa10::new(200)),
            PulseWidthUs::new(1000)
        );
        assert_eq!(
            model.calculate_base_fuel(Rpm::new(3000), Kpa10::new(700)),
            PulseWidthUs::new(2500)
        );
        assert_eq!(
            model.calculate_base_fuel(Rpm::new(8000), Kpa10::new(1700)),
            PulseWidthUs::new(4000)
        );
    }
}
