#![allow(dead_code)]

mod breathing;
mod energy;
mod shape;
mod types;

pub use breathing::*;
pub use energy::*;
pub use shape::*;
pub use types::*;

#[cfg(test)]
mod tests {
    use crate::types::*;

    use super::*;

    #[test]
    fn brake_thermal_efficiency_does_not_subtract_losses_twice() {
        let input = EnergyTorqueInput {
            fuel_energy_per_cycle: EnergyMicroJ(6_500_000_000),
            eta_x1000: 265,
            efficiency_basis: EfficiencyBasis::BrakeThermal,
            friction_torque_nm_x100: TorqueNmX100(2000),
            pumping_torque_nm_x100: TorqueNmX100(500),
            accessory_torque_nm_x100: TorqueNmX100(300),
        };

        assert_eq!(
            brake_torque_from_energy(input),
            torque_from_cycle_energy(EnergyMicroJ(6_500_000_000), 265)
        );
    }

    #[test]
    fn indicated_thermal_efficiency_subtracts_losses_once() {
        let input = EnergyTorqueInput {
            fuel_energy_per_cycle: EnergyMicroJ(6_500_000_000),
            eta_x1000: 265,
            efficiency_basis: EfficiencyBasis::IndicatedThermal,
            friction_torque_nm_x100: TorqueNmX100(2000),
            pumping_torque_nm_x100: TorqueNmX100(500),
            accessory_torque_nm_x100: TorqueNmX100(300),
        };

        let base = torque_from_cycle_energy(EnergyMicroJ(6_500_000_000), 265);
        assert_eq!(brake_torque_from_energy(input), TorqueNmX100(base.0 - 2800));
    }

    #[test]
    fn backsolve_reports_brake_and_indicated_efficiency_requirements() {
        let report = torque_backsolve(
            Rpm(3600),
            TorqueNmX100(14_000),
            TorqueNmX100(13_700),
            MassUg(1_900_000),
            MassUg(145_000),
            EnergyMicroJ(6_200_000_000),
            TorqueNmX100(2_000),
            1993,
        );

        assert!(report.eta_bte_required_x1000 > 0);
        assert!(report.eta_ite_required_x1000 > report.eta_bte_required_x1000);
        assert!(report.bmep_target_bar_x100.0 > report.bmep_sim_bar_x100.0);
    }

    #[test]
    fn breathing_presets_capture_expected_complexity_order() {
        let sohc = EngineBreathingClass::TwoValveSohcCarbLowCompression.preset();
        let fixed = EngineBreathingClass::FourValveDohcFixedCam.preset();
        let phased = EngineBreathingClass::FourValveDohcCamPhased.preset();

        assert!(phased.peak_ve_x1000 > sohc.peak_ve_x1000);
        assert!(phased.high_rpm_ve_retention_x1000 > sohc.high_rpm_ve_retention_x1000);
        assert!(phased.high_rpm_ve_retention_x1000 > fixed.high_rpm_ve_retention_x1000);
        assert!(!sohc.expects_cam_phasing);
        assert!(phased.expects_cam_phasing);
    }

    #[test]
    fn torque_retention_reports_power_peak_ratio() {
        assert_eq!(
            torque_retention_x1000(TorqueNmX100(22_800), TorqueNmX100(24_500)),
            930
        );
    }

    #[test]
    fn ford_eao_sohc_breathing_fixture_hits_broad_din_net_range() {
        let class = EngineBreathingClass::TwoValveSohcCarbLowCompression;
        let preset = class.preset();
        let peak = estimate_breathing_point(BreathingEstimateInput {
            displacement_cc: 1993,
            rpm: Rpm(preset.peak_ve_rpm as u32),
            afr_x100: 1323,
            fuel_lhv_j_per_kg: 43_000_000,
            eta_x1000: preset.expected_bte_max_x1000,
            efficiency_basis: EfficiencyBasis::BrakeThermal,
            friction_torque_nm_x100: TorqueNmX100(0),
            pumping_torque_nm_x100: TorqueNmX100(0),
            accessory_torque_nm_x100: TorqueNmX100(0),
            breathing_class: class,
        });
        let power = estimate_breathing_point(BreathingEstimateInput {
            rpm: Rpm(preset.power_peak_rpm as u32),
            ..BreathingEstimateInput {
                displacement_cc: 1993,
                rpm: Rpm(0),
                afr_x100: 1323,
                fuel_lhv_j_per_kg: 43_000_000,
                eta_x1000: preset.expected_bte_max_x1000,
                efficiency_basis: EfficiencyBasis::BrakeThermal,
                friction_torque_nm_x100: TorqueNmX100(0),
                pumping_torque_nm_x100: TorqueNmX100(0),
                accessory_torque_nm_x100: TorqueNmX100(0),
                breathing_class: class,
            }
        });
        let dohc_retention = EngineBreathingClass::FourValveDohcCamPhased
            .preset()
            .high_rpm_ve_retention_x1000;

        assert!((13_000..=14_500).contains(&peak.torque_nm_x100.0));
        assert!((5_800..=6_600).contains(&power.power_kw_x100));
        assert!(preset.high_rpm_ve_retention_x1000 < dohc_retention);
    }

    #[test]
    fn bmw_m50b25tu_cam_phased_fixture_retains_high_rpm_torque() {
        let class = EngineBreathingClass::FourValveDohcCamPhased;
        let preset = class.preset();
        let common = BreathingEstimateInput {
            displacement_cc: 2494,
            rpm: Rpm(0),
            afr_x100: 1294,
            fuel_lhv_j_per_kg: 43_000_000,
            eta_x1000: 310,
            efficiency_basis: EfficiencyBasis::BrakeThermal,
            friction_torque_nm_x100: TorqueNmX100(0),
            pumping_torque_nm_x100: TorqueNmX100(0),
            accessory_torque_nm_x100: TorqueNmX100(0),
            breathing_class: class,
        };
        let peak = estimate_breathing_point(BreathingEstimateInput {
            rpm: Rpm(preset.peak_ve_rpm as u32),
            ..common
        });
        let power = estimate_breathing_point(BreathingEstimateInput {
            rpm: Rpm(preset.power_peak_rpm as u32),
            ..common
        });
        let retention = torque_retention_x1000(power.torque_nm_x100, peak.torque_nm_x100);
        let fixed_retention = EngineBreathingClass::FourValveDohcFixedCam
            .preset()
            .high_rpm_ve_retention_x1000;

        assert!((24_000..=26_000).contains(&peak.torque_nm_x100.0));
        assert!((13_400..=14_800).contains(&power.power_kw_x100));
        assert!((880..=950).contains(&retention));
        assert!(preset.high_rpm_ve_retention_x1000 > fixed_retention);
    }

    fn m50tu_cam_model() -> SwitchedCamPhasingModel {
        SwitchedCamPhasingModel {
            advance_deg_x10: 250,
            enable_min_rpm: 1200,
            enable_max_rpm: 4300,
            transition_width_rpm: 400,
            min_tps_x1000: 700,
            low_rpm_gain_x1000: 1400,
            midrange_plateau_x1000: 930,
        }
    }

    fn m50tu_intake_model() -> IntakeTuningModel {
        IntakeTuningModel {
            resonance_center_rpm: 5000,
            resonance_width_rpm: 1500,
            resonance_gain_x1000: 15,
            low_speed_fill_gain_x1000: 30,
        }
    }

    fn m50tu_bte_model() -> BteShapeModel {
        BteShapeModel {
            min_bsfc_rpm: 3500,
            low_rpm_heat_loss_penalty_x1000: 995,
            high_rpm_friction_penalty_x1000: 995,
            max_midrange_gain_x1000: 1000,
        }
    }

    #[test]
    fn curve_shape_diagnostics_report_required_bmep_multiplier() {
        let diag = curve_shape_diagnostics(CurveShapeInput {
            rpm: Rpm(1500),
            reference_torque_nm_x100: TorqueNmX100(19_000),
            simulated_torque_nm_x100: TorqueNmX100(14_010),
            displacement_cc: 2494,
            ve_x1000: 720,
            bte_x1000: 245,
            fuel_energy_per_cycle: EnergyMicroJ(6_000_000_000),
            cam_switch_state: CamSwitchState::On,
            cam_switch_factor_x1000: 1300,
            intake_tuning_factor_x1000: 1012,
            bte_shape_factor_x1000: 997,
        });

        assert_eq!(diag.required_torque_multiplier_x1000, 1356);
        assert!((955..=960).contains(&diag.reference_bmep_bar_x100.0));
        assert!((704..=708).contains(&diag.simulated_bmep_bar_x100.0));
        assert_eq!(diag.required_bmep_multiplier_x1000, 1357);
        assert_eq!(diag.cam_switch_state, CamSwitchState::On);
    }

    #[test]
    fn switched_cam_shape_adds_low_rpm_and_limits_midrange_without_touching_high_rpm() {
        let cam = m50tu_cam_model();

        assert!(switched_cam_factor_x1000(cam, Rpm(1500), 1000) > 1250);
        assert!(switched_cam_factor_x1000(cam, Rpm(2000), 1000) > 1150);
        assert!((995..=1005).contains(&switched_cam_factor_x1000(cam, Rpm(2500), 1000)));
        assert!(switched_cam_factor_x1000(cam, Rpm(3500), 1000) < 950);
        assert!((995..=1005).contains(&switched_cam_factor_x1000(cam, Rpm(5000), 1000)));
        assert_eq!(switched_cam_factor_x1000(cam, Rpm(2000), 500), 1000);
    }

    #[test]
    fn m50tu_shape_fixture_hits_broad_technical_chart_ranges() {
        struct Sample {
            rpm: u32,
            sim_torque_nm_x100: i32,
            min_torque_nm_x100: i32,
            max_torque_nm_x100: i32,
        }

        let samples = [
            Sample {
                rpm: 1500,
                sim_torque_nm_x100: 14_010,
                min_torque_nm_x100: 17_500,
                max_torque_nm_x100: 20_500,
            },
            Sample {
                rpm: 2000,
                sim_torque_nm_x100: 17_160,
                min_torque_nm_x100: 18_500,
                max_torque_nm_x100: 21_500,
            },
            Sample {
                rpm: 2500,
                sim_torque_nm_x100: 20_410,
                min_torque_nm_x100: 19_500,
                max_torque_nm_x100: 21_500,
            },
            Sample {
                rpm: 3000,
                sim_torque_nm_x100: 22_770,
                min_torque_nm_x100: 20_000,
                max_torque_nm_x100: 22_000,
            },
            Sample {
                rpm: 3500,
                sim_torque_nm_x100: 24_570,
                min_torque_nm_x100: 21_500,
                max_torque_nm_x100: 23_500,
            },
            Sample {
                rpm: 4000,
                sim_torque_nm_x100: 25_390,
                min_torque_nm_x100: 23_000,
                max_torque_nm_x100: 25_000,
            },
            Sample {
                rpm: 4200,
                sim_torque_nm_x100: 25_270,
                min_torque_nm_x100: 23_800,
                max_torque_nm_x100: 25_200,
            },
            Sample {
                rpm: 4500,
                sim_torque_nm_x100: 25_100,
                min_torque_nm_x100: 23_500,
                max_torque_nm_x100: 25_000,
            },
            Sample {
                rpm: 5000,
                sim_torque_nm_x100: 24_180,
                min_torque_nm_x100: 23_500,
                max_torque_nm_x100: 25_000,
            },
            Sample {
                rpm: 5500,
                sim_torque_nm_x100: 23_000,
                min_torque_nm_x100: 22_500,
                max_torque_nm_x100: 24_000,
            },
            Sample {
                rpm: 5900,
                sim_torque_nm_x100: 21_710,
                min_torque_nm_x100: 21_000,
                max_torque_nm_x100: 22_500,
            },
            Sample {
                rpm: 6000,
                sim_torque_nm_x100: 21_390,
                min_torque_nm_x100: 20_500,
                max_torque_nm_x100: 22_000,
            },
            Sample {
                rpm: 6500,
                sim_torque_nm_x100: 18_980,
                min_torque_nm_x100: 17_000,
                max_torque_nm_x100: 19_500,
            },
        ];

        let cam = m50tu_cam_model();
        let intake = m50tu_intake_model();
        let bte = m50tu_bte_model();
        let mut peak_torque = TorqueNmX100(0);
        let mut peak_power_kw_x100 = 0;

        for sample in samples {
            let point = shaped_torque_point(ShapedTorqueInput {
                torque_nm_x100: TorqueNmX100(sample.sim_torque_nm_x100),
                rpm: Rpm(sample.rpm),
                tps_x1000: 1000,
                displacement_cc: 2494,
                cam,
                intake,
                bte,
            });

            assert!(
                (sample.min_torque_nm_x100..=sample.max_torque_nm_x100)
                    .contains(&point.torque_nm_x100.0),
                "rpm {} torque {} outside {}..={}",
                sample.rpm,
                point.torque_nm_x100.0,
                sample.min_torque_nm_x100,
                sample.max_torque_nm_x100
            );
            peak_torque = peak_torque.max(point.torque_nm_x100);
            peak_power_kw_x100 = peak_power_kw_x100.max(point.power_kw_x100);
        }

        assert!((23_800..=25_200).contains(&peak_torque.0));
        assert!((13_400..=14_800).contains(&peak_power_kw_x100));
    }

    #[test]
    fn m50tu_specific_shape_does_not_modify_generic_cam_phased_preset() {
        let before = EngineBreathingClass::FourValveDohcCamPhased.preset();
        let _ = shaped_torque_point(ShapedTorqueInput {
            torque_nm_x100: TorqueNmX100(25_000),
            rpm: Rpm(3500),
            tps_x1000: 1000,
            displacement_cc: 2494,
            cam: m50tu_cam_model(),
            intake: m50tu_intake_model(),
            bte: m50tu_bte_model(),
        });
        let after = EngineBreathingClass::FourValveDohcCamPhased.preset();

        assert_eq!(before, after);
    }

    fn din_net_reference() -> ReferencePowerMetadata {
        ReferencePowerMetadata {
            rating_basis: RatingBasis::DINNet,
            correction_standard: CorrectionStandard::DIN70020,
            measured_at: MeasurementLocation::EngineDynoCrank,
            intake_system: TestIntakeSystem::Production,
            exhaust_system: TestExhaustSystem::Production,
            accessories: AccessorySet::Production,
            ambient_temp_k_x10: 2930,
            ambient_pressure_pa: 101_325,
            humidity_x1000: 0,
            fuel: FuelSpec {
                stoich_afr_x100: 1470,
                lower_heating_value_j_per_kg: 43_000_000,
                octane_x10: 950,
            },
        }
    }

    #[test]
    fn rating_basis_guard_rejects_gross_as_din_net() {
        let mut reference = din_net_reference();
        reference.rating_basis = RatingBasis::SAEGross;

        assert_eq!(
            validate_reference_comparison(RatingBasis::DINNet, reference),
            Err(ReferenceComparisonError::RatingBasisMismatch)
        );
    }

    #[test]
    fn rating_basis_guard_rejects_ambiguous_catalog_claims() {
        let mut reference = din_net_reference();
        reference.measured_at = MeasurementLocation::CatalogClaim;

        assert_eq!(
            validate_reference_comparison(RatingBasis::DINNet, reference),
            Err(ReferenceComparisonError::AmbiguousReference)
        );
    }

    #[test]
    fn brake_vs_indicated_loss_guard_keeps_brake_mep_from_subtracting_losses() {
        let brake = mep_breakdown_for_efficiency_basis(
            EfficiencyBasis::BrakeThermal,
            ImepBarX100(1000),
            FmepBarX100(120),
            PmepBarX100(40),
            BmepBarX100(20),
        );
        let indicated = mep_breakdown_for_efficiency_basis(
            EfficiencyBasis::IndicatedThermal,
            ImepBarX100(1000),
            FmepBarX100(120),
            PmepBarX100(40),
            BmepBarX100(20),
        );

        assert_eq!(brake.bmep_bar_x100, BmepBarX100(1000));
        assert_eq!(indicated.bmep_bar_x100, BmepBarX100(820));
    }
}
