use crate::{estimation::types::*, pressure::bmep_bar_x100, types::*};

use super::energy::brake_torque_from_energy;
use super::shape::power_kw_x100;

pub fn torque_retention_x1000(
    torque_at_power_peak: TorqueNmX100,
    peak_torque: TorqueNmX100,
) -> u16 {
    if torque_at_power_peak.0 <= 0 || peak_torque.0 <= 0 {
        return 0;
    }
    ((torque_at_power_peak.0 as i64 * 1000) / peak_torque.0 as i64).clamp(0, u16::MAX as i64) as u16
}

pub fn estimate_breathing_point(input: BreathingEstimateInput) -> BreathingEstimatePoint {
    let preset = input.breathing_class.preset();
    let ve_x1000 = ve_for_breathing_preset(preset, input.rpm);
    let air_mass = air_mass_for_standard_conditions_ug(input.displacement_cc, ve_x1000);
    let fuel_mass = if input.afr_x100 == 0 {
        MassUg(0)
    } else {
        MassUg((air_mass.0 as u64 * 100 / input.afr_x100 as u64).min(u32::MAX as u64) as u32)
    };
    let energy = EnergyMicroJ(
        (fuel_mass.0 as u128 * input.fuel_lhv_j_per_kg as u128 / 1000).min(i64::MAX as u128) as i64,
    );
    let torque = brake_torque_from_energy(EnergyTorqueInput {
        fuel_energy_per_cycle: energy,
        eta_x1000: input.eta_x1000,
        efficiency_basis: input.efficiency_basis,
        friction_torque_nm_x100: input.friction_torque_nm_x100,
        pumping_torque_nm_x100: input.pumping_torque_nm_x100,
        accessory_torque_nm_x100: input.accessory_torque_nm_x100,
    });

    BreathingEstimatePoint {
        rpm: input.rpm,
        ve_x1000,
        torque_nm_x100: torque,
        power_kw_x100: power_kw_x100(torque, input.rpm),
        bmep_bar_x100: bmep_bar_x100(torque, input.displacement_cc),
    }
}

pub fn ve_for_breathing_preset(preset: BreathingPreset, rpm: Rpm) -> u16 {
    let rpm = rpm.0.min(u16::MAX as u32) as u16;
    let idle_ve = preset.peak_ve_x1000 as u32 * 700 / 1000;
    if rpm <= preset.peak_ve_rpm {
        let span = preset.peak_ve_rpm.max(1) as u32;
        let rise = preset.peak_ve_x1000 as u32 - idle_ve;
        return (idle_ve + rise * rpm as u32 / span) as u16;
    }

    let power_retention = power_peak_retention_x1000(preset) as u32;
    let power_ve = preset.peak_ve_x1000 as u32 * power_retention / 1000;
    if rpm <= preset.power_peak_rpm {
        let span = preset
            .power_peak_rpm
            .saturating_sub(preset.peak_ve_rpm)
            .max(1) as u32;
        let travel = rpm.saturating_sub(preset.peak_ve_rpm) as u32;
        let drop = preset.peak_ve_x1000 as u32 - power_ve;
        return (preset.peak_ve_x1000 as u32 - drop * travel / span) as u16;
    }

    let overrev_ve = preset.peak_ve_x1000 as u32 * preset.overrev_falloff_x1000 as u32 / 1000;
    let span = 1000u32;
    let travel = (rpm - preset.power_peak_rpm).min(1000) as u32;
    let drop = power_ve.saturating_sub(overrev_ve);
    (power_ve - drop * travel / span) as u16
}

fn power_peak_retention_x1000(preset: BreathingPreset) -> u16 {
    if preset.expects_cam_phasing {
        preset.high_rpm_ve_retention_x1000
    } else {
        ((1000u32 + preset.high_rpm_ve_retention_x1000 as u32) / 2) as u16
    }
}

fn air_mass_for_standard_conditions_ug(displacement_cc: u32, ve_x1000: u16) -> MassUg {
    // Dry air at 101.325 kPa and 20 C is about 1.204 kg/m3.
    MassUg(
        (displacement_cc as u128 * 1204u128 * ve_x1000 as u128 / 1000).min(u32::MAX as u128) as u32,
    )
}
