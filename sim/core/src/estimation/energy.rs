use crate::{
    estimation::types::*, losses::brake_torque_from_indicated, pressure::bmep_bar_x100, types::*,
};

pub fn brake_torque_from_energy(input: EnergyTorqueInput) -> TorqueNmX100 {
    let base = torque_from_cycle_energy(input.fuel_energy_per_cycle, input.eta_x1000);
    match input.efficiency_basis {
        EfficiencyBasis::BrakeThermal => base,
        EfficiencyBasis::IndicatedThermal => brake_torque_from_indicated(
            base,
            input.friction_torque_nm_x100,
            input.pumping_torque_nm_x100,
            input.accessory_torque_nm_x100,
        ),
    }
}

pub fn validate_reference_comparison(
    model_basis: RatingBasis,
    reference: ReferencePowerMetadata,
) -> Result<(), ReferenceComparisonError> {
    if reference.rating_basis != model_basis {
        return Err(ReferenceComparisonError::RatingBasisMismatch);
    }
    if reference.correction_standard == CorrectionStandard::Unknown
        || reference.measured_at == MeasurementLocation::CatalogClaim
        || reference.intake_system == TestIntakeSystem::Unknown
        || reference.exhaust_system == TestExhaustSystem::Unknown
        || reference.accessories == AccessorySet::Unknown
    {
        return Err(ReferenceComparisonError::AmbiguousReference);
    }
    Ok(())
}

pub fn mep_breakdown_for_efficiency_basis(
    basis: EfficiencyBasis,
    imep_bar_x100: ImepBarX100,
    fmep_bar_x100: FmepBarX100,
    pmep_bar_x100: PmepBarX100,
    amep_bar_x100: BmepBarX100,
) -> MepBreakdown {
    let total_loss = fmep_bar_x100
        .0
        .saturating_add(pmep_bar_x100.0)
        .saturating_add(amep_bar_x100.0);
    let bmep = match basis {
        EfficiencyBasis::BrakeThermal => imep_bar_x100.0,
        EfficiencyBasis::IndicatedThermal => imep_bar_x100.0.saturating_sub(total_loss),
    };

    MepBreakdown {
        imep_bar_x100,
        fmep_bar_x100,
        pmep_bar_x100,
        amep_bar_x100,
        bmep_bar_x100: BmepBarX100(bmep),
    }
}

pub fn torque_from_cycle_energy(energy: EnergyMicroJ, eta_x1000: u16) -> TorqueNmX100 {
    if energy.0 <= 0 || eta_x1000 == 0 {
        return TorqueNmX100(0);
    }
    // torque_Nm_x100 = energy_uJ * eta/1000 / 1e6 * 100 / (4*pi).
    // 1/(4*pi) is approximated by 113/1420.
    let numerator = energy.0 as i128 * eta_x1000 as i128 * 113;
    let denominator = 14_200_000_000i128;
    TorqueNmX100((numerator / denominator).clamp(i32::MIN as i128, i32::MAX as i128) as i32)
}

pub fn eta_required_x1000(torque: TorqueNmX100, energy: EnergyMicroJ) -> u16 {
    if torque.0 <= 0 || energy.0 <= 0 {
        return 0;
    }
    // eta_x1000 = torque_Nm * 4*pi * 1000 / energy_J.
    let numerator = torque.0 as i128 * 14_200_000_000i128;
    let denominator = energy.0 as i128 * 113;
    let value = numerator / denominator;
    value.clamp(0, u16::MAX as i128) as u16
}

#[allow(clippy::too_many_arguments)]
pub fn torque_backsolve(
    rpm: Rpm,
    target_torque: TorqueNmX100,
    simulated_torque: TorqueNmX100,
    air_mass_per_cycle: MassUg,
    fuel_mass_per_cycle: MassUg,
    fuel_energy_per_cycle: EnergyMicroJ,
    loss_torque: TorqueNmX100,
    displacement_cc: u32,
) -> TorqueBacksolve {
    TorqueBacksolve {
        rpm,
        target_torque_nm_x100: target_torque,
        simulated_torque_nm_x100: simulated_torque,
        air_mass_per_cycle_ug: air_mass_per_cycle,
        fuel_mass_per_cycle_ug: fuel_mass_per_cycle,
        fuel_energy_per_cycle,
        eta_bte_required_x1000: eta_required_x1000(target_torque, fuel_energy_per_cycle),
        eta_ite_required_x1000: eta_required_x1000(
            TorqueNmX100(target_torque.0.saturating_add(loss_torque.0)),
            fuel_energy_per_cycle,
        ),
        bmep_target_bar_x100: bmep_bar_x100(target_torque, displacement_cc),
        bmep_sim_bar_x100: bmep_bar_x100(simulated_torque, displacement_cc),
        loss_torque_nm_x100: loss_torque,
    }
}
