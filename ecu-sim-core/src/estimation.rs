use crate::{losses::brake_torque_from_indicated, pressure::bmep_bar_x100, types::*};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum EfficiencyBasis {
    BrakeThermal,
    IndicatedThermal,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum RatingBasis {
    DINNet,
    SAENet,
    SAEGross,
    ISO1585Net,
    ChassisWheel,
    ChassisEstimatedCrank,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CorrectionStandard {
    None,
    SAEJ1349,
    SAEJ1995,
    DIN70020,
    ISO1585,
    EWG80_1269,
    Unknown,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum MeasurementLocation {
    EngineDynoCrank,
    ChassisWheel,
    ChassisEstimatedCrank,
    CatalogClaim,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum TestIntakeSystem {
    Production,
    OpenElement,
    BenchReference,
    Unknown,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum TestExhaustSystem {
    Production,
    OpenHeader,
    BenchReference,
    Unknown,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum AccessorySet {
    Production,
    Partial,
    None,
    Unknown,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct FuelSpec {
    pub stoich_afr_x100: u16,
    pub lower_heating_value_j_per_kg: u32,
    pub octane_x10: u16,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ReferencePowerMetadata {
    pub rating_basis: RatingBasis,
    pub correction_standard: CorrectionStandard,
    pub measured_at: MeasurementLocation,
    pub intake_system: TestIntakeSystem,
    pub exhaust_system: TestExhaustSystem,
    pub accessories: AccessorySet,
    pub ambient_temp_k_x10: u16,
    pub ambient_pressure_pa: u32,
    pub humidity_x1000: u16,
    pub fuel: FuelSpec,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ReferenceComparisonError {
    RatingBasisMismatch,
    AmbiguousReference,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum EngineBreathingClass {
    TwoValvePushrodLowCompression,
    TwoValveSohcCarbLowCompression,
    TwoValveSohcInjection,
    FourValveDohcFixedCam,
    FourValveDohcCamPhased,
    FourValveDohcHighSpecificOutput,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct BreathingPreset {
    pub peak_ve_x1000: u16,
    pub peak_ve_rpm: u16,
    pub power_peak_rpm: u16,
    pub high_rpm_ve_retention_x1000: u16,
    pub overrev_falloff_x1000: u16,
    pub expected_peak_bmep_bar_x100: u16,
    pub expected_bte_min_x1000: u16,
    pub expected_bte_max_x1000: u16,
    pub default_rating_basis: RatingBasis,
    pub expects_cam_phasing: bool,
}

impl EngineBreathingClass {
    pub const fn preset(self) -> BreathingPreset {
        match self {
            Self::TwoValvePushrodLowCompression => BreathingPreset {
                peak_ve_x1000: 780,
                peak_ve_rpm: 3200,
                power_peak_rpm: 4200,
                high_rpm_ve_retention_x1000: 650,
                overrev_falloff_x1000: 560,
                expected_peak_bmep_bar_x100: 760,
                expected_bte_min_x1000: 200,
                expected_bte_max_x1000: 270,
                default_rating_basis: RatingBasis::SAEGross,
                expects_cam_phasing: false,
            },
            Self::TwoValveSohcCarbLowCompression => BreathingPreset {
                peak_ve_x1000: 849,
                peak_ve_rpm: 3600,
                power_peak_rpm: 4400,
                high_rpm_ve_retention_x1000: 740,
                overrev_falloff_x1000: 610,
                expected_peak_bmep_bar_x100: 880,
                expected_bte_min_x1000: 220,
                expected_bte_max_x1000: 275,
                default_rating_basis: RatingBasis::DINNet,
                expects_cam_phasing: false,
            },
            Self::TwoValveSohcInjection => BreathingPreset {
                peak_ve_x1000: 900,
                peak_ve_rpm: 4200,
                power_peak_rpm: 5200,
                high_rpm_ve_retention_x1000: 760,
                overrev_falloff_x1000: 650,
                expected_peak_bmep_bar_x100: 900,
                expected_bte_min_x1000: 235,
                expected_bte_max_x1000: 300,
                default_rating_basis: RatingBasis::DINNet,
                expects_cam_phasing: false,
            },
            Self::FourValveDohcFixedCam => BreathingPreset {
                peak_ve_x1000: 980,
                peak_ve_rpm: 4700,
                power_peak_rpm: 5900,
                high_rpm_ve_retention_x1000: 860,
                overrev_falloff_x1000: 760,
                expected_peak_bmep_bar_x100: 1010,
                expected_bte_min_x1000: 255,
                expected_bte_max_x1000: 325,
                default_rating_basis: RatingBasis::DINNet,
                expects_cam_phasing: false,
            },
            Self::FourValveDohcCamPhased => BreathingPreset {
                peak_ve_x1000: 1000,
                peak_ve_rpm: 4200,
                power_peak_rpm: 5900,
                high_rpm_ve_retention_x1000: 930,
                overrev_falloff_x1000: 850,
                expected_peak_bmep_bar_x100: 1160,
                expected_bte_min_x1000: 270,
                expected_bte_max_x1000: 340,
                default_rating_basis: RatingBasis::DINNet,
                expects_cam_phasing: true,
            },
            Self::FourValveDohcHighSpecificOutput => BreathingPreset {
                peak_ve_x1000: 1120,
                peak_ve_rpm: 6500,
                power_peak_rpm: 7600,
                high_rpm_ve_retention_x1000: 980,
                overrev_falloff_x1000: 900,
                expected_peak_bmep_bar_x100: 1220,
                expected_bte_min_x1000: 280,
                expected_bte_max_x1000: 360,
                default_rating_basis: RatingBasis::DINNet,
                expects_cam_phasing: true,
            },
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct EnergyTorqueInput {
    pub fuel_energy_per_cycle: EnergyMicroJ,
    pub eta_x1000: u16,
    pub efficiency_basis: EfficiencyBasis,
    pub friction_torque_nm_x100: TorqueNmX100,
    pub pumping_torque_nm_x100: TorqueNmX100,
    pub accessory_torque_nm_x100: TorqueNmX100,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct LossBreakdown {
    pub fmep_bar_x100: FmepBarX100,
    pub pmep_bar_x100: PmepBarX100,
    pub amep_bar_x100: BmepBarX100,
    pub friction_torque_nm_x100: TorqueNmX100,
    pub pumping_torque_nm_x100: TorqueNmX100,
    pub accessory_torque_nm_x100: TorqueNmX100,
    pub total_loss_torque_nm_x100: TorqueNmX100,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct MepBreakdown {
    pub imep_bar_x100: ImepBarX100,
    pub fmep_bar_x100: FmepBarX100,
    pub pmep_bar_x100: PmepBarX100,
    pub amep_bar_x100: BmepBarX100,
    pub bmep_bar_x100: BmepBarX100,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct TorqueBacksolve {
    pub rpm: Rpm,
    pub target_torque_nm_x100: TorqueNmX100,
    pub simulated_torque_nm_x100: TorqueNmX100,
    pub air_mass_per_cycle_ug: MassUg,
    pub fuel_mass_per_cycle_ug: MassUg,
    pub fuel_energy_per_cycle: EnergyMicroJ,
    pub eta_bte_required_x1000: u16,
    pub eta_ite_required_x1000: u16,
    pub bmep_target_bar_x100: BmepBarX100,
    pub bmep_sim_bar_x100: BmepBarX100,
    pub loss_torque_nm_x100: TorqueNmX100,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct BreathingEstimateInput {
    pub displacement_cc: u32,
    pub rpm: Rpm,
    pub afr_x100: u16,
    pub fuel_lhv_j_per_kg: u32,
    pub eta_x1000: u16,
    pub efficiency_basis: EfficiencyBasis,
    pub friction_torque_nm_x100: TorqueNmX100,
    pub pumping_torque_nm_x100: TorqueNmX100,
    pub accessory_torque_nm_x100: TorqueNmX100,
    pub breathing_class: EngineBreathingClass,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct BreathingEstimatePoint {
    pub rpm: Rpm,
    pub ve_x1000: u16,
    pub torque_nm_x100: TorqueNmX100,
    pub power_kw_x100: i32,
    pub bmep_bar_x100: BmepBarX100,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CamSwitchState {
    Off,
    On,
    Transition,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct SwitchedCamPhasingModel {
    pub advance_deg_x10: i16,
    pub enable_min_rpm: u16,
    pub enable_max_rpm: u16,
    pub transition_width_rpm: u16,
    pub min_tps_x1000: u16,
    pub low_rpm_gain_x1000: u16,
    pub midrange_plateau_x1000: u16,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct IntakeTuningModel {
    pub resonance_center_rpm: u16,
    pub resonance_width_rpm: u16,
    pub resonance_gain_x1000: u16,
    pub low_speed_fill_gain_x1000: u16,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct BteShapeModel {
    pub min_bsfc_rpm: u16,
    pub low_rpm_heat_loss_penalty_x1000: u16,
    pub high_rpm_friction_penalty_x1000: u16,
    pub max_midrange_gain_x1000: u16,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct CurveShapeInput {
    pub rpm: Rpm,
    pub reference_torque_nm_x100: TorqueNmX100,
    pub simulated_torque_nm_x100: TorqueNmX100,
    pub displacement_cc: u32,
    pub ve_x1000: u16,
    pub bte_x1000: u16,
    pub fuel_energy_per_cycle: EnergyMicroJ,
    pub cam_switch_state: CamSwitchState,
    pub vanos_factor_x1000: u16,
    pub intake_tuning_factor_x1000: u16,
    pub bte_shape_factor_x1000: u16,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct CurveShapeDiagnostics {
    pub rpm: Rpm,
    pub reference_torque_nm_x100: TorqueNmX100,
    pub simulated_torque_nm_x100: TorqueNmX100,
    pub reference_bmep_bar_x100: BmepBarX100,
    pub simulated_bmep_bar_x100: BmepBarX100,
    pub required_torque_multiplier_x1000: u16,
    pub required_bmep_multiplier_x1000: u16,
    pub ve_x1000: u16,
    pub bte_x1000: u16,
    pub fuel_mep_bar_x100: BmepBarX100,
    pub cam_switch_state: CamSwitchState,
    pub vanos_factor_x1000: u16,
    pub intake_tuning_factor_x1000: u16,
    pub bte_shape_factor_x1000: u16,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ShapedTorqueInput {
    pub torque_nm_x100: TorqueNmX100,
    pub rpm: Rpm,
    pub tps_x1000: u16,
    pub displacement_cc: u32,
    pub cam: SwitchedCamPhasingModel,
    pub intake: IntakeTuningModel,
    pub bte: BteShapeModel,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ShapedTorquePoint {
    pub rpm: Rpm,
    pub torque_nm_x100: TorqueNmX100,
    pub power_kw_x100: i32,
    pub bmep_bar_x100: BmepBarX100,
    pub cam_switch_state: CamSwitchState,
    pub vanos_factor_x1000: u16,
    pub intake_tuning_factor_x1000: u16,
    pub bte_shape_factor_x1000: u16,
}

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

pub fn cam_switch_state(
    model: SwitchedCamPhasingModel,
    rpm: Rpm,
    tps_x1000: u16,
) -> CamSwitchState {
    if tps_x1000 < model.min_tps_x1000 || rpm.0 < model.enable_min_rpm as u32 {
        return CamSwitchState::Off;
    }

    let transition = model.transition_width_rpm.max(1) as u32;
    let enable_min = model.enable_min_rpm as u32;
    let enable_max = model.enable_max_rpm as u32;
    if rpm.0 < enable_min.saturating_add(transition)
        || rpm.0 > enable_max.saturating_sub(transition)
    {
        if rpm.0 <= enable_max.saturating_add(transition) {
            return CamSwitchState::Transition;
        }
        return CamSwitchState::Off;
    }

    CamSwitchState::On
}

pub fn switched_cam_factor_x1000(model: SwitchedCamPhasingModel, rpm: Rpm, tps_x1000: u16) -> u16 {
    if tps_x1000 < model.min_tps_x1000 || rpm.0 < model.enable_min_rpm as u32 {
        return 1000;
    }

    let transition = model.transition_width_rpm.max(1) as u32;
    let enable_min = model.enable_min_rpm as u32;
    let enable_max = model.enable_max_rpm as u32;
    let low_full_rpm = enable_min.saturating_add(transition);
    let low_neutral_rpm = low_full_rpm.saturating_add(900);
    if rpm.0 < low_full_rpm {
        return interp_u16(
            1000,
            model.low_rpm_gain_x1000,
            rpm.0.saturating_sub(enable_min),
            transition,
        );
    }
    if rpm.0 <= low_neutral_rpm {
        return interp_u16(
            model.low_rpm_gain_x1000,
            1000,
            rpm.0.saturating_sub(low_full_rpm),
            low_neutral_rpm.saturating_sub(low_full_rpm).max(1),
        );
    }

    let plateau_start = low_neutral_rpm.saturating_add(500);
    if rpm.0 < plateau_start {
        return 1000;
    }

    let plateau_end = enable_max.saturating_sub(transition);
    if rpm.0 <= plateau_end {
        return model.midrange_plateau_x1000;
    }

    let off_rpm = enable_max.saturating_add(transition);
    if rpm.0 <= off_rpm {
        return interp_u16(
            model.midrange_plateau_x1000,
            1000,
            rpm.0.saturating_sub(plateau_end),
            off_rpm.saturating_sub(plateau_end).max(1),
        );
    }

    1000
}

pub fn intake_tuning_factor_x1000(model: IntakeTuningModel, rpm: Rpm) -> u16 {
    let low_end = 2500u32;
    let low_extra = if rpm.0 < low_end {
        model.low_speed_fill_gain_x1000 as u32 * (low_end - rpm.0) / low_end
    } else {
        0
    };

    let center = model.resonance_center_rpm as i32;
    let width = model.resonance_width_rpm.max(1) as i32;
    let distance = (rpm.0 as i32 - center).abs();
    let resonance_extra = if distance < width {
        model.resonance_gain_x1000 as i32 * (width - distance) / width
    } else {
        0
    }
    .max(0) as u32;

    (1000u32 + low_extra + resonance_extra).min(u16::MAX as u32) as u16
}

pub fn bte_shape_factor_x1000(model: BteShapeModel, rpm: Rpm) -> u16 {
    let min_bsfc = model.min_bsfc_rpm.max(1) as u32;
    let mid_cap = model.max_midrange_gain_x1000.max(1);
    if rpm.0 <= min_bsfc {
        return interp_u16(
            model.low_rpm_heat_loss_penalty_x1000,
            mid_cap,
            rpm.0,
            min_bsfc,
        );
    }

    let high_end = min_bsfc.saturating_add(3000);
    if rpm.0 >= high_end {
        return model.high_rpm_friction_penalty_x1000;
    }

    interp_u16(
        mid_cap,
        model.high_rpm_friction_penalty_x1000,
        rpm.0.saturating_sub(min_bsfc),
        high_end.saturating_sub(min_bsfc).max(1),
    )
}

pub fn apply_shape_factor_x1000(torque: TorqueNmX100, factor_x1000: u16) -> TorqueNmX100 {
    TorqueNmX100(
        (torque.0 as i64 * factor_x1000 as i64 / 1000).clamp(i32::MIN as i64, i32::MAX as i64)
            as i32,
    )
}

pub fn shaped_torque_point(input: ShapedTorqueInput) -> ShapedTorquePoint {
    let vanos_factor = switched_cam_factor_x1000(input.cam, input.rpm, input.tps_x1000);
    let intake_factor = intake_tuning_factor_x1000(input.intake, input.rpm);
    let bte_factor = bte_shape_factor_x1000(input.bte, input.rpm);
    let combined = (vanos_factor as u64)
        .saturating_mul(intake_factor as u64)
        .saturating_mul(bte_factor as u64)
        / 1_000_000;
    let torque =
        apply_shape_factor_x1000(input.torque_nm_x100, combined.min(u16::MAX as u64) as u16);

    ShapedTorquePoint {
        rpm: input.rpm,
        torque_nm_x100: torque,
        power_kw_x100: power_kw_x100(torque, input.rpm),
        bmep_bar_x100: bmep_bar_x100(torque, input.displacement_cc),
        cam_switch_state: cam_switch_state(input.cam, input.rpm, input.tps_x1000),
        vanos_factor_x1000: vanos_factor,
        intake_tuning_factor_x1000: intake_factor,
        bte_shape_factor_x1000: bte_factor,
    }
}

pub fn curve_shape_diagnostics(input: CurveShapeInput) -> CurveShapeDiagnostics {
    let reference_bmep = bmep_bar_x100(input.reference_torque_nm_x100, input.displacement_cc);
    let simulated_bmep = bmep_bar_x100(input.simulated_torque_nm_x100, input.displacement_cc);
    let fuel_mep = bmep_bar_x100(
        torque_from_cycle_energy(input.fuel_energy_per_cycle, 1000),
        input.displacement_cc,
    );

    CurveShapeDiagnostics {
        rpm: input.rpm,
        reference_torque_nm_x100: input.reference_torque_nm_x100,
        simulated_torque_nm_x100: input.simulated_torque_nm_x100,
        reference_bmep_bar_x100: reference_bmep,
        simulated_bmep_bar_x100: simulated_bmep,
        required_torque_multiplier_x1000: required_multiplier_x1000(
            input.reference_torque_nm_x100.0,
            input.simulated_torque_nm_x100.0,
        ),
        required_bmep_multiplier_x1000: required_multiplier_x1000(
            reference_bmep.0,
            simulated_bmep.0,
        ),
        ve_x1000: input.ve_x1000,
        bte_x1000: input.bte_x1000,
        fuel_mep_bar_x100: fuel_mep,
        cam_switch_state: input.cam_switch_state,
        vanos_factor_x1000: input.vanos_factor_x1000,
        intake_tuning_factor_x1000: input.intake_tuning_factor_x1000,
        bte_shape_factor_x1000: input.bte_shape_factor_x1000,
    }
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

fn power_kw_x100(torque: TorqueNmX100, rpm: Rpm) -> i32 {
    if torque.0 <= 0 || rpm.0 == 0 {
        return 0;
    }
    (torque.0 as i64 * rpm.0 as i64 / 9549) as i32
}

fn interp_u16(start: u16, end: u16, travel: u32, span: u32) -> u16 {
    if span == 0 {
        return end;
    }
    let travel = travel.min(span) as i64;
    let span = span as i64;
    let start = start as i64;
    let end = end as i64;
    (start + (end - start) * travel / span).clamp(0, u16::MAX as i64) as u16
}

fn required_multiplier_x1000(reference: i32, simulated: i32) -> u16 {
    if reference <= 0 || simulated <= 0 {
        return 0;
    }
    (reference as i64 * 1000 / simulated as i64).clamp(0, u16::MAX as i64) as u16
}

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

#[cfg(test)]
mod tests {
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
            vanos_factor_x1000: 1300,
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
