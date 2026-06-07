use crate::types::*;

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
    pub cam_switch_factor_x1000: u16,
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
    pub cam_switch_factor_x1000: u16,
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
    pub cam_switch_factor_x1000: u16,
    pub intake_tuning_factor_x1000: u16,
    pub bte_shape_factor_x1000: u16,
}
