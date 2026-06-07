use ecu_board_api::{
    FullEcuOutputProfile, IgnitionOutputProfile, InjectionOutputProfile, RuntimeOutputProfile,
};

mod build;
mod matching;
mod model;
mod validation;

#[cfg(test)]
mod tests;

pub use build::{
    BoardBuildMetadata, BoardFeatureBindings, BoardId, BuildArtifact, BuildInvocationError,
    CargoFeatureSet, FeatureBinding, FirmwareBuildInvocation, FirmwareBuildMode, FirmwareBuildPlan,
    FirstRunChecklist, FirstRunChecklistItem, RecipeFeature, TunerStudioProfileSelection,
    MAX_CARGO_FEATURES, MAX_FIRST_RUN_CHECKLIST_ITEMS,
};
pub use model::{
    BoardSelection, InputSourceSet, OutputTopology, RuntimeProfile, SafetyPolicy, SubsystemSet,
};
pub use validation::RecipeValidationError;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct FirmwareRecipe {
    pub name: &'static str,
    pub board: BoardSelection,
    pub runtime: RuntimeProfile,
    pub subsystems: SubsystemSet,
    pub inputs: InputSourceSet,
    pub outputs: OutputTopology,
    pub safety: SafetyPolicy,
    pub ts_profile: Option<&'static str>,
}

impl FirmwareRecipe {
    pub const fn rev_limiter() -> Self {
        Self {
            name: "rev_limiter",
            board: BoardSelection::AnyCompatible,
            runtime: RuntimeProfile::rev_limiter(),
            subsystems: SubsystemSet::rev_limiter(),
            inputs: InputSourceSet::rpm_only(),
            outputs: OutputTopology::ignition_cut(1),
            safety: SafetyPolicy::watchdog_required(),
            ts_profile: None,
        }
    }

    pub const fn ignition_only_wasted_spark(cylinder_count: u8) -> Self {
        Self {
            name: "ignition_only_wasted_spark",
            board: BoardSelection::AnyCompatible,
            runtime: RuntimeProfile::new(RuntimeOutputProfile::crank_only_wasted_spark(
                cylinder_count,
            )),
            subsystems: SubsystemSet::ignition_only(),
            inputs: InputSourceSet::crank_rpm(),
            outputs: OutputTopology::ignition_channels(wasted_spark_channels(cylinder_count)),
            safety: SafetyPolicy::watchdog_required(),
            ts_profile: None,
        }
    }

    pub const fn injection_only_single_point() -> Self {
        Self {
            name: "injection_only_single_point",
            board: BoardSelection::AnyCompatible,
            runtime: RuntimeProfile::new(RuntimeOutputProfile::single_point_injection()),
            subsystems: SubsystemSet::injection_only(),
            inputs: InputSourceSet::rpm_only(),
            outputs: OutputTopology::injector_channels(1),
            safety: SafetyPolicy::watchdog_required(),
            ts_profile: None,
        }
    }

    pub const fn injection_only_batch(channel_count: u8) -> Self {
        Self {
            name: "injection_only_batch",
            board: BoardSelection::AnyCompatible,
            runtime: RuntimeProfile::new(RuntimeOutputProfile::batch_injection(channel_count)),
            subsystems: SubsystemSet::injection_only(),
            inputs: InputSourceSet::rpm_only(),
            outputs: OutputTopology::injector_channels(channel_count),
            safety: SafetyPolicy::watchdog_required(),
            ts_profile: None,
        }
    }

    pub const fn full_ecu(output_profile: FullEcuOutputProfile) -> Self {
        Self {
            name: "full_ecu",
            board: BoardSelection::AnyCompatible,
            runtime: RuntimeProfile::new(RuntimeOutputProfile::full_ecu(output_profile)),
            subsystems: SubsystemSet::full_ecu(),
            inputs: InputSourceSet::crank_cam_load(),
            outputs: OutputTopology::full_ecu(
                full_ecu_ignition_channels(output_profile),
                full_ecu_injector_channels(output_profile),
                full_ecu_aux_channels(output_profile),
            ),
            safety: SafetyPolicy::watchdog_required(),
            ts_profile: None,
        }
    }

    pub const fn for_board(mut self, board: BoardSelection) -> Self {
        self.board = board;
        self
    }
}

const fn wasted_spark_channels(cylinder_count: u8) -> u8 {
    let channels = cylinder_count / 2;
    if channels == 0 {
        1
    } else if channels > 8 {
        8
    } else {
        channels
    }
}

const fn full_ecu_ignition_channels(profile: FullEcuOutputProfile) -> u8 {
    match profile.ignition {
        IgnitionOutputProfile::WastedSpark { coils, .. }
        | IgnitionOutputProfile::CoilOnPlug { coils, .. } => coils,
    }
}

const fn full_ecu_injector_channels(profile: FullEcuOutputProfile) -> u8 {
    match profile.injection {
        InjectionOutputProfile::Sequential { channels, .. } => channels,
    }
}

const fn full_ecu_aux_channels(profile: FullEcuOutputProfile) -> u8 {
    let mut count = 0;
    let mut idx = 0;
    while idx < profile.aux_safety.off_on_limp.len() {
        if profile.aux_safety.off_on_limp[idx].is_some() {
            count += 1;
        }
        idx += 1;
    }
    count
}
