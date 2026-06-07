use ecu_board_api::{BoardCapabilities, RuntimeOutputProfile, SparkOutputMode};

use super::build::{BoardId, RecipeFeature};
use super::FirmwareRecipe;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RecipeValidationError {
    UnsupportedRevLimiter,
    UnsupportedIgnitionOnly,
    UnsupportedInjectionOnly,
    UnsupportedFullEcu,
    MissingRpmInput,
    MissingTriggerInput,
    MissingCamInput,
    MissingLoadSensor,
    IgnitionChannelCount { required: u8, available: u8 },
    InjectorChannelCount { required: u8, available: u8 },
    AuxChannelCount { required: u8, available: u8 },
    MissingTelemetry,
    MissingCalibrationPersistence,
    MissingWatchdog,
    BoardSelectionMismatch { requested: BoardId, actual: BoardId },
    UnsupportedFeatureBinding { feature: RecipeFeature },
    CargoFeatureCapacityExceeded,
    FirstRunChecklistCapacityExceeded,
    TunerStudioProfileWithoutSubsystem,
    UnsupportedWastedSparkCylinderCount { cylinder_count: u8 },
}

impl FirmwareRecipe {
    pub const fn validate_for(
        self,
        capabilities: BoardCapabilities,
    ) -> Result<(), RecipeValidationError> {
        if self.subsystems.rev_limiter && !capabilities.supports_rev_limiter() {
            return Err(RecipeValidationError::UnsupportedRevLimiter);
        }
        if self.subsystems.ignition
            && !self.subsystems.injection
            && !capabilities.supports_ignition_only()
        {
            return Err(RecipeValidationError::UnsupportedIgnitionOnly);
        }
        if self.subsystems.injection
            && !self.subsystems.ignition
            && !capabilities.supports_injection_only()
        {
            return Err(RecipeValidationError::UnsupportedInjectionOnly);
        }
        if self.subsystems.ignition
            && self.subsystems.injection
            && !capabilities.supports_full_ecu()
        {
            return Err(RecipeValidationError::UnsupportedFullEcu);
        }
        if self.inputs.rpm && !capabilities.rpm_input {
            return Err(RecipeValidationError::MissingRpmInput);
        }
        if self.inputs.trigger && !capabilities.trigger_input {
            return Err(RecipeValidationError::MissingTriggerInput);
        }
        if self.inputs.cam && !capabilities.cam_input {
            return Err(RecipeValidationError::MissingCamInput);
        }
        if self.inputs.load_sensor && !capabilities.supports_load_sensor() {
            return Err(RecipeValidationError::MissingLoadSensor);
        }
        if let RuntimeOutputProfile::IgnitionOnly(profile) = self.runtime.output_profile {
            if matches!(profile.mode, SparkOutputMode::WastedSpark)
                && !is_supported_wasted_spark_cylinder_count(profile.cylinder_count)
            {
                return Err(RecipeValidationError::UnsupportedWastedSparkCylinderCount {
                    cylinder_count: profile.cylinder_count,
                });
            }
        }
        if self.outputs.ignition_channels > capabilities.ignition_channels {
            return Err(RecipeValidationError::IgnitionChannelCount {
                required: self.outputs.ignition_channels,
                available: capabilities.ignition_channels,
            });
        }
        if self.outputs.injector_channels > capabilities.injector_channels {
            return Err(RecipeValidationError::InjectorChannelCount {
                required: self.outputs.injector_channels,
                available: capabilities.injector_channels,
            });
        }
        if self.outputs.aux_channels > capabilities.aux_channels {
            return Err(RecipeValidationError::AuxChannelCount {
                required: self.outputs.aux_channels,
                available: capabilities.aux_channels,
            });
        }
        if self.subsystems.telemetry && !capabilities.telemetry {
            return Err(RecipeValidationError::MissingTelemetry);
        }
        if self.subsystems.persistence && !capabilities.calibration_persistence {
            return Err(RecipeValidationError::MissingCalibrationPersistence);
        }
        if self.safety.watchdog_required && !capabilities.watchdog {
            return Err(RecipeValidationError::MissingWatchdog);
        }
        if self.ts_profile.is_some() && !self.subsystems.tuner_studio {
            return Err(RecipeValidationError::TunerStudioProfileWithoutSubsystem);
        }

        Ok(())
    }
}

const fn is_supported_wasted_spark_cylinder_count(cylinder_count: u8) -> bool {
    cylinder_count >= 2 && cylinder_count <= 16 && cylinder_count.is_multiple_of(2)
}
