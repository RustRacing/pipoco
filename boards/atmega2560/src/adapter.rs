use ecu_board_api::{
    engine_time_authorizes_full_sequential, AuxCommand, AuxCommandBatch, AuxOutput,
    BoardCapabilities, BoardResourceLimits, RuntimeOutputProfile, SensorSnapshot, TelemetryFrame,
};
use ecu_board_profiles::{
    BoardBuildMetadata, FirmwareBuildPlan, FirmwareRecipe, FirstRunChecklistItem,
    TunerStudioProfileSelection,
};
use ecu_domain::{Degrees10, EngineTimeAuthority};
use ecu_runtime::{
    ActionExecutor, ActionOutputBatchAdapter, AuthorityStepInputs, ControlInputs, EngineRuntime,
    EnrichmentInputs, IgnitionInputs, LambdaTrimInputs, TorqueInputs,
};

use crate::errors::{
    Atmega2560BridgeError, Atmega2560BuildPlanError, Atmega2560PreparedStepError,
    Atmega2560RecipePrepareError,
};
use crate::profile::{
    m50b25tu_speeduino_m5x_rev23_board_profile, speeduino_m5x_rev23_board_profile_for_full_ecu,
    Atmega2560BoardProfile, SPEEDUINO_M5X_REV23_ATMEGA2560_BUILD_METADATA,
    SPEEDUINO_M5X_REV23_CAPABILITIES, SPEEDUINO_M5X_REV23_PIN_MAP,
    SPEEDUINO_M5X_REV23_RESOURCE_LIMITS,
};
use crate::profile::{MAX_AUX_COMMANDS, MAX_OUTPUT_TRANSITIONS};
use crate::step_io::{
    Atmega2560MappedOutputBatch, Atmega2560MappedOutputTransition, Atmega2560MappedStepOutput,
    Atmega2560StepInput, Atmega2560StepOutput,
};
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Atmega2560PreparedFirmware {
    plan: FirmwareBuildPlan,
    adapter: Atmega2560BoardAdapter,
}

impl Atmega2560PreparedFirmware {
    pub const fn plan(&self) -> &FirmwareBuildPlan {
        &self.plan
    }

    pub const fn adapter(&self) -> &Atmega2560BoardAdapter {
        &self.adapter
    }

    pub fn adapter_mut(&mut self) -> &mut Atmega2560BoardAdapter {
        &mut self.adapter
    }

    pub fn into_adapter(self) -> Atmega2560BoardAdapter {
        self.adapter
    }

    pub(crate) fn step_mapped(
        &mut self,
        input: Atmega2560StepInput,
    ) -> Result<Atmega2560MappedStepOutput, Atmega2560PreparedStepError> {
        let step = self.adapter.step(input)?;
        let mut mapped_outputs = Atmega2560MappedOutputBatch::new();

        for transition in step.outputs.iter() {
            let Some(pin) = SPEEDUINO_M5X_REV23_PIN_MAP.output_pin(transition.output) else {
                return Err(Atmega2560PreparedStepError::UnmappedOutput(
                    transition.output,
                ));
            };
            mapped_outputs
                .push(Atmega2560MappedOutputTransition {
                    transition: *transition,
                    pin,
                })
                .map_err(|()| {
                    Atmega2560PreparedStepError::Bridge(Atmega2560BridgeError::OutputBatchFull)
                })?;
        }

        Ok(Atmega2560MappedStepOutput {
            mapped_outputs,
            step,
        })
    }
}

/// ATmega2560-compatible runtime boundary for a supplied board/runtime profile.
///
/// Pin names, timer compare units, ADC channels, and interrupt vectors are not
/// claimed here. A chip/HAL crate must bind these logical commands to verified
/// board pins.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Atmega2560BoardAdapter {
    runtime: EngineRuntime,
    profile: Atmega2560BoardProfile,
}

impl Atmega2560BoardAdapter {
    pub const fn speeduino_m5x_rev23_build_metadata() -> BoardBuildMetadata {
        SPEEDUINO_M5X_REV23_ATMEGA2560_BUILD_METADATA
    }

    pub const fn speeduino_m5x_rev23_capabilities() -> BoardCapabilities {
        SPEEDUINO_M5X_REV23_CAPABILITIES
    }

    pub const fn speeduino_m5x_rev23_resource_limits() -> BoardResourceLimits {
        SPEEDUINO_M5X_REV23_RESOURCE_LIMITS
    }

    pub const fn resource_limits(&self) -> BoardResourceLimits {
        SPEEDUINO_M5X_REV23_RESOURCE_LIMITS
    }

    pub fn prepare_speeduino_m5x_rev23_recipe(
        recipe: FirmwareRecipe,
    ) -> Result<Atmega2560PreparedFirmware, Atmega2560RecipePrepareError> {
        let runtime_output_profile = match recipe.runtime.output_profile {
            RuntimeOutputProfile::FullEcu(profile) => profile,
            output_profile => {
                return Err(Atmega2560RecipePrepareError::UnsupportedRuntimeProfile {
                    output_profile,
                });
            }
        };
        let plan = recipe.resolve_build_plan(Self::speeduino_m5x_rev23_build_metadata())?;
        validate_speeduino_m5x_rev23_build_plan(plan)?;
        Ok(Atmega2560PreparedFirmware {
            plan,
            adapter: Self::new(speeduino_m5x_rev23_board_profile_for_full_ecu(
                runtime_output_profile,
            )),
        })
    }

    /// Compatibility helper for metadata-only callers.
    ///
    /// `FirmwareBuildPlan` intentionally does not carry runtime topology, so
    /// this cannot be the generic recipe seam. Prefer
    /// [`Self::prepare_speeduino_m5x_rev23_recipe`] when preparing firmware from
    /// a user-selected recipe.
    pub(crate) fn prepare_speeduino_m5x_rev23_build_plan(
        plan: FirmwareBuildPlan,
    ) -> Result<Atmega2560PreparedFirmware, Atmega2560BuildPlanError> {
        validate_speeduino_m5x_rev23_build_plan(plan)?;
        Ok(Atmega2560PreparedFirmware {
            plan,
            adapter: Self::m50b25tu_speeduino_m5x_rev23(),
        })
    }

    pub fn new(profile: Atmega2560BoardProfile) -> Self {
        let mut runtime = EngineRuntime::new();
        runtime.configure_full_ecu(profile.runtime_output_profile);
        Self { runtime, profile }
    }

    pub(crate) fn m50b25tu_speeduino_m5x_rev23() -> Self {
        Self::new(m50b25tu_speeduino_m5x_rev23_board_profile())
    }

    pub const fn profile(&self) -> &Atmega2560BoardProfile {
        &self.profile
    }

    pub fn runtime(&self) -> &EngineRuntime {
        &self.runtime
    }

    pub fn runtime_mut(&mut self) -> &mut EngineRuntime {
        &mut self.runtime
    }

    pub fn step(
        &mut self,
        input: Atmega2560StepInput,
    ) -> Result<Atmega2560StepOutput, Atmega2560BridgeError> {
        let authority = effective_engine_time_authority(input);
        let result = self.runtime.step_with_authority(
            authority_step_inputs(input, authority),
            conservative_control_inputs(input),
        );
        let mut output = Atmega2560StepOutput::new(telemetry(input, &self.runtime, &self.profile));

        let mut executor =
            ActionOutputBatchAdapter::<MAX_OUTPUT_TRANSITIONS, MAX_AUX_COMMANDS>::new();
        executor
            .execute_batch(result.actions)
            .map_err(Atmega2560BridgeError::from)?;
        output.outputs = *executor.output_transitions();
        output.aux = validate_speeduino_m5x_rev23_aux_batch(executor.aux_commands())?;
        let status = executor.status();
        output.cancel_scheduled_outputs = status.cancel_scheduled_outputs();
        output.persist_calibration = status.persist_calibration;

        Ok(output)
    }
}

fn validate_speeduino_m5x_rev23_build_plan(
    plan: FirmwareBuildPlan,
) -> Result<(), Atmega2560BuildPlanError> {
    let metadata = SPEEDUINO_M5X_REV23_ATMEGA2560_BUILD_METADATA;
    if plan.board_id != metadata.board_id {
        return Err(Atmega2560BuildPlanError::BoardId {
            expected: metadata.board_id,
            actual: plan.board_id,
        });
    }
    if plan.artifact != metadata.artifact {
        return Err(Atmega2560BuildPlanError::Artifact {
            expected: metadata.artifact,
            actual: plan.artifact,
        });
    }
    if plan.pin_map_id != metadata.pin_map_id {
        return Err(Atmega2560BuildPlanError::PinMap {
            expected: metadata.pin_map_id,
            actual: plan.pin_map_id,
        });
    }
    if plan.runtime_build_id != metadata.runtime_build_id {
        return Err(Atmega2560BuildPlanError::RuntimeBuild {
            expected: metadata.runtime_build_id,
            actual: plan.runtime_build_id,
        });
    }
    if !plan.cargo_features.is_empty() {
        return Err(Atmega2560BuildPlanError::CargoFeaturesUnsupported);
    }
    if plan.ts_profile != TunerStudioProfileSelection::None {
        return Err(Atmega2560BuildPlanError::TunerStudioProfileUnsupported {
            actual: plan.ts_profile,
        });
    }

    const EXPECTED_FIRST_RUN: &[Option<FirstRunChecklistItem>] = &[
        Some(FirstRunChecklistItem::VerifyPinMap(
            SPEEDUINO_M5X_REV23_ATMEGA2560_BUILD_METADATA.pin_map_id,
        )),
        Some(FirstRunChecklistItem::VerifyTriggerWiring),
        Some(FirstRunChecklistItem::VerifyLoadSensor),
    ];
    if plan.first_run.as_slice() != EXPECTED_FIRST_RUN {
        return Err(Atmega2560BuildPlanError::FirstRunChecklist {
            expected: EXPECTED_FIRST_RUN,
            actual: plan.first_run,
        });
    }

    Ok(())
}

pub(crate) fn validate_speeduino_m5x_rev23_aux_batch<const N: usize>(
    batch: &AuxCommandBatch<N>,
) -> Result<AuxCommandBatch<N>, Atmega2560BridgeError> {
    let mut validated = AuxCommandBatch::new();
    for command in batch.iter() {
        validate_speeduino_m5x_rev23_aux_command(*command)?;
        validated
            .push(*command)
            .map_err(|_| Atmega2560BridgeError::AuxBatchFull)?;
    }
    Ok(validated)
}

pub(crate) const fn speeduino_m5x_rev23_aux_pin(command: AuxCommand) -> Option<u8> {
    let pin = match command.output {
        AuxOutput::SafetyRelay(0) => SPEEDUINO_M5X_REV23_PIN_MAP.low_current_pins[0],
        AuxOutput::SafetyRelay(1) => SPEEDUINO_M5X_REV23_PIN_MAP.low_current_pins[1],
        AuxOutput::Indicator(0) => SPEEDUINO_M5X_REV23_PIN_MAP.low_current_pins[2],
        AuxOutput::Pwm(channel) if channel.get() == 0 => {
            SPEEDUINO_M5X_REV23_PIN_MAP.low_current_pins[3]
        }
        AuxOutput::Digital(channel) if channel.get() == 4 => {
            SPEEDUINO_M5X_REV23_PIN_MAP.low_current_pins[4]
        }
        AuxOutput::FrequencyOut(0) => SPEEDUINO_M5X_REV23_PIN_MAP.tach1_pin,
        AuxOutput::SafetyRelay(2) => SPEEDUINO_M5X_REV23_PIN_MAP.tach2_pin,
        AuxOutput::SafetyRelay(_)
        | AuxOutput::Indicator(_)
        | AuxOutput::FrequencyOut(_)
        | AuxOutput::Digital(_)
        | AuxOutput::Pwm(_) => return None,
    };
    Some(pin.get())
}

pub(crate) const fn validate_speeduino_m5x_rev23_aux_command(
    command: AuxCommand,
) -> Result<(), Atmega2560BridgeError> {
    if speeduino_m5x_rev23_aux_pin(command).is_some() {
        Ok(())
    } else {
        Err(Atmega2560BridgeError::UnsupportedAuxCommand(command))
    }
}

fn effective_engine_time_authority(input: Atmega2560StepInput) -> EngineTimeAuthority {
    input.engine_time_authority
}

fn authority_step_inputs(
    input: Atmega2560StepInput,
    authority: EngineTimeAuthority,
) -> AuthorityStepInputs {
    AuthorityStepInputs {
        now_us: input.now_us,
        rpm: input.rpm.get() as u32,
        load_kpa10: input.load_kpa10.get() as u32,
        angle_x10: input.crank_angle_x10.get() as i32,
        authority,
        launch_armed: input.launch_armed,
        flat_shift_armed: input.flat_shift_armed,
        safety_latch_request: false,
    }
}

fn conservative_control_inputs(input: Atmega2560StepInput) -> ControlInputs {
    ControlInputs {
        enrichment: EnrichmentInputs {
            now_us: input.now_us,
            clt_c: input.coolant_temp_c10 / 10,
            cranking: input.rpm.get() < 450,
            just_started: false,
            tpsdot_pct_s: 0,
            mapdot_kpa_s: 0,
        },
        lambda: LambdaTrimInputs {
            now_us: input.now_us,
            clt_c: input.coolant_temp_c10 / 10,
            just_started: false,
            lambda_valid: input.lambda.get() > 0,
            measured_lambda100: input.lambda,
            requested_open_loop: false,
        },
        torque: TorqueInputs::new(100, 0, 100, 100, 100),
        ignition: IgnitionInputs::new(Degrees10::new(100), 0, 0, 0, false, input.rpm),
        fuel_sensors: ecu_runtime::FuelSensorInputs {
            maf_valid: false,
            maf_x100: 0,
            iat_c10: input.intake_temp_c10,
            vbatt_mv: input.battery_mv,
            baro_valid: false,
            baro_kpa10: ecu_domain::Kpa10::new(1010),
        },
        knock_intensity_x100: 0,
    }
}

fn telemetry(
    input: Atmega2560StepInput,
    runtime: &EngineRuntime,
    profile: &Atmega2560BoardProfile,
) -> TelemetryFrame {
    let snapshot = runtime.snapshot();
    let authority = runtime.engine_time_authority();
    let ignition_profile_mode = if engine_time_authorizes_full_sequential(authority) {
        profile.ignition_profile_mode
    } else {
        profile.ignition_profile_authority_blocked_mode
    };
    TelemetryFrame::new(
        SensorSnapshot::new_with_engine_time_authority(
            input.now_us,
            input.rpm,
            input.load_kpa10,
            input.throttle,
            input.coolant_temp_c10,
            input.intake_temp_c10,
            input.battery_mv,
            input.lambda,
            authority,
            snapshot.engine.phase,
        ),
        profile.profile_id,
        profile.ignition_profile_id,
        ignition_profile_mode,
        profile.pin_map_id,
        profile.runtime_build_id,
        snapshot.engine.mode,
        snapshot.faults.fault,
        snapshot.faults.severity,
        snapshot.control.ignition_advance,
        snapshot.control.dwell,
        snapshot.control.fuel_pulse_width,
    )
}

#[cfg(test)]
mod tests {
    use super::*;
    use ecu_domain::{Kpa10, Micros, Rpm};

    #[test]
    fn authority_step_inputs_preserve_launch_arming() {
        let input =
            Atmega2560StepInput::bench_synced(Micros::new(1_000), Rpm::new(3_000), Kpa10::new(800))
                .with_launch_armed(true)
                .with_flat_shift_armed(false);
        let mapped = authority_step_inputs(input, input.engine_time_authority);

        assert!(mapped.launch_armed);
        assert!(!mapped.flat_shift_armed);
        assert_eq!(mapped.authority, input.engine_time_authority);
    }

    #[test]
    fn authority_step_inputs_preserve_flat_shift_arming() {
        let input =
            Atmega2560StepInput::bench_synced(Micros::new(1_000), Rpm::new(3_000), Kpa10::new(800))
                .with_launch_armed(false)
                .with_flat_shift_armed(true);
        let mapped = authority_step_inputs(input, input.engine_time_authority);

        assert!(!mapped.launch_armed);
        assert!(mapped.flat_shift_armed);
        assert_eq!(mapped.authority, input.engine_time_authority);
    }
}
