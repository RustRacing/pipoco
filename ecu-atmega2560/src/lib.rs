#![cfg_attr(not(test), no_std)]
#![forbid(unsafe_code)]

use ecu_board_api::{
    engine_time_authorizes_full_sequential, AuxCommandBatch, EcuOutput, IgnitionProfileId,
    IgnitionProfileMode, OutputLevel, OutputTransition, OutputTransitionBatch, PinMapId, ProfileId,
    RuntimeBuildId, SensorSnapshot, TelemetryFrame,
};
use ecu_board_profiles::M50B25TU_FULL_COP;
use ecu_domain::{
    AbsoluteTimeAuthority, CrankSyncState, Degrees10, EngineTimeAuthority, Kpa10, Lambda100,
    Micros, Percent, PhaseSyncState, Rpm, Ticks,
};
use ecu_runtime::{
    Action, ControlInputs, EngineRuntime, EnrichmentInputs, IgnitionInputs, LambdaTrimInputs,
    StepInputs, TorqueInputs,
};

pub const TARGET_TRIPLE: &str = "avr-none";
pub const TARGET_CPU: &str = "atmega2560";
pub const CLOCK_HZ: u32 = 16_000_000;
pub const MAX_OUTPUT_TRANSITIONS: usize = 24;
pub const MAX_AUX_COMMANDS: usize = 16;

pub const PROFILE_ID_M50B25TU_FULL_COP: ProfileId = ProfileId::new(0x5026);
pub const IGNITION_PROFILE_ID_SEQUENTIAL_COP_6: IgnitionProfileId = IgnitionProfileId::new(6);
pub const PIN_MAP_SPEEDUINO_M5X_REV23_SCHEMATIC: PinMapId = PinMapId::new(0x023);
pub const RUNTIME_BUILD_ID_ATMEGA2560_BRIDGE: RuntimeBuildId = RuntimeBuildId::new(1);

#[repr(transparent)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Default)]
pub struct Atmega2560DigitalPin(u8);

impl Atmega2560DigitalPin {
    pub const fn new(value: u8) -> Self {
        Self(value)
    }

    pub const fn get(self) -> u8 {
        self.0
    }
}

#[repr(transparent)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Default)]
pub struct Atmega2560AnalogPin(u8);

impl Atmega2560AnalogPin {
    pub const fn new(value: u8) -> Self {
        Self(value)
    }

    pub const fn get(self) -> u8 {
        self.0
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SpeeduinoM5xConditionedInput {
    CrankVr1,
    CamVr2,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SpeeduinoM5xRev23PinMap {
    pub injector_pins: [Atmega2560DigitalPin; 6],
    pub ignition_pins: [Atmega2560DigitalPin; 6],
    pub crank_input: SpeeduinoM5xConditionedInput,
    pub cam_input: SpeeduinoM5xConditionedInput,
    pub tach1_pin: Atmega2560DigitalPin,
    pub tach2_pin: Atmega2560DigitalPin,
    pub low_current_pins: [Atmega2560DigitalPin; 5],
    pub reset_pin: Atmega2560DigitalPin,
    pub iat_pin: Atmega2560AnalogPin,
    pub clt_pin: Atmega2560AnalogPin,
    pub tps_pin: Atmega2560AnalogPin,
    pub map_pin: Atmega2560AnalogPin,
    pub battery_pin: Atmega2560AnalogPin,
    pub oxygen_pin: Atmega2560AnalogPin,
}

impl SpeeduinoM5xRev23PinMap {
    pub const fn output_pin(self, output: EcuOutput) -> Option<Atmega2560DigitalPin> {
        match output {
            EcuOutput::Injector(channel) => {
                let idx = channel.get() as usize;
                if idx < self.injector_pins.len() {
                    Some(self.injector_pins[idx])
                } else {
                    None
                }
            }
            EcuOutput::Ignition(channel) => {
                let idx = channel.get() as usize;
                if idx < self.ignition_pins.len() {
                    Some(self.ignition_pins[idx])
                } else {
                    None
                }
            }
        }
    }
}

/// Schematic-derived Speeduino-M5x Rev 2.3 CPU pin map for the Bosch 88-pin
/// Motronic board.
///
/// Evidence source:
/// `Speeduino-M5x-PCBs/m50-m40-m60_Pnp/Rev 2.3/Schematic__speeduino
/// compatible PCB for bosch 88pin motronic rev2.3.pdf`.
pub const SPEEDUINO_M5X_REV23_PIN_MAP: SpeeduinoM5xRev23PinMap = SpeeduinoM5xRev23PinMap {
    injector_pins: [
        Atmega2560DigitalPin::new(8),
        Atmega2560DigitalPin::new(9),
        Atmega2560DigitalPin::new(10),
        Atmega2560DigitalPin::new(11),
        Atmega2560DigitalPin::new(12),
        Atmega2560DigitalPin::new(50),
    ],
    ignition_pins: [
        Atmega2560DigitalPin::new(40),
        Atmega2560DigitalPin::new(38),
        Atmega2560DigitalPin::new(52),
        Atmega2560DigitalPin::new(48),
        Atmega2560DigitalPin::new(36),
        Atmega2560DigitalPin::new(34),
    ],
    crank_input: SpeeduinoM5xConditionedInput::CrankVr1,
    cam_input: SpeeduinoM5xConditionedInput::CamVr2,
    tach1_pin: Atmega2560DigitalPin::new(19),
    tach2_pin: Atmega2560DigitalPin::new(18),
    low_current_pins: [
        Atmega2560DigitalPin::new(45),
        Atmega2560DigitalPin::new(47),
        Atmega2560DigitalPin::new(49),
        Atmega2560DigitalPin::new(51),
        Atmega2560DigitalPin::new(53),
    ],
    reset_pin: Atmega2560DigitalPin::new(43),
    iat_pin: Atmega2560AnalogPin::new(0),
    clt_pin: Atmega2560AnalogPin::new(1),
    tps_pin: Atmega2560AnalogPin::new(2),
    map_pin: Atmega2560AnalogPin::new(3),
    battery_pin: Atmega2560AnalogPin::new(4),
    oxygen_pin: Atmega2560AnalogPin::new(8),
};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Atmega2560BridgeError {
    OutputBatchFull,
    AuxBatchFull,
}

/// Inputs already sampled by board-specific register glue.
///
/// This crate intentionally does not read ATmega registers. It is the portable
/// runtime-to-board boundary that a future HAL crate can call after sampling
/// timers, ADCs, trigger sync state, and cam state.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Atmega2560StepInput {
    pub now_us: Micros,
    pub rpm: Rpm,
    pub load_kpa10: Kpa10,
    pub crank_angle_x10: Degrees10,
    pub throttle: Percent,
    pub coolant_temp_c10: i16,
    pub intake_temp_c10: i16,
    pub battery_mv: u16,
    pub lambda: Lambda100,
    pub engine_time_authority: EngineTimeAuthority,
    pub synced: bool,
    pub cam_seen: bool,
}

impl Atmega2560StepInput {
    pub const fn expert_manual_authority() -> EngineTimeAuthority {
        EngineTimeAuthority::new(
            CrankSyncState::PrimaryLocked,
            PhaseSyncState::CamValidated720,
            AbsoluteTimeAuthority::ExpertManual,
            EngineTimeAuthority::MAX_CONFIDENCE_X1000,
            0,
        )
    }

    pub const fn bench_synced(now_us: Micros, rpm: Rpm, load_kpa10: Kpa10) -> Self {
        Self {
            now_us,
            rpm,
            load_kpa10,
            crank_angle_x10: Degrees10::new(0),
            throttle: Percent::new(50),
            coolant_temp_c10: 850,
            intake_temp_c10: 300,
            battery_mv: 13_800,
            lambda: Lambda100::new(100),
            engine_time_authority: Self::expert_manual_authority(),
            synced: true,
            cam_seen: true,
        }
    }

    pub const fn with_engine_time_authority(self, authority: EngineTimeAuthority) -> Self {
        Self {
            now_us: self.now_us,
            rpm: self.rpm,
            load_kpa10: self.load_kpa10,
            crank_angle_x10: self.crank_angle_x10,
            throttle: self.throttle,
            coolant_temp_c10: self.coolant_temp_c10,
            intake_temp_c10: self.intake_temp_c10,
            battery_mv: self.battery_mv,
            lambda: self.lambda,
            engine_time_authority: authority,
            synced: authority.has_primary_lock(),
            cam_seen: matches!(
                authority.phase,
                PhaseSyncState::CamObserved720 | PhaseSyncState::CamValidated720
            ),
        }
    }
}

/// Fixed-capacity board command set emitted by one runtime step.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Atmega2560StepOutput {
    pub outputs: OutputTransitionBatch<MAX_OUTPUT_TRANSITIONS>,
    pub aux: AuxCommandBatch<MAX_AUX_COMMANDS>,
    pub telemetry: TelemetryFrame,
    pub cancel_scheduled_outputs: bool,
    pub persist_calibration: bool,
}

impl Atmega2560StepOutput {
    pub const fn new(telemetry: TelemetryFrame) -> Self {
        Self {
            outputs: OutputTransitionBatch::new(),
            aux: AuxCommandBatch::new(),
            telemetry,
            cancel_scheduled_outputs: false,
            persist_calibration: false,
        }
    }
}

/// ATmega2560-compatible runtime boundary for the M50B25TU full sequential
/// injector and coil-on-plug profile.
///
/// Pin names, timer compare units, ADC channels, and interrupt vectors are not
/// claimed here. A chip/HAL crate must bind these logical commands to verified
/// board pins.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Atmega2560M50Bridge {
    runtime: EngineRuntime,
}

impl Atmega2560M50Bridge {
    pub fn new() -> Self {
        let mut runtime = EngineRuntime::new();
        runtime.configure_m50_full_cop();
        Self { runtime }
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
        self.runtime.set_engine_time_authority(authority);
        let result = self.runtime.step(
            step_inputs(input, authority),
            conservative_control_inputs(input),
        );
        let mut output = Atmega2560StepOutput::new(telemetry(input, &self.runtime));

        for action in result.actions.iter() {
            match action {
                Action::ArmScheduler {
                    injection,
                    ignition,
                } => {
                    push_output_pair(
                        &mut output.outputs,
                        EcuOutput::Injector(injection.plan.output.channel()),
                        injection.start_at,
                        injection.end_at,
                    )?;
                    push_output_pair(
                        &mut output.outputs,
                        EcuOutput::Ignition(ignition.plan.output.channel()),
                        ignition.start_at,
                        ignition.end_at,
                    )?;
                }
                Action::CancelScheduler(_) => {
                    output.cancel_scheduled_outputs = true;
                }
                Action::ApplyAux(commands) => {
                    for command in commands.iter() {
                        output
                            .aux
                            .push(*command)
                            .map_err(|_| Atmega2560BridgeError::AuxBatchFull)?;
                    }
                }
                Action::PersistCalibration => {
                    output.persist_calibration = true;
                }
                Action::PublishSnapshot | Action::Idle => {}
                #[allow(deprecated)]
                Action::SetFan(on) => {
                    let level = if on {
                        OutputLevel::High
                    } else {
                        OutputLevel::Low
                    };
                    output
                        .aux
                        .push(ecu_board_api::AuxCommand::new(
                            ecu_board_api::AuxOutput::Fan,
                            ecu_board_api::AuxValue::Level(level),
                        ))
                        .map_err(|_| Atmega2560BridgeError::AuxBatchFull)?;
                }
            }
        }

        Ok(output)
    }
}

impl Default for Atmega2560M50Bridge {
    fn default() -> Self {
        Self::new()
    }
}

fn push_output_pair(
    batch: &mut OutputTransitionBatch<MAX_OUTPUT_TRANSITIONS>,
    output: EcuOutput,
    start_at: Micros,
    end_at: Micros,
) -> Result<(), Atmega2560BridgeError> {
    batch
        .push(OutputTransition::new(
            output,
            OutputLevel::High,
            Ticks::new(start_at.get()),
        ))
        .map_err(|_| Atmega2560BridgeError::OutputBatchFull)?;
    batch
        .push(OutputTransition::new(
            output,
            OutputLevel::Low,
            Ticks::new(end_at.get()),
        ))
        .map_err(|_| Atmega2560BridgeError::OutputBatchFull)?;
    Ok(())
}

fn legacy_engine_time_authority(synced: bool, cam_seen: bool) -> EngineTimeAuthority {
    if !synced {
        return EngineTimeAuthority::none();
    }

    let phase = if cam_seen {
        PhaseSyncState::CamObserved720
    } else {
        PhaseSyncState::CrankOnly360
    };
    EngineTimeAuthority::new(
        CrankSyncState::PrimaryLocked,
        phase,
        AbsoluteTimeAuthority::GeometryOnly,
        EngineTimeAuthority::MAX_CONFIDENCE_X1000,
        0,
    )
}

fn effective_engine_time_authority(input: Atmega2560StepInput) -> EngineTimeAuthority {
    if input.engine_time_authority != EngineTimeAuthority::none() {
        input.engine_time_authority
    } else {
        legacy_engine_time_authority(input.synced, input.cam_seen)
    }
}

fn authority_cam_seen(authority: EngineTimeAuthority) -> bool {
    matches!(
        authority.phase,
        PhaseSyncState::CamObserved720 | PhaseSyncState::CamValidated720
    )
}

fn step_inputs(input: Atmega2560StepInput, authority: EngineTimeAuthority) -> StepInputs {
    StepInputs {
        now_us: input.now_us,
        rpm: input.rpm.get() as u32,
        load_kpa10: input.load_kpa10.get() as u32,
        angle_x10: input.crank_angle_x10.get() as i32,
        trigger_synced: authority.has_primary_lock(),
        cam_seen: authority_cam_seen(authority),
        launch_armed: false,
        flat_shift_armed: false,
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
            clt_c: input.coolant_temp_c10 / 10,
            lambda_valid: input.lambda.get() > 0,
            measured_lambda100: input.lambda,
            requested_open_loop: false,
        },
        torque: TorqueInputs::new(100, 0, 100, 100, 100),
        ignition: IgnitionInputs::new(Degrees10::new(100), 0, 0, 0, false, input.rpm),
    }
}

fn telemetry(input: Atmega2560StepInput, runtime: &EngineRuntime) -> TelemetryFrame {
    let snapshot = runtime.snapshot();
    let authority = runtime.engine_time_authority();
    let ignition_profile_mode = if engine_time_authorizes_full_sequential(authority) {
        IgnitionProfileMode::SequentialCop
    } else {
        IgnitionProfileMode::SequentialCopAuthorityBlocked
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
        PROFILE_ID_M50B25TU_FULL_COP,
        IGNITION_PROFILE_ID_SEQUENTIAL_COP_6,
        ignition_profile_mode,
        PIN_MAP_SPEEDUINO_M5X_REV23_SCHEMATIC,
        RUNTIME_BUILD_ID_ATMEGA2560_BRIDGE,
        snapshot.engine.mode,
        snapshot.faults.fault,
        snapshot.faults.severity,
        snapshot.control.ignition_advance,
        snapshot.control.dwell,
        snapshot.control.fuel_pulse_width,
    )
}

pub const fn profile_cylinders() -> u8 {
    M50B25TU_FULL_COP.engine.cylinders
}

pub const fn profile_requires_cam_phase() -> bool {
    M50B25TU_FULL_COP.cam.phase_required_for_sequential
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn bridge_uses_m50_profile_facts() {
        assert_eq!(profile_cylinders(), 6);
        assert!(profile_requires_cam_phase());
    }

    #[test]
    fn schematic_pin_map_exposes_six_injectors_and_six_ignitions() {
        assert_eq!(
            SPEEDUINO_M5X_REV23_PIN_MAP
                .injector_pins
                .map(|pin| pin.get()),
            [8, 9, 10, 11, 12, 50]
        );
        assert_eq!(
            SPEEDUINO_M5X_REV23_PIN_MAP
                .ignition_pins
                .map(|pin| pin.get()),
            [40, 38, 52, 48, 36, 34]
        );
    }

    #[test]
    fn expert_manual_authority_emits_six_injection_and_six_ignition_windows() {
        let mut bridge = Atmega2560M50Bridge::new();
        let output = bridge
            .step(Atmega2560StepInput::bench_synced(
                Micros::new(1_000),
                Rpm::new(3_000),
                Kpa10::new(800),
            ))
            .unwrap();

        assert_eq!(output.outputs.len(), 24);
        assert!(!output.cancel_scheduled_outputs);
        assert_eq!(output.telemetry.profile_id, PROFILE_ID_M50B25TU_FULL_COP);
        assert_eq!(
            output.telemetry.ignition_profile_mode,
            IgnitionProfileMode::SequentialCop
        );
        assert_eq!(
            output.telemetry.snapshot.engine_time.source(),
            AbsoluteTimeAuthority::ExpertManual
        );
        assert!(
            output
                .telemetry
                .snapshot
                .engine_time
                .full_sequential_authorized
        );
        assert_eq!(
            output.telemetry.pin_map_id,
            PIN_MAP_SPEEDUINO_M5X_REV23_SCHEMATIC
        );
    }

    #[test]
    fn synced_step_outputs_all_map_to_schematic_pins() {
        let mut bridge = Atmega2560M50Bridge::new();
        let output = bridge
            .step(Atmega2560StepInput::bench_synced(
                Micros::new(1_000),
                Rpm::new(3_000),
                Kpa10::new(800),
            ))
            .unwrap();

        let mut injector_seen = [false; 6];
        let mut ignition_seen = [false; 6];
        for transition in output.outputs.iter() {
            let pin = SPEEDUINO_M5X_REV23_PIN_MAP.output_pin(transition.output);
            assert!(pin.is_some());

            match transition.output {
                EcuOutput::Injector(channel) => {
                    injector_seen[channel.get() as usize] = true;
                }
                EcuOutput::Ignition(channel) => {
                    ignition_seen[channel.get() as usize] = true;
                }
            }
        }

        assert_eq!(injector_seen, [true; 6]);
        assert_eq!(ignition_seen, [true; 6]);
    }

    #[test]
    fn unsynced_step_emits_no_scheduled_windows() {
        let mut bridge = Atmega2560M50Bridge::new();
        let mut input =
            Atmega2560StepInput::bench_synced(Micros::new(1_000), Rpm::new(3_000), Kpa10::new(800));
        input.engine_time_authority = EngineTimeAuthority::none();
        input.synced = false;
        input.cam_seen = false;

        let output = bridge.step(input).unwrap();
        assert_eq!(output.outputs.len(), 0);
    }

    #[test]
    fn geometry_only_authority_emits_no_full_sequential_cop_output() {
        let mut bridge = Atmega2560M50Bridge::new();
        let authority = EngineTimeAuthority::new(
            CrankSyncState::PrimaryLocked,
            PhaseSyncState::CamValidated720,
            AbsoluteTimeAuthority::GeometryOnly,
            EngineTimeAuthority::MAX_CONFIDENCE_X1000,
            0,
        );
        let input =
            Atmega2560StepInput::bench_synced(Micros::new(1_000), Rpm::new(3_000), Kpa10::new(800))
                .with_engine_time_authority(authority);

        let output = bridge.step(input).unwrap();

        assert_eq!(output.outputs.len(), 0);
        assert_eq!(
            output.telemetry.ignition_profile_mode,
            IgnitionProfileMode::SequentialCopAuthorityBlocked
        );
        assert_eq!(
            output.telemetry.snapshot.engine_time.source(),
            AbsoluteTimeAuthority::GeometryOnly
        );
        assert_eq!(
            output.telemetry.snapshot.engine_time.summary,
            ecu_domain::SyncState::Synced
        );
        assert!(
            !output
                .telemetry
                .snapshot
                .engine_time
                .full_sequential_authorized
        );
    }

    #[test]
    fn legacy_synced_cam_seen_flags_only_report_geometry_authority() {
        let mut bridge = Atmega2560M50Bridge::new();
        let mut input =
            Atmega2560StepInput::bench_synced(Micros::new(1_000), Rpm::new(3_000), Kpa10::new(800));
        input.engine_time_authority = EngineTimeAuthority::none();
        input.synced = true;
        input.cam_seen = true;

        let output = bridge.step(input).unwrap();

        assert_eq!(output.outputs.len(), 0);
        assert_eq!(
            output.telemetry.snapshot.engine_time.source(),
            AbsoluteTimeAuthority::GeometryOnly
        );
        assert!(
            !output
                .telemetry
                .snapshot
                .engine_time
                .full_sequential_authorized
        );
    }
}
