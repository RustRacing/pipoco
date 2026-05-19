use crate::{
    combustion::CombustionFrame,
    dyno::DynoFrame,
    fuel::InjectionCommand,
    knock::KnockFrame,
    sensors::SensorSnapshot,
    spark::SparkCommand,
    telemetry::TelemetryFrame,
    trigger::{FaultInput, TriggerEdge},
    types::*,
    PlantConfigError,
};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct DriverInput {
    pub throttle_x1000: u16,
    pub starter_enabled: bool,
    pub load_torque_nm_x100: TorqueNmX100,
}

impl DriverInput {
    pub const fn idle() -> Self {
        Self {
            throttle_x1000: 0,
            starter_enabled: false,
            load_torque_nm_x100: TorqueNmX100(0),
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct EnvironmentInput {
    pub ambient_c10: Celsius10,
    pub coolant_c10: Celsius10,
    pub battery_mv: Millivolts,
}

impl EnvironmentInput {
    pub const fn standard() -> Self {
        Self {
            ambient_c10: Celsius10(250),
            coolant_c10: Celsius10(800),
            battery_mv: Millivolts(13500),
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct EcuOutputFrame<const CYL: usize, const MAX_EVENTS: usize> {
    pub fuel_cut: bool,
    pub spark_cut: bool,
    pub idle_command_x1000: u16,
    pub injection_events: FixedSlice<InjectionCommand, MAX_EVENTS>,
    pub spark_events: FixedSlice<SparkCommand, MAX_EVENTS>,
}

impl<const CYL: usize, const MAX_EVENTS: usize> EcuOutputFrame<CYL, MAX_EVENTS> {
    pub const fn empty() -> Self {
        Self {
            fuel_cut: false,
            spark_cut: false,
            idle_command_x1000: 0,
            injection_events: FixedSlice::empty(InjectionCommand::empty()),
            spark_events: FixedSlice::empty(SparkCommand::empty()),
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct PlantStepInput<const CYL: usize, const MAX_EVENTS: usize> {
    pub dt_us: Micros,
    pub driver: DriverInput,
    pub environment: EnvironmentInput,
    pub ecu_outputs: EcuOutputFrame<CYL, MAX_EVENTS>,
    pub faults: FaultInput,
}

impl<const CYL: usize, const MAX_EVENTS: usize> PlantStepInput<CYL, MAX_EVENTS> {
    pub const fn idle(dt_us: Micros) -> Self {
        Self {
            dt_us,
            driver: DriverInput::idle(),
            environment: EnvironmentInput::standard(),
            ecu_outputs: EcuOutputFrame::empty(),
            faults: FaultInput::none(),
        }
    }

    pub fn validate(&self, _cylinder_count: u8) -> Result<(), PlantStepError> {
        if self.driver.throttle_x1000 > 1000 || self.ecu_outputs.idle_command_x1000 > 1000 {
            return Err(PlantStepError::InvalidInput);
        }
        for command in self.ecu_outputs.injection_events.as_slice() {
            if command.pulse_width_us.0 > command.deadtime_us.0
                && command.injector_flow_ug_per_us.0 == 0
            {
                return Err(PlantStepError::InvalidInput);
            }
        }
        Ok(())
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ConsumedEcuEvents<const MAX_EVENTS: usize> {
    pub injection_count: usize,
    pub spark_count: usize,
    pub ignored_injection_count: usize,
    pub ignored_spark_count: usize,
    pub injection_fuel_mass_ug: [MassUg; MAX_EVENTS],
}

impl<const MAX_EVENTS: usize> ConsumedEcuEvents<MAX_EVENTS> {
    pub const fn empty() -> Self {
        Self {
            injection_count: 0,
            spark_count: 0,
            ignored_injection_count: 0,
            ignored_spark_count: 0,
            injection_fuel_mass_ug: [MassUg(0); MAX_EVENTS],
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum DiagnosticKind {
    InvalidCylinderIndex,
    InputEventIgnoredDueToCut,
    OutputCapacityExceeded,
    CombustionMisfire,
    TriggerEdgeSuppressedByFault,
    TriggerEdgeDuplicatedByFault,
    PhysicalSaturation,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct DiagnosticEvent {
    pub timestamp_us: Micros,
    pub kind: DiagnosticKind,
    pub cylinder: Option<CylinderIndex>,
}

impl DiagnosticEvent {
    pub const fn empty() -> Self {
        Self {
            timestamp_us: Micros(0),
            kind: DiagnosticKind::CombustionMisfire,
            cylinder: None,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct PlantDiagnostics {
    pub last_step_us: Micros,
    pub validated_config: bool,
    pub overflow_count: u16,
    pub events: FixedSlice<DiagnosticEvent, MAX_DIAGNOSTIC_EVENTS_PER_STEP>,
}

impl PlantDiagnostics {
    pub const fn empty() -> Self {
        Self {
            last_step_us: Micros(0),
            validated_config: false,
            overflow_count: 0,
            events: FixedSlice::empty(DiagnosticEvent::empty()),
        }
    }

    pub fn push(
        &mut self,
        timestamp_us: Micros,
        kind: DiagnosticKind,
        cylinder: Option<CylinderIndex>,
    ) {
        if self
            .events
            .push(DiagnosticEvent {
                timestamp_us,
                kind,
                cylinder,
            })
            .is_err()
        {
            self.overflow_count = self.overflow_count.saturating_add(1);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn diagnostics_count_overflowed_events() {
        let mut diagnostics = PlantDiagnostics::empty();

        for i in 0..(MAX_DIAGNOSTIC_EVENTS_PER_STEP + 3) {
            diagnostics.push(Micros(i as u32), DiagnosticKind::CombustionMisfire, None);
        }

        assert_eq!(diagnostics.events.len(), MAX_DIAGNOSTIC_EVENTS_PER_STEP);
        assert_eq!(diagnostics.overflow_count, 3);
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct PlantStepOutput<const CYL: usize, const MAX_EDGES: usize, const MAX_EVENTS: usize> {
    pub sensors: SensorSnapshot,
    pub trigger_edges: FixedSlice<TriggerEdge, MAX_EDGES>,
    pub consumed_events: ConsumedEcuEvents<MAX_EVENTS>,
    pub combustion: CombustionFrame<CYL>,
    pub dyno: DynoFrame,
    pub knock: KnockFrame<CYL>,
    pub telemetry: TelemetryFrame<CYL>,
    pub diagnostics: PlantDiagnostics,
}

impl<const CYL: usize, const MAX_EDGES: usize, const MAX_EVENTS: usize>
    PlantStepOutput<CYL, MAX_EDGES, MAX_EVENTS>
{
    pub const fn empty() -> Self {
        Self {
            sensors: SensorSnapshot::empty(),
            trigger_edges: FixedSlice::empty(TriggerEdge::empty()),
            consumed_events: ConsumedEcuEvents::empty(),
            combustion: CombustionFrame::empty(),
            dyno: DynoFrame::empty(),
            knock: KnockFrame::empty(),
            telemetry: TelemetryFrame::empty(),
            diagnostics: PlantDiagnostics::empty(),
        }
    }

    pub fn clear_event_buffers(&mut self) {
        self.trigger_edges.clear();
        self.consumed_events = ConsumedEcuEvents::empty();
        self.combustion = CombustionFrame::empty();
        self.dyno = DynoFrame::empty();
        self.knock = KnockFrame::empty();
        self.telemetry = TelemetryFrame::empty();
        self.diagnostics = PlantDiagnostics::empty();
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PlantStepError {
    InvalidConfig(PlantConfigError),
    InvalidInput,
    OutputCapacityExceeded,
}
