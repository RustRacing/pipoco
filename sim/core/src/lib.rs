#![no_std]
#![doc = "Engine plant simulator core."]
#![doc = ""]
pub(crate) mod air;
pub(crate) mod burn;
pub(crate) mod combustion;
pub mod config;
pub(crate) mod crank;
pub(crate) mod cycle;
pub(crate) mod dyno;
#[cfg(test)]
mod estimation;
pub(crate) mod exhaust;
pub mod fuel;
pub(crate) mod geometry;
pub mod io;
pub(crate) mod knock;
pub(crate) mod lambda;
pub(crate) mod losses;
pub(crate) mod pressure;
pub(crate) mod residual;
pub mod sensors;
pub mod spark;
pub mod state;
pub(crate) mod step;
pub(crate) mod telemetry;
pub(crate) mod thermo;
pub mod trigger;
pub mod types;

use crate::config::{PlantConfig as CorePlantConfig, PlantConfigError as CorePlantConfigError};
use crate::io::PlantStepError as CorePlantStepError;
use crate::state::PlantState as CorePlantState;

/// Stable facade for ordinary simulator users.
pub mod prelude {
    pub use crate::{
        config::{
            AirConfig, CrankConfig, DynoConfig, DynoMode, EngineGeometryConfig, FuelConfig,
            LoadMode, PlantConfig, PlantConfigError, PlantPhysicsMode, SensorConfig, SparkConfig,
            TriggerConfig,
        },
        io::{
            DiagnosticEvent, DiagnosticKind, DriverInput, EcuOutputFrame, EnvironmentInput,
            PlantDiagnostics, PlantStepError, PlantStepInput, PlantStepOutput,
        },
        sensors::SensorSnapshot,
        trigger::TriggerEdge,
        types::{
            BmepBarX100, Celsius10, CrankDeg10, CylinderIndex, Degrees10, FixedSlice, Kpa10,
            MassUg, MicrogramsPerMicros, Micros, Millis, Millivolts, Rpm, TorqueNmX100,
            MAX_CYLINDERS, MAX_DIAGNOSTIC_EVENTS_PER_STEP, MAX_ECU_EVENTS_PER_STEP,
            MAX_TRIGGER_EDGES_PER_STEP,
        },
        Plant,
    };
}

pub use crate::state::{InitialPlantState, MisfireReason};
pub use prelude::*;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Plant<const CYL: usize, const MAX_EDGES: usize, const MAX_EVENTS: usize> {
    config: CorePlantConfig<CYL>,
    state: CorePlantState<CYL>,
}

impl<const CYL: usize, const MAX_EDGES: usize, const MAX_EVENTS: usize>
    Plant<CYL, MAX_EDGES, MAX_EVENTS>
{
    pub const fn new(config: CorePlantConfig<CYL>) -> Self {
        Self {
            config,
            state: CorePlantState::new(),
        }
    }

    pub fn validate_config(config: &CorePlantConfig<CYL>) -> Result<(), CorePlantConfigError> {
        config.validate()
    }

    pub const fn config(&self) -> &CorePlantConfig<CYL> {
        &self.config
    }

    pub const fn state(&self) -> &CorePlantState<CYL> {
        &self.state
    }

    pub fn reset(&mut self, initial: InitialPlantState<CYL>) {
        self.state = CorePlantState::from_initial(initial);
    }

    pub fn step(
        &mut self,
        input: &PlantStepInput<CYL, MAX_EVENTS>,
        output: &mut PlantStepOutput<CYL, MAX_EDGES, MAX_EVENTS>,
    ) -> Result<(), CorePlantStepError> {
        self.config
            .validate()
            .map_err(PlantStepError::InvalidConfig)?;
        step::step_plant(&self.config, &mut self.state, input, output)
    }
}
