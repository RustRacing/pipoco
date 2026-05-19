#![no_std]

pub mod air;
pub mod burn;
pub mod combustion;
pub mod config;
pub mod crank;
pub mod cycle;
pub mod dyno;
pub mod estimation;
pub mod exhaust;
pub mod fuel;
pub mod geometry;
pub mod io;
pub mod knock;
pub mod lambda;
pub mod losses;
pub mod pressure;
pub mod residual;
pub mod scenario;
pub mod sensors;
pub mod spark;
pub mod state;
pub mod step;
pub mod telemetry;
pub mod thermo;
pub mod trigger;
pub mod types;

pub use air::*;
pub use burn::*;
pub use combustion::*;
pub use config::*;
pub use crank::*;
pub use cycle::*;
pub use dyno::*;
pub use estimation::*;
pub use exhaust::*;
pub use fuel::*;
pub use geometry::*;
pub use io::*;
pub use knock::*;
pub use lambda::*;
pub use losses::*;
pub use pressure::*;
pub use residual::*;
pub use scenario::*;
pub use sensors::*;
pub use spark::*;
pub use state::*;
pub use step::*;
pub use telemetry::*;
pub use thermo::*;
pub use trigger::*;
pub use types::*;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Plant<const CYL: usize, const MAX_EDGES: usize, const MAX_EVENTS: usize> {
    pub config: PlantConfig<CYL>,
    pub state: PlantState<CYL>,
}

impl<const CYL: usize, const MAX_EDGES: usize, const MAX_EVENTS: usize>
    Plant<CYL, MAX_EDGES, MAX_EVENTS>
{
    pub const fn new(config: PlantConfig<CYL>) -> Self {
        Self {
            config,
            state: PlantState::new(),
        }
    }

    pub fn validate_config(config: &PlantConfig<CYL>) -> Result<(), PlantConfigError> {
        config.validate()
    }

    pub fn reset(&mut self, initial: InitialPlantState<CYL>) {
        self.state = PlantState::from_initial(initial);
    }

    pub fn step(
        &mut self,
        input: &PlantStepInput<CYL, MAX_EVENTS>,
        output: &mut PlantStepOutput<CYL, MAX_EDGES, MAX_EVENTS>,
    ) -> Result<(), PlantStepError> {
        self.config
            .validate()
            .map_err(PlantStepError::InvalidConfig)?;
        step::step_plant(&self.config, &mut self.state, input, output)
    }
}
