use crate::{io::*, state::InitialPlantState, Plant};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ScenarioStep<const CYL: usize, const MAX_EVENTS: usize> {
    pub input: PlantStepInput<CYL, MAX_EVENTS>,
}

impl<const CYL: usize, const MAX_EVENTS: usize> ScenarioStep<CYL, MAX_EVENTS> {
    pub const fn new(input: PlantStepInput<CYL, MAX_EVENTS>) -> Self {
        Self { input }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ScenarioResult {
    pub steps_run: usize,
}

pub fn run_scenario<
    const CYL: usize,
    const MAX_EDGES: usize,
    const MAX_EVENTS: usize,
    const STEPS: usize,
>(
    plant: &mut Plant<CYL, MAX_EDGES, MAX_EVENTS>,
    initial: InitialPlantState<CYL>,
    steps: &[ScenarioStep<CYL, MAX_EVENTS>; STEPS],
    output: &mut PlantStepOutput<CYL, MAX_EDGES, MAX_EVENTS>,
) -> Result<ScenarioResult, PlantStepError> {
    plant.reset(initial);
    let mut i = 0;
    while i < STEPS {
        plant.step(&steps[i].input, output)?;
        i += 1;
    }
    Ok(ScenarioResult { steps_run: STEPS })
}
