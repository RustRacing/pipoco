use super::types::RuntimeSemanticEngineMode;

// ---------------------------------------------------------------------------
// v11 Runtime Semantic Torque Evaluator
// ---------------------------------------------------------------------------

/// Input for the runtime semantic torque evaluator.
///
/// This is semantic-oracle scaffolding for FM0016 comparisons, not a product
/// runtime observation surface. Product torque observations still come from
/// `EngineRuntime::step`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct RuntimeSemanticTorqueInput {
    /// Throttle position sensor reading (x100).
    pub tps_x100: u16,
    /// Current engine mode.
    pub mode: RuntimeSemanticEngineMode,
    /// Fuel cut is active.
    pub fuel_cut: bool,
    /// Spark cut is active.
    pub spark_cut: bool,
    /// Safety latch is set.
    pub safety_latched: bool,
    /// Soft rev limiter is active.
    pub rev_soft_active: bool,
    /// Hard rev limiter is active.
    pub rev_hard_active: bool,
    /// Launch cut is active.
    pub launch_cut: bool,
    /// Flat-shift cut is active.
    pub flat_shift_cut: bool,
}

/// Output from the runtime semantic torque evaluator.
///
/// This is a test-only semantic mirror of the frozen torque pipeline. Do not
/// treat these values as product-owned runtime observations.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct RuntimeSemanticTorqueResult {
    /// Torque request (x1000) — from TPS input.
    pub torque_request_x1000: u16,
    /// Torque allowed (x1000) — after limiter ceiling.
    pub torque_allowed_x1000: u16,
    /// Torque actuated (x1000) — after cut gating.
    pub torque_actuated_x1000: u16,
    /// Fuel trim (x1000) — mirrors torque_actuated.
    pub fuel_trim_x1000: u16,
    /// Spark trim (x1000) — mirrors torque_actuated.
    pub spark_trim_x1000: u16,
}

/// Torque scale constant (x1000 max = 1.0).
pub const TORQUE_SCALE_X1000_MAX: u16 = 1000;

fn clamp_x1000(value: u16) -> u16 {
    core::cmp::min(value, TORQUE_SCALE_X1000_MAX)
}

/// Mode-based torque ceiling: Off/Shutdown → 0, Cranking/Running → 1000.
fn torque_mode_ceiling(mode: RuntimeSemanticEngineMode) -> u16 {
    match mode {
        RuntimeSemanticEngineMode::Off | RuntimeSemanticEngineMode::Shutdown => 0,
        RuntimeSemanticEngineMode::Cranking | RuntimeSemanticEngineMode::Running => {
            TORQUE_SCALE_X1000_MAX
        }
    }
}

/// Evaluate torque request/allowed/actuated using frozen oracle semantics.
///
/// This mirrors `spec_oracle::torque::torque_pipeline_step` exactly for
/// conformance scaffolding. The product runtime path remains separate.
#[must_use]
pub fn runtime_semantic_evaluate_torque(
    input: RuntimeSemanticTorqueInput,
) -> RuntimeSemanticTorqueResult {
    // Stage 1: request from TPS.
    // Keep the stage names aligned with the frozen oracle so fixture diffs are
    // easy to compare, but do not read this as a product path.
    let torque_request_x1000 = clamp_x1000(input.tps_x100 / 10);

    // Stage 2: limiter ceiling from mode + hard rev cut.
    let limiter_ceiling_x1000 = torque_mode_ceiling(input.mode);
    let torque_allowed_x1000 = if input.rev_hard_active {
        0 // hard rev cut zeroes allowed
    } else {
        core::cmp::min(torque_request_x1000, limiter_ceiling_x1000)
    };

    // Stage 3+4: cut gating.
    let any_cut = input.safety_latched
        || input.fuel_cut
        || input.spark_cut
        || input.launch_cut
        || input.flat_shift_cut;
    let torque_actuated_x1000 = if any_cut { 0 } else { torque_allowed_x1000 };

    // Stage 5: trim projection.
    let fuel_trim_x1000 = torque_actuated_x1000;
    let spark_trim_x1000 = torque_actuated_x1000;

    RuntimeSemanticTorqueResult {
        torque_request_x1000,
        torque_allowed_x1000,
        torque_actuated_x1000,
        fuel_trim_x1000,
        spark_trim_x1000,
    }
}
