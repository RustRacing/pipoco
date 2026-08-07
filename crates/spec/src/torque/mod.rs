use crate::{EngineMode, InputSnapshot};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
/// Frozen-spec torque pipeline inputs.
///
/// These inputs and outputs belong to the spec/oracle side of the comparison.
/// Product runtime observations should not be conflated with this x1000
/// pipeline.
pub struct TorquePipelineInputs {
    pub request_x1000: u16,
    pub limiter_ceiling_x1000: u16,
    pub safety_latched: bool,
    pub fuel_cut: bool,
    pub spark_cut: bool,
    /// Rev hard limiter is active — forces torque_allowed to 0 (hard cut has
    /// no torque capacity since both fuel and spark are cut).
    pub rev_hard_active: bool,
    /// Launch limiter cut is active — zeroes torque_actuated since both fuel
    /// and spark are cut during the launch pull.
    pub launch_cut: bool,
    /// Flat-shift limiter cut is active — zeroes torque_actuated.
    pub flat_shift_cut: bool,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
/// Frozen-spec torque pipeline result.
///
/// The x1000 values here are comparison targets for conformance tests, not
/// product-owned runtime observations.
pub struct TorquePipelineResult {
    pub torque_request_x1000: u16,
    pub torque_allowed_x1000: u16,
    pub torque_actuated_x1000: u16,
    pub fuel_trim_x1000: u16,
    pub spark_trim_x1000: u16,
}

pub const TORQUE_SCALE_X1000_MAX: u16 = 1000;

#[must_use]
pub fn derive_torque_request_x1000(input: InputSnapshot) -> u16 {
    clamp_x1000(input.tps_x100 / 10)
}

#[must_use]
pub fn mode_limiter_ceiling_x1000(mode: EngineMode) -> u16 {
    match mode {
        EngineMode::Off | EngineMode::Shutdown => 0,
        EngineMode::Cranking | EngineMode::Running => TORQUE_SCALE_X1000_MAX,
    }
}

#[must_use]
pub fn torque_pipeline_step(inputs: TorquePipelineInputs) -> TorquePipelineResult {
    let torque_request_x1000 = clamp_x1000(inputs.request_x1000);
    let limiter_ceiling_x1000 = clamp_x1000(inputs.limiter_ceiling_x1000);

    // Stage 2: limiter stack output.
    let torque_allowed_x1000 = if inputs.rev_hard_active {
        // Hard rev cut zeroes torque_allowed so no torque capacity is reported.
        0
    } else {
        core::cmp::min(torque_request_x1000, limiter_ceiling_x1000)
    };

    // Stage 3 + 4: safety/cut clamps and final actuation.
    let any_cut = inputs.safety_latched
        || inputs.fuel_cut
        || inputs.spark_cut
        || inputs.launch_cut
        || inputs.flat_shift_cut;
    let torque_actuated_x1000 = if any_cut { 0 } else { torque_allowed_x1000 };

    // Stage 5: deterministic trim projection from actuated torque.
    let fuel_trim_x1000 = torque_actuated_x1000;
    let spark_trim_x1000 = torque_actuated_x1000;

    TorquePipelineResult {
        torque_request_x1000,
        torque_allowed_x1000,
        torque_actuated_x1000,
        fuel_trim_x1000,
        spark_trim_x1000,
    }
}

#[inline]
fn clamp_x1000(value: u16) -> u16 {
    core::cmp::min(value, TORQUE_SCALE_X1000_MAX)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{AfrOverride, Kpa10, Micros, Millivolts, Rpm, SyncState, TempC10};

    #[test]
    fn derive_request_maps_tps_x100_to_x1000_and_clamps() {
        let mut input = InputSnapshot {
            t_us: Micros::new(0),
            rpm: Rpm::new(0),
            map_kpa10: Kpa10::new(1000),
            load_kpa10: Kpa10::new(1000),
            tps_x100: 5370,
            clt_c10: TempC10::new(800),
            iat_c10: TempC10::new(200),
            baro_kpa10: Kpa10::new(1000),
            vbatt_mv: Millivolts::new(12_000),
            knock_intensity_x100: 0,
            launch_armed: false,
            flat_shift_armed: false,
            sync: SyncState::Synced,
            fuel_cut: false,
            spark_cut: false,
            mode: EngineMode::Running,
            target_afr_override_x100: AfrOverride::None,
        };

        assert_eq!(derive_torque_request_x1000(input), 537);

        input.tps_x100 = 20_000;
        assert_eq!(derive_torque_request_x1000(input), 1000);
    }

    #[test]
    fn pipeline_preserves_request_to_allowed_to_actuated_order() {
        let result = torque_pipeline_step(TorquePipelineInputs {
            request_x1000: 900,
            limiter_ceiling_x1000: 750,
            safety_latched: false,
            fuel_cut: false,
            spark_cut: false,
            rev_hard_active: false,
            launch_cut: false,
            flat_shift_cut: false,
        });

        assert_eq!(result.torque_request_x1000, 900);
        assert_eq!(result.torque_allowed_x1000, 750);
        assert_eq!(result.torque_actuated_x1000, 750);
        assert_eq!(result.fuel_trim_x1000, 750);
        assert_eq!(result.spark_trim_x1000, 750);
    }

    #[test]
    fn cuts_and_safety_force_actuated_to_zero_without_reordering_upstream() {
        let result = torque_pipeline_step(TorquePipelineInputs {
            request_x1000: 650,
            limiter_ceiling_x1000: 700,
            safety_latched: true,
            fuel_cut: false,
            spark_cut: false,
            rev_hard_active: false,
            launch_cut: false,
            flat_shift_cut: false,
        });

        assert_eq!(result.torque_request_x1000, 650);
        assert_eq!(result.torque_allowed_x1000, 650);
        assert_eq!(result.torque_actuated_x1000, 0);
        assert_eq!(result.fuel_trim_x1000, 0);
        assert_eq!(result.spark_trim_x1000, 0);
    }
}
