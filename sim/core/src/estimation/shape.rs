use crate::{estimation::types::*, pressure::bmep_bar_x100, types::*};

use super::energy::torque_from_cycle_energy;

pub fn cam_switch_state(
    model: SwitchedCamPhasingModel,
    rpm: Rpm,
    tps_x1000: u16,
) -> CamSwitchState {
    if tps_x1000 < model.min_tps_x1000 || rpm.0 < model.enable_min_rpm as u32 {
        return CamSwitchState::Off;
    }

    let transition = model.transition_width_rpm.max(1) as u32;
    let enable_min = model.enable_min_rpm as u32;
    let enable_max = model.enable_max_rpm as u32;
    if rpm.0 < enable_min.saturating_add(transition)
        || rpm.0 > enable_max.saturating_sub(transition)
    {
        if rpm.0 <= enable_max.saturating_add(transition) {
            return CamSwitchState::Transition;
        }
        return CamSwitchState::Off;
    }

    CamSwitchState::On
}

pub fn switched_cam_factor_x1000(model: SwitchedCamPhasingModel, rpm: Rpm, tps_x1000: u16) -> u16 {
    if tps_x1000 < model.min_tps_x1000 || rpm.0 < model.enable_min_rpm as u32 {
        return 1000;
    }

    let transition = model.transition_width_rpm.max(1) as u32;
    let enable_min = model.enable_min_rpm as u32;
    let enable_max = model.enable_max_rpm as u32;
    let low_full_rpm = enable_min.saturating_add(transition);
    let low_neutral_rpm = low_full_rpm.saturating_add(900);
    if rpm.0 < low_full_rpm {
        return interp_u16(
            1000,
            model.low_rpm_gain_x1000,
            rpm.0.saturating_sub(enable_min),
            transition,
        );
    }
    if rpm.0 <= low_neutral_rpm {
        return interp_u16(
            model.low_rpm_gain_x1000,
            1000,
            rpm.0.saturating_sub(low_full_rpm),
            low_neutral_rpm.saturating_sub(low_full_rpm).max(1),
        );
    }

    let plateau_start = low_neutral_rpm.saturating_add(500);
    if rpm.0 < plateau_start {
        return 1000;
    }

    let plateau_end = enable_max.saturating_sub(transition);
    if rpm.0 <= plateau_end {
        return model.midrange_plateau_x1000;
    }

    let off_rpm = enable_max.saturating_add(transition);
    if rpm.0 <= off_rpm {
        return interp_u16(
            model.midrange_plateau_x1000,
            1000,
            rpm.0.saturating_sub(plateau_end),
            off_rpm.saturating_sub(plateau_end).max(1),
        );
    }

    1000
}

pub fn intake_tuning_factor_x1000(model: IntakeTuningModel, rpm: Rpm) -> u16 {
    let low_end = 2500u32;
    let low_extra = if rpm.0 < low_end {
        model.low_speed_fill_gain_x1000 as u32 * (low_end - rpm.0) / low_end
    } else {
        0
    };

    let center = model.resonance_center_rpm as i32;
    let width = model.resonance_width_rpm.max(1) as i32;
    let distance = (rpm.0 as i32 - center).abs();
    let resonance_extra = if distance < width {
        model.resonance_gain_x1000 as i32 * (width - distance) / width
    } else {
        0
    }
    .max(0) as u32;

    (1000u32 + low_extra + resonance_extra).min(u16::MAX as u32) as u16
}

pub fn bte_shape_factor_x1000(model: BteShapeModel, rpm: Rpm) -> u16 {
    let min_bsfc = model.min_bsfc_rpm.max(1) as u32;
    let mid_cap = model.max_midrange_gain_x1000.max(1);
    if rpm.0 <= min_bsfc {
        return interp_u16(
            model.low_rpm_heat_loss_penalty_x1000,
            mid_cap,
            rpm.0,
            min_bsfc,
        );
    }

    let high_end = min_bsfc.saturating_add(3000);
    if rpm.0 >= high_end {
        return model.high_rpm_friction_penalty_x1000;
    }

    interp_u16(
        mid_cap,
        model.high_rpm_friction_penalty_x1000,
        rpm.0.saturating_sub(min_bsfc),
        high_end.saturating_sub(min_bsfc).max(1),
    )
}

pub fn apply_shape_factor_x1000(torque: TorqueNmX100, factor_x1000: u16) -> TorqueNmX100 {
    TorqueNmX100(
        (torque.0 as i64 * factor_x1000 as i64 / 1000).clamp(i32::MIN as i64, i32::MAX as i64)
            as i32,
    )
}

pub fn shaped_torque_point(input: ShapedTorqueInput) -> ShapedTorquePoint {
    let cam_switch_factor = switched_cam_factor_x1000(input.cam, input.rpm, input.tps_x1000);
    let intake_factor = intake_tuning_factor_x1000(input.intake, input.rpm);
    let bte_factor = bte_shape_factor_x1000(input.bte, input.rpm);
    let combined = (cam_switch_factor as u64)
        .saturating_mul(intake_factor as u64)
        .saturating_mul(bte_factor as u64)
        / 1_000_000;
    let torque =
        apply_shape_factor_x1000(input.torque_nm_x100, combined.min(u16::MAX as u64) as u16);

    ShapedTorquePoint {
        rpm: input.rpm,
        torque_nm_x100: torque,
        power_kw_x100: power_kw_x100(torque, input.rpm),
        bmep_bar_x100: bmep_bar_x100(torque, input.displacement_cc),
        cam_switch_state: cam_switch_state(input.cam, input.rpm, input.tps_x1000),
        cam_switch_factor_x1000: cam_switch_factor,
        intake_tuning_factor_x1000: intake_factor,
        bte_shape_factor_x1000: bte_factor,
    }
}

pub fn curve_shape_diagnostics(input: CurveShapeInput) -> CurveShapeDiagnostics {
    let reference_bmep = bmep_bar_x100(input.reference_torque_nm_x100, input.displacement_cc);
    let simulated_bmep = bmep_bar_x100(input.simulated_torque_nm_x100, input.displacement_cc);
    let fuel_mep = bmep_bar_x100(
        torque_from_cycle_energy(input.fuel_energy_per_cycle, 1000),
        input.displacement_cc,
    );

    CurveShapeDiagnostics {
        rpm: input.rpm,
        reference_torque_nm_x100: input.reference_torque_nm_x100,
        simulated_torque_nm_x100: input.simulated_torque_nm_x100,
        reference_bmep_bar_x100: reference_bmep,
        simulated_bmep_bar_x100: simulated_bmep,
        required_torque_multiplier_x1000: required_multiplier_x1000(
            input.reference_torque_nm_x100.0,
            input.simulated_torque_nm_x100.0,
        ),
        required_bmep_multiplier_x1000: required_multiplier_x1000(
            reference_bmep.0,
            simulated_bmep.0,
        ),
        ve_x1000: input.ve_x1000,
        bte_x1000: input.bte_x1000,
        fuel_mep_bar_x100: fuel_mep,
        cam_switch_state: input.cam_switch_state,
        cam_switch_factor_x1000: input.cam_switch_factor_x1000,
        intake_tuning_factor_x1000: input.intake_tuning_factor_x1000,
        bte_shape_factor_x1000: input.bte_shape_factor_x1000,
    }
}

pub(super) fn power_kw_x100(torque: TorqueNmX100, rpm: Rpm) -> i32 {
    if torque.0 <= 0 || rpm.0 == 0 {
        return 0;
    }
    (torque.0 as i64 * rpm.0 as i64 / 9549) as i32
}

fn interp_u16(start: u16, end: u16, travel: u32, span: u32) -> u16 {
    if span == 0 {
        return end;
    }
    let travel = travel.min(span) as i64;
    let span = span as i64;
    let start = start as i64;
    let end = end as i64;
    (start + (end - start) * travel / span).clamp(0, u16::MAX as i64) as u16
}

fn required_multiplier_x1000(reference: i32, simulated: i32) -> u16 {
    if reference <= 0 || simulated <= 0 {
        return 0;
    }
    (reference as i64 * 1000 / simulated as i64).clamp(0, u16::MAX as i64) as u16
}
