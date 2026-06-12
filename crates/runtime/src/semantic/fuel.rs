use super::types::*;

// ---------------------------------------------------------------------------
// v9 Runtime Semantic Fuel Evaluator — pure, deterministic, no_std
// ---------------------------------------------------------------------------

#[inline]
pub(super) const fn clamp_u16_s(value: u16, lo: u16, hi: u16) -> u16 {
    if value < lo {
        lo
    } else if value > hi {
        hi
    } else {
        value
    }
}

#[inline]
const fn mul_div_floor_u64(num: u64, mul: u64, div: u64) -> u64 {
    match (num * mul).checked_div(div) {
        Some(value) => value,
        None => 0,
    }
}

#[inline]
fn mul_ratio_x1000_floor(value: u32, ratio_x1000: u32) -> u32 {
    ((value as u64) * (ratio_x1000 as u64) / 1000) as u32
}

/// Find the segment index for a clipped value using left-closed/right-open
/// intervals, with the final upper boundary treated as closed.
pub(super) fn semantic_find_segment(axis: &RuntimeSemanticAxis16, x: u16) -> usize {
    let len = axis.len as usize;
    if len < 2 {
        return 0;
    }
    let clipped = clamp_u16_s(x, axis.values[0], axis.values[len - 1]);
    let mut idx = 0usize;
    while idx + 1 < len {
        let lo = axis.values[idx];
        let hi = axis.values[idx + 1];
        let is_last = idx + 1 == len - 1;
        if clipped >= lo && (clipped < hi || (is_last && clipped == hi)) {
            return idx;
        }
        idx += 1;
    }
    len - 2
}

/// Linear interpolation for u16 (floor semantics).
fn semantic_lerp_u16(x0: u16, x1: u16, y0: u16, y1: u16, x: u16) -> u32 {
    if x1 <= x0 {
        return y0 as u32;
    }
    let x_clip = clamp_u16_s(x, x0, x1);
    let num = (x_clip - x0) as i64;
    let den = (x1 - x0) as i64;
    let delta = y1 as i64 - y0 as i64;
    let product = delta * num;
    let quotient = product / den;
    let remainder = product % den;
    let correction = if product < 0 && remainder != 0 { -1 } else { 0 };
    (y0 as i64 + quotient + correction) as u32
}

/// Bilinear interpolation on a 2D table (floor semantics).
pub(super) fn semantic_bilerp_u16(table: &RuntimeSemanticTable2dU16, rpm: u16, load: u16) -> u32 {
    let len_rpm = table.rpm_axis.len as usize;
    let len_load = table.load_axis.len as usize;
    if len_rpm < 2 || len_load < 2 {
        return table.values[0][0] as u32;
    }
    let rpm_idx = semantic_find_segment(&table.rpm_axis, rpm);
    let load_idx = semantic_find_segment(&table.load_axis, load);

    let rpm_lo = table.rpm_axis.values[rpm_idx];
    let rpm_hi = table.rpm_axis.values[(rpm_idx + 1).min(len_rpm - 1)];
    let load_lo = table.load_axis.values[load_idx];
    let load_hi = table.load_axis.values[(load_idx + 1).min(len_load - 1)];

    let v00 = table.values[load_idx][rpm_idx];
    let v01 = table.values[(load_idx + 1).min(len_load - 1)][rpm_idx];
    let v10 = table.values[load_idx][(rpm_idx + 1).min(len_rpm - 1)];
    let v11 = table.values[(load_idx + 1).min(len_load - 1)][(rpm_idx + 1).min(len_rpm - 1)];

    let interp_lo = semantic_lerp_u16(rpm_lo, rpm_hi, v00, v10, rpm);
    let interp_hi = semantic_lerp_u16(rpm_lo, rpm_hi, v01, v11, rpm);
    semantic_lerp_u16(load_lo, load_hi, interp_lo as u16, interp_hi as u16, load)
}

/// Curve lookup with clipping and left-closed/right-open semantics.
fn semantic_curve_lookup(curve: &RuntimeSemanticCurve16U16, x: u16) -> u32 {
    let len = curve.axis.len as usize;
    if len < 2 {
        return curve.values[0] as u32;
    }
    let idx = semantic_find_segment(&curve.axis, x);
    let next_idx = (idx + 1).min(len - 1);
    semantic_lerp_u16(
        curve.axis.values[idx],
        curve.axis.values[next_idx],
        curve.values[idx],
        curve.values[next_idx],
        x,
    )
}

/// Validate that an axis is strictly increasing and length is valid.
fn validate_axis(axis: &RuntimeSemanticAxis16) -> Result<(), RuntimeSemanticFuelError> {
    let len = axis.len as usize;
    if !(2..=RUNTIME_SEMANTIC_TABLE_LEN).contains(&len) {
        return Err(RuntimeSemanticFuelError::AxisLenInvalid);
    }
    let mut idx = 0usize;
    while idx + 1 < len {
        if axis.values[idx] >= axis.values[idx + 1] {
            return Err(RuntimeSemanticFuelError::AxisNotStrictlyIncreasing);
        }
        idx += 1;
    }
    Ok(())
}

/// Validate the calibration axes.
fn validate_calibration(cal: &RuntimeSemanticCalibration) -> Result<(), RuntimeSemanticFuelError> {
    validate_axis(&cal.ve_table.rpm_axis)?;
    validate_axis(&cal.ve_table.load_axis)?;
    validate_axis(&cal.afr_target_table.rpm_axis)?;
    validate_axis(&cal.afr_target_table.load_axis)?;
    validate_axis(&cal.deadtime_table_us.rpm_axis)?;
    validate_axis(&cal.deadtime_table_us.load_axis)?;
    validate_axis(&cal.clt_corr_curve.axis)?;
    validate_axis(&cal.iat_corr_curve.axis)?;
    validate_axis(&cal.baro_corr_curve.axis)?;
    validate_axis(&cal.vbat_corr_curve.axis)?;
    validate_axis(&cal.cranking_curve.axis)?;
    validate_axis(&cal.afterstart_table.rpm_axis)?;
    validate_axis(&cal.afterstart_table.load_axis)?;
    validate_axis(&cal.warmup_curve.axis)?;
    validate_axis(&cal.ae_tps_threshold_curve.axis)?;
    validate_axis(&cal.ae_map_threshold_curve.axis)?;
    validate_axis(&cal.ae_shot_curve_us.axis)?;
    validate_axis(&cal.ae_decay_steps_curve.axis)?;
    validate_axis(&cal.ae_decay_ratio_curve_x1000.axis)?;
    Ok(())
}

/// Evaluate AE pulse width and update AE state.
fn evaluate_ae(
    cal: &RuntimeSemanticCalibration,
    input: &RuntimeSemanticInputSnapshot,
    state: &mut RuntimeSemanticState,
) {
    let load_delta = input.load_kpa10.get() as i32 - state.last_valid_load_kpa10 as i32;
    let map_delta = input.map_kpa10.get() as i32 - state.last_valid_map_kpa10 as i32;
    let abs_load_delta = if load_delta < 0 {
        load_delta.saturating_neg() as u32
    } else {
        load_delta as u32
    };
    let abs_map_delta = if map_delta < 0 {
        map_delta.saturating_neg() as u32
    } else {
        map_delta as u32
    };

    let rpm = input.rpm.get();
    let ae_tps_threshold = semantic_curve_lookup(&cal.ae_tps_threshold_curve, rpm);
    let ae_map_threshold =
        semantic_curve_lookup(&cal.ae_map_threshold_curve, input.load_kpa10.get());

    let triggered = abs_load_delta >= ae_tps_threshold || abs_map_delta >= ae_map_threshold;

    if triggered {
        state.ae_pulse_us = semantic_curve_lookup(&cal.ae_shot_curve_us, input.load_kpa10.get());
        state.ae_decay_steps_remaining =
            semantic_curve_lookup(&cal.ae_decay_steps_curve, input.load_kpa10.get()) as u16;
        state.ae_active = true;
    } else if state.ae_decay_steps_remaining > 0 {
        let decay_index = state.ae_decay_steps_remaining - 1;
        let decay_ratio_x1000 = semantic_curve_lookup(&cal.ae_decay_ratio_curve_x1000, decay_index);
        state.ae_pulse_us = ((state.ae_pulse_us as u64) * (decay_ratio_x1000 as u64) / 1000) as u32;
        state.ae_decay_steps_remaining = decay_index;
    } else {
        state.ae_pulse_us = 0;
        state.ae_active = false;
    }
}

/// Hysteresis latch helper.
fn latch_with_hysteresis(current: bool, value: u16, threshold: u16, hysteresis: u16) -> bool {
    if current {
        value > threshold.saturating_sub(hysteresis)
    } else {
        value >= threshold
    }
}

/// Determine final cut flags using the v9 priority arbiter.
fn evaluate_cut_arbitration(
    cal: &RuntimeSemanticCalibration,
    input: &RuntimeSemanticInputSnapshot,
    state: &mut RuntimeSemanticState,
) -> (bool, bool) {
    // Priority 1: Safety latch
    let safety_latch = input.fuel_cut
        || input.spark_cut
        || matches!(input.mode, RuntimeSemanticEngineMode::Shutdown)
        || state.sensor_plausibility_latched;

    // Safety latch self-holding: clears when mode is Off AND cuts are false
    if state.safety_latched {
        if matches!(input.mode, RuntimeSemanticEngineMode::Off)
            && !input.fuel_cut
            && !input.spark_cut
            && !state.sensor_plausibility_latched
        {
            state.safety_latched = false;
        }
    } else if safety_latch {
        state.safety_latched = true;
    }

    if state.safety_latched {
        return (true, true);
    }

    // Priority 2: Hard rev limit
    state.rev_hard_active = latch_with_hysteresis(
        state.rev_hard_active,
        input.rpm.get(),
        cal.hard_rev_rpm,
        cal.rev_hysteresis_rpm,
    );
    if state.rev_hard_active {
        return (true, true);
    }

    // Priority 3: Launch cut
    if input.launch_armed && input.rpm.get() >= cal.launch_rpm_limit {
        if cal.launch_cut_cycles == 0 {
            return (true, true);
        }
        let phase = state.launch_cut_cycle_count % (cal.launch_cut_cycles + 1);
        if phase < cal.launch_cut_cycles {
            state.launch_active = true;
            return (true, true);
        }
    }

    // Priority 4: Flat-shift cut
    if input.flat_shift_armed && input.rpm.get() >= cal.flat_shift_rpm_min {
        if cal.flat_shift_cut_cycles == 0 {
            return (true, true);
        }
        let phase = state.flat_shift_cut_cycle_count % (cal.flat_shift_cut_cycles + 1);
        if phase < cal.flat_shift_cut_cycles {
            state.flat_shift_active = true;
            return (true, true);
        }
    }

    // Priority 5: DFCO (Running mode only)
    if matches!(input.mode, RuntimeSemanticEngineMode::Running) {
        if state.dfco_active {
            state.dfco_active =
                input.rpm.get() > cal.dfco_exit_rpm && input.tps_x100 <= cal.dfco_exit_tps_x100;
        } else if input.rpm.get() >= cal.dfco_entry_rpm
            && input.tps_x100 <= cal.dfco_entry_tps_x100
            && input.map_kpa10.get() <= cal.dfco_entry_map_kpa10
        {
            state.dfco_qualify_counter = state.dfco_qualify_counter.saturating_add(1);
            if state.dfco_qualify_counter >= cal.dfco_delay_cycles {
                state.dfco_active = true;
            }
        } else {
            state.dfco_qualify_counter = 0;
        }
        if state.dfco_active {
            return (true, false);
        }
    }

    // Priority 6: Soft rev spark cut (no fuel cut)
    state.rev_soft_active = latch_with_hysteresis(
        state.rev_soft_active,
        input.rpm.get(),
        cal.soft_rev_rpm,
        cal.rev_hysteresis_rpm,
    );
    if state.rev_soft_active {
        return (false, true);
    }

    // Priority 7: Knock (no cuts in v9)
    let knock_active = input.knock_intensity_x100 >= cal.knock_threshold_x100;
    if knock_active {
        state.knock_recovery_counter = 0;
        if state.knock_retard_deg10 < (cal.knock_retard_max_deg10 as i16) {
            let step = cal.knock_retard_step_deg10 as i16;
            state.knock_retard_deg10 = state.knock_retard_deg10.saturating_add(step);
            if state.knock_retard_deg10 > cal.knock_retard_max_deg10 as i16 {
                state.knock_retard_deg10 = cal.knock_retard_max_deg10 as i16;
            }
        }
    } else if state.knock_retard_deg10 > 0 {
        if state.knock_recovery_counter >= cal.knock_recovery_delay_cycles {
            let step = cal.knock_recovery_step_deg10 as i16;
            state.knock_retard_deg10 = state.knock_retard_deg10.saturating_sub(step);
            if state.knock_retard_deg10 < 0 {
                state.knock_retard_deg10 = 0;
            }
            state.knock_recovery_counter = 0;
        } else {
            state.knock_recovery_counter = state.knock_recovery_counter.saturating_add(1);
        }
    } else {
        state.knock_recovery_counter = 0;
    }

    // Priority 8: No cut
    (false, false)
}

fn semantic_knock_retard_for_trim(
    cal: &RuntimeSemanticCalibration,
    input: &RuntimeSemanticInputSnapshot,
    current_retard_deg10: i16,
    recovery_counter: u16,
) -> i16 {
    let detected = input.knock_intensity_x100 >= cal.knock_threshold_x100;
    if detected {
        let step = cal.knock_retard_step_deg10 as i16;
        let max = cal.knock_retard_max_deg10 as i16;
        let next = current_retard_deg10.saturating_add(step);
        if next > max {
            max
        } else {
            next
        }
    } else if current_retard_deg10 > 0 {
        if recovery_counter >= cal.knock_recovery_delay_cycles {
            let step = cal.knock_recovery_step_deg10 as i16;
            let next = current_retard_deg10.saturating_sub(step);
            if next < 0 {
                0
            } else {
                next
            }
        } else {
            current_retard_deg10
        }
    } else {
        0
    }
}

// --------------------------------------------------------------------------
// v11 Runtime Semantic Lambda PI Helpers
// --------------------------------------------------------------------------

/// Absolute value for i32 without using std.
#[inline]
fn semantic_abs_i32(value: i32) -> i32 {
    if value < 0 {
        value.saturating_neg()
    } else {
        value
    }
}

/// Apply deadband to raw lambda error.
#[inline]
fn semantic_effective_lambda_error(raw: i32) -> i32 {
    if semantic_abs_i32(raw) <= RUNTIME_SEMANTIC_LAMBDA_DEADBAND_X1000 {
        0
    } else {
        raw
    }
}

/// Saturating conversion from i32 to u16.
#[inline]
fn semantic_i32_to_u16_saturating(value: i32) -> u16 {
    if value < 0 {
        0
    } else if value > i32::from(u16::MAX) {
        u16::MAX
    } else {
        value as u16
    }
}

/// Floor-divide with correct negative-product semantics matching spec.
/// quotient = product / denom; if product < 0 and product % denom != 0, subtract 1.
fn semantic_mul_div_floor_i32(numer: i32, factor: i32, denom: i32) -> i32 {
    // Use i64 intermediates to avoid overflow in product
    let product = numer as i64 * factor as i64;
    let denom_i64 = denom as i64;
    if denom_i64 == 0 {
        return 0;
    }
    let quotient = product / denom_i64;
    let remainder = product % denom_i64;
    let correction = if product < 0 && remainder != 0 { -1 } else { 0 };
    (quotient + correction) as i32
}

/// Evaluate one step of the lambda closed-loop PI controller.
///
/// Returns `(lambda_correction_x1000, RuntimeSemanticPiIntegratorState)`.
///
/// The AE state used here must be the post-evaluate_ae state so that the
/// freeze gate uses the same `ae_active` as the spec oracle.
/// The `fuel_cut` and `spark_cut` parameters must be the post-arbiter values
/// (not raw input cuts) to match the spec oracle's freeze gate inputs.
pub(crate) fn runtime_semantic_lambda_step(
    cal: &RuntimeSemanticCalibration,
    input: &RuntimeSemanticInputSnapshot,
    fuel_cut: bool,
    spark_cut: bool,
    lambda_integrator_acc: i32,
    ae_active_after_eval: bool,
) -> (u16, RuntimeSemanticPiIntegratorState) {
    // Effective error is always 0 in v11 (frozen oracle path uses lambda_error_x1000=0)
    let error = semantic_effective_lambda_error(RUNTIME_SEMANTIC_LAMBDA_ERROR_X1000);

    // P term: floor(error * kp / 1000)
    let p_term = semantic_mul_div_floor_i32(error, cal.lambda_kp_x1000 as i32, 1000);

    // I step: floor(error * ki / 1000)
    let i_step = semantic_mul_div_floor_i32(error, cal.lambda_ki_x1000 as i32, 1000);

    let acc = lambda_integrator_acc;
    let corr_pre = 1000 + p_term + acc;

    // Freeze gate — uses post-arbiter cuts to match spec oracle
    let freeze_gate = input.clt_c10 < 700 || ae_active_after_eval || fuel_cut || spark_cut;

    // Anti-windup freeze
    let anti_windup_freeze = (corr_pre <= i32::from(RUNTIME_SEMANTIC_LAMBDA_CORR_MIN_X1000)
        && i_step < 0)
        || (corr_pre >= i32::from(RUNTIME_SEMANTIC_LAMBDA_CORR_MAX_X1000) && i_step > 0);

    let freeze = freeze_gate || anti_windup_freeze;

    let acc_next = if freeze {
        acc
    } else {
        let sum = acc.saturating_add(i_step);
        // Clamp to [RUNTIME_SEMANTIC_LAMBDA_MIN_ACC, RUNTIME_SEMANTIC_LAMBDA_MAX_ACC]
        sum.clamp(
            RUNTIME_SEMANTIC_LAMBDA_MIN_ACC,
            RUNTIME_SEMANTIC_LAMBDA_MAX_ACC,
        )
    };

    let correction_pre = 1000 + p_term + acc_next;
    let lambda_correction_x1000 = {
        let raw = semantic_i32_to_u16_saturating(correction_pre);
        raw.clamp(
            RUNTIME_SEMANTIC_LAMBDA_CORR_MIN_X1000,
            RUNTIME_SEMANTIC_LAMBDA_CORR_MAX_X1000,
        )
    };

    (
        lambda_correction_x1000,
        RuntimeSemanticPiIntegratorState {
            acc: acc_next,
            min_acc: RUNTIME_SEMANTIC_LAMBDA_MIN_ACC,
            max_acc: RUNTIME_SEMANTIC_LAMBDA_MAX_ACC,
            frozen: freeze,
        },
    )
}

/// The main v9 semantic fuel evaluator.
///
/// Pure, deterministic, no_std/no_alloc, Verus-friendly.
pub fn runtime_semantic_evaluate_fuel(
    cal: &RuntimeSemanticCalibration,
    input: RuntimeSemanticInputSnapshot,
    mut state: RuntimeSemanticState,
) -> Result<RuntimeSemanticFuelObservations, RuntimeSemanticFuelError> {
    // Validate calibration axes
    validate_calibration(cal)?;

    // VE lookup
    let ve_pct_x100 = semantic_bilerp_u16(&cal.ve_table, input.rpm.get(), input.load_kpa10.get());

    // Target AFR
    // Spec: override is returned directly (clamped to [500, 3000]), not via bilerp
    let target_afr_x100 = match input.target_afr_override_x100 {
        RuntimeSemanticAfrOverride::Some(v) => clamp_u16_s(v, 500, 3000) as u32,
        RuntimeSemanticAfrOverride::None => semantic_bilerp_u16(
            &cal.afr_target_table,
            input.rpm.get(),
            input.load_kpa10.get(),
        ),
    };

    if target_afr_x100 == 0 {
        return Err(RuntimeSemanticFuelError::ZeroTargetAfr);
    }

    // Base PW
    let pw_base_us =
        mul_div_floor_u64(cal.required_fuel_us as u64, ve_pct_x100 as u64, 10_000) as u32;

    // Air PW
    if cal.pref_kpa10 == 0 {
        return Err(RuntimeSemanticFuelError::ZeroPrefKpa);
    }
    let pw_air_us = mul_div_floor_u64(
        pw_base_us as u64,
        input.map_kpa10.get() as u64,
        cal.pref_kpa10 as u64,
    ) as u32;

    // Corrections
    let deadtime_us: u32 = semantic_bilerp_u16(
        &cal.deadtime_table_us,
        input.vbatt_mv,
        input.baro_kpa10.get(),
    );

    let clt_corr_x1000 = semantic_curve_lookup(&cal.clt_corr_curve, input.clt_c10.max(0) as u16);
    let iat_corr_x1000 = semantic_curve_lookup(&cal.iat_corr_curve, input.iat_c10.max(0) as u16);
    let baro_corr_x1000 = semantic_curve_lookup(&cal.baro_corr_curve, input.baro_kpa10.get());
    let vbat_corr_x1000 = semantic_curve_lookup(&cal.vbat_corr_curve, input.vbatt_mv);

    let cranking_corr_x1000 = if matches!(input.mode, RuntimeSemanticEngineMode::Cranking) {
        semantic_curve_lookup(&cal.cranking_curve, input.clt_c10.max(0) as u16)
    } else {
        1000
    };

    let afterstart_corr_x1000 = if matches!(input.mode, RuntimeSemanticEngineMode::Running)
        && state.afterstart_cycle_count <= cal.afterstart_window_cycles as u32
    {
        let cycles = state.afterstart_cycle_count.min(u16::MAX as u32) as u16;
        semantic_bilerp_u16(&cal.afterstart_table, cycles, input.clt_c10.max(0) as u16)
    } else {
        1000
    };

    let warmup_corr_x1000 = semantic_curve_lookup(&cal.warmup_curve, input.clt_c10.max(0) as u16);
    let afr_corr_x1000 =
        mul_div_floor_u64(cal.stoich_afr_x100 as u64, 1000, target_afr_x100 as u64) as u32;
    let trim_corr_x1000: u32 = 1000;

    // Evaluate AE first so ae_active state is available for lambda freeze gate.
    // This matches the spec oracle order: ae_step before lambda_step.
    evaluate_ae(cal, &input, &mut state);
    let ae_active_after_eval = state.ae_active;

    let knock_retard_before_arbiter = state.knock_retard_deg10;
    let knock_recovery_counter_before_arbiter = state.knock_recovery_counter;

    // Call cut arbitration ONCE and reuse the result.
    // Calling twice would mutate state between calls and change the second result.
    let (fuel_cut_post_arbiter, spark_cut_post_arbiter) =
        evaluate_cut_arbitration(cal, &input, &mut state);

    // Compute lambda PI correction using the v11 semantic lambda step.
    // The freeze gate uses post-arbiter cuts to match the spec oracle.
    let (lambda_correction_x1000, lambda_integrator_state) = runtime_semantic_lambda_step(
        cal,
        &input,
        fuel_cut_post_arbiter,
        spark_cut_post_arbiter,
        state.lambda_integrator_acc,
        ae_active_after_eval,
    );

    let lambda_corr_x1000 = lambda_correction_x1000 as u32;

    // Apply corrections in order
    let mut pw = pw_air_us;
    pw = mul_ratio_x1000_floor(pw, cranking_corr_x1000);
    pw = mul_ratio_x1000_floor(pw, afterstart_corr_x1000);
    pw = mul_ratio_x1000_floor(pw, warmup_corr_x1000);
    pw = mul_ratio_x1000_floor(pw, clt_corr_x1000);
    pw = mul_ratio_x1000_floor(pw, iat_corr_x1000);
    pw = mul_ratio_x1000_floor(pw, baro_corr_x1000);
    pw = mul_ratio_x1000_floor(pw, vbat_corr_x1000);
    pw = mul_ratio_x1000_floor(pw, afr_corr_x1000);
    pw = mul_ratio_x1000_floor(pw, lambda_corr_x1000);
    pw = mul_ratio_x1000_floor(pw, trim_corr_x1000);

    // Add AE pulse and deadtime (saturating)
    pw = pw.saturating_add(state.ae_pulse_us);
    pw = pw.saturating_add(deadtime_us);

    // Reuse the already-computed cut arbitration result
    let (fuel_cut, spark_cut) = (fuel_cut_post_arbiter, spark_cut_post_arbiter);
    let soft_rev_active_for_trim = latch_with_hysteresis(
        state.rev_soft_active,
        input.rpm.get(),
        cal.soft_rev_rpm,
        cal.rev_hysteresis_rpm,
    );
    let soft_rev_trim_deg10 = if soft_rev_active_for_trim {
        -(cal.soft_retard_max_deg10 as i16)
    } else {
        0
    };
    let knock_retard_for_trim = semantic_knock_retard_for_trim(
        cal,
        &input,
        knock_retard_before_arbiter,
        knock_recovery_counter_before_arbiter,
    );
    let advance_deg10_trim = soft_rev_trim_deg10.saturating_sub(knock_retard_for_trim);

    // If direct input fuel_cut, zero corrected PW
    let pw_corr_us = if fuel_cut || input.fuel_cut {
        0
    } else {
        pw.min(cal.pw_max_us)
    };

    Ok(RuntimeSemanticFuelObservations {
        ve_pct_x100: ve_pct_x100 as u16,
        target_afr_x100: target_afr_x100 as u16,
        pw_base_us,
        pw_air_us,
        pw_corr_us,
        fuel_cut,
        spark_cut,
        lambda_correction_x1000,
        lambda_integrator_state,
        advance_deg10_trim,
    })
}
