use super::*;

fn apply_lambda_trim(base: PulseWidthUs, trim_x100: i16) -> PulseWidthUs {
    if trim_x100 <= 0 {
        return PulseWidthUs::new(0);
    }

    let scaled = (base.get() * trim_x100 as u32) / 100;
    PulseWidthUs::new(scaled)
}

fn lambda_trim_to_x1000(trim_x100: i16) -> u16 {
    if trim_x100 <= 0 {
        0
    } else {
        (u32::try_from(trim_x100).unwrap_or(0) * 10).min(u32::from(u16::MAX)) as u16
    }
}

fn direct_acceleration_enrichment_pulse_us(
    base_fuel: PulseWidthUs,
    enrichment: ecu_control::EnrichmentResult,
) -> u16 {
    let without_accel = ecu_control::EnrichmentResult::new(
        enrichment.startup_x100,
        enrichment.warmup_x100,
        enrichment.after_start_x100,
        100,
    )
    .apply_to(base_fuel);
    enrichment
        .apply_to(base_fuel)
        .get()
        .saturating_sub(without_accel.get()) as u16
}

fn direct_afterstart_observations(
    state: ecu_control::AfterStartState,
    now_us: Micros,
    cfg: &ecu_control::AfterStartConfig,
) -> (bool, u16, ecu_control::FuelAfterstartWindowMode) {
    if !state.active() {
        return (false, 0, ecu_control::FuelAfterstartWindowMode::Inactive);
    }

    (
        true,
        state.remaining_ms(now_us, cfg),
        ecu_control::FuelAfterstartWindowMode::Milliseconds,
    )
}

fn semantic_afterstart_observations(
    window_cycles: u16,
    state: RuntimeSemanticState,
) -> (bool, u16, ecu_control::FuelAfterstartWindowMode) {
    if window_cycles == 0 {
        return (false, 0, ecu_control::FuelAfterstartWindowMode::Inactive);
    }

    let consumed = state.afterstart_cycle_count.min(u32::from(window_cycles)) as u16;
    let remaining = window_cycles.saturating_sub(consumed);
    if remaining == 0 {
        return (false, 0, ecu_control::FuelAfterstartWindowMode::Inactive);
    }

    (
        true,
        remaining,
        ecu_control::FuelAfterstartWindowMode::Cycles,
    )
}

fn startup_observations(
    state: ecu_control::StartupState,
    now_us: Micros,
    cfg: &ecu_control::StartupConfig,
) -> (bool, u16, ecu_control::FuelStartupWindowMode) {
    if !state.active() {
        return (false, 0, ecu_control::FuelStartupWindowMode::Inactive);
    }

    (
        true,
        state.remaining_ms(now_us, cfg),
        ecu_control::FuelStartupWindowMode::Milliseconds,
    )
}

fn warmup_observations(
    clt_c: i16,
    cfg: &ecu_control::WarmupConfig,
) -> (bool, u16, ecu_control::FuelWarmupTemperatureMode) {
    let correction_x100 = cfg.compute_percent_x100(clt_c);
    let temperature_mode = cfg.temperature_mode(clt_c);
    let active = correction_x100 != 100;
    (active, correction_x100, temperature_mode)
}

fn semantic_warmup_observations(
    curve: &crate::RuntimeSemanticCurve16U16,
    clt_c10: i16,
    correction_x1000: u16,
) -> (bool, u16, ecu_control::FuelWarmupTemperatureMode) {
    let x = clt_c10.max(0) as u16;
    let len = curve.axis.len as usize;
    let temperature_mode = if len < 2 {
        ecu_control::FuelWarmupTemperatureMode::NeutralFallback
    } else if x <= curve.axis.values[0] {
        ecu_control::FuelWarmupTemperatureMode::ColdClamp
    } else if x >= curve.axis.values[len - 1] {
        ecu_control::FuelWarmupTemperatureMode::HotClamp
    } else {
        ecu_control::FuelWarmupTemperatureMode::Interpolating
    };
    let correction_x100 = ((u32::from(correction_x1000) + 5) / 10).min(u32::from(u16::MAX)) as u16;
    let active = correction_x1000 != 1000;
    (active, correction_x100, temperature_mode)
}

fn semantic_idle_observations(
    calibration: &RuntimeSemanticCalibration,
    obs: &crate::RuntimeSemanticFuelObservations,
) -> (bool, u16, i32, i32, i32, bool) {
    let active = calibration.idle_target_rpm != 0
        || calibration.idle_base_duty_x1000 != 0
        || calibration.idle_kp_x1000 != 0
        || calibration.idle_ki_x1000 != 0
        || obs.idle_duty_x1000 != 0
        || obs.idle_integrator_state.acc != 0;

    (
        active,
        obs.idle_duty_x1000,
        obs.idle_integrator_state.acc,
        obs.idle_integrator_state.min_acc,
        obs.idle_integrator_state.max_acc,
        obs.idle_integrator_state.frozen,
    )
}

fn semantic_lambda_correction_observations(
    obs: &crate::RuntimeSemanticFuelObservations,
) -> (u16, i32, i32, i32, bool) {
    (
        obs.lambda_correction_x1000,
        obs.lambda_integrator_state.acc,
        obs.lambda_integrator_state.min_acc,
        obs.lambda_integrator_state.max_acc,
        obs.lambda_integrator_state.frozen,
    )
}

/// Build the product `FuelIntent` from one semantic fuel evaluation (review 003).
///
/// Shared by the `SpeedDensityVe`, `AlphaN`, and `Maf` strategy arms so the
/// observation-mapping implementation exists once.
fn semantic_fuel_intent(
    calibration: &crate::RuntimeSemanticCalibration,
    input: &FuelInputSnapshot,
    target_angle_deg10: Degrees10,
    obs: crate::RuntimeSemanticFuelObservations,
    next_state: RuntimeSemanticState,
) -> FuelIntent {
    let pw = PulseWidthUs::new(obs.pw_corr_us);
    let (warmup_active, warmup_correction_x100, warmup_temperature_mode) =
        semantic_warmup_observations(
            &calibration.warmup_curve,
            input.clt_c10,
            obs.warmup_corr_x1000,
        );
    let (afterstart_active, afterstart_window_remaining, afterstart_window_mode) =
        semantic_afterstart_observations(calibration.afterstart_window_cycles, next_state);
    let (
        idle_active,
        idle_duty_x1000,
        idle_integrator_acc,
        idle_integrator_min_acc,
        idle_integrator_max_acc,
        idle_integrator_frozen,
    ) = semantic_idle_observations(calibration, &obs);
    let (
        lambda_correction_x1000,
        lambda_integrator_acc,
        lambda_integrator_min_acc,
        lambda_integrator_max_acc,
        lambda_integrator_frozen,
    ) = semantic_lambda_correction_observations(&obs);
    FuelIntent {
        pulse_width_us: pw,
        target_angle_deg10,
        fuel_cut: obs.fuel_cut,
        spark_cut: obs.spark_cut,
        observations: FuelObservations {
            ve_pct_x100: Some(obs.ve_pct_x100),
            target_afr_x100: Some(obs.target_afr_x100),
            pw_base_us: obs.pw_base_us.min(u32::from(u16::MAX)) as u16,
            pw_air_us: Some(obs.pw_air_us.min(u32::from(u16::MAX)) as u16),
            pw_corr_us: pw.get() as u16,
            warmup_active,
            warmup_correction_x100,
            warmup_temperature_mode,
            startup_active: false,
            startup_window_remaining: 0,
            startup_window_mode: ecu_control::FuelStartupWindowMode::Inactive,
            afterstart_active,
            afterstart_window_remaining,
            afterstart_window_mode,
            lambda_ae_freeze_active: next_state.ae_active,
            ae_pulse_us: next_state.ae_pulse_us.min(u32::from(u16::MAX)) as u16,
            ae_decay_steps_remaining: next_state.ae_decay_steps_remaining,
            lambda_correction_x1000,
            lambda_integrator_acc,
            lambda_integrator_min_acc,
            lambda_integrator_max_acc,
            lambda_integrator_frozen,
            idle_active,
            idle_duty_x1000,
            idle_integrator_acc,
            idle_integrator_min_acc,
            idle_integrator_max_acc,
            idle_integrator_frozen,
            advance_deg10_trim: obs.advance_deg10_trim,
            strategy_is_direct_pw: false,
        },
    }
}

impl EngineRuntime {
    fn clear_semantic_runtime_flags(&mut self) {
        self.rev_soft_active = false;
        self.rev_hard_active = false;
        self.launch_active = false;
        self.flat_shift_active = false;
        self.knock_retard_deg10 = 0;
    }

    fn apply_semantic_runtime_flags(&mut self, state: RuntimeSemanticState) {
        self.rev_soft_active = state.rev_soft_active;
        self.rev_hard_active = state.rev_hard_active;
        self.launch_active = state.launch_active;
        self.flat_shift_active = state.flat_shift_active;
        self.safety_latched = state.safety_latched;
        self.knock_retard_deg10 = state.knock_retard_deg10;
    }

    fn apply_direct_runtime_flags(
        &mut self,
        mode: FuelEngineMode,
        safety_latch_request: bool,
    ) -> bool {
        self.rev_soft_active = false;
        self.rev_hard_active = false;
        self.launch_active = false;
        self.flat_shift_active = false;
        self.knock_retard_deg10 = 0;

        if self.safety_latched && matches!(mode, FuelEngineMode::Off) && !safety_latch_request {
            self.safety_latched = false;
        } else if safety_latch_request {
            self.safety_latched = true;
        }

        self.safety_latched
    }

    pub(super) fn compose_control(
        &mut self,
        validated: &ValidatedInputs,
        inputs: ControlInputs,
        launch_armed: bool,
        flat_shift_armed: bool,
        safety_latch_request: bool,
    ) -> ControlPlan {
        self.knock_intensity_x100 = inputs.knock_intensity_x100;
        let fuel_input =
            self.fuel_input_snapshot(validated, inputs, launch_armed, flat_shift_armed);
        let mut fuel_intent = self.compute_fuel_intent(fuel_input, safety_latch_request);
        let base_fuel = fuel_intent.pulse_width_us;
        let startup_config = self.planners.startup_config;
        let warmup_config = self.planners.warmup_config;
        let after_start_config = self.planners.after_start_config;
        let acceleration_config = self.planners.acceleration_config;
        let lambda_config = self.planners.lambda_config;
        let dwell_config = self.planners.dwell_config;
        let enrichment = self.planners.enrichment.update(
            inputs.enrichment,
            &startup_config,
            &warmup_config,
            &after_start_config,
            &acceleration_config,
        );
        let (startup_active, startup_window_remaining, startup_window_mode) = startup_observations(
            self.planners.enrichment.startup,
            inputs.enrichment.now_us,
            &startup_config,
        );
        if fuel_intent.observations.strategy_is_direct_pw {
            let (warmup_active, warmup_correction_x100, warmup_temperature_mode) =
                warmup_observations(inputs.enrichment.clt_c, &warmup_config);
            fuel_intent.observations.warmup_active = warmup_active;
            fuel_intent.observations.warmup_correction_x100 = warmup_correction_x100;
            fuel_intent.observations.warmup_temperature_mode = warmup_temperature_mode;
        }
        fuel_intent.observations.startup_active = startup_active;
        fuel_intent.observations.startup_window_remaining = startup_window_remaining;
        fuel_intent.observations.startup_window_mode = startup_window_mode;
        if fuel_intent.observations.strategy_is_direct_pw {
            let (afterstart_active, afterstart_window_remaining, afterstart_window_mode) =
                direct_afterstart_observations(
                    self.planners.enrichment.after_start,
                    inputs.enrichment.now_us,
                    &after_start_config,
                );
            fuel_intent.observations.afterstart_active = afterstart_active;
            fuel_intent.observations.afterstart_window_remaining = afterstart_window_remaining;
            fuel_intent.observations.afterstart_window_mode = afterstart_window_mode;
            fuel_intent.observations.lambda_ae_freeze_active = enrichment.acceleration_x100 > 100;
            fuel_intent.observations.ae_pulse_us =
                direct_acceleration_enrichment_pulse_us(base_fuel, enrichment);
            fuel_intent.observations.ae_decay_steps_remaining = 0;
        }
        let lambda = self.planners.lambda.update(
            inputs.lambda,
            &lambda_config,
            validated.load_kpa10,
            fuel_intent.observations.lambda_ae_freeze_active,
            fuel_intent.fuel_cut || fuel_intent.spark_cut,
        );
        if fuel_intent.observations.strategy_is_direct_pw {
            fuel_intent.observations.lambda_correction_x1000 =
                lambda_trim_to_x1000(lambda.trim_x100);
            fuel_intent.observations.lambda_integrator_acc = 0;
            fuel_intent.observations.lambda_integrator_min_acc = 0;
            fuel_intent.observations.lambda_integrator_max_acc = 0;
            fuel_intent.observations.lambda_integrator_frozen = false;
        }
        let mut torque = self.planners.torque.evaluate(inputs.torque);
        if self.rev_hard_active && torque.allowed_x100 != 0 {
            torque.allowed_x100 = 0;
            torque.reason = TorqueLimitReason::RevLimiter;
        }
        let ignition = self.planners.ignition.plan(inputs.ignition, &dwell_config);
        let enriched_fuel = if fuel_intent.observations.strategy_is_direct_pw {
            apply_lambda_trim(enrichment.apply_to(base_fuel), lambda.trim_x100)
        } else {
            base_fuel
        };

        self.control = ControlState {
            fuel_pulse_width: enriched_fuel,
            ignition_advance: ignition.advance_deg10,
            dwell: ignition.dwell_us,
            lambda_target: lambda.target_lambda100,
            torque_limit_x100: torque.allowed_x100,
        };

        ControlPlan {
            base_fuel,
            enriched_fuel,
            enrichment,
            lambda,
            torque,
            ignition,
            fuel_cut: fuel_intent.fuel_cut,
            spark_cut: fuel_intent.spark_cut,
            fuel_intent,
        }
    }

    pub(crate) fn fuel_input_snapshot(
        &self,
        validated: &ValidatedInputs,
        inputs: ControlInputs,
        launch_armed: bool,
        flat_shift_armed: bool,
    ) -> FuelInputSnapshot {
        FuelInputSnapshot {
            now_us: inputs.enrichment.now_us,
            rpm: validated.rpm,
            map_kpa10: validated.load_kpa10,
            load_kpa10: validated.load_kpa10,
            tps_x100: inputs
                .torque
                .driver_request_x100
                .max(inputs.torque.idle_request_x100),
            knock_intensity_x100: inputs.knock_intensity_x100,
            maf_x100: inputs.fuel_sensors.maf_x100,
            clt_c10: inputs.enrichment.clt_c.saturating_mul(10),
            iat_c10: inputs.fuel_sensors.iat_c10,
            baro_kpa10: if inputs.fuel_sensors.baro_valid {
                inputs.fuel_sensors.baro_kpa10
            } else {
                crate::FuelSensorInputs::explicit_substitutions().baro_kpa10
            },
            vbatt_mv: inputs.fuel_sensors.vbatt_mv,
            maf_valid: inputs.fuel_sensors.maf_valid,
            lambda_valid: inputs.lambda.lambda_valid,
            lambda_measured: inputs.lambda.measured_lambda100,
            requested_open_loop: inputs.lambda.requested_open_loop,
            baro_valid: inputs.fuel_sensors.baro_valid,
            sync: self.engine.sync,
            mode: match self.engine.phase {
                EnginePhase::Off => FuelEngineMode::Off,
                EnginePhase::Cranking => FuelEngineMode::Cranking,
                EnginePhase::Running => FuelEngineMode::Running,
                EnginePhase::Stopping => FuelEngineMode::Shutdown,
            },
            launch_armed,
            flat_shift_armed,
            fuel_cut_request: self.direct_fuel_cut_request,
            spark_cut_request: self.direct_spark_cut_request,
            target_afr_override_x100: FuelAfrOverride::None,
        }
    }

    fn semantic_fuel_cut_intent(&self) -> FuelIntent {
        FuelIntent {
            pulse_width_us: PulseWidthUs::new(0),
            target_angle_deg10: self.engine.angle_x10,
            fuel_cut: true,
            spark_cut: true,
            observations: FuelObservations {
                ve_pct_x100: None,
                target_afr_x100: None,
                pw_base_us: 0,
                pw_air_us: None,
                pw_corr_us: 0,
                warmup_active: false,
                warmup_correction_x100: 100,
                warmup_temperature_mode: ecu_control::FuelWarmupTemperatureMode::Inactive,
                startup_active: false,
                startup_window_remaining: 0,
                startup_window_mode: ecu_control::FuelStartupWindowMode::Inactive,
                afterstart_active: false,
                afterstart_window_remaining: 0,
                afterstart_window_mode: ecu_control::FuelAfterstartWindowMode::Inactive,
                lambda_ae_freeze_active: false,
                ae_pulse_us: 0,
                ae_decay_steps_remaining: 0,
                lambda_correction_x1000: 1000,
                lambda_integrator_acc: 0,
                lambda_integrator_min_acc: 0,
                lambda_integrator_max_acc: 0,
                lambda_integrator_frozen: false,
                idle_active: false,
                idle_duty_x1000: 0,
                idle_integrator_acc: 0,
                idle_integrator_min_acc: 0,
                idle_integrator_max_acc: 0,
                idle_integrator_frozen: false,
                advance_deg10_trim: 0,
                strategy_is_direct_pw: false,
            },
        }
    }

    fn compute_fuel_intent(
        &mut self,
        input: FuelInputSnapshot,
        safety_latch_request: bool,
    ) -> FuelIntent {
        let sync = input.sync;
        let mode = input.mode;
        if let RuntimeFuelStrategy::DirectPulseWidthTable(fuel_model) =
            self.planners.fuel_strategy.clone()
        {
            let safety_latched = self.apply_direct_runtime_flags(input.mode, safety_latch_request);
            let shutdown_cut = matches!(input.mode, FuelEngineMode::Shutdown)
                || self.engine.mode == ControlMode::Shutdown;
            let sync_loss_cut = matches!(
                self.engine.engine_time_authority.crank,
                CrankSyncState::SyncLost
            );
            let fuel_cut =
                input.fuel_cut_request || safety_latched || shutdown_cut || sync_loss_cut;
            let spark_cut =
                input.spark_cut_request || safety_latched || shutdown_cut || sync_loss_cut;
            let raw_pw = fuel_model.calculate_base_fuel(input.rpm, input.load_kpa10);
            let pw = if fuel_cut {
                PulseWidthUs::new(0)
            } else {
                raw_pw
            };
            return FuelIntent {
                pulse_width_us: pw,
                target_angle_deg10: self.engine.angle_x10,
                fuel_cut,
                spark_cut,
                observations: FuelObservations {
                    ve_pct_x100: None,
                    target_afr_x100: None,
                    pw_base_us: raw_pw.get() as u16,
                    pw_air_us: None,
                    pw_corr_us: pw.get() as u16,
                    warmup_active: false,
                    warmup_correction_x100: 100,
                    warmup_temperature_mode: ecu_control::FuelWarmupTemperatureMode::Inactive,
                    startup_active: false,
                    startup_window_remaining: 0,
                    startup_window_mode: ecu_control::FuelStartupWindowMode::Inactive,
                    afterstart_active: false,
                    afterstart_window_remaining: 0,
                    afterstart_window_mode: ecu_control::FuelAfterstartWindowMode::Inactive,
                    lambda_ae_freeze_active: false,
                    ae_pulse_us: 0,
                    ae_decay_steps_remaining: 0,
                    lambda_correction_x1000: 1000,
                    lambda_integrator_acc: 0,
                    lambda_integrator_min_acc: 0,
                    lambda_integrator_max_acc: 0,
                    lambda_integrator_frozen: false,
                    idle_active: false,
                    idle_duty_x1000: 0,
                    idle_integrator_acc: 0,
                    idle_integrator_min_acc: 0,
                    idle_integrator_max_acc: 0,
                    idle_integrator_frozen: false,
                    advance_deg10_trim: 0,
                    strategy_is_direct_pw: true,
                },
            };
        }

        let mut semantic_runtime_state = None;
        let fuel_intent = match &mut self.planners.fuel_strategy {
            RuntimeFuelStrategy::DirectPulseWidthTable(_) => unreachable!(),
            RuntimeFuelStrategy::SpeedDensityVe { calibration, state } => {
                let semantic_input = Self::semantic_input_for_strategy(
                    input,
                    FuelLoadSource::Map,
                    sync,
                    mode,
                    safety_latch_request,
                );
                match runtime_semantic_evaluate_fuel_with_state(calibration, semantic_input, *state)
                {
                    Ok((obs, next_state)) => {
                        *state = next_state;
                        semantic_runtime_state = Some(next_state);
                        semantic_fuel_intent(
                            calibration,
                            &input,
                            self.engine.angle_x10,
                            obs,
                            next_state,
                        )
                    }
                    Err(_) => self.semantic_fuel_cut_intent(),
                }
            }
            RuntimeFuelStrategy::AlphaN { calibration, state } => {
                let semantic_input = Self::semantic_input_for_strategy(
                    input,
                    FuelLoadSource::Tps,
                    sync,
                    mode,
                    safety_latch_request,
                );
                match runtime_semantic_evaluate_fuel_with_state(calibration, semantic_input, *state)
                {
                    Ok((obs, next_state)) => {
                        *state = next_state;
                        semantic_runtime_state = Some(next_state);
                        semantic_fuel_intent(
                            calibration,
                            &input,
                            self.engine.angle_x10,
                            obs,
                            next_state,
                        )
                    }
                    Err(_) => self.semantic_fuel_cut_intent(),
                }
            }
            RuntimeFuelStrategy::Maf { calibration, state } => {
                if !input.maf_valid {
                    self.semantic_fuel_cut_intent()
                } else {
                    let mut semantic_input = Self::semantic_input_for_strategy(
                        input,
                        FuelLoadSource::Map,
                        sync,
                        mode,
                        safety_latch_request,
                    );
                    semantic_input.load_kpa10 = Kpa10::new(input.maf_x100);
                    match runtime_semantic_evaluate_fuel_with_state(
                        calibration,
                        semantic_input,
                        *state,
                    ) {
                        Ok((obs, next_state)) => {
                            *state = next_state;
                            semantic_runtime_state = Some(next_state);
                            semantic_fuel_intent(
                                calibration,
                                &input,
                                self.engine.angle_x10,
                                obs,
                                next_state,
                            )
                        }
                        Err(_) => self.semantic_fuel_cut_intent(),
                    }
                }
            }
        };
        if let Some(state) = semantic_runtime_state {
            self.apply_semantic_runtime_flags(state);
        } else {
            self.clear_semantic_runtime_flags();
        }
        fuel_intent
    }
}
