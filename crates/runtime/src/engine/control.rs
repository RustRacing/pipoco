use super::*;

impl EngineRuntime {
    pub(super) fn compose_control(
        &mut self,
        validated: &ValidatedInputs,
        inputs: ControlInputs,
    ) -> ControlPlan {
        let fuel_input = self.fuel_input_snapshot(validated, inputs);
        let fuel_intent = self.compute_fuel_intent(fuel_input);
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
        let lambda = self.planners.lambda.update(inputs.lambda, &lambda_config);
        let torque = self.planners.torque.evaluate(inputs.torque);
        let ignition = self.planners.ignition.plan(inputs.ignition, &dwell_config);
        let enriched_fuel = enrichment.apply_to(base_fuel);

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

    fn fuel_input_snapshot(
        &self,
        validated: &ValidatedInputs,
        inputs: ControlInputs,
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
            maf_x100: 0,
            clt_c10: inputs.enrichment.clt_c.saturating_mul(10),
            iat_c10: 250,
            baro_kpa10: Kpa10::new(1010),
            vbatt_mv: 12_000,
            lambda_valid: inputs.lambda.lambda_valid,
            lambda_measured: inputs.lambda.measured_lambda100,
            sync: self.engine.sync,
            mode: match self.engine.phase {
                EnginePhase::Off => FuelEngineMode::Off,
                EnginePhase::Cranking => FuelEngineMode::Cranking,
                EnginePhase::Running => FuelEngineMode::Running,
                EnginePhase::Stopping => FuelEngineMode::Shutdown,
            },
            fuel_cut_request: false,
            target_afr_override_x100: FuelAfrOverride::None,
        }
    }

    fn compute_fuel_intent(&mut self, input: FuelInputSnapshot) -> FuelIntent {
        let sync = input.sync;
        let mode = input.mode;
        match &mut self.planners.fuel_strategy {
            RuntimeFuelStrategy::DirectPulseWidthTable(fuel_model) => {
                let pw = fuel_model.calculate_base_fuel(input.rpm, input.load_kpa10);
                FuelIntent {
                    pulse_width_us: pw,
                    target_angle_deg10: self.engine.angle_x10,
                    fuel_cut: false,
                    spark_cut: false,
                    observations: FuelObservations {
                        ve_pct_x100: None,
                        target_afr_x100: None,
                        pw_base_us: pw.get(),
                        pw_corr_us: pw.get(),
                        strategy_is_direct_pw: true,
                    },
                }
            }
            RuntimeFuelStrategy::SpeedDensityVe { calibration, state } => {
                let semantic_input =
                    Self::semantic_input_for_strategy(input, FuelLoadSource::Map, sync, mode);
                match runtime_semantic_evaluate_fuel(calibration, semantic_input, *state) {
                    Ok(obs) => {
                        let pw = PulseWidthUs::new(obs.pw_corr_us.min(u32::from(u16::MAX)) as u16);
                        FuelIntent {
                            pulse_width_us: pw,
                            target_angle_deg10: self.engine.angle_x10,
                            fuel_cut: obs.fuel_cut,
                            spark_cut: obs.spark_cut,
                            observations: FuelObservations {
                                ve_pct_x100: Some(obs.ve_pct_x100),
                                target_afr_x100: Some(obs.target_afr_x100),
                                pw_base_us: obs.pw_base_us.min(u32::from(u16::MAX)) as u16,
                                pw_corr_us: pw.get(),
                                strategy_is_direct_pw: false,
                            },
                        }
                    }
                    Err(_) => FuelIntent {
                        pulse_width_us: PulseWidthUs::new(0),
                        target_angle_deg10: self.engine.angle_x10,
                        fuel_cut: true,
                        spark_cut: true,
                        observations: FuelObservations {
                            ve_pct_x100: None,
                            target_afr_x100: None,
                            pw_base_us: 0,
                            pw_corr_us: 0,
                            strategy_is_direct_pw: false,
                        },
                    },
                }
            }
            RuntimeFuelStrategy::AlphaN { calibration, state } => {
                let semantic_input =
                    Self::semantic_input_for_strategy(input, FuelLoadSource::Tps, sync, mode);
                match runtime_semantic_evaluate_fuel(calibration, semantic_input, *state) {
                    Ok(obs) => {
                        let pw = PulseWidthUs::new(obs.pw_corr_us.min(u32::from(u16::MAX)) as u16);
                        FuelIntent {
                            pulse_width_us: pw,
                            target_angle_deg10: self.engine.angle_x10,
                            fuel_cut: obs.fuel_cut,
                            spark_cut: obs.spark_cut,
                            observations: FuelObservations {
                                ve_pct_x100: Some(obs.ve_pct_x100),
                                target_afr_x100: Some(obs.target_afr_x100),
                                pw_base_us: obs.pw_base_us.min(u32::from(u16::MAX)) as u16,
                                pw_corr_us: pw.get(),
                                strategy_is_direct_pw: false,
                            },
                        }
                    }
                    Err(_) => FuelIntent {
                        pulse_width_us: PulseWidthUs::new(0),
                        target_angle_deg10: self.engine.angle_x10,
                        fuel_cut: true,
                        spark_cut: true,
                        observations: FuelObservations {
                            ve_pct_x100: None,
                            target_afr_x100: None,
                            pw_base_us: 0,
                            pw_corr_us: 0,
                            strategy_is_direct_pw: false,
                        },
                    },
                }
            }
            RuntimeFuelStrategy::Maf { calibration, state } => {
                let mut semantic_input =
                    Self::semantic_input_for_strategy(input, FuelLoadSource::Map, sync, mode);
                semantic_input.load_kpa10 = Kpa10::new(input.maf_x100);
                match runtime_semantic_evaluate_fuel(calibration, semantic_input, *state) {
                    Ok(obs) => {
                        let pw = PulseWidthUs::new(obs.pw_corr_us.min(u32::from(u16::MAX)) as u16);
                        FuelIntent {
                            pulse_width_us: pw,
                            target_angle_deg10: self.engine.angle_x10,
                            fuel_cut: obs.fuel_cut,
                            spark_cut: obs.spark_cut,
                            observations: FuelObservations {
                                ve_pct_x100: Some(obs.ve_pct_x100),
                                target_afr_x100: Some(obs.target_afr_x100),
                                pw_base_us: obs.pw_base_us.min(u32::from(u16::MAX)) as u16,
                                pw_corr_us: pw.get(),
                                strategy_is_direct_pw: false,
                            },
                        }
                    }
                    Err(_) => FuelIntent {
                        pulse_width_us: PulseWidthUs::new(0),
                        target_angle_deg10: self.engine.angle_x10,
                        fuel_cut: true,
                        spark_cut: true,
                        observations: FuelObservations {
                            ve_pct_x100: None,
                            target_afr_x100: None,
                            pw_base_us: 0,
                            pw_corr_us: 0,
                            strategy_is_direct_pw: false,
                        },
                    },
                }
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use ecu_control::{EnrichmentInputs, IgnitionInputs, LambdaTrimInputs, TorqueInputs};

    fn control_inputs(now_us: u32, rpm: u16) -> ControlInputs {
        ControlInputs {
            enrichment: EnrichmentInputs {
                now_us: Micros::new(now_us),
                clt_c: -10,
                cranking: false,
                just_started: false,
                tpsdot_pct_s: 0,
                mapdot_kpa_s: 0,
            },
            lambda: LambdaTrimInputs {
                clt_c: 80,
                lambda_valid: true,
                measured_lambda100: Lambda100::new(92),
                requested_open_loop: false,
            },
            torque: TorqueInputs::new(72, 84, 100, 100, 100),
            ignition: IgnitionInputs::new(Degrees10::new(120), 0, 0, 0, false, Rpm::new(rpm)),
        }
    }

    #[test]
    fn compose_control_uses_runtime_configured_control_configs() {
        let mut runtime = EngineRuntime::new();
        runtime.configure_warmup_enrichment(WarmupConfig {
            start_c: -20,
            end_c: 20,
            max_percent_x100: 180,
            min_percent_x100: 100,
        });
        runtime.configure_lambda_trim(LambdaTrimConfig {
            closed_loop_target: Lambda100::new(105),
            min_trim_x100: 90,
            max_trim_x100: 130,
            gain_x10: 10,
            ..LambdaTrimConfig::DEFAULT
        });
        runtime.configure_dwell(DwellConfig {
            base_dwell_us: 3200,
            min_dwell_us: 1200,
            max_dwell_us: 4200,
            rpm_dwell_trim_us: 0,
            rpm_trim_start: Rpm::new(1000),
            rpm_trim_end: Rpm::new(8000),
        });

        let result = runtime.step(
            StepInputs {
                now_us: Micros::new(1_000),
                rpm: 3000,
                load_kpa10: 700,
                angle_x10: 1200,
                trigger_synced: true,
                cam_seen: true,
                launch_armed: false,
                flat_shift_armed: false,
            },
            control_inputs(1_000, 3000),
        );

        assert_eq!(result.control.enrichment.warmup_x100, 160);
        assert_eq!(result.control.lambda.target_lambda100, Lambda100::new(105));
        assert_eq!(result.control.lambda.trim_x100, 113);
        assert_eq!(result.control.ignition.dwell_us, DwellUs::new(3200));
    }

    #[test]
    fn fuel_input_snapshot_uses_available_live_runtime_inputs() {
        let runtime = EngineRuntime::new();
        let inputs = control_inputs(2_000, 4500);
        let snapshot = runtime.fuel_input_snapshot(
            &ValidatedInputs {
                rpm: Rpm::new(4500),
                load_kpa10: Kpa10::new(930),
                angle_x10: Degrees10::new(2400),
                clamped: false,
            },
            inputs,
        );

        assert_eq!(snapshot.now_us, Micros::new(2_000));
        assert_eq!(snapshot.rpm, Rpm::new(4500));
        assert_eq!(snapshot.map_kpa10, Kpa10::new(930));
        assert_eq!(snapshot.load_kpa10, Kpa10::new(930));
        assert_eq!(snapshot.tps_x100, 84);
        assert_eq!(snapshot.clt_c10, -100);
        assert!(snapshot.lambda_valid);
        assert_eq!(snapshot.lambda_measured, Lambda100::new(92));
    }
}
