#[cfg(test)]
mod tests {
    use super::super::*;
    use crate::FuelSensorInputs;
    use crate::{
        RuntimeSemanticAxis16, RuntimeSemanticCalibration, RuntimeSemanticCurve16U16,
        RuntimeSemanticDeadtimeTableU16, RuntimeSemanticState, RuntimeSemanticTable2dU16,
        RUNTIME_SEMANTIC_TABLE_LEN,
    };
    use ecu_control::{EnrichmentInputs, IgnitionInputs, LambdaTrimInputs, TorqueInputs};

    fn semantic_axis() -> RuntimeSemanticAxis16 {
        let mut values = [0u16; RUNTIME_SEMANTIC_TABLE_LEN];
        let mut i = 0;
        while i < RUNTIME_SEMANTIC_TABLE_LEN {
            values[i] = i as u16;
            i += 1;
        }
        RuntimeSemanticAxis16 {
            len: RUNTIME_SEMANTIC_TABLE_LEN as u8,
            values,
        }
    }

    fn semantic_curve_u16(value: u16) -> RuntimeSemanticCurve16U16 {
        RuntimeSemanticCurve16U16 {
            axis: semantic_axis(),
            values: [value; RUNTIME_SEMANTIC_TABLE_LEN],
        }
    }

    fn semantic_ae_freeze_calibration() -> RuntimeSemanticCalibration {
        let axis = semantic_axis();
        RuntimeSemanticCalibration {
            ve_table: RuntimeSemanticTable2dU16 {
                rpm_axis: axis,
                load_axis: axis,
                values: [[7000; RUNTIME_SEMANTIC_TABLE_LEN]; RUNTIME_SEMANTIC_TABLE_LEN],
            },
            afr_target_table: RuntimeSemanticTable2dU16 {
                rpm_axis: axis,
                load_axis: axis,
                values: [[1470; RUNTIME_SEMANTIC_TABLE_LEN]; RUNTIME_SEMANTIC_TABLE_LEN],
            },
            deadtime_table_us: RuntimeSemanticDeadtimeTableU16 {
                vbat_mv_axis: axis,
                pressure_kpa10_axis: axis,
                values: [[0; RUNTIME_SEMANTIC_TABLE_LEN]; RUNTIME_SEMANTIC_TABLE_LEN],
            },
            clt_corr_curve: semantic_curve_u16(1000),
            iat_corr_curve: semantic_curve_u16(1000),
            baro_corr_curve: semantic_curve_u16(1000),
            vbat_corr_curve: semantic_curve_u16(1000),
            cranking_curve: semantic_curve_u16(1000),
            afterstart_table: RuntimeSemanticTable2dU16 {
                rpm_axis: axis,
                load_axis: axis,
                values: [[1000; RUNTIME_SEMANTIC_TABLE_LEN]; RUNTIME_SEMANTIC_TABLE_LEN],
            },
            warmup_curve: semantic_curve_u16(1000),
            ae_tps_threshold_curve: semantic_curve_u16(1),
            ae_map_threshold_curve: semantic_curve_u16(1),
            ae_shot_curve_us: semantic_curve_u16(250),
            ae_decay_steps_curve: semantic_curve_u16(2),
            ae_decay_ratio_curve_x1000: semantic_curve_u16(1000),
            required_fuel_us: 1000,
            pref_kpa10: 1000,
            stoich_afr_x100: 1470,
            pw_max_us: 20_000,
            afterstart_window_cycles: 0,
            dfco_entry_rpm: 9_000,
            dfco_exit_rpm: 8_900,
            dfco_entry_tps_x100: 1,
            dfco_exit_tps_x100: 2,
            dfco_entry_map_kpa10: 20,
            dfco_delay_cycles: 1,
            soft_rev_rpm: 9_000,
            hard_rev_rpm: 10_000,
            rev_hysteresis_rpm: 100,
            soft_retard_max_deg10: 0,
            idle_target_rpm: 0,
            idle_base_duty_x1000: 0,
            idle_kp_x1000: 0,
            idle_ki_x1000: 0,
            launch_rpm_limit: 9_000,
            launch_cut_cycles: 0,
            flat_shift_rpm_min: 9_000,
            flat_shift_cut_cycles: 0,
            knock_threshold_x100: 10_000,
            knock_retard_step_deg10: 0,
            knock_retard_max_deg10: 0,
            knock_recovery_step_deg10: 0,
            knock_recovery_delay_cycles: 0,
            lambda_kp_x1000: 0,
            lambda_ki_x1000: 0,
        }
    }

    fn semantic_afterstart_calibration(
        window_cycles: u16,
        correction_x1000: u16,
    ) -> RuntimeSemanticCalibration {
        let mut calibration = semantic_ae_freeze_calibration();
        calibration.afterstart_table = RuntimeSemanticTable2dU16 {
            rpm_axis: semantic_axis(),
            load_axis: semantic_axis(),
            values: [[correction_x1000; RUNTIME_SEMANTIC_TABLE_LEN]; RUNTIME_SEMANTIC_TABLE_LEN],
        };
        calibration.afterstart_window_cycles = window_cycles;
        calibration.ae_tps_threshold_curve = semantic_curve_u16(u16::MAX);
        calibration.ae_map_threshold_curve = semantic_curve_u16(u16::MAX);
        calibration.ae_shot_curve_us = semantic_curve_u16(0);
        calibration.ae_decay_steps_curve = semantic_curve_u16(0);
        calibration
    }

    fn semantic_warmup_calibration(correction_x1000: u16) -> RuntimeSemanticCalibration {
        let mut calibration = semantic_ae_freeze_calibration();
        calibration.warmup_curve = crate::RuntimeSemanticCurve16U16 {
            axis: semantic_axis(),
            values: [correction_x1000; RUNTIME_SEMANTIC_TABLE_LEN],
        };
        calibration
    }

    fn test_fuel_model() -> BaseFuelModel {
        let rpm_bins = [
            Rpm::new(500),
            Rpm::new(1000),
            Rpm::new(1500),
            Rpm::new(2000),
            Rpm::new(2500),
            Rpm::new(3000),
            Rpm::new(3500),
            Rpm::new(4000),
            Rpm::new(4500),
            Rpm::new(5000),
            Rpm::new(5500),
            Rpm::new(6000),
            Rpm::new(6500),
            Rpm::new(7000),
            Rpm::new(7500),
            Rpm::new(8000),
        ];
        let load_bins = [
            Kpa10::new(200),
            Kpa10::new(300),
            Kpa10::new(400),
            Kpa10::new(500),
            Kpa10::new(600),
            Kpa10::new(700),
            Kpa10::new(800),
            Kpa10::new(900),
            Kpa10::new(1000),
            Kpa10::new(1100),
            Kpa10::new(1200),
            Kpa10::new(1300),
            Kpa10::new(1400),
            Kpa10::new(1500),
            Kpa10::new(1600),
            Kpa10::new(1700),
        ];
        let mut pulse_widths = [[PulseWidthUs::new(0); 16]; 16];
        pulse_widths[5][5] = PulseWidthUs::new(2500);
        BaseFuelModel::new(rpm_bins, load_bins, pulse_widths)
    }

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
                now_us: Micros::new(now_us),
                clt_c: 80,
                just_started: false,
                lambda_valid: true,
                measured_lambda100: Lambda100::new(92),
                requested_open_loop: false,
            },
            torque: TorqueInputs::new(72, 84, 100, 100, 100),
            ignition: IgnitionInputs::new(Degrees10::new(120), 0, 0, 0, false, Rpm::new(rpm)),
            fuel_sensors: FuelSensorInputs::default(),
            knock_intensity_x100: 0,
        }
    }

    #[test]
    fn compose_control_uses_runtime_configured_control_configs() {
        let mut runtime = EngineRuntime::new();
        runtime.configure_fuel_model(test_fuel_model());
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
                safety_latch_request: false,
            },
            control_inputs(1_000, 3000),
        );

        assert_eq!(result.control.enrichment.warmup_x100, 160);
        assert_eq!(result.control.lambda.target_lambda100, Lambda100::new(105));
        assert_eq!(result.control.lambda.trim_x100, 113);
        assert_eq!(result.control.enriched_fuel, PulseWidthUs::new(4520));
        assert_eq!(result.control.ignition.dwell_us, DwellUs::new(3200));
    }

    #[test]
    fn compose_control_leaves_direct_pulse_width_untrimmed_when_lambda_is_open_loop() {
        let mut runtime = EngineRuntime::new();
        runtime.configure_fuel_model(test_fuel_model());
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

        let mut inputs = control_inputs(1_000, 3000);
        inputs.lambda.requested_open_loop = true;
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
                safety_latch_request: false,
            },
            inputs,
        );

        assert!(!result.control.lambda.active);
        assert_eq!(result.control.lambda.trim_x100, 100);
        assert_eq!(result.control.enriched_fuel, PulseWidthUs::new(4000));
    }

    #[test]
    fn compose_control_marks_low_load_as_open_loop_lambda_state() {
        let mut runtime = EngineRuntime::new();
        runtime.configure_fuel_model(test_fuel_model());
        runtime.configure_lambda_trim(LambdaTrimConfig {
            open_loop_target: Lambda100::new(98),
            enable_load_kpa10: Kpa10::new(300),
            disable_load_kpa10: Kpa10::new(250),
            ..LambdaTrimConfig::DEFAULT
        });

        let result = runtime.step(
            StepInputs {
                now_us: Micros::new(1_000),
                rpm: 3000,
                load_kpa10: 200,
                angle_x10: 1200,
                trigger_synced: true,
                cam_seen: true,
                launch_armed: false,
                flat_shift_armed: false,
                safety_latch_request: false,
            },
            control_inputs(1_000, 3000),
        );

        assert_eq!(result.control.lambda.mode, crate::LambdaMode::OpenLoop);
        assert!(!result.control.lambda.active);
        assert_eq!(
            result.control.lambda.disable_reason,
            crate::LambdaDisableReason::LowLoadGate
        );
        assert_eq!(result.control.lambda.target_lambda100, Lambda100::new(98));
    }

    #[test]
    fn compose_control_marks_power_reduction_cut_as_frozen_lambda_state() {
        for (fuel_cut, spark_cut) in [(true, false), (false, true)] {
            let mut runtime = EngineRuntime::new();
            runtime.configure_fuel_model(test_fuel_model());
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
            runtime.set_direct_cut_requests(fuel_cut, spark_cut);

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
                    safety_latch_request: false,
                },
                control_inputs(1_000, 3000),
            );

            assert_eq!(result.control.lambda.mode, crate::LambdaMode::ClosedLoop);
            assert!(!result.control.lambda.active);
            assert_eq!(
                result.control.lambda.disable_reason,
                crate::LambdaDisableReason::PowerReductionCut
            );
            assert_eq!(result.control.lambda.target_lambda100, Lambda100::new(105));
        }
    }

    #[test]
    fn compose_control_marks_semantic_ae_freeze_as_frozen_lambda_state() {
        let mut runtime = EngineRuntime::new();
        runtime.configure_speed_density_ve(
            semantic_ae_freeze_calibration(),
            RuntimeSemanticState::default(),
        );
        runtime.configure_lambda_trim(LambdaTrimConfig {
            closed_loop_target: Lambda100::new(105),
            min_trim_x100: 90,
            max_trim_x100: 130,
            gain_x10: 10,
            ..LambdaTrimConfig::DEFAULT
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
                safety_latch_request: false,
            },
            control_inputs(1_000, 3000),
        );

        assert!(!result.control.fuel_cut);
        assert!(!result.control.spark_cut);
        assert_eq!(result.control.lambda.mode, crate::LambdaMode::ClosedLoop);
        assert!(!result.control.lambda.active);
        assert_eq!(
            result.control.lambda.disable_reason,
            crate::LambdaDisableReason::AccelerationEnrichment
        );
        assert_eq!(result.control.lambda.target_lambda100, Lambda100::new(105));
        assert!(
            result
                .control
                .fuel_intent
                .observations
                .lambda_ae_freeze_active
        );
        assert_eq!(result.control.fuel_intent.observations.ae_pulse_us, 250);
        assert_eq!(
            result
                .control
                .fuel_intent
                .observations
                .ae_decay_steps_remaining,
            2
        );
    }

    #[test]
    fn compose_control_tracks_direct_afterstart_window_state() {
        let mut runtime = EngineRuntime::new();
        runtime.configure_fuel_model(test_fuel_model());

        let mut inputs = control_inputs(1_000, 3000);
        inputs.enrichment.just_started = true;
        let first = runtime.step(
            StepInputs {
                now_us: Micros::new(1_000),
                rpm: 3000,
                load_kpa10: 700,
                angle_x10: 1200,
                trigger_synced: true,
                cam_seen: true,
                launch_armed: false,
                flat_shift_armed: false,
                safety_latch_request: false,
            },
            inputs,
        );

        assert!(first.control.fuel_intent.observations.afterstart_active);
        assert_eq!(
            first
                .control
                .fuel_intent
                .observations
                .afterstart_window_remaining,
            5000
        );
        assert_eq!(
            first
                .control
                .fuel_intent
                .observations
                .afterstart_window_mode,
            ecu_control::FuelAfterstartWindowMode::Milliseconds
        );

        let second = runtime.step(
            StepInputs {
                now_us: Micros::new(2_001_000),
                rpm: 3000,
                load_kpa10: 700,
                angle_x10: 1200,
                trigger_synced: true,
                cam_seen: true,
                launch_armed: false,
                flat_shift_armed: false,
                safety_latch_request: false,
            },
            control_inputs(2_001_000, 3000),
        );

        assert!(second.control.fuel_intent.observations.afterstart_active);
        assert_eq!(
            second
                .control
                .fuel_intent
                .observations
                .afterstart_window_remaining,
            3000
        );
        assert_eq!(
            second
                .control
                .fuel_intent
                .observations
                .afterstart_window_mode,
            ecu_control::FuelAfterstartWindowMode::Milliseconds
        );
    }

    #[test]
    fn compose_control_tracks_startup_window_state() {
        let mut runtime = EngineRuntime::new();
        runtime.configure_fuel_model(test_fuel_model());

        let mut cranking_inputs = control_inputs(1_000, 3000);
        cranking_inputs.enrichment.cranking = true;
        let cranking = runtime.step(
            StepInputs {
                now_us: Micros::new(1_000),
                rpm: 3000,
                load_kpa10: 700,
                angle_x10: 1200,
                trigger_synced: true,
                cam_seen: true,
                launch_armed: false,
                flat_shift_armed: false,
                safety_latch_request: false,
            },
            cranking_inputs,
        );

        assert!(cranking.control.fuel_intent.observations.startup_active);
        assert_eq!(
            cranking
                .control
                .fuel_intent
                .observations
                .startup_window_remaining,
            3000
        );
        assert_eq!(
            cranking
                .control
                .fuel_intent
                .observations
                .startup_window_mode,
            ecu_control::FuelStartupWindowMode::Milliseconds
        );

        let taper = runtime.step(
            StepInputs {
                now_us: Micros::new(2_001_000),
                rpm: 3000,
                load_kpa10: 700,
                angle_x10: 1200,
                trigger_synced: true,
                cam_seen: true,
                launch_armed: false,
                flat_shift_armed: false,
                safety_latch_request: false,
            },
            control_inputs(2_001_000, 3000),
        );

        assert!(taper.control.fuel_intent.observations.startup_active);
        assert_eq!(
            taper
                .control
                .fuel_intent
                .observations
                .startup_window_remaining,
            1000
        );
        assert_eq!(
            taper.control.fuel_intent.observations.startup_window_mode,
            ecu_control::FuelStartupWindowMode::Milliseconds
        );
    }

    #[test]
    fn compose_control_tracks_warmup_temperature_state() {
        let mut runtime = EngineRuntime::new();
        runtime.configure_fuel_model(test_fuel_model());
        runtime.configure_warmup_enrichment(WarmupConfig {
            start_c: 0,
            end_c: 100,
            max_percent_x100: 150,
            min_percent_x100: 100,
        });

        let mut cold_inputs = control_inputs(1_000, 3000);
        cold_inputs.enrichment.clt_c = -10;
        let cold = runtime.step(
            StepInputs {
                now_us: Micros::new(1_000),
                rpm: 3000,
                load_kpa10: 700,
                angle_x10: 1200,
                trigger_synced: true,
                cam_seen: true,
                launch_armed: false,
                flat_shift_armed: false,
                safety_latch_request: false,
            },
            cold_inputs,
        );
        assert!(cold.control.fuel_intent.observations.warmup_active);
        assert_eq!(
            cold.control.fuel_intent.observations.warmup_correction_x100,
            150
        );
        assert_eq!(
            cold.control
                .fuel_intent
                .observations
                .warmup_temperature_mode,
            ecu_control::FuelWarmupTemperatureMode::ColdClamp
        );

        let mut interpolating_inputs = control_inputs(2_000, 3000);
        interpolating_inputs.enrichment.clt_c = 50;
        let interpolating = runtime.step(
            StepInputs {
                now_us: Micros::new(2_000),
                rpm: 3000,
                load_kpa10: 700,
                angle_x10: 1200,
                trigger_synced: true,
                cam_seen: true,
                launch_armed: false,
                flat_shift_armed: false,
                safety_latch_request: false,
            },
            interpolating_inputs,
        );
        assert!(interpolating.control.fuel_intent.observations.warmup_active);
        assert_eq!(
            interpolating
                .control
                .fuel_intent
                .observations
                .warmup_correction_x100,
            125
        );
        assert_eq!(
            interpolating
                .control
                .fuel_intent
                .observations
                .warmup_temperature_mode,
            ecu_control::FuelWarmupTemperatureMode::Interpolating
        );

        let mut hot_inputs = control_inputs(3_000, 3000);
        hot_inputs.enrichment.clt_c = 100;
        let hot = runtime.step(
            StepInputs {
                now_us: Micros::new(3_000),
                rpm: 3000,
                load_kpa10: 700,
                angle_x10: 1200,
                trigger_synced: true,
                cam_seen: true,
                launch_armed: false,
                flat_shift_armed: false,
                safety_latch_request: false,
            },
            hot_inputs,
        );
        assert!(!hot.control.fuel_intent.observations.warmup_active);
        assert_eq!(
            hot.control.fuel_intent.observations.warmup_correction_x100,
            100
        );
        assert_eq!(
            hot.control.fuel_intent.observations.warmup_temperature_mode,
            ecu_control::FuelWarmupTemperatureMode::HotClamp
        );
    }

    #[test]
    fn compose_control_tracks_semantic_warmup_correction_from_strategy_curve() {
        let mut runtime = EngineRuntime::new();
        runtime.configure_speed_density_ve(
            semantic_warmup_calibration(1200),
            RuntimeSemanticState::default(),
        );
        runtime.configure_warmup_enrichment(WarmupConfig {
            start_c: -20,
            end_c: 60,
            max_percent_x100: 140,
            min_percent_x100: 100,
        });

        let mut inputs = control_inputs(1_000, 3000);
        inputs.enrichment.clt_c = -10;
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
                safety_latch_request: false,
            },
            inputs,
        );

        assert!(result.control.fuel_intent.observations.warmup_active);
        assert_eq!(
            result
                .control
                .fuel_intent
                .observations
                .warmup_correction_x100,
            120
        );
        assert_eq!(
            result
                .control
                .fuel_intent
                .observations
                .warmup_temperature_mode,
            ecu_control::FuelWarmupTemperatureMode::ColdClamp
        );
    }

    #[test]
    fn compose_control_tracks_semantic_afterstart_cycle_state() {
        let mut runtime = EngineRuntime::new();
        runtime.configure_speed_density_ve(
            semantic_afterstart_calibration(3, 1200),
            RuntimeSemanticState::default(),
        );

        let first = runtime.step(
            StepInputs {
                now_us: Micros::new(1_000),
                rpm: 3000,
                load_kpa10: 700,
                angle_x10: 1200,
                trigger_synced: true,
                cam_seen: true,
                launch_armed: false,
                flat_shift_armed: false,
                safety_latch_request: false,
            },
            control_inputs(1_000, 3000),
        );
        assert!(first.control.fuel_intent.observations.afterstart_active);
        assert_eq!(
            first
                .control
                .fuel_intent
                .observations
                .afterstart_window_remaining,
            2
        );
        assert_eq!(
            first
                .control
                .fuel_intent
                .observations
                .afterstart_window_mode,
            ecu_control::FuelAfterstartWindowMode::Cycles
        );

        let second = runtime.step(
            StepInputs {
                now_us: Micros::new(2_000),
                rpm: 3000,
                load_kpa10: 700,
                angle_x10: 1200,
                trigger_synced: true,
                cam_seen: true,
                launch_armed: false,
                flat_shift_armed: false,
                safety_latch_request: false,
            },
            control_inputs(2_000, 3000),
        );
        assert!(second.control.fuel_intent.observations.afterstart_active);
        assert_eq!(
            second
                .control
                .fuel_intent
                .observations
                .afterstart_window_remaining,
            1
        );

        let third = runtime.step(
            StepInputs {
                now_us: Micros::new(3_000),
                rpm: 3000,
                load_kpa10: 700,
                angle_x10: 1200,
                trigger_synced: true,
                cam_seen: true,
                launch_armed: false,
                flat_shift_armed: false,
                safety_latch_request: false,
            },
            control_inputs(3_000, 3000),
        );
        assert!(!third.control.fuel_intent.observations.afterstart_active);
        assert_eq!(
            third
                .control
                .fuel_intent
                .observations
                .afterstart_window_remaining,
            0
        );
        assert_eq!(
            third
                .control
                .fuel_intent
                .observations
                .afterstart_window_mode,
            ecu_control::FuelAfterstartWindowMode::Inactive
        );
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
            false,
            false,
        );

        assert_eq!(snapshot.now_us, Micros::new(2_000));
        assert_eq!(snapshot.rpm, Rpm::new(4500));
        assert_eq!(snapshot.map_kpa10, Kpa10::new(930));
        assert_eq!(snapshot.load_kpa10, Kpa10::new(930));
        assert_eq!(snapshot.tps_x100, 84);
        assert_eq!(snapshot.maf_x100, 0);
        assert!(!snapshot.maf_valid);
        assert_eq!(snapshot.clt_c10, -100);
        assert_eq!(snapshot.iat_c10, 250);
        assert_eq!(snapshot.baro_kpa10, Kpa10::new(1010));
        assert!(!snapshot.baro_valid);
        assert_eq!(snapshot.vbatt_mv, 12_000);
        assert!(snapshot.lambda_valid);
        assert_eq!(snapshot.lambda_measured, Lambda100::new(92));
        assert!(!snapshot.requested_open_loop);
        assert!(!snapshot.launch_armed);
        assert!(!snapshot.flat_shift_armed);
    }

    #[test]
    fn fuel_input_snapshot_uses_explicit_fuel_sensor_inputs() {
        let runtime = EngineRuntime::new();
        let mut inputs = control_inputs(2_500, 4200);
        inputs.fuel_sensors = FuelSensorInputs {
            maf_valid: true,
            maf_x100: 555,
            iat_c10: 315,
            vbatt_mv: 13_200,
            baro_valid: true,
            baro_kpa10: Kpa10::new(980),
        };
        let snapshot = runtime.fuel_input_snapshot(
            &ValidatedInputs {
                rpm: Rpm::new(4200),
                load_kpa10: Kpa10::new(870),
                angle_x10: Degrees10::new(1200),
                clamped: false,
            },
            inputs,
            false,
            false,
        );

        assert!(snapshot.maf_valid);
        assert_eq!(snapshot.maf_x100, 555);
        assert_eq!(snapshot.iat_c10, 315);
        assert!(snapshot.baro_valid);
        assert_eq!(snapshot.baro_kpa10, Kpa10::new(980));
        assert_eq!(snapshot.vbatt_mv, 13_200);
    }
}
