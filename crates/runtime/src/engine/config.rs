use super::*;

impl EngineRuntime {
    fn legacy_knock_reason_active(&self) -> bool {
        if self.knock_retard_deg10 > 0 {
            return true;
        }

        match &self.planners.fuel_strategy {
            RuntimeFuelStrategy::SpeedDensityVe { calibration, .. }
            | RuntimeFuelStrategy::AlphaN { calibration, .. }
            | RuntimeFuelStrategy::Maf { calibration, .. } => {
                self.knock_intensity_x100 >= calibration.knock_threshold_x100
            }
            RuntimeFuelStrategy::DirectPulseWidthTable(_) => false,
        }
    }

    pub fn new() -> Self {
        Self {
            engine: EngineState {
                sync: SyncState::Unsynced,
                engine_time_authority: EngineTimeAuthority::none(),
                phase: EnginePhase::Off,
                mode: ControlMode::OpenLoop,
                rpm: Rpm::new(0),
                load_kpa10: Kpa10::new(0),
                angle_x10: Degrees10::new(0),
            },
            control: ControlState {
                fuel_pulse_width: PulseWidthUs::new(0),
                ignition_advance: Degrees10::new(0),
                dwell: DwellUs::new(0),
                lambda_target: Lambda100::new(100),
                torque_limit_x100: 100,
            },
            faults: FaultState {
                fault: FaultCode::None,
                severity: FaultSeverity::Info,
                cancel_reason: CancelReason::Manual,
            },
            calibration: CalibrationState {
                active: CalibrationSnapshot::default(),
                staged_dirty: false,
            },
            scheduler: SchedulerState::new(),
            planners: ControlPlannerState::default(),
            runtime_snapshot: RuntimeSnapshot {
                engine: EngineState {
                    sync: SyncState::Unsynced,
                    engine_time_authority: EngineTimeAuthority::none(),
                    phase: EnginePhase::Off,
                    mode: ControlMode::OpenLoop,
                    rpm: Rpm::new(0),
                    load_kpa10: Kpa10::new(0),
                    angle_x10: Degrees10::new(0),
                },
                control: ControlState {
                    fuel_pulse_width: PulseWidthUs::new(0),
                    ignition_advance: Degrees10::new(0),
                    dwell: DwellUs::new(0),
                    lambda_target: Lambda100::new(100),
                    torque_limit_x100: 100,
                },
                faults: FaultState {
                    fault: FaultCode::None,
                    severity: FaultSeverity::Info,
                    cancel_reason: CancelReason::Manual,
                },
                calibration: CalibrationState {
                    active: CalibrationSnapshot::default(),
                    staged_dirty: false,
                },
                scheduler: SchedulerState::new(),
                output_profile: RuntimeOutputProfile::default(),
                fuel_strategy_mode: crate::RuntimeFuelStrategyMode::default(),
                rev_soft_active: false,
                rev_hard_active: false,
                launch_active: false,
                flat_shift_active: false,
                safety_latched: false,
                direct_fuel_cut_request: false,
                direct_spark_cut_request: false,
                fuel_cut: false,
                spark_cut: false,
                legacy_cut_reason_code: 0,
                knock_intensity_x100: 0,
                knock_retard_deg10: 0,
            },
            calibration_snapshot: CalibrationSnapshot::default(),
            output_profile: RuntimeOutputProfile::default(),
            rev_soft_active: false,
            rev_hard_active: false,
            launch_active: false,
            flat_shift_active: false,
            direct_fuel_cut_request: false,
            direct_spark_cut_request: false,
            safety_latched: false,
            fuel_cut: false,
            spark_cut: false,
            knock_intensity_x100: 0,
            knock_retard_deg10: 0,
        }
    }

    pub fn snapshot(&self) -> RuntimeSnapshot {
        self.runtime_snapshot
    }

    pub fn legacy_cut_flags(&self) -> RuntimeLegacyCutFlags {
        let paired_cut = self.direct_fuel_cut_request
            || self.direct_spark_cut_request
            || self.engine.mode == ControlMode::Shutdown
            || self.safety_latched
            || self.rev_hard_active
            || self.launch_active
            || self.flat_shift_active;

        if paired_cut {
            RuntimeLegacyCutFlags {
                fuel_cut: true,
                spark_cut: true,
            }
        } else {
            RuntimeLegacyCutFlags {
                fuel_cut: self.fuel_cut,
                spark_cut: self.spark_cut,
            }
        }
    }

    pub fn legacy_cut_reason_code(&self) -> u8 {
        if self.safety_latched
            || self.direct_fuel_cut_request
            || self.direct_spark_cut_request
            || self.engine.mode == ControlMode::Shutdown
        {
            1
        } else if self.rev_hard_active {
            2
        } else if self.launch_active {
            3
        } else if self.flat_shift_active {
            4
        } else if self.fuel_cut && !self.spark_cut {
            5
        } else if self.spark_cut {
            6
        } else if self.legacy_knock_reason_active() {
            7
        } else {
            0
        }
    }

    pub fn calibration_snapshot(&self) -> CalibrationSnapshot {
        self.calibration_snapshot
    }

    pub fn scheduler_state(&self) -> SchedulerState {
        self.scheduler
    }

    pub fn fuel_strategy(&self) -> &RuntimeFuelStrategy {
        &self.planners.fuel_strategy
    }

    pub fn set_direct_cut_requests(&mut self, fuel_cut_request: bool, spark_cut_request: bool) {
        self.direct_fuel_cut_request = fuel_cut_request;
        self.direct_spark_cut_request = spark_cut_request;
    }

    pub fn configure_fuel_model(&mut self, fuel_model: BaseFuelModel) {
        self.planners.fuel_strategy = RuntimeFuelStrategy::DirectPulseWidthTable(fuel_model);
    }

    pub fn configure_startup_enrichment(&mut self, config: StartupConfig) {
        self.planners.startup_config = config;
    }

    pub fn configure_warmup_enrichment(&mut self, config: WarmupConfig) {
        self.planners.warmup_config = config;
    }

    pub fn configure_after_start_enrichment(&mut self, config: AfterStartConfig) {
        self.planners.after_start_config = config;
    }

    pub fn configure_acceleration_enrichment(&mut self, config: AccelerationConfig) {
        self.planners.acceleration_config = config;
    }

    pub fn configure_lambda_trim(&mut self, config: LambdaTrimConfig) {
        self.planners.lambda_config = config;
    }

    pub fn configure_dwell(&mut self, config: DwellConfig) {
        self.planners.dwell_config = config;
    }

    pub fn configure_runtime_fuel_model(&mut self, strategy: RuntimeFuelStrategy) {
        self.planners.fuel_strategy = strategy;
    }

    pub fn configure_speed_density_ve(
        &mut self,
        calibration: RuntimeSemanticCalibration,
        state: RuntimeSemanticState,
    ) {
        self.configure_speed_density_ve_with_load_source(calibration, state);
    }

    pub fn configure_speed_density_ve_with_load_source(
        &mut self,
        calibration: RuntimeSemanticCalibration,
        state: RuntimeSemanticState,
    ) {
        self.planners.fuel_strategy = RuntimeFuelStrategy::SpeedDensityVe { calibration, state };
    }

    pub fn configure_alpha_n(
        &mut self,
        calibration: RuntimeSemanticCalibration,
        state: RuntimeSemanticState,
    ) {
        self.planners.fuel_strategy = RuntimeFuelStrategy::AlphaN { calibration, state };
    }

    pub fn configure_maf(
        &mut self,
        calibration: RuntimeSemanticCalibration,
        state: RuntimeSemanticState,
    ) {
        self.planners.fuel_strategy = RuntimeFuelStrategy::Maf { calibration, state };
    }

    pub(crate) fn semantic_input_for_strategy(
        input: FuelInputSnapshot,
        load_source: FuelLoadSource,
        sync: SyncState,
        mode: FuelEngineMode,
        safety_latch_request: bool,
    ) -> RuntimeSemanticInputSnapshot {
        let selected_load = match load_source {
            FuelLoadSource::Map => input.map_kpa10,
            FuelLoadSource::Tps => Kpa10::new(input.tps_x100),
        };
        RuntimeSemanticInputSnapshot {
            t_us: input.now_us,
            rpm: input.rpm,
            map_kpa10: input.map_kpa10,
            load_kpa10: selected_load,
            tps_x100: input.tps_x100,
            knock_intensity_x100: input.knock_intensity_x100,
            clt_c10: input.clt_c10,
            iat_c10: input.iat_c10,
            baro_kpa10: input.baro_kpa10,
            vbatt_mv: input.vbatt_mv,
            lambda_valid: input.lambda_valid,
            lambda_measured: input.lambda_measured,
            requested_open_loop: input.requested_open_loop,
            launch_armed: input.launch_armed,
            flat_shift_armed: input.flat_shift_armed,
            sync,
            fuel_cut: false,
            spark_cut: false,
            direct_fuel_cut_request: input.fuel_cut_request,
            direct_spark_cut_request: input.spark_cut_request,
            safety_latch_request,
            mode: match mode {
                FuelEngineMode::Off => RuntimeSemanticEngineMode::Off,
                FuelEngineMode::Cranking => RuntimeSemanticEngineMode::Cranking,
                FuelEngineMode::Running => RuntimeSemanticEngineMode::Running,
                FuelEngineMode::Shutdown => RuntimeSemanticEngineMode::Shutdown,
            },
            target_afr_override_x100: match input.target_afr_override_x100 {
                FuelAfrOverride::None => RuntimeSemanticAfrOverride::None,
                FuelAfrOverride::Some(v) => RuntimeSemanticAfrOverride::Some(v),
            },
        }
    }

    pub fn configure_output_profile(&mut self, profile: RuntimeOutputProfile) {
        self.output_profile = profile;
    }

    pub fn configure_crank_only_wasted_spark(&mut self, cylinder_count: u8) {
        self.configure_output_profile(RuntimeOutputProfile::crank_only_wasted_spark(
            cylinder_count,
        ));
    }

    pub fn configure_crank_only_single_coil(&mut self, cylinder_count: u8) {
        self.configure_output_profile(RuntimeOutputProfile::crank_only_single_coil(cylinder_count));
    }

    pub fn configure_single_point_injection(&mut self) {
        self.configure_output_profile(RuntimeOutputProfile::single_point_injection());
    }

    pub fn configure_batch_injection(&mut self, channel_count: u8) {
        self.configure_output_profile(RuntimeOutputProfile::batch_injection(channel_count));
    }

    pub fn configure_full_ecu(&mut self, profile: FullEcuOutputProfile) {
        self.configure_output_profile(RuntimeOutputProfile::full_ecu(profile));
    }

    pub fn output_profile(&self) -> RuntimeOutputProfile {
        self.output_profile
    }

    pub fn engine_time_authority(&self) -> EngineTimeAuthority {
        self.engine.engine_time_authority
    }

    pub fn try_set_engine_time_authority(
        &mut self,
        authority: EngineTimeAuthority,
    ) -> Result<(), RuntimeAuthorityError> {
        self.set_engine_time_authority_checked_inner(authority)?;
        self.refresh_snapshot();
        Ok(())
    }

    pub fn set_engine_time_authority(&mut self, authority: EngineTimeAuthority) {
        self.set_engine_time_authority_inner(authority);
        self.refresh_snapshot();
    }

    pub(super) fn set_engine_time_authority_inner(&mut self, authority: EngineTimeAuthority) {
        let authority = authority
            .validate()
            .map(|_| authority)
            .unwrap_or_else(|_| EngineTimeAuthority::none());
        self.apply_engine_time_authority(authority);
    }

    fn set_engine_time_authority_checked_inner(
        &mut self,
        authority: EngineTimeAuthority,
    ) -> Result<(), RuntimeAuthorityError> {
        authority
            .validate()
            .map_err(|reason| RuntimeAuthorityError { authority, reason })?;
        self.apply_engine_time_authority(authority);
        Ok(())
    }

    fn apply_engine_time_authority(&mut self, authority: EngineTimeAuthority) {
        self.engine.engine_time_authority = authority;
        self.engine.sync = authority.compatibility_summary();
        self.engine.phase = engine_phase_from_authority(authority, self.engine.rpm);
    }

    pub fn set_fault_state(
        &mut self,
        fault: FaultCode,
        severity: FaultSeverity,
        cancel_reason: CancelReason,
    ) {
        self.faults = FaultState {
            fault,
            severity,
            cancel_reason,
        };
        self.refresh_snapshot();
    }

    pub fn set_staged_dirty(&mut self, dirty: bool) {
        self.calibration.staged_dirty = dirty;
        self.refresh_snapshot();
    }

    pub fn apply_sensor_sample(&mut self, rpm: Rpm, load_kpa10: Kpa10, angle_x10: Degrees10) {
        self.engine.rpm = rpm;
        self.engine.load_kpa10 = load_kpa10;
        self.engine.angle_x10 = angle_x10;
        self.refresh_snapshot();
    }
}
