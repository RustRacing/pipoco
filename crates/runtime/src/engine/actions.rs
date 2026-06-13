use super::*;

impl EngineRuntime {
    pub(super) fn emit_actions(
        &mut self,
        now_us: Micros,
        control: &ControlPlan,
    ) -> ActionBatch<RUNTIME_ACTION_CAP> {
        let mut actions = ActionBatch::new();

        if self.engine.mode == ControlMode::Shutdown
            || self.faults.fault == FaultCode::SafetyCut
            || self.faults.severity == FaultSeverity::Critical
        {
            self.scheduler.on_hard_safety_shutdown();
            let _ = actions.push(Action::CancelScheduler(CancelReason::SafetyShutdown));
        } else if self.engine.sync == SyncState::Unsynced && self.scheduler.is_armed() {
            self.scheduler.on_sync_loss();
            let _ = actions.push(Action::CancelScheduler(CancelReason::SyncLoss));
        } else if output_profile_requires_full_sequential_authority(self.output_profile)
            && !runtime_full_sequential_authorized(self.engine.engine_time_authority)
        {
            if self.scheduler.is_armed() {
                self.scheduler.on_sync_loss();
                let _ = actions.push(Action::CancelScheduler(CancelReason::SyncLoss));
            } else {
                let _ = actions.push(Action::Idle);
            }
        } else if matches!(self.engine.sync, SyncState::Locked { .. }) {
            self.scheduler.on_sync_recovered();
            match self.output_profile {
                RuntimeOutputProfile::LegacySingleChannel => {
                    let injection_end = Micros::new(
                        now_us
                            .get()
                            .saturating_add(control.enriched_fuel.get() as u32)
                            .max(now_us.get().saturating_add(2)),
                    );
                    let ignition_end = Micros::new(
                        now_us
                            .get()
                            .saturating_add(control.ignition.dwell_us.get() as u32)
                            .max(now_us.get().saturating_add(2)),
                    );
                    if let (Ok(injection), Ok(ignition)) = (
                        self.scheduler.schedule_injection(
                            now_us,
                            Micros::new(now_us.get().saturating_add(1)),
                            injection_end,
                            InjectionPlan {
                                output: ExclusiveChannel::new(
                                    OutputGroup::Injector,
                                    ChannelId::new(1),
                                ),
                                pulse_width: control.enriched_fuel,
                            },
                        ),
                        self.scheduler.schedule_ignition(
                            now_us,
                            Micros::new(now_us.get().saturating_add(1)),
                            ignition_end,
                            self.make_ignition_plan(control, now_us),
                        ),
                    ) {
                        let _ = actions.push(Action::ArmScheduler {
                            injection,
                            ignition,
                        });
                    } else {
                        self.scheduler.cancel_group(OutputGroup::Injector);
                        self.scheduler.cancel_group(OutputGroup::Ignition);
                        let _ = actions.push(Action::Idle);
                    }
                }
                RuntimeOutputProfile::IgnitionOnly(profile) => {
                    if self.engine.rpm.get() == 0 || control.spark_cut {
                        let _ = actions.push(Action::Idle);
                    } else {
                        let event_count = IgnitionScheduler::new(profile).event_count();
                        for event_index in 0..event_count {
                            let ignition =
                                self.make_ignition_only_plan(profile, event_index, control, now_us);
                            if let Ok(ignition) = self.scheduler.schedule_ignition(
                                now_us,
                                ignition.start_at,
                                ignition.end_at,
                                ignition.plan,
                            ) {
                                let _ = actions.push(Action::ArmIgnition(ignition));
                            }
                        }
                        if actions.is_empty() {
                            let _ = actions.push(Action::Idle);
                        }
                    }
                }
                RuntimeOutputProfile::InjectionOnly(profile) => {
                    if self.engine.rpm.get() == 0
                        || control.fuel_cut
                        || control.enriched_fuel.get() == 0
                    {
                        let _ = actions.push(Action::Idle);
                    } else {
                        let event_count = InjectionScheduler::new(profile).event_count();
                        for event_index in 0..event_count {
                            let injection = self.make_injection_only_plan(
                                profile,
                                event_index,
                                control,
                                now_us,
                            );
                            if let Ok(injection) = self.scheduler.schedule_injection(
                                now_us,
                                injection.start_at,
                                injection.end_at,
                                injection.plan,
                            ) {
                                let _ = actions.push(Action::ArmInjection(injection));
                            }
                        }
                        if actions.is_empty() {
                            let _ = actions.push(Action::Idle);
                        }
                    }
                }
                RuntimeOutputProfile::FullEcu(profile) => {
                    let event_count = profile.event_count();
                    if event_count == 0 {
                        let _ = actions.push(Action::Idle);
                    } else {
                        let cycle_slot_us = profile.cycle_slot_us(self.engine.rpm);
                        for slot in 0..event_count {
                            let slot_offset = cycle_slot_us.saturating_mul(slot as u32);
                            let injection_start = Micros::new(
                                now_us.get().saturating_add(slot_offset).saturating_add(1),
                            );
                            let injection_end = Micros::new(
                                injection_start
                                    .get()
                                    .saturating_add(control.enriched_fuel.get() as u32)
                                    .max(injection_start.get().saturating_add(1)),
                            );
                            let ignition_start = Micros::new(
                                now_us.get().saturating_add(slot_offset).saturating_add(1),
                            );
                            let ignition_end = Micros::new(
                                ignition_start
                                    .get()
                                    .saturating_add(control.ignition.dwell_us.get() as u32)
                                    .max(ignition_start.get().saturating_add(1)),
                            );
                            if let Ok(injection) = self.scheduler.schedule_injection(
                                now_us,
                                injection_start,
                                injection_end,
                                InjectionPlan {
                                    output: ExclusiveChannel::new(
                                        OutputGroup::Injector,
                                        profile.injector_channel(slot),
                                    ),
                                    pulse_width: control.enriched_fuel,
                                },
                            ) {
                                let _ = actions.push(Action::ArmInjection(injection));
                            }
                            if let Ok(ignition) = self.scheduler.schedule_ignition(
                                now_us,
                                ignition_start,
                                ignition_end,
                                ecu_scheduler::IgnitionPlan {
                                    output: ExclusiveChannel::new(
                                        OutputGroup::Ignition,
                                        profile.ignition_channel(slot),
                                    ),
                                    dwell: control.ignition.dwell_us,
                                    advance: control.ignition.advance_deg10,
                                },
                            ) {
                                let _ = actions.push(Action::ArmIgnition(ignition));
                            }
                        }
                        if actions.is_empty() {
                            let _ = actions.push(Action::Idle);
                        }
                    }
                }
            }
        } else {
            let _ = actions.push(Action::Idle);
        }

        if self.engine.mode == ControlMode::LimpHome {
            let _ = actions.push(Action::ApplyAux(self.limp_home_aux_commands()));
        }

        if self.calibration.staged_dirty {
            let _ = actions.push(Action::PersistCalibration);
        }

        let _ = actions.push(Action::PublishSnapshot);
        actions
    }

    fn limp_home_aux_commands(&self) -> AuxCommandBatch<RUNTIME_AUX_COMMAND_CAP> {
        let mut commands = AuxCommandBatch::new();
        let _ = commands.push(AuxCommand::new(
            AuxOutput::SafetyRelay(1),
            AuxValue::Level(OutputLevel::High),
        ));

        if let RuntimeOutputProfile::FullEcu(profile) = self.output_profile {
            for output in profile.aux_safety.off_on_limp.into_iter().flatten() {
                let _ = commands.push(AuxCommand::new(output, AuxValue::Off));
            }
        }

        commands
    }

    fn make_ignition_plan(
        &self,
        control: &ControlPlan,
        _now_us: Micros,
    ) -> ecu_scheduler::IgnitionPlan {
        ecu_scheduler::IgnitionPlan {
            output: ExclusiveChannel::new(OutputGroup::Ignition, ChannelId::new(1)),
            dwell: control.ignition.dwell_us,
            advance: control.ignition.advance_deg10,
        }
    }

    fn crank_snapshot(&self, now_us: Micros) -> CrankSnapshot {
        CrankSnapshot::new(
            now_us,
            self.engine.rpm,
            self.engine.angle_x10,
            self.engine.engine_time_authority,
        )
    }

    fn make_ignition_only_plan(
        &self,
        profile: SparkOutputProfile,
        event_index: usize,
        control: &ControlPlan,
        now_us: Micros,
    ) -> TimedIgnitionPlan {
        IgnitionScheduler::new(profile).plan_event(
            self.crank_snapshot(now_us),
            event_index,
            SparkPlan::new(control.ignition.dwell_us, control.ignition.advance_deg10),
        )
    }

    fn make_injection_only_plan(
        &self,
        profile: FuelOutputProfile,
        event_index: usize,
        control: &ControlPlan,
        now_us: Micros,
    ) -> TimedInjectionPlan {
        InjectionScheduler::new(profile).plan_event(
            self.crank_snapshot(now_us),
            event_index,
            FuelPlan::new(control.enriched_fuel),
        )
    }
}
