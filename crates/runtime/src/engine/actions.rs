use super::*;

impl EngineRuntime {
    fn push_output_action<const N: usize>(
        &mut self,
        actions: &mut ActionBatch<N>,
        action: Action,
        now_us: Micros,
    ) {
        match action {
            Action::ArmScheduler { .. }
            | Action::ArmInjection(_)
            | Action::ArmIgnition(_)
            | Action::ApplyAux(_) => {
                self.record_output_stage(OutputStage::OutputIntent, StageOutcome::Accepted, now_us);
                if actions.push(action) {
                    self.record_output_stage(
                        OutputStage::OutputPlanner,
                        StageOutcome::Planned,
                        now_us,
                    );
                } else {
                    self.record_output_stage(
                        OutputStage::OutputPlanner,
                        StageOutcome::Dropped,
                        now_us,
                    );
                }
            }
            Action::CancelScheduler(_) => {
                let _ = actions.push(action);
            }
            _ => {
                let _ = actions.push(action);
            }
        }
    }

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
            self.push_output_action(
                &mut actions,
                Action::CancelScheduler(CancelReason::SafetyShutdown),
                now_us,
            );
        } else if self.engine.sync == SyncState::Unsynced && self.scheduler.is_armed() {
            self.scheduler.on_sync_loss();
            self.push_output_action(
                &mut actions,
                Action::CancelScheduler(CancelReason::SyncLoss),
                now_us,
            );
        } else if output_profile_requires_full_sequential_authority(self.output_profile)
            && !runtime_full_sequential_authorized(self.engine.engine_time_authority)
        {
            if self.scheduler.is_armed() {
                self.scheduler.on_sync_loss();
                self.push_output_action(
                    &mut actions,
                    Action::CancelScheduler(CancelReason::SyncLoss),
                    now_us,
                );
            } else {
                let _ = actions.push(Action::Idle);
            }
        } else if matches!(self.engine.sync, SyncState::Locked { .. }) {
            self.scheduler.on_sync_recovered();
            match self.output_profile {
                RuntimeOutputProfile::LegacySingleChannel => {
                    let allow_injection = !control.fuel_cut && control.enriched_fuel.get() > 0;
                    let allow_ignition = !control.spark_cut && control.ignition.dwell_us.get() > 0;
                    let injection = if allow_injection {
                        let injection_end = Micros::new(
                            now_us
                                .get()
                                .saturating_add(control.enriched_fuel.get())
                                .max(now_us.get().saturating_add(2)),
                        );
                        self.scheduler
                            .schedule_injection(
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
                            )
                            .ok()
                    } else {
                        self.scheduler.cancel_group(OutputGroup::Injector);
                        None
                    };
                    let ignition = if allow_ignition {
                        let ignition_end = Micros::new(
                            now_us
                                .get()
                                .saturating_add(control.ignition.dwell_us.get() as u32)
                                .max(now_us.get().saturating_add(2)),
                        );
                        self.scheduler
                            .schedule_ignition(
                                now_us,
                                Micros::new(now_us.get().saturating_add(1)),
                                ignition_end,
                                self.make_ignition_plan(control, now_us),
                            )
                            .ok()
                    } else {
                        self.scheduler.cancel_group(OutputGroup::Ignition);
                        None
                    };

                    match (injection, ignition) {
                        (Some(injection), Some(ignition)) => {
                            self.push_output_action(
                                &mut actions,
                                Action::ArmScheduler {
                                    injection,
                                    ignition,
                                },
                                now_us,
                            );
                        }
                        (Some(injection), None) => {
                            self.push_output_action(
                                &mut actions,
                                Action::ArmInjection(injection),
                                now_us,
                            );
                        }
                        (None, Some(ignition)) => {
                            self.push_output_action(
                                &mut actions,
                                Action::ArmIgnition(ignition),
                                now_us,
                            );
                        }
                        (None, None) => {
                            let _ = actions.push(Action::Idle);
                        }
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
                                self.push_output_action(
                                    &mut actions,
                                    Action::ArmIgnition(ignition),
                                    now_us,
                                );
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
                                self.push_output_action(
                                    &mut actions,
                                    Action::ArmInjection(injection),
                                    now_us,
                                );
                            }
                        }
                        if actions.is_empty() {
                            let _ = actions.push(Action::Idle);
                        }
                    }
                }
                RuntimeOutputProfile::FullEcu(profile) => {
                    let event_count = profile.event_count();
                    let allow_injection = !control.fuel_cut && control.enriched_fuel.get() > 0;
                    let allow_ignition = !control.spark_cut && control.ignition.dwell_us.get() > 0;
                    if event_count == 0 || self.engine.rpm.get() == 0 {
                        let _ = actions.push(Action::Idle);
                    } else {
                        for slot in 0..event_count {
                            let injection =
                                self.make_full_ecu_injection_plan(profile, slot, control, now_us);
                            let ignition =
                                self.make_full_ecu_ignition_plan(profile, slot, control, now_us);
                            if allow_injection {
                                if let Ok(injection) = self.scheduler.schedule_injection(
                                    now_us,
                                    injection.start_at,
                                    injection.end_at,
                                    injection.plan,
                                ) {
                                    self.push_output_action(
                                        &mut actions,
                                        Action::ArmInjection(injection),
                                        now_us,
                                    );
                                }
                            }
                            if allow_ignition {
                                if let Ok(ignition) = self.scheduler.schedule_ignition(
                                    now_us,
                                    ignition.start_at,
                                    ignition.end_at,
                                    ignition.plan,
                                ) {
                                    self.push_output_action(
                                        &mut actions,
                                        Action::ArmIgnition(ignition),
                                        now_us,
                                    );
                                }
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
            self.push_output_action(
                &mut actions,
                Action::ApplyAux(self.limp_home_aux_commands()),
                now_us,
            );
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

    fn make_full_ecu_ignition_plan(
        &self,
        profile: FullEcuOutputProfile,
        event_index: usize,
        control: &ControlPlan,
        now_us: Micros,
    ) -> TimedIgnitionPlan {
        FullEcuIgnitionScheduler::new(profile.ignition, profile.event_count()).plan_event(
            self.crank_snapshot(now_us),
            event_index,
            SparkPlan::new(control.ignition.dwell_us, control.ignition.advance_deg10),
        )
    }

    fn make_full_ecu_injection_plan(
        &self,
        profile: FullEcuOutputProfile,
        event_index: usize,
        control: &ControlPlan,
        now_us: Micros,
    ) -> TimedInjectionPlan {
        FullEcuInjectionScheduler::new(profile.injection).plan_event(
            self.crank_snapshot(now_us),
            event_index,
            FuelPlan::new(control.enriched_fuel),
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn test_injection(channel: u8, start_at: u32, end_at: u32) -> TimedInjectionPlan {
        TimedInjectionPlan {
            plan: InjectionPlan {
                output: ExclusiveChannel::new(OutputGroup::Injector, ChannelId::new(channel)),
                pulse_width: ecu_domain::PulseWidthUs::new(end_at.saturating_sub(start_at)),
            },
            start_at: Micros::new(start_at),
            end_at: Micros::new(end_at),
        }
    }

    fn test_ignition(channel: u8, start_at: u32, end_at: u32) -> TimedIgnitionPlan {
        TimedIgnitionPlan {
            plan: ecu_scheduler::IgnitionPlan {
                output: ExclusiveChannel::new(OutputGroup::Ignition, ChannelId::new(channel)),
                dwell: DwellUs::new(end_at.saturating_sub(start_at) as u16),
                advance: ecu_domain::Degrees10::new(120),
            },
            start_at: Micros::new(start_at),
            end_at: Micros::new(end_at),
        }
    }

    #[test]
    fn push_output_action_records_intent_and_planner_only() {
        let mut runtime = EngineRuntime::new();
        let mut actions = ActionBatch::<4>::new();

        runtime.push_output_action(
            &mut actions,
            Action::ArmInjection(test_injection(2, 100, 140)),
            Micros::new(55),
        );

        let counters = runtime.output_assembly_counters();
        assert_eq!(counters.output_intent.seen, 1);
        assert_eq!(counters.output_planner.seen, 1);
        assert_eq!(counters.output_planner.accepted, 1);
        assert_eq!(counters.output_planner.dropped, 0);
        assert_eq!(counters.output_admission.seen, 0);
        assert!(actions
            .iter()
            .any(|action| matches!(action, Action::ArmInjection(_))));
    }

    #[test]
    fn push_output_action_does_not_record_cancel_admission() {
        let mut runtime = EngineRuntime::new();
        let mut actions = ActionBatch::<2>::new();

        runtime.push_output_action(
            &mut actions,
            Action::CancelScheduler(CancelReason::SyncLoss),
            Micros::new(77),
        );

        let counters = runtime.output_assembly_counters();
        assert_eq!(counters.output_intent.seen, 0);
        assert_eq!(counters.output_planner.seen, 0);
        assert_eq!(counters.output_admission.seen, 0);
    }

    #[test]
    fn push_output_action_marks_planner_dropped_when_batch_is_full() {
        let mut runtime = EngineRuntime::new();
        let mut actions = ActionBatch::<0>::new();

        runtime.push_output_action(
            &mut actions,
            Action::ArmIgnition(test_ignition(1, 200, 260)),
            Micros::new(91),
        );

        let counters = runtime.output_assembly_counters();
        assert_eq!(counters.output_intent.seen, 1);
        assert_eq!(counters.output_planner.seen, 1);
        assert_eq!(counters.output_planner.accepted, 0);
        assert_eq!(counters.output_planner.dropped, 1);
        assert_eq!(counters.output_admission.seen, 0);
        assert!(actions.is_empty());
    }
}
