use ecu_board_api::{AuxCommand, AuxCommandBatch, AuxOutput, AuxValue, OutputLevel};
use ecu_domain::{Degrees10, Lambda100, Micros, Rpm};
use ecu_runtime::{
    Action, ControlInputs, EnrichmentInputs, IgnitionInputs, LambdaTrimInputs,
    RuntimeScheduledLevel, RuntimeScheduledOutputKind, RuntimeScheduledTransition, StepResult,
    TorqueInputs,
};

use crate::{EcuSimOutputEvent, EcuSimOutputKind, EcuSimStatus};

use super::model::EcuSimHandle;

impl EcuSimHandle {
    pub(super) fn drain_sim_steps(&mut self) -> EcuSimStatus {
        let mut status = EcuSimStatus::Ok;
        while let Some(step) = self.sim.drain_one() {
            if let Some(result) = step.result {
                if self.capture_step_result(result) == EcuSimStatus::ErrEventOverflow {
                    status = EcuSimStatus::ErrEventOverflow;
                }
            }
        }
        status
    }

    fn capture_step_result(&mut self, result: StepResult) -> EcuSimStatus {
        let mut status = EcuSimStatus::Ok;
        for action in result.actions.iter() {
            if self.capture_action(action) == EcuSimStatus::ErrEventOverflow {
                status = EcuSimStatus::ErrEventOverflow;
            }
        }
        status
    }

    fn capture_action(&mut self, action: Action) -> EcuSimStatus {
        match action {
            Action::ArmScheduler { .. } | Action::ArmInjection(_) | Action::ArmIgnition(_) => {
                self.capture_scheduled_action(action)
            }
            Action::ApplyAux(batch) => self.capture_aux_batch(&batch),
            Action::CancelScheduler(_)
            | Action::PublishSnapshot
            | Action::PersistCalibration
            | Action::Idle => EcuSimStatus::Ok,
        }
    }

    fn capture_scheduled_action(&mut self, action: Action) -> EcuSimStatus {
        match action.export_scheduled_transitions::<4>() {
            Ok(batch) => {
                let mut status = EcuSimStatus::Ok;
                for transition in batch.iter() {
                    if self.capture_transition(transition) == EcuSimStatus::ErrEventOverflow {
                        status = EcuSimStatus::ErrEventOverflow;
                    }
                }
                status
            }
            Err(_) => EcuSimStatus::Ok,
        }
    }

    fn capture_aux_batch<const N: usize>(&mut self, batch: &AuxCommandBatch<N>) -> EcuSimStatus {
        let mut status = EcuSimStatus::Ok;
        for command in batch.iter() {
            if self.capture_aux_command(*command) == EcuSimStatus::ErrEventOverflow {
                status = EcuSimStatus::ErrEventOverflow;
            }
        }
        status
    }

    fn capture_aux_command(&mut self, command: AuxCommand) -> EcuSimStatus {
        match command.output {
            AuxOutput::SafetyRelay(1) => self.push_output(EcuSimOutputEvent {
                time_us: self.now_us,
                channel: 0,
                kind: EcuSimOutputKind::Fan as i32,
                high: u8::from(matches!(command.value, AuxValue::Level(OutputLevel::High))),
            }),
            _ => EcuSimStatus::Ok,
        }
    }

    fn capture_transition(&mut self, transition: RuntimeScheduledTransition) -> EcuSimStatus {
        self.push_output(EcuSimOutputEvent {
            time_us: transition.at_us.get(),
            channel: self
                .cfg
                .map_channel(transition.kind, transition.channel.get()),
            kind: match transition.kind {
                RuntimeScheduledOutputKind::Injector => EcuSimOutputKind::Injector as i32,
                RuntimeScheduledOutputKind::Ignition => EcuSimOutputKind::Ignition as i32,
            },
            high: match transition.level {
                RuntimeScheduledLevel::Low => 0,
                RuntimeScheduledLevel::High => 1,
            },
        })
    }

    fn push_output(&mut self, event: EcuSimOutputEvent) -> EcuSimStatus {
        match self.outputs.push_sorted(event) {
            Ok(()) => EcuSimStatus::Ok,
            Err(()) => {
                self.overflow_latched = true;
                EcuSimStatus::ErrEventOverflow
            }
        }
    }

    pub(crate) fn dequeue_event(&mut self) -> Option<EcuSimOutputEvent> {
        self.outputs.pop_front()
    }

    pub(crate) fn outputs_empty(&self) -> bool {
        self.outputs.is_empty()
    }

    pub(crate) fn clear_overflow_latch(&mut self) {
        self.overflow_latched = false;
    }

    pub(super) fn control_inputs(&self, now_us: u32) -> ControlInputs {
        let clt_c = self.sensors.clt_c10 / 10;
        let measured_lambda = if self.sensors.lambda_x100 == 0 {
            100
        } else {
            self.sensors.lambda_x100
        };
        let throttle_pct = (u32::from(self.sensors.tps_x100) / 100).min(100) as u16;
        let driver_request = throttle_pct.max(20);

        ControlInputs {
            enrichment: EnrichmentInputs {
                now_us: Micros::new(now_us),
                clt_c,
                cranking: self.rpm > 0 && self.rpm < 400,
                just_started: false,
                tpsdot_pct_s: 0,
                mapdot_kpa_s: 0,
            },
            lambda: LambdaTrimInputs {
                clt_c,
                lambda_valid: self.sensors.lambda_valid != 0,
                measured_lambda100: Lambda100::new(measured_lambda),
                requested_open_loop: self.sensors.lambda_valid == 0,
            },
            torque: TorqueInputs::new(driver_request, 30, 100, 100, 100),
            ignition: IgnitionInputs::new(Degrees10::new(100), 0, 0, 0, false, Rpm::new(self.rpm)),
            knock_intensity_x100: 0,
        }
    }
}
