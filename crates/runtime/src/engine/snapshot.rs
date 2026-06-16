use super::*;

impl EngineRuntime {
    pub(super) fn apply_control_cut_state(&mut self, control: &ControlPlan) {
        self.fuel_cut = control.fuel_cut;
        self.spark_cut = control.spark_cut;
    }

    pub fn apply_decoder_observation(&mut self, observation: DecoderObservation) {
        self.apply_decoder_observation_inner(observation, true);
    }

    fn apply_decoder_observation_inner(&mut self, observation: DecoderObservation, refresh: bool) {
        match observation {
            DecoderObservation::Trigger(trigger) => {
                self.engine.rpm = trigger.rpm;
                self.engine.angle_x10 = trigger.angle_x10;
                let cam_seen = matches!(
                    self.engine.engine_time_authority.phase,
                    PhaseSyncState::CamObserved720 | PhaseSyncState::CamValidated720
                );
                let authority = derive_engine_time_authority(
                    self.engine.engine_time_authority,
                    trigger.synced,
                    cam_seen,
                    trigger.rpm,
                );
                self.set_engine_time_authority_inner(authority);
            }
            DecoderObservation::Cam(cam) => {
                let authority = derive_engine_time_authority(
                    self.engine.engine_time_authority,
                    self.engine.engine_time_authority.has_primary_lock(),
                    cam.cam_seen,
                    self.engine.rpm,
                );
                self.set_engine_time_authority_inner(authority);
            }
        }

        if refresh {
            self.refresh_snapshot();
        }
    }

    pub(super) fn refresh_snapshot(&mut self) {
        self.runtime_snapshot = RuntimeSnapshot {
            engine: self.engine,
            control: self.control,
            faults: self.faults,
            rev_soft_active: self.rev_soft_active,
            rev_hard_active: self.rev_hard_active,
            launch_active: self.launch_active,
            flat_shift_active: self.flat_shift_active,
            safety_latched: self.safety_latched,
            fuel_cut: self.fuel_cut,
            spark_cut: self.spark_cut,
            legacy_cut_reason_code: self.legacy_cut_reason_code(),
            knock_intensity_x100: self.knock_intensity_x100,
            knock_retard_deg10: self.knock_retard_deg10,
        };
        self.calibration_snapshot = self.calibration.active;
    }
}
