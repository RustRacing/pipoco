use ecu_calibration::{
    ActiveCalibration, Calibration, CalibrationHardwareTargetId, CalibrationRevision,
    CalibrationRuntimeBuildId, CalibrationSnapshot, ExpertTriggerCalibration, KvStore,
    PersistedCalibrationPackage, StagedCalibration,
};
use ecu_ts::persistence::{CalibrationPackageSession, PageStoreProvider, PersistedTsPageStore};
use ecu_ts::server::PageError;
use ecu_ts::{CalibrationEditSurface, CalibrationPackageApplyResult};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CompatCalibrationSession {
    surface: CalibrationEditSurface,
}

impl CompatCalibrationSession {
    pub fn new(expert_trigger: ExpertTriggerCalibration) -> Self {
        let active_revision = CalibrationRevision::new(0);
        let active = ActiveCalibration::new(active_revision, Calibration::default());
        let mut staged_calibration = Calibration::default();
        staged_calibration.set_expert_trigger(expert_trigger);
        let staged = StagedCalibration::new(active_revision, staged_calibration);
        Self {
            surface: CalibrationEditSurface::new(CalibrationSnapshot { active, staged }),
        }
    }

    pub fn from_persisted_store<P: PageStoreProvider, KV: KvStore>(
        store: &PersistedTsPageStore<P, KV>,
    ) -> Self {
        Self::new(store.expert_trigger_calibration())
    }

    pub fn surface(&self) -> &CalibrationEditSurface {
        &self.surface
    }

    pub fn surface_mut(&mut self) -> &mut CalibrationEditSurface {
        &mut self.surface
    }
}

impl Default for CompatCalibrationSession {
    fn default() -> Self {
        Self::new(ExpertTriggerCalibration::default())
    }
}

impl CalibrationPackageSession for CompatCalibrationSession {
    fn export_current_package(
        &self,
        runtime_build_id: CalibrationRuntimeBuildId,
        hardware_target_id: CalibrationHardwareTargetId,
    ) -> PersistedCalibrationPackage {
        self.surface
            .export_current_package(runtime_build_id, hardware_target_id)
    }

    fn import_candidate_package(
        &mut self,
        candidate: PersistedCalibrationPackage,
        runtime_build_id: CalibrationRuntimeBuildId,
        hardware_target_id: CalibrationHardwareTargetId,
    ) -> CalibrationPackageApplyResult {
        self.surface
            .import_candidate_package(candidate, runtime_build_id, hardware_target_id)
    }

    fn apply_expert_trigger_page(&mut self, data: &[u8]) -> Result<(), PageError> {
        self.surface
            .apply_expert_trigger_page(data)
            .map(|_| ())
            .map_err(|_| PageError::Invalid)
    }

    fn expert_trigger_calibration(&self) -> ExpertTriggerCalibration {
        self.surface.expert_trigger_calibration()
    }
}
