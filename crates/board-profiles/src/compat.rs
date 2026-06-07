use ecu_board_api::BoardCapabilities;

use crate::model::*;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FirstStartLoadSource {
    MapSpeedDensity,
    Maf,
    TpsAlphaN,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct FirstStartPreset {
    pub load_source: FirstStartLoadSource,
    pub required_sensor_roles: [Option<SensorInventoryRole>; 8],
    pub required_aux_outputs: [Option<AuxOutputRole>; 4],
    pub required_injector_outputs: u8,
    pub required_ignition_outputs: u8,
    pub requires_trigger_angle_authority: bool,
    pub requires_sync_loss_fuel_cut: bool,
    pub requires_sync_loss_ignition_cut: bool,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct BoardFirstStartSensorCapabilities {
    pub crank: bool,
    pub cam: bool,
    pub map: bool,
    pub tps: bool,
    pub clt: bool,
    pub iat: bool,
    pub maf: bool,
    pub vbatt: bool,
    pub lambda: bool,
    pub vss: bool,
    pub knock_front: bool,
    pub knock_rear: bool,
    pub baro: bool,
}

impl BoardFirstStartSensorCapabilities {
    pub const fn supports(self, role: SensorInventoryRole) -> bool {
        match role {
            SensorInventoryRole::Crank => self.crank,
            SensorInventoryRole::Cam => self.cam,
            SensorInventoryRole::Tps => self.tps,
            SensorInventoryRole::Clt => self.clt,
            SensorInventoryRole::Iat => self.iat,
            SensorInventoryRole::Maf => self.maf,
            SensorInventoryRole::Vbatt => self.vbatt,
            SensorInventoryRole::Lambda => self.lambda,
            SensorInventoryRole::KnockFront => self.knock_front,
            SensorInventoryRole::KnockRear => self.knock_rear,
            SensorInventoryRole::Vss => self.vss,
            SensorInventoryRole::Map => self.map,
            SensorInventoryRole::Baro => self.baro,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct BoardFirstStartOutputCapabilities {
    pub injector_outputs: u8,
    pub ignition_outputs: u8,
    pub aux_outputs: [Option<AuxOutputRole>; 11],
}

impl BoardFirstStartOutputCapabilities {
    pub const fn empty() -> Self {
        Self {
            injector_outputs: 0,
            ignition_outputs: 0,
            aux_outputs: [None; 11],
        }
    }

    pub fn supports_aux(self, role: AuxOutputRole) -> bool {
        self.aux_outputs.contains(&Some(role))
    }
}

impl Default for BoardFirstStartOutputCapabilities {
    fn default() -> Self {
        Self::empty()
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct BoardFirstStartSafetyCapabilities {
    pub sync_loss_cuts_fuel: bool,
    pub sync_loss_cuts_ignition: bool,
    pub trigger_angle_has_authority: bool,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct BoardFirstStartCapabilities {
    pub sensors: BoardFirstStartSensorCapabilities,
    pub outputs: BoardFirstStartOutputCapabilities,
    pub safety: BoardFirstStartSafetyCapabilities,
}

impl BoardFirstStartCapabilities {
    /// Builds the first-start compatibility surface from the canonical board
    /// capability contract.
    ///
    /// `BoardCapabilities` is the recipe-level source of truth for facts it can
    /// express: trigger/cam inputs, load-source availability, and output counts.
    /// `BoardFirstStartCapabilities` is only a first-run/evidence overlay on top
    /// of those facts. Bench or harness evidence that does not exist in
    /// `BoardCapabilities` stays explicit and defaults to absent here: CLT/IAT,
    /// VBATT, VSS, knock, baro, aux-output roles, and safety authority.
    pub const fn from_board_capabilities(capabilities: BoardCapabilities) -> Self {
        Self {
            sensors: BoardFirstStartSensorCapabilities {
                crank: capabilities.trigger_input,
                cam: capabilities.cam_input,
                map: capabilities.load_sources.map,
                tps: capabilities.load_sources.tps,
                clt: false,
                iat: false,
                maf: capabilities.load_sources.maf,
                vbatt: false,
                lambda: capabilities.load_sources.lambda,
                vss: false,
                knock_front: false,
                knock_rear: false,
                baro: false,
            },
            outputs: BoardFirstStartOutputCapabilities {
                injector_outputs: capabilities.injector_channels,
                ignition_outputs: capabilities.ignition_channels,
                aux_outputs: [None; 11],
            },
            safety: BoardFirstStartSafetyCapabilities {
                sync_loss_cuts_fuel: false,
                sync_loss_cuts_ignition: false,
                trigger_angle_has_authority: false,
            },
        }
    }

    /// Adds explicit first-start sensor evidence without rebuilding the whole
    /// capability value or changing the canonical facts seeded from
    /// `BoardCapabilities`.
    pub const fn with_sensor_evidence(mut self, role: SensorInventoryRole) -> Self {
        match role {
            SensorInventoryRole::Crank => self.sensors.crank = true,
            SensorInventoryRole::Cam => self.sensors.cam = true,
            SensorInventoryRole::Tps => self.sensors.tps = true,
            SensorInventoryRole::Clt => self.sensors.clt = true,
            SensorInventoryRole::Iat => self.sensors.iat = true,
            SensorInventoryRole::Maf => self.sensors.maf = true,
            SensorInventoryRole::Vbatt => self.sensors.vbatt = true,
            SensorInventoryRole::Lambda => self.sensors.lambda = true,
            SensorInventoryRole::KnockFront => self.sensors.knock_front = true,
            SensorInventoryRole::KnockRear => self.sensors.knock_rear = true,
            SensorInventoryRole::Vss => self.sensors.vss = true,
            SensorInventoryRole::Map => self.sensors.map = true,
            SensorInventoryRole::Baro => self.sensors.baro = true,
        }
        self
    }

    /// Adds one explicit first-start aux-output role. Out-of-range slots are
    /// ignored so callers can keep fixed-size, allocation-free construction.
    pub const fn with_aux_output_evidence(mut self, slot: usize, role: AuxOutputRole) -> Self {
        if slot < 11 {
            self.outputs.aux_outputs[slot] = Some(role);
        }
        self
    }

    /// Adds explicit first-start safety evidence. These facts are not derived
    /// from `BoardCapabilities` because they require implementation or bench
    /// evidence rather than recipe-level channel availability.
    pub const fn with_safety_evidence(mut self, safety: BoardFirstStartSafetyCapabilities) -> Self {
        self.safety = safety;
        self
    }
}

impl From<BoardCapabilities> for BoardFirstStartCapabilities {
    /// Derives the first-start overlay from the canonical recipe-level board
    /// capability contract.
    fn from(capabilities: BoardCapabilities) -> Self {
        Self::from_board_capabilities(capabilities)
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ProfileCompatibilityIssue {
    MissingSensor(SensorInventoryRole),
    MissingAuxOutput(AuxOutputRole),
    InjectorOutputCount { required: u8, available: u8 },
    IgnitionOutputCount { required: u8, available: u8 },
    MissingTriggerAngleAuthority,
    MissingSyncLossFuelCut,
    MissingSyncLossIgnitionCut,
    IssueOverflow,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ProfileCompatibilityReport {
    issues: [Option<ProfileCompatibilityIssue>; 16],
    issue_count: u8,
}

impl ProfileCompatibilityReport {
    pub const CAPACITY: usize = 16;

    pub const fn new() -> Self {
        Self {
            issues: [None; Self::CAPACITY],
            issue_count: 0,
        }
    }

    pub const fn ready(self) -> bool {
        self.issue_count == 0
    }

    pub const fn issue_count(self) -> usize {
        self.issue_count as usize
    }

    pub fn issues(&self) -> &[Option<ProfileCompatibilityIssue>] {
        &self.issues[..self.issue_count()]
    }

    fn push(&mut self, issue: ProfileCompatibilityIssue) {
        let idx = self.issue_count as usize;
        if idx < self.issues.len() {
            self.issues[idx] = Some(issue);
            self.issue_count += 1;
        } else {
            self.issues[self.issues.len() - 1] = Some(ProfileCompatibilityIssue::IssueOverflow);
        }
    }
}

impl Default for ProfileCompatibilityReport {
    fn default() -> Self {
        Self::new()
    }
}

pub fn conservative_first_start_preset(
    profile: &EngineBoardProfile,
    load_source: FirstStartLoadSource,
) -> FirstStartPreset {
    let mut required_sensor_roles = [None; 8];
    let mut sensor_len = 0;
    push_sensor(
        &mut required_sensor_roles,
        &mut sensor_len,
        SensorInventoryRole::Crank,
    );
    if profile.cam.phase_required_for_sequential {
        push_sensor(
            &mut required_sensor_roles,
            &mut sensor_len,
            SensorInventoryRole::Cam,
        );
    }
    push_sensor(
        &mut required_sensor_roles,
        &mut sensor_len,
        SensorInventoryRole::Tps,
    );
    push_sensor(
        &mut required_sensor_roles,
        &mut sensor_len,
        SensorInventoryRole::Clt,
    );
    push_sensor(
        &mut required_sensor_roles,
        &mut sensor_len,
        SensorInventoryRole::Iat,
    );
    push_sensor(
        &mut required_sensor_roles,
        &mut sensor_len,
        SensorInventoryRole::Vbatt,
    );
    match load_source {
        FirstStartLoadSource::MapSpeedDensity => {
            push_sensor(
                &mut required_sensor_roles,
                &mut sensor_len,
                SensorInventoryRole::Map,
            );
        }
        FirstStartLoadSource::Maf => {
            push_sensor(
                &mut required_sensor_roles,
                &mut sensor_len,
                SensorInventoryRole::Maf,
            );
        }
        FirstStartLoadSource::TpsAlphaN => {}
    }

    let mut required_aux_outputs = [None; 4];
    let mut aux_len = 0;
    push_aux(
        &mut required_aux_outputs,
        &mut aux_len,
        AuxOutputRole::FuelPump,
    );
    if profile.aux.outputs.contains(&AuxOutputRole::VvtIntake) {
        push_aux(
            &mut required_aux_outputs,
            &mut aux_len,
            AuxOutputRole::VvtIntake,
        );
    }

    FirstStartPreset {
        load_source,
        required_sensor_roles,
        required_aux_outputs,
        required_injector_outputs: profile.injection.channels,
        required_ignition_outputs: required_ignition_outputs(profile.ignition.topology),
        requires_trigger_angle_authority: true,
        requires_sync_loss_fuel_cut: true,
        requires_sync_loss_ignition_cut: true,
    }
}

pub fn check_profile_board_compatibility(
    profile: &EngineBoardProfile,
    preset: &FirstStartPreset,
    board: &BoardFirstStartCapabilities,
) -> ProfileCompatibilityReport {
    let mut report = ProfileCompatibilityReport::new();

    for role in preset.required_sensor_roles.iter().flatten() {
        if !board.sensors.supports(*role) {
            report.push(ProfileCompatibilityIssue::MissingSensor(*role));
        }
    }

    for role in preset.required_aux_outputs.iter().flatten() {
        if !board.outputs.supports_aux(*role) {
            report.push(ProfileCompatibilityIssue::MissingAuxOutput(*role));
        }
    }

    if board.outputs.injector_outputs < preset.required_injector_outputs {
        report.push(ProfileCompatibilityIssue::InjectorOutputCount {
            required: preset.required_injector_outputs,
            available: board.outputs.injector_outputs,
        });
    }
    if board.outputs.ignition_outputs < preset.required_ignition_outputs {
        report.push(ProfileCompatibilityIssue::IgnitionOutputCount {
            required: preset.required_ignition_outputs,
            available: board.outputs.ignition_outputs,
        });
    }
    if preset.requires_trigger_angle_authority && !board.safety.trigger_angle_has_authority {
        report.push(ProfileCompatibilityIssue::MissingTriggerAngleAuthority);
    }
    if preset.requires_sync_loss_fuel_cut
        && !(board.safety.sync_loss_cuts_fuel && profile.safety.sync_loss_cut_fuel)
    {
        report.push(ProfileCompatibilityIssue::MissingSyncLossFuelCut);
    }
    if preset.requires_sync_loss_ignition_cut
        && !(board.safety.sync_loss_cuts_ignition && profile.safety.sync_loss_cut_ignition)
    {
        report.push(ProfileCompatibilityIssue::MissingSyncLossIgnitionCut);
    }

    report
}

fn required_ignition_outputs(topology: IgnitionTopology) -> u8 {
    match topology {
        IgnitionTopology::WastedSpark { coils } | IgnitionTopology::CoilOnPlug { coils } => coils,
    }
}

fn push_sensor(
    roles: &mut [Option<SensorInventoryRole>; 8],
    len: &mut usize,
    role: SensorInventoryRole,
) {
    if *len < roles.len() {
        roles[*len] = Some(role);
        *len += 1;
    }
}

fn push_aux(roles: &mut [Option<AuxOutputRole>; 4], len: &mut usize, role: AuxOutputRole) {
    if *len < roles.len() {
        roles[*len] = Some(role);
        *len += 1;
    }
}
