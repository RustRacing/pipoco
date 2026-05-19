//! Deterministic closed-loop plant for host ECU-in-loop tests.
//!
//! This is intentionally a causal plant, not a thermodynamic model. It consumes
//! ECU output transitions and feeds back RPM, MAP, crank angle, and basic sensor
//! values so tests can prove that output suppression and event ordering affect
//! future inputs.

use ecu_domain::{ChannelId, Degrees10, Kpa10, Lambda100, Micros, Rpm};
use ecu_io::{OutputLevel, OutputTransition, OutputTransitionKind, SensorFrame};

use crate::output_capture::FixedTransitionBuffer;

pub const PLANT_MAX_CHANNELS: usize = 16;
pub const PLANT_NO_CYLINDER: u8 = u8::MAX;

/// Metadata needed by the causal plant.
pub trait PlantProfile: Copy {
    fn cylinders(self) -> u8;
    fn firing_order(self) -> [u8; PLANT_MAX_CHANNELS];
    fn firing_len(self) -> u8;
    fn displacement_cc(self) -> u16;
    fn idle_rpm(self) -> u16;
    fn redline_rpm(self) -> u16;
    fn injector_flow_cc_per_min(self) -> u16;
    fn injector_cylinder(self, channel: ChannelId) -> Option<u8>;
    fn ignition_cylinder(self, channel: ChannelId) -> Option<u8>;
}

/// Fixed metadata profile suitable for deterministic host scenarios.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct FixedPlantProfile {
    pub cylinders: u8,
    pub firing_order: [u8; PLANT_MAX_CHANNELS],
    pub firing_len: u8,
    pub displacement_cc: u16,
    pub idle_rpm: u16,
    pub redline_rpm: u16,
    pub injector_flow_cc_per_min: u16,
    pub injector_channel_to_cylinder: [u8; PLANT_MAX_CHANNELS],
    pub ignition_channel_to_cylinder: [u8; PLANT_MAX_CHANNELS],
}

impl FixedPlantProfile {
    /// Four-cylinder test profile with direct injector and ignition channels.
    pub const fn inline_four() -> Self {
        Self {
            cylinders: 4,
            firing_order: [0, 2, 3, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            firing_len: 4,
            displacement_cc: 2000,
            idle_rpm: 850,
            redline_rpm: 6500,
            injector_flow_cc_per_min: 240,
            injector_channel_to_cylinder: [
                0,
                1,
                2,
                3,
                PLANT_NO_CYLINDER,
                PLANT_NO_CYLINDER,
                PLANT_NO_CYLINDER,
                PLANT_NO_CYLINDER,
                PLANT_NO_CYLINDER,
                PLANT_NO_CYLINDER,
                PLANT_NO_CYLINDER,
                PLANT_NO_CYLINDER,
                PLANT_NO_CYLINDER,
                PLANT_NO_CYLINDER,
                PLANT_NO_CYLINDER,
                PLANT_NO_CYLINDER,
            ],
            ignition_channel_to_cylinder: [
                0,
                1,
                2,
                3,
                PLANT_NO_CYLINDER,
                PLANT_NO_CYLINDER,
                PLANT_NO_CYLINDER,
                PLANT_NO_CYLINDER,
                PLANT_NO_CYLINDER,
                PLANT_NO_CYLINDER,
                PLANT_NO_CYLINDER,
                PLANT_NO_CYLINDER,
                PLANT_NO_CYLINDER,
                PLANT_NO_CYLINDER,
                PLANT_NO_CYLINDER,
                PLANT_NO_CYLINDER,
            ],
        }
    }

    /// Two-cylinder profile matching the Batch 1-2 harness channel layout:
    /// injectors on channels 0/1 and ignition on channels 2/3.
    pub const fn harness_two_cylinder() -> Self {
        Self {
            cylinders: 2,
            firing_order: [0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            firing_len: 2,
            displacement_cc: 1000,
            idle_rpm: 900,
            redline_rpm: 6500,
            injector_flow_cc_per_min: 200,
            injector_channel_to_cylinder: [
                0,
                1,
                PLANT_NO_CYLINDER,
                PLANT_NO_CYLINDER,
                PLANT_NO_CYLINDER,
                PLANT_NO_CYLINDER,
                PLANT_NO_CYLINDER,
                PLANT_NO_CYLINDER,
                PLANT_NO_CYLINDER,
                PLANT_NO_CYLINDER,
                PLANT_NO_CYLINDER,
                PLANT_NO_CYLINDER,
                PLANT_NO_CYLINDER,
                PLANT_NO_CYLINDER,
                PLANT_NO_CYLINDER,
                PLANT_NO_CYLINDER,
            ],
            ignition_channel_to_cylinder: [
                PLANT_NO_CYLINDER,
                PLANT_NO_CYLINDER,
                0,
                1,
                PLANT_NO_CYLINDER,
                PLANT_NO_CYLINDER,
                PLANT_NO_CYLINDER,
                PLANT_NO_CYLINDER,
                PLANT_NO_CYLINDER,
                PLANT_NO_CYLINDER,
                PLANT_NO_CYLINDER,
                PLANT_NO_CYLINDER,
                PLANT_NO_CYLINDER,
                PLANT_NO_CYLINDER,
                PLANT_NO_CYLINDER,
                PLANT_NO_CYLINDER,
            ],
        }
    }

    fn mapped_cylinder(map: [u8; PLANT_MAX_CHANNELS], channel: ChannelId) -> Option<u8> {
        let idx = channel.get() as usize;
        if idx >= PLANT_MAX_CHANNELS {
            return None;
        }
        match map[idx] {
            PLANT_NO_CYLINDER => None,
            cylinder => Some(cylinder),
        }
    }
}

impl PlantProfile for FixedPlantProfile {
    fn cylinders(self) -> u8 {
        self.cylinders
    }

    fn firing_order(self) -> [u8; PLANT_MAX_CHANNELS] {
        self.firing_order
    }

    fn firing_len(self) -> u8 {
        self.firing_len
    }

    fn displacement_cc(self) -> u16 {
        self.displacement_cc
    }

    fn idle_rpm(self) -> u16 {
        self.idle_rpm
    }

    fn redline_rpm(self) -> u16 {
        self.redline_rpm
    }

    fn injector_flow_cc_per_min(self) -> u16 {
        self.injector_flow_cc_per_min
    }

    fn injector_cylinder(self, channel: ChannelId) -> Option<u8> {
        Self::mapped_cylinder(self.injector_channel_to_cylinder, channel)
    }

    fn ignition_cylinder(self, channel: ChannelId) -> Option<u8> {
        Self::mapped_cylinder(self.ignition_channel_to_cylinder, channel)
    }
}

/// Injector fuel conversion model.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct InjectorModel {
    pub flow_cc_per_min: u16,
    pub fuel_density_mg_per_ml: u16,
    pub fuel_molecular_mass_mg_per_mmol: u16,
}

impl InjectorModel {
    pub const fn gasoline(flow_cc_per_min: u16) -> Self {
        Self {
            flow_cc_per_min,
            fuel_density_mg_per_ml: 745,
            fuel_molecular_mass_mg_per_mmol: 114,
        }
    }

    /// Converts injector open time to fuel mass in micrograms.
    pub fn fuel_mass_ug(self, pulse_width_us: u32) -> u32 {
        let mass = pulse_width_us as u64
            * self.flow_cc_per_min as u64
            * self.fuel_density_mg_per_ml as u64
            * 1_000;
        div_u64_clamp_u32(mass, 60_000_000)
    }
}

/// External plant controls for one integration step.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct PlantControls {
    pub throttle_x100: u16,
    pub starter_on: bool,
    pub load_torque_x100: i32,
    pub vbatt_mv: u16,
    pub fault: PlantFault,
}

impl PlantControls {
    pub const fn idle() -> Self {
        Self {
            throttle_x100: 8,
            starter_on: false,
            load_torque_x100: 400,
            vbatt_mv: 12_500,
            fault: PlantFault::None,
        }
    }
}

/// Deterministic plant fault injection.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PlantFault {
    None,
    SuppressCombustion,
    MapStuck(Kpa10),
    RpmDropout,
}

/// Tunable limits for the causal plant.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct PlantLimits {
    pub min_dwell_us: u32,
    pub max_pulse_width_us: u32,
    pub max_fuel_spark_gap_us: u32,
    pub inertia_x1000: u32,
    pub drag_x1000: u32,
    pub starter_torque_x100: i32,
    pub max_combustion_torque_x100: u32,
}

impl PlantLimits {
    pub const fn conservative() -> Self {
        Self {
            min_dwell_us: 800,
            max_pulse_width_us: 25_000,
            max_fuel_spark_gap_us: 80_000,
            inertia_x1000: 700,
            drag_x1000: 160,
            starter_torque_x100: 2_500,
            max_combustion_torque_x100: 9_000,
        }
    }
}

/// Error returned while consuming output transitions.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PlantError {
    UnknownInjectorChannel(ChannelId),
    UnknownIgnitionChannel(ChannelId),
    UnknownCylinder(u8),
    UnmatchedInjectorClose(ChannelId),
    UnmatchedIgnitionFire(ChannelId),
    PulseTooLong {
        channel: ChannelId,
        pulse_width_us: u32,
    },
}

/// Combustion event produced by a valid fuel and spark pair.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CombustionEvent {
    pub at_us: Micros,
    pub cylinder: u8,
    pub fuel_ug: u32,
    pub dwell_us: u32,
    pub torque_x100: u32,
}

/// Aggregate result from consuming a capture buffer.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct PlantConsumeReport {
    pub transitions: u16,
    pub combustion_events: u16,
}

/// Public plant snapshot after a step.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct PlantSnapshot {
    pub now_us: Micros,
    pub rpm: Rpm,
    pub map_kpa10: Kpa10,
    pub crank_angle_deg10: Degrees10,
    pub throttle_x100: u16,
    pub vbatt_mv: u16,
    pub last_torque_x100: i32,
    pub combustion_events: u32,
    pub injector_events: u32,
    pub spark_events: u32,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct ChannelPulseState {
    high_since: Option<Micros>,
    last_width_us: u32,
}

impl ChannelPulseState {
    const fn new() -> Self {
        Self {
            high_since: None,
            last_width_us: 0,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct CylinderState {
    pending_fuel_ug: u32,
    last_fuel_at_us: Option<Micros>,
}

impl CylinderState {
    const fn new() -> Self {
        Self {
            pending_fuel_ug: 0,
            last_fuel_at_us: None,
        }
    }
}

/// Closed-loop plant state.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ClosedLoopPlant<P: PlantProfile> {
    profile: P,
    injector: InjectorModel,
    limits: PlantLimits,
    injector_channels: [ChannelPulseState; PLANT_MAX_CHANNELS],
    ignition_channels: [ChannelPulseState; PLANT_MAX_CHANNELS],
    cylinders: [CylinderState; PLANT_MAX_CHANNELS],
    now_us: Micros,
    rpm: u32,
    map_kpa10: u16,
    crank_angle_deg10: i32,
    pending_torque_x100: i32,
    last_torque_x100: i32,
    combustion_events: u32,
    injector_events: u32,
    spark_events: u32,
}

impl<P: PlantProfile> ClosedLoopPlant<P> {
    pub fn new(profile: P, injector: InjectorModel) -> Self {
        Self {
            profile,
            injector,
            limits: PlantLimits::conservative(),
            injector_channels: [ChannelPulseState::new(); PLANT_MAX_CHANNELS],
            ignition_channels: [ChannelPulseState::new(); PLANT_MAX_CHANNELS],
            cylinders: [CylinderState::new(); PLANT_MAX_CHANNELS],
            now_us: Micros::new(0),
            rpm: 0,
            map_kpa10: 1_000,
            crank_angle_deg10: 0,
            pending_torque_x100: 0,
            last_torque_x100: 0,
            combustion_events: 0,
            injector_events: 0,
            spark_events: 0,
        }
    }

    pub fn with_limits(mut self, limits: PlantLimits) -> Self {
        self.limits = limits;
        self
    }

    pub fn set_initial_rpm(&mut self, rpm: Rpm) {
        self.rpm = rpm.get() as u32;
    }

    pub fn snapshot(&self, controls: PlantControls) -> PlantSnapshot {
        PlantSnapshot {
            now_us: self.now_us,
            rpm: self.sensor_rpm(controls),
            map_kpa10: self.sensor_map(controls),
            crank_angle_deg10: Degrees10::new(self.crank_angle_deg10 as i16),
            throttle_x100: controls.throttle_x100,
            vbatt_mv: controls.vbatt_mv,
            last_torque_x100: self.last_torque_x100,
            combustion_events: self.combustion_events,
            injector_events: self.injector_events,
            spark_events: self.spark_events,
        }
    }

    pub fn sensor_frame(&self, at_us: Micros, controls: PlantControls) -> SensorFrame {
        SensorFrame {
            at_us,
            rpm: self.sensor_rpm(controls),
            map_kpa10: self.sensor_map(controls),
            angle_x10: Degrees10::new(self.crank_angle_deg10 as i16),
            tps_x100: controls.throttle_x100,
            clt_c10: 800,
            iat_c10: 300,
            vbatt_mv: controls.vbatt_mv,
            baro_kpa10: Kpa10::new(1_000),
            lambda_valid: false,
            lambda_x100: Lambda100::new(100),
        }
    }

    pub fn consume_capture<const N: usize>(
        &mut self,
        capture: &FixedTransitionBuffer<N>,
    ) -> Result<PlantConsumeReport, PlantError> {
        let mut report = PlantConsumeReport::default();
        let mut idx = 0usize;
        while idx < capture.len() {
            if let Some(transition) = capture.get(idx) {
                report.transitions = report.transitions.saturating_add(1);
                if self.apply_transition(transition)?.is_some() {
                    report.combustion_events = report.combustion_events.saturating_add(1);
                }
            }
            idx += 1;
        }
        Ok(report)
    }

    pub fn apply_transition(
        &mut self,
        transition: OutputTransition,
    ) -> Result<Option<CombustionEvent>, PlantError> {
        match transition.kind {
            OutputTransitionKind::Injector => self.apply_injector_transition(transition),
            OutputTransitionKind::Ignition => self.apply_ignition_transition(transition),
            OutputTransitionKind::Idle | OutputTransitionKind::Fan => Ok(None),
        }
    }

    pub fn advance_to(&mut self, now_us: Micros, controls: PlantControls) -> PlantSnapshot {
        let dt_us = elapsed_us(self.now_us, now_us);
        let torque = if matches!(controls.fault, PlantFault::SuppressCombustion) {
            0
        } else {
            self.pending_torque_x100
        };
        let starter = if controls.starter_on {
            self.limits.starter_torque_x100
        } else {
            0
        };
        let drag = ((self.rpm as u64 * self.limits.drag_x1000 as u64) / 1_000) as i32;
        let net_torque = torque + starter - controls.load_torque_x100 - drag;
        let rpm_delta =
            (net_torque as i64 * dt_us as i64) / (self.limits.inertia_x1000 as i64 * 1_000);
        let next_rpm = self.rpm as i64 + rpm_delta;
        let redline_limit = self.profile.redline_rpm() as u32 + 500;
        self.rpm = clamp_i64_to_u32(next_rpm, 0, redline_limit);
        self.crank_angle_deg10 = advance_angle_deg10(self.crank_angle_deg10, self.rpm, dt_us);
        self.map_kpa10 =
            derive_map_kpa10(self.rpm, controls.throttle_x100, controls.load_torque_x100);
        self.now_us = now_us;
        self.last_torque_x100 = torque;
        self.pending_torque_x100 = 0;
        self.snapshot(controls)
    }

    fn apply_injector_transition(
        &mut self,
        transition: OutputTransition,
    ) -> Result<Option<CombustionEvent>, PlantError> {
        let cylinder = self
            .profile
            .injector_cylinder(transition.channel)
            .ok_or(PlantError::UnknownInjectorChannel(transition.channel))?;
        self.validate_cylinder(cylinder)?;
        let idx = transition.channel.get() as usize;
        match transition.level {
            OutputLevel::High => {
                self.injector_channels[idx].high_since = Some(transition.at_us);
                Ok(None)
            }
            OutputLevel::Low => {
                let start = self.injector_channels[idx]
                    .high_since
                    .take()
                    .ok_or(PlantError::UnmatchedInjectorClose(transition.channel))?;
                let pulse_width_us = elapsed_us(start, transition.at_us);
                if pulse_width_us > self.limits.max_pulse_width_us {
                    return Err(PlantError::PulseTooLong {
                        channel: transition.channel,
                        pulse_width_us,
                    });
                }
                self.injector_channels[idx].last_width_us = pulse_width_us;
                let fuel_ug = self.injector.fuel_mass_ug(pulse_width_us);
                let cylinder_idx = cylinder as usize;
                self.cylinders[cylinder_idx].pending_fuel_ug = self.cylinders[cylinder_idx]
                    .pending_fuel_ug
                    .saturating_add(fuel_ug);
                self.cylinders[cylinder_idx].last_fuel_at_us = Some(transition.at_us);
                self.injector_events = self.injector_events.wrapping_add(1);
                Ok(None)
            }
        }
    }

    fn apply_ignition_transition(
        &mut self,
        transition: OutputTransition,
    ) -> Result<Option<CombustionEvent>, PlantError> {
        let cylinder = self
            .profile
            .ignition_cylinder(transition.channel)
            .ok_or(PlantError::UnknownIgnitionChannel(transition.channel))?;
        self.validate_cylinder(cylinder)?;
        let idx = transition.channel.get() as usize;
        match transition.level {
            OutputLevel::High => {
                self.ignition_channels[idx].high_since = Some(transition.at_us);
                Ok(None)
            }
            OutputLevel::Low => {
                let start = self.ignition_channels[idx]
                    .high_since
                    .take()
                    .ok_or(PlantError::UnmatchedIgnitionFire(transition.channel))?;
                let dwell_us = elapsed_us(start, transition.at_us);
                self.ignition_channels[idx].last_width_us = dwell_us;
                self.spark_events = self.spark_events.wrapping_add(1);
                if dwell_us < self.limits.min_dwell_us {
                    return Ok(None);
                }
                self.try_combustion(transition.at_us, cylinder, dwell_us)
            }
        }
    }

    fn try_combustion(
        &mut self,
        at_us: Micros,
        cylinder: u8,
        dwell_us: u32,
    ) -> Result<Option<CombustionEvent>, PlantError> {
        self.validate_cylinder(cylinder)?;
        let cylinder_idx = cylinder as usize;
        let state = &mut self.cylinders[cylinder_idx];
        let Some(fuel_at) = state.last_fuel_at_us else {
            return Ok(None);
        };
        let gap_us = elapsed_us(fuel_at, at_us);
        if gap_us > self.limits.max_fuel_spark_gap_us {
            state.pending_fuel_ug = 0;
            state.last_fuel_at_us = None;
            return Ok(None);
        };
        if state.pending_fuel_ug == 0 {
            return Ok(None);
        };
        let fuel_ug = state.pending_fuel_ug;
        state.pending_fuel_ug = 0;
        state.last_fuel_at_us = None;
        let torque_x100 = fuel_to_torque_x100(fuel_ug, self.limits.max_combustion_torque_x100);
        self.pending_torque_x100 = self.pending_torque_x100.saturating_add(torque_x100 as i32);
        self.combustion_events = self.combustion_events.wrapping_add(1);
        Ok(Some(CombustionEvent {
            at_us,
            cylinder,
            fuel_ug,
            dwell_us,
            torque_x100,
        }))
    }

    fn sensor_rpm(&self, controls: PlantControls) -> Rpm {
        if matches!(controls.fault, PlantFault::RpmDropout) {
            Rpm::new(0)
        } else {
            Rpm::new(clamp_u32_to_u16(self.rpm))
        }
    }

    fn sensor_map(&self, controls: PlantControls) -> Kpa10 {
        match controls.fault {
            PlantFault::MapStuck(map) => map,
            _ => Kpa10::new(self.map_kpa10),
        }
    }

    fn validate_cylinder(&self, cylinder: u8) -> Result<(), PlantError> {
        if cylinder as usize >= PLANT_MAX_CHANNELS || cylinder >= self.profile.cylinders() {
            Err(PlantError::UnknownCylinder(cylinder))
        } else {
            Ok(())
        }
    }
}

fn elapsed_us(start: Micros, end: Micros) -> u32 {
    end.get().wrapping_sub(start.get())
}

fn div_u64_clamp_u32(numerator: u64, denominator: u64) -> u32 {
    let value = numerator / denominator;
    if value > u32::MAX as u64 {
        u32::MAX
    } else {
        value as u32
    }
}

fn fuel_to_torque_x100(fuel_ug: u32, max_torque_x100: u32) -> u32 {
    let torque = fuel_ug.saturating_mul(18);
    if torque > max_torque_x100 {
        max_torque_x100
    } else {
        torque
    }
}

fn clamp_i64_to_u32(value: i64, min: u32, max: u32) -> u32 {
    if value < min as i64 {
        min
    } else if value > max as i64 {
        max
    } else {
        value as u32
    }
}

fn clamp_u32_to_u16(value: u32) -> u16 {
    if value > u16::MAX as u32 {
        u16::MAX
    } else {
        value as u16
    }
}

fn advance_angle_deg10(current: i32, rpm: u32, dt_us: u32) -> i32 {
    let delta = ((rpm as u64 * dt_us as u64 * 3) / 50_000) as i32;
    (current + delta).rem_euclid(7200)
}

fn derive_map_kpa10(rpm: u32, throttle_x100: u16, load_torque_x100: i32) -> u16 {
    let throttle_term = throttle_x100 as i32 * 7;
    let rpm_vacuum = (rpm as i32 / 12).min(450);
    let load_term = (load_torque_x100 / 20).clamp(0, 300);
    let raw = 260 + throttle_term + load_term - rpm_vacuum;
    raw.clamp(180, 1_050) as u16
}

#[cfg(test)]
mod tests {
    use super::*;

    fn transition(
        at_us: u32,
        kind: OutputTransitionKind,
        channel: u8,
        level: OutputLevel,
    ) -> OutputTransition {
        OutputTransition {
            at_us: Micros::new(at_us),
            kind,
            channel: ChannelId::new(channel),
            level,
        }
    }

    fn plant() -> ClosedLoopPlant<FixedPlantProfile> {
        let profile = FixedPlantProfile::inline_four();
        ClosedLoopPlant::new(
            profile,
            InjectorModel::gasoline(profile.injector_flow_cc_per_min()),
        )
    }

    #[derive(Clone, Copy)]
    struct BadProfile;

    impl PlantProfile for BadProfile {
        fn cylinders(self) -> u8 {
            1
        }

        fn firing_order(self) -> [u8; PLANT_MAX_CHANNELS] {
            [0; PLANT_MAX_CHANNELS]
        }

        fn firing_len(self) -> u8 {
            1
        }

        fn displacement_cc(self) -> u16 {
            500
        }

        fn idle_rpm(self) -> u16 {
            900
        }

        fn redline_rpm(self) -> u16 {
            6_000
        }

        fn injector_flow_cc_per_min(self) -> u16 {
            200
        }

        fn injector_cylinder(self, _channel: ChannelId) -> Option<u8> {
            Some(9)
        }

        fn ignition_cylinder(self, _channel: ChannelId) -> Option<u8> {
            Some(9)
        }
    }

    #[test]
    fn injector_model_converts_pulse_width_to_fuel_mass() {
        let injector = InjectorModel::gasoline(240);

        assert_eq!(injector.fuel_mass_ug(0), 0);
        assert!(injector.fuel_mass_ug(3_000) > injector.fuel_mass_ug(1_000));
    }

    #[test]
    fn fuel_and_spark_create_combustion_event() {
        let mut plant = plant();

        plant
            .apply_transition(transition(
                1_000,
                OutputTransitionKind::Injector,
                0,
                OutputLevel::High,
            ))
            .unwrap();
        plant
            .apply_transition(transition(
                4_000,
                OutputTransitionKind::Injector,
                0,
                OutputLevel::Low,
            ))
            .unwrap();
        plant
            .apply_transition(transition(
                5_000,
                OutputTransitionKind::Ignition,
                0,
                OutputLevel::High,
            ))
            .unwrap();
        let event = plant
            .apply_transition(transition(
                7_000,
                OutputTransitionKind::Ignition,
                0,
                OutputLevel::Low,
            ))
            .unwrap();

        assert!(event.is_some());
        assert_eq!(plant.snapshot(PlantControls::idle()).combustion_events, 1);
    }

    #[test]
    fn missing_fuel_or_spark_produces_no_combustion() {
        let mut no_fuel = plant();
        no_fuel
            .apply_transition(transition(
                1_000,
                OutputTransitionKind::Ignition,
                0,
                OutputLevel::High,
            ))
            .unwrap();
        let event = no_fuel
            .apply_transition(transition(
                3_000,
                OutputTransitionKind::Ignition,
                0,
                OutputLevel::Low,
            ))
            .unwrap();
        assert!(event.is_none());

        let mut no_spark = plant();
        no_spark
            .apply_transition(transition(
                1_000,
                OutputTransitionKind::Injector,
                0,
                OutputLevel::High,
            ))
            .unwrap();
        no_spark
            .apply_transition(transition(
                4_000,
                OutputTransitionKind::Injector,
                0,
                OutputLevel::Low,
            ))
            .unwrap();
        assert_eq!(
            no_spark.snapshot(PlantControls::idle()).combustion_events,
            0
        );
    }

    #[test]
    fn capture_buffer_can_drive_plant() {
        let mut capture = FixedTransitionBuffer::<8>::new();
        capture
            .push(transition(
                1_000,
                OutputTransitionKind::Injector,
                0,
                OutputLevel::High,
            ))
            .unwrap();
        capture
            .push(transition(
                4_000,
                OutputTransitionKind::Injector,
                0,
                OutputLevel::Low,
            ))
            .unwrap();
        capture
            .push(transition(
                5_000,
                OutputTransitionKind::Ignition,
                0,
                OutputLevel::High,
            ))
            .unwrap();
        capture
            .push(transition(
                7_000,
                OutputTransitionKind::Ignition,
                0,
                OutputLevel::Low,
            ))
            .unwrap();

        let mut plant = plant();
        let report = plant.consume_capture(&capture).unwrap();

        assert_eq!(report.transitions, 4);
        assert_eq!(report.combustion_events, 1);
    }

    #[test]
    fn invalid_profile_cylinder_returns_error_instead_of_panicking() {
        let mut plant = ClosedLoopPlant::new(BadProfile, InjectorModel::gasoline(200));
        let err = plant
            .apply_transition(transition(
                1_000,
                OutputTransitionKind::Injector,
                0,
                OutputLevel::High,
            ))
            .unwrap_err();

        assert_eq!(err, PlantError::UnknownCylinder(9));
    }

    #[test]
    fn combustion_changes_future_rpm_and_map() {
        let mut baseline_plant = plant();
        let baseline = baseline_plant.advance_to(Micros::new(100_000), PlantControls::idle());

        let mut powered = plant();
        powered
            .apply_transition(transition(
                1_000,
                OutputTransitionKind::Injector,
                0,
                OutputLevel::High,
            ))
            .unwrap();
        powered
            .apply_transition(transition(
                6_000,
                OutputTransitionKind::Injector,
                0,
                OutputLevel::Low,
            ))
            .unwrap();
        powered
            .apply_transition(transition(
                7_000,
                OutputTransitionKind::Ignition,
                0,
                OutputLevel::High,
            ))
            .unwrap();
        powered
            .apply_transition(transition(
                10_000,
                OutputTransitionKind::Ignition,
                0,
                OutputLevel::Low,
            ))
            .unwrap();
        let snapshot = powered.advance_to(Micros::new(100_000), PlantControls::idle());

        assert!(snapshot.rpm.get() > baseline.rpm.get());
        assert_ne!(snapshot.map_kpa10.get(), baseline.map_kpa10.get());
    }
}
