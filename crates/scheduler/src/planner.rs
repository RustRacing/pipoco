use ecu_domain::Rpm;

use crate::{
    CrankSnapshot, ExclusiveChannel, FuelOutputProfile, FuelPlan, IgnitionPlan, InjectionPlan,
    Micros, OutputGroup, SparkOutputProfile, SparkPlan, TimedIgnitionPlan, TimedInjectionPlan,
    CRANK_REV_DEGREES10,
};

/// Converts spark intent plus crank state into timed ignition output plans.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct IgnitionScheduler {
    profile: SparkOutputProfile,
}

impl IgnitionScheduler {
    pub const fn new(profile: SparkOutputProfile) -> Self {
        Self { profile }
    }

    pub const fn profile(self) -> SparkOutputProfile {
        self.profile
    }

    pub fn event_count(self) -> usize {
        self.profile.events_per_crank_rev()
    }

    pub fn plan_event(
        self,
        crank: CrankSnapshot,
        event_index: usize,
        spark: SparkPlan,
    ) -> TimedIgnitionPlan {
        let tdc = self.profile.event_tdc_angle_deg10(event_index);
        let target = norm_deg10(
            i32::from(tdc) - i32::from(spark.advance.get()),
            CRANK_REV_DEGREES10,
        );
        let current = norm_deg10(crank.angle_deg10.get() as i32, CRANK_REV_DEGREES10);
        let delta = forward_angle_delta_deg10(current, target, CRANK_REV_DEGREES10);
        let spark_delay_us = micros_for_angle_delta(delta, crank.rpm);
        let spark_at = Micros::new(crank.now_us.get().saturating_add(spark_delay_us));
        let dwell_us = u32::from(spark.dwell.get());
        let dwell_start = if spark_delay_us > dwell_us {
            Micros::new(crank.now_us.get().saturating_add(spark_delay_us - dwell_us))
        } else {
            crank.now_us
        };
        let fire_at = if spark_at.get() > dwell_start.get() {
            spark_at
        } else {
            Micros::new(dwell_start.get().saturating_add(1))
        };

        TimedIgnitionPlan {
            plan: IgnitionPlan {
                output: ExclusiveChannel::new(
                    OutputGroup::Ignition,
                    self.profile.ignition_channel(event_index),
                ),
                dwell: spark.dwell,
                advance: spark.advance,
            },
            start_at: dwell_start,
            end_at: fire_at,
        }
    }
}

/// Converts fuel intent into timed injector output plans.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct InjectionScheduler {
    profile: FuelOutputProfile,
}

impl InjectionScheduler {
    pub const fn new(profile: FuelOutputProfile) -> Self {
        Self { profile }
    }

    pub const fn profile(self) -> FuelOutputProfile {
        self.profile
    }

    pub fn event_count(self) -> usize {
        self.profile.events_per_pulse()
    }

    pub fn plan_event(
        self,
        crank: CrankSnapshot,
        event_index: usize,
        fuel: FuelPlan,
    ) -> TimedInjectionPlan {
        let start_at = crank.now_us;
        let end_at = Micros::new(
            start_at
                .get()
                .saturating_add(u32::from(fuel.pulse_width.get()))
                .max(start_at.get().saturating_add(1)),
        );

        TimedInjectionPlan {
            plan: InjectionPlan {
                output: ExclusiveChannel::new(
                    OutputGroup::Injector,
                    self.profile.injector_channel(event_index),
                ),
                pulse_width: fuel.pulse_width,
            },
            start_at,
            end_at,
        }
    }
}

fn norm_deg10(value: i32, modulo: u16) -> u16 {
    value.rem_euclid(i32::from(modulo)) as u16
}

fn forward_angle_delta_deg10(current: u16, target: u16, modulo: u16) -> u16 {
    if target > current {
        target - current
    } else {
        modulo - current + target
    }
}

fn micros_for_angle_delta(delta_deg10: u16, rpm: Rpm) -> u32 {
    let rpm = u64::from(rpm.get());
    if rpm == 0 {
        return 0;
    }

    let micros = 60_000_000u64 * u64::from(delta_deg10) / (u64::from(CRANK_REV_DEGREES10) * rpm);
    micros.clamp(1, u64::from(u32::MAX)) as u32
}
