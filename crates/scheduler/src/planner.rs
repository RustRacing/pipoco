use ecu_domain::{
    forward_angle_delta_deg10, micros_for_angle_delta, norm_deg10, CRANK_REV_DEGREES10,
    ENGINE_CYCLE_DEGREES10,
};

use crate::{
    CrankSnapshot, ExclusiveChannel, FuelOutputProfile, FuelPlan, IgnitionOutputProfile,
    IgnitionPlan, InjectionOutputProfile, InjectionPlan, Micros, OutputGroup, SparkOutputProfile,
    SparkPlan, TimedIgnitionPlan, TimedInjectionPlan,
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
        let earliest_start = Micros::new(crank.now_us.get().saturating_add(1));
        let dwell_start = if spark_delay_us > dwell_us {
            Micros::new(crank.now_us.get().saturating_add(spark_delay_us - dwell_us))
        } else {
            earliest_start
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
        let event_count = self.event_count().max(1) as u32;
        let rev_slot_deg10 = (u32::from(CRANK_REV_DEGREES10) / event_count).max(1) as u16;
        let current = norm_deg10(crank.angle_deg10.get() as i32, CRANK_REV_DEGREES10);
        let target = ((event_index as u32 % event_count) * u32::from(rev_slot_deg10)) as u16;
        let delta = forward_angle_delta_deg10(current, target, CRANK_REV_DEGREES10);
        let start_delay_us = micros_for_angle_delta(delta.max(1), crank.rpm);
        let start_at = Micros::new(crank.now_us.get().saturating_add(start_delay_us));
        let end_at = Micros::new(
            start_at
                .get()
                .saturating_add(fuel.pulse_width.get())
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

/// Converts full-ECU ignition intent plus phased crank state into timed plans.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct FullEcuIgnitionScheduler {
    profile: IgnitionOutputProfile,
    event_count: usize,
}

impl FullEcuIgnitionScheduler {
    pub const fn new(profile: IgnitionOutputProfile, event_count: usize) -> Self {
        Self {
            profile,
            event_count,
        }
    }

    pub const fn profile(self) -> IgnitionOutputProfile {
        self.profile
    }

    pub const fn event_count(self) -> usize {
        self.event_count
    }

    pub fn plan_event(
        self,
        crank: CrankSnapshot,
        event_index: usize,
        spark: SparkPlan,
    ) -> TimedIgnitionPlan {
        let tdc = full_cycle_slot_angle_deg10(event_index, self.event_count);
        let target = norm_deg10(
            i32::from(tdc) - i32::from(spark.advance.get()),
            ENGINE_CYCLE_DEGREES10,
        );
        let current = norm_deg10(crank.angle_deg10.get() as i32, ENGINE_CYCLE_DEGREES10);
        let delta = forward_angle_delta_deg10(current, target, ENGINE_CYCLE_DEGREES10);
        let spark_delay_us = micros_for_angle_delta(delta, crank.rpm);
        let spark_at = Micros::new(crank.now_us.get().saturating_add(spark_delay_us));
        let dwell_us = u32::from(spark.dwell.get());
        let earliest_start = Micros::new(crank.now_us.get().saturating_add(1));
        let dwell_start = if spark_delay_us > dwell_us {
            Micros::new(crank.now_us.get().saturating_add(spark_delay_us - dwell_us))
        } else {
            earliest_start
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

/// Converts full-ECU fuel intent plus phased crank state into timed plans.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct FullEcuInjectionScheduler {
    profile: InjectionOutputProfile,
}

impl FullEcuInjectionScheduler {
    pub const fn new(profile: InjectionOutputProfile) -> Self {
        Self { profile }
    }

    pub const fn profile(self) -> InjectionOutputProfile {
        self.profile
    }

    pub fn event_count(self) -> usize {
        self.profile.event_count()
    }

    pub fn plan_event(
        self,
        crank: CrankSnapshot,
        event_index: usize,
        fuel: FuelPlan,
    ) -> TimedInjectionPlan {
        let event_count = self.event_count();
        let current = norm_deg10(crank.angle_deg10.get() as i32, ENGINE_CYCLE_DEGREES10);
        let target = full_cycle_slot_angle_deg10(event_index, event_count);
        let delta = forward_angle_delta_deg10(current, target, ENGINE_CYCLE_DEGREES10);
        let start_delay_us = micros_for_angle_delta(delta.max(1), crank.rpm);
        let start_at = Micros::new(crank.now_us.get().saturating_add(start_delay_us));
        let end_at = Micros::new(
            start_at
                .get()
                .saturating_add(fuel.pulse_width.get())
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

fn full_cycle_slot_angle_deg10(event_index: usize, event_count: usize) -> u16 {
    let events = event_count.max(1) as u32;
    let slot = (event_index % event_count.max(1)) as u32;
    ((slot * u32::from(ENGINE_CYCLE_DEGREES10)) / events) as u16
}
