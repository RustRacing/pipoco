use ecu_domain::{Degrees10, Rpm, Ticks};

use crate::profile::TriggerSpeed;

use crate::ENGINE_CYCLE_DEGREES10;

pub(crate) const RATIO_SCALE_X1000: u64 = 1000;
pub(crate) const CRANK_REV_DEGREES10: u32 = 3600;
const MICROS_PER_MINUTE: u64 = 60_000_000;

pub const fn normalize_engine_cycle_deg10(angle: Degrees10) -> Degrees10 {
    let mut normalized = angle.get() % ENGINE_CYCLE_DEGREES10;
    if normalized < 0 {
        normalized += ENGINE_CYCLE_DEGREES10;
    }

    Degrees10::new(normalized)
}

pub(crate) fn normalize_engine_cycle_deg10_i32(angle: i32) -> Degrees10 {
    let mut normalized = angle % (ENGINE_CYCLE_DEGREES10 as i32);
    if normalized < 0 {
        normalized += ENGINE_CYCLE_DEGREES10 as i32;
    }

    Degrees10::new(normalized as i16)
}

pub(crate) const fn elapsed_ticks(timestamp: Ticks, previous: Ticks) -> Ticks {
    Ticks::new(timestamp.get().wrapping_sub(previous.get()))
}

pub(crate) fn missing_tooth_gap_detected(
    interval: Ticks,
    previous_interval: Ticks,
    threshold_x1000: u16,
) -> Result<(bool, u16), crate::diag::SyncLossReason> {
    let interval_ticks = interval.get();
    let previous_ticks = previous_interval.get();
    if interval_ticks == 0 || previous_ticks == 0 {
        return Err(crate::diag::SyncLossReason::InvalidGapRatio);
    }

    let scaled_interval = u64::from(interval_ticks).saturating_mul(RATIO_SCALE_X1000);
    let scaled_threshold = u64::from(previous_ticks).saturating_mul(u64::from(threshold_x1000));
    let ratio = scaled_interval / u64::from(previous_ticks);

    Ok((scaled_interval >= scaled_threshold, saturating_u16(ratio)))
}

pub(crate) fn rpm_from_tooth_interval(
    interval: Ticks,
    nominal_teeth: u8,
    primary_speed: TriggerSpeed,
) -> Rpm {
    if interval.get() == 0 || nominal_teeth == 0 {
        return Rpm::new(0);
    }

    let revolutions_per_primary_rev = match primary_speed {
        TriggerSpeed::Crank => 1,
        TriggerSpeed::Cam => 2,
    };
    let revolution_ticks = u64::from(interval.get()).saturating_mul(u64::from(nominal_teeth));
    if revolution_ticks == 0 {
        return Rpm::new(0);
    }

    let rpm = MICROS_PER_MINUTE.saturating_mul(revolutions_per_primary_rev) / revolution_ticks;
    Rpm::new(saturating_u16(rpm))
}

fn saturating_u16(value: u64) -> u16 {
    if value > u64::from(u16::MAX) {
        u16::MAX
    } else {
        value as u16
    }
}
