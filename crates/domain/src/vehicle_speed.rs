use crate::{Micros, VehicleSpeedKph10};

/// Generic vehicle-speed pulse decoder configuration.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct VehicleSpeedPulseConfig {
    /// Number of VSS pulses per kilometer.
    pub pulses_per_km: u32,
    /// No-pulse timeout before reporting zero speed.
    pub timeout_us: u32,
    /// Clamp output to a configured plausible maximum.
    pub max_speed_kph10: VehicleSpeedKph10,
}

impl VehicleSpeedPulseConfig {
    pub const fn new(
        pulses_per_km: u32,
        timeout_us: u32,
        max_speed_kph10: VehicleSpeedKph10,
    ) -> Self {
        Self {
            pulses_per_km,
            timeout_us,
            max_speed_kph10,
        }
    }

    pub const fn validate(self) -> Result<(), VehicleSpeedPulseConfigError> {
        if self.pulses_per_km == 0 {
            return Err(VehicleSpeedPulseConfigError::ZeroPulsesPerKm);
        }
        if self.timeout_us == 0 {
            return Err(VehicleSpeedPulseConfigError::ZeroTimeout);
        }
        if self.max_speed_kph10.get() == 0 {
            return Err(VehicleSpeedPulseConfigError::ZeroMaxSpeed);
        }
        Ok(())
    }
}

impl Default for VehicleSpeedPulseConfig {
    fn default() -> Self {
        Self {
            pulses_per_km: 1,
            timeout_us: 1_000_000,
            max_speed_kph10: VehicleSpeedKph10::new(3_000),
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum VehicleSpeedPulseConfigError {
    ZeroPulsesPerKm,
    ZeroTimeout,
    ZeroMaxSpeed,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct VehicleSpeedPulseState {
    pub last_pulse_us: Option<Micros>,
    pub speed_kph10: VehicleSpeedKph10,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct VehicleSpeedPulseInput {
    pub now_us: Micros,
    pub pulse_seen: bool,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct VehicleSpeedPulseResult {
    pub speed_kph10: VehicleSpeedKph10,
    pub signal_present: bool,
    pub next_state: VehicleSpeedPulseState,
}

pub fn vehicle_speed_pulse_step(
    config: &VehicleSpeedPulseConfig,
    state: &VehicleSpeedPulseState,
    input: VehicleSpeedPulseInput,
) -> VehicleSpeedPulseResult {
    if config.validate().is_err() {
        return VehicleSpeedPulseResult {
            speed_kph10: VehicleSpeedKph10::new(0),
            signal_present: false,
            next_state: VehicleSpeedPulseState::default(),
        };
    }

    let mut next = *state;
    let mut signal_present = state.last_pulse_us.is_some();

    if input.pulse_seen {
        if let Some(last) = state.last_pulse_us {
            if let Some(period_us) = monotonic_elapsed_us(last, input.now_us) {
                if period_us > 0 {
                    next.speed_kph10 = speed_from_period_us(config, period_us);
                }
            } else {
                next.speed_kph10 = VehicleSpeedKph10::new(0);
            }
        }
        next.last_pulse_us = Some(input.now_us);
        signal_present = true;
    } else if let Some(last) = state.last_pulse_us {
        match monotonic_elapsed_us(last, input.now_us) {
            Some(elapsed_us) if elapsed_us > config.timeout_us => {
                next.speed_kph10 = VehicleSpeedKph10::new(0);
                signal_present = false;
            }
            Some(_) => {}
            None => {
                next.speed_kph10 = VehicleSpeedKph10::new(0);
                signal_present = false;
            }
        }
    }

    VehicleSpeedPulseResult {
        speed_kph10: next.speed_kph10,
        signal_present,
        next_state: next,
    }
}

fn monotonic_elapsed_us(last: Micros, now: Micros) -> Option<u32> {
    now.get().checked_sub(last.get())
}

fn speed_from_period_us(config: &VehicleSpeedPulseConfig, period_us: u32) -> VehicleSpeedKph10 {
    let denom = period_us as u64 * config.pulses_per_km as u64;
    let kph10 = 36_000_000_000u64 / denom;
    VehicleSpeedKph10::new((kph10.min(config.max_speed_kph10.get() as u64)) as u16)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn vehicle_speed_pulse_decoder_reports_speed_after_second_pulse() {
        let config = VehicleSpeedPulseConfig::new(10_000, 500_000, VehicleSpeedKph10::new(3_000));
        let first = vehicle_speed_pulse_step(
            &config,
            &VehicleSpeedPulseState::default(),
            VehicleSpeedPulseInput {
                now_us: Micros::new(1_000),
                pulse_seen: true,
            },
        );
        assert_eq!(first.speed_kph10.get(), 0);
        assert!(first.signal_present);

        let second = vehicle_speed_pulse_step(
            &config,
            &first.next_state,
            VehicleSpeedPulseInput {
                now_us: Micros::new(101_000),
                pulse_seen: true,
            },
        );

        assert_eq!(second.speed_kph10.get(), 36);
        assert!(second.signal_present);
    }

    #[test]
    fn vehicle_speed_pulse_decoder_times_out_to_zero() {
        let config = VehicleSpeedPulseConfig::new(10_000, 100_000, VehicleSpeedKph10::new(3_000));
        let state = VehicleSpeedPulseState {
            last_pulse_us: Some(Micros::new(10_000)),
            speed_kph10: VehicleSpeedKph10::new(500),
        };

        let result = vehicle_speed_pulse_step(
            &config,
            &state,
            VehicleSpeedPulseInput {
                now_us: Micros::new(120_001),
                pulse_seen: false,
            },
        );

        assert_eq!(result.speed_kph10.get(), 0);
        assert!(!result.signal_present);
    }

    #[test]
    fn vehicle_speed_pulse_decoder_clamps_implausible_speed() {
        let config = VehicleSpeedPulseConfig::new(10_000, 500_000, VehicleSpeedKph10::new(2_500));
        let state = VehicleSpeedPulseState {
            last_pulse_us: Some(Micros::new(1_000)),
            speed_kph10: VehicleSpeedKph10::new(0),
        };

        let result = vehicle_speed_pulse_step(
            &config,
            &state,
            VehicleSpeedPulseInput {
                now_us: Micros::new(1_001),
                pulse_seen: true,
            },
        );

        assert_eq!(result.speed_kph10.get(), 2_500);
    }

    #[test]
    fn signal_present_tracks_recent_pulse_activity_not_nonzero_speed() {
        let config = VehicleSpeedPulseConfig::new(10_000, 100_000, VehicleSpeedKph10::new(3_000));
        let first = vehicle_speed_pulse_step(
            &config,
            &VehicleSpeedPulseState::default(),
            VehicleSpeedPulseInput {
                now_us: Micros::new(10),
                pulse_seen: true,
            },
        );

        assert_eq!(first.speed_kph10.get(), 0);
        assert!(first.signal_present);

        let still_recent = vehicle_speed_pulse_step(
            &config,
            &first.next_state,
            VehicleSpeedPulseInput {
                now_us: Micros::new(20),
                pulse_seen: false,
            },
        );

        assert_eq!(still_recent.speed_kph10.get(), 0);
        assert!(still_recent.signal_present);
    }

    #[test]
    fn non_monotonic_timestamp_does_not_create_max_speed() {
        let config = VehicleSpeedPulseConfig::new(10_000, 100_000, VehicleSpeedKph10::new(3_000));
        let state = VehicleSpeedPulseState {
            last_pulse_us: Some(Micros::new(10_000)),
            speed_kph10: VehicleSpeedKph10::new(500),
        };

        let result = vehicle_speed_pulse_step(
            &config,
            &state,
            VehicleSpeedPulseInput {
                now_us: Micros::new(9_999),
                pulse_seen: true,
            },
        );

        assert_eq!(result.speed_kph10.get(), 0);
        assert!(result.signal_present);
        assert_eq!(result.next_state.last_pulse_us, Some(Micros::new(9_999)));
    }

    #[test]
    fn wrapback_without_pulse_clears_signal_instead_of_extending_timeout() {
        let config = VehicleSpeedPulseConfig::new(10_000, 100_000, VehicleSpeedKph10::new(3_000));
        let state = VehicleSpeedPulseState {
            last_pulse_us: Some(Micros::new(u32::MAX - 10)),
            speed_kph10: VehicleSpeedKph10::new(500),
        };

        let result = vehicle_speed_pulse_step(
            &config,
            &state,
            VehicleSpeedPulseInput {
                now_us: Micros::new(5),
                pulse_seen: false,
            },
        );

        assert_eq!(result.speed_kph10.get(), 0);
        assert!(!result.signal_present);
    }

    #[test]
    fn invalid_config_reports_no_signal() {
        let config = VehicleSpeedPulseConfig::new(0, 100_000, VehicleSpeedKph10::new(3_000));
        let state = VehicleSpeedPulseState {
            last_pulse_us: Some(Micros::new(10_000)),
            speed_kph10: VehicleSpeedKph10::new(500),
        };

        let result = vehicle_speed_pulse_step(
            &config,
            &state,
            VehicleSpeedPulseInput {
                now_us: Micros::new(20_000),
                pulse_seen: true,
            },
        );

        assert_eq!(result.next_state, VehicleSpeedPulseState::default());
        assert_eq!(result.speed_kph10.get(), 0);
        assert!(!result.signal_present);
    }
}
