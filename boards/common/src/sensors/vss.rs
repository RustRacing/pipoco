use ecu_domain::{
    vehicle_speed_pulse_step, Micros, VehicleSpeedKph10, VehicleSpeedPulseConfig,
    VehicleSpeedPulseInput, VehicleSpeedPulseState,
};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct VehicleSpeedPulseDecoder {
    config: VehicleSpeedPulseConfig,
    state: VehicleSpeedPulseState,
}

impl VehicleSpeedPulseDecoder {
    pub const fn new(config: VehicleSpeedPulseConfig) -> Self {
        Self {
            config,
            state: VehicleSpeedPulseState {
                last_pulse_us: None,
                speed_kph10: VehicleSpeedKph10::new(0),
            },
        }
    }

    pub fn on_pulse(&mut self, now_us: Micros) -> VehicleSpeedKph10 {
        let result = vehicle_speed_pulse_step(
            &self.config,
            &self.state,
            VehicleSpeedPulseInput {
                now_us,
                pulse_seen: true,
            },
        );
        self.state = result.next_state;
        result.speed_kph10
    }

    pub fn poll(&mut self, now_us: Micros) -> VehicleSpeedKph10 {
        let result = vehicle_speed_pulse_step(
            &self.config,
            &self.state,
            VehicleSpeedPulseInput {
                now_us,
                pulse_seen: false,
            },
        );
        self.state = result.next_state;
        result.speed_kph10
    }

    pub const fn speed(&self) -> VehicleSpeedKph10 {
        self.state.speed_kph10
    }

    pub const fn state(&self) -> VehicleSpeedPulseState {
        self.state
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn pulse_decoder_updates_after_second_edge() {
        let mut decoder = VehicleSpeedPulseDecoder::new(VehicleSpeedPulseConfig::new(
            10_000,
            500_000,
            VehicleSpeedKph10::new(3_000),
        ));

        assert_eq!(decoder.on_pulse(Micros::new(1_000)).get(), 0);
        assert_eq!(decoder.on_pulse(Micros::new(101_000)).get(), 36);
        assert_eq!(decoder.speed().get(), 36);
    }

    #[test]
    fn pulse_decoder_poll_applies_timeout() {
        let mut decoder = VehicleSpeedPulseDecoder::new(VehicleSpeedPulseConfig::new(
            10_000,
            100_000,
            VehicleSpeedKph10::new(3_000),
        ));

        decoder.on_pulse(Micros::new(1_000));
        decoder.on_pulse(Micros::new(101_000));
        assert_eq!(decoder.poll(Micros::new(201_001)).get(), 0);
    }
}
