use ecu_domain::{Degrees10, Lambda100, Micros, Rpm};
use ecu_runtime::{
    ControlInputs, EnrichmentInputs, IgnitionInputs, LambdaTrimInputs, TorqueInputs,
};

/// Board-side sensor/control values needed to construct runtime control inputs.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SplitControlSignals {
    pub clt_c: i16,
    pub lambda_valid: bool,
    pub measured_lambda100: Lambda100,
    pub requested_open_loop: bool,
    pub tpsdot_pct_s: i16,
    pub mapdot_kpa_s: i16,
    pub spark_advance_x10: Degrees10,
    pub ignition_rpm: Rpm,
}

impl SplitControlSignals {
    pub const fn warm_bringup(ignition_rpm: Rpm) -> Self {
        Self {
            clt_c: 80,
            lambda_valid: true,
            measured_lambda100: Lambda100::new(100),
            requested_open_loop: false,
            tpsdot_pct_s: 0,
            mapdot_kpa_s: 0,
            spark_advance_x10: Degrees10::new(100),
            ignition_rpm,
        }
    }
}

pub fn split_control_inputs(now_us: Micros, signals: SplitControlSignals) -> ControlInputs {
    ControlInputs {
        enrichment: EnrichmentInputs {
            now_us,
            clt_c: signals.clt_c,
            cranking: false,
            just_started: false,
            tpsdot_pct_s: signals.tpsdot_pct_s,
            mapdot_kpa_s: signals.mapdot_kpa_s,
        },
        lambda: LambdaTrimInputs {
            clt_c: signals.clt_c,
            lambda_valid: signals.lambda_valid,
            measured_lambda100: signals.measured_lambda100,
            requested_open_loop: signals.requested_open_loop,
        },
        torque: TorqueInputs::new(90, 90, 90, 90, 90),
        ignition: IgnitionInputs::new(
            signals.spark_advance_x10,
            0,
            0,
            0,
            false,
            signals.ignition_rpm,
        ),
    }
}

pub trait SplitControlSignalsSource {
    type Error;

    fn signals(&mut self, ignition_rpm: Rpm) -> Result<SplitControlSignals, Self::Error>;
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct WarmBringupControlSignals;

impl SplitControlSignalsSource for WarmBringupControlSignals {
    type Error = core::convert::Infallible;

    fn signals(&mut self, ignition_rpm: Rpm) -> Result<SplitControlSignals, Self::Error> {
        Ok(SplitControlSignals::warm_bringup(ignition_rpm))
    }
}

pub fn split_control_inputs_from<S: SplitControlSignalsSource>(
    source: &mut S,
    now_us: Micros,
    ignition_rpm: Rpm,
) -> Result<ControlInputs, S::Error> {
    Ok(split_control_inputs(now_us, source.signals(ignition_rpm)?))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn warm_bringup_uses_live_ignition_rpm() {
        let now = Micros::new(12_345);
        let inputs = split_control_inputs(now, SplitControlSignals::warm_bringup(Rpm::new(1_750)));

        assert_eq!(inputs.enrichment.now_us, now);
        assert_eq!(inputs.enrichment.clt_c, 80);
        assert_eq!(inputs.lambda.measured_lambda100, Lambda100::new(100));
        assert_eq!(inputs.ignition.rpm, Rpm::new(1_750));
    }

    #[test]
    fn split_control_inputs_preserves_board_sensor_derivatives() {
        let signals = SplitControlSignals {
            clt_c: 42,
            lambda_valid: false,
            measured_lambda100: Lambda100::new(93),
            requested_open_loop: true,
            tpsdot_pct_s: 12,
            mapdot_kpa_s: -7,
            spark_advance_x10: Degrees10::new(150),
            ignition_rpm: Rpm::new(2_200),
        };

        let inputs = split_control_inputs(Micros::new(20), signals);

        assert_eq!(inputs.enrichment.clt_c, 42);
        assert_eq!(inputs.enrichment.tpsdot_pct_s, 12);
        assert_eq!(inputs.enrichment.mapdot_kpa_s, -7);
        assert!(!inputs.lambda.lambda_valid);
        assert!(inputs.lambda.requested_open_loop);
        assert_eq!(inputs.ignition.base_advance_deg10, Degrees10::new(150));
        assert_eq!(inputs.ignition.rpm, Rpm::new(2_200));
    }

    #[test]
    fn split_control_inputs_from_uses_signal_source() {
        let mut source = WarmBringupControlSignals;
        let inputs =
            split_control_inputs_from(&mut source, Micros::new(77), Rpm::new(2_500)).unwrap();

        assert_eq!(inputs.enrichment.now_us, Micros::new(77));
        assert_eq!(inputs.enrichment.clt_c, 80);
        assert_eq!(inputs.ignition.rpm, Rpm::new(2_500));
    }
}
