use ecu_domain::{Degrees10, Lambda100, Micros, Rpm};
use ecu_runtime::{
    ControlInputs, EnrichmentInputs, FuelSensorInputs, IgnitionInputs, LambdaTrimInputs,
    TorqueInputs,
};

/// Board-side sensor/control values needed to construct runtime control inputs.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SplitControlSignals {
    pub clt_c: i16,
    pub launch_armed: bool,
    pub flat_shift_armed: bool,
    pub lambda_valid: bool,
    pub measured_lambda100: Lambda100,
    pub requested_open_loop: bool,
    pub tpsdot_pct_s: i16,
    pub mapdot_kpa_s: i16,
    pub maf_valid: bool,
    pub maf_x100: u16,
    pub iat_c10: i16,
    pub vbatt_mv: u16,
    pub baro_valid: bool,
    pub baro_kpa10: ecu_domain::Kpa10,
    pub spark_advance_x10: Degrees10,
    pub ignition_rpm: Rpm,
}

impl SplitControlSignals {
    pub const fn warm_bringup(ignition_rpm: Rpm) -> Self {
        Self {
            clt_c: 80,
            launch_armed: false,
            flat_shift_armed: false,
            lambda_valid: true,
            measured_lambda100: Lambda100::new(100),
            requested_open_loop: false,
            tpsdot_pct_s: 0,
            mapdot_kpa_s: 0,
            maf_valid: false,
            maf_x100: 0,
            iat_c10: 250,
            vbatt_mv: 12_000,
            baro_valid: false,
            baro_kpa10: ecu_domain::Kpa10::new(1010),
            spark_advance_x10: Degrees10::new(100),
            ignition_rpm,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SplitControlFrame {
    pub control: ControlInputs,
    pub launch_armed: bool,
    pub flat_shift_armed: bool,
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
            now_us,
            clt_c: signals.clt_c,
            just_started: false,
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
        fuel_sensors: FuelSensorInputs {
            maf_valid: signals.maf_valid,
            maf_x100: signals.maf_x100,
            iat_c10: signals.iat_c10,
            vbatt_mv: signals.vbatt_mv,
            baro_valid: signals.baro_valid,
            baro_kpa10: signals.baro_kpa10,
        },
        knock_intensity_x100: 0,
    }
}

pub fn split_control_frame_from<S: SplitControlSignalsSource>(
    source: &mut S,
    now_us: Micros,
    ignition_rpm: Rpm,
) -> Result<SplitControlFrame, S::Error> {
    let signals = source.signals(ignition_rpm)?;
    Ok(SplitControlFrame {
        control: split_control_inputs(now_us, signals),
        launch_armed: signals.launch_armed,
        flat_shift_armed: signals.flat_shift_armed,
    })
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
    Ok(split_control_frame_from(source, now_us, ignition_rpm)?.control)
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
            launch_armed: true,
            flat_shift_armed: false,
            lambda_valid: false,
            measured_lambda100: Lambda100::new(93),
            requested_open_loop: true,
            tpsdot_pct_s: 12,
            mapdot_kpa_s: -7,
            maf_valid: true,
            maf_x100: 456,
            iat_c10: 310,
            vbatt_mv: 13_100,
            baro_valid: true,
            baro_kpa10: ecu_domain::Kpa10::new(995),
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
        assert!(inputs.fuel_sensors.maf_valid);
        assert_eq!(inputs.fuel_sensors.maf_x100, 456);
        assert_eq!(inputs.fuel_sensors.iat_c10, 310);
        assert_eq!(inputs.fuel_sensors.vbatt_mv, 13_100);
        assert!(inputs.fuel_sensors.baro_valid);
        assert_eq!(inputs.fuel_sensors.baro_kpa10, ecu_domain::Kpa10::new(995));
    }

    #[test]
    fn split_control_frame_from_preserves_arming_and_rpm() {
        let signals = SplitControlSignals {
            clt_c: 42,
            launch_armed: true,
            flat_shift_armed: true,
            lambda_valid: false,
            measured_lambda100: Lambda100::new(93),
            requested_open_loop: true,
            tpsdot_pct_s: 12,
            mapdot_kpa_s: -7,
            maf_valid: false,
            maf_x100: 0,
            iat_c10: 250,
            vbatt_mv: 12_000,
            baro_valid: false,
            baro_kpa10: ecu_domain::Kpa10::new(1010),
            spark_advance_x10: Degrees10::new(150),
            ignition_rpm: Rpm::new(2_200),
        };

        struct Source {
            signals: SplitControlSignals,
            calls: u8,
        }

        impl SplitControlSignalsSource for Source {
            type Error = core::convert::Infallible;

            fn signals(&mut self, ignition_rpm: Rpm) -> Result<SplitControlSignals, Self::Error> {
                self.calls += 1;
                let mut signals = self.signals;
                signals.ignition_rpm = ignition_rpm;
                Ok(signals)
            }
        }

        let mut source = Source { signals, calls: 0 };
        let frame =
            split_control_frame_from(&mut source, Micros::new(20), Rpm::new(3_300)).unwrap();

        assert_eq!(source.calls, 1);
        assert!(frame.launch_armed);
        assert!(frame.flat_shift_armed);
        assert_eq!(frame.control.enrichment.clt_c, 42);
        assert_eq!(frame.control.ignition.rpm, Rpm::new(3_300));
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

    #[test]
    fn warm_bringup_defaults_arming_flags_to_false() {
        let signals = SplitControlSignals::warm_bringup(Rpm::new(1_100));

        assert!(!signals.launch_armed);
        assert!(!signals.flat_shift_armed);
    }
}
