use crate::{
    params::{BurnModel, FiredCycleConfig, KnockModelConfig},
    state::{ExhaustObservation, FiredCycleResult, KnockObservation, OpenSystemCycleResult},
};

pub fn estimate_knock(
    result: &FiredCycleResult,
    config: FiredCycleConfig,
    knock: KnockModelConfig,
) -> KnockObservation {
    let global_cycle_start = result
        .samples
        .first()
        .map(|_| core::f64::consts::PI)
        .unwrap_or(core::f64::consts::PI);
    let knock_start_rad = result
        .samples
        .first()
        .map(|sample| sample.crank_angle_rad)
        .unwrap_or(config.closed_cycle_start_rad);
    let combustion_start_rad = combustion_start_angle(config, global_cycle_start);
    let burn_end_rad = result
        .samples
        .iter()
        .find(|sample| sample.burn_fraction >= 0.999)
        .map(|sample| sample.crank_angle_rad)
        .unwrap_or_else(|| {
            let burn_duration_rad = match config.combustion.burn_model {
                BurnModel::SingleWiebe { duration_rad, .. } => duration_rad,
                BurnModel::DoubleWiebe {
                    premixed_duration_rad,
                    main_duration_rad,
                    ..
                } => premixed_duration_rad.max(main_duration_rad),
            };
            combustion_start_rad + burn_duration_rad
        });
    let mut integral = 0.0;
    let mut onset = None;
    let gamma_u = config.gas.gamma();
    let p_soc = pressure_at_or_before(result, combustion_start_rad).unwrap_or(
        result
            .samples
            .first()
            .map(|s| s.pressure_pa)
            .unwrap_or(101_325.0),
    );
    let t_soc = temperature_at_or_before(result, combustion_start_rad).unwrap_or(
        result
            .samples
            .first()
            .map(|s| s.temperature_k)
            .unwrap_or(config.initial_temperature_k),
    );

    for pair in result.samples.windows(2) {
        let current = pair[0];
        let next = pair[1];
        if current.crank_angle_rad < knock_start_rad || current.crank_angle_rad > burn_end_rad {
            continue;
        }
        let omega_rad_per_s = config.rpm * core::f64::consts::TAU / 60.0;
        let dt = (next.crank_angle_rad - current.crank_angle_rad) / omega_rad_per_s;
        let p_atm = current.pressure_pa / 101_325.0;
        let tu = t_soc * (current.pressure_pa / p_soc).powf((gamma_u - 1.0) / gamma_u);
        let tau = knock.a
            * (knock.octane_number / 100.0).powf(knock.n1)
            * p_atm.powf(knock.n2)
            * (knock.b / tu).exp();
        if tau.is_finite() && tau > 0.0 {
            integral += dt / tau;
        }
        if onset.is_none() && integral >= 1.0 {
            onset = Some(current.crank_angle_rad);
        }
    }

    KnockObservation {
        knock_integral: integral,
        knock_margin: (1.0 - integral).clamp(-1.0, 1.0),
        predicted_onset_angle_rad: onset,
    }
}

pub(crate) fn combustion_start_angle(config: FiredCycleConfig, cycle_start: f64) -> f64 {
    let firing_tdc_rad = cycle_start + 3.0 * core::f64::consts::PI;
    firing_tdc_rad - config.combustion.spark_angle_rad
        + spark_to_soc_delay(config.combustion.burn_model)
}

fn spark_to_soc_delay(model: BurnModel) -> f64 {
    match model {
        BurnModel::SingleWiebe {
            spark_to_soc_delay_rad,
            ..
        } => spark_to_soc_delay_rad,
        BurnModel::DoubleWiebe {
            spark_to_soc_delay_rad,
            ..
        } => spark_to_soc_delay_rad,
    }
}

pub fn observe_lambda(result: &FiredCycleResult) -> f64 {
    result.lambda
}

pub fn estimate_exhaust(result: &FiredCycleResult) -> ExhaustObservation {
    let mean_exhaust_temperature_k = result
        .samples
        .last()
        .map(|sample| sample.temperature_k)
        .unwrap_or(0.0);
    ExhaustObservation {
        mean_exhaust_temperature_k,
        exhaust_enthalpy_j: result.exhaust_enthalpy_j,
    }
}

pub fn estimate_exhaust_from_open_cycle(result: &OpenSystemCycleResult) -> ExhaustObservation {
    let mean_exhaust_temperature_k = if result.exhaust_enthalpy_j > 0.0 {
        result.exhaust_enthalpy_temperature_j / result.exhaust_enthalpy_j
    } else {
        result
            .samples
            .last()
            .map(|sample| sample.cylinder_temperature_k)
            .unwrap_or(0.0)
    };

    ExhaustObservation {
        mean_exhaust_temperature_k,
        exhaust_enthalpy_j: result.exhaust_enthalpy_j,
    }
}

fn pressure_at_or_before(result: &FiredCycleResult, angle_rad: f64) -> Option<f64> {
    result
        .samples
        .iter()
        .take_while(|sample| sample.crank_angle_rad <= angle_rad)
        .last()
        .map(|sample| sample.pressure_pa)
}

fn temperature_at_or_before(result: &FiredCycleResult, angle_rad: f64) -> Option<f64> {
    result
        .samples
        .iter()
        .take_while(|sample| sample.crank_angle_rad <= angle_rad)
        .last()
        .map(|sample| sample.temperature_k)
}
