#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct LambdaTransportConfig {
    pub delay_crank_deg: u16,
    pub sensor_tau_ms: u16,
    pub exhaust_mixing_x1000: u16,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct LambdaTransportState<const N: usize> {
    pub ring: [u16; N],
    pub idx: usize,
    pub filled: usize,
    pub sensor_lambda_x1000: u16,
}

impl<const N: usize> LambdaTransportState<N> {
    #[cfg(test)]
    pub const fn new(initial_lambda_x1000: u16) -> Self {
        Self {
            ring: [initial_lambda_x1000; N],
            idx: 0,
            filled: 0,
            sensor_lambda_x1000: initial_lambda_x1000,
        }
    }
}

pub fn update_lambda_transport<const N: usize>(
    state: &mut LambdaTransportState<N>,
    config: LambdaTransportConfig,
    cylinder_lambda_x1000: u16,
    dt_ms: u16,
) -> u16 {
    if N == 0 {
        state.sensor_lambda_x1000 = cylinder_lambda_x1000;
        return state.sensor_lambda_x1000;
    }

    let delay_slots = delay_slots::<N>(config.delay_crank_deg);
    let read_idx = (state.idx + N - delay_slots) % N;
    let delayed_lambda = if state.filled < delay_slots {
        state.sensor_lambda_x1000
    } else {
        state.ring[read_idx]
    };
    let previous = state.ring[state.idx] as u32;
    let mix = config.exhaust_mixing_x1000.min(1000) as u32;
    let mixed = (previous * mix + cylinder_lambda_x1000 as u32 * (1000 - mix)) / 1000;

    state.ring[state.idx] = mixed.min(u16::MAX as u32) as u16;
    state.idx = (state.idx + 1) % N;
    state.filled = (state.filled + 1).min(N);
    state.sensor_lambda_x1000 = first_order_sensor(
        state.sensor_lambda_x1000,
        delayed_lambda,
        config.sensor_tau_ms,
        dt_ms,
    );
    state.sensor_lambda_x1000
}

fn delay_slots<const N: usize>(delay_crank_deg: u16) -> usize {
    if N == 0 {
        return 0;
    }
    let slots = (delay_crank_deg as usize).div_ceil(180).max(1);
    slots.min(N)
}

fn first_order_sensor(current: u16, target: u16, tau_ms: u16, dt_ms: u16) -> u16 {
    if tau_ms == 0 {
        return target;
    }
    let gain_x1000 = (dt_ms as u32 * 1000 / tau_ms as u32).clamp(1, 1000) as i32;
    let delta = target as i32 - current as i32;
    (current as i32 + delta * gain_x1000 / 1000).clamp(0, u16::MAX as i32) as u16
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn lambda_transport_delay_hides_enrichment_until_fifo_delay() {
        let config = LambdaTransportConfig {
            delay_crank_deg: 720,
            sensor_tau_ms: 0,
            exhaust_mixing_x1000: 0,
        };
        let mut state = LambdaTransportState::<8>::new(1000);

        assert_eq!(update_lambda_transport(&mut state, config, 900, 10), 1000);
        assert_eq!(update_lambda_transport(&mut state, config, 900, 10), 1000);
        assert_eq!(update_lambda_transport(&mut state, config, 900, 10), 1000);
        assert_eq!(update_lambda_transport(&mut state, config, 900, 10), 1000);
        assert_eq!(update_lambda_transport(&mut state, config, 900, 10), 900);
    }

    #[test]
    fn lambda_transport_sensor_lag_moves_gradually() {
        let config = LambdaTransportConfig {
            delay_crank_deg: 180,
            sensor_tau_ms: 100,
            exhaust_mixing_x1000: 0,
        };
        let mut state = LambdaTransportState::<4>::new(1000);

        update_lambda_transport(&mut state, config, 800, 10);
        let first = update_lambda_transport(&mut state, config, 800, 10);
        let second = update_lambda_transport(&mut state, config, 800, 10);

        assert!(first < 1000);
        assert!(second < first);
        assert!(second > 800);
    }
}
