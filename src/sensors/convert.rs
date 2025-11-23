//! ADC and divider conversion helpers (no_std)

/// Convert ADC counts to millivolts
pub fn counts_to_mv(counts: u16, vref_mv: u16, bits: u8) -> u32 {
    let max = (1u32 << bits) - 1;
    (counts as u32) * (vref_mv as u32) / max
}

/// Divider configuration for resistive sensors
#[derive(Copy, Clone)]
pub enum DividerConfig {
    /// Vref -- R_known -- node -- R_sensor -- GND (pullup on top)
    PullupTop,
    /// Vref -- R_sensor -- node -- R_known -- GND (pulldown at bottom)
    PulldownBottom,
}

/// Compute sensor resistance (ohms) from divider voltage
pub fn node_mv_to_resistance_ohms(
    v_node_mv: u32,
    vref_mv: u32,
    r_known_ohms: u32,
    cfg: DividerConfig,
) -> u32 {
    match cfg {
        DividerConfig::PullupTop => {
            // Vnode = Vref * Rs / (Rk + Rs) => Rs = Rk * Vnode / (Vref - Vnode)
            if v_node_mv >= vref_mv {
                return u32::MAX;
            }
            (r_known_ohms as u64 * v_node_mv as u64 / (vref_mv as u64 - v_node_mv as u64)) as u32
        }
        DividerConfig::PulldownBottom => {
            // Vnode = Vref * Rk / (Rk + Rs) => Rs = Rk * (Vref - Vnode) / Vnode
            if v_node_mv == 0 {
                return u32::MAX;
            }
            (r_known_ohms as u64 * (vref_mv as u64 - v_node_mv as u64) / v_node_mv as u64) as u32
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_counts_to_mv() {
        assert_eq!(counts_to_mv(2048, 3300, 12), 1650);
    }

    #[test]
    fn test_divider_pullup() {
        let r = node_mv_to_resistance_ohms(1650, 3300, 10000, DividerConfig::PullupTop);
        // 10k top, 1/2 Vref -> R_sensor ~= 10k
        assert!(r > 9000 && r < 11000);
    }
}
