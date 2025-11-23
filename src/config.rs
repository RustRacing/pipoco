//! ECU configuration for outputs and modes
//!
//! Defines injection/ignition modes, cylinder count, firing order, and channel maps.

use crate::scheduler::Channel;

/// Injection strategy
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum InjectionMode {
    /// All injectors fire together (batch)
    Batch,
    /// One injector per event following firing order
    Sequential,
}

/// Ignition strategy
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum IgnitionMode {
    /// Wasted spark pairs fire together
    Wasted,
    /// One coil per event following firing order (COP)
    Sequential,
}

/// Static output channel map
#[derive(Copy, Clone)]
pub struct OutputChannels {
    pub inj_channels: [Channel; 16],
    pub inj_count: u8,
    pub ign_channels: [Channel; 16],
    pub ign_count: u8,
}

impl OutputChannels {
    pub const fn for_4ch() -> Self {
        let mut inj = [Channel::from_index(0); 16];
        inj[0] = Channel::INJ1;
        inj[1] = Channel::INJ2;
        let mut ign = [Channel::from_index(0); 16];
        ign[0] = Channel::IGN1;
        ign[1] = Channel::IGN2;
        Self {
            inj_channels: inj,
            inj_count: 2,
            ign_channels: ign,
            ign_count: 2,
        }
    }
}

/// Engine configuration for scheduling
#[derive(Copy, Clone)]
pub struct EcuConfig {
    /// Number of cylinders
    pub cylinders: u8,
    /// Firing order as cylinder numbers (1-based)
    pub firing_order: &'static [u8],
    /// Injection mode
    pub injection_mode: InjectionMode,
    /// Ignition mode
    pub ignition_mode: IgnitionMode,
    /// Whether cam phase is available (true sequential requires phase knowledge)
    pub has_cam: bool,
    /// Output channel map
    pub outputs: OutputChannels,
    /// Per-cylinder injection angle offset in degrees*10 (0..3600)
    /// Index 0 corresponds to cylinder 1, etc. Only the first `cylinders` entries are used.
    pub inj_angle_btdc_x10: [u16; 16],
    /// Absolute TDC angle for each cylinder in degrees*10 relative to decoder's 0° reference.
    /// Only first `cylinders` entries are used.
    pub tdc_per_cyl_x10: [u16; 16],
    /// Reference offset (deg*10) for decoder 0° alignment, if needed for calibration.
    pub tooth0_angle_x10: u16,
}

impl EcuConfig {
    /// Conservative default: 4-cyl, batch injection, wasted spark, 1-3-4-2
    pub const fn default_4c_batch_wasted() -> Self {
        const FO: &[u8] = &[1, 3, 4, 2];
        const ANGLES: [u16; 16] = [0; 16];
        const TDC: [u16; 16] = [0; 16];
        Self {
            cylinders: 4,
            firing_order: FO,
            injection_mode: InjectionMode::Batch,
            ignition_mode: IgnitionMode::Wasted,
            has_cam: false,
            outputs: OutputChannels::for_4ch(),
            inj_angle_btdc_x10: ANGLES,
            tdc_per_cyl_x10: TDC,
            tooth0_angle_x10: 0,
        }
    }
}
