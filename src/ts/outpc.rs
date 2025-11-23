//! OUTPC (runtime data block) exposed to TunerStudio
//!
//! Keep it small and aligned. Scales chosen for TS-friendly units.

#[repr(C, packed)]
#[derive(Copy, Clone, Default)]
pub struct Outpc {
    pub rpm: u16,         // RPM
    pub map_kpa_x10: u16, // kPa*10
    pub tps_percent: u8,  // %
    pub clt_c: i16,       // degC
    pub iat_c: i16,       // degC
    pub vbatt_mv: u16,    // mV
    pub lambda_x100: u16, // lambda*100 (optional)
    pub pw_us: u16,       // injector pulse width (us)
    pub dwell_us: u16,    // coil dwell (us)
    pub advance_x10: i16, // ignition advance deg*10
    pub synced: u8,       // 0/1
    // Extended fields for TS parity
    pub target_afr_x10: u16,        // AFR*10
    pub ego_correction_percent: u8, // % (100 = no change)
    pub ego_sensor: u8,             // 0:none 1:NB 2:WB
    pub mapdot_kpa_s: i16,          // kPa/s
    pub tpsdot_pct_s: i16,          // %/s
    pub inj_duty_x10: u16,          // %*10
    pub idle_duty_x10: u16,         // %*10
    pub fan_state: u8,              // 0/1
    pub engine_state: u16,          // bitfield: WUE/ASE/CL/DFCO
    pub baro_kpa: u16,              // kPa
    pub gear: u8,                   // 0=unknown
}

impl Outpc {
    pub fn as_bytes(&self) -> &[u8] {
        unsafe {
            core::slice::from_raw_parts(
                (self as *const Outpc) as *const u8,
                core::mem::size_of::<Outpc>(),
            )
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn outpc_size_stable() {
        assert!(core::mem::size_of::<Outpc>() <= 64);
    }
}
