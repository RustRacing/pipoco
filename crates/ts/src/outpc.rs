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
    pub vehicle_speed_kph_x10: u16, // km/h*10
    pub maf_x100: u16,              // source-native airflow*100
    pub knock_x100: u16,            // normalized knock level*100
    pub cam_phase_deg10: i16,       // measured cam phase deg*10
    pub cam_phase_valid: u8,        // 0/1
    pub sensor_validity_flags: u8,  // bit0 MAF, bit1 knock, bit2 VSS, bit3 lambda
}

impl Outpc {
    pub const WIRE_LEN: usize = 48;

    pub fn encode(&self, out: &mut [u8]) -> Option<usize> {
        if out.len() < Self::WIRE_LEN {
            return None;
        }

        out[0..2].copy_from_slice(&self.rpm.to_le_bytes());
        out[2..4].copy_from_slice(&self.map_kpa_x10.to_le_bytes());
        out[4] = self.tps_percent;
        out[5..7].copy_from_slice(&self.clt_c.to_le_bytes());
        out[7..9].copy_from_slice(&self.iat_c.to_le_bytes());
        out[9..11].copy_from_slice(&self.vbatt_mv.to_le_bytes());
        out[11..13].copy_from_slice(&self.lambda_x100.to_le_bytes());
        out[13..15].copy_from_slice(&self.pw_us.to_le_bytes());
        out[15..17].copy_from_slice(&self.dwell_us.to_le_bytes());
        out[17..19].copy_from_slice(&self.advance_x10.to_le_bytes());
        out[19] = self.synced;
        out[20..22].copy_from_slice(&self.target_afr_x10.to_le_bytes());
        out[22] = self.ego_correction_percent;
        out[23] = self.ego_sensor;
        out[24..26].copy_from_slice(&self.mapdot_kpa_s.to_le_bytes());
        out[26..28].copy_from_slice(&self.tpsdot_pct_s.to_le_bytes());
        out[28..30].copy_from_slice(&self.inj_duty_x10.to_le_bytes());
        out[30..32].copy_from_slice(&self.idle_duty_x10.to_le_bytes());
        out[32] = self.fan_state;
        out[33..35].copy_from_slice(&self.engine_state.to_le_bytes());
        out[35..37].copy_from_slice(&self.baro_kpa.to_le_bytes());
        out[37] = self.gear;
        out[38..40].copy_from_slice(&self.vehicle_speed_kph_x10.to_le_bytes());
        out[40..42].copy_from_slice(&self.maf_x100.to_le_bytes());
        out[42..44].copy_from_slice(&self.knock_x100.to_le_bytes());
        out[44..46].copy_from_slice(&self.cam_phase_deg10.to_le_bytes());
        out[46] = self.cam_phase_valid;
        out[47] = self.sensor_validity_flags;

        Some(Self::WIRE_LEN)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    const _: () = assert!(Outpc::WIRE_LEN <= 64);

    #[test]
    fn outpc_size_stable() {
        assert_eq!(Outpc::WIRE_LEN, 48);
    }

    #[test]
    fn outpc_sensor_extension_offsets_stay_stable() {
        assert_eq!(core::mem::size_of::<Outpc>(), 48);
        assert_eq!(core::mem::offset_of!(Outpc, vehicle_speed_kph_x10), 38);
        assert_eq!(core::mem::offset_of!(Outpc, maf_x100), 40);
        assert_eq!(core::mem::offset_of!(Outpc, knock_x100), 42);
        assert_eq!(core::mem::offset_of!(Outpc, cam_phase_deg10), 44);
        assert_eq!(core::mem::offset_of!(Outpc, cam_phase_valid), 46);
        assert_eq!(core::mem::offset_of!(Outpc, sensor_validity_flags), 47);
    }

    #[test]
    fn outpc_wire_schema_is_explicit_little_endian() {
        let outpc = Outpc {
            rpm: 0x0102,
            map_kpa_x10: 0x0304,
            tps_percent: 5,
            clt_c: -6,
            iat_c: 0x0708,
            vbatt_mv: 0x090A,
            lambda_x100: 0x0B0C,
            pw_us: 0x0D0E,
            dwell_us: 0x0F10,
            advance_x10: -0x1112,
            synced: 0x13,
            target_afr_x10: 0x1415,
            ego_correction_percent: 0x16,
            ego_sensor: 0x17,
            mapdot_kpa_s: -0x1819,
            tpsdot_pct_s: 0x1A1B,
            inj_duty_x10: 0x1C1D,
            idle_duty_x10: 0x1E1F,
            fan_state: 0x20,
            engine_state: 0x2122,
            baro_kpa: 0x2324,
            gear: 0x25,
            vehicle_speed_kph_x10: 0x2627,
            maf_x100: 0x2829,
            knock_x100: 0x2A2B,
            cam_phase_deg10: -0x2C2D,
            cam_phase_valid: 0x2E,
            sensor_validity_flags: 0x2F,
        };
        let mut encoded = [0u8; Outpc::WIRE_LEN];
        assert_eq!(outpc.encode(&mut encoded), Some(Outpc::WIRE_LEN));

        let mut expected = [0u8; Outpc::WIRE_LEN];
        expected[0..2].copy_from_slice(&0x0102u16.to_le_bytes());
        expected[2..4].copy_from_slice(&0x0304u16.to_le_bytes());
        expected[4] = 5;
        expected[5..7].copy_from_slice(&(-6i16).to_le_bytes());
        expected[7..9].copy_from_slice(&0x0708i16.to_le_bytes());
        expected[9..11].copy_from_slice(&0x090Au16.to_le_bytes());
        expected[11..13].copy_from_slice(&0x0B0Cu16.to_le_bytes());
        expected[13..15].copy_from_slice(&0x0D0Eu16.to_le_bytes());
        expected[15..17].copy_from_slice(&0x0F10u16.to_le_bytes());
        expected[17..19].copy_from_slice(&(-0x1112i16).to_le_bytes());
        expected[19] = 0x13;
        expected[20..22].copy_from_slice(&0x1415u16.to_le_bytes());
        expected[22] = 0x16;
        expected[23] = 0x17;
        expected[24..26].copy_from_slice(&(-0x1819i16).to_le_bytes());
        expected[26..28].copy_from_slice(&0x1A1Bi16.to_le_bytes());
        expected[28..30].copy_from_slice(&0x1C1Du16.to_le_bytes());
        expected[30..32].copy_from_slice(&0x1E1Fu16.to_le_bytes());
        expected[32] = 0x20;
        expected[33..35].copy_from_slice(&0x2122u16.to_le_bytes());
        expected[35..37].copy_from_slice(&0x2324u16.to_le_bytes());
        expected[37] = 0x25;
        expected[38..40].copy_from_slice(&0x2627u16.to_le_bytes());
        expected[40..42].copy_from_slice(&0x2829u16.to_le_bytes());
        expected[42..44].copy_from_slice(&0x2A2Bu16.to_le_bytes());
        expected[44..46].copy_from_slice(&(-0x2C2Di16).to_le_bytes());
        expected[46] = 0x2E;
        expected[47] = 0x2F;

        assert_eq!(encoded, expected);
    }

    #[test]
    fn outpc_encode_rejects_short_buffer() {
        let mut short = [0u8; Outpc::WIRE_LEN - 1];
        assert_eq!(Outpc::default().encode(&mut short), None);
    }
}
