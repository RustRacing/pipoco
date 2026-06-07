use super::layout::{ANGLES_PAGE_LEN, EXPERT_TRIGGER_PAGE_LEN, FUEL_PAGE_LEN, IGN_PAGE_LEN};
use ecu_calibration::{KvError, KvStore};

pub use ecu_calibration::kv::{
    PERSIST_KEY_ANGLES, PERSIST_KEY_EXPERT_TRIGGER, PERSIST_KEY_FUEL, PERSIST_KEY_IGN,
    PERSIST_KNOWN_KEYS,
};

fn key_slot(key: &[u8]) -> Option<usize> {
    PERSIST_KNOWN_KEYS
        .iter()
        .position(|candidate| *candidate == key)
}

/// Simple fixed-size RAM KV for fuel, ignition, and angle pages.
/// Useful as a default when bring-up without flash persistence.
pub struct RamKv512 {
    fuel: Option<[u8; FUEL_PAGE_LEN]>,
    ign: Option<[u8; IGN_PAGE_LEN]>,
    angles: Option<[u8; ANGLES_PAGE_LEN]>,
    expert_trigger: Option<[u8; EXPERT_TRIGGER_PAGE_LEN]>,
}

impl RamKv512 {
    pub const fn new() -> Self {
        Self {
            fuel: None,
            ign: None,
            angles: None,
            expert_trigger: None,
        }
    }
}

impl Default for RamKv512 {
    fn default() -> Self {
        Self::new()
    }
}

impl KvStore for RamKv512 {
    fn read(&mut self, key: &[u8], out: &mut [u8]) -> Result<usize, KvError> {
        match key_slot(key) {
            Some(0) => {
                if let Some(data) = &self.fuel {
                    if out.len() < FUEL_PAGE_LEN {
                        return Err(KvError::Io);
                    }
                    out[..FUEL_PAGE_LEN].copy_from_slice(&data[..]);
                    Ok(FUEL_PAGE_LEN)
                } else {
                    Err(KvError::NotFound)
                }
            }
            Some(1) => {
                if let Some(data) = &self.ign {
                    if out.len() < IGN_PAGE_LEN {
                        return Err(KvError::Io);
                    }
                    out[..IGN_PAGE_LEN].copy_from_slice(&data[..]);
                    Ok(IGN_PAGE_LEN)
                } else {
                    Err(KvError::NotFound)
                }
            }
            Some(2) => {
                if let Some(data) = &self.angles {
                    if out.len() < ANGLES_PAGE_LEN {
                        return Err(KvError::Io);
                    }
                    out[..ANGLES_PAGE_LEN].copy_from_slice(&data[..]);
                    Ok(ANGLES_PAGE_LEN)
                } else {
                    Err(KvError::NotFound)
                }
            }
            Some(3) => {
                if let Some(data) = &self.expert_trigger {
                    if out.len() < EXPERT_TRIGGER_PAGE_LEN {
                        return Err(KvError::Io);
                    }
                    out[..EXPERT_TRIGGER_PAGE_LEN].copy_from_slice(&data[..]);
                    Ok(EXPERT_TRIGGER_PAGE_LEN)
                } else {
                    Err(KvError::NotFound)
                }
            }
            _ => Err(KvError::NotFound),
        }
    }
    fn write(&mut self, key: &[u8], data: &[u8]) -> Result<(), KvError> {
        match key_slot(key) {
            Some(0) => {
                if data.len() != FUEL_PAGE_LEN {
                    return Err(KvError::Io);
                }
                let mut arr = [0u8; FUEL_PAGE_LEN];
                arr.copy_from_slice(data);
                self.fuel = Some(arr);
                Ok(())
            }
            Some(1) => {
                if data.len() != IGN_PAGE_LEN {
                    return Err(KvError::Io);
                }
                let mut arr = [0u8; IGN_PAGE_LEN];
                arr.copy_from_slice(data);
                self.ign = Some(arr);
                Ok(())
            }
            Some(2) => {
                if data.len() != ANGLES_PAGE_LEN {
                    return Err(KvError::Io);
                }
                let mut arr = [0u8; ANGLES_PAGE_LEN];
                arr.copy_from_slice(data);
                self.angles = Some(arr);
                Ok(())
            }
            Some(3) => {
                if data.len() != EXPERT_TRIGGER_PAGE_LEN {
                    return Err(KvError::Io);
                }
                let mut arr = [0u8; EXPERT_TRIGGER_PAGE_LEN];
                arr.copy_from_slice(data);
                self.expert_trigger = Some(arr);
                Ok(())
            }
            _ => Err(KvError::NotFound),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn shared_persistence_keys_match_calibration() {
        assert_eq!(PERSIST_KNOWN_KEYS, ecu_calibration::kv::PERSIST_KNOWN_KEYS);
    }

    #[test]
    fn ram_kv_roundtrips_known_keys() {
        let mut kv = RamKv512::new();
        let payload_fuel = [0x11u8; FUEL_PAGE_LEN];
        let payload_ign = [0x22u8; IGN_PAGE_LEN];
        let payload_angles = [0x33u8; ANGLES_PAGE_LEN];
        let payload_expert = [0x44u8; EXPERT_TRIGGER_PAGE_LEN];

        kv.write(PERSIST_KEY_FUEL, &payload_fuel)
            .expect("write fuel");
        let mut fuel_out = [0u8; FUEL_PAGE_LEN];
        assert_eq!(
            kv.read(PERSIST_KEY_FUEL, &mut fuel_out).expect("read fuel"),
            FUEL_PAGE_LEN
        );
        assert_eq!(fuel_out, payload_fuel);

        kv.write(PERSIST_KEY_IGN, &payload_ign).expect("write ign");
        let mut ign_out = [0u8; IGN_PAGE_LEN];
        assert_eq!(
            kv.read(PERSIST_KEY_IGN, &mut ign_out).expect("read ign"),
            IGN_PAGE_LEN
        );
        assert_eq!(ign_out, payload_ign);

        kv.write(PERSIST_KEY_ANGLES, &payload_angles)
            .expect("write angles");
        let mut angles_out = [0u8; ANGLES_PAGE_LEN];
        assert_eq!(
            kv.read(PERSIST_KEY_ANGLES, &mut angles_out)
                .expect("read angles"),
            ANGLES_PAGE_LEN
        );
        assert_eq!(angles_out, payload_angles);

        kv.write(PERSIST_KEY_EXPERT_TRIGGER, &payload_expert)
            .expect("write expert trigger");
        let mut expert_out = [0u8; EXPERT_TRIGGER_PAGE_LEN];
        assert_eq!(
            kv.read(PERSIST_KEY_EXPERT_TRIGGER, &mut expert_out)
                .expect("read expert trigger"),
            EXPERT_TRIGGER_PAGE_LEN
        );
        assert_eq!(expert_out, payload_expert);
    }
}
