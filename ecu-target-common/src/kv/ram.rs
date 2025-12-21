use ecu_core::persist::{KvError, KvStore};

/// Simple fixed-size RAM KV for two 512-byte pages (fuel, ign).
/// Useful as a default when bring-up without flash persistence.
pub struct RamKv512 {
    fuel: Option<[u8; 512]>,
    ign: Option<[u8; 512]>,
    angles: Option<[u8; 68]>,
}

impl RamKv512 {
    pub const fn new() -> Self {
        Self {
            fuel: None,
            ign: None,
            angles: None,
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
        match key {
            b"fuel" => {
                if let Some(data) = &self.fuel { if out.len() < 512 { return Err(KvError::Io); } out[..512].copy_from_slice(&data[..]); Ok(512) } else { Err(KvError::NotFound) }
            }
            b"ign" => {
                if let Some(data) = &self.ign { if out.len() < 512 { return Err(KvError::Io); } out[..512].copy_from_slice(&data[..]); Ok(512) } else { Err(KvError::NotFound) }
            }
            b"angles" => {
                if let Some(data) = &self.angles { if out.len() < 68 { return Err(KvError::Io); } out[..68].copy_from_slice(&data[..]); Ok(68) } else { Err(KvError::NotFound) }
            }
            _ => Err(KvError::NotFound),
        }
    }
    fn write(&mut self, key: &[u8], data: &[u8]) -> Result<(), KvError> {
        match key {
            b"fuel" => {
                if data.len() != 512 { return Err(KvError::Io); }
                let mut arr = [0u8; 512]; arr.copy_from_slice(data); self.fuel = Some(arr); Ok(())
            }
            b"ign" => {
                if data.len() != 512 { return Err(KvError::Io); }
                let mut arr = [0u8; 512]; arr.copy_from_slice(data); self.ign = Some(arr); Ok(())
            }
            b"angles" => {
                if data.len() != 68 { return Err(KvError::Io); }
                let mut arr = [0u8; 68]; arr.copy_from_slice(data); self.angles = Some(arr); Ok(())
            }
            _ => Err(KvError::NotFound),
        }
    }
}
