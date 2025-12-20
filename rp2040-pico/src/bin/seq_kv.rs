#![allow(dead_code)]
// Minimal RAM-backed KV stub to satisfy formatting/build when `flash-kv` is enabled
// on non-ARM hosts. On real hardware use `flash_kv::FlashKv` (ts_ecu.rs picks it
// when target_arch = "arm").

use ecu_core::persist::{KvError, KvStore};

pub struct SeqKv {
    fuel: Option<[u8; 512]>,
    ign: Option<[u8; 512]>,
    angles: Option<[u8; 68]>,
}

impl SeqKv {
    pub const fn new() -> Self {
        Self {
            fuel: None,
            ign: None,
            angles: None,
        }
    }
}

impl Default for SeqKv {
    fn default() -> Self {
        Self::new()
    }
}

impl KvStore for SeqKv {
    fn read(&mut self, key: &[u8], out: &mut [u8]) -> Result<usize, KvError> {
        match key {
            b"fuel" => {
                if let Some(buf) = &self.fuel {
                    if out.len() < 512 {
                        return Err(KvError::Io);
                    }
                    out[..512].copy_from_slice(buf);
                    Ok(512)
                } else {
                    Err(KvError::NotFound)
                }
            }
            b"ign" => {
                if let Some(buf) = &self.ign {
                    if out.len() < 512 {
                        return Err(KvError::Io);
                    }
                    out[..512].copy_from_slice(buf);
                    Ok(512)
                } else {
                    Err(KvError::NotFound)
                }
            }
            b"angles" => {
                if let Some(buf) = &self.angles {
                    if out.len() < 68 {
                        return Err(KvError::Io);
                    }
                    out[..68].copy_from_slice(buf);
                    Ok(68)
                } else {
                    Err(KvError::NotFound)
                }
            }
            _ => Err(KvError::Io),
        }
    }

    fn write(&mut self, key: &[u8], data: &[u8]) -> Result<(), KvError> {
        match key {
            b"fuel" => {
                if data.len() != 512 {
                    return Err(KvError::Io);
                }
                let mut buf = [0u8; 512];
                buf.copy_from_slice(&data[..512]);
                self.fuel = Some(buf);
                Ok(())
            }
            b"ign" => {
                if data.len() != 512 {
                    return Err(KvError::Io);
                }
                let mut buf = [0u8; 512];
                buf.copy_from_slice(&data[..512]);
                self.ign = Some(buf);
                Ok(())
            }
            b"angles" => {
                if data.len() != 68 {
                    return Err(KvError::Io);
                }
                let mut buf = [0u8; 68];
                buf.copy_from_slice(&data[..68]);
                self.angles = Some(buf);
                Ok(())
            }
            _ => Err(KvError::Io),
        }
    }
}
