#![allow(dead_code)]
// Minimal RAM-backed KV stub to satisfy formatting/build when `flash-kv` is enabled.
// Replace with real flash-backed sequential storage as needed.

use ecu_core::persist::{KvError, KvStore};

pub struct SeqKv {
    fuel: Option<[u8; 512]>,
    ign: Option<[u8; 512]>,
}

impl SeqKv {
    pub const fn new() -> Self {
        Self {
            fuel: None,
            ign: None,
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
            _ => Err(KvError::Io),
        }
    }

    fn write(&mut self, key: &[u8], data: &[u8]) -> Result<(), KvError> {
        if data.len() != 512 {
            return Err(KvError::Io);
        }
        let mut buf = [0u8; 512];
        buf.copy_from_slice(&data[..512]);
        match key {
            b"fuel" => {
                self.fuel = Some(buf);
                Ok(())
            }
            b"ign" => {
                self.ign = Some(buf);
                Ok(())
            }
            _ => Err(KvError::Io),
        }
    }
}
