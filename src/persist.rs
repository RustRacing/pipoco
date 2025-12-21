//! Persistence abstraction for platform key/value stores
//!
//! The goal is to avoid hard-coding flash details in `ecu-core`. Targets
//! implement `KvStore` using a concrete backend (e.g. sequential-storage,
//! tickv, EKV). Core code composes this via `PersistedPageStore`.

/// Errors from key/value store operations
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum KvError {
    NotFound,
    NoSpace,
    Io,
}

/// Minimal key/value interface for small blobs (e.g., 512-byte pages)
pub trait KvStore {
    /// Read value for `key` into `out`, returning number of bytes copied
    fn read(&mut self, key: &[u8], out: &mut [u8]) -> Result<usize, KvError>;
    /// Write `data` for `key`, replacing previous value
    fn write(&mut self, key: &[u8], data: &[u8]) -> Result<(), KvError>;
}

/// RAM-backed test KV store for small blobs
#[cfg(any(test, feature = "test-utils"))]
pub struct RamKv<const N: usize> {
    fuel: Option<[u8; N]>,
    ign: Option<[u8; N]>,
    ltft: Option<[u8; N]>,
}

#[cfg(any(test, feature = "test-utils"))]
impl<const N: usize> RamKv<N> {
    pub const fn new() -> Self {
        Self {
            fuel: None,
            ign: None,
            ltft: None,
        }
    }
}

#[cfg(any(test, feature = "test-utils"))]
impl<const N: usize> Default for RamKv<N> {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(any(test, feature = "test-utils"))]
impl<const N: usize> KvStore for RamKv<N> {
    fn read(&mut self, key: &[u8], out: &mut [u8]) -> Result<usize, KvError> {
        let src = if key == b"fuel" {
            self.fuel.as_ref().map(|b| &b[..])
        } else if key == b"ign" {
            self.ign.as_ref().map(|b| &b[..])
        } else if key == b"ltft" {
            self.ltft.as_ref().map(|b| &b[..])
        } else {
            None
        };
        if let Some(data) = src {
            if out.len() < data.len() {
                return Err(KvError::Io);
            }
            out[..data.len()].copy_from_slice(data);
            Ok(data.len())
        } else {
            Err(KvError::NotFound)
        }
    }
    fn write(&mut self, key: &[u8], data: &[u8]) -> Result<(), KvError> {
        if data.len() != N {
            return Err(KvError::Io);
        }
        let mut arr = [0u8; N];
        arr.copy_from_slice(data);
        if key == b"fuel" {
            self.fuel = Some(arr);
            Ok(())
        } else if key == b"ign" {
            self.ign = Some(arr);
            Ok(())
        } else if key == b"ltft" {
            self.ltft = Some(arr);
            Ok(())
        } else {
            Err(KvError::Io)
        }
    }
}
