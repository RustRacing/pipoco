//! Persistence abstraction for small calibration key/value stores.
//!
//! The raw `PERSIST_KEY_*` constants and [`RamKv`] store below are legacy
//! setup-page compatibility for TunerStudio byte pages. They are not the
//! canonical calibration model API.

/// Errors from key/value store operations.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum KvError {
    NotFound,
    NoSpace,
    Io,
    EngineRunning,
}

/// Errors reported by calibration persistence helpers.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum PersistError {
    Fail,
    EngineRunning,
}

/// Legacy raw setup-page persistence keys used by current page store scaffolding.
pub const PERSIST_KEY_FUEL: &[u8] = b"fuel";
pub const PERSIST_KEY_IGN: &[u8] = b"ign";
pub const PERSIST_KEY_ANGLES: &[u8] = b"angles";
pub const PERSIST_KEY_EXPERT_TRIGGER: &[u8] = b"expert_trigger";

/// Known legacy raw setup-page persistence keys in page-store order.
pub const PERSIST_KNOWN_KEYS: [&[u8]; 4] = [
    PERSIST_KEY_FUEL,
    PERSIST_KEY_IGN,
    PERSIST_KEY_ANGLES,
    PERSIST_KEY_EXPERT_TRIGGER,
];

pub const fn persist_known_keys() -> [&'static [u8]; 4] {
    PERSIST_KNOWN_KEYS
}

/// Minimal key/value interface for small blobs, such as TunerStudio pages.
pub trait KvStore {
    /// Read value for `key` into `out`, returning number of bytes copied.
    fn read(&mut self, key: &[u8], out: &mut [u8]) -> Result<usize, KvError>;

    /// Write `data` for `key`, replacing the previous value.
    fn write(&mut self, key: &[u8], data: &[u8]) -> Result<(), KvError>;
}

fn key_slot(key: &[u8]) -> Option<usize> {
    PERSIST_KNOWN_KEYS
        .iter()
        .position(|candidate| *candidate == key)
}

/// RAM-backed KV store for tests, simulators, and bring-up paths.
pub struct RamKv<const N: usize> {
    fuel: Option<[u8; N]>,
    ign: Option<[u8; N]>,
    angles: Option<[u8; N]>,
    expert_trigger: Option<[u8; N]>,
}

impl<const N: usize> RamKv<N> {
    pub const fn new() -> Self {
        Self {
            fuel: None,
            ign: None,
            angles: None,
            expert_trigger: None,
        }
    }
}

impl<const N: usize> Default for RamKv<N> {
    fn default() -> Self {
        Self::new()
    }
}

impl<const N: usize> KvStore for RamKv<N> {
    fn read(&mut self, key: &[u8], out: &mut [u8]) -> Result<usize, KvError> {
        let src = match key_slot(key) {
            Some(0) => self.fuel.as_ref().map(|b| &b[..]),
            Some(1) => self.ign.as_ref().map(|b| &b[..]),
            Some(2) => self.angles.as_ref().map(|b| &b[..]),
            Some(3) => self.expert_trigger.as_ref().map(|b| &b[..]),
            _ => None,
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
        match key_slot(key) {
            Some(0) => {
                self.fuel = Some(arr);
                Ok(())
            }
            Some(1) => {
                self.ign = Some(arr);
                Ok(())
            }
            Some(2) => {
                self.angles = Some(arr);
                Ok(())
            }
            Some(3) => {
                self.expert_trigger = Some(arr);
                Ok(())
            }
            _ => Err(KvError::Io),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn known_persistence_keys_are_canonical() {
        assert_eq!(
            persist_known_keys(),
            [
                b"fuel" as &[u8],
                b"ign" as &[u8],
                b"angles" as &[u8],
                b"expert_trigger" as &[u8],
            ]
        );
    }

    #[test]
    fn ram_kv_roundtrips_known_keys() {
        let mut kv = RamKv::<4>::new();
        let payload = [1u8, 2, 3, 4];

        for &key in PERSIST_KNOWN_KEYS.iter() {
            kv.write(key, &payload).expect("write known key");
            let mut out = [0u8; 4];
            assert_eq!(kv.read(key, &mut out).expect("read known key"), 4);
            assert_eq!(out, payload);
        }
    }

    #[test]
    fn ram_kv_rejects_unknown_key() {
        let mut kv = RamKv::<4>::new();
        let payload = [1u8, 2, 3, 4];
        assert_eq!(kv.write(b"other", &payload), Err(KvError::Io));
        let mut out = [0u8; 4];
        assert_eq!(kv.read(b"other", &mut out), Err(KvError::NotFound));
    }
}
