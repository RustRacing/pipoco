use crate::kv::layout::{ANGLES_PAGE_LEN, EXPERT_TRIGGER_PAGE_LEN, FUEL_PAGE_LEN, IGN_PAGE_LEN};
use crate::kv::ram::{
    PERSIST_KEY_ANGLES, PERSIST_KEY_EXPERT_TRIGGER, PERSIST_KEY_FUEL, PERSIST_KEY_IGN,
};
use ecu_calibration::{ExpertTriggerCalibration, ExpertTriggerRecordError};
use ecu_core::persist::{KvError, KvStore};
use ecu_core::ts::pages::{EcuPageStore, PAGE_EXPERT_TRIGGER, PAGE_FUEL, PAGE_IGN};
use ecu_core::ts::server::{PageError, PageStore, PersistError};
use ecu_core::EcuState;

/// Combined page store that exposes all ECU pages (fuel/ign/sensors/AE/DFCO)
/// and persists writable setup pages via a KvStore.
///
/// The TS service needs to keep borrowing the live `EcuState` elsewhere, so this
/// store keeps a raw pointer and rebuilds the page view per call instead of
/// holding a long-lived `&mut EcuState` borrow.
pub struct PersistedEcuPageStore<KV: KvStore> {
    state: *mut EcuState,
    kv: KV,
    expert_trigger: ExpertTriggerCalibration,
}

impl<KV: KvStore> PersistedEcuPageStore<KV> {
    pub fn new(state: &mut EcuState, kv: KV) -> Self {
        Self {
            state: state as *mut EcuState,
            kv,
            expert_trigger: ExpertTriggerCalibration::default(),
        }
    }

    fn build_inner(&mut self) -> EcuPageStore<'_> {
        // Safety: the caller owns the `EcuState` and this wrapper is the only
        // place that rebuilds the page view. The raw pointer exists so the TS
        // service can keep borrowing the live state independently.
        let state = unsafe { &mut *self.state };
        state.page_store()
    }

    /// Attempt to load persisted pages (fuel/ign) into the in-memory tables.
    pub fn try_load(&mut self) {
        let mut buf = [0u8; FUEL_PAGE_LEN];
        let fuel = self.kv.read(PERSIST_KEY_FUEL, &mut buf).ok().and_then(|n| {
            if n == FUEL_PAGE_LEN {
                Some(buf)
            } else {
                None
            }
        });
        let mut buf = [0u8; IGN_PAGE_LEN];
        let ign = self.kv.read(PERSIST_KEY_IGN, &mut buf).ok().and_then(|n| {
            if n == IGN_PAGE_LEN {
                Some(buf)
            } else {
                None
            }
        });
        let mut abuf = [0u8; ANGLES_PAGE_LEN];
        let angles = self
            .kv
            .read(PERSIST_KEY_ANGLES, &mut abuf)
            .ok()
            .and_then(|n| {
                if n == ANGLES_PAGE_LEN {
                    Some(abuf)
                } else {
                    None
                }
            });
        let mut expert_buf = [0u8; EXPERT_TRIGGER_PAGE_LEN];
        let expert_trigger = self
            .kv
            .read(PERSIST_KEY_EXPERT_TRIGGER, &mut expert_buf)
            .ok()
            .and_then(|n| {
                if n == EXPERT_TRIGGER_PAGE_LEN {
                    ExpertTriggerCalibration::decode_record(&expert_buf).ok()
                } else {
                    None
                }
            });

        let mut inner = self.build_inner();
        if let Some(buf) = fuel {
            let _ = inner.write_page(PAGE_FUEL, &buf);
        }
        if let Some(buf) = ign {
            let _ = inner.write_page(PAGE_IGN, &buf);
        }
        if let Some(abuf) = angles {
            let _ = inner.write_page(ecu_core::ts::pages::PAGE_ANGLES, &abuf);
        }
        if let Some(expert_trigger) = expert_trigger {
            self.expert_trigger = expert_trigger;
        }
    }

    /// Reset persisted pages in memory to safe defaults (does not write KV until burn)
    pub fn factory_reset(&mut self) {
        let mut inner = self.build_inner();
        // Fuel defaults
        let _ = inner.write_page(PAGE_FUEL, &[0u8; FUEL_PAGE_LEN]);
        // Ign defaults
        let _ = inner.write_page(PAGE_IGN, &[0u8; IGN_PAGE_LEN]);
        // Angles defaults (zeros, and default cam timeout)
        let mut angles = [0u8; ANGLES_PAGE_LEN];
        // tooth0_angle_x10=0 at [64..66], cam_timeout_ms default=500 at [66..68]
        angles[66..68].copy_from_slice(&500u16.to_le_bytes());
        let _ = inner.write_page(ecu_core::ts::pages::PAGE_ANGLES, &angles);
        self.expert_trigger = ExpertTriggerCalibration::default();
    }
}

fn record_error_to_page_error(err: ExpertTriggerRecordError) -> PageError {
    match err {
        ExpertTriggerRecordError::WrongSize => PageError::WrongSize,
        ExpertTriggerRecordError::InvalidEnum | ExpertTriggerRecordError::InvalidCalibration(_) => {
            PageError::Invalid
        }
    }
}

fn kv_error_to_persist_error(err: KvError) -> PersistError {
    match err {
        KvError::EngineRunning => PersistError::EngineRunning,
        _ => PersistError::Fail,
    }
}

impl<KV: KvStore> PageStore for PersistedEcuPageStore<KV> {
    fn page_len(&self, page: u8) -> Option<usize> {
        if page == PAGE_EXPERT_TRIGGER {
            return Some(EXPERT_TRIGGER_PAGE_LEN);
        }
        // Safety: see `build_inner`.
        let state = unsafe { &mut *self.state };
        let inner = state.page_store();
        inner.page_len(page)
    }
    fn read_page(&self, page: u8, out: &mut [u8]) -> Option<usize> {
        if page == PAGE_EXPERT_TRIGGER {
            return self.expert_trigger.encode_record(out).ok();
        }
        // Safety: see `build_inner`.
        let state = unsafe { &mut *self.state };
        let inner = state.page_store();
        inner.read_page(page, out)
    }
    fn write_page(&mut self, page: u8, data: &[u8]) -> Result<(), PageError> {
        if page == PAGE_EXPERT_TRIGGER {
            if data.len() != EXPERT_TRIGGER_PAGE_LEN {
                return Err(PageError::WrongSize);
            }
            let proposed = ExpertTriggerCalibration::decode_record(data)
                .map_err(record_error_to_page_error)?;
            proposed
                .validate_transition_from(&self.expert_trigger)
                .map_err(|_| PageError::Invalid)?;
            self.expert_trigger = proposed;
            return Ok(());
        }
        let mut inner = self.build_inner();
        inner.write_page(page, data)
    }
    fn burn(&mut self) -> Result<(), PersistError> {
        // Serialize both pages and store via KV
        let fuel = {
            let mut buf = [0u8; FUEL_PAGE_LEN];
            let inner = self.build_inner();
            if inner.read_page(PAGE_FUEL, &mut buf).unwrap_or(0) == FUEL_PAGE_LEN {
                Some(buf)
            } else {
                None
            }
        };
        let ign = {
            let mut buf = [0u8; IGN_PAGE_LEN];
            let inner = self.build_inner();
            if inner.read_page(PAGE_IGN, &mut buf).unwrap_or(0) == IGN_PAGE_LEN {
                Some(buf)
            } else {
                None
            }
        };
        let angles = {
            let mut buf = [0u8; ANGLES_PAGE_LEN];
            let inner = self.build_inner();
            if inner
                .read_page(ecu_core::ts::pages::PAGE_ANGLES, &mut buf)
                .unwrap_or(0)
                == ANGLES_PAGE_LEN
            {
                Some(buf)
            } else {
                None
            }
        };
        let expert_trigger = {
            let mut buf = [0u8; EXPERT_TRIGGER_PAGE_LEN];
            self.expert_trigger
                .encode_record(&mut buf)
                .map_err(|_| PersistError::Fail)?;
            Some(buf)
        };
        if let Some(buf) = fuel {
            self.kv
                .write(PERSIST_KEY_FUEL, &buf)
                .map_err(kv_error_to_persist_error)?;
        }
        if let Some(buf) = ign {
            self.kv
                .write(PERSIST_KEY_IGN, &buf)
                .map_err(kv_error_to_persist_error)?;
        }
        if let Some(buf) = angles {
            self.kv
                .write(PERSIST_KEY_ANGLES, &buf)
                .map_err(kv_error_to_persist_error)?;
        }
        if let Some(buf) = expert_trigger {
            match self.kv.write(PERSIST_KEY_EXPERT_TRIGGER, &buf) {
                Ok(()) => {}
                Err(KvError::EngineRunning) => return Err(PersistError::EngineRunning),
                Err(_) if self.expert_trigger == ExpertTriggerCalibration::default() => {}
                Err(_) => return Err(PersistError::Fail),
            }
        }
        Ok(())
    }
}
