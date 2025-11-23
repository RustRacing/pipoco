//! TunerStudio server: minimal handler for SIG/OUTPC/PING and page R/W (Phase 2)
use super::{outpc::Outpc, proto};

/// Provider for OUTPC data (decoupled from EcuState)
pub trait OutpcProvider {
    fn fill_outpc(&self, out: &mut Outpc);
}

/// Backing store for pages exposed to TunerStudio (fuel table, ignition table, etc.)
pub trait PageStore {
    fn page_len(&self, page: u8) -> Option<usize>;
    fn read_page(&self, page: u8, out: &mut [u8]) -> Option<usize>;
    fn write_page(&mut self, page: u8, data: &[u8]) -> Result<(), PageError>;
    fn burn(&mut self) -> Result<(), PersistError> {
        Err(PersistError::Unsupported)
    }
}

#[derive(Debug, Copy, Clone)]
pub enum PageError {
    Invalid,
    WrongSize,
}
#[derive(Debug, Copy, Clone)]
pub enum PersistError {
    Unsupported,
    Fail,
}

#[derive(Default)]
pub struct NoPages;
impl PageStore for NoPages {
    fn page_len(&self, _page: u8) -> Option<usize> {
        None
    }
    fn read_page(&self, _page: u8, _out: &mut [u8]) -> Option<usize> {
        None
    }
    fn write_page(&mut self, _page: u8, _data: &[u8]) -> Result<(), PageError> {
        Err(PageError::Invalid)
    }
}

#[derive(Copy, Clone, Default, Debug)]
pub struct ServerStats {
    pub rx_ok: u32,
    pub rx_invalid: u32,
    pub read_ok: u32,
    pub write_ok: u32,
    pub write_err: u32,
    pub burn_ok: u32,
    pub burn_err: u32,
}

pub struct TunerstudioServer<P: OutpcProvider, S: PageStore> {
    signature: &'static [u8],
    provider: P,
    store: S,
    stats: ServerStats,
}

impl<P: OutpcProvider, S: PageStore> TunerstudioServer<P, S> {
    pub const fn new(signature: &'static [u8], provider: P, store: S) -> Self {
        Self {
            signature,
            provider,
            store,
            stats: ServerStats {
                rx_ok: 0,
                rx_invalid: 0,
                read_ok: 0,
                write_ok: 0,
                write_err: 0,
                burn_ok: 0,
                burn_err: 0,
            },
        }
    }

    /// Handle a single request frame in `req`, writing reply into `out`.
    /// Returns reply length on success.
    pub fn handle(&mut self, req: &[u8], out: &mut [u8]) -> Option<usize> {
        let decoded = proto::decode_request(req);
        let (cmd, payload) = match decoded {
            Some(v) => v,
            None => {
                self.stats.rx_invalid = self.stats.rx_invalid.saturating_add(1);
                return None;
            }
        };
        self.stats.rx_ok = self.stats.rx_ok.saturating_add(1);
        match cmd {
            proto::Cmd::Sig => proto::encode_reply(proto::Cmd::Sig, self.signature, out),
            proto::Cmd::Outpc => {
                let mut block = Outpc::default();
                self.provider.fill_outpc(&mut block);
                proto::encode_reply(proto::Cmd::Outpc, block.as_bytes(), out)
            }
            proto::Cmd::Ping => proto::encode_reply(proto::Cmd::Ping, b"PONG", out),
            proto::Cmd::ReadPage => {
                if payload.len() != 1 {
                    return None;
                }
                let page = payload[0];
                let len = self.store.page_len(page)?;
                let mut buf = [0u8; 1024];
                let n = self.store.read_page(page, &mut buf[..len])?;
                self.stats.read_ok = self.stats.read_ok.saturating_add(1);
                proto::encode_reply(proto::Cmd::ReadPage, &buf[..n], out)
            }
            proto::Cmd::WritePage => {
                if payload.len() < 2 {
                    return None;
                }
                let page = payload[0];
                let data = &payload[1..];
                if self.store.write_page(page, data).is_ok() {
                    self.stats.write_ok = self.stats.write_ok.saturating_add(1);
                    proto::encode_reply(proto::Cmd::WritePage, b"OK", out)
                } else {
                    self.stats.write_err = self.stats.write_err.saturating_add(1);
                    proto::encode_reply(proto::Cmd::WritePage, b"ERR", out)
                }
            }
            proto::Cmd::Burn => {
                if self.store.burn().is_ok() {
                    self.stats.burn_ok = self.stats.burn_ok.saturating_add(1);
                    proto::encode_reply(proto::Cmd::Burn, b"OK", out)
                } else {
                    self.stats.burn_err = self.stats.burn_err.saturating_add(1);
                    proto::encode_reply(proto::Cmd::Burn, b"ERR", out)
                }
            }
            // These commands are handled by target wrappers (ts_ecu, etc.)
            proto::Cmd::OutputTest | proto::Cmd::ToothStats => None,
        }
    }

    pub fn stats(&self) -> ServerStats {
        self.stats
    }
    pub fn reset_stats(&mut self) {
        self.stats = ServerStats::default();
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    struct Dummy;
    impl OutpcProvider for Dummy {
        fn fill_outpc(&self, out: &mut Outpc) {
            out.rpm = 1234;
            out.tps_percent = 42;
        }
    }

    #[test]
    fn test_sig_outpc_ping() {
        let mut srv = TunerstudioServer::new(b"IPW-ECU V0.1", Dummy, NoPages);
        // SIG
        let mut req = [0u8; 64];
        let len = proto::encode_reply(proto::Cmd::Sig, &[], &mut req).unwrap();
        let mut out = [0u8; 128];
        let rlen = srv.handle(&req[..len], &mut out).unwrap();
        assert!(rlen > 0);
        // OUTPC request
        let len = proto::encode_reply(proto::Cmd::Outpc, &[], &mut req).unwrap();
        let rlen = srv.handle(&req[..len], &mut out).unwrap();
        assert!(rlen > 0);
        // PING
        let len = proto::encode_reply(proto::Cmd::Ping, &[], &mut req).unwrap();
        let rlen = srv.handle(&req[..len], &mut out).unwrap();
        assert!(rlen > 0);
    }

    #[derive(Default)]
    struct DummyStore {
        page: [u8; 8],
    }
    impl PageStore for DummyStore {
        fn page_len(&self, page: u8) -> Option<usize> {
            if page == 1 {
                Some(8)
            } else {
                None
            }
        }
        fn read_page(&self, page: u8, out: &mut [u8]) -> Option<usize> {
            if page != 1 {
                return None;
            };
            out[..8].copy_from_slice(&self.page);
            Some(8)
        }
        fn write_page(&mut self, page: u8, data: &[u8]) -> Result<(), PageError> {
            if page != 1 || data.len() != 8 {
                return Err(PageError::WrongSize);
            };
            self.page.copy_from_slice(&data[..8]);
            Ok(())
        }
    }

    #[test]
    fn test_read_write_page() {
        let store = DummyStore::default();
        let mut srv = TunerstudioServer::new(b"IPW-ECU V0.1", Dummy, store);
        let mut req = [0u8; 64];
        let mut out = [0u8; 128];
        // Write page 1
        let mut payload = [0u8; 9];
        payload[0] = 1;
        payload[1..].copy_from_slice(&[1, 2, 3, 4, 5, 6, 7, 8]);
        let len = proto::encode_reply(proto::Cmd::WritePage, &payload, &mut req).unwrap();
        let _ = srv.handle(&req[..len], &mut out).unwrap();
        // Read page 1
        let len = proto::encode_reply(proto::Cmd::ReadPage, &[1], &mut req).unwrap();
        let rlen = srv.handle(&req[..len], &mut out).unwrap();
        let decoded = proto::decode_request(&out[..rlen]).unwrap();
        assert_eq!(decoded.0, proto::Cmd::ReadPage);
        assert_eq!(decoded.1, &[1, 2, 3, 4, 5, 6, 7, 8][..]);
    }
}
