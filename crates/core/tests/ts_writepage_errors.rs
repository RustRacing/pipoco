//! Verify TS server returns ERR on wrong-sized WritePage payloads

use ecu_core::compat::EcuState;
use ecu_core::ts::pages::{
    EcuPageStore, PAGE_AE, PAGE_DFCO, PAGE_DIAG, PAGE_DIAG_LOG, PAGE_FUEL, PAGE_IGN, PAGE_LIMITS,
    PAGE_SENSORS, PAGE_SNAPSHOT,
};
use ecu_ts::outpc::Outpc;
use ecu_ts::proto::{self, Cmd};
use ecu_ts::server::{OutpcProvider, TunerstudioServer};

struct Provider {
    state: *const EcuState,
}
impl OutpcProvider for Provider {
    fn fill_outpc(&self, out: &mut Outpc) {
        let s = unsafe { &*self.state };
        out.rpm = s.rpm();
        out.synced = if s.synced() { 1 } else { 0 };
    }
}

fn with_server<R>(f: impl FnOnce(&mut TunerstudioServer<Provider, EcuPageStore<'_>>) -> R) -> R {
    let mut state = EcuState::new();
    let provider = Provider {
        state: &state as *const _,
    };
    let store = state.page_store();
    let mut server = TunerstudioServer::new(ecu_ts::TS_SIGNATURE, provider, store);
    f(&mut server)
}

fn assert_write_err(
    server: &mut TunerstudioServer<Provider, EcuPageStore<'_>>,
    page: u8,
    data: &[u8],
    req: &mut [u8],
    out: &mut [u8],
) {
    let mut payload = [0u8; 1024];
    payload[0] = page;
    payload[1..1 + data.len()].copy_from_slice(data);
    let len = proto::encode_reply(Cmd::WritePage, &payload[..1 + data.len()], req).unwrap();
    let rlen = server.handle(&req[..len], out).unwrap();
    let (cmd, payload) = proto::decode_request(&out[..rlen]).unwrap();
    assert_eq!(cmd, Cmd::WritePage);
    assert_eq!(payload, b"ERR");
}

#[test]
fn write_wrong_sizes_return_err() {
    with_server(|server| {
        let mut req = [0u8; 1024];
        let mut out = [0u8; 1024];

        assert_write_err(server, PAGE_FUEL, &[0, 0, 0], &mut req, &mut out);
        assert_write_err(server, PAGE_IGN, &[0, 0, 0, 0], &mut req, &mut out);
        assert_write_err(server, PAGE_SENSORS, &[0], &mut req, &mut out);
        assert_write_err(server, PAGE_AE, &[0, 0], &mut req, &mut out);
        assert_write_err(server, PAGE_DFCO, &[0, 0, 0, 0], &mut req, &mut out);
        assert_write_err(server, PAGE_LIMITS, &[0, 0, 0, 0], &mut req, &mut out);
    });
}

#[test]
fn write_limits_with_invalid_trigger_bits_returns_err() {
    with_server(|server| {
        let mut req = [0u8; 1024];
        let mut out = [0u8; 1024];
        let mut data = [0u8; 10];

        data[0..2].copy_from_slice(&100u16.to_le_bytes());
        data[2..4].copy_from_slice(&3000u16.to_le_bytes());
        data[4] = 0;
        data[5] = 100;
        data[6..8].copy_from_slice(&3u16.to_le_bytes());
        data[8] = 0b100;

        assert_write_err(server, PAGE_LIMITS, &data, &mut req, &mut out);
    });
}

#[test]
fn write_read_only_pages_returns_err() {
    with_server(|server| {
        let mut req = [0u8; 1024];
        let mut out = [0u8; 1024];

        for page in [PAGE_DIAG, PAGE_DIAG_LOG, PAGE_SNAPSHOT] {
            assert_write_err(server, page, &[0; 16], &mut req, &mut out);
        }
    });
}
