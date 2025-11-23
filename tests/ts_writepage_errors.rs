//! Verify TS server returns ERR on wrong-sized WritePage payloads

use ecu_core::ts::outpc::Outpc;
use ecu_core::ts::pages::{
    EcuStatePageStore, PAGE_AE, PAGE_DFCO, PAGE_FUEL, PAGE_IGN, PAGE_SENSORS,
};
use ecu_core::ts::proto::{self, Cmd};
use ecu_core::ts::{OutpcProvider, TunerstudioServer};
use ecu_core::EcuState;

struct Provider {
    state: *const EcuState,
}
impl OutpcProvider for Provider {
    fn fill_outpc(&self, out: &mut Outpc) {
        let s = unsafe { &*self.state };
        out.rpm = s.rpm;
        out.synced = if s.synced { 1 } else { 0 };
    }
}

#[test]
fn write_wrong_sizes_return_err() {
    let mut state = EcuState::new();
    let provider = Provider {
        state: &state as *const _,
    };
    let store = EcuStatePageStore {
        fuel: &mut state.ipw_table,
        ign: &mut state.ignition_table,
    };
    let mut server = TunerstudioServer::new(b"IPW-ECU V0.1", provider, store);

    let mut req = [0u8; 1024];
    let mut out = [0u8; 1024];

    // Fuel page wrong length (should be 512)
    let mut bad = [0u8; 8];
    bad[0] = PAGE_FUEL; // page
                        // Only 3 bytes of data instead of 512
    let len = proto::encode_reply(Cmd::WritePage, &bad[..4], &mut req).unwrap();
    let rlen = server.handle(&req[..len], &mut out).unwrap();
    let (cmd, payload) = proto::decode_request(&out[..rlen]).unwrap();
    assert_eq!(cmd, Cmd::WritePage);
    assert_eq!(payload, b"ERR");

    // Ignition page wrong length
    bad[0] = PAGE_IGN;
    let len = proto::encode_reply(Cmd::WritePage, &bad[..5], &mut req).unwrap();
    let rlen = server.handle(&req[..len], &mut out).unwrap();
    let (cmd, payload) = proto::decode_request(&out[..rlen]).unwrap();
    assert_eq!(cmd, Cmd::WritePage);
    assert_eq!(payload, b"ERR");

    // Sensors page wrong length (should be 128)
    let mut s = [0u8; 2];
    s[0] = PAGE_SENSORS;
    let len = proto::encode_reply(Cmd::WritePage, &s, &mut req).unwrap();
    let rlen = server.handle(&req[..len], &mut out).unwrap();
    let (cmd, payload) = proto::decode_request(&out[..rlen]).unwrap();
    assert_eq!(cmd, Cmd::WritePage);
    assert_eq!(payload, b"ERR");

    // AE page wrong length (should be 16)
    let mut a = [0u8; 3];
    a[0] = PAGE_AE;
    let len = proto::encode_reply(Cmd::WritePage, &a, &mut req).unwrap();
    let rlen = server.handle(&req[..len], &mut out).unwrap();
    let (cmd, payload) = proto::decode_request(&out[..rlen]).unwrap();
    assert_eq!(cmd, Cmd::WritePage);
    assert_eq!(payload, b"ERR");

    // DFCO page wrong length (should be 16)
    let mut d = [0u8; 5];
    d[0] = PAGE_DFCO;
    let len = proto::encode_reply(Cmd::WritePage, &d, &mut req).unwrap();
    let rlen = server.handle(&req[..len], &mut out).unwrap();
    let (cmd, payload) = proto::decode_request(&out[..rlen]).unwrap();
    assert_eq!(cmd, Cmd::WritePage);
    assert_eq!(payload, b"ERR");
}
