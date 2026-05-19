use ecu_core::ts::outpc::Outpc;
use ecu_core::ts::pages::EcuStatePageStore;
use ecu_core::ts::proto::{self, Cmd};
use ecu_core::ts::{OutpcProvider, TunerstudioServer};
use ecu_core::EcuState;

struct Provider {
    state: *const EcuState,
}
impl OutpcProvider for Provider {
    fn fill_outpc(&self, out: &mut Outpc) {
        let s = unsafe { &*self.state };
        out.rpm = s.rpm();
    }
}

#[test]
fn stats_increment_on_success_and_error() {
    let mut state = EcuState::new();
    let provider = Provider {
        state: &state as *const _,
    };
    let pages = EcuStatePageStore {
        fuel: &mut state.config.ipw_table,
        ign: &mut state.config.ignition_table,
    };
    let mut server = TunerstudioServer::new(b"IPW-ECU V0.1", provider, pages);

    // Valid SIG
    let mut req = [0u8; 64];
    let len = proto::encode_reply(Cmd::Sig, &[], &mut req).unwrap();
    let mut out = [0u8; 128];
    let _ = server.handle(&req[..len], &mut out);
    let st = server.stats();
    assert_eq!(st.rx_ok, 1);

    // Invalid CRC
    let mut bad = req;
    bad[len - 1] ^= 0xFF;
    let _ = server.handle(&bad[..len], &mut out);
    let st2 = server.stats();
    assert_eq!(st2.rx_invalid, 1);

    // Write with wrong size → ERR
    let mut payload = [0u8; 4];
    payload[0] = 1; // PAGE_FUEL
    let l2 = proto::encode_reply(Cmd::WritePage, &payload, &mut req).unwrap();
    let _ = server.handle(&req[..l2], &mut out);
    let st3 = server.stats();
    assert!(st3.write_err >= 1);
}
