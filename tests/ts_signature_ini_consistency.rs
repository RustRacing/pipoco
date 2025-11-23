//! Ensure INI signature matches server signature

use ecu_core::ts::outpc::Outpc;
use ecu_core::ts::pages::EcuStatePageStore;
use ecu_core::ts::proto::{self, Cmd};
use ecu_core::ts::{OutpcProvider, TunerstudioServer};
use ecu_core::EcuState;
use std::fs;

struct Provider {
    state: *const EcuState,
}
impl OutpcProvider for Provider {
    fn fill_outpc(&self, out: &mut Outpc) {
        let s = unsafe { &*self.state };
        out.rpm = s.rpm;
    }
}

#[test]
fn ini_signature_matches_server() {
    // Read INI signature line
    let ini = fs::read_to_string("../ts/IPW-ECU.ini").expect("load ini");
    let sig_line = ini
        .lines()
        .find(|l| l.trim_start().starts_with("signature"))
        .expect("signature line");
    let ini_sig = sig_line.split('=').nth(1).unwrap().trim().trim_matches('"');

    // Get server signature via SIG reply
    let mut state = EcuState::new();
    let provider = Provider {
        state: &state as *const _,
    };
    let pages = EcuStatePageStore {
        fuel: &mut state.ipw_table,
        ign: &mut state.ignition_table,
    };
    let mut server = TunerstudioServer::new(b"IPW-ECU V0.1", provider, pages);
    let mut req = [0u8; 64];
    let len = proto::encode_reply(Cmd::Sig, &[], &mut req).unwrap();
    let mut out = [0u8; 64];
    let rlen = server.handle(&req[..len], &mut out).expect("sig reply");
    let (_cmd, payload) = proto::decode_request(&out[..rlen]).expect("decode");
    let server_sig = std::str::from_utf8(payload).unwrap();

    assert_eq!(
        server_sig, ini_sig,
        "INI signature must match server signature"
    );
}
