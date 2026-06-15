//! Ensure INI signature matches server signature

use ecu_compat::compat::EcuState;
use ecu_ts::outpc::Outpc;
use ecu_ts::pages::FuelIgnPageStore;
use ecu_ts::proto::{self, Cmd};
use ecu_ts::server::{OutpcProvider, TunerstudioServer};
use std::fs;
use std::path::PathBuf;

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
fn ini_signature_matches_server() {
    // Read INI signature line
    let ini_path = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("tests/assets/IPW-ECU.ini");
    let ini = fs::read_to_string(&ini_path).expect("load ini");
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
    let pages = FuelIgnPageStore {
        fuel: &mut state.config.ipw_table,
        ign: &mut state.config.ignition_table,
    };
    let mut server = TunerstudioServer::new(ecu_ts::TS_SIGNATURE, provider, pages);
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
