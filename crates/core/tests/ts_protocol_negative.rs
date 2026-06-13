//! Negative protocol tests inspired by FreeMS2’s packet approach

use ecu_core::compat::EcuState;
use ecu_ts::outpc::Outpc;
use ecu_ts::pages::FuelIgnPageStore;
use ecu_ts::proto::{self, Cmd};
use ecu_ts::serial::FrameAssembler;
use ecu_ts::server::{OutpcProvider, TunerstudioServer};

struct Provider {
    state: *const EcuState,
}
impl OutpcProvider for Provider {
    fn fill_outpc(&self, out: &mut Outpc) {
        let s = unsafe { &*self.state };
        out.rpm = s.rpm();
        out.tps_percent = s.tps_percent();
        out.vbatt_mv = s.battery_voltage_mv();
        out.synced = if s.synced() { 1 } else { 0 };
    }
}

fn with_server<R>(
    f: impl FnOnce(&mut TunerstudioServer<Provider, FuelIgnPageStore<'_>>) -> R,
) -> R {
    let mut state = EcuState::new();
    let provider = Provider {
        state: &state as *const _,
    };
    let pages = FuelIgnPageStore {
        fuel: &mut state.config.ipw_table,
        ign: &mut state.config.ignition_table,
    };
    let mut server = TunerstudioServer::new(ecu_ts::TS_SIGNATURE, provider, pages);
    f(&mut server)
}

fn unknown_cmd_frame(out: &mut [u8]) -> usize {
    let len = 1 + 2;
    out[0] = (proto::MAGIC & 0xff) as u8;
    out[1] = (proto::MAGIC >> 8) as u8;
    out[2] = (len & 0xff) as u8;
    out[3] = (len >> 8) as u8;
    out[4] = 0x7f;
    let crc = proto::crc16_ccitt(&out[4..5]);
    out[5] = (crc & 0xff) as u8;
    out[6] = (crc >> 8) as u8;
    7
}

#[test]
fn bad_crc_is_dropped() {
    with_server(|server| {
        // Build a valid SIG request then corrupt the CRC
        let mut req = [0u8; 64];
        let len = proto::encode_reply(Cmd::Sig, &[], &mut req).unwrap();
        req[len - 1] ^= 0xFF;
        let mut out = [0u8; 64];
        let resp = server.handle(&req[..len], &mut out);
        assert!(resp.is_none(), "Server should drop bad CRC frames");
    });
}

#[test]
fn unknown_command_is_dropped_even_with_valid_crc() {
    let mut req = [0u8; 64];
    let len = unknown_cmd_frame(&mut req);

    with_server(|server| {
        let mut out = [0u8; 64];
        assert!(server.handle(&req[..len], &mut out).is_none());
    });
}

#[test]
fn truncated_frame_is_ignored() {
    with_server(|server| {
        // Build a valid OUTPC request then truncate it
        let mut req = [0u8; 64];
        let len = proto::encode_reply(Cmd::Outpc, &[], &mut req).unwrap();
        let truncated = len.saturating_sub(3);
        let mut out = [0u8; 64];
        let resp = server.handle(&req[..truncated], &mut out);
        assert!(resp.is_none(), "Server should ignore truncated frames");
    });
}

#[test]
fn malformed_read_page_payloads_are_ignored() {
    for payload in [&[][..], &[1, 2][..]] {
        with_server(|server| {
            let mut req = [0u8; 64];
            let len = proto::encode_reply(Cmd::ReadPage, payload, &mut req).unwrap();
            let mut out = [0u8; 64];
            assert!(server.handle(&req[..len], &mut out).is_none());
        });
    }
}

#[test]
fn assembler_skips_garbage() {
    let mut asm = FrameAssembler::new();
    // Feed random garbage and ensure no frame is produced
    let garbage = [0xDE, 0xAD, 0xBE, 0xEF, 0x00, 0xFF, 0x01];
    asm.feed(&garbage);
    let mut out = [0u8; 64];
    assert!(asm.try_pop(&mut out).is_none());
}
