//! Negative protocol tests inspired by FreeMS2’s packet approach

use ecu_core::ts::outpc::Outpc;
use ecu_core::ts::pages::EcuStatePageStore;
use ecu_core::ts::proto::{self, Cmd};
use ecu_core::ts::serial::FrameAssembler;
use ecu_core::ts::{OutpcProvider, TunerstudioServer};
use ecu_core::EcuState;

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

#[test]
fn bad_crc_is_dropped() {
    let mut state = EcuState::new();
    let provider = Provider {
        state: &state as *const _,
    };
    let pages = EcuStatePageStore {
        fuel: &mut state.config.ipw_table,
        ign: &mut state.config.ignition_table,
    };
    let mut server = TunerstudioServer::new(b"IPW-ECU V0.1", provider, pages);

    // Build a valid SIG request then corrupt the CRC
    let mut req = [0u8; 64];
    let len = proto::encode_reply(Cmd::Sig, &[], &mut req).unwrap();
    // Corrupt last byte (CRC)
    req[len - 1] ^= 0xFF;
    let mut out = [0u8; 64];
    let resp = server.handle(&req[..len], &mut out);
    assert!(resp.is_none(), "Server should drop bad CRC frames");
}

#[test]
fn truncated_frame_is_ignored() {
    let mut state = EcuState::new();
    let provider = Provider {
        state: &state as *const _,
    };
    let pages = EcuStatePageStore {
        fuel: &mut state.config.ipw_table,
        ign: &mut state.config.ignition_table,
    };
    let mut server = TunerstudioServer::new(b"IPW-ECU V0.1", provider, pages);

    // Build a valid OUTPC request then truncate it
    let mut req = [0u8; 64];
    let len = proto::encode_reply(Cmd::Outpc, &[], &mut req).unwrap();
    let truncated = len.saturating_sub(3);
    let mut out = [0u8; 64];
    let resp = server.handle(&req[..truncated], &mut out);
    assert!(resp.is_none(), "Server should ignore truncated frames");
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
