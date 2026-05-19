#![no_main]

use ecu_spec::{ts_dispatch_step, TsDispatchState};
use libfuzzer_sys::fuzz_target;

const TS_PROTO_FUZZ_MAX_BYTES: usize = 64;

fuzz_target!(|data: &[u8]| {
    if data.len() > TS_PROTO_FUZZ_MAX_BYTES {
        return;
    }

    let dispatch = std::panic::catch_unwind(|| ts_dispatch_step(data));
    assert!(dispatch.is_ok(), "ts_dispatch_step panicked");

    let dispatch = match dispatch {
        Ok(result) => result,
        Err(_) => return,
    };
    assert!((1..=6).contains(&(dispatch.state_len as usize)));

    let last_state = dispatch.states[(dispatch.state_len as usize) - 1];
    assert_eq!(last_state, TsDispatchState::Idle);
});
