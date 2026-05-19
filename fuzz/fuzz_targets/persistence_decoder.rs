#![no_main]

use ecu_spec::{persist_decode, PERSIST_RECORD_MAX_BYTES};
use libfuzzer_sys::fuzz_target;

fuzz_target!(|data: &[u8]| {
    if data.len() > PERSIST_RECORD_MAX_BYTES {
        return;
    }

    let decoded = std::panic::catch_unwind(|| persist_decode(data));
    assert!(decoded.is_ok(), "persist_decode panicked");

    let _ = match decoded {
        Ok(result) => result,
        Err(_) => return,
    };
});
