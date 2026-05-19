#![cfg_attr(not(test), no_std)]

#[cfg(all(feature = "capture-gpio", feature = "capture-pio"))]
compile_error!("features `capture-gpio` and `capture-pio` are mutually exclusive");

pub use ecu_target_common::adapter::{BoardAdapter, BoardAdapterError, BoardEvent};

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn board_adapter_reexport_compiles() {
        let _ = core::mem::size_of::<BoardAdapter<(), (), (), (), (), ()>>();
        let _ = core::mem::size_of::<BoardAdapterError<(), (), (), (), (), ()>>();
        let _ = core::mem::size_of::<BoardEvent>();
    }
}
