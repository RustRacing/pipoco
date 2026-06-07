#![no_std]

pub mod ts {
    pub mod outpc;
    pub mod page_store;
    pub mod service;
    pub mod state_ptr;
    #[cfg(feature = "ts-usb")]
    pub mod usb_cdc;
}

pub mod kv {
    pub mod ab;
    pub mod layout;
    pub mod ram;
}

pub mod adapter;
pub mod bringup;
pub mod capture;
pub mod control_inputs;
pub mod live_inputs;
#[cfg(any(test, feature = "test-support"))]
pub mod noop;
pub mod outputs;
pub mod sensor_sample;
pub mod split_tick;
pub mod sensors {
    pub mod adc_pipeline;
    pub mod map;
    pub mod vss;
}
pub mod trigger_adapter;
