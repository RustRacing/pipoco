#![no_std]

pub mod ts {
    pub mod service;
    pub mod store;
    pub mod usb_cdc;
}

pub mod kv {
    pub mod ram;
}

pub mod capture;
pub mod outputs;
pub mod sensors {
    pub mod adc_pipeline;
}

pub mod runtime;
